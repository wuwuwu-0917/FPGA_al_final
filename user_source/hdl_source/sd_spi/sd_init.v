//=============================================================================
//  sd_init.v  ---  SD 卡 SPI 模式初始化（移植自参考工程 sd_ctrl_top/sd_init.v）
//
//  相对参考实现的改动（本任务补丁）：
//    0. I_start 电平：高=运行；拉低=异步复位到 st_idle（状态/计数器立即清，
//       sd_ctrl 头注释写明了上层契约）
//    1. 增加 O_sdhc（OCR 的 CCS：1=SDHC/SDXC 块地址，0=SDSC 字节地址）
//       和 O_error（初始化失败，电平）
//    2. DIV_FREQ=100（25MHz/100=250kHz），RETRY_MAX=3，ACMD41_MAX=100
//    3. 增加 CMD58 / CMD16 命令常量
//    4. ACMD41 成功后发 CMD58 读 OCR：R1==0 且 CCS=1 -> done；
//       CCS=0 -> CMD16(512B) -> done
//    5. cmd0/cmd8/cmd55/acmd41/cmd58/cmd16 的等待态都加 100ms 超时
//       （OVER_TIME_NUM=25000 @250kHz），超时重试 RETRY_MAX 次，仍失败
//       O_error<=1 回 st_idle
//    6. st_init_done 里清 O_error
//    7. 进 st_wait_cmd0 时清超时计数器（参考实现不清，多次重试会累积）
//    8. 超时重发前插 st_retry_gap：CS 拉高 >=8 个 250kHz 拍再重发
//       （卡在 CS 低的半截命令后可能不认重发的命令）
//    9. CMD55 回 0x00 视为卡已初始化，直接去 CMD58；ACMD41 连续回非 0
//       达到 ACMD41_MAX 次仍未完成 -> O_error 回 idle（防无界轮询）
//   10. CMD58 检查 R1==0：坏响应不更新 O_sdhc（否则 SDSC 会被误判成 SDHC）
//   11. CMD8 同时检查 echo 电压 0001 和 check pattern 0xAA
//   12. CMD55 回非 0x01/0x00 视为终结性错误（防无界重发）
//
//  终结性错误置 O_error 的分支：
//      六条超时重试用尽 / CMD0 回非 0x01 / CMD8 echo 或 pattern 错 /
//      CMD16 被拒 / ACMD41 轮询超限 / CMD58 回非 0x00 / CMD55 回非 0x01/0x00。
//    O_error 保持到 st_init_done 或 !I_start 才清。
//
//  时钟约定（SPI mode 0）：
//    发送侧在 div_clk 上升沿改 sd_mosi（= sd_clk 下降沿）；
//    SD 卡在 sd_clk 上升沿采样 MOSI；
//    SD 卡在 sd_clk 下降沿驱动 MISO，本模块在 div_clk_180deg（= ~div_clk，
//    即 sd_clk 上升沿）锁存 MISO。
//=============================================================================
module sd_init(
    input  wire       clk_ref,       // 25MHz
    input  wire       rst_n,         // 低有效
    input  wire       I_start,       // 初始化启动（电平，拉低=异步复位）
    input  wire       sd_miso,       // SD 卡 SPI 输入
    output wire       sd_clk,        // 250kHz 初始化时钟（= ~div_clk）
    output reg        sd_cs,         // 片选
    output reg        sd_mosi,       // SPI 输出
    output reg        sd_init_done,  // 初始化完成
    output reg        O_sdhc,        // 1=块地址（SDHC/SDXC），0=字节地址（SDSC）
    output reg        O_error        // 超时重试用尽 / CMD0 非 0x01 /
                                     // CMD8 echo 错 / CMD16 被拒 /
                                     // ACMD41 轮询超限 / CMD58 非 0
                                     // （电平，st_init_done 或 !I_start 清）
    );

//--------------------------------------------------------------------------
// 命令常量
//--------------------------------------------------------------------------
parameter  CMD0   = {8'h40,8'h00,8'h00,8'h00,8'h00,8'h95};
parameter  CMD8   = {8'h48,8'h00,8'h00,8'h01,8'haa,8'h87};
parameter  CMD55  = {8'h77,8'h00,8'h00,8'h00,8'h00,8'hff};
parameter  ACMD41 = {8'h69,8'h40,8'h00,8'h00,8'h00,8'hff};
parameter  CMD58  = {8'h7A,8'h00,8'h00,8'h00,8'h00,8'hFF};
parameter  CMD16  = {8'h50,8'h00,8'h00,8'h02,8'h00,8'hFF};

//--------------------------------------------------------------------------
// 参数
//--------------------------------------------------------------------------
parameter  DIV_FREQ      = 100;      // 25MHz/100 = 250kHz
parameter  POWER_ON_NUM  = 5000;     // 5000 x 4us = 20ms
parameter  OVER_TIME_NUM = 25000;    // 25000 x 4us = 100ms
parameter  RETRY_MAX     = 3;        // 同一命令最大尝试次数
parameter  ACMD41_MAX    = 100;      // ACMD41 轮询上限（防无界循环）
parameter  GAP_NUM       = 8;        // 重试前 CS 拉高的 250kHz 拍数

//--------------------------------------------------------------------------
// 状态编码
//--------------------------------------------------------------------------
localparam [3:0] st_idle        = 4'd0;   // 上电等待
localparam [3:0] st_send_cmd0   = 4'd1;   // 发 CMD0
localparam [3:0] st_wait_cmd0   = 4'd2;   // 等 CMD0 响应
localparam [3:0] st_send_cmd8   = 4'd3;   // 发 CMD8 + 等 R7
localparam [3:0] st_send_cmd55  = 4'd4;   // 发 CMD55 + 等 R1
localparam [3:0] st_send_acmd41 = 4'd5;   // 发 ACMD41 + 等 R1
localparam [3:0] st_send_cmd58  = 4'd6;   // 发 CMD58 + 等 R3
localparam [3:0] st_send_cmd16  = 4'd7;   // 发 CMD16 + 等 R1
localparam [3:0] st_retry_gap   = 4'd8;   // 超时重试前 CS 高间隙
localparam [3:0] st_init_done   = 4'd9;   // 初始化完成

//--------------------------------------------------------------------------
// reg / wire
//--------------------------------------------------------------------------
reg  [3:0]  cur_state;
reg  [3:0]  next_state;

reg  [7:0]  div_cnt;          // 分频计数器
reg         div_clk;          // 250kHz
reg  [12:0] poweron_cnt;      // 上电等待计数器
reg         res_en;           // 响应接收完成（1 拍）
reg  [47:0] res_data;         // 响应移位寄存器 /* synthesis keep */
reg         res_flag;         // 正在接收响应
reg  [5:0]  res_bit_cnt;      // 响应位计数

reg  [5:0]  cmd_bit_cnt;      // 命令位计数
reg  [15:0] over_time_cnt;    // 等待超时计数器
reg         over_time_en;     // 超时脉冲（1 拍）
reg  [3:0]  retry_cnt;        // 超时重试计数
reg  [3:0]  retry_target;     // 超时后重发的状态
reg  [3:0]  gap_cnt;          // CS 高间隙计数
reg  [7:0]  acmd41_poll_cnt;  // ACMD41 轮询次数

wire        div_clk_180deg;   // = ~div_clk，即 sd_clk 上升沿
wire        cmd_wait_phase;   // 48bit 命令已发完，处于等响应阶段
wire        wait_state;       // 当前处于任一需要超时保护的等待态
wire        acmd41_fatal;     // ACMD41 轮询超限

//--------------------------------------------------------------------------
// main code
//--------------------------------------------------------------------------
assign sd_clk          = ~div_clk;
assign div_clk_180deg  = ~div_clk;
assign cmd_wait_phase  = (cmd_bit_cnt > 6'd47);
assign wait_state      = (cur_state == st_wait_cmd0) ||
                         (((cur_state == st_send_cmd8)   ||
                           (cur_state == st_send_cmd55)  ||
                           (cur_state == st_send_acmd41) ||
                           (cur_state == st_send_cmd58)  ||
                           (cur_state == st_send_cmd16)) && cmd_wait_phase);
assign acmd41_fatal    = (cur_state == st_send_acmd41) && res_en &&
                         (res_data[47:40] != 8'h00) &&
                         (acmd41_poll_cnt >= ACMD41_MAX-1);

// 时钟分频：div_clk = 25MHz/100 = 250kHz（不受 I_start 影响，持续运行）
always @(posedge clk_ref or negedge rst_n) begin
    if (!rst_n) begin
        div_clk <= 1'b0;
        div_cnt <= 8'd0;
    end
    else begin
        if (div_cnt == DIV_FREQ/2 - 1) begin
            div_clk <= ~div_clk;
            div_cnt <= 8'd0;
        end
        else
            div_cnt <= div_cnt + 8'd1;
    end
end

// I_start 复位同步器：异步复位、同步释放（div_clk 域 2 级）。
// 直接拿 I_start 当异步复位时，释放沿若落在 div_clk 边沿附近可能让状态机
// 亚稳态；这里异步清零、同步释放，既支持任意长度的低脉冲中止初始化，
// 又保证释放沿安全。
reg  [1:0] start_rst_sync;
always @(posedge div_clk or negedge rst_n or negedge I_start) begin
    if (!rst_n)
        start_rst_sync <= 2'b00;       // 上电确定性：两级初值清 0
    else if (!I_start)
        start_rst_sync <= 2'b00;
    else
        start_rst_sync <= {start_rst_sync[0], 1'b1};
end
wire start_rst_n = start_rst_sync[1];

// 接收 SD 卡响应：在 div_clk_180deg 的上升沿（= sd_clk 上升沿）锁存 MISO。
// 起始条件为 MISO 的第一个 0（R1/R3/R7 的首字节最高位都是 0）。
// 统一按 6 字节接收，不足的部分由卡驱动的高电平补齐。
// I_start 拉低按异步复位处理（保证任意长度的低脉冲都能中止初始化）。
always @(posedge div_clk_180deg or negedge rst_n or negedge start_rst_n) begin
    if (!rst_n) begin
        res_en      <= 1'b0;
        res_data    <= 48'd0;
        res_flag    <= 1'b0;
        res_bit_cnt <= 6'd0;
    end
    else if (!I_start || !start_rst_n) begin
        res_en      <= 1'b0;
        res_data    <= 48'd0;
        res_flag    <= 1'b0;
        res_bit_cnt <= 6'd0;
    end
    else begin
        if (sd_miso == 1'b0 && res_flag == 1'b0) begin
            res_flag    <= 1'b1;
            res_data    <= {res_data[46:0], sd_miso};
            res_bit_cnt <= 6'd1;
            res_en      <= 1'b0;
        end
        else if (res_flag) begin
            res_data    <= {res_data[46:0], sd_miso};
            if (res_bit_cnt == 6'd47) begin
                res_flag    <= 1'b0;
                res_bit_cnt <= 6'd0;
                res_en      <= 1'b1;
            end
            else
                res_bit_cnt <= res_bit_cnt + 6'd1;
        end
        else
            res_en <= 1'b0;
    end
end

// 状态寄存器（I_start 拉低异步复位、同步释放）
always @(posedge div_clk or negedge rst_n or negedge start_rst_n) begin
    if (!rst_n)
        cur_state <= st_idle;
    else if (!I_start || !start_rst_n)
        cur_state <= st_idle;
    else
        cur_state <= next_state;
end

// 次态逻辑
always @(*) begin
    next_state = cur_state;
    case (cur_state)
        st_idle : begin
            if (poweron_cnt == POWER_ON_NUM)
                next_state = st_send_cmd0;
        end
        st_send_cmd0 : begin
            if (cmd_bit_cnt == 6'd47)
                next_state = st_wait_cmd0;
        end
        st_wait_cmd0 : begin
            if (res_en) begin
                if (res_data[47:40] == 8'h01)     // 复位成功，仍在 idle
                    next_state = st_send_cmd8;
                else
                    next_state = st_idle;         // 坏卡：output 里置 O_error
            end
            else if (over_time_en)
                next_state = (retry_cnt >= RETRY_MAX-1) ? st_idle : st_retry_gap;
        end
        st_send_cmd8 : begin
            if (res_en) begin
                // R7: R1 + 00 00 01 AA（电压 0001 + check pattern 0xAA）
                if ((res_data[19:16] == 4'b0001) &&
                    (res_data[15:8]  == 8'haa))
                    next_state = st_send_cmd55;
                else
                    next_state = st_idle;
            end
            else if (over_time_en)
                next_state = (retry_cnt >= RETRY_MAX-1) ? st_idle : st_retry_gap;
        end
        st_send_cmd55 : begin
            if (res_en) begin
                if (res_data[47:40] == 8'h01)     // 卡仍 idle
                    next_state = st_send_acmd41;
                else if (res_data[47:40] == 8'h00) // 已初始化，跳过 ACMD41
                    next_state = st_send_cmd58;
                else
                    next_state = st_idle;         // 非 0x01/0x00：终结性错误，
                                                  // output 里置 O_error（防无界重发）
            end
            else if (over_time_en)
                next_state = (retry_cnt >= RETRY_MAX-1) ? st_idle : st_retry_gap;
        end
        st_send_acmd41 : begin
            if (res_en) begin
                if (res_data[47:40] == 8'h00)     // 初始化完成
                    next_state = st_send_cmd58;
                else if (acmd41_fatal)            // 轮询超限
                    next_state = st_idle;
                else
                    next_state = st_send_cmd55;   // 卡还在忙
            end
            else if (over_time_en)
                next_state = (retry_cnt >= RETRY_MAX-1) ? st_idle : st_retry_gap;
        end
        st_send_cmd58 : begin
            if (res_en) begin
                if (res_data[47:40] != 8'h00)     // R1 错：不采 CCS
                    next_state = st_idle;
                else
                    next_state = res_data[38] ? st_init_done : st_send_cmd16;
            end
            else if (over_time_en)
                next_state = (retry_cnt >= RETRY_MAX-1) ? st_idle : st_retry_gap;
        end
        st_send_cmd16 : begin
            if (res_en) begin
                if (res_data[47:40] == 8'h00)
                    next_state = st_init_done;
                else
                    next_state = st_idle;         // 参数被拒
            end
            else if (over_time_en)
                next_state = (retry_cnt >= RETRY_MAX-1) ? st_idle : st_retry_gap;
        end
        st_retry_gap : begin
            // CS 拉高 >= GAP_NUM 个 250kHz 拍后重发
            if (gap_cnt >= GAP_NUM-1)
                next_state = retry_target;
            else
                next_state = st_retry_gap;
        end
        st_init_done : next_state = st_init_done;
        default :      next_state = st_idle;
    endcase
end

// 输出/计数器（div_clk 上升沿；sd_clk = ~div_clk，因此 MOSI 在 sd_clk 下降沿变化）
// I_start 拉低异步复位、同步释放
always @(posedge div_clk or negedge rst_n or negedge start_rst_n) begin
    if (!rst_n) begin
        sd_cs          <= 1'b1;
        sd_mosi        <= 1'b1;
        sd_init_done   <= 1'b0;
        O_sdhc         <= 1'b0;
        O_error        <= 1'b0;
        cmd_bit_cnt    <= 6'd0;
        over_time_cnt  <= 16'd0;
        over_time_en   <= 1'b0;
        retry_cnt      <= 4'd0;
        retry_target   <= st_send_cmd0;
        gap_cnt        <= 4'd0;
        acmd41_poll_cnt<= 8'd0;
        poweron_cnt    <= 13'd0;
    end
    else if (!I_start || !start_rst_n) begin
        sd_cs          <= 1'b1;
        sd_mosi        <= 1'b1;
        sd_init_done   <= 1'b0;
        O_sdhc         <= 1'b0;
        O_error        <= 1'b0;
        cmd_bit_cnt    <= 6'd0;
        over_time_cnt  <= 16'd0;
        over_time_en   <= 1'b0;
        retry_cnt      <= 4'd0;
        retry_target   <= st_send_cmd0;
        gap_cnt        <= 4'd0;
        acmd41_poll_cnt<= 8'd0;
        poweron_cnt    <= 13'd0;
    end
    else begin
        over_time_en <= 1'b0;

        // ---- 初始化失败标志（电平，保持到 init_done / !I_start）----
        // 六条超时重试用尽：!res_en 避免“第 3 次超时那拍恰好响应到达”误置
        if (over_time_en && !res_en && (retry_cnt >= RETRY_MAX-1))
            O_error <= 1'b1;
        // ACMD41 轮询超限
        if (acmd41_fatal)
            O_error <= 1'b1;

        // ---- 超时计数（只在等待响应期间计数；离开等待态立即清零）----
        if (!wait_state) begin
            over_time_cnt <= 16'd0;
        end
        else if (over_time_cnt == OVER_TIME_NUM - 1) begin
            over_time_cnt <= 16'd0;
            over_time_en  <= 1'b1;
        end
        else begin
            over_time_cnt <= over_time_cnt + 16'd1;
        end

        // ---- 重试计数：响应优先（同拍 res_en 清零，不加 1）；
        //      纯超时才加 1；idle 清零 ----
        if (cur_state == st_idle)
            retry_cnt <= 4'd0;
        else if (res_en)
            retry_cnt <= 4'd0;
        else if (over_time_en)
            retry_cnt <= retry_cnt + 4'd1;

        // ---- ACMD41 轮询计数（每次 ACMD41 回非 0 加 1；idle 清零）----
        if (cur_state == st_idle)
            acmd41_poll_cnt <= 8'd0;
        else if ((cur_state == st_send_acmd41) && res_en &&
                 (res_data[47:40] != 8'h00))
            acmd41_poll_cnt <= acmd41_poll_cnt + 8'd1;

        // ---- 上电等待计数 ----
        if (cur_state == st_idle) begin
            if (poweron_cnt < POWER_ON_NUM)
                poweron_cnt <= poweron_cnt + 13'd1;
        end
        else
            poweron_cnt <= 13'd0;

        case (cur_state)
            st_idle : begin
                sd_cs   <= 1'b1;
                sd_mosi <= 1'b1;
            end
            st_send_cmd0 : begin
                cmd_bit_cnt <= cmd_bit_cnt + 6'd1;
                sd_cs       <= 1'b0;
                sd_mosi     <= CMD0[6'd47 - cmd_bit_cnt];
                if (cmd_bit_cnt == 6'd47)
                    cmd_bit_cnt <= 6'd0;
            end
            st_wait_cmd0 : begin
                sd_mosi <= 1'b1;
                if (res_en) begin
                    sd_cs <= 1'b1;             // 收到响应后释放 CS，进入 SPI 模式
                    // CMD0 只能回 0x01（idle）；其它值 = 卡异常/无卡
                    if (res_data[47:40] != 8'h01)
                        O_error <= 1'b1;
                end
                else if (over_time_en) begin
                    retry_target <= st_send_cmd0;
                end
            end
            st_send_cmd8 : begin
                if (!cmd_wait_phase) begin
                    cmd_bit_cnt <= cmd_bit_cnt + 6'd1;
                    sd_cs       <= 1'b0;
                    sd_mosi     <= CMD8[6'd47 - cmd_bit_cnt];
                end
                else begin
                    sd_mosi <= 1'b1;
                    if (res_en) begin
                        sd_cs       <= 1'b1;
                        cmd_bit_cnt <= 6'd0;
                        // R7 echo 必须是 0001 + check pattern AA
                        if ((res_data[19:16] != 4'b0001) ||
                            (res_data[15:8]  != 8'haa))
                            O_error <= 1'b1;
                    end
                    else if (over_time_en) begin
                        cmd_bit_cnt  <= 6'd0;
                        retry_target <= st_send_cmd8;
                    end
                end
            end
            st_send_cmd55 : begin
                if (!cmd_wait_phase) begin
                    cmd_bit_cnt <= cmd_bit_cnt + 6'd1;
                    sd_cs       <= 1'b0;
                    sd_mosi     <= CMD55[6'd47 - cmd_bit_cnt];
                end
                else begin
                    sd_mosi <= 1'b1;
                    if (res_en) begin
                        sd_cs       <= 1'b1;
                        cmd_bit_cnt <= 6'd0;
                        // 非 0x01/0x00：终结性错误（不能无限重发 CMD55）
                        if ((res_data[47:40] != 8'h01) &&
                            (res_data[47:40] != 8'h00))
                            O_error <= 1'b1;
                    end
                    else if (over_time_en) begin
                        cmd_bit_cnt  <= 6'd0;
                        retry_target <= st_send_cmd55;
                    end
                end
            end
            st_send_acmd41 : begin
                if (!cmd_wait_phase) begin
                    cmd_bit_cnt <= cmd_bit_cnt + 6'd1;
                    sd_cs       <= 1'b0;
                    sd_mosi     <= ACMD41[6'd47 - cmd_bit_cnt];
                end
                else begin
                    sd_mosi <= 1'b1;
                    if (res_en) begin
                        sd_cs       <= 1'b1;
                        cmd_bit_cnt <= 6'd0;
                    end
                    else if (over_time_en) begin
                        // ACMD41 必须在 CMD55 之后，重试整个 CMD55+ACMD41
                        cmd_bit_cnt  <= 6'd0;
                        retry_target <= st_send_cmd55;
                    end
                end
            end
            st_send_cmd58 : begin
                if (!cmd_wait_phase) begin
                    cmd_bit_cnt <= cmd_bit_cnt + 6'd1;
                    sd_cs       <= 1'b0;
                    sd_mosi     <= CMD58[6'd47 - cmd_bit_cnt];
                end
                else begin
                    sd_mosi <= 1'b1;
                    if (res_en) begin
                        sd_cs       <= 1'b1;
                        cmd_bit_cnt <= 6'd0;
                        if (res_data[47:40] != 8'h00)
                            O_error <= 1'b1;   // R1 错：不采 CCS
                        else
                            O_sdhc  <= res_data[38];   // OCR bit30 = CCS
                    end
                    else if (over_time_en) begin
                        cmd_bit_cnt  <= 6'd0;
                        retry_target <= st_send_cmd58;
                    end
                end
            end
            st_send_cmd16 : begin
                if (!cmd_wait_phase) begin
                    cmd_bit_cnt <= cmd_bit_cnt + 6'd1;
                    sd_cs       <= 1'b0;
                    sd_mosi     <= CMD16[6'd47 - cmd_bit_cnt];
                end
                else begin
                    sd_mosi <= 1'b1;
                    if (res_en) begin
                        sd_cs       <= 1'b1;
                        cmd_bit_cnt <= 6'd0;
                        if (res_data[47:40] != 8'h00)
                            O_error <= 1'b1;   // CMD16 被拒
                    end
                    else if (over_time_en) begin
                        cmd_bit_cnt  <= 6'd0;
                        retry_target <= st_send_cmd16;
                    end
                end
            end
            st_retry_gap : begin
                // 超时重发前拉高 CS，保持 >= GAP_NUM 个 250kHz 拍
                sd_cs   <= 1'b1;
                sd_mosi <= 1'b1;
                if (gap_cnt >= GAP_NUM-1)
                    gap_cnt <= 4'd0;
                else
                    gap_cnt <= gap_cnt + 4'd1;
            end
            st_init_done : begin
                sd_init_done <= 1'b1;
                O_error      <= 1'b0;
                sd_cs        <= 1'b1;
                sd_mosi      <= 1'b1;
                cmd_bit_cnt  <= 6'd0;
            end
            default : begin
                sd_cs   <= 1'b1;
                sd_mosi <= 1'b1;
            end
        endcase
    end
end

endmodule
