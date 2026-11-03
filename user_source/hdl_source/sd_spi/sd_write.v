//=============================================================================
//  sd_write.v  ---  SD 卡 SPI 单块写（CMD24，512 字节），移植并加固
//
//  相对参考工程 sd_ctrl_top/sd_write.v 的改动（本任务补丁）：
//    0. 单时钟：删除 clk_ref_180deg，主状态机用 posedge clk；
//       接收响应/检测卡 busy 的 always 块用 negedge clk
//       （sd_clk = ~clk，negedge clk 就是 sd_clk 上升沿，MISO 的稳定采样点）
//    1. 增加 I_sdhc（地址模式）、O_error（写失败，电平，下一块启动时清）、
//       O_r1（最近一次 CMD24 的 R1）
//    2. CMD24 参数按 I_sdhc 换算：
//       SDHC/SDXC（块地址）: arg = LBA
//       SDSC（字节地址）    : arg = LBA << 9
//    3. 修参考实现的第 257 个 wr_req：只有 data_cnt < 255 才提前请求下一字，
//       每块正好 256 个 wr_req（配合块头发出的第一个请求 = 256 字）
//    4. state 1 收到 R1 后检查 res_data != 8'h00 -> O_error=1 回 state 0
//    5. state1/5 加 16bit 超时（~2.6ms @25MHz）；state6 加 24bit 超时
//       （500ms = 12,500,000 @25MHz）；超时 -> O_error=1 回 state 0
//    6. state 5 检查数据响应 token：res_data[7:3] != 5'b00101 -> O_error=1。
//       线上 token = xxx0 SSS 1（空闲 MISO 为 1，accepted = 0xE5）；
//       接收器从第一个 0（起始位）起捕获 8 位，捕获 0xE5 得到 0x28，
//       [7:3] = 起始 0 + 状态 010 + 结束 1 = 5'b00101；0xEB 捕获 0x58
//       （[7:3]=01011）会被拒绝。
//
//  用户接口契约（sd_ctrl 头注释同步）：
//    wr_start_en : 单拍启动（sd_ctrl 里与 init_done 相与）
//    wr_req      : 1 拍请求，在 state2 头字节 bit_cnt==14 / state3 每字
//                  bit_cnt==14 拉高，2 拍后才采样 wr_data；上游 FIFO 用
//                  wr_req 作读使能（寄存输出）即可对上
//    wr_busy     : 覆盖整个块（命令 + 数据 + token + 卡 busy）
//    wr_data     : 在 wr_req 之后的下一个时钟沿更新、保持到下一次 wr_req
//=============================================================================
module sd_write(
    input  wire        clk,          // 25MHz（sd_clk = ~clk）
    input  wire        rst_n,        // 低有效
    input  wire        sd_miso,      // SD 卡 SPI 输入
    output reg         sd_cs,        // 片选
    output reg         sd_mosi,      // SPI 输出
    // 用户写接口
    input  wire        wr_start_en,  // 单拍启动
    input  wire [31:0] wr_sec_addr,  // 写扇区/LBA
    input  wire [15:0] wr_data,      // 写数据
    output reg         wr_busy,      // 写忙
    output reg         wr_req,       // 取下一字（1 拍）
    // 配置/状态
    input  wire        I_sdhc,       // 1=块地址，0=字节地址
    output reg         O_error,      // 写失败（电平，保持到下一块启动才清）
    output reg  [7:0]  O_r1          // 最近一次 CMD24 的 R1（成功/失败都记录）
    );

//--------------------------------------------------------------------------
// 参数
//--------------------------------------------------------------------------
parameter  HEAD_BYTE       = 8'hfe;            // 数据头
parameter  TIME_OUT_SHORT  = 16'hffff;         // 65535/25MHz = 2.62ms
parameter  TIME_OUT_LONG   = 24'd12_500_000;   // 12.5e6/25MHz = 500ms
                                               // （可被实例参数覆盖，便于小值仿真）

//--------------------------------------------------------------------------
// reg / wire
//--------------------------------------------------------------------------
reg            wr_en_d0;          // wr_start_en 打拍
reg            wr_en_d1;
reg            res_en;            // 收到 8bit 响应（1 拍）
reg    [7:0]   res_data;          // 响应数据
reg            res_flag;          // 正在接收响应
reg    [5:0]   res_bit_cnt;       // 响应位计数

reg    [3:0]   wr_ctrl_cnt;       // 写控制状态
reg    [47:0]  cmd_wr;            // CMD24 命令字
reg    [5:0]   cmd_bit_cnt;       // 命令位计数
reg    [3:0]   bit_cnt;           // 位计数（每字 0~15）
reg    [8:0]   data_cnt;          // 已发字数 0~255
reg    [15:0]  wr_data_t;         // 寄存写数据
reg            detect_done_flag;  // 检测卡 busy 使能
reg    [7:0]   detect_data;       // 卡 busy 检测移位

// 超时
reg    [23:0]  over_time_cnt;
reg            to_en;             // 超时脉冲（1 拍）
reg    [3:0]   wr_ctrl_cnt_d;     // 状态延时，用于进入新状态时清超时计数
wire           to_wait;           // 处于需要超时保护的等待阶段
wire           to_hit;            // 计数到阈值

wire           pos_wr_en;         // wr_start_en 上升沿（1 拍）

//--------------------------------------------------------------------------
// main code
//--------------------------------------------------------------------------
assign pos_wr_en = (~wr_en_d1) & wr_en_d0;

// 超时判断：state1 等 R1 / state5 等数据响应 token / state6 等卡 busy
assign to_wait = ((wr_ctrl_cnt == 4'd1) && (cmd_bit_cnt > 6'd47)) ||
                 (wr_ctrl_cnt == 4'd5) ||
                 (wr_ctrl_cnt == 4'd6);
assign to_hit  = (wr_ctrl_cnt == 4'd6) ? (over_time_cnt >= TIME_OUT_LONG - 1)
                                       : (over_time_cnt >= TIME_OUT_SHORT - 1);

// wr_start_en 打拍
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        wr_en_d0 <= 1'b0;
        wr_en_d1 <= 1'b0;
    end
    else begin
        wr_en_d0 <= wr_start_en;
        wr_en_d1 <= wr_en_d0;
    end
end

// 接收 SD 卡响应：negedge clk = sd_clk 上升沿（MISO 稳定点）
always @(negedge clk or negedge rst_n) begin
    if (!rst_n) begin
        res_en      <= 1'b0;
        res_data    <= 8'd0;
        res_flag    <= 1'b0;
        res_bit_cnt <= 6'd0;
    end
    else begin
        // MISO 的第一个 0 作为起始位（R1 / data response token 都以 0 开头）
        if (sd_miso == 1'b0 && res_flag == 1'b0) begin
            res_flag    <= 1'b1;
            res_data    <= {res_data[6:0], sd_miso};
            res_bit_cnt <= 6'd1;
            res_en      <= 1'b0;
        end
        else if (res_flag) begin
            res_data <= {res_data[6:0], sd_miso};
            if (res_bit_cnt == 6'd7) begin
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

// 检测卡 busy：和接收响应一样在 negedge clk（= sd_clk 上升沿）采样 MISO。
// 卡在 sd_clk 下降沿驱动 MISO，这里错开半拍采样，避免同时沿竞争。
always @(negedge clk or negedge rst_n) begin
    if (!rst_n)
        detect_data <= 8'd0;
    else if (detect_done_flag)
        detect_data <= {detect_data[6:0], sd_miso};
    else
        detect_data <= 8'd0;
end

// 超时计数器
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        over_time_cnt <= 24'd0;
        to_en         <= 1'b0;
        wr_ctrl_cnt_d <= 4'd0;
    end
    else begin
        wr_ctrl_cnt_d <= wr_ctrl_cnt;
        if (!to_wait || (wr_ctrl_cnt != wr_ctrl_cnt_d)) begin
            over_time_cnt <= 24'd0;
            to_en         <= 1'b0;
        end
        else if (to_hit) begin
            over_time_cnt <= 24'd0;
            to_en         <= 1'b1;
        end
        else begin
            over_time_cnt <= over_time_cnt + 24'd1;
            to_en         <= 1'b0;
        end
    end
end

// 写状态机
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        sd_cs            <= 1'b1;
        sd_mosi          <= 1'b1;
        wr_ctrl_cnt      <= 4'd0;
        wr_busy          <= 1'b0;
        cmd_wr           <= 48'd0;
        cmd_bit_cnt      <= 6'd0;
        bit_cnt          <= 4'd0;
        wr_data_t        <= 16'd0;
        data_cnt         <= 9'd0;
        wr_req           <= 1'b0;
        detect_done_flag <= 1'b0;
        O_error          <= 1'b0;
        O_r1             <= 8'd0;
    end
    else begin
        wr_req <= 1'b0;
        case (wr_ctrl_cnt)
            //------------------------------------------------------------
            4'd0 : begin                     // 空闲
                wr_busy  <= 1'b0;
                sd_cs    <= 1'b1;
                sd_mosi  <= 1'b1;
                if (pos_wr_en) begin
                    // CMD24 参数：SDHC 用 LBA，SDSC 用字节地址 LBA<<9。
                    // SDSC 取 wr_sec_addr[22:0] 左移 9 位 = 32bit 字节地址，
                    // 对应 SDSC 上限 2^23 块 = 4GiB。
                    cmd_wr      <= {8'h58,
                                    I_sdhc ? wr_sec_addr : {wr_sec_addr[22:0], 9'b0},
                                    8'hff};
                    wr_ctrl_cnt <= 4'd1;
                    wr_busy     <= 1'b1;
                    O_error     <= 1'b0;     // 新块清错误
                    data_cnt    <= 9'd0;
                    bit_cnt     <= 4'd0;
                end
            end
            //------------------------------------------------------------
            4'd1 : begin                     // 发 CMD24 + 等 R1
                if (cmd_bit_cnt <= 6'd47) begin
                    cmd_bit_cnt <= cmd_bit_cnt + 6'd1;
                    sd_cs       <= 1'b0;
                    sd_mosi     <= cmd_wr[6'd47 - cmd_bit_cnt];
                end
                else begin
                    sd_mosi <= 1'b1;
                    if (res_en) begin
                        // O_r1 = 最近一次 CMD24 的 R1，成功/失败都记录
                        O_r1 <= res_data;
                        if (res_data != 8'h00) begin
                            O_error     <= 1'b1;
                            wr_ctrl_cnt <= 4'd0;
                            cmd_bit_cnt <= 6'd0;
                        end
                        else begin
                            wr_ctrl_cnt <= 4'd2;
                            cmd_bit_cnt <= 6'd0;
                            bit_cnt     <= 4'd1;
                        end
                    end
                    else if (to_en) begin
                        O_error     <= 1'b1;
                        wr_ctrl_cnt <= 4'd0;
                        cmd_bit_cnt <= 6'd0;
                    end
                end
            end
            //------------------------------------------------------------
            4'd2 : begin                     // 间隔 + 数据头 0xFE
                bit_cnt <= bit_cnt + 4'd1;
                // state1 已把 bit_cnt 置 1：bit_cnt=1~7 共 7 拍空闲（MOSI=1），
                // bit_cnt=8~15 发送数据头 0xFE（MSB first）
                if (bit_cnt >= 4'd8 && bit_cnt <= 4'd15) begin
                    sd_mosi <= HEAD_BYTE[4'd15 - bit_cnt];
                    if (bit_cnt == 4'd14)
                        wr_req <= 1'b1;      // 请求第一个数据字
                    else if (bit_cnt == 4'd15)
                        wr_ctrl_cnt <= wr_ctrl_cnt + 4'd1;
                end
            end
            //------------------------------------------------------------
            4'd3 : begin                     // 写 256 个 16bit 字
                bit_cnt <= bit_cnt + 4'd1;
                if (bit_cnt == 4'd0) begin
                    sd_mosi   <= wr_data[4'd15 - bit_cnt];
                    wr_data_t <= wr_data;    // 寄存，防止发送中途变化
                end
                else
                    sd_mosi <= wr_data_t[4'd15 - bit_cnt];
                // 补丁 3：data_cnt == 255（最后一个字）不再请求下一字。
                // 参考写成 <= 255，每块多出第 257 个 wr_req，会把下一块的
                // 第一个字提前弹掉。
                if ((bit_cnt == 4'd14) && (data_cnt < 9'd255))
                    wr_req <= 1'b1;
                if (bit_cnt == 4'd15) begin
                    data_cnt <= data_cnt + 9'd1;
                    if (data_cnt == 9'd255) begin
                        data_cnt    <= 9'd0;
                        wr_ctrl_cnt <= wr_ctrl_cnt + 4'd1;
                    end
                end
            end
            //------------------------------------------------------------
            4'd4 : begin                     // 2 字节 CRC（SPI 模式不检查）
                bit_cnt <= bit_cnt + 4'd1;
                sd_mosi <= 1'b1;
                if (bit_cnt == 4'd15)
                    wr_ctrl_cnt <= wr_ctrl_cnt + 4'd1;
            end
            //------------------------------------------------------------
            4'd5 : begin                     // 等数据响应 token
                if (res_en) begin
                    // 数据响应 token 线上格式: x x x 0 S S S 1
                    //   （空闲 MISO = 1，accepted = 8'hE5，CRC 错 = 8'hEB，
                    //     写错 = 8'hED）
                    // 接收器从 MISO 第一个 0（起始位）起捕获 8 位：accepted
                    //   0xE5 捕获为 0x28，[7:3] = 0 + 010 + 1 = 5'b00101。
                    //   同时检查起始位/结束位，比只看 [6:4] 更严格。
                    if (res_data[7:3] != 5'b00101) begin
                        O_error     <= 1'b1;
                        wr_ctrl_cnt <= 4'd0;
                        cmd_bit_cnt <= 6'd0;
                    end
                    else
                        wr_ctrl_cnt <= wr_ctrl_cnt + 4'd1;
                end
                else if (to_en) begin
                    O_error     <= 1'b1;
                    wr_ctrl_cnt <= 4'd0;
                    cmd_bit_cnt <= 6'd0;
                end
            end
            //------------------------------------------------------------
            4'd6 : begin                     // 等卡 busy 释放（MISO 回到高）
                detect_done_flag <= 1'b1;
                if (detect_data == 8'hff) begin
                    wr_ctrl_cnt      <= wr_ctrl_cnt + 4'd1;
                    detect_done_flag <= 1'b0;
                end
                else if (to_en) begin
                    O_error          <= 1'b1;
                    detect_done_flag <= 1'b0;
                    wr_ctrl_cnt      <= 4'd0;
                    cmd_bit_cnt      <= 6'd0;
                end
            end
            //------------------------------------------------------------
            default : begin                  // 拉高 CS，state 7..15 共 9 拍后回空闲
                sd_cs       <= 1'b1;
                wr_ctrl_cnt <= wr_ctrl_cnt + 4'd1;
            end
        endcase
    end
end

endmodule
