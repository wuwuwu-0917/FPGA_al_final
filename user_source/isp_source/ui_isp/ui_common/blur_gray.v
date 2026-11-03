//=============================================================================
//  blur_gray.v --- 大半径灰度模糊（纵向盒式滤波）
//
//  来源：FPGA_lvjing/rtl/blur_gray.v
//  相对原文件的改动（都为了综合，算法一字未动）：
//    1) `row_cnt % NR` 换成 mod-NR 寄存器 rslot
//       —— 原写法对 32bit 计数器取模 33，综合会生出一个除法器（几百 LUT）。
//    2) `acc[4][480]` 三维数组改成 generate 内的每 lane 独立一维数组
//       —— TD 不支持三维非压缩数组；而且一维独立数组才能各自推成双口 RAM。
//    3) 所有乘法显式写位宽（Verilog 的乘法位宽由上下文决定，显式写更保险）
//    4) 存储阵列加 `/* fehdl force_ram=1 */`
//    5) 删掉 O_word_idx（纯调试）
//
//  ---------------------------------------------------------------------------
//  【算法】blur(x,y) = (1/NR) * Σ gray(x, y')   y' = y-RADIUS .. y+RADIUS
//          NR = 2*RADIUS + 1
//      实现是滑动求和：写第 r 行时 acc[w] <= acc[w] + gray_new - gray_old，
//      其中 gray_old 就是准备被覆盖的那一格（read-before-write）。
//
//  【输出的行偏移】输出字序号 n 代表 **行 (row(n) - RADIUS)、字 word(n)** 的模糊值。
//      也就是说输出在空间上比输入早 RADIUS 行 —— 由 pencil_front 用 word_delay 对齐。
//
//  【时间延迟】2 拍（BRAM 读 1 拍 + 计算寄存 1 拍）
//
//  【暖机】复位后前 NR 行累加器还没填满，输出偏小。真机是连续视频流，
//          只有开机后第一帧的前 NR 行受影响，可接受。
//
//  【为什么行号刻意不在 I_tuser 处复位】
//      ring 下标是 row mod NR；LINES_PER_FRAME(=1080) 不是 NR(=33) 的整数倍，
//      每帧清 0 会让 ring 相位跳一下 -> 每帧开头几行模糊全错。
//      纵向滑动本来就是跨帧连续的，所以只在 !I_rst_n 时清。
//=============================================================================
`timescale 1ns/1ps

module blur_gray #(
    parameter integer PIX_PER_WORD    = 4,
    parameter integer RGB_W           = PIX_PER_WORD * 24,   // 96
    parameter integer GRAY_W          = PIX_PER_WORD * 8,    // 32
    parameter integer WORDS_PER_LINE  = 480,
    parameter integer RADIUS          = 16                  // 纵向半径（行）
)(
    input  wire                  I_clk,
    input  wire                  I_rst_n,

    input  wire [RGB_W-1:0]      I_tdata,
    input  wire                  I_tvalid,
    input  wire                  I_tlast,
    input  wire                  I_tuser,

    // 4 个像素的模糊灰度：像素 p 在 O_blur[p*8 +: 8]
    // ⚠️ 代表 **行 (row - RADIUS)** 那一行
    output reg  [GRAY_W-1:0]     O_blur,

    output reg                   O_tvalid,
    output reg                   O_tlast,
    output reg                   O_tuser
);

    localparam integer NR      = 2*RADIUS + 1;
    localparam integer ACC_W   = 16;                     // 255*65 = 16575 < 65536
    localparam integer ACC_MAX = 255 * NR;
    localparam integer LAT     = 2;                      // 时间延迟（拍）

    //-------------------------------------------------------------------------
    // 本地 clog2
    //-------------------------------------------------------------------------
    function integer clog2_f;
        input integer v;
        integer i;
        begin
            clog2_f = 0;
            for (i = v - 1; i > 0; i = i >> 1)
                clog2_f = clog2_f + 1;
        end
    endfunction

    localparam integer AW = clog2_f(NR * WORDS_PER_LINE);

    //-------------------------------------------------------------------------
    // 除法常数：从 NR 推出来，不给外部传（传两个参数太容易不同步）
    //   已逐值验过：x = 0..255*NR 全部满足 (x*C)>>S == x/NR
    //-------------------------------------------------------------------------
    function integer divc_of;
        input integer nr;
        begin
            case (nr)
                17:      divc_of = 15421;   // S=18
                33:      divc_of = 1986;    // S=16
                65:      divc_of = 4033;    // S=18
                default: divc_of = 0;
            endcase
        end
    endfunction

    function integer divs_of;
        input integer nr;
        begin
            case (nr)
                17:      divs_of = 18;
                33:      divs_of = 16;
                65:      divs_of = 18;
                default: divs_of = 0;
            endcase
        end
    endfunction

    localparam integer DIV_C = divc_of(NR);
    localparam integer DIV_S = divs_of(NR);

    localparam [15:0] DIV_C16 = DIV_C;
    localparam [AW-1:0] WPL_L = WORDS_PER_LINE;

    //-------------------------------------------------------------------------
    // 灰度
    //-------------------------------------------------------------------------
    wire [GRAY_W-1:0] in_gray;

    rgb2gray #(
        .PIX_PER_WORD (PIX_PER_WORD)
    ) u_rgb2gray (
        .I_rgb  (I_tdata ),
        .O_gray (in_gray)
    );

    //-------------------------------------------------------------------------
    // 输入计数器（靠字数推进，不在 tuser 处清 0）
    //-------------------------------------------------------------------------
    reg [15:0] w_cnt;
    reg [5:0]  rslot;        // row_cnt mod NR（NR <= 64）
    reg [31:0] row_cnt;      // 真实行号（只用于暖机判断）

    always @(posedge I_clk) begin
        if (!I_rst_n) begin
            w_cnt   <= 16'd0;
            rslot   <= 6'd0;
            row_cnt <= 32'd0;
        end else if (I_tvalid) begin
            if (w_cnt == WORDS_PER_LINE - 1) begin
                w_cnt   <= 16'd0;
                rslot   <= (rslot == NR-1) ? 6'd0 : (rslot + 6'd1);
                row_cnt <= row_cnt + 32'd1;
            end else begin
                w_cnt <= w_cnt + 16'd1;
            end
        end
    end

    //-------------------------------------------------------------------------
    // 行缓存：NR 行，每行 WORDS_PER_LINE 个 GRAY_W 位灰度字
    //   同一拍对同一地址先读后写：读到的是 NR 行之前写进去的那一行
    //-------------------------------------------------------------------------
    (* ram_style = "block" *) reg [GRAY_W-1:0] mem [0:NR*WORDS_PER_LINE-1];  /* fehdl force_ram=1, ram_style="bram" */

    wire [AW-1:0] addr = rslot * WPL_L + w_cnt;

    reg [GRAY_W-1:0] old_row;      // 离开窗口的那一行（NR 行前写的）

    always @(posedge I_clk) begin
        if (I_tvalid) begin
            old_row   <= mem[addr];      // 先读（拿到旧值）
            mem[addr] <= in_gray;        // 后写 —— BRAM "read-first" 模式
        end
    end

    //-------------------------------------------------------------------------
    // 打一拍，和新行 / 旧行对齐
    //-------------------------------------------------------------------------
    reg [GRAY_W-1:0] in_gray_d1;
    reg              vld_d1;
    reg [15:0]       w_cnt_d1;
    reg [31:0]       row_cnt_d1;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            in_gray_d1 <= {GRAY_W{1'b0}};
            vld_d1     <= 1'b0;
            w_cnt_d1   <= 16'd0;
            row_cnt_d1 <= 32'd0;
        end else begin
            in_gray_d1 <= in_gray;
            vld_d1     <= I_tvalid;
            w_cnt_d1   <= w_cnt;
            row_cnt_d1 <= row_cnt;
        end
    end

    //-------------------------------------------------------------------------
    // 暖机保护（没有这两条，仿真会是一坨清不掉的 X）
    //   acc_init : 第 0 行直接把新值当累加值，不读 acc_q、不减 old_row
    //   warm_ok  : 前 NR 行只加不减
    //-------------------------------------------------------------------------
    wire acc_init = (row_cnt_d1 == 32'd0);
    wire warm_ok  = (row_cnt_d1 >= NR);

    wire [GRAY_W-1:0] blur_comb;    // 四条 lane 的除法结果，组合

    genvar l;
    generate
        for (l = 0; l < PIX_PER_WORD; l = l + 1) begin : g_acc

            // 每 lane 一块独立的累加器 RAM（不能用三维数组，见文件头改动 2）
            (* ram_style = "block" *) reg [ACC_W-1:0] acc_mem [0:WORDS_PER_LINE-1];  /* fehdl force_ram=1, ram_style="bram" */
            reg [ACC_W-1:0] acc_q;

            wire [ACC_W-1:0] new_px = {8'd0, in_gray_d1[l*8 +: 8]};
            wire [ACC_W-1:0] old_px = (warm_ok && !acc_init)
                                    ? {8'd0, old_row[l*8 +: 8]} : {ACC_W{1'b0}};

            wire [ACC_W-1:0] new_acc = acc_init ? new_px : (acc_q + new_px - old_px);

            // 同一 always 内先读后写 -> 推成简单双口 RAM
            always @(posedge I_clk) begin
                if (I_tvalid) acc_q          <= acc_mem[w_cnt];
                if (vld_d1)   acc_mem[w_cnt_d1] <= new_acc;
            end

            // 输出 = new_acc / NR（常数乘 + 移位；钳一下防止暖机期越界）
            wire [ACC_W-1:0] ac = (new_acc > ACC_MAX) ? ACC_MAX[ACC_W-1:0] : new_acc;
            wire [31:0]      pr = ac * DIV_C16;

            assign blur_comb[l*8 +: 8] = (pr >> DIV_S) & 32'h000000FF;
        end
    endgenerate

    //----------------------------------------------------------------------
    // ★ O_blur 必须按**有效拍**寄存，不能按 vld_d1（"有效拍之后那一拍"）
    //
    //   blur_comb 在"有效拍 k 之后一拍"才算好，一直保持到下一个有效拍之后一拍。
    //   原版在 vld_d1 时寄存：
    //      · 无空档（每拍都有字）时，O_blur@拍m = 拍(m-2) 的模糊 —— 正确
    //      · 有 478 拍空档时，O_blur 早就被推成了拍(m-1) 的模糊 —— **横向早一个字**
    //   症状：只在行尾这种横向突变处出错，粗尺度整体错 4 个像素。
    //
    //   改成"在有效拍上寄存 blur_comb"：此时 blur_comb 已经稳定地等于
    //   上一拍的模糊，于是 O_blur@拍m = 拍(m-2)，空档/无空档都对。
    //----------------------------------------------------------------------
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)   O_blur <= {GRAY_W{1'b0}};
        else if (I_tvalid) O_blur <= blur_comb;
    end

    //-------------------------------------------------------------------------
    // 同步信号：延迟 LAT 拍
    //-------------------------------------------------------------------------
    reg vld_d2, lst_d1, lst_d2, usr_d1, usr_d2;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            vld_d2 <= 1'b0; lst_d1 <= 1'b0; lst_d2 <= 1'b0;
            usr_d1 <= 1'b0; usr_d2 <= 1'b0;
        end else begin
            vld_d2 <= vld_d1;
            lst_d1 <= I_tlast;  lst_d2 <= lst_d1;
            usr_d1 <= I_tuser;  usr_d2 <= usr_d1;
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_tvalid <= 1'b0; O_tlast <= 1'b0; O_tuser <= 1'b0;
        end else begin
            O_tvalid <= vld_d2;
            O_tlast  <= lst_d2;
            O_tuser  <= usr_d2;
        end
    end

    wire unused_ok = I_tlast;

endmodule
