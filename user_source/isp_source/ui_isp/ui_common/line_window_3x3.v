//=============================================================================
//  line_window_3x3.v --- 4 像素/拍 的 3×3 滑动窗口（行缓存 + 水平邻居）
//
//  来源：FPGA_lvjing/rtl/line_window_3x3.v
//  相对原文件的改动（都为了综合，算法一字未动）：
//    1) `line_cnt % 3` 换成 mod-3 寄存器 lslot
//       —— 原写法对 32bit 计数器取模，综合会生出一个除法器（几百 LUT），
//          而 line_cnt 本来就每行 +1，用 mod-3 计数器等价且免费。
//    2) 三块行缓存加 `/* fehdl force_ram=1 */`，保证 TD 推成 ERAM
//    3) 删掉 O_word_idx（纯调试，32 个触发器）
//
//  ---------------------------------------------------------------------------
//  【为什么是"字"级窗口而不是"像素"级】
//  一拍 4 个像素（96bit）。输出字 W 的 4 个像素，横向邻域最多跨到
//  列 4W-1 .. 4W+4，也就是字 W-1、W、W+1 三个字。所以窗口是"3 行 × 3 字"。
//
//  【时序】总延迟 = 480（晚一行）+ 3（行内晚 3 字）= 483，
//          再加最后 `O_tvalid <= vld_d` 寄存的一拍 = **端到端 484 拍**。
//          ⚠️ pencil_front 的 D 公式里用的就是 484（WPL+4），别按 483 算。
//
//  【字格式】缓存与输出的字是 128bit：{gray[31:0], rgb[95:0]}
//      像素 p 的 RGB  在 word[p*24 +: 24]
//      像素 p 的灰度 在 word[96 + p*8 +: 8]
//
//  【边界钳位】字 W=0 时"字 W-1"整个落在图像外，"字 W=479"时"字 W+1"同理。
//      处理方式是整字复制本行最左/最右像素。
//      ⚠️ 各行要取各行的像素（上行 O_01、中行 O_11、下行 O_21），
//         而且要用本拍的值（top_q1/mid_q1/lv3），不能用 O_11 寄存器 ——
//         那是上一拍的字，会整体错一个字。
//
//  【★ 按"有效拍"推进，不是按拍】
//  真机的 ISP 流每行 480 个有效字之后有 ~478 拍空档（传感器 bursts 完一行就歇着）。
//  原版所有流水线寄存器都是**每拍**移位的，空档期会移位垃圾、并且 O_tvalid 会冻在
//  高电平（下游收到成倍假数据）。这里把**所有**流水线寄存器改成只在 I_tvalid 时推进，
//  于是"拍"一律理解为"有效拍"。真机连续视频流下与原来完全等价。
//
//  【前提】输入每行恰好 WORDS_PER_LINE 个**有效**字。
//=============================================================================
`timescale 1ns/1ps

module line_window_3x3 #(
    parameter integer PIX_PER_WORD    = 4,
    parameter integer RGB_W           = PIX_PER_WORD * 24,   // 96  —— 输入接口宽度
    parameter integer GRAY_W          = PIX_PER_WORD * 8,    // 32
    parameter integer WORD_W          = RGB_W + GRAY_W,      // 128 —— 缓存/输出字宽
    parameter integer WORDS_PER_LINE  = 480
)(
    input  wire                  I_clk,
    input  wire                  I_rst_n,

    // 输入仍是 96bit 纯 RGB
    input  wire [RGB_W-1:0]      I_tdata,
    input  wire                  I_tvalid,
    input  wire                  I_tlast,
    input  wire                  I_tuser,

    // 3 行 × 3 字窗口：
    //   O_0x = 行 R-1（上）  O_1x = 行 R（中，产出像素在这行）
    //   O_2x = 行 R+1（下）  Ox0 = 字 W-1   Ox1 = 字 W   Ox2 = 字 W+1
    output reg  [WORD_W-1:0]     O_00, O_01, O_02,
    output reg  [WORD_W-1:0]     O_10, O_11, O_12,
    output reg  [WORD_W-1:0]     O_20, O_21, O_22,

    // 与窗口对齐的同步（输出字 W 的同步 = 输入字 W 的同步）
    output wire                  O_tvalid,
    output wire                  O_tlast,
    output wire                  O_tuser
);

    localparam integer LAT_INT = WORDS_PER_LINE + 3;   // 483

    //--------------------------------------------------------------------------
    // 灰度：在进缓存之前算（一拍 4 次，而不是窗口输出上算 4×9 次）
    //--------------------------------------------------------------------------
    wire [GRAY_W-1:0] in_gray;

    rgb2gray #(
        .PIX_PER_WORD (PIX_PER_WORD)
    ) u_rgb2gray (
        .I_rgb  (I_tdata ),
        .O_gray (in_gray)
    );

    // 拼成缓存用的 128bit 字：{gray[31:0], rgb[95:0]}
    wire [WORD_W-1:0] w_in = {in_gray, I_tdata};

    //--------------------------------------------------------------------------
    // 输入计数器
    //   lslot = 行号 mod 3（替代原版的 line_cnt % 3，见文件头改动 1）
    //--------------------------------------------------------------------------
    reg [15:0] w_cnt;        // 行内字第几（0..479）
    reg [1:0]  lslot;        // 行号 mod 3

    always @(posedge I_clk) begin
        if (!I_rst_n) begin
            w_cnt <= 16'd0;
            lslot <= 2'd0;
        end else if (I_tvalid) begin
            if (I_tuser) begin
                w_cnt <= 16'd1;
                lslot <= 2'd0;
            end else if (w_cnt == WORDS_PER_LINE - 1) begin
                w_cnt <= 16'd0;
                lslot <= (lslot == 2'd2) ? 2'd0 : (lslot + 2'd1);
            end else begin
                w_cnt <= w_cnt + 16'd1;
            end
        end
    end

    //--------------------------------------------------------------------------
    // 3 块行缓存轮转
    //   行 r 写 RAM[r mod 3]
    //   行 R   = lslot-1  -> mid_slot
    //   行 R-1 = lslot-2  -> top_slot
    //--------------------------------------------------------------------------
    (* ram_style = "block" *) reg [WORD_W-1:0] ram0 [0:WORDS_PER_LINE-1];  /* fehdl force_ram=1, ram_style="bram" */
    (* ram_style = "block" *) reg [WORD_W-1:0] ram1 [0:WORDS_PER_LINE-1];  /* fehdl force_ram=1, ram_style="bram" */
    (* ram_style = "block" *) reg [WORD_W-1:0] ram2 [0:WORDS_PER_LINE-1];  /* fehdl force_ram=1, ram_style="bram" */

    wire [1:0] wr_slot  = lslot;
    wire [1:0] mid_slot = (lslot == 2'd0) ? 2'd2 : (lslot - 2'd1);   // (lslot-1) mod 3
    wire [1:0] top_slot = (lslot == 2'd2) ? 2'd0 : (lslot + 2'd1);   // (lslot+1) mod 3

    // 读地址 = w_cnt（推导见原文件：行缓存读总共晚两级）
    wire [15:0] rd_addr = w_cnt;

    always @(posedge I_clk) begin
        if (I_tvalid) begin
            case (wr_slot)
                2'd0:    ram0[w_cnt] <= w_in;
                2'd1:    ram1[w_cnt] <= w_in;
                default: ram2[w_cnt] <= w_in;
            endcase
        end
    end

    reg [WORD_W-1:0] mid_raw, top_raw;

    // 简单双口读：本拍给地址、下一拍出数据
    always @(posedge I_clk) begin
        if (I_tvalid) begin
            case (mid_slot)
                2'd0:    mid_raw <= ram0[rd_addr];
                2'd1:    mid_raw <= ram1[rd_addr];
                default: mid_raw <= ram2[rd_addr];
            endcase
            case (top_slot)
                2'd0:    top_raw <= ram0[rd_addr];
                2'd1:    top_raw <= ram1[rd_addr];
                default: top_raw <= ram2[rd_addr];
            endcase
        end
    end

    // 3 级流水：q0 = W+1, q1 = W, q2 = W-1
    reg [WORD_W-1:0] mid_q0, mid_q1, mid_q2;
    reg [WORD_W-1:0] top_q0, top_q1, top_q2;

    always @(posedge I_clk) begin
        if (I_tvalid) begin
            mid_q2 <= mid_q1;  mid_q1 <= mid_q0;  mid_q0 <= mid_raw;
            top_q2 <= top_q1;  top_q1 <= top_q0;  top_q0 <= top_raw;
        end
    end

    //--------------------------------------------------------------------------
    // 活行（= 行 R+1）：输入流打 2/3/4 拍 -> W+1 / W / W-1
    //--------------------------------------------------------------------------
    reg [WORD_W-1:0] lv1, lv2, lv3, lv4;

    always @(posedge I_clk) begin
        if (I_tvalid) begin
            lv1 <= w_in;
            lv2 <= lv1;
            lv3 <= lv2;
            lv4 <= lv3;
        end
    end

    //--------------------------------------------------------------------------
    // 输出 + 边界钳位
    //--------------------------------------------------------------------------
    wire [WORD_W-1:0] cur_top = top_q1;   // 本拍输出的"字 W"，上行 R-1
    wire [WORD_W-1:0] cur_mid = mid_q1;   // 本拍输出的"字 W"，中行 R
    wire [WORD_W-1:0] cur_bot = lv3;      // 本拍输出的"字 W"，下行 R+1

    wire [23:0] bl_t = cur_top[23:0],   br_t = cur_top[95:72];
    wire [23:0] bl_m = cur_mid[23:0],   br_m = cur_mid[95:72];
    wire [23:0] bl_b = cur_bot[23:0],   br_b = cur_bot[95:72];
    wire [7:0]  gl_t = cur_top[103:96], gr_t = cur_top[127:120];
    wire [7:0]  gl_m = cur_mid[103:96], gr_m = cur_mid[127:120];
    wire [7:0]  gl_b = cur_bot[103:96], gr_b = cur_bot[127:120];

    wire at_w0    = (w_cnt == 16'd3);        // 输出字 W=0 的那一拍（W = w_cnt-3）
    wire at_wlast = (w_cnt == 16'd2);        // 输出字 W=479 的那一拍（w_cnt 已回绕）

    // 左补边：整字 4 个像素都填"本行字 W 的最左像素"
    wire [WORD_W-1:0] padL_top = {{PIX_PER_WORD{gl_t}}, {PIX_PER_WORD{bl_t}}};
    wire [WORD_W-1:0] padL_mid = {{PIX_PER_WORD{gl_m}}, {PIX_PER_WORD{bl_m}}};
    wire [WORD_W-1:0] padL_bot = {{PIX_PER_WORD{gl_b}}, {PIX_PER_WORD{bl_b}}};

    // 右补边：整字 4 个像素都填"本行字 W 的最右像素"
    wire [WORD_W-1:0] padR_top = {{PIX_PER_WORD{gr_t}}, {PIX_PER_WORD{br_t}}};
    wire [WORD_W-1:0] padR_mid = {{PIX_PER_WORD{gr_m}}, {PIX_PER_WORD{br_m}}};
    wire [WORD_W-1:0] padR_bot = {{PIX_PER_WORD{gr_b}}, {PIX_PER_WORD{br_b}}};

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_00 <= {WORD_W{1'b0}}; O_01 <= {WORD_W{1'b0}}; O_02 <= {WORD_W{1'b0}};
            O_10 <= {WORD_W{1'b0}}; O_11 <= {WORD_W{1'b0}}; O_12 <= {WORD_W{1'b0}};
            O_20 <= {WORD_W{1'b0}}; O_21 <= {WORD_W{1'b0}}; O_22 <= {WORD_W{1'b0}};
        end else if (I_tvalid) begin
            O_01 <= top_q1;    // 上行 字W
            O_11 <= mid_q1;    // 中行 字W
            O_21 <= lv3;       // 下行 字W

            // 字 W-1：W=0 时最左像素没有左邻居 -> 复制本行字 W 的最左像素
            if (at_w0) begin
                O_00 <= padL_top;
                O_10 <= padL_mid;
                O_20 <= padL_bot;
            end else begin
                O_00 <= top_q2;
                O_10 <= mid_q2;
                O_20 <= lv4;
            end

            // 字 W+1：W=479 时最右像素没有右邻居 -> 复制本行字 W 的最右像素
            if (at_wlast) begin
                O_02 <= padR_top;
                O_12 <= padR_mid;
                O_22 <= padR_bot;
            end else begin
                O_02 <= top_q0;
                O_12 <= mid_q0;
                O_22 <= lv2;
            end
        end
    end

    //--------------------------------------------------------------------------
    // 同步：把输入的 tvalid/tuser/tlast 原样延迟 LAT_INT 拍，
    //       输出再寄存一级 -> 端到端 483 + 1 = 484 拍
    //--------------------------------------------------------------------------
    //----------------------------------------------------------------------
    //  ★ 同步信号按"有效拍"延迟 484 拍
    //
    //    数据 O_11 在有效拍 m 上是输入有效拍 (m-484) 的那一行，
    //    所以 lst/usr 用一条 484 深、只在 I_tvalid 时移位的移位寄存器，
    //    输出取最高位并**组合地与上 I_tvalid**：
    //      * 空档期 I_tvalid=0 -> 三个输出全 0（不会把上一次的 valid 冻住）
    //      * 移位只在有效拍发生，所以内容在空档期保持、不会移进垃圾
    //----------------------------------------------------------------------
    reg [LAT_INT:0] usr_sr, lst_sr;      // 484 深
    reg [9:0]       beat_cnt;            // 已收到的有效拍数（饱和）

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            usr_sr   <= {(LAT_INT+1){1'b0}};
            lst_sr   <= {(LAT_INT+1){1'b0}};
            beat_cnt <= 10'd0;
        end else if (I_tvalid) begin
            usr_sr <= {usr_sr[LAT_INT-1:0], I_tuser};
            lst_sr <= {lst_sr[LAT_INT-1:0], I_tlast };
            if (beat_cnt != LAT_INT + 1)
                beat_cnt <= beat_cnt + 10'd1;
        end
    end

    wire warm_ok = (beat_cnt >= LAT_INT + 1);   // 已吃满 484 个有效拍

    assign O_tvalid = I_tvalid & warm_ok;
    assign O_tuser  = I_tvalid & usr_sr[LAT_INT];
    assign O_tlast  = I_tvalid & lst_sr[LAT_INT];

    // I_tlast 用上了（见上），这里不再需要哑消费
    wire unused_ok = 1'b0;

endmodule
