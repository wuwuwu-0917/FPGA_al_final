//=============================================================================
//  comic_v11.v --- 漫画 v11（FPGA_lvjing 里用户指定存档的那一版）的硬件移植
//
//  来源：FPGA_lvjing/rtl/comic_v11.v
//
//  ---------------------------------------------------------------------------
//  【相对原文件的改动 —— 都为了综合/为了真机的空档流，算法一字未动】
//
//   1) 色调曲线 256×8 从"可写 RAM"改成 **常数 case 函数**
//      2026-10-18 晚：p3 把中段抬亮 30%（tone[128]=166），上板偏过曝，
//      换成 **半强度版** = round((i + p3[i])/2)：tone[64]=74 tone[128]=147 tone[192]=210
//      仍是单调、0→0、255→255；要退回 p3 就把表换回去
//      原版靠 TB 在开跑前写 256 次；真机没有 CPU 能写它。
//      要换曲线就改 `tone_of()` 那张表（p3 / p5 两套成品在
//      FPGA_lvjing/out/comic/v11_rtl/tone_lut_p*.mem）。
//
//   2) `row_cnt % NR` 换成 mod-NR 寄存器 —— 原写法会生成一个 32bit 除法器
//
//   3) `acc[4][480]` 三维数组改成 generate 里每 lane 一块一维数组 ——
//      TD 不支持三维非压缩数组；而且一维独立数组才能各自推成双口 RAM
//
//   4) `$clog2` 换成自带 clog2_f
//
//   5) 所有大数组加 `/* fehdl force_ram=1 */`，保证 TD 推成 ERAM
//
//   6) **整条链改成按"有效拍"推进**（真机 ISP 流每行 480 个有效字之后
//      有约 478 拍空档）：
//        · lstA/lstB 只在 I_tvalid 时移位
//        · 输出级 8a/8b/8c 的寄存器只在对应 valid 时推进
//      原版这些寄存器为每拍移位，空档期会出现错位（与 blur_gray.O_blur 同类问题）
//
//   7) 删掉 I_word_idx / O_word_idx / O_dbg_* —— 只用于调试；
//      I_word_idx 在原版里**不参与任何数据通路**，只用来算 O_word_idx
//
//  ---------------------------------------------------------------------------
//  【流水】
//    O_11 的灰度 g
//      → 纵向 13 抽头（13 行缓存 + 滑动和）→ 横向 13 抽头（居中，中心 = 字 W-2）
//                                                     → C   （内容行 r-6，字 W-2）
//      → 平涂 gS = C + ((g−C)·clamp((|g−C|−10)·48))>>8
//      → 色调 gT = TONE[gS]
//      → 清晰度 gT2 = gT + ((gT − TONE[Hbox33(blurw)])·3)>>4
//      → 色度 col = gT2 + (mean3(RGB−g)·88)>>6
//      → 墨   ink = clamp((C − g − 12)·14)
//      → out  = col + ((34 − col)·ink)>>8
//
//  【对齐】N6 = 6*WPL（纵向 13 抽头中心在 r-6）
//          N_G  = N6 + 4     灰度 / blurw
//          N_CH = N6 + 8     色度
//
//  【色度的 3×3 要从窗口"拼"出来】line_window_3x3 给的是**整字**的
//    W-1 / W / W+1，不是逐像素移位，所以这里自己把 lane 接过去。
//
//  【BRAM】13 行灰度缓存 195K + 滑动和 27K + g 延时 101K + blurw 延时 101K
//          + 色度延时 286K ≈ 710 Kbit
//
//  【验证】tb_comic_filter：与原工程 RTL 逐像素对拍 + 空档无关性自检
//=============================================================================
`timescale 1ns/1ps

module comic_v11 #(
    parameter integer PIX_PER_WORD    = 4,
    parameter integer RGB_W           = PIX_PER_WORD * 24,   // 96
    parameter integer GRAY_W          = PIX_PER_WORD * 8,    // 32
    parameter integer WORD_W          = RGB_W + GRAY_W,      // 128
    parameter integer WORDS_PER_LINE  = 480,
    parameter integer LINES_PER_FRAME = 1080,
    parameter integer SHARP_EN        = 1,        // 0 = 关掉"33×33 带通清晰度"
    parameter integer INK_TH          = 12,       // 墨线阈值：C-g 超过它才出墨
    parameter integer INK_GAIN        = 14,       // 墨线增益：每超 1 级灰度加多少
    // 平涂强度：gS = C + (g−C)·w/256，w = clamp((|g−C|−FLAT_TH)·FLAT_GAIN)
    //   |g−C| <= FLAT_TH 的像素完全平涂成局部均值 C
    //   原工程 10/48 —— 实测有 71.2% 的像素被完全平涂，画面偏"糊"
    //   2026-10-18 晚改成 4/32：只有真正平坦的像素才抹平，细节保留
    parameter integer FLAT_TH         = 4,        // 平涂死区
    parameter integer FLAT_GAIN       = 32,       // 平涂过渡斜率
    // 清晰度死区：|gT−Lb| <= SHARP_TH 的像素**不锐化**
    //   锐化会把传感器噪声和残留的高频抖动一起放大（实测帧间差 +6%），
    //   加一个死区让只有真正的结构边缘才被锐化
    parameter integer SHARP_TH        = 20        // 0 = 无死区（原工程行为）
)(
    input  wire                  I_clk,
    input  wire                  I_rst_n,

    // 3×3 字级窗口（{gray[31:0], rgb[95:0]}）
    input  wire [WORD_W-1:0]     O_00, O_01, O_02,
                                 O_10, O_11, O_12,
                                 O_20, O_21, O_22,

    // 与窗口同拍、已对齐的纵向模糊（RADIUS=16）
    input  wire [GRAY_W-1:0]     I_blurw,

    input  wire                  I_tvalid,
    input  wire                  I_tlast,
    input  wire                  I_tuser,

    // ★ 运行时参数（2026-10-21 移植：由 FPGA_al 的 OSD 菜单驱动）
    //   不接时把它们接成上面 parameter 的默认值即可。
    input  wire [7:0]            I_ink_th,
    input  wire [7:0]            I_ink_gain,
    input  wire [7:0]            I_flat_th,
    input  wire [7:0]            I_flat_gain,
    input  wire [7:0]            I_sharp_th,

    output reg  [RGB_W-1:0]      O_rgb,
    output reg                   O_tvalid,
    output reg                   O_tlast,
    output reg                   O_tuser
);

    // 运行时参数显式转成有符号线网，位宽与原来的 localparam 一致
    wire signed [9:0]  INK_TH_S   = $signed({2'b00, I_ink_th});
    wire signed [19:0] INK_GAIN_S = $signed({12'd0, I_ink_gain});
    wire [8:0]         FLAT_TH_U   = {1'b0, I_flat_th};
    wire signed [15:0] FLAT_TH_S   = $signed({8'd0, I_flat_th});
    wire signed [15:0] FLAT_GAIN_S = $signed({8'd0, I_flat_gain});
    wire [8:0]         SHARP_TH_U  = {1'b0, I_sharp_th};


    //--------------------------------------------------------------------------
    // 对齐常数
    //--------------------------------------------------------------------------
    localparam integer N6    = 6 * WORDS_PER_LINE;
    localparam integer N_G   = N6 + 4;
    localparam integer N_CH  = N6 + 8;

    //--------------------------------------------------------------------------
    // 本地 clog2
    //--------------------------------------------------------------------------
    function integer clog2_f;
        input integer v;
        integer i;
        begin
            clog2_f = 0;
            for (i = v - 1; i > 0; i = i >> 1)
                clog2_f = clog2_f + 1;
        end
    endfunction

    //--------------------------------------------------------------------------
    // 色调曲线（固化 FPGA_lvjing/out/comic/v11_rtl/tone_lut_p3.mem）
    //   单调、0->0、255->255、中段抬亮。要换曲线改这张表即可。
    //--------------------------------------------------------------------------
    function [7:0] tone_of;
        input [7:0] a;
        begin
            case (a)
            8'd0: tone_of = 8'd0;
            8'd1: tone_of = 8'd0;
            8'd2: tone_of = 8'd1;
            8'd3: tone_of = 8'd2;
            8'd4: tone_of = 8'd2;
            8'd5: tone_of = 8'd2;
            8'd6: tone_of = 8'd4;
            8'd7: tone_of = 8'd4;
            8'd8: tone_of = 8'd4;
            8'd9: tone_of = 8'd6;
            8'd10: tone_of = 8'd6;
            8'd11: tone_of = 8'd6;
            8'd12: tone_of = 8'd8;
            8'd13: tone_of = 8'd8;
            8'd14: tone_of = 8'd9;
            8'd15: tone_of = 8'd10;
            8'd16: tone_of = 8'd11;
            8'd17: tone_of = 8'd12;
            8'd18: tone_of = 8'd12;
            8'd19: tone_of = 8'd14;
            8'd20: tone_of = 8'd15;
            8'd21: tone_of = 8'd16;
            8'd22: tone_of = 8'd18;
            8'd23: tone_of = 8'd20;
            8'd24: tone_of = 8'd22;
            8'd25: tone_of = 8'd24;
            8'd26: tone_of = 8'd25;
            8'd27: tone_of = 8'd27;
            8'd28: tone_of = 8'd28;
            8'd29: tone_of = 8'd30;
            8'd30: tone_of = 8'd32;
            8'd31: tone_of = 8'd34;
            8'd32: tone_of = 8'd36;
            8'd33: tone_of = 8'd38;
            8'd34: tone_of = 8'd38;
            8'd35: tone_of = 8'd40;
            8'd36: tone_of = 8'd42;
            8'd37: tone_of = 8'd43;
            8'd38: tone_of = 8'd44;
            8'd39: tone_of = 8'd46;
            8'd40: tone_of = 8'd47;
            8'd41: tone_of = 8'd48;
            8'd42: tone_of = 8'd50;
            8'd43: tone_of = 8'd51;
            8'd44: tone_of = 8'd52;
            8'd45: tone_of = 8'd54;
            8'd46: tone_of = 8'd55;
            8'd47: tone_of = 8'd56;
            8'd48: tone_of = 8'd57;
            8'd49: tone_of = 8'd58;
            8'd50: tone_of = 8'd59;
            8'd51: tone_of = 8'd60;
            8'd52: tone_of = 8'd61;
            8'd53: tone_of = 8'd62;
            8'd54: tone_of = 8'd63;
            8'd55: tone_of = 8'd64;
            8'd56: tone_of = 8'd65;
            8'd57: tone_of = 8'd66;
            8'd58: tone_of = 8'd68;
            8'd59: tone_of = 8'd68;
            8'd60: tone_of = 8'd70;
            8'd61: tone_of = 8'd70;
            8'd62: tone_of = 8'd72;
            8'd63: tone_of = 8'd73;
            8'd64: tone_of = 8'd74;
            8'd65: tone_of = 8'd75;
            8'd66: tone_of = 8'd76;
            8'd67: tone_of = 8'd77;
            8'd68: tone_of = 8'd78;
            8'd69: tone_of = 8'd79;
            8'd70: tone_of = 8'd80;
            8'd71: tone_of = 8'd82;
            8'd72: tone_of = 8'd82;
            8'd73: tone_of = 8'd84;
            8'd74: tone_of = 8'd85;
            8'd75: tone_of = 8'd86;
            8'd76: tone_of = 8'd87;
            8'd77: tone_of = 8'd88;
            8'd78: tone_of = 8'd89;
            8'd79: tone_of = 8'd90;
            8'd80: tone_of = 8'd92;
            8'd81: tone_of = 8'd93;
            8'd82: tone_of = 8'd94;
            8'd83: tone_of = 8'd96;
            8'd84: tone_of = 8'd97;
            8'd85: tone_of = 8'd98;
            8'd86: tone_of = 8'd99;
            8'd87: tone_of = 8'd100;
            8'd88: tone_of = 8'd102;
            8'd89: tone_of = 8'd103;
            8'd90: tone_of = 8'd104;
            8'd91: tone_of = 8'd105;
            8'd92: tone_of = 8'd106;
            8'd93: tone_of = 8'd108;
            8'd94: tone_of = 8'd109;
            8'd95: tone_of = 8'd110;
            8'd96: tone_of = 8'd112;
            8'd97: tone_of = 8'd114;
            8'd98: tone_of = 8'd114;
            8'd99: tone_of = 8'd116;
            8'd100: tone_of = 8'd118;
            8'd101: tone_of = 8'd118;
            8'd102: tone_of = 8'd120;
            8'd103: tone_of = 8'd122;
            8'd104: tone_of = 8'd123;
            8'd105: tone_of = 8'd124;
            8'd106: tone_of = 8'd126;
            8'd107: tone_of = 8'd128;
            8'd108: tone_of = 8'd128;
            8'd109: tone_of = 8'd130;
            8'd110: tone_of = 8'd131;
            8'd111: tone_of = 8'd132;
            8'd112: tone_of = 8'd134;
            8'd113: tone_of = 8'd134;
            8'd114: tone_of = 8'd136;
            8'd115: tone_of = 8'd136;
            8'd116: tone_of = 8'd138;
            8'd117: tone_of = 8'd138;
            8'd118: tone_of = 8'd140;
            8'd119: tone_of = 8'd140;
            8'd120: tone_of = 8'd141;
            8'd121: tone_of = 8'd142;
            8'd122: tone_of = 8'd142;
            8'd123: tone_of = 8'd143;
            8'd124: tone_of = 8'd144;
            8'd125: tone_of = 8'd144;
            8'd126: tone_of = 8'd146;
            8'd127: tone_of = 8'd146;
            8'd128: tone_of = 8'd147;
            8'd129: tone_of = 8'd148;
            8'd130: tone_of = 8'd149;
            8'd131: tone_of = 8'd150;
            8'd132: tone_of = 8'd151;
            8'd133: tone_of = 8'd152;
            8'd134: tone_of = 8'd154;
            8'd135: tone_of = 8'd154;
            8'd136: tone_of = 8'd156;
            8'd137: tone_of = 8'd156;
            8'd138: tone_of = 8'd158;
            8'd139: tone_of = 8'd159;
            8'd140: tone_of = 8'd160;
            8'd141: tone_of = 8'd161;
            8'd142: tone_of = 8'd162;
            8'd143: tone_of = 8'd163;
            8'd144: tone_of = 8'd164;
            8'd145: tone_of = 8'd166;
            8'd146: tone_of = 8'd166;
            8'd147: tone_of = 8'd168;
            8'd148: tone_of = 8'd168;
            8'd149: tone_of = 8'd169;
            8'd150: tone_of = 8'd170;
            8'd151: tone_of = 8'd171;
            8'd152: tone_of = 8'd172;
            8'd153: tone_of = 8'd173;
            8'd154: tone_of = 8'd174;
            8'd155: tone_of = 8'd174;
            8'd156: tone_of = 8'd176;
            8'd157: tone_of = 8'd176;
            8'd158: tone_of = 8'd178;
            8'd159: tone_of = 8'd178;
            8'd160: tone_of = 8'd179;
            8'd161: tone_of = 8'd180;
            8'd162: tone_of = 8'd181;
            8'd163: tone_of = 8'd182;
            8'd164: tone_of = 8'd183;
            8'd165: tone_of = 8'd184;
            8'd166: tone_of = 8'd185;
            8'd167: tone_of = 8'd186;
            8'd168: tone_of = 8'd187;
            8'd169: tone_of = 8'd188;
            8'd170: tone_of = 8'd190;
            8'd171: tone_of = 8'd190;
            8'd172: tone_of = 8'd192;
            8'd173: tone_of = 8'd192;
            8'd174: tone_of = 8'd194;
            8'd175: tone_of = 8'd194;
            8'd176: tone_of = 8'd196;
            8'd177: tone_of = 8'd197;
            8'd178: tone_of = 8'd198;
            8'd179: tone_of = 8'd199;
            8'd180: tone_of = 8'd200;
            8'd181: tone_of = 8'd200;
            8'd182: tone_of = 8'd202;
            8'd183: tone_of = 8'd202;
            8'd184: tone_of = 8'd203;
            8'd185: tone_of = 8'd204;
            8'd186: tone_of = 8'd205;
            8'd187: tone_of = 8'd206;
            8'd188: tone_of = 8'd206;
            8'd189: tone_of = 8'd208;
            8'd190: tone_of = 8'd208;
            8'd191: tone_of = 8'd209;
            8'd192: tone_of = 8'd210;
            8'd193: tone_of = 8'd211;
            8'd194: tone_of = 8'd212;
            8'd195: tone_of = 8'd213;
            8'd196: tone_of = 8'd214;
            8'd197: tone_of = 8'd214;
            8'd198: tone_of = 8'd216;
            8'd199: tone_of = 8'd216;
            8'd200: tone_of = 8'd218;
            8'd201: tone_of = 8'd218;
            8'd202: tone_of = 8'd219;
            8'd203: tone_of = 8'd220;
            8'd204: tone_of = 8'd221;
            8'd205: tone_of = 8'd222;
            8'd206: tone_of = 8'd222;
            8'd207: tone_of = 8'd223;
            8'd208: tone_of = 8'd224;
            8'd209: tone_of = 8'd224;
            8'd210: tone_of = 8'd226;
            8'd211: tone_of = 8'd226;
            8'd212: tone_of = 8'd227;
            8'd213: tone_of = 8'd228;
            8'd214: tone_of = 8'd228;
            8'd215: tone_of = 8'd230;
            8'd216: tone_of = 8'd230;
            8'd217: tone_of = 8'd232;
            8'd218: tone_of = 8'd232;
            8'd219: tone_of = 8'd234;
            8'd220: tone_of = 8'd234;
            8'd221: tone_of = 8'd235;
            8'd222: tone_of = 8'd236;
            8'd223: tone_of = 8'd236;
            8'd224: tone_of = 8'd238;
            8'd225: tone_of = 8'd238;
            8'd226: tone_of = 8'd238;
            8'd227: tone_of = 8'd240;
            8'd228: tone_of = 8'd240;
            8'd229: tone_of = 8'd240;
            8'd230: tone_of = 8'd242;
            8'd231: tone_of = 8'd242;
            8'd232: tone_of = 8'd242;
            8'd233: tone_of = 8'd244;
            8'd234: tone_of = 8'd244;
            8'd235: tone_of = 8'd244;
            8'd236: tone_of = 8'd245;
            8'd237: tone_of = 8'd246;
            8'd238: tone_of = 8'd246;
            8'd239: tone_of = 8'd246;
            8'd240: tone_of = 8'd247;
            8'd241: tone_of = 8'd248;
            8'd242: tone_of = 8'd248;
            8'd243: tone_of = 8'd249;
            8'd244: tone_of = 8'd250;
            8'd245: tone_of = 8'd250;
            8'd246: tone_of = 8'd250;
            8'd247: tone_of = 8'd251;
            8'd248: tone_of = 8'd252;
            8'd249: tone_of = 8'd252;
            8'd250: tone_of = 8'd252;
            8'd251: tone_of = 8'd253;
            8'd252: tone_of = 8'd254;
            8'd253: tone_of = 8'd254;
            8'd254: tone_of = 8'd254;
            8'd255: tone_of = 8'd255;
            default: tone_of = a;
            endcase
        end
    endfunction

    //--------------------------------------------------------------------------
    // clamp 函数（有符号 -> 0..255）
    //--------------------------------------------------------------------------
    function [7:0] clam8;
        input signed [19:0] v;
        begin
            if (v < 0)        clam8 = 8'd0;
            else if (v > 255) clam8 = 8'd255;
            else              clam8 = v[7:0];
        end
    endfunction

    //--------------------------------------------------------------------------
    // 输入灰度（窗口中心字的灰度字段）
    //--------------------------------------------------------------------------
    wire [GRAY_W-1:0] g_in = O_11[RGB_W +: GRAY_W];

    //==========================================================================
    // 1) 纵向 13 抽头盒式（半径 6）—— 滑动和
    //==========================================================================
    localparam integer NR      = 13;
    localparam integer ACC_W   = 14;                // 255*13 = 3315
    localparam integer ACC_MAX = 255 * NR;
    localparam integer DIV_C   = 2521;              // (x*2521)>>15 == x/13, x<=3315
    localparam integer DIV_S   = 15;
    localparam integer AW      = clog2_f(NR * WORDS_PER_LINE);
    localparam [AW-1:0] WPL_L  = WORDS_PER_LINE;
    localparam [15:0]   DIV_C16 = DIV_C;

    reg [15:0] w_cnt;
    reg [4:0]  rslot;          // row_cnt mod 13（替代原来的 row_cnt % NR）
    reg [31:0] row_cnt;

    always @(posedge I_clk) begin
        if (!I_rst_n) begin
            w_cnt   <= 16'd0;
            rslot   <= 5'd0;
            row_cnt <= 32'd0;
        end else if (I_tvalid) begin
            // ★ 2026-11-01 用 I_tlast 逐行重同步。
            //   行缓存地址是 vaddr = rslot*480 + w_cnt，相位原来只靠
            //   「每行恰好 480 个字」这个假设撑着。只要某时刻少一个/多一个字，
            //   窗口就永久错开且不会自愈——而原图/浮雕/铅笔三路是逐点/逐行
            //   运算、不看绝对位置，所以只有漫画会坏，表现是每帧都在漂。
            //   （tb_comic_sync 复现：少一个字之后每一行都停在 w_cnt=478。）
            //
            //   tlast 是每行最后一个字，正常帧里 w_cnt 本来就等于 479，
            //   两个条件同时成立，所以正常路径行为完全不变；
            //   只有计数真出错时，这一拍才把它拉回来。
            //
            //   注意**不要在 I_tuser 上复位 rslot**：1080 = 13*83 + 1，
            //   帧首的 rslot 是 (帧号 mod 13)，不是 0，复位会破坏 13 行窗口。
            if (I_tlast || (w_cnt == WORDS_PER_LINE - 1)) begin
                w_cnt   <= 16'd0;
                rslot   <= (rslot == NR-1) ? 5'd0 : (rslot + 5'd1);
                row_cnt <= row_cnt + 32'd1;
            end else begin
                w_cnt <= w_cnt + 16'd1;
            end
        end
    end

    // 行缓存：13 行 × WPL × GRAY_W，同一地址先读后写
    (* ram_style = "block" *) reg [GRAY_W-1:0] vmem [0:NR*WORDS_PER_LINE-1];  /* fehdl force_ram=1, ram_style="bram" */

    wire [AW-1:0] vaddr = rslot * WPL_L + w_cnt;

    reg [GRAY_W-1:0] old_row;

    always @(posedge I_clk) begin
        if (I_tvalid) begin
            old_row     <= vmem[vaddr];
            vmem[vaddr] <= g_in;
        end
    end

    // 打一拍，和新行 / 旧行对齐
    reg [GRAY_W-1:0] g_d1;
    reg              vld_d1;
    reg [15:0]       w_cnt_d1;
    reg [31:0]       row_cnt_d1;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            g_d1       <= {GRAY_W{1'b0}};
            vld_d1     <= 1'b0;
            w_cnt_d1   <= 16'd0;
            row_cnt_d1 <= 32'd0;
        end else begin
            g_d1       <= g_in;
            vld_d1     <= I_tvalid;
            w_cnt_d1   <= w_cnt;
            row_cnt_d1 <= row_cnt;
        end
    end

    wire acc_init = (row_cnt_d1 == 32'd0);
    wire warm_ok  = (row_cnt_d1 >= NR);

    wire [GRAY_W-1:0] Cv;        // 纵向 13 抽头均值（内容行 r-6）

    genvar l;
    generate
        for (l = 0; l < PIX_PER_WORD; l = l + 1) begin : g_acc

            // 每 lane 一块独立的累加器 RAM（不用三维数组）
            (* ram_style = "block" *) reg [ACC_W-1:0] acc_mem [0:WORDS_PER_LINE-1];  /* fehdl force_ram=1, ram_style="bram" */
            reg [ACC_W-1:0] acc_q;

            wire [ACC_W-1:0] new_px = {6'd0, g_d1[l*8 +: 8]};
            wire [ACC_W-1:0] old_px = (warm_ok && !acc_init)
                                    ? {6'd0, old_row[l*8 +: 8]} : {ACC_W{1'b0}};

            wire [ACC_W-1:0] new_acc = acc_init ? new_px : (acc_q + new_px - old_px);

            // 同一 always 内先读后写 -> 推成简单双口 RAM
            always @(posedge I_clk) begin
                if (I_tvalid) acc_q             <= acc_mem[w_cnt];
                if (vld_d1)   acc_mem[w_cnt_d1] <= new_acc;
            end

            wire [ACC_W-1:0] ac  = (new_acc > ACC_MAX) ? ACC_MAX[ACC_W-1:0] : new_acc;
            wire [31:0]      pr  = ac * DIV_C16;

            reg [7:0] cr;
            always @(posedge I_clk or negedge I_rst_n) begin
                if (!I_rst_n)   cr <= 8'd0;
                else if (vld_d1) cr <= (pr >> DIV_S) & 32'h000000FF;
            end
            assign Cv[l*8 +: 8] = cr;
        end
    endgenerate

    //==========================================================================
    // 1b) 横向 13 抽头（居中，输出中心 = 字 W-2）
    //==========================================================================
    reg [15:0] wlC;
    reg        lstA, lstB;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin lstA <= 1'b0; lstB <= 1'b0; end
        else if (I_tvalid) begin            // ★ 只在有效拍移位（真机流有空档）
            lstA <= I_tlast;
            lstB <= lstA;
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) wlC <= 16'd0;
        else if (vld_d1) wlC <= lstB ? 16'd0 : (wlC + 16'd1);
    end

    reg [GRAY_W-1:0] w0C_r;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) w0C_r <= {GRAY_W{1'b0}};
        else if (vld_d1 && (wlC == 16'd0)) w0C_r <= Cv;
    end
    wire [GRAY_W-1:0] w0C = (wlC == 16'd0) ? Cv : w0C_r;

    reg [GRAY_W-1:0] C1, C2, C3, C4;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin C1 <= 0; C2 <= 0; C3 <= 0; C4 <= 0; end
        else if (vld_d1) begin C1 <= Cv; C2 <= C1; C3 <= C2; C4 <= C3; end
    end

    // 补边判据要用**输出（中心）字**的位置：中心在 W-2，所以行内位置是 wlC-2。
    // wlC<2 时中心还在上一行 -> 窗口里的字全是上一行的真实数据，一个都不能补。
    wire [15:0] wlC_e = (wlC < 16'd2) ? 16'hFFFF : wlC;
    wire [GRAY_W-1:0] swC1 = (wlC_e >= 16'd1) ? C1 : w0C;
    wire [GRAY_W-1:0] swC2 = (wlC_e >= 16'd2) ? C2 : w0C;
    wire [GRAY_W-1:0] swC3 = (wlC_e >= 16'd3) ? C3 : w0C;
    wire [GRAY_W-1:0] swC4 = (wlC_e >= 16'd4) ? C4 : w0C;

    // 20 lane：L[0..3]=W-4 ... L[16..19]=W；输出中心字 = W-2（lane 8..11）
    wire [5*GRAY_W-1:0] LC = {Cv, swC1, swC2, swC3, swC4};

    wire [GRAY_W-1:0] Cw;        // 13×13 盒式均值（内容行 r-6，字 W-2）
    generate
        for (l = 0; l < PIX_PER_WORD; l = l + 1) begin : g_h13
            wire [11:0] s13 = LC[(2+l)*8 +: 8] + LC[(3+l)*8 +: 8] + LC[(4+l)*8 +: 8]
                            + LC[(5+l)*8 +: 8] + LC[(6+l)*8 +: 8] + LC[(7+l)*8 +: 8]
                            + LC[(8+l)*8 +: 8] + LC[(9+l)*8 +: 8] + LC[(10+l)*8 +: 8]
                            + LC[(11+l)*8 +: 8] + LC[(12+l)*8 +: 8] + LC[(13+l)*8 +: 8]
                            + LC[(14+l)*8 +: 8];
            wire [26:0] pr13 = s13 * 27'd2521;
            assign Cw[l*8 +: 8] = pr13[26:15];
        end
    endgenerate

    //==========================================================================
    // 2) 两条内容延时（对齐到 C）
    //==========================================================================
    wire [GRAY_W-1:0] g_d;
    wire              g_dv, g_dl, g_du;

    word_delay #(.W(GRAY_W), .N(N_G)) u_dly_g (
        .I_clk(I_clk), .I_rst_n(I_rst_n),
        .I_data(g_in), .I_valid(I_tvalid), .I_last(I_tlast), .I_user(I_tuser),
        .O_data(g_d), .O_valid(g_dv), .O_last(g_dl), .O_user(g_du)
    );

    wire [GRAY_W-1:0] bw_d;
    wire              bw_dv, bw_dl, bw_du;

    word_delay #(.W(GRAY_W), .N(N_G)) u_dly_bw (
        .I_clk(I_clk), .I_rst_n(I_rst_n),
        .I_data(I_blurw), .I_valid(I_tvalid), .I_last(I_tlast), .I_user(I_tuser),
        .O_data(bw_d), .O_valid(bw_dv), .O_last(bw_dl), .O_user(bw_du)
    );

    //==========================================================================
    // 3) 平涂 + 色调曲线
    //==========================================================================
    //  ★ 2026-10-18：13 抽头 + 平涂 + 色调 ROM 原来是**一条**组合链（20 级 / 17.9ns），
    //     100MHz 域（10ns）收不住。这里把 Cw 和 g_d 各打一拍，**只给平涂/色调这条
    //     路**用；墨和色度（C_1..C_4 / G_1..G_4）继续用原来的 Cw / g_d，完全不动。
    //     相应地把 gT 的延时从 4 拍改成 3 拍，这样 gT2 的输出相位与原来一模一样。
    reg  [GRAY_W-1:0] Cw_s, g_d_s;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            Cw_s  <= {GRAY_W{1'b0}};
            g_d_s <= {GRAY_W{1'b0}};
        end else if (vld_d1) begin
            Cw_s  <= Cw;
            g_d_s <= g_d;
        end
    end

    wire [GRAY_W-1:0] gS;

    generate
        for (l = 0; l < PIX_PER_WORD; l = l + 1) begin : g_flat
            wire [7:0] gc = Cw_s [l*8 +: 8];
            wire [7:0] gg = g_d_s[l*8 +: 8];

            wire signed [8:0] d  = $signed({1'b0, gg}) - $signed({1'b0, gc});
            wire        [8:0] ad = d[8] ? (~d + 9'd1) : d;          // |d|
            wire signed [15:0] wv = (ad > FLAT_TH_U)
                                  ? (($signed({7'd0, ad}) - FLAT_TH_S) * FLAT_GAIN_S)
                                  : 16'sd0;
            wire        [7:0]  w  = (wv > 16'sd255) ? 8'd255 : wv[7:0];

            wire signed [18:0] dw = d * $signed({2'b00, w});         // d*w
            wire signed [18:0] gv = $signed({11'd0, gc}) + (dw >>> 8);

            assign gS[l*8 +: 8] = clam8({{1{gv[18]}}, gv});          // 19 -> 20 带符号
        end
    endgenerate

    //  ★ 2026-10-18 晚：色调 ROM 那级也是长路径（乘完直接进 256 项查表 mux，
    //     实测 -1.035ns）。这里把 ROM 输出打一拍，gT 链相应从 3 拍减到 2 拍，
    //     gT2 的输出相位保持不变（对拍 0 差异）。
    wire [GRAY_W-1:0] gT_w;
    generate
        for (l = 0; l < PIX_PER_WORD; l = l + 1) begin : g_lut1
            assign gT_w[l*8 +: 8] = tone_of(gS[l*8 +: 8]);
        end
    endgenerate

    reg  [GRAY_W-1:0] gT;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)   gT <= {GRAY_W{1'b0}};
        else if (vld_d1) gT <= gT_w;
    end

    //==========================================================================
    // 4) 横向 33 抽头（因果）：把 blurw 变成 box2d(g,16,16)
    //    9 个字 = 36 lane；lane q 的窗口 = L[q .. q+32]
    //==========================================================================
    reg [15:0] wl33;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) wl33 <= 16'd0;
        else if (bw_dv) wl33 <= bw_dl ? 16'd0 : (wl33 + 16'd1);
    end

    reg [GRAY_W-1:0] w0w33_r;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) w0w33_r <= {GRAY_W{1'b0}};
        else if (bw_dv && (wl33 == 16'd0)) w0w33_r <= bw_d;
    end
    wire [GRAY_W-1:0] w0w33 = (wl33 == 16'd0) ? bw_d : w0w33_r;

    reg [GRAY_W-1:0] s1, s2, s3, s4, s5, s6, s7, s8;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            s1 <= 0; s2 <= 0; s3 <= 0; s4 <= 0;
            s5 <= 0; s6 <= 0; s7 <= 0; s8 <= 0;
        end else if (bw_dv) begin
            s1 <= bw_d; s2 <= s1; s3 <= s2; s4 <= s3;
            s5 <= s4;   s6 <= s5; s7 <= s6; s8 <= s7;
        end
    end

    // 中心字 = W-4，wl33<4 时中心还在上一行 -> 不补边
    wire [15:0] wl33_e = (wl33 < 16'd4) ? 16'hFFFF : wl33;
    wire [GRAY_W-1:0] sw1 = (wl33_e >= 16'd1) ? s1 : w0w33;
    wire [GRAY_W-1:0] sw2 = (wl33_e >= 16'd2) ? s2 : w0w33;
    wire [GRAY_W-1:0] sw3 = (wl33_e >= 16'd3) ? s3 : w0w33;
    wire [GRAY_W-1:0] sw4 = (wl33_e >= 16'd4) ? s4 : w0w33;
    wire [GRAY_W-1:0] sw5 = (wl33_e >= 16'd5) ? s5 : w0w33;
    wire [GRAY_W-1:0] sw6 = (wl33_e >= 16'd6) ? s6 : w0w33;
    wire [GRAY_W-1:0] sw7 = (wl33_e >= 16'd7) ? s7 : w0w33;
    wire [GRAY_W-1:0] sw8 = (wl33_e >= 16'd8) ? s8 : w0w33;

    // 36 lane：低端 = 8 个字之前，高端 = 当前字
    wire [9*GRAY_W-1:0] LL = {bw_d, sw1, sw2, sw3, sw4, sw5, sw6, sw7, sw8};

    wire [13:0] t1_00 = {6'd0, LL[0*8 +: 8]} + {6'd0, LL[1*8 +: 8]};
    wire [13:0] t1_01 = {6'd0, LL[2*8 +: 8]} + {6'd0, LL[3*8 +: 8]};
    wire [13:0] t1_02 = {6'd0, LL[4*8 +: 8]} + {6'd0, LL[5*8 +: 8]};
    wire [13:0] t1_03 = {6'd0, LL[6*8 +: 8]} + {6'd0, LL[7*8 +: 8]};
    wire [13:0] t1_04 = {6'd0, LL[8*8 +: 8]} + {6'd0, LL[9*8 +: 8]};
    wire [13:0] t1_05 = {6'd0, LL[10*8 +: 8]} + {6'd0, LL[11*8 +: 8]};
    wire [13:0] t1_06 = {6'd0, LL[12*8 +: 8]} + {6'd0, LL[13*8 +: 8]};
    wire [13:0] t1_07 = {6'd0, LL[14*8 +: 8]} + {6'd0, LL[15*8 +: 8]};
    wire [13:0] t1_08 = {6'd0, LL[16*8 +: 8]} + {6'd0, LL[17*8 +: 8]};
    wire [13:0] t1_09 = {6'd0, LL[18*8 +: 8]} + {6'd0, LL[19*8 +: 8]};
    wire [13:0] t1_10 = {6'd0, LL[20*8 +: 8]} + {6'd0, LL[21*8 +: 8]};
    wire [13:0] t1_11 = {6'd0, LL[22*8 +: 8]} + {6'd0, LL[23*8 +: 8]};
    wire [13:0] t1_12 = {6'd0, LL[24*8 +: 8]} + {6'd0, LL[25*8 +: 8]};
    wire [13:0] t1_13 = {6'd0, LL[26*8 +: 8]} + {6'd0, LL[27*8 +: 8]};
    wire [13:0] t1_14 = {6'd0, LL[28*8 +: 8]} + {6'd0, LL[29*8 +: 8]};
    wire [13:0] t1_15 = {6'd0, LL[30*8 +: 8]} + {6'd0, LL[31*8 +: 8]};
    wire [13:0] t1_16 = {6'd0, LL[32*8 +: 8]};
    wire [13:0] t2_00 = t1_00 + t1_01;
    wire [13:0] t2_01 = t1_02 + t1_03;
    wire [13:0] t2_02 = t1_04 + t1_05;
    wire [13:0] t2_03 = t1_06 + t1_07;
    wire [13:0] t2_04 = t1_08 + t1_09;
    wire [13:0] t2_05 = t1_10 + t1_11;
    wire [13:0] t2_06 = t1_12 + t1_13;
    wire [13:0] t2_07 = t1_14 + t1_15;
    wire [13:0] t3_00 = t2_00 + t2_01;
    wire [13:0] t3_01 = t2_02 + t2_03;
    wire [13:0] t3_02 = t2_04 + t2_05;
    wire [13:0] t3_03 = t2_06 + t2_07;
    wire [13:0] t4_00 = t3_00 + t3_01;
    wire [13:0] t4_01 = t3_02 + t3_03;
    wire [13:0] t5_00 = t4_00 + t4_01;
    wire [13:0] t6_00 = t5_00 + t1_16;
    wire [13:0] hs0 = t6_00;

    wire [13:0] hs1 = hs0 + {6'd0, LL[33*8 +: 8]} - {6'd0, LL[ 0*8 +: 8]};
    wire [13:0] hs2 = hs1 + {6'd0, LL[34*8 +: 8]} - {6'd0, LL[ 1*8 +: 8]};
    wire [13:0] hs3 = hs2 + {6'd0, LL[35*8 +: 8]} - {6'd0, LL[ 2*8 +: 8]};

    // 除以 33：和 blur_gray 同一组常数（x<=8415 逐值验过）
    function [7:0] div33;
        input [13:0] x;
        reg [24:0] m;
        begin
            m = x * 25'd1986;
            div33 = m[23:16];
        end
    endfunction

    //  ★ 2026-10-18：33 抽头原来是**串联**的 14 级加法器（实测整条路径 -4.938ns / 17.9ns）。
    //     改成**平衡树**后只剩 6 级加法，这是一条时钟内能收敛的关键。
    wire [GRAY_W-1:0] Bh = {div33(hs3), div33(hs2), div33(hs1), div33(hs0)};

    //  ★★ 2026-10-18 晚：在 Bh 后面补**两级**流水，两个作用合一
    //
    //   (1) 时序：Bh 那条路（BRAM→33抽头→乘法→256项查表→算术）实测 17.9ns，
    //       83.33MHz(12ns) 收不住。加两级之后切成三段。
    //   (2) 对齐：Lb 必须和 gT_2 对应同一个像素。按"有效拍"数：
    //         g_d_s  = g_d + 1 拍
    //         gT     = g_d + 2 拍（经 g_d_s）
    //         gT_1   = g_d + 3 拍
    //         gT_2   = g_d + 4 拍
    //       而 blur_gray 的 O_blur 本身比 g_in 晚 2 拍（O_blur 在 I_tvalid 时
    //       寄存 blur_comb，而 blur_comb 已是上一拍的累加结果），
    //       所以 Lb 自然落在 g_d + 2 拍 —— 正好**差 2 拍**。
    //       补两级之后 Lb 落在 g_d + 4 拍，与 gT_2 对齐。
    //
    //  ⚠️ 之前试过改 gT 链（gT_4/5/6）都失败，是因为方向反了：
    //     该补的是 Bh 这一侧，不是 gT 那一侧。
    //
    //  ⚠️ 使能用 bw_dv（不是 vld_d1）。bw_dv 与 vld_d1 的"有效拍"集合相同、
    //     只是相位差；按有效拍推进的链只要级数相同就保持对齐。
    reg [GRAY_W-1:0] Bh_r1, Bh_r2;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            Bh_r1 <= {GRAY_W{1'b0}};
            Bh_r2 <= {GRAY_W{1'b0}};
        end else if (bw_dv) begin
            Bh_r1 <= Bh;
            Bh_r2 <= Bh_r1;
        end
    end

    // TONE[box2d(g,16,16)]
    wire [GRAY_W-1:0] Lb;
    generate
        for (l = 0; l < PIX_PER_WORD; l = l + 1) begin : g_lut2
            assign Lb[l*8 +: 8] = tone_of(Bh_r2[l*8 +: 8]);
        end
    endgenerate

    //==========================================================================
    // 5) 清晰度：gT2 = gT + (gT − Lb)·3/16，gT 要晚 4 个字对齐
    //==========================================================================
    reg [GRAY_W-1:0] gT_1, gT_2;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            gT_1 <= 0; gT_2 <= 0;
        end else if (vld_d1) begin
            gT_1 <= gT; gT_2 <= gT_1; 
        end
    end

    wire [GRAY_W-1:0] gT2;
    generate
        for (l = 0; l < PIX_PER_WORD; l = l + 1) begin : g_clar
            wire signed [8:0] H_raw = $signed({1'b0, gT_2[l*8 +: 8]})
                                    - $signed({1'b0, Lb  [l*8 +: 8]});
            // ★ 死区：小幅度的高频（噪声/抖动）不参与锐化
            wire        [8:0] Hab  = H_raw[8] ? (~H_raw + 9'd1) : H_raw;
            wire signed [8:0] H    = (Hab <= SHARP_TH_U) ? 9'sd0 : H_raw;
            wire signed [11:0] h3 = H * 12'sd3;
            wire signed [12:0] v  = $signed({4'd0, gT_2[l*8 +: 8]}) + (h3 >>> 4);
            assign gT2[l*8 +: 8] = (SHARP_EN == 0) ? gT_2[l*8 +: 8]
                                                   : clam8({{7{v[12]}}, v});
        end
    endgenerate

    //==========================================================================
    // 6) 色度：ch_i = (Σ9(RGB_i − g)·3641)>>15
    //==========================================================================
    wire [95:0] ch_now;

    // 每行"左邻 / 右邻"的 **lane 对齐** 字（lane q = 像素 x-1 / x+1）
    wire [95:0] upL = {O_01[71:48], O_01[47:24], O_01[23:0], O_00[95:72]};
    wire [95:0] upR = {O_02[23:0],  O_01[95:72], O_01[71:48], O_01[47:24]};
    wire [95:0] midL= {O_11[71:48], O_11[47:24], O_11[23:0], O_10[95:72]};
    wire [95:0] midR= {O_12[23:0],  O_11[95:72], O_11[71:48], O_11[47:24]};
    wire [95:0] dnL = {O_21[71:48], O_21[47:24], O_21[23:0], O_20[95:72]};
    wire [95:0] dnR = {O_22[23:0],  O_21[95:72], O_21[71:48], O_21[47:24]};

    wire [31:0] upLg = {O_01[RGB_W+16 +: 8], O_01[RGB_W+ 8 +: 8], O_01[RGB_W+0 +: 8], O_00[RGB_W+24 +: 8]};
    wire [31:0] upRg = {O_02[RGB_W+ 0 +: 8], O_01[RGB_W+24 +: 8], O_01[RGB_W+16 +: 8], O_01[RGB_W+8 +: 8]};
    wire [31:0] midLg= {O_11[RGB_W+16 +: 8], O_11[RGB_W+ 8 +: 8], O_11[RGB_W+0 +: 8], O_10[RGB_W+24 +: 8]};
    wire [31:0] midRg= {O_12[RGB_W+ 0 +: 8], O_11[RGB_W+24 +: 8], O_11[RGB_W+16 +: 8], O_11[RGB_W+8 +: 8]};
    wire [31:0] dnLg = {O_21[RGB_W+16 +: 8], O_21[RGB_W+ 8 +: 8], O_21[RGB_W+0 +: 8], O_20[RGB_W+24 +: 8]};
    wire [31:0] dnRg = {O_22[RGB_W+ 0 +: 8], O_21[RGB_W+24 +: 8], O_21[RGB_W+16 +: 8], O_21[RGB_W+8 +: 8]};

    function signed [8:0] d9;
        input [7:0] a;
        input [7:0] b;
        begin
            d9 = $signed({1'b0, a}) - $signed({1'b0, b});
        end
    endfunction

    genvar q;
    generate
        for (q = 0; q < PIX_PER_WORD; q = q + 1) begin : g_chr
            wire [7:0] r0 = upL [q*24+16 +: 8], g0 = upLg [q*8 +: 8];
            wire [7:0] r1 = O_01[q*24+16 +: 8], g1 = O_01[RGB_W+q*8 +: 8];
            wire [7:0] r2 = upR [q*24+16 +: 8], g2 = upRg [q*8 +: 8];
            wire [7:0] r3 = midL[q*24+16 +: 8], g3 = midLg[q*8 +: 8];
            wire [7:0] r4 = O_11[q*24+16 +: 8], g4 = O_11[RGB_W+q*8 +: 8];
            wire [7:0] r5 = midR[q*24+16 +: 8], g5 = midRg[q*8 +: 8];
            wire [7:0] r6 = dnL [q*24+16 +: 8], g6 = dnLg [q*8 +: 8];
            wire [7:0] r7 = O_21[q*24+16 +: 8], g7 = O_21[RGB_W+q*8 +: 8];
            wire [7:0] r8 = dnR [q*24+16 +: 8], g8 = dnRg [q*8 +: 8];

            wire [7:0] b0 = upL [q*24     +: 8], c0 = g0;
            wire [7:0] b1 = O_01[q*24     +: 8], c1 = g1;
            wire [7:0] b2 = upR [q*24     +: 8], c2 = g2;
            wire [7:0] b3 = midL[q*24     +: 8], c3 = g3;
            wire [7:0] b4 = O_11[q*24     +: 8], c4 = g4;
            wire [7:0] b5 = midR[q*24     +: 8], c5 = g5;
            wire [7:0] b6 = dnL [q*24     +: 8], c6 = g6;
            wire [7:0] b7 = O_21[q*24     +: 8], c7 = g7;
            wire [7:0] b8 = dnR [q*24     +: 8], c8 = g8;

            wire [7:0] h0 = upL [q*24+ 8 +: 8];
            wire [7:0] h1 = O_01[q*24+ 8 +: 8];
            wire [7:0] h2 = upR [q*24+ 8 +: 8];
            wire [7:0] h3 = midL[q*24+ 8 +: 8];
            wire [7:0] h4 = O_11[q*24+ 8 +: 8];
            wire [7:0] h5 = midR[q*24+ 8 +: 8];
            wire [7:0] h6 = dnL [q*24+ 8 +: 8];
            wire [7:0] h7 = O_21[q*24+ 8 +: 8];
            wire [7:0] h8 = dnR [q*24+ 8 +: 8];

            // (RGB − gray)，有符号；写成函数，避免 signed/unsigned 混算陷阱
            wire signed [12:0] sR = d9(r0,g0)+d9(r1,g1)+d9(r2,g2)+d9(r3,g3)+d9(r4,g4)
                                  + d9(r5,g5)+d9(r6,g6)+d9(r7,g7)+d9(r8,g8);
            wire signed [12:0] sG = d9(h0,c0)+d9(h1,c1)+d9(h2,c2)+d9(h3,c3)+d9(h4,c4)
                                  + d9(h5,c5)+d9(h6,c6)+d9(h7,c7)+d9(h8,c8);
            wire signed [12:0] sB = d9(b0,g0)+d9(b1,g1)+d9(b2,g2)+d9(b3,g3)+d9(b4,g4)
                                  + d9(b5,g5)+d9(b6,g6)+d9(b7,g7)+d9(b8,g8);

            wire signed [28:0] mR = sR * 29'sd3641;
            wire signed [28:0] mG = sG * 29'sd3641;
            wire signed [28:0] mB = sB * 29'sd3641;

            wire signed [13:0] cR = mR >>> 15;
            wire signed [13:0] cG = mG >>> 15;
            wire signed [13:0] cB = mB >>> 15;

            assign ch_now[q*24 + 16 +: 8] = (cR > 127) ? 8'sd127 : (cR < -128) ? -8'sd128 : cR[7:0];
            assign ch_now[q*24 +  8 +: 8] = (cG > 127) ? 8'sd127 : (cG < -128) ? -8'sd128 : cG[7:0];
            assign ch_now[q*24      +: 8] = (cB > 127) ? 8'sd127 : (cB < -128) ? -8'sd128 : cB[7:0];
        end
    endgenerate

    wire [95:0] ch_d;
    wire        ch_dv, ch_dl, ch_du;

    word_delay #(.W(96), .N(N_CH)) u_dly_ch (
        .I_clk(I_clk), .I_rst_n(I_rst_n),
        .I_data(ch_now), .I_valid(I_tvalid), .I_last(I_tlast), .I_user(I_tuser),
        .O_data(ch_d), .O_valid(ch_dv), .O_last(ch_dl), .O_user(ch_du)
    );

    //==========================================================================
    // 7) C / g / gT2 再延 4 个字（对齐 33 抽头的 16 像素）
    //==========================================================================
    reg [GRAY_W-1:0] C_1, C_2, C_3, C_4;
    reg [GRAY_W-1:0] G_1, G_2, G_3, G_4;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            C_1 <= 0; C_2 <= 0; C_3 <= 0; C_4 <= 0;
            G_1 <= 0; G_2 <= 0; G_3 <= 0; G_4 <= 0;
        end else if (vld_d1) begin
            C_1 <= Cw;   C_2 <= C_1; C_3 <= C_2; C_4 <= C_3;
            G_1 <= g_d;  G_2 <= G_1; G_3 <= G_2; G_4 <= G_3;
        end
    end

    //==========================================================================
    // 8) 墨 + 合成（3 级流水）
    //     ★ 全部按**有效拍**推进（ch_dv 是这条通路的 valid）
    //==========================================================================
    reg [GRAY_W-1:0] gT2_r;
    reg [95:0]       ch_r;
    reg [GRAY_W-1:0] C_r, G_r;
    reg              v1, l1, u1;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            gT2_r <= 0; ch_r <= 0; C_r <= 0; G_r <= 0;
            v1 <= 0; l1 <= 0; u1 <= 0;
        end else begin
            v1 <= ch_dv;  l1 <= ch_dl;  u1 <= ch_du;      // 脉冲按拍延迟
            if (ch_dv) begin
                gT2_r <= gT2; ch_r <= ch_d; C_r <= C_4; G_r <= G_4;
            end
        end
    end

    wire [GRAY_W-1:0] ink_w;
    wire [RGB_W-1:0]  col_w;

    generate
        for (l = 0; l < PIX_PER_WORD; l = l + 1) begin : g_mix
            wire signed [9:0] dv = $signed({2'd0, C_r[l*8 +: 8]})
                                 - $signed({2'd0, G_r[l*8 +: 8]}) - INK_TH_S;
            wire signed [19:0] iv = dv * INK_GAIN_S;
            wire [7:0] ink = (iv < 0) ? 8'd0 : (iv > 255) ? 8'd255 : iv[7:0];
            assign ink_w[l*8 +: 8] = ink;

            wire signed [7:0]  chR = ch_r[l*24 + 16 +: 8];
            wire signed [7:0]  chG = ch_r[l*24 +  8 +: 8];
            wire signed [7:0]  chB = ch_r[l*24      +: 8];
            wire signed [15:0] pR = chR * 16'sd88;
            wire signed [15:0] pG = chG * 16'sd88;
            wire signed [15:0] pB = chB * 16'sd88;
            wire signed [11:0] vR = $signed({4'd0, gT2_r[l*8 +: 8]}) + (pR >>> 6);
            wire signed [11:0] vG = $signed({4'd0, gT2_r[l*8 +: 8]}) + (pG >>> 6);
            wire signed [11:0] vB = $signed({4'd0, gT2_r[l*8 +: 8]}) + (pB >>> 6);
            wire [7:0] cR8 = (vR < 0) ? 8'd0 : (vR > 255) ? 8'd255 : vR[7:0];
            wire [7:0] cG8 = (vG < 0) ? 8'd0 : (vG > 255) ? 8'd255 : vG[7:0];
            wire [7:0] cB8 = (vB < 0) ? 8'd0 : (vB > 255) ? 8'd255 : vB[7:0];
            assign col_w[l*24 + 16 +: 8] = cR8;
            assign col_w[l*24 +  8 +: 8] = cG8;
            assign col_w[l*24      +: 8] = cB8;
        end
    endgenerate

    reg [GRAY_W-1:0] ink_r;
    reg [RGB_W-1:0]  col_r;
    reg              v2, l2, u2;

    //--------------------------------------------------------------------------
    // 墨线短线过滤（2026-10-24）
    //
    //  目标：去掉长度 < 3 像素的水平墨线段（噪声/细纹理的碎线），保留真轮廓。
    //
    //  ★ 不能用形态学腐蚀：漫画墨线本来就只有 1~2 像素宽，3x3 腐蚀会把**所有**
    //    1 像素宽的线整条吃掉，不管它多长。
    //    按长度过滤只能用 run-length 判据。
    //
    //  判据：对像素 c，左右各看 2 个像素，命中任一 3 连续窗口就保留：
    //      keep = (b[c-2]&b[c-1]&b[c]) | (b[c-1]&b[c]&b[c+1]) | (b[c]&b[c+1]&b[c+2])
    //      b[k] = (ink[k] != 0)
    //  这样长度 1~2 的段整体消失，长度>=3 的段**两端也保留**（不像腐蚀那样缩短）。
    //  保留时输出原来的 8bit ink 值而不是二值，所以描边强度低时墨线不会整体消失。
    //
    //  时序：ink_r 是上一个字(W-1)，ink_w 是当前字(W)，ik_hist 是上上个字(W-2)的
    //        末 2 个像素 —— 三者拼起来正好覆盖 ±2 窗口。而且 ink_r 本来就比输入
    //        晚 1 个字，col_r 和它同拍，所以**色度不需要额外延迟**，总延迟不变。
    //--------------------------------------------------------------------------
    reg [7:0] ik_hist0, ik_hist1;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            ink_r <= 0; col_r <= 0; v2 <= 0; l2 <= 0; u2 <= 0;
            ik_hist0 <= 0; ik_hist1 <= 0;
        end else begin
            v2 <= v1; l2 <= l1; u2 <= u1;
            if (v1) begin
                ink_r <= ink_w; col_r <= col_w;
                ik_hist0 <= ink_r[23:16];      // 前一字倒数第 2 像素
                ik_hist1 <= ink_r[31:24];      // 前一字最后 1 像素
            end
        end
    end

    // 8 像素窗口（低字节在前）：[0,1]=前字末 2 个，[2..5]=当前字，[6,7]=后字头 2 个
    wire [63:0] ik_win = {ink_w[15:8], ink_w[7:0],
                          ink_r[31:24], ink_r[23:16], ink_r[15:8], ink_r[7:0],
                          ik_hist1, ik_hist0};

    wire [GRAY_W-1:0] ink_f;          // 过滤后的墨线（和 col_r 同拍）

    generate
        for (l = 0; l < PIX_PER_WORD; l = l + 1) begin : g_ink_flt
            wire b0 = (ik_win[(l+0)*8 +: 8] != 8'd0);
            wire b1 = (ik_win[(l+1)*8 +: 8] != 8'd0);
            wire b2 = (ik_win[(l+2)*8 +: 8] != 8'd0);
            wire b3 = (ik_win[(l+3)*8 +: 8] != 8'd0);
            wire b4 = (ik_win[(l+4)*8 +: 8] != 8'd0);
            wire keep = (b0 & b1 & b2) | (b1 & b2 & b3) | (b2 & b3 & b4);
            assign ink_f[l*8 +: 8] = keep ? ink_r[l*8 +: 8] : 8'd0;
        end
    endgenerate

    wire [RGB_W-1:0] out_w;
    reg  [RGB_W-1:0] out_r;      // ★ 必须和原版一样多一级，否则数据比 valid 早一拍
    reg              v3, l3, u3;

    generate
        for (l = 0; l < PIX_PER_WORD; l = l + 1) begin : g_out
            wire [7:0] cR = col_r[l*24 + 16 +: 8];
            wire [7:0] cG = col_r[l*24 +  8 +: 8];
            wire [7:0] cB = col_r[l*24      +: 8];
            wire [7:0] ik = ink_f[l*8 +: 8];   // ★ 用过滤后的墨线（短线已去掉）

            wire signed [9:0] dR = 10'sd34 - $signed({2'd0, cR});
            wire signed [9:0] dG = 10'sd34 - $signed({2'd0, cG});
            wire signed [9:0] dB = 10'sd34 - $signed({2'd0, cB});

            wire signed [19:0] mR = dR * $signed({2'd0, ik});
            wire signed [19:0] mG = dG * $signed({2'd0, ik});
            wire signed [19:0] mB = dB * $signed({2'd0, ik});

            wire signed [12:0] oR = $signed({5'd0, cR}) + (mR >>> 8);
            wire signed [12:0] oG = $signed({5'd0, cG}) + (mG >>> 8);
            wire signed [12:0] oB = $signed({5'd0, cB}) + (mB >>> 8);

            wire [7:0] r8 = (oR < 0) ? 8'd0 : (oR > 255) ? 8'd255 : oR[7:0];
            wire [7:0] g8 = (oG < 0) ? 8'd0 : (oG > 255) ? 8'd255 : oG[7:0];
            wire [7:0] b8 = (oB < 0) ? 8'd0 : (oB > 255) ? 8'd255 : oB[7:0];

            assign out_w[l*24 +: 24] = {r8, g8, b8};
        end
    endgenerate

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) out_r <= {RGB_W{1'b0}};
        else if (v2)  out_r <= out_w;
    end

    // ★★ 2026-10-22 上板修正：数据 / valid / tlast 必须**同拍**
    //
    //   原来：O_rgb <= out_r（数据链 3 拍），O_tvalid/O_tlast <= v3/l3（4 拍），
    //   数据和行边界差 1 拍；实测 tlast 还比数据晚 4 个字，导致
    //   ui_fit_scaler 按错误的行边界切、混合时出现重影。
    //
    //   现在三者都直接跟随 ch_d 这一级（0 拍），总延迟 = N_CH = 2888，
    //   和 raw / pencil / emboss 三条支路**完全一致**。
    //   （out_w 本身用 C_4/G_4 做清晰度带通，那是窗口内的空间对齐，不是流水延迟。）
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_rgb <= {RGB_W{1'b0}}; O_tvalid <= 1'b0; O_tlast <= 1'b0; O_tuser <= 1'b0;
            v3 <= 0; l3 <= 0; u3 <= 0;
        end else begin
            v3 <= v2; l3 <= l2; u3 <= u2;
            O_rgb    <= out_r;
            O_tvalid <= v3;
            O_tlast  <= l3;
            O_tuser  <= u3;
        end
    end

    wire unused_ok = |{g_dv, g_dl, g_du, bw_du, LINES_PER_FRAME};

endmodule
