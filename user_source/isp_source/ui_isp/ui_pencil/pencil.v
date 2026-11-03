//=============================================================================
//  pencil.v --- 彩色铅笔（10 级流水）
//
//  来源：FPGA_lvjing/rtl/pencil.v 的 I_style=2 / I_color=3 路径
//
//  ★ 2026-10-18：黑白素描分支已按用户要求**整个删掉**。
//     注意这只影响源码可读性 —— COLOR 本来就是编译期常数，TD 早就把
//     COLOR==0 那条分支裁掉了，删掉它省下的只有 ~100 个触发器 + 几个 mux。
//     如果以后要黑白版：把输出改回 `{sk2, sk2, sk2}`（sk2 = clamp(255-dv_all)）。
//
//  ---------------------------------------------------------------------------
//  【相对原版 1：砍掉 style 0/1】
//      style 0/1（sobel 那两条）删掉，不再需要 sobel.v，省 ~1.5k LUT。
//      **算式与原版逐字一致**，只是不用的分支不生成。
//
//  【相对原版 2：纯组合 -> 10 级流水】
//      原版是**纯组合**，从 3×3 窗口寄存器一路算到输出，约 24 级 LUT。
//      本工程的 hs_rx_clk 域（74.25MHz / 13.47ns）本来只剩 +0.566ns 裕量，
//      一条 24 级的组合路径必然违例。所以切成 10 级，每级 <= 5 级 LUT：
//
//          S1  sum9 / tex6 / tone            （9 邻居求和、笔触疏密、色深）
//          S2  blur = (sum9×3641)>>15        （== sum9/9，逐值验过）
//                 texp = 87 + (tex6×166)>>8
//          S3  rc = RECIP[blur]              （256×16 常数表）
//                 w1 = (tone×texp)>>8
//          S4  gR = gray×rc ；w2 = 排线
//          S5  t_dg = clamp(gR>>8) ；w3 = clamp(w2×331>>8)
//          S6  dv = (255-t_dg)×K
//          S7  dv_all = dv+dvc ；sk2=BW ；dline=线稿墨量
//          S8  cfs = w3 + (dline×153)>>8
//          S9  qr = 255 - ((255-R)×cfs)>>8
//          S10 er = qr + 彩色线稿压深（0.344×局部均值 + 1.5×边缘色度）
//          S11 色彩强度（0..64% 灰度→彩铅，>64% 色度放大 + 压暗）
//
//      **只加寄存器、不改任何运算**，所以与原版逐像素等价
//      （实测 tb_pencil_filter 22.6 万像素逐点对拍 0 误差）。
//      流水级只在 I_valid=1 时推进（真机流有空档），控制信号在
//      ui_pencil_filter_96 里同期延 10 个**有效拍**。
//
//  ---------------------------------------------------------------------------
//  【衬底：两尺度颜色减淡（彩色铅笔的"线稿墨量"就是它）】
//      细尺度   blur  = 3×3 均值
//               t     = 钳位(gray×256/blur, 0, 255)        颜色减淡
//               dev   = 255 - t
//      粗尺度   gradc = 16 行半径的大模糊（blur_gray 给，已经除过 NR）
//               t_c   = 钳位(gray×256/gradc, 0, 255)
//               devc  = 255 - t_c
//      合成     dv_all = dev×K + max(devc−TH,0)×8          相加后一起钳位
//               out    = 钳位(255 - dv_all, 0, 255)
//
//  为什么必须放大起伏：只用减淡（K=1）时实测只剩 4% 的像素有内容（一张白纸），
//  因为减淡的响应天生单边且极弱。K=6 时 14.9%，毛发排线/胡须/眼眶都出来了。
//  为什么还要粗尺度：只有 3×3 时手办图只有 9% 有内容，五官/发丝/褶皱全在
//  5~100 像素尺度上、3×3 根本看不见；加粗尺度后 38%（用户验收那版）。
//
//  ---------------------------------------------------------------------------
//  【彩色铅笔 —— "像画出来的"那一版】
//
//      cf = S·tone·(0.35+0.65·tex)·hat/255  +  0.6·line
//           ↑亮处留纸白   ↑细纹理当笔触  ↑排线  ↑线稿（减淡 0.6）
//
//      tex   = |gray − 上一行同列 gray| × 32          笔触疏密（×32 是移位，免费）
//      tone  = 255 − gray                             暗部大 -> 亮处留白
//      texp  = 87 + (tex×166)>>8                      = 0.35 + 0.65·tex
//      w1    = (tone × texp) >> 8
//      w2    = ((行+列) mod 4 < 2) ? w1 : w1>>1       45° 排线，占空 50%
//      w3    = clamp((w2×331)>>8)                     ×S(1.3)
//      line  = clamp(dv_all, 0, 255)                  线稿墨量
//      cfs   = clamp(w3 + (line×153)>>8, 0, 255)      + 线稿×0.6
//      out   = 255 − ((255−rgb)×cfs>>8)
//                                                     上色（白纸保持 255，混合阶段减法叠加）
//
//  为什么不能只用"整体压一层色"：那是**均匀往白里混 = 一层白雾**
//  （实测纯白像素只有 0.3%）。真实彩铅的签名是
//  **亮处留纸白 + 暗处/有纹理处才堆色 + 看得见笔触**，也就是"方差大、均值小"，
//  所以涂色量 cf 必须**逐像素变化**。
//
//  【排线在硬件上几乎免费】一拍 4 个像素的列号都是 4 的倍数，
//  所以 (列+行) mod 4 == (行 mod 4) + q —— 一个计数器 + 两个比较器。
//  `I_hatchph` 由 pencil_front 按输出行号给出。
//
//  ⚠️ 已知问题（原工程记录在案）：排线是固定"周期 4、占空 50%"的 45° 图案，
//     1:1 全屏下能看出规则网点。成品是 1920×1080 实时 1:1，所以会原样带上屏，
//     只是比 5~10 倍缩略图淡约 3.4 倍。要改就调 pencil.v 的排线那一段。
//
//  ---------------------------------------------------------------------------
//  【定点误差】（都在 Python 端逐值比过，不是估计）
//      blur = (sum9 × 3641) >> 15        与 sum9/9 完全相等（0..2295 全部零偏差）
//      rc   = 65535 / max(blur,1)        与理想值 gray×256/blur 最大差 1/255
//      blur=0 时（9 个邻居全黑）取 recip[1]，不会除零
//
//  【位宽】sum9 <= 2295(12b) | sum9×3641 <= 8.36M(24b) | gray×rc <= 16.7M(24b)
//          w2×331 <= 84405(17b) | (255-rgb)×cfs <= 65025(16b) | qr×210 <= 53550
//=============================================================================
`timescale 1ns/1ps

module pencil #(
    parameter integer PIX_PER_WORD = 4,
    parameter integer RGB_W        = PIX_PER_WORD * 24,   // 96
    parameter integer GRAY_W       = PIX_PER_WORD * 8,    // 32
    parameter integer WORD_W       = RGB_W + GRAY_W,      // 128
    parameter integer DODGE        = 6,                   // 细尺度起伏放大倍数 K
    parameter integer COARSE       = 8,                   // 粗尺度起伏放大倍数 Kc
    parameter integer LAT          = 11                   // 流水级数（顶层据此延迟控制信号）
)(
    input  wire               I_clk,
    input  wire               I_rst_n,
    // 有效拍指示：**所有流水级只在 I_valid=1 时推进**
    //   （真机流每行 480 个有效字之后有空档，不能按拍推进）
    input  wire               I_valid,

    // 3×3 字级窗口（约定见 line_window_3x3）
    input  wire [WORD_W-1:0]  O_00, O_01, O_02, O_10, O_11, O_12, O_20, O_21, O_22,

    // 粗尺度：由 pencil_front 对齐送进来的 16 行模糊灰度（4 个像素）
    input  wire [GRAY_W-1:0]  I_blurw,

    // 彩色铅笔排线相位 = 输出行号 & 3（由 pencil_front 给出）
    input  wire [1:0]         I_hatchph,

    // ★ 运行时参数（2026-10-21 移植：由 FPGA_al 的 OSD 菜单驱动）
    //   不接时把它们接成上面 parameter 的默认值即可。
    input  wire [7:0]         I_dodge,      // 固定 6（线条强度滑块已删除）
    input  wire [7:0]         I_coarse,     // 0..254 边缘阈值（越大越只保留强边缘）
    // ★ 2026-10-24：色彩强度 9bit。
    //   0 = 纯黑白素描；256 = 原彩铅；>256 = 围绕亮度放大色度（饱和度）。
    //   UI p4 0..100 映射到 I_color 0..400；
    //   0..64 段经二次曲线慢起（10% 只有约 2% 色），64 以后进入饱和度档位。
    input  wire [8:0]         I_color,

    // 4 个像素的 RGB888 输出：像素 q 在 O_rgb[q*24 +: 24] = {R,G,B}
    output wire [RGB_W-1:0]   O_rgb
);

    wire [7:0] DODGE4  = I_dodge;
    wire [7:0] EDGE_TH = I_coarse;

    // ★ 2026-10-25：色彩强度重新映射（UI p4 0..100 -> 效果坐标 e）
    //   新 30% = 旧 60% 效果；新 60% ≈ 旧 100% 效果；新 60-100% 继续加深。
    //   UI p4:   0..30  -> e 0..60
    //           30..60  -> e 60..97
    //           60..100 -> e 97..157
    wire [7:0] p4_c = I_color >> 2;                 // 0..100
    wire [7:0] e_p4 = (p4_c <= 8'd30) ? (p4_c << 1) :
                      ((p4_c <= 8'd60) ? (8'd60 + (p4_c - 8'd30) + ((p4_c - 8'd30) >> 2)) :
                                         (8'd97 + (p4_c - 8'd60) + ((p4_c - 8'd60) >> 1)));

    // 色彩混合：e^2/16，e=60 约 225/256（旧 60%），e>=64 满色（旧 100%）
    wire [15:0] e_sq_c = e_p4 * e_p4;
    wire [9:0]  e_sh_c = e_sq_c >> 4;
    wire [8:0]  pen_color_k = (e_sh_c > 10'd256) ? 9'd256 : e_sh_c[8:0];

    // 饱和度档位（按 e）：1.25 / 1.5 / 1.75 / 2.0 / 2.5 / 3.0 / 3.5
    wire       pen_sat_en = (e_p4 > 8'd64);
    wire [2:0] pen_sat_mode = (e_p4 > 8'd145) ? 3'd6 :   // 3.5x
                              (e_p4 > 8'd130) ? 3'd5 :   // 3.0x
                              (e_p4 > 8'd110) ? 3'd4 :   // 2.5x
                              (e_p4 > 8'd94)  ? 3'd3 :   // 2.0x
                              (e_p4 > 8'd84)  ? 3'd2 :   // 1.75x
                              (e_p4 > 8'd74)  ? 3'd1 :   // 1.5x
                                                3'd0;    // 1.25x

    // 压暗档位（按 e）：6.25% / 12.5% / 18.75% / 25%
    //   新 30% (e=60) -> 6.25%，新 60% (e=97) -> 12.5%，
    //   新 60-100% 继续加深到 25%。
    wire [2:0] pen_dark_mode = (e_p4 > 8'd130) ? 3'd3 :
                               (e_p4 > 8'd100) ? 3'd2 :
                               (e_p4 > 8'd80)  ? 3'd1 :
                               (e_p4 > 8'd40)  ? 3'd0 :
                                                3'd4;   // 不压暗

    // ★ 2026-10-25：帧级控制量预寄存，避免 I_color→档位 进入像素路径
    reg [8:0] pen_color_k_r;
    reg       pen_sat_en_r;
    reg [2:0] pen_sat_mode_r, pen_dark_mode_r;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            pen_color_k_r  <= 9'd0;
            pen_sat_en_r   <= 1'b0;
            pen_sat_mode_r <= 3'd0;
            pen_dark_mode_r<= 3'd4;
        end else begin
            pen_color_k_r  <= pen_color_k;
            pen_sat_en_r   <= pen_sat_en;
            pen_sat_mode_r <= pen_sat_mode;
            pen_dark_mode_r<= pen_dark_mode;
        end
    end

    // 压暗函数：按档位做 6.25% / 12.5% / 18.75% / 25%
    function [7:0] darken8;
        input [7:0] v;
        input [2:0] mode;
        begin
            case(mode)
                3'd0: darken8 = v - (v >> 4);
                3'd1: darken8 = v - (v >> 3);
                3'd2: darken8 = v - (v >> 3) - (v >> 4);
                3'd3: darken8 = v - (v >> 2);
                default: darken8 = v;
            endcase
        end
    endfunction

    // 彩色线稿的目标色：
    //   line = 局部填充亮度(0.344×16行均值) + 1.5×(边缘像素色度)
    //   亮度取局部填充亮度，线不会因为边缘像素本身很暗而变成灰黑；
    //   色度取边缘像素并放大 1.5×，保留物体颜色。
    function [7:0] line_col_local;
        input [7:0] v;      // 边缘像素的彩铅色
        input [7:0] el;     // 该色的亮度
        input [7:0] gb;     // 局部填充亮度基准
        reg signed [10:0] ch;
        reg signed [10:0] s;
        begin
            ch = $signed({3'd0, v}) - $signed({3'd0, el});
            ch = ch + (ch >>> 1);           // 色度 ×1.5
            s = $signed({3'd0, gb}) + ch;
            line_col_local = (s < 11'sd0) ? 8'd0 :
                             (s > 11'sd255) ? 8'd255 : s[7:0];
        end
    endfunction

    // 线稿混入：只压深、不提亮（d = max(v−line, 0)），按 4 档比例混入
    //   lvl=0/1/2/3 -> 0、1/4、1/2、3/4；full=1 -> 完全取 line 色
    function [7:0] line_mix8;
        input [7:0] v;
        input [7:0] l;
        input [1:0] lvl;
        input       full;
        reg signed [10:0] d;
        reg signed [10:0] m;
        begin
            d = ($signed({3'd0, v}) > $signed({3'd0, l}))
                ? ($signed({3'd0, v}) - $signed({3'd0, l})) : 11'sd0;
            m = full        ? $signed({3'd0, l}) :
                (lvl == 2'd3) ? ($signed({3'd0, v}) - (d >>> 1) - (d >>> 2)) :
                (lvl == 2'd2) ? ($signed({3'd0, v}) - (d >>> 1)) :
                (lvl == 2'd1) ? ($signed({3'd0, v}) - (d >>> 2)) :
                                $signed({3'd0, v});
            line_mix8 = (m < 11'sd0) ? 8'd0 :
                        (m > 11'sd255) ? 8'd255 : m[7:0];
        end
    endfunction

    //--------------------------------------------------------------------------
    // 倒数表：recip[b] = 65535 / max(b,1)
    //   原版用 `wire [15:0] recip[0:255]` + generate assign，TD 对
    //   "wire 数组 + generate 赋值"支持不稳，这里改成 case 函数（等价 ROM）。
    //--------------------------------------------------------------------------
    function [15:0] recip_of;
        input [7:0] b;
        begin
            case (b)
            8'd0  : recip_of = 16'd65535;
            8'd1  : recip_of = 16'd65535;
            8'd2  : recip_of = 16'd32767;
            8'd3  : recip_of = 16'd21845;
            8'd4  : recip_of = 16'd16383;
            8'd5  : recip_of = 16'd13107;
            8'd6  : recip_of = 16'd10922;
            8'd7  : recip_of = 16'd9362;
            8'd8  : recip_of = 16'd8191;
            8'd9  : recip_of = 16'd7281;
            8'd10 : recip_of = 16'd6553;
            8'd11 : recip_of = 16'd5957;
            8'd12 : recip_of = 16'd5461;
            8'd13 : recip_of = 16'd5041;
            8'd14 : recip_of = 16'd4681;
            8'd15 : recip_of = 16'd4369;
            8'd16 : recip_of = 16'd4095;
            8'd17 : recip_of = 16'd3855;
            8'd18 : recip_of = 16'd3640;
            8'd19 : recip_of = 16'd3449;
            8'd20 : recip_of = 16'd3276;
            8'd21 : recip_of = 16'd3120;
            8'd22 : recip_of = 16'd2978;
            8'd23 : recip_of = 16'd2849;
            8'd24 : recip_of = 16'd2730;
            8'd25 : recip_of = 16'd2621;
            8'd26 : recip_of = 16'd2520;
            8'd27 : recip_of = 16'd2427;
            8'd28 : recip_of = 16'd2340;
            8'd29 : recip_of = 16'd2259;
            8'd30 : recip_of = 16'd2184;
            8'd31 : recip_of = 16'd2114;
            8'd32 : recip_of = 16'd2047;
            8'd33 : recip_of = 16'd1985;
            8'd34 : recip_of = 16'd1927;
            8'd35 : recip_of = 16'd1872;
            8'd36 : recip_of = 16'd1820;
            8'd37 : recip_of = 16'd1771;
            8'd38 : recip_of = 16'd1724;
            8'd39 : recip_of = 16'd1680;
            8'd40 : recip_of = 16'd1638;
            8'd41 : recip_of = 16'd1598;
            8'd42 : recip_of = 16'd1560;
            8'd43 : recip_of = 16'd1524;
            8'd44 : recip_of = 16'd1489;
            8'd45 : recip_of = 16'd1456;
            8'd46 : recip_of = 16'd1424;
            8'd47 : recip_of = 16'd1394;
            8'd48 : recip_of = 16'd1365;
            8'd49 : recip_of = 16'd1337;
            8'd50 : recip_of = 16'd1310;
            8'd51 : recip_of = 16'd1285;
            8'd52 : recip_of = 16'd1260;
            8'd53 : recip_of = 16'd1236;
            8'd54 : recip_of = 16'd1213;
            8'd55 : recip_of = 16'd1191;
            8'd56 : recip_of = 16'd1170;
            8'd57 : recip_of = 16'd1149;
            8'd58 : recip_of = 16'd1129;
            8'd59 : recip_of = 16'd1110;
            8'd60 : recip_of = 16'd1092;
            8'd61 : recip_of = 16'd1074;
            8'd62 : recip_of = 16'd1057;
            8'd63 : recip_of = 16'd1040;
            8'd64 : recip_of = 16'd1023;
            8'd65 : recip_of = 16'd1008;
            8'd66 : recip_of = 16'd992;
            8'd67 : recip_of = 16'd978;
            8'd68 : recip_of = 16'd963;
            8'd69 : recip_of = 16'd949;
            8'd70 : recip_of = 16'd936;
            8'd71 : recip_of = 16'd923;
            8'd72 : recip_of = 16'd910;
            8'd73 : recip_of = 16'd897;
            8'd74 : recip_of = 16'd885;
            8'd75 : recip_of = 16'd873;
            8'd76 : recip_of = 16'd862;
            8'd77 : recip_of = 16'd851;
            8'd78 : recip_of = 16'd840;
            8'd79 : recip_of = 16'd829;
            8'd80 : recip_of = 16'd819;
            8'd81 : recip_of = 16'd809;
            8'd82 : recip_of = 16'd799;
            8'd83 : recip_of = 16'd789;
            8'd84 : recip_of = 16'd780;
            8'd85 : recip_of = 16'd771;
            8'd86 : recip_of = 16'd762;
            8'd87 : recip_of = 16'd753;
            8'd88 : recip_of = 16'd744;
            8'd89 : recip_of = 16'd736;
            8'd90 : recip_of = 16'd728;
            8'd91 : recip_of = 16'd720;
            8'd92 : recip_of = 16'd712;
            8'd93 : recip_of = 16'd704;
            8'd94 : recip_of = 16'd697;
            8'd95 : recip_of = 16'd689;
            8'd96 : recip_of = 16'd682;
            8'd97 : recip_of = 16'd675;
            8'd98 : recip_of = 16'd668;
            8'd99 : recip_of = 16'd661;
            8'd100: recip_of = 16'd655;
            8'd101: recip_of = 16'd648;
            8'd102: recip_of = 16'd642;
            8'd103: recip_of = 16'd636;
            8'd104: recip_of = 16'd630;
            8'd105: recip_of = 16'd624;
            8'd106: recip_of = 16'd618;
            8'd107: recip_of = 16'd612;
            8'd108: recip_of = 16'd606;
            8'd109: recip_of = 16'd601;
            8'd110: recip_of = 16'd595;
            8'd111: recip_of = 16'd590;
            8'd112: recip_of = 16'd585;
            8'd113: recip_of = 16'd579;
            8'd114: recip_of = 16'd574;
            8'd115: recip_of = 16'd569;
            8'd116: recip_of = 16'd564;
            8'd117: recip_of = 16'd560;
            8'd118: recip_of = 16'd555;
            8'd119: recip_of = 16'd550;
            8'd120: recip_of = 16'd546;
            8'd121: recip_of = 16'd541;
            8'd122: recip_of = 16'd537;
            8'd123: recip_of = 16'd532;
            8'd124: recip_of = 16'd528;
            8'd125: recip_of = 16'd524;
            8'd126: recip_of = 16'd520;
            8'd127: recip_of = 16'd516;
            8'd128: recip_of = 16'd511;
            8'd129: recip_of = 16'd508;
            8'd130: recip_of = 16'd504;
            8'd131: recip_of = 16'd500;
            8'd132: recip_of = 16'd496;
            8'd133: recip_of = 16'd492;
            8'd134: recip_of = 16'd489;
            8'd135: recip_of = 16'd485;
            8'd136: recip_of = 16'd481;
            8'd137: recip_of = 16'd478;
            8'd138: recip_of = 16'd474;
            8'd139: recip_of = 16'd471;
            8'd140: recip_of = 16'd468;
            8'd141: recip_of = 16'd464;
            8'd142: recip_of = 16'd461;
            8'd143: recip_of = 16'd458;
            8'd144: recip_of = 16'd455;
            8'd145: recip_of = 16'd451;
            8'd146: recip_of = 16'd448;
            8'd147: recip_of = 16'd445;
            8'd148: recip_of = 16'd442;
            8'd149: recip_of = 16'd439;
            8'd150: recip_of = 16'd436;
            8'd151: recip_of = 16'd434;
            8'd152: recip_of = 16'd431;
            8'd153: recip_of = 16'd428;
            8'd154: recip_of = 16'd425;
            8'd155: recip_of = 16'd422;
            8'd156: recip_of = 16'd420;
            8'd157: recip_of = 16'd417;
            8'd158: recip_of = 16'd414;
            8'd159: recip_of = 16'd412;
            8'd160: recip_of = 16'd409;
            8'd161: recip_of = 16'd407;
            8'd162: recip_of = 16'd404;
            8'd163: recip_of = 16'd402;
            8'd164: recip_of = 16'd399;
            8'd165: recip_of = 16'd397;
            8'd166: recip_of = 16'd394;
            8'd167: recip_of = 16'd392;
            8'd168: recip_of = 16'd390;
            8'd169: recip_of = 16'd387;
            8'd170: recip_of = 16'd385;
            8'd171: recip_of = 16'd383;
            8'd172: recip_of = 16'd381;
            8'd173: recip_of = 16'd378;
            8'd174: recip_of = 16'd376;
            8'd175: recip_of = 16'd374;
            8'd176: recip_of = 16'd372;
            8'd177: recip_of = 16'd370;
            8'd178: recip_of = 16'd368;
            8'd179: recip_of = 16'd366;
            8'd180: recip_of = 16'd364;
            8'd181: recip_of = 16'd362;
            8'd182: recip_of = 16'd360;
            8'd183: recip_of = 16'd358;
            8'd184: recip_of = 16'd356;
            8'd185: recip_of = 16'd354;
            8'd186: recip_of = 16'd352;
            8'd187: recip_of = 16'd350;
            8'd188: recip_of = 16'd348;
            8'd189: recip_of = 16'd346;
            8'd190: recip_of = 16'd344;
            8'd191: recip_of = 16'd343;
            8'd192: recip_of = 16'd341;
            8'd193: recip_of = 16'd339;
            8'd194: recip_of = 16'd337;
            8'd195: recip_of = 16'd336;
            8'd196: recip_of = 16'd334;
            8'd197: recip_of = 16'd332;
            8'd198: recip_of = 16'd330;
            8'd199: recip_of = 16'd329;
            8'd200: recip_of = 16'd327;
            8'd201: recip_of = 16'd326;
            8'd202: recip_of = 16'd324;
            8'd203: recip_of = 16'd322;
            8'd204: recip_of = 16'd321;
            8'd205: recip_of = 16'd319;
            8'd206: recip_of = 16'd318;
            8'd207: recip_of = 16'd316;
            8'd208: recip_of = 16'd315;
            8'd209: recip_of = 16'd313;
            8'd210: recip_of = 16'd312;
            8'd211: recip_of = 16'd310;
            8'd212: recip_of = 16'd309;
            8'd213: recip_of = 16'd307;
            8'd214: recip_of = 16'd306;
            8'd215: recip_of = 16'd304;
            8'd216: recip_of = 16'd303;
            8'd217: recip_of = 16'd302;
            8'd218: recip_of = 16'd300;
            8'd219: recip_of = 16'd299;
            8'd220: recip_of = 16'd297;
            8'd221: recip_of = 16'd296;
            8'd222: recip_of = 16'd295;
            8'd223: recip_of = 16'd293;
            8'd224: recip_of = 16'd292;
            8'd225: recip_of = 16'd291;
            8'd226: recip_of = 16'd289;
            8'd227: recip_of = 16'd288;
            8'd228: recip_of = 16'd287;
            8'd229: recip_of = 16'd286;
            8'd230: recip_of = 16'd284;
            8'd231: recip_of = 16'd283;
            8'd232: recip_of = 16'd282;
            8'd233: recip_of = 16'd281;
            8'd234: recip_of = 16'd280;
            8'd235: recip_of = 16'd278;
            8'd236: recip_of = 16'd277;
            8'd237: recip_of = 16'd276;
            8'd238: recip_of = 16'd275;
            8'd239: recip_of = 16'd274;
            8'd240: recip_of = 16'd273;
            8'd241: recip_of = 16'd271;
            8'd242: recip_of = 16'd270;
            8'd243: recip_of = 16'd269;
            8'd244: recip_of = 16'd268;
            8'd245: recip_of = 16'd267;
            8'd246: recip_of = 16'd266;
            8'd247: recip_of = 16'd265;
            8'd248: recip_of = 16'd264;
            8'd249: recip_of = 16'd263;
            8'd250: recip_of = 16'd262;
            8'd251: recip_of = 16'd261;
            8'd252: recip_of = 16'd260;
            8'd253: recip_of = 16'd259;
            8'd254: recip_of = 16'd258;
            8'd255: recip_of = 16'd257;
            default: recip_of = 16'd65535;
            endcase
        end
    endfunction

    //--------------------------------------------------------------------------
    // 三行 × 12 像素灰度（位宽必须 3*GRAY_W，写成 GRAY_W 会被截断）
    //--------------------------------------------------------------------------
    wire [3*GRAY_W-1:0] top12 = {O_02[WORD_W-1:RGB_W], O_01[WORD_W-1:RGB_W], O_00[WORD_W-1:RGB_W]};
    wire [3*GRAY_W-1:0] mid12 = {O_12[WORD_W-1:RGB_W], O_11[WORD_W-1:RGB_W], O_10[WORD_W-1:RGB_W]};
    wire [3*GRAY_W-1:0] bot12 = {O_22[WORD_W-1:RGB_W], O_21[WORD_W-1:RGB_W], O_20[WORD_W-1:RGB_W]};

    genvar q;
    generate
        for (q = 0; q < PIX_PER_WORD; q = q + 1) begin : g_px

            // -------- 9 个邻居（q 是 genvar，下标全是常数，综合出来是纯连线）--------
            wire [7:0] p00 = top12[(q+3)*8 +: 8];
            wire [7:0] p01 = top12[(q+4)*8 +: 8];
            wire [7:0] p02 = top12[(q+5)*8 +: 8];
            wire [7:0] p10 = mid12[(q+3)*8 +: 8];
            wire [7:0] p11 = mid12[(q+4)*8 +: 8];
            wire [7:0] p12 = mid12[(q+5)*8 +: 8];
            wire [7:0] p20 = bot12[(q+3)*8 +: 8];
            wire [7:0] p21 = bot12[(q+4)*8 +: 8];
            wire [7:0] p22 = bot12[(q+5)*8 +: 8];

            wire [7:0] gray  = p11;                       // 中心像素灰度
            wire [7:0] gradc = I_blurw[q*8 +: 8];         // 粗尺度模糊值

            // 中心像素的原始 RGB（彩色铅笔上色用）
            wire [7:0] cr0 = O_11[q*24+16 +: 8];
            wire [7:0] cg0 = O_11[q*24+8  +: 8];
            wire [7:0] cb0 = O_11[q*24    +: 8];

            //==================================================================
            // S1：9 邻居求和 / 笔触疏密 / 色深
            //==================================================================
            wire [11:0] sum9_c = p00+p01+p02+p10+p11+p12+p20+p21+p22;   // <= 2295

            // 笔触疏密 = 纵向差分 |中心 - 上一行（同列）| × 32
            wire [15:0] difx_c = (gray > p01) ? ({8'd0,gray} - {8'd0,p01})
                                              : ({8'd0,p01} - {8'd0,gray});
            wire [12:0] dif5_c = {difx_c[7:0], 5'b0};                   // <= 255×32
            wire [7:0]  tex6_c = (dif5_c > 13'd255) ? 8'd255 : dif5_c[7:0];

            wire [7:0]  tone_c = 8'd255 - gray;                         // 暗部大 -> 亮处留白

            reg  [11:0] sum9_r;
            reg  [7:0]  gray_s1, gradc_s1, tex6_s1, tone_s1;
            reg  [7:0]  cr_s1, cg_s1, cb_s1;
            reg  [1:0]  hph_s1;

            //==================================================================
            // S2：blur = sum9 / 9 ；texp = 0.35 + 0.65·tex
            //==================================================================
            wire [23:0] sum9M_c = {12'd0, sum9_r} * 24'd3641;           // <= 8.36M
            wire [7:0]  blur_c  = sum9M_c[22:15];

            wire [15:0] texpx_c = {8'd0, tex6_s1} * 16'd166;
            wire [7:0]  texp_c  = 8'd87 + texpx_c[15:8];                // <= 87+165

            reg  [7:0]  blur_r;
            reg  [7:0]  gray_s2, gradc_s2, texp_s2, tone_s2;
            reg  [7:0]  cr_s2, cg_s2, cb_s2;
            reg  [1:0]  hph_s2;

            //==================================================================
            // S3：查倒数表 ；w1 = tone × texp
            //==================================================================
            wire [15:0] rc_c  = recip_of(blur_r);
            wire [15:0] rcc_c = recip_of(gradc_s2);
            wire [15:0] w1x_c = {8'd0, tone_s2} * {8'd0, texp_s2};
            wire [7:0]  w1_c  = w1x_c[15:8];

            reg  [15:0] rc_r, rcc_r;
            reg  [7:0]  gray_s3, w1_s3;
            // 局部填充亮度 = 16 行均值 × 88/256，提前算好并一路带到 S10
            wire [15:0] gb_mul_c = ({8'd0, gradc_s2} << 6) +
                                   ({8'd0, gradc_s2} << 4) +
                                   ({8'd0, gradc_s2} << 3);
            wire [7:0]  gbase_c  = gb_mul_c[15:8];
            reg  [7:0]  gbase_s3, gbase_s4, gbase_s5, gbase_s6,
                        gbase_s7, gbase_s8, gbase_s9, gbase_s10;
            reg  [7:0]  cr_s3, cg_s3, cb_s3;
            reg  [1:0]  hph_s3;

            //==================================================================
            // S4：gray × 倒数 ；45° 排线
            //==================================================================
            wire [23:0] gR_c  = {8'd0, gray_s3} * {8'd0, rc_r};         // <= 16.7M
            wire [23:0] gRc_c = {8'd0, gray_s3} * {8'd0, rcc_r};

            // 排线周期 4、占空 50%：亮的那一半满涂，暗的那一半减半
            //   (列+行) mod 4 == (行 mod 4) + q   —— 列号都是 4 的倍数
            wire [1:0]  ph_c  = hph_s3 + q;
            wire        hat_c = (ph_c < 2'd2);
            wire [7:0]  w2_c  = hat_c ? w1_s3 : {1'b0, w1_s3[7:1]};     // >>1 是移位

            reg  [23:0] gR_r, gRc_r;
            reg  [7:0]  w2_s4;
            reg  [7:0]  cr_s4, cg_s4, cb_s4;

            //==================================================================
            // S5：颜色减淡结果（钳到 255）；w3 = w2 × S
            //==================================================================
            wire [15:0] sh_c  = gR_r  >> 8;
            wire [15:0] shc_c = gRc_r >> 8;
            wire [7:0]  tdg_c = (sh_c  > 16'd255) ? 8'd255 : sh_c[7:0];
            wire [7:0]  tc_c  = (shc_c > 16'd255) ? 8'd255 : shc_c[7:0];

            wire [16:0] w3x_c = {9'd0, w2_s4} * 17'd331;                // <= 84405
            wire [7:0]  w3_c  = (w3x_c[16:8] > 8'd255) ? 8'd255 : w3x_c[16:8];

            reg  [7:0]  tdg_r, tc_r, w3_s5;
            reg  [7:0]  cr_s5, cg_s5, cb_s5;

            //==================================================================
            // S6：起伏放大
            //==================================================================
            wire [7:0]  dev_c  = 8'd255 - tdg_r;
            wire [7:0]  devc_c = 8'd255 - tc_r;
            wire [15:0] dv_c   = dev_c  * {8'd0, DODGE4};               // <= 1530
            // 边缘阈值：只保留超过 EDGE_TH 的粗尺度差，再固定 ×8（clean_new Kc）
            wire [7:0]  devc_eff = (devc_c > EDGE_TH) ? (devc_c - EDGE_TH) : 8'd0;
            wire [15:0] dvc_c    = devc_eff * 16'd8;                    // <= 2040

            reg  [15:0] dv_r, dvc_r;
            reg  [7:0]  w3_s6;
            reg  [7:0]  cr_s6, cg_s6, cb_s6;

            //==================================================================
            // S7：两尺度相加后一起钳位（不要各自钳了再加，那样会丢进位）
            //==================================================================
            wire [16:0] dv_all_c = {1'b0, dv_r} + {1'b0, dvc_r};
            wire [7:0]  dline_c  = (dv_all_c >= 17'd255) ? 8'd255 : dv_all_c[7:0];  // 线稿墨量

            reg  [7:0]  dline_s7, w3_s7;
            reg  [7:0]  cr_s7, cg_s7, cb_s7;

            //==================================================================
            // S8：涂色量 cfs = w3 + 线稿×0.6
            //==================================================================
            wire [15:0] d6x_c  = {8'd0, dline_s7} * 16'd153;            // <= 39015
            wire [8:0]  cfsx_c = {1'b0, w3_s7} + d6x_c[15:8];
            wire [7:0]  cfs_c  = (cfsx_c > 9'd255) ? 8'd255 : cfsx_c[7:0];

            reg  [7:0]  cfs_s8;
            reg  [7:0]  dline_s8;   // v3 黑白路径：dline 一路跟到 S10
            reg  [7:0]  cr_s8, cg_s8, cb_s8;

            //==================================================================
            // S9：上色，白纸 -> 原色，混入比例 = cfs
            //==================================================================
            wire [15:0] pr_c = {8'd0, (8'd255 - cr_s8)} * {8'd0, cfs_s8};
            wire [15:0] pg_c = {8'd0, (8'd255 - cg_s8)} * {8'd0, cfs_s8};
            wire [15:0] pb_c = {8'd0, (8'd255 - cb_s8)} * {8'd0, cfs_s8};
            wire [7:0]  qr_c = 8'd255 - pr_c[15:8];
            wire [7:0]  qg_c = 8'd255 - pg_c[15:8];
            wire [7:0]  qb_c = 8'd255 - pb_c[15:8];

            reg  [7:0]  qr_s9, qg_s9, qb_s9;
            reg  [7:0]  dline_s9;

            //==================================================================
            // S10：保持色彩深度 + 线稿压深
            //   2026-10-25：去掉原来的 "+45 抬黑场 / ×210>>8 压缩"。
            //   白纸保持 255，由最终混合阶段用减法叠加
            //   （见 ui_filter_domain_new：out = raw + (filt-255)*strength）。
            //   2026-10-25：彩色线稿压深。
            //       line = 0.344×16行局部均值 + 1.5×边缘像素色度
            //   线是"物体颜色的深色实线"，不是被 255−dline 压成的灰黑。
            //   混入量按 dline 分 5 档（128/160/192/224 -> 1/4..1）。
            //==================================================================
            wire [7:0]  er_c  = qr_s9;
            wire [7:0]  eg_c  = qg_s9;
            wire [7:0]  eb_c  = qb_s9;
            wire [7:0]  lum_c = 8'd255 - dline_s9;

            // 先做原来的按档压暗（原先在 S11 做，挪到这里可缩短 S11 路径）
            wire [7:0]  er_dk_c = darken8(er_c, pen_dark_mode_r);
            wire [7:0]  eg_dk_c = darken8(eg_c, pen_dark_mode_r);
            wire [7:0]  eb_dk_c = darken8(eb_c, pen_dark_mode_r);

            // 线稿浓淡档位：dline 越大，越接近 line 色
            //   128/160/192/224 -> 1/4、1/2、3/4、全量
            wire        line_full_c = (dline_s9 >= 8'd224);
            wire [1:0]  line_lvl_c  = (dline_s9 >= 8'd192) ? 2'd3 :
                                      (dline_s9 >= 8'd160) ? 2'd2 :
                                      (dline_s9 >= 8'd128) ? 2'd1 : 2'd0;

            // 边缘像素亮度 Y = (R + 2G + B) / 4
            wire [9:0]  line_ysum_c = {2'd0, er_dk_c} + {1'd0, eg_dk_c, 1'b0} +
                                      {2'd0, eb_dk_c};
            wire [7:0]  line_yl_c   = line_ysum_c[9:2];

            wire [7:0] er_line_c = line_col_local(er_dk_c, line_yl_c, gbase_s10);
            wire [7:0] eg_line_c = line_col_local(eg_dk_c, line_yl_c, gbase_s10);
            wire [7:0] eb_line_c = line_col_local(eb_dk_c, line_yl_c, gbase_s10);

            wire [7:0] er_mix_c = line_mix8(er_dk_c, er_line_c, line_lvl_c, line_full_c);
            wire [7:0] eg_mix_c = line_mix8(eg_dk_c, eg_line_c, line_lvl_c, line_full_c);
            wire [7:0] eb_mix_c = line_mix8(eb_dk_c, eb_line_c, line_lvl_c, line_full_c);

            reg  [7:0]  er_s10, eg_s10, eb_s10;
            reg  [7:0]  lum_s10;

            //==============================================================
            // S11：色彩强度
            //   0..64%  : 从黑白素描连续过渡到原彩铅（I_color 0..256）
            //   >64%    : 固定原彩铅，逐档放大色度并轻微压暗
            //   色度总倍数 1.25 / 1.5 / 1.75 / 2.0
            //==============================================================
            wire [7:0]  lum_base = lum_s10;   // S10 的黑白素描（纸白 255）

            // 压暗 + 线稿压深已在 S10 完成，这里直接用
            wire [7:0]  er_use = er_s10;
            wire [7:0]  eg_use = eg_s10;
            wire [7:0]  eb_use = eb_s10;

            // 彩铅输出的亮度 Y = (R + 2G + B) / 4
            wire [9:0]  er_sum_c = {2'd0, er_use} + {1'd0, eg_use, 1'b0} + {2'd0, eb_use};
            wire [7:0]  er_lum_c = er_sum_c[9:2];

            wire signed [9:0] dR_c = $signed({2'd0, er_use}) - $signed({2'd0, lum_base});
            wire signed [9:0] dG_c = $signed({2'd0, eg_use}) - $signed({2'd0, lum_base});
            wire signed [9:0] dB_c = $signed({2'd0, eb_use}) - $signed({2'd0, lum_base});

            // 0..256：从素描灰度过渡到原彩铅；>256 时固定为 256（原彩铅）
            wire signed [19:0] mR_c = dR_c * $signed({1'b0, pen_color_k_r});
            wire signed [19:0] mG_c = dG_c * $signed({1'b0, pen_color_k_r});
            wire signed [19:0] mB_c = dB_c * $signed({1'b0, pen_color_k_r});

            wire signed [10:0] baseR_c = pen_sat_en_r ? $signed({3'd0, er_use})
                                                    : ($signed({3'd0, lum_base}) + (mR_c >>> 8));
            wire signed [10:0] baseG_c = pen_sat_en_r ? $signed({3'd0, eg_use})
                                                    : ($signed({3'd0, lum_base}) + (mG_c >>> 8));
            wire signed [10:0] baseB_c = pen_sat_en_r ? $signed({3'd0, eb_use})
                                                    : ($signed({3'd0, lum_base}) + (mB_c >>> 8));

            // >64%：围绕 Y 放大色度。boost 档位：
            //   1.25x -> ch>>2, 1.5x -> ch>>1, 1.75x -> ch-(ch>>2),
            //   2.0x -> ch, 2.5x -> ch+(ch>>1), 3.0x -> ch<<1, 3.5x -> ch<<1+(ch>>1)
            wire signed [10:0] chR_c = $signed({3'd0, er_use}) - $signed({3'd0, er_lum_c});
            wire signed [10:0] chG_c = $signed({3'd0, eg_use}) - $signed({3'd0, er_lum_c});
            wire signed [10:0] chB_c = $signed({3'd0, eb_use}) - $signed({3'd0, er_lum_c});

            wire signed [10:0] bR_c = !pen_sat_en_r ? 11'sd0 :
                                      (pen_sat_mode_r == 3'd0) ? (chR_c >>> 2) :
                                      (pen_sat_mode_r == 3'd1) ? (chR_c >>> 1) :
                                      (pen_sat_mode_r == 3'd2) ? (chR_c - (chR_c >>> 2)) :
                                      (pen_sat_mode_r == 3'd3) ? chR_c :
                                      (pen_sat_mode_r == 3'd4) ? (chR_c + (chR_c >>> 1)) :
                                      (pen_sat_mode_r == 3'd5) ? (chR_c << 1) :
                                                               ((chR_c << 1) + (chR_c >>> 1));
            wire signed [10:0] bG_c = !pen_sat_en_r ? 11'sd0 :
                                      (pen_sat_mode_r == 3'd0) ? (chG_c >>> 2) :
                                      (pen_sat_mode_r == 3'd1) ? (chG_c >>> 1) :
                                      (pen_sat_mode_r == 3'd2) ? (chG_c - (chG_c >>> 2)) :
                                      (pen_sat_mode_r == 3'd3) ? chG_c :
                                      (pen_sat_mode_r == 3'd4) ? (chG_c + (chG_c >>> 1)) :
                                      (pen_sat_mode_r == 3'd5) ? (chG_c << 1) :
                                                               ((chG_c << 1) + (chG_c >>> 1));
            wire signed [10:0] bB_c = !pen_sat_en_r ? 11'sd0 :
                                      (pen_sat_mode_r == 3'd0) ? (chB_c >>> 2) :
                                      (pen_sat_mode_r == 3'd1) ? (chB_c >>> 1) :
                                      (pen_sat_mode_r == 3'd2) ? (chB_c - (chB_c >>> 2)) :
                                      (pen_sat_mode_r == 3'd3) ? chB_c :
                                      (pen_sat_mode_r == 3'd4) ? (chB_c + (chB_c >>> 1)) :
                                      (pen_sat_mode_r == 3'd5) ? (chB_c << 1) :
                                                               ((chB_c << 1) + (chB_c >>> 1));

            wire signed [10:0] sR_c = baseR_c + bR_c;
            wire signed [10:0] sG_c = baseG_c + bG_c;
            wire signed [10:0] sB_c = baseB_c + bB_c;

            wire [7:0] fr_c = (sR_c < 11'sd0) ? 8'd0 : (sR_c > 11'sd255) ? 8'd255 : sR_c[7:0];
            wire [7:0] fg_c = (sG_c < 11'sd0) ? 8'd0 : (sG_c > 11'sd255) ? 8'd255 : sG_c[7:0];
            wire [7:0] fb_c = (sB_c < 11'sd0) ? 8'd0 : (sB_c > 11'sd255) ? 8'd255 : sB_c[7:0];

            reg  [7:0] fr_s11, fg_s11, fb_s11;

            //------------------------------------------------------------------
            // 11 级流水寄存器（只加寄存器，不改运算）
            //------------------------------------------------------------------
            always @(posedge I_clk or negedge I_rst_n) begin
                if (!I_rst_n) begin
                    sum9_r  <= 12'd0;  gray_s1 <= 8'd0;  gradc_s1 <= 8'd0;
                    tex6_s1 <= 8'd0;   tone_s1 <= 8'd0;
                    cr_s1   <= 8'd0;   cg_s1   <= 8'd0;  cb_s1 <= 8'd0;
                    hph_s1  <= 2'd0;
                    blur_r  <= 8'd0;   gray_s2 <= 8'd0;  gradc_s2 <= 8'd0;
                    texp_s2 <= 8'd0;   tone_s2 <= 8'd0;
                    cr_s2   <= 8'd0;   cg_s2   <= 8'd0;  cb_s2 <= 8'd0;
                    hph_s2  <= 2'd0;
                    rc_r    <= 16'd0;  rcc_r   <= 16'd0; gray_s3  <= 8'd0;
                    w1_s3   <= 8'd0;
                    cr_s3   <= 8'd0;   cg_s3   <= 8'd0;  cb_s3 <= 8'd0;
                    hph_s3  <= 2'd0;
                    gbase_s3 <= 8'd0; gbase_s4 <= 8'd0; gbase_s5 <= 8'd0; gbase_s6 <= 8'd0;
                    gbase_s7 <= 8'd0; gbase_s8 <= 8'd0; gbase_s9 <= 8'd0; gbase_s10<= 8'd0;
                    gR_r    <= 24'd0;  gRc_r   <= 24'd0; w2_s4 <= 8'd0;
                    cr_s4   <= 8'd0;   cg_s4   <= 8'd0;  cb_s4 <= 8'd0;
                    tdg_r   <= 8'd0;   tc_r    <= 8'd0;  w3_s5 <= 8'd0;
                    cr_s5   <= 8'd0;   cg_s5   <= 8'd0;  cb_s5 <= 8'd0;
                    dv_r    <= 16'd0;  dvc_r   <= 16'd0; w3_s6 <= 8'd0;
                    cr_s6   <= 8'd0;   cg_s6   <= 8'd0;  cb_s6 <= 8'd0;
                    dline_s7<= 8'd0;   w3_s7 <= 8'd0;
                    cr_s7   <= 8'd0;   cg_s7   <= 8'd0;  cb_s7 <= 8'd0;
                    cfs_s8  <= 8'd0;   dline_s8 <= 8'd0;
                    cr_s8   <= 8'd0;   cg_s8   <= 8'd0;  cb_s8 <= 8'd0;
                    qr_s9   <= 8'd0;   qg_s9   <= 8'd0;  qb_s9 <= 8'd0;
                    er_s10  <= 8'd0;   eg_s10  <= 8'd0;  eb_s10<= 8'd0;
                    lum_s10 <= 8'd0;
                end else if (I_valid) begin
                    // S1
                    sum9_r  <= sum9_c;   gray_s1 <= gray;   gradc_s1 <= gradc;
                    tex6_s1 <= tex6_c;   tone_s1 <= tone_c;
                    cr_s1   <= cr0;      cg_s1   <= cg0;    cb_s1 <= cb0;
                    hph_s1  <= I_hatchph;
                    // S2
                    blur_r  <= blur_c;   gray_s2 <= gray_s1; gradc_s2 <= gradc_s1;
                    texp_s2 <= texp_c;   tone_s2 <= tone_s1;
                    cr_s2   <= cr_s1;    cg_s2   <= cg_s1;   cb_s2 <= cb_s1;
                    hph_s2  <= hph_s1;
                    // S3
                    rc_r    <= rc_c;     rcc_r   <= rcc_c;   gray_s3 <= gray_s2;
                    w1_s3   <= w1_c;     gbase_s3<= gbase_c;
                    cr_s3   <= cr_s2;    cg_s3   <= cg_s2;   cb_s3 <= cb_s2;
                    hph_s3  <= hph_s2;
                    // S4
                    gR_r    <= gR_c;     gRc_r   <= gRc_c;   w2_s4 <= w2_c;
                    cr_s4   <= cr_s3;    cg_s4   <= cg_s3;   cb_s4 <= cb_s3;
                    gbase_s4<= gbase_s3;
                    // S5
                    tdg_r   <= tdg_c;    tc_r    <= tc_c;    w3_s5 <= w3_c;
                    cr_s5   <= cr_s4;    cg_s5   <= cg_s4;   cb_s5 <= cb_s4;
                    gbase_s5<= gbase_s4;
                    // S6
                    dv_r    <= dv_c;     dvc_r   <= dvc_c;   w3_s6 <= w3_s5;
                    cr_s6   <= cr_s5;    cg_s6   <= cg_s5;   cb_s6 <= cb_s5;
                    gbase_s6<= gbase_s5;
                    // S7
                    dline_s7 <= dline_c; w3_s7 <= w3_s6;
                    cr_s7   <= cr_s6;    cg_s7   <= cg_s6;   cb_s7 <= cb_s6;
                    gbase_s7<= gbase_s6;
                    // S8
                    cfs_s8  <= cfs_c;
                    dline_s8<= dline_s7;
                    cr_s8   <= cr_s7;    cg_s8   <= cg_s7;   cb_s8 <= cb_s7;
                    gbase_s8<= gbase_s7;
                    // S9
                    qr_s9   <= qr_c;     qg_s9   <= qg_c;    qb_s9 <= qb_c;
                    dline_s9<= dline_s8; gbase_s9<= gbase_s8;
                    // S10
                    er_s10  <= er_mix_c; eg_s10 <= eg_mix_c; eb_s10 <= eb_mix_c;
                    lum_s10 <= lum_c;    gbase_s10<= gbase_s9;
                    // S11
                    fr_s11  <= fr_c;     fg_s11  <= fg_c;    fb_s11<= fb_c;
                end
            end


            assign O_rgb[q*24 +: 24] = {fr_s11, fg_s11, fb_s11};
        end
    endgenerate

    wire unused_lat = (LAT > 0);

endmodule
