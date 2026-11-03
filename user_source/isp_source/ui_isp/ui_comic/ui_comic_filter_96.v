//=============================================================================
//  ui_comic_filter_96.v --- 漫画 v11 滤镜的 96bit / 5进5出 包装（接 ISP 链 B 点）
//
//  插入点与铅笔完全相同：ui_Saturation_adj 输出 ↔ ui_data96_128 输入。
//
//  【结构】pencil_front（3×3 窗口 + 16 行大模糊，已验证）-> comic_v11
//          ——comic_v11 需要"与窗口对齐的 16 行纵向模糊"，正是 pencil_front 提供的
//
//  【延迟】
//    ENABLE=0：0 拍（纯直通，不花资源）
//    ENABLE=1：D + 484 + (6*WPL + 8) + 3 = 7198 + 484 + 2888 + 3 = 10573 拍
//              数据与控制信号由 comic_v11 自己同步产生，下游帧/行边界不变
//
//  【资源】≈ 1.2 Mbit（pencil_front 的 word_delay + blur_gray）+ 710 Kbit
//          （comic_v11 自己的 13 行缓存 / 三条延时）≈ 103 块 ERAM20K
//=============================================================================
`timescale 1ns/1ps

module ui_comic_filter_96 #(
    parameter integer IMG_WIDTH  = 1920,
    parameter integer IMG_HEIGHT = 1080,
    parameter integer ENABLE     = 1,
    parameter integer RADIUS     = 16,
    // 1 = 打开"16×32 带通清晰度"（gT2 = gT + (gT−Lb)·3/16）
    //     2026-10-18 晚：在 Bh 后面补了两级流水，既把那 17.9ns 切成三段，
    //     又正好补上 Lb 相对 gT_2 差的 2 个有效拍（详见 comic_v11.v 里的注释）
    parameter integer SHARP_EN   = 1,
    // 墨线灵敏度（2026-10-18 上板实测：原值 C-g>12、×14 在室内场景下墨线过细）
    //   TH   8 / GAIN 20  -> 轮廓明显变粗变黑，又不会把纹理都变成点
    //   想还原原工程行为就传 12 / 14
    parameter integer INK_TH     = 8,
    parameter integer INK_GAIN   = 20,
    // 平涂/清晰度默认值（comic_v11.v 的原默认）
    parameter integer FLAT_TH    = 4,
    parameter integer FLAT_GAIN  = 32,
    parameter integer SHARP_TH   = 20
)(
    input  wire        I_clk,
    input  wire        I_rst_n,

    input  wire [95:0] I_tdata,
    input  wire        I_tvalid,
    input  wire        I_tlast,
    input  wire        I_tuser,
    input  wire        I_tready,

    output wire [95:0] O_tdata,
    output wire        O_tvalid,
    output wire        O_tlast,
    output wire        O_tuser,
    output wire        O_tready
);

    localparam integer WPL = IMG_WIDTH / 4;

    // 这条链没有真实反压，原样透传
    assign O_tready = I_tready;

    generate
    if (ENABLE == 1) begin : g_on

        wire [127:0] w00, w01, w02, w10, w11, w12, w20, w21, w22;
        wire [31:0]  blurw;
        wire [1:0]   hphw;
        wire         f_vld, f_lst, f_usr;
        wire [95:0]  com_rgb;
        wire         com_vld, com_lst, com_usr;

        pencil_front #(
            .PIX_PER_WORD    (4              ),
            .RGB_W           (96             ),
            .GRAY_W          (32             ),
            .WORD_W          (128            ),
            .WORDS_PER_LINE  (WPL            ),
            .RADIUS          (RADIUS         )
        ) u_front (
            .I_clk   (I_clk   ),
            .I_rst_n (I_rst_n ),
            .I_tdata (I_tdata ),
            .I_tvalid(I_tvalid),
            .I_tlast (I_tlast ),
            .I_tuser (I_tuser ),
            .O_00(w00), .O_01(w01), .O_02(w02),
            .O_10(w10), .O_11(w11), .O_12(w12),
            .O_20(w20), .O_21(w21), .O_22(w22),
            .O_blurw (blurw ),
            .O_tvalid(f_vld ),
            .O_tlast (f_lst ),
            .O_tuser (f_usr ),
            .O_hatchph(hphw )      // 漫画不用排线，悬空
        );

        comic_v11 #(
            .PIX_PER_WORD    (4          ),
            .RGB_W           (96         ),
            .GRAY_W          (32         ),
            .WORD_W          (128        ),
            .WORDS_PER_LINE  (WPL        ),
            .LINES_PER_FRAME (IMG_HEIGHT ),
            .SHARP_EN        (SHARP_EN   ),
            .INK_TH          (INK_TH     ),
            .INK_GAIN        (INK_GAIN   )
        ) u_comic (
            .I_clk   (I_clk   ),
            .I_rst_n (I_rst_n ),
            .O_00(w00), .O_01(w01), .O_02(w02),
            .O_10(w10), .O_11(w11), .O_12(w12),
            .O_20(w20), .O_21(w21), .O_22(w22),
            .I_blurw (blurw   ),
            .I_tvalid(f_vld   ),
            .I_tlast (f_lst   ),
            .I_tuser (f_usr   ),
            .I_ink_th   (INK_TH   ),
            .I_ink_gain (INK_GAIN ),
            .I_flat_th  (FLAT_TH  ),
            .I_flat_gain(FLAT_GAIN),
            .I_sharp_th (SHARP_TH ),
            .O_rgb   (com_rgb ),
            .O_tvalid(com_vld ),
            .O_tlast (com_lst ),
            .O_tuser (com_usr )
        );

        assign O_tdata  = com_rgb;
        assign O_tvalid = com_vld;
        assign O_tlast  = com_lst;
        assign O_tuser  = com_usr;

    end else begin : g_off

        // 直通：0 拍延迟，一个 LUT 都不花
        assign O_tdata  = I_tdata;
        assign O_tvalid = I_tvalid;
        assign O_tlast  = I_tlast;
        assign O_tuser  = I_tuser;

    end
    endgenerate

endmodule
