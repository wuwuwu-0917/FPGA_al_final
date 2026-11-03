//=============================================================================
//  pencil_front.v --- 铅笔的前端：把"3×3 窗口"和"大半径模糊"对齐输出
//
//  来源：FPGA_lvjing/rtl/pencil_front.v
//  改动：删掉 O_word_idx（调试用）；O_hatchph 保留（彩色铅笔排线相位）
//
//  ---------------------------------------------------------------------------
//  【为什么要单独一个模块干这件事】
//
//  铅笔 = 细尺度（3×3 减淡）+ 粗尺度（大半径减淡）相加。
//  两个尺度要**逐像素**相加，就必须严格对齐。但它们天然不对齐：
//
//      blur_gray        ：输出在 tick t 代表  行(t−2) − RADIUS   （空间上早 R 行）
//      line_window_3x3  ：端到端延迟 L = WPL + 4 = 484 拍
//
//  把送进窗口的流往后延 D 个字后，窗口在 tick t 代表 行(t−D−L)。
//  要求它和模糊代表同一个像素：
//
//          t − D − L = t − 2 − RADIUS·WPL
//      =>  D = RADIUS·WPL − L + 2 = WPL·(RADIUS−1) − 2
//
//  ⚠️ L 是**窗口端到端的真实拍数 = WORDS_PER_LINE + 4**，不是注释里写的 "+3"：
//     line_window_3x3 把同步先延迟 LAT_INT(=WPL+3) 拍，最后又用
//     `O_tvalid <= vld_d` 再寄存一级，所以真实延迟多 1 拍。
//     按 +3 算会整体错**一个字**（4 个像素），症状是"模糊和窗口差 4 列"，
//     只有把 dump 出来的模糊值横向滚 4 像素才对齐。
//
//      WPL=480, RADIUS=16 -> D = 7198
//
//  ⚠️ 这个式子很容易写错，第一版曾写成 WPL·RADIUS−1（多了整整一行）。
//     错一行的症状不是"整幅错位"，而是"细节全糊掉" —— 粗尺度项整体错开 1 行。
//     **只能靠真图逐像素对拍抓**（人造图案查不出来）。改这里务必重跑 tb。
//
//  【同步用谁的】以窗口的 O_tvalid/O_tlast/O_tuser 为准（模糊启动得早，
//  等窗口出数时模糊早就暖好了）。O_blurw 只在 O_tvalid 为高时有意义。
//
//  【资源】D=7198 时：延时 7198×99bit ≈ 712 Kbit + 模糊自身 ≈ 537 Kbit
//=============================================================================
`timescale 1ns/1ps

module pencil_front #(
    parameter integer PIX_PER_WORD    = 4,
    parameter integer RGB_W           = PIX_PER_WORD * 24,   // 96
    parameter integer GRAY_W          = PIX_PER_WORD * 8,    // 32
    parameter integer WORD_W          = RGB_W + GRAY_W,      // 128
    parameter integer WORDS_PER_LINE  = 480,
    parameter integer RADIUS          = 16,
    // ★ 2026-10-28：像素滤镜在 dly_data 抽头上，固有延迟比四路对齐点多
    //   PIXEL_ALIGN 拍。把窗口和模糊**同时**再延迟同样的拍数，四路整体后移，
    //   像素路就和 2892 对齐点重合，不需要在 ui_filter_domain_new 里再加链。
    parameter integer PIXEL_ALIGN     = 474
)(
    input  wire                  I_clk,
    input  wire                  I_rst_n,

    input  wire [RGB_W-1:0]      I_tdata,
    input  wire                  I_tvalid,
    input  wire                  I_tlast,
    input  wire                  I_tuser,

    // 3×3 字级窗口（约定同 line_window_3x3）
    output wire [WORD_W-1:0]     O_00, O_01, O_02,
    output wire [WORD_W-1:0]     O_10, O_11, O_12,
    output wire [WORD_W-1:0]     O_20, O_21, O_22,

    // 与窗口**同拍对齐**的大半径模糊灰度（4 个像素，像素 p 在 [p*8 +: 8]）
    // ⚠️ 只在 O_tvalid 为高时有意义
    output wire [GRAY_W-1:0]     O_blurw,

    // 以窗口为准的同步
    output wire                  O_tvalid,
    output wire                  O_tlast,
    output wire                  O_tuser,

    // 彩色铅笔排线用的相位 = 输出行号 & 3（见 pencil.v 的 I_hatchph）
    output wire [1:0]            O_hatchph,

    // ★ 2026-10-28 像素滤镜只读抽头：
    //   这是喂给 3×3 窗口之前的流（输入延迟 D=7198），只增加输出，
    //   不改变窗口/模糊/任何现有滤镜的时序。
    output wire [RGB_W-1:0]      O_tap_tdata,
    output wire                  O_tap_tvalid,
    output wire                  O_tap_tlast,
    output wire                  O_tap_tuser
);

    // D = WPL·(RADIUS−1) − 2（推导见文件头，别改成 WPL·RADIUS−1）
    localparam integer D = WORDS_PER_LINE * (RADIUS - 1) - 2;

    //--------------------------------------------------------------------------
    // 1) 输入延时 D 个字 -> 喂给 3×3 窗口
    //--------------------------------------------------------------------------
    wire [RGB_W-1:0] dly_data;
    wire             dly_valid, dly_last, dly_user;

    // 像素滤镜抽头（只读，见端口说明）
    assign O_tap_tdata  = dly_data;
    assign O_tap_tvalid = dly_valid;
    assign O_tap_tlast  = dly_last;
    assign O_tap_tuser  = dly_user;

    word_delay #(.W(RGB_W), .N(D)) u_dly (
        .I_clk (I_clk), .I_rst_n (I_rst_n),
        .I_data (I_tdata), .I_valid (I_tvalid), .I_last (I_tlast), .I_user (I_tuser),
        .O_data (dly_data), .O_valid (dly_valid), .O_last (dly_last), .O_user (dly_user)
    );

    //--------------------------------------------------------------------------
    // 1.5) 像素对齐：窗口输入再延迟 PIXEL_ALIGN 拍
    //      像素滤镜抽头在延迟**之前**，所以窗口/模糊整体后移后，
    //      像素路（tap + 8 行）正好和四路的 2892 对齐点重合。
    //--------------------------------------------------------------------------
    wire [RGB_W-1:0] win_data;
    wire             win_valid, win_last, win_user;

    word_delay #(.W(RGB_W), .N(PIXEL_ALIGN)) u_dly_win (
        .I_clk (I_clk), .I_rst_n (I_rst_n),
        .I_data (dly_data), .I_valid (dly_valid), .I_last (dly_last), .I_user (dly_user),
        .O_data (win_data), .O_valid (win_valid), .O_last (win_last), .O_user (win_user)
    );

    //--------------------------------------------------------------------------
    // 2) 3×3 窗口（吃延时后的流）
    //--------------------------------------------------------------------------
    line_window_3x3 #(
        .PIX_PER_WORD    (PIX_PER_WORD),
        .RGB_W           (RGB_W),
        .GRAY_W          (GRAY_W),
        .WORD_W          (WORD_W),
        .WORDS_PER_LINE  (WORDS_PER_LINE)
    ) u_win (
        .I_clk (I_clk), .I_rst_n (I_rst_n),
        .I_tdata (win_data), .I_tvalid (win_valid), .I_tlast (win_last), .I_tuser (win_user),
        .O_00(O_00), .O_01(O_01), .O_02(O_02),
        .O_10(O_10), .O_11(O_11), .O_12(O_12),
        .O_20(O_20), .O_21(O_21), .O_22(O_22),
        .O_tvalid(O_tvalid), .O_tlast(O_tlast), .O_tuser(O_tuser)
    );

    //--------------------------------------------------------------------------
    // 3) 大半径模糊（吃**原始**流，不延时）—— 这样它的行偏移恰好抵消
    //--------------------------------------------------------------------------
    wire [GRAY_W-1:0] blur_raw;
    wire bw_vld, bw_lst, bw_usr;

    blur_gray #(
        .PIX_PER_WORD    (PIX_PER_WORD),
        .RGB_W           (RGB_W),
        .GRAY_W          (GRAY_W),
        .WORDS_PER_LINE  (WORDS_PER_LINE),
        .RADIUS          (RADIUS)
    ) u_blur (
        .I_clk (I_clk), .I_rst_n (I_rst_n),
        .I_tdata (I_tdata), .I_tvalid (I_tvalid), .I_tlast (I_tlast), .I_tuser (I_tuser),
        .O_blur  (blur_raw),
        .O_tvalid (bw_vld), .O_tlast (bw_lst), .O_tuser (bw_usr)
    );

    // 模糊和窗口同步后移 PIXEL_ALIGN 拍，保持两者的行对齐
    wire [GRAY_W-1:0] blur_dly;
    wire bw_vld_d, bw_lst_d, bw_usr_d;
    word_delay #(.W(GRAY_W), .N(PIXEL_ALIGN)) u_dly_blur (
        .I_clk (I_clk), .I_rst_n (I_rst_n),
        .I_data (blur_raw), .I_valid (bw_vld), .I_last (bw_lst), .I_user (bw_usr),
        .O_data (blur_dly), .O_valid (bw_vld_d), .O_last (bw_lst_d), .O_user (bw_usr_d)
    );
    assign O_blurw = blur_dly;

    // 模糊的同步信号本模块用不上（以窗口为准），显式消费掉避免告警
    wire unused_ok = |{bw_vld, bw_lst, bw_usr, bw_vld_d, bw_lst_d, bw_usr_d};

    //--------------------------------------------------------------------------
    // 排线相位：输出行号 & 3
    //   排线图案是 (x+y) 的周期；一拍 4 个像素的列号都是 4 的倍数，
    //   所以 (列+行)&3 == (行&3)，lane 在 pencil 内部自己加。
    //   只在**有效拍**上推进（真机流有空档）。
    //--------------------------------------------------------------------------
    reg [1:0] hph;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) hph <= 2'd0;
        else if (O_tvalid && O_tlast) hph <= hph + 2'd1;
    end

    assign O_hatchph = hph;

endmodule
