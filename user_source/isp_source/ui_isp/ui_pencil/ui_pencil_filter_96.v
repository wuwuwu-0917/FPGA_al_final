//=============================================================================
//  pencil_filter_96.v --- 铅笔滤镜的 96bit / 5进5出 包装（接 ISP 链 B 点）
//
//  ---------------------------------------------------------------------------
//  【插在哪】ui_isp_top 里 ui_Saturation_adj 的输出 ↔ ui_data96_128 的输入
//          （即 Saturation_O_tdata / _tvalid / _tlast / _tuser / _tready）
//
//  【接口】和《ISP滤镜插入模块接口清单》第三节完全一致：
//          I_tdata 96bit = 一拍 4 个像素，像素 p 在 [p*24 +: 24] = {R,G,B}
//          I_tlast 每行第 480 个字那一拍；I_tuser 每帧第一拍
//
//  【反压】这条链没有反压（上游 O_tready 被硬接 1'b1），所以本模块**每拍都收**，
//          I_tready/O_tready 只是原样透传，不做任何判断。
//
//  【延迟】
//    ENABLE=0：0 拍（纯直通）
//    ENABLE=1：D + 484 + 10 = 7198 + 484 + 10 = 7692 拍
//              （D 见 pencil_front，484 是 line_window_3x3，10 是 pencil 的流水级数）
//          数据与控制信号延迟完全一致，所以下游看到的帧/行边界仍然正确。
//
//  【参数】
//    DODGE  = 6   细尺度起伏放大倍数 K
//    COARSE = 8   粗尺度起伏放大倍数 Kc
//    RADIUS = 16  粗尺度模糊半径（行）
//    这三个是用户 2026-10-15 验收定型的值，别随手改。
//    ⚠️ 改 RADIUS 会连带改 D，pencil_front 里自动重算，但**必须重跑 tb_pencil_filter**。
//=============================================================================
`timescale 1ns/1ps

module ui_pencil_filter_96 #(
    parameter integer IMG_WIDTH  = 1920,
    parameter integer IMG_HEIGHT = 1080,   // 目前只做文档参数，模块不依赖它
    parameter integer ENABLE     = 1,      // 0 = 直通（不上滤镜），1 = 上铅笔
    parameter integer DODGE      = 6,
    parameter integer COARSE     = 8,
    parameter integer RADIUS     = 16
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
    localparam integer LAT = 10;            // pencil 的流水级数（彩色通路需要 10 级）
    localparam [3:0]   LAT4 = LAT;          // 截断，避免 integer 参数位选

    // 这条链没有真实反压，原样透传
    assign O_tready = I_tready;

    generate
    if (ENABLE == 1) begin : g_on

        wire [127:0] w00, w01, w02, w10, w11, w12, w20, w21, w22;
        wire [31:0]  blurw;
        wire [1:0]   hphw;
        wire         f_vld, f_lst, f_usr;
        wire [95:0]  pen_rgb;

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
            .O_hatchph(hphw)
        );

        pencil #(
            .PIX_PER_WORD (4     ),
            .RGB_W        (96    ),
            .GRAY_W       (32    ),
            .WORD_W       (128   ),
            .DODGE        (DODGE ),
            .COARSE       (COARSE),
            .LAT          (LAT   )
        ) u_pencil (
            .I_clk   (I_clk   ),
            .I_rst_n (I_rst_n ),
            .I_valid (f_vld   ),
            .O_00(w00), .O_01(w01), .O_02(w02),
            .O_10(w10), .O_11(w11), .O_12(w12),
            .O_20(w20), .O_21(w21), .O_22(w22),
            .I_blurw (blurw   ),
            .I_hatchph(hphw   ),
            .I_dodge (DODGE[3:0] ),
            .I_coarse(COARSE[3:0]),
            .O_rgb   (pen_rgb )
        );

        //----------------------------------------------------------------------
        // 控制信号延迟 LAT 个**有效拍**，与 pencil 的流水对齐
        //   pencil 的流水寄存器只在 f_vld=1 时推进，所以这里也必须同样门控；
        //   输出再与上 f_vld，保证空档期三个同步信号都是 0（不会冻在高电平）。
        //----------------------------------------------------------------------
        // 深度 LAT(=7)：bit[6] 就正好是 7 个有效拍之前的那个脉冲
        reg [LAT-1:0] lst_sr, usr_sr;

        always @(posedge I_clk or negedge I_rst_n) begin
            if (!I_rst_n) begin
                lst_sr <= {LAT{1'b0}};
                usr_sr <= {LAT{1'b0}};
            end else if (f_vld) begin
                lst_sr <= {lst_sr[LAT-2:0], f_lst};
                usr_sr <= {usr_sr[LAT-2:0], f_usr};
            end
        end

        //------------------------------------------------------------------
        // pencil 的 7 级流水要吃满 7 个有效拍，第 8 个有效拍起 pen_rgb 才是
        // 第 (m-7) 个前级拍的结果。所以输出 valid 也要同时等这 7 拍，
        // 否则前 7 个输出字是流水线里的垃圾。
        //------------------------------------------------------------------
        reg [3:0] wcnt;

        always @(posedge I_clk or negedge I_rst_n) begin
            if (!I_rst_n)          wcnt <= 4'd0;
            else if (f_vld && (wcnt != LAT4))      wcnt <= wcnt + 4'd1;
        end

        wire w_ok = (wcnt >= LAT4);

        assign O_tdata  = pen_rgb;
        assign O_tvalid = f_vld & w_ok;
        assign O_tlast  = f_vld & w_ok & lst_sr[LAT-1];
        assign O_tuser  = f_vld & w_ok & usr_sr[LAT-1];

    end else begin : g_off

        // 直通：0 拍延迟，一个 LUT 都不花
        assign O_tdata  = I_tdata;
        assign O_tvalid = I_tvalid;
        assign O_tlast  = I_tlast;
        assign O_tuser  = I_tuser;

    end
    endgenerate

endmodule
