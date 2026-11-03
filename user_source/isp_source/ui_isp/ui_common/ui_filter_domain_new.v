//=============================================================================
//  ui_filter_domain_new.v --- 滤镜集成层（四路并联，运行时多模式）
//
//  ---------------------------------------------------------------------------
//  【和 clean_new 的 ui_filter_domain.v 的区别】
//
//    clean_new：  铅笔 → 漫画 **串联**，PENCIL_ENABLE/COMIC_ENABLE 编译期二选一，
//                 两个 pencil_front 各占 1.2 Mbit。
//
//    本模块：     共用 **一个** pencil_front，铅笔/漫画/浮雕/直通 **四路并联**，
//                 I_mode 运行时选择，模式在帧首切换。
//
//  ---------------------------------------------------------------------------
//  【延迟（全部按有效拍）】
//
//      pencil_front           7682
//      直通（O_11）           +2888  = 10570
//      铅笔（10 级流水）      +2878  = 10570
//      浮雕（打 1 拍）        +2887  = 10570
//      漫画（内部 2888）       0     = 10570
//
//    四路**全部对齐到 10570**，所以切模式时画面不会跳。
//    代价是三条对齐延迟链（96bit × ~2888）≈ 42 块 ERAM20K。
//
//  ---------------------------------------------------------------------------
//  【同步信号】
//
//    pencil 和 emboss 的输出没有 valid/last/user（它们只按有效拍推进），
//    所以这里统一用 front 的 f_vld/f_lst/f_usr 送进 word_delay，
//    word_delay 内部会连控制一起延迟，天然对齐。
//
//    comic_v11 自带 O_tvalid/O_tlast/O_tuser，直接用。
//
//  ---------------------------------------------------------------------------
//  【参数映射】（由 FPGA_al 的 OSD 菜单驱动）
//
//    I_param1  混合强度 0..100（0 = 全原图，100 = 全滤镜）
//    I_param2  漫画 FLAT_GAIN / 浮雕 gain（铅笔线条强度已删除，不再使用）
//    I_param3  铅笔边缘阈值 / 漫画 INK_TH / 浮雕 dir
//    I_param4  漫画 INK_GAIN 0..32
//    I_param5  漫画 SHARP_TH 0..64
//=============================================================================
`timescale 1ns/1ps

module ui_filter_domain_new #(
    parameter integer PENCIL_RADIUS = 16,
    parameter integer COMIC_RADIUS  = 16,
    parameter integer COMIC_SHARP   = 1,
    parameter integer IMG_WIDTH     = 1920,
    parameter integer IMG_HEIGHT    = 1080,
    parameter integer FIFO_AW       = 9        // 512 深
)(
    // 相机域（74.25MHz）
    input  wire        I_clk,
    // 滤镜域（75MHz）
    input  wire        I_clk_flt,
    input  wire        I_rst_n,

    input  wire [95:0] I_tdata,
    input  wire        I_tvalid,
    input  wire        I_tlast,
    input  wire        I_tuser,

    // 控制（相机域寄存器输出，慢速；内部同步到滤镜域）
    //   I_mode: 0 原图 / 1 漫画 / 2 浮雕 / 3 铅笔
    input  wire [2:0]  I_mode,
    input  wire [7:0]  I_param1,
    input  wire [7:0]  I_param2,
    input  wire [7:0]  I_param3,
    input  wire [7:0]  I_param4,
    input  wire [7:0]  I_param5,

    // 原图对比：1 = 分割线左侧原图、右侧滤镜；I_split = 左侧占比 0..100
    input  wire        I_compare,
    input  wire [7:0]  I_split,

    output wire [95:0] O_tdata,
    output wire        O_tvalid,
    output wire        O_tlast,
    output wire        O_tuser,

    // 实测帧率（帧/秒），每 1 秒刷新一次；慢变量，调用方打两拍同步即可
    output reg  [7:0]  O_fps
);

    localparam integer FW     = 98;                 // {tuser, tlast, tdata[95:0]}
    localparam integer WPL    = IMG_WIDTH / 4;      // 480
    localparam integer WATER  = 128;                // 攒水深度（滤镜拍）
    localparam integer FRONT_LAT = 7682;            // front 的有效拍延迟（D=7198 + 窗口 484）

    // ★ 2026-10-28 像素滤镜对齐
    //   pixel_style 挂在 pencil_front 的 dly_data 抽头上（窗口之前）。
    //   它的固有延迟 = 8 行 + ERAM/表流水 = 3850 有效拍。
    //   pencil_front 把窗口和模糊同时再延迟 PIXEL_ALIGN 拍：
    //       像素路相对新 w11 = 3850 - (484 + PIXEL_ALIGN) = 2892
    //   正好和现有四路的 2892 对齐，不需要在滤镜域再挂对齐链。
    //   PIXEL_LAT/PIXEL_ALIGN 必须在 pixel TB 里实测；同步 BRAM 化后每加一拍改这里。
    localparam integer PIXEL_LAT   = 3850;
    localparam integer PIXEL_ALIGN = PIXEL_LAT - 484 - 2892;   // 474
    localparam integer FRONT_LAT2  = FRONT_LAT + PIXEL_ALIGN;  // 8156
    localparam integer TGT_LAT     = FRONT_LAT2 + 2892;        // 11048

    //==========================================================================
    // 1) 输入 FIFO：相机域 -> 滤镜域（+ 攒水，照抄 clean_new 的做法）
    //
    //  攒水的目的：把每行 ~60 段空档压成 1~2 段，避免 word_delay / 模糊链
    //  在空档边界错位导致"边缘跳动"。代价是多 WATER 个滤镜拍延迟。
    //==========================================================================
    wire [FW-1:0] w_dout;
    wire          w_valid, w_empty, w_full;

    reg [7:0] whold;
    always @(posedge I_clk_flt or negedge I_rst_n) begin
        if (!I_rst_n)                 whold <= 8'd0;
        else if (w_empty)             whold <= 8'd0;
        else if (whold != WATER[7:0]) whold <= whold + 8'd1;
    end

    wire f_read = ~w_empty & (whold == WATER[7:0]);

    sfifo #(
        .DATA_WIDTH_W   (FW      ),
        .DATA_WIDTH_R   (FW      ),
        .ADDR_WIDTH_W   (FIFO_AW ),
        .ADDR_WIDTH_R   (FIFO_AW ),
        .AL_FULL_NUM    (480     ),
        .AL_EMPTY_NUM   (2       ),
        .SHOW_AHEAD_EN  (1'b1    ),
        .OUTREG_EN      ("NOREG" )
    ) u_in_fifo (
        .rst        ( ~I_rst_n                    ),
        .clkw       ( I_clk                       ),
        .we         ( I_tvalid                    ),
        .di         ( {I_tuser, I_tlast, I_tdata} ),
        .full_flag  ( w_full                      ),
        .afull      (                             ),
        .clkr       ( I_clk_flt                   ),
        .re         ( f_read                      ),
        .dout       ( w_dout                      ),
        .valid      ( w_valid                     ),
        .empty_flag ( w_empty                     ),
        .aempty     (                             ),
        .wrusedw    (                             ),
        .rdusedw    (                             )
    );

    wire [95:0] f_i_data = w_dout[95:0];
    wire        f_i_last = w_dout[96];
    wire        f_i_user = w_dout[97];

    //==========================================================================
    // 2) 参数同步：相机域 -> 滤镜域（两级；参数是帧级慢变量）
    //==========================================================================
    reg [2:0] mode_s1, mode_s2;
    reg [7:0] p1_s1, p1_s2, p2_s1, p2_s2, p3_s1, p3_s2, p4_s1, p4_s2, p5_s1, p5_s2;
    reg       cmp_s1, cmp_s2;
    reg [7:0] spl_s1, spl_s2;

    always @(posedge I_clk_flt or negedge I_rst_n) begin
        if (!I_rst_n) begin
            mode_s1 <= 3'd0; mode_s2 <= 3'd0;
            p1_s1 <= 8'd75; p1_s2 <= 8'd75;
            p2_s1 <= 8'd6;  p2_s2 <= 8'd6;
            p3_s1 <= 8'd8;  p3_s2 <= 8'd8;
            p4_s1 <= 8'd20; p4_s2 <= 8'd20;
            p5_s1 <= 8'd20; p5_s2 <= 8'd20;
            cmp_s1 <= 1'b0; cmp_s2 <= 1'b0;
            spl_s1 <= 8'd50; spl_s2 <= 8'd50;
        end
        else begin
            mode_s1 <= I_mode;  mode_s2 <= mode_s1;
            p1_s1 <= I_param1;  p1_s2 <= p1_s1;
            p2_s1 <= I_param2;  p2_s2 <= p2_s1;
            p3_s1 <= I_param3;  p3_s2 <= p3_s1;
            p4_s1 <= I_param4;  p4_s2 <= p4_s1;
            p5_s1 <= I_param5;  p5_s2 <= p5_s1;
            cmp_s1 <= I_compare; cmp_s2 <= cmp_s1;
            spl_s1 <= I_split;   spl_s2 <= spl_s1;
        end
    end

    // 各模式的内部参数（同一个 param 在不同模式下语义不同）
    //
    // ★★ 2026-10-21 上板修正：UI 参数是 0..100，**不能直接截低位**
    //    （p2[3:0] 会让 5 和 21 得到同一个值 -> 调参循环）。
    //    统一改成"高位截断 = 线性缩放"：
    //        p[6:3]  -> 0..12（100/128*16）
    //        p[6:2]  -> 0..25
    //        p[6:1]  -> 0..50
    //    DODGE/COARSE/浮雕深度只有 0..15 档，用 p*1.25>>3 把 0..100 均匀铺满：
    //        0..100 -> 0..15，每 ~6.7 个 UI 值跨 1 档（短按 +1 也能看到变化）
    // 各模式的内部参数（同一个 param 在不同模式下语义不同）
    // ★ 2026-10-22 回滚到 2026-10-21 22:53 版本：资源临界时先保证画面
    // ★ 2026-10-24 上板反馈"线条强度和边缘阈值有循环"：
    //   原来取 p2[3:0]/p3[3:0]，UI 0..100 每 16 就回绕一圈（16->0、32->0…），
    //   而且 secondary 默认 80 的低 4 位正好是 0，等于一直是 0。
    //   改成高位截取 = 单调缩放：0..100 -> 0..12（每 8 个 UI 值跨 1 档）。
    //   铅笔内部 DODGE4/COARSE4 是 4bit（物理上限 15），12 已经够用。
    // ★ 2026-10-25：铅笔的"线条强度"滑块已删除（上板确认无效）。
    //   细尺度固定为 clean_new 验收值 DODGE=6。
    //   边缘阈值 p3：0..100 -> 0..156 的二次曲线（p3²/64）。
    //   线性阈值时 0..20 就把弱边缘删得差不多，20..80 看起来没变化；
    //   二次曲线让 0..20 只轻微收紧，20..80 持续删中等边缘，80..100 再收紧强边缘。
    //   数值越大 -> 只保留越强的边缘（与"阈值"方向一致）。
    wire [15:0] p3_sq     = p3_s2 * p3_s2;            // <= 10000
    wire [7:0] pen_dodge  = 8'd6;
    wire [7:0] pen_coarse = p3_sq >> 6;               // 0..156
    // 色彩强度：UI 0..100 -> 内部 0..400（×4）。
    //   0   = v3 黑白素描（clean_NBpencil 同款）
    //   64  = 原色（I_color=256 时 out = 原彩铅）
    //   100 = 4 倍过饱和，色彩被明显加强
    wire [8:0] pen_color  = {p4_s2, 2'b00};

    // ★★ 2026-10-23 上板反馈"描边强度和平滑强度的功能互换了"——查实了：
    //   UI 漫画模式的 4 个滑块在 hdmi_filter_ui.v 里是
    //     p1 = 强度      默认 50   范围 0..100
    //     p2 = 描边强度  默认 40   范围 0..100   (label 23，字模逐字核对)
    //     p3 = 墨线阈值  默认 3    范围 0..10    (label 33，2026-10-25 只保留前 10 档)
    //     p4 = 平滑强度  默认 100  范围 0..100   (label 5)
    //   之前的映射把 p2 接到平涂、p4 接到墨线，正好对调了。
    //
    //   现在以 clean_new 的验收值 INK_TH=8 / INK_GAIN=20 / FLAT_TH=4 /
    //   FLAT_GAIN=32 / SHARP_TH=20 为锚点：
    //     描边强度 40  -> INK_GAIN = p2>>1 = 20    （0..100 -> 0..50）
    //     平滑强度 100 -> FLAT_GAIN = p4*0.32 ≈ (p4>>1)-(p4>>3)-(p4>>4) = 32
    //   墨线阈值 p3 直接给 INK_TH（0..10，默认 3）。
    //   comic_v11 里没有色调量化级，所以原来的"色阶数量"本来就没硬件实现，
    //   2026-10-23 按用户要求把这个滑块改成墨线阈值，字模改为"墨线阈值"。
    // ★ 2026-10-23 第二轮上板反馈：
    //   (a) 描边强度"前 50% 变化大、后 50% 几乎没变化"。
    //       ink = clamp((C−G−TH)·INK_GAIN)，真实图像 C−G 只有 10~20，
    //       减掉 TH=8 之后只剩 2~12，所以 INK_GAIN 一过 25 就全饱和到 255。
    //       原来 0..100 -> 0..50 的后一半全在饱和区，白给。
    //       ⇒ 压到 0..25：p2>>2，默认 80 -> 20（clean_new 验收值）。
    //   (b) 平滑强度"调了没效果"。原来动的是 FLAT_GAIN（32..94），
    //       而 w=(|g−C|−TH)·GAIN 一过 255 就钳位，|g−C| 在 7~12 之间的像素
    //       本来就少，所以两端的差别被淹没了。**真正决定平滑量的是死区
    //       FLAT_TH**：|g−C| <= FLAT_TH 的像素被完全抹成局部均值。
    //       ⇒ 平滑强度改控 FLAT_TH（0..12，p4>>3），FLAT_GAIN 固定 32。
    //       默认 p4=32 -> TH=4（clean_new 验收值），p4=100 -> TH=12（明显平滑）。
    wire [7:0] cm_ink_gain  = {4'd0, p2_s2[7:2]};
    // ★ 2026-10-24：平滑强度滑块撤掉了（效果不线性、用户要求去掉），
    //   固定成 clean_new 验收过的那组值。p4 不再参与运算。
    wire [7:0] cm_flat_th   = 8'd4;
    wire [7:0] cm_flat_gain = 8'd32;
    wire [7:0] cm_ink_th    = p3_s2;
    wire [7:0] cm_sharp_th  = 8'd20;

    // ★ 2026-10-22：原来 p2_s2[3:0] 是 4bit 截断，UI 每 16 就回绕一圈
    //   （16->0, 32->0 ...）。改成高位截取 = 单调缩放：
    //   UI 7->0, 8->1, 60->7, 100->12，上限正好是浮雕增益的物理最大值 12。
    wire [3:0] emb_gain = {1'b0, p2_s2[6:3]};
    wire [1:0] emb_dir  = p3_s2[1:0];

    // ★ 2026-10-28 像素滤镜：p2（UI 模式4 的“风格模式”）0/1/2 -> D/E/F
    //   p3 在像素模式不使用。p2 被 UI 限制在 0..2，这里再做一次钳位。
    wire [1:0] pix_style = (p2_s2 > 8'd2) ? 2'd2 : p2_s2[1:0];


    //==========================================================================
    // 3) 共用前端：3×3 窗口 + 16 行大模糊
    //==========================================================================
    wire [127:0] w00, w01, w02, w10, w11, w12, w20, w21, w22;
    wire [31:0]  blurw;
    wire         f_vld, f_lst, f_usr;
    wire [1:0]   f_hph;

    // 像素滤镜抽头（窗口之前的 96bit 原像素流）
    wire [95:0]  tap_data;
    wire         tap_valid, tap_last, tap_user;

    pencil_front #(
        .PIX_PER_WORD   (4            ),
        .RGB_W          (96           ),
        .GRAY_W         (32           ),
        .WORD_W         (128          ),
        .WORDS_PER_LINE (WPL          ),
        .RADIUS         (PENCIL_RADIUS),
        .PIXEL_ALIGN    (PIXEL_ALIGN  )
    ) u_front (
        .I_clk    (I_clk_flt),
        .I_rst_n  (I_rst_n  ),
        .I_tdata  (f_i_data ),
        .I_tvalid (f_read   ),
        .I_tlast  (f_i_last ),
        .I_tuser  (f_i_user ),
        .O_00(w00), .O_01(w01), .O_02(w02),
        .O_10(w10), .O_11(w11), .O_12(w12),
        .O_20(w20), .O_21(w21), .O_22(w22),
        .O_blurw  (blurw    ),
        .O_tvalid (f_vld    ),
        .O_tlast  (f_lst    ),
        .O_tuser  (f_usr    ),
        .O_hatchph(f_hph    ),
        .O_tap_tdata (tap_data ),
        .O_tap_tvalid(tap_valid),
        .O_tap_tlast (tap_last ),
        .O_tap_tuser (tap_user )
    );

    //==========================================================================
    // 3.5) 暖机屏蔽（避免复位撤销阶段的不定态传播）
    //
    //   line_window_3x3 的 3 行行缓存在**前 3 行**里 top/bottom 还是 RAM 初值 X；
    //   pencil 的 10 级流水、emboss 的 1 拍、comic 的 4 级对齐也一样，
    //   开头几拍为不定态。不定态会顺着 word_delay 写入 FIFO 并传到显示端。
    //
    //   这里数满 1536 个有效拍（≈3.2 行）之前的支路输出一律替换成 0。
    //   只影响开机后前 3 行，之后 warm_ok 恒为 1。
    //==========================================================================
    //   阈值取 4608 = comic 的输出延迟 2888 + 窗口暖机约 1720。
    //   代价是开机后约 9.6 行不输出，之后 warm_ok 恒为 1。
    reg [12:0] warm_cnt;
    always @(posedge I_clk_flt or negedge I_rst_n) begin
        if (!I_rst_n)                          warm_cnt <= 13'd0;
        else if (f_vld && warm_cnt != 13'd4608) warm_cnt <= warm_cnt + 1'b1;
    end
    wire warm_ok = (warm_cnt == 13'd4608);

    //==========================================================================
    // 4) 三路滤镜（全部吃同一个窗口）
    //==========================================================================
    // 4.1 铅笔（10 级流水，输出无 valid）
    wire [95:0] pen_rgb;
    pencil #(
        .PIX_PER_WORD (4        ),
        .RGB_W        (96       ),
        .GRAY_W       (32       ),
        .WORD_W       (128      ),
        .DODGE        (6        ),
        .COARSE       (8        ),
        .LAT          (10       )
    ) u_pencil (
        .I_clk   (I_clk_flt),
        .I_rst_n (I_rst_n  ),
        .I_valid (f_vld    ),
        .O_00(w00), .O_01(w01), .O_02(w02),
        .O_10(w10), .O_11(w11), .O_12(w12),
        .O_20(w20), .O_21(w21), .O_22(w22),
        .I_blurw (blurw    ),
        .I_hatchph(f_hph   ),
        .I_dodge (pen_dodge ),
        .I_coarse(pen_coarse),
        .I_color (pen_color ),
        .O_rgb   (pen_rgb  )
    );

    // 4.2 漫画（内部 2888 拍，自带同步）
    wire [95:0] com_rgb;
    wire        com_vld, com_lst, com_usr;
    comic_v11 #(
        .PIX_PER_WORD    (4          ),
        .RGB_W           (96         ),
        .GRAY_W          (32         ),
        .WORD_W          (128        ),
        .WORDS_PER_LINE  (WPL        ),
        .LINES_PER_FRAME (IMG_HEIGHT ),
        .SHARP_EN        (COMIC_SHARP),
        .INK_TH          (12         ),
        .INK_GAIN        (14         ),
        .FLAT_TH         (4          ),
        .FLAT_GAIN       (32         ),
        .SHARP_TH        (20         )
    ) u_comic (
        .I_clk   (I_clk_flt ),
        .I_rst_n (I_rst_n   ),
        .O_00(w00), .O_01(w01), .O_02(w02),
        .O_10(w10), .O_11(w11), .O_12(w12),
        .O_20(w20), .O_21(w21), .O_22(w22),
        .I_blurw (blurw ),
        .I_tvalid(f_vld ),
        .I_tlast (f_lst ),
        .I_tuser (f_usr ),
        .I_ink_th   (cm_ink_th   ),
        .I_ink_gain (cm_ink_gain ),
        .I_flat_th  (cm_flat_th  ),
        .I_flat_gain(cm_flat_gain),
        .I_sharp_th (cm_sharp_th ),
        .O_rgb   (com_rgb),
        .O_tvalid(com_vld),
        .O_tlast (com_lst),
        .O_tuser (com_usr)
    );

    // 4.3 浮雕（1 拍延迟，输出无 valid）
    wire [95:0] emb_rgb;
    emboss #(
        .PIX_PER_WORD (4  ),
        .RGB_W        (96 ),
        .GRAY_W       (32 ),
        .WORD_W       (128)
    ) u_emboss (
        .I_clk   (I_clk_flt),
        .I_rst_n (I_rst_n  ),
        .I_valid (f_vld    ),
        .I_dir   (emb_dir  ),
        .O_00(w00), .O_01(w01), .O_02(w02),
        .O_10(w10), .O_11(w11), .O_12(w12),
        .O_20(w20), .O_21(w21), .O_22(w22),
        .I_blurw (blurw    ),
        .I_gain  (emb_gain ),
        .I_color (2'd0     ),      // 灰度浮雕（回滚版）
        .I_K     (4'd12    ),
        .I_CH    (7'd26    ),
        .O_rgb   (emb_rgb  )
    );

    // 4.4 像素（复用 xiangsu/rtl/pixel_style_opt.v 的算法）
    //     抽头在 3×3 窗口之前；输出自带 valid/last/user，延迟 8 行。
    //     表暂由 TB 通过层次 $readmemh 预装；正式版改成 ERAM INIT_FILE。
    wire [95:0] pix_rgb;
    wire        pix_vld, pix_lst, pix_usr;

    pixel_style_bram #(
        .WORDS_PER_LINE  (WPL        ),
        .LINES_PER_FRAME (IMG_HEIGHT ),
        .RGB_W           (96         )
    ) u_pixel (
        .I_clk      (I_clk_flt),
        .I_rst_n    (I_rst_n  ),
        .I_tdata    (tap_data ),
        .I_tvalid   (tap_valid),
        .I_tlast    (tap_last ),
        .I_tuser    (tap_user ),
        .I_style    (pix_style),
        .O_tdata    (pix_rgb  ),
        .O_tvalid   (pix_vld  ),
        .O_tlast    (pix_lst  ),
        .O_tuser    (pix_usr  )
    );

    //==========================================================================
    // 5) 五路对齐到 TGT_LAT（现有四路 2892 + 像素路多出的 ALIGN_DELAY）
    //==========================================================================
    //----------------------------------------------------------------------
    // ★ 2026-10-30 资源压缩（方案 A）：四路模式对齐延时合并
    //
    //   原来 raw / pen / emb 各挂一个约 2890 字的 ERAM 环
    //   （u_dly_raw 2892 / u_dly_pen 2881 / u_dly_emb 2891，合计 36 ERAM），
    //   唯一目的是在 mux 之前把三路拉到同一时刻——而同一时刻只显示一路。
    //
    //   改成「小残差触发器 + 一次 mux + 一个共享环」：
    //       共享环 N = min(三路原 N) = 2881
    //       残差    = 原 N − 2881  →  raw 11 拍、emb 10 拍、pen 0 拍
    //       漫画原本 0 拍（自带 2888+4），比共享环多 11 拍，补 11 拍
    //
    //   残差用触发器实现（约 3.2k 个 flop，占 slice 不占 ERAM），
    //   预计省 20~28 ERAM。像素级对齐与观感完全不变。
    //
    //   代价：切模式时共享环里还是上一路的数据，帧头约 6 行（2881 字）是旧画面。
    //   mode_cur 只在帧首（com_usr）切换，过渡落在帧边界，实际看不出。
    //
    //   ⚠️ 残差 11 / 10 是由下面三个原 N 值算出来的，改任何一个原 N 都要重算。
    //----------------------------------------------------------------------
    wire [95:0] pen_rgb_m = warm_ok ? pen_rgb : 96'd0;
    wire [95:0] emb_rgb_m = warm_ok ? emb_rgb : 96'd0;

    // 铅笔：pencil 的 10 级流水**只在有效拍推进**，所以 f_vld 时 pen_rgb
    //       已经是 10 个有效拍前的像素，tlast/tuser 也必须按有效拍延迟 10 拍
    //       （2026-10-21 上板实测 rd%480=469 修出来的，别动）。
    reg [10:0] pen_lst_sr, pen_usr_sr;
    always @(posedge I_clk_flt or negedge I_rst_n) begin
        if (!I_rst_n) begin
            pen_lst_sr <= 11'd0;
            pen_usr_sr <= 11'd0;
        end
        else if (f_vld) begin
            pen_lst_sr <= {pen_lst_sr[9:0], f_lst};
            pen_usr_sr <= {pen_usr_sr[9:0], f_usr};
        end
    end

    // 浮雕：输出是 n−1，tlast/tuser 延 1 个有效拍
    reg emb_lst_d1, emb_usr_d1;
    always @(posedge I_clk_flt or negedge I_rst_n) begin
        if (!I_rst_n) begin
            emb_lst_d1 <= 1'b0;
            emb_usr_d1 <= 1'b0;
        end
        else if (f_vld) begin
            emb_lst_d1 <= f_lst;
            emb_usr_d1 <= f_usr;
        end
    end

    //---------------- 残差移位寄存器（触发器，不进 ERAM）----------------
    // 每格一个字的 {data 96b, last, user, valid}；只在有效拍推进。
    // 复位清零，避免首帧把 X 灌进共享环。
    // 位序： [0]=valid [1]=user [2]=last [98:3]=data

    // 原图：**保留独立环**——「原图对比」需要一路与 mux 无关的原始流
    //   （分割线左侧显示原图），所以这一路不能并进共享环。
    wire [95:0] raw_d;
    wire        raw_vld, raw_lst, raw_usr;
    word_delay #(.W(96), .N(2892)) u_dly_raw (
        .I_clk  (I_clk_flt),
        .I_rst_n(I_rst_n  ),
        .I_data (w11[95:0]),
        .I_valid(f_vld    ),
        .I_last (f_lst    ),
        .I_user (f_usr    ),
        .O_data (raw_d    ),
        .O_valid(raw_vld  ),
        .O_last (raw_lst  ),
        .O_user (raw_usr  )
    );

    // 铅笔 0 拍（2881 − 2881）：直接进共享环
    wire [98:0] pen_pout = {pen_rgb_m, pen_lst_sr[10], pen_usr_sr[10], f_vld};

    // 浮雕 10 拍（2891 − 2881）
    wire [98:0] emb_pin  = {emb_rgb_m, emb_lst_d1, emb_usr_d1, f_vld};
    reg  [10*99-1:0] emb_rsr;
    wire [98:0] emb_pout = emb_rsr[9*99 +: 99];
    always @(posedge I_clk_flt or negedge I_rst_n) begin
        if (!I_rst_n) emb_rsr <= {(10*99){1'b0}};
        else if (f_vld) emb_rsr <= {emb_rsr[9*99-1:0], emb_pin};
    end

    // 漫画：**不进共享环**（它自带 2888+4 = 2892，本来就和原对齐点一致）
    //   tlast/tuser 的 4 拍对齐已在 comic_v11 内部补齐（2026-10-23 修"图像重叠"）
    wire [95:0] com_d = warm_ok ? com_rgb : 96'd0;

    //==========================================================================
    // 6) 模式 mux（帧首切换）
    //==========================================================================
    reg [2:0] mode_cur;
    always @(posedge I_clk_flt or negedge I_rst_n) begin
        if (!I_rst_n)       mode_cur <= 3'd0;
        else if (com_usr)   mode_cur <= mode_s2;   // 以漫画的帧首为准（四路已对齐）
    end

    // 三路（原图 / 铅笔 / 浮雕）已由残差对齐到同一时刻，先 mux，再进**
    // 唯一的共享 2881 字环**；漫画走自己的 11 拍残差。
    wire [98:0] pre_p = (mode_cur == 3'd2) ? emb_pout : pen_pout;

    wire [95:0] mux_d;
    wire        mux_vld, mux_lst, mux_usr;
    word_delay #(.W(96), .N(2881)) u_dly_mode (
        .I_clk  (I_clk_flt),
        .I_rst_n(I_rst_n  ),
        .I_data (pre_p[98:3]),
        .I_valid(f_vld      ),
        .I_last (pre_p[2]   ),
        .I_user (pre_p[1]   ),
        .O_data (mux_d    ),
        .O_valid(mux_vld  ),
        .O_last (mux_lst  ),
        .O_user (mux_usr  )
    );

    // mode: 0=原图(独立环) 1=漫画 2/3=浮雕/铅笔(共享环) 4=像素
    wire cur_pen_emb = (mode_cur == 3'd2) || (mode_cur == 3'd3);

    wire [95:0] cur_filt = (mode_cur == 3'd1) ? com_d   :
                           cur_pen_emb        ? mux_d   : raw_d;
    wire        cur_vld  = (mode_cur == 3'd1) ? com_vld :
                           cur_pen_emb        ? mux_vld : raw_vld;
    wire        cur_lst  = (mode_cur == 3'd1) ? com_lst :
                           cur_pen_emb        ? mux_lst : raw_lst;
    wire        cur_usr  = (mode_cur == 3'd1) ? com_usr :
                           cur_pen_emb        ? mux_usr : raw_usr;

    //==========================================================================
    // 6.5 五路汇合：pencil_front 已把窗口/模糊整体后移 PIXEL_ALIGN，
    //     像素路和 cur_* 都落在同一个 2892 对齐点，直接 mux 即可。
    //==========================================================================
    wire [95:0] fin_filt = (mode_cur == 3'd4) ? pix_rgb : cur_filt;
    wire        fin_vld  = (mode_cur == 3'd4) ? pix_vld : cur_vld;
    wire        fin_lst  = (mode_cur == 3'd4) ? pix_lst : cur_lst;
    wire        fin_usr  = (mode_cur == 3'd4) ? pix_usr : cur_usr;

    //==========================================================================
    // 7) 对比分割 + 强度混合
    //==========================================================================
    // 7.1 输出端字计数（和 fin_* 天然对齐）
    reg [9:0] w_cnt;
    always @(posedge I_clk_flt or negedge I_rst_n) begin
        if (!I_rst_n)          w_cnt <= 10'd0;
        else if (fin_vld) begin
            if (fin_lst)       w_cnt <= 10'd0;
            else               w_cnt <= w_cnt + 10'd1;
        end
    end

    // split 0..100 -> 0..1920：split*1920/100 = split*19.2
    //   19.2 ≈ 4915/256（4915/256 = 19.199，误差 0.006%），用移位代替除法
    wire [23:0] spl_mul = spl_s2 * 16'd4915;      // ≤ 100*4915 = 491500
    wire [11:0] split_x = spl_mul[19:8];          // >>8，≤ 1920

    wire [11:0] x_pix = {w_cnt, 2'b00};           // 4*w_cnt（q 在下面逐像素加）

    // 7.2 强度混合：strength 0..100 -> Q7 0..125
    // ★ 2026-10-22：强度改非线性（两段折线）
    //   0..60 段斜率 1.75：UI 60 -> Q7 105（约 84% 效果）
    //   60..100 段斜率 0.5：UI 100 -> Q7 125（100%）
    //   滑块对外仍是均匀 0..100，只是前段每格涨得多、后段涨得少。
    //   两段都是纯移位加，不占 DSP。
    wire [7:0] str_lo   = p1_s2 + (p1_s2 >> 1) + (p1_s2 >> 2);
    wire [7:0] str_hi_x = p1_s2 - 8'd60;
    wire [7:0] str_hi   = 8'd105 + (str_hi_x >> 1);
    wire [7:0] str_q7   = (p1_s2 <= 8'd60) ? str_lo : str_hi;

    function [7:0] clamp8;
        input signed [10:0] v;
        begin
            clamp8 = (v < 11'sd0) ? 8'd0 : (v > 11'sd255) ? 8'd255 : v[7:0];
        end
    endfunction

    wire [95:0] mix_rgb;    // 混合后的像素（写进输出 FIFO）

    genvar q;
    generate
        for (q = 0; q < 4; q = q + 1) begin : g_mix
            wire [7:0] filt_r = fin_filt[q*24+16 +: 8];
            wire [7:0] filt_g = fin_filt[q*24+8  +: 8];
            wire [7:0] filt_b = fin_filt[q*24    +: 8];

            wire [7:0] raw_r  = raw_d[q*24+16 +: 8];
            wire [7:0] raw_g  = raw_d[q*24+8  +: 8];
            wire [7:0] raw_b  = raw_d[q*24    +: 8];

            // 对比：分割线左侧用原图
            wire [11:0] xq = x_pix + q;
            wire use_raw = cmp_s2 && (mode_cur != 3'd0) && (xq < split_x);

            // ★ 2026-10-24：铅笔色彩强度 = 0 时，pencil 输出本身是灰度，
            //   但强度混合仍会混入原图颜色（str_q7 最大 125/128，默认 75% 时
            //   混入 12.5%），所以看起来"没有完全去色"。
            //   p4=0 时把参与混合的原图侧也换成灰度，保证 R=G=B；
            //   use_raw（对比左半屏）仍保留真正的原图颜色。
            wire [9:0] raw_sum = {2'd0, raw_r} + {1'd0, raw_g, 1'b0} + {2'd0, raw_b};
            wire [7:0] raw_lum = raw_sum[9:2];          // (R + 2G + B) / 4
            wire       pen_gray = (mode_cur == 3'd3) && (pen_color == 9'd0);
            wire [7:0] mix_raw_r = (pen_gray && !use_raw) ? raw_lum : raw_r;
            wire [7:0] mix_raw_g = (pen_gray && !use_raw) ? raw_lum : raw_g;
            wire [7:0] mix_raw_b = (pen_gray && !use_raw) ? raw_lum : raw_b;

            wire [7:0] base_r = use_raw ? raw_r : filt_r;
            wire [7:0] base_g = use_raw ? raw_g : filt_g;
            wire [7:0] base_b = use_raw ? raw_b : filt_b;

            // 强度混合（原图与滤镜之间）
            // ★ 2026-10-25：恢复普通交叉淡入，保留白纸和彩铅质感。
            //   减法叠加虽然不会提亮，但会把彩铅的"纸白 + 层涂"效果洗掉，
            //   黑白素描也会变成"原图 + 暗线"，不是用户要的铅笔稿。
            wire signed [10:0] dr = $signed({3'd0, base_r}) - $signed({3'd0, mix_raw_r});
            wire signed [10:0] dg = $signed({3'd0, base_g}) - $signed({3'd0, mix_raw_g});
            wire signed [10:0] db = $signed({3'd0, base_b}) - $signed({3'd0, mix_raw_b});

            wire signed [18:0] mr = dr * $signed({1'b0, str_q7});
            wire signed [18:0] mg = dg * $signed({1'b0, str_q7});
            wire signed [18:0] mb = db * $signed({1'b0, str_q7});

            wire signed [10:0] out_r = $signed({3'd0, mix_raw_r}) + (mr >>> 7);
            wire signed [10:0] out_g = $signed({3'd0, mix_raw_g}) + (mg >>> 7);
            wire signed [10:0] out_b = $signed({3'd0, mix_raw_b}) + (mb >>> 7);

            assign mix_rgb[q*24+16 +: 8] = clamp8(out_r);
            assign mix_rgb[q*24+8  +: 8] = clamp8(out_g);
            assign mix_rgb[q*24    +: 8] = clamp8(out_b);
        end
    endgenerate

    //==========================================================================
    // 8) 输出 FIFO：滤镜域 -> 相机域
    //==========================================================================
    wire [FW-1:0] r_dout;
    wire          r_valid, r_empty, r_full;

    sfifo #(
        .DATA_WIDTH_W   (FW      ),
        .DATA_WIDTH_R   (FW      ),
        .ADDR_WIDTH_W   (FIFO_AW ),
        .ADDR_WIDTH_R   (FIFO_AW ),
        .AL_FULL_NUM    (480     ),
        .AL_EMPTY_NUM   (2       ),
        .SHOW_AHEAD_EN  (1'b1    ),
        .OUTREG_EN      ("NOREG" )
    ) u_out_fifo (
        .rst        ( ~I_rst_n                    ),
        .clkw       ( I_clk_flt                   ),
        .we         ( fin_vld                     ),
        .di         ( {fin_usr, fin_lst, mix_rgb} ),
        .full_flag  ( r_full                      ),
        .afull      (                             ),
        .clkr       ( I_clk                       ),
        .re         ( ~r_empty                    ),
        .dout       ( r_dout                      ),
        .valid      ( r_valid                     ),
        .empty_flag ( r_empty                     ),
        .aempty     (                             ),
        .wrusedw    (                             ),
        .rdusedw    (                             )
    );

    // ★ 必须用 ~r_empty 门住全部输出（空时 dout 是旧值，会冒出假 tuser/tlast）
    assign O_tvalid = ~r_empty;
    assign O_tdata  = r_dout[95:0];
    assign O_tlast  = ~r_empty & r_dout[96];
    assign O_tuser  = ~r_empty & r_dout[97];

    wire unused_ok = |{w_full, r_full, w_valid, r_valid, com_lst, com_usr,
                       raw_vld, raw_lst, raw_usr, pen_pout[1], emb_pout[1], f_hph};

    //==========================================================================
    // 8) 帧率统计（滤镜域 75.002MHz）
    //
    //    数 1 秒内过了几个帧首（f_usr）。27bit 计数器足够（1.34 亿）。
    //    测的是**实际流过滤镜的帧率**——FIFO 溢出丢帧会直接反映出来，
    //    正是 OSD 上想看的那个数。
    //    O_fps 每秒才变一次，是慢变量，跨到 150MHz 只需要两级同步。
    //==========================================================================
    localparam integer FPS_CLK_HZ = 75002000;      // PLL C1 = 75.002 MHz

    reg [26:0] fps_sec;
    reg [7:0]  fps_cnt;

    always @(posedge I_clk_flt or negedge I_rst_n) begin
        if (!I_rst_n) begin
            fps_sec <= 27'd0;
            fps_cnt <= 8'd0;
            O_fps   <= 8'd0;
        end
        else if (fps_sec == FPS_CLK_HZ - 1) begin
            fps_sec <= 27'd0;
            fps_cnt <= 8'd0;
            O_fps   <= fps_cnt;
        end
        else begin
            fps_sec <= fps_sec + 27'd1;
            if (f_vld && f_usr) fps_cnt <= fps_cnt + 8'd1;
        end
    end

endmodule
