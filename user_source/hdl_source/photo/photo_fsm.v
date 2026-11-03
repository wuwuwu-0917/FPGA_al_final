`timescale 1ns/1ps
//=============================================================================
//  photo_fsm.v  ---  双键拍照总状态机（全部在 25MHz 域）
//
//  流程（docs/superpowers/specs/2026-10-26-photo-capture-sd-design.md）：
//
//    S_BOOT       等 DDR PLL 锁定（同步后 2 拍）
//    S_SD_INIT    I_init_start 保持**电平 1**，等 O_init_done；
//                 O_init_error 或 2s 超时 -> 拉低 >=2 拍再拉高重试，
//                 3 次失败 -> S_ERROR
//    S_IDLE       等 UI 的 toggle 请求（3FF + 边沿），且必须 O_axi_idle
//    S_CAPTURE    等 photo_capture 的 **dma_done**（不是 frame_done）；
//                 溢出帧永远不翻 dma_done，靠 I_cap_overflow_tgl 判失败
//    S_WRITE 的失败判据：块 busy 600ms、写卡进度 2s（每块 commit 清零）、总时长 30s
//    S_WRITE_WAIT 等 32 拍 -> pack_start 单拍 + 翻 O_rd_start_tgl
//    S_WRITE      回读 -> photo_pack -> sd_blk_buf -> sd_ctrl 单块写循环
//    S_DONE       1s 后回 S_IDLE
//    S_ERROR      2s 后回 S_IDLE
//
//  块状态机（只在 S_WRITE 内跑）：
//    B_WAIT       缓冲里有整块（>=256 字）且还没写满 BLOCKS_PER_PHOTO
//                 -> blk_start / wr_start_r 各 1 拍
//    B_START      清脉冲（★ blk_start 必须单拍：见 sd_blk_buf 头注释，
//                 保持高会让每块 256 次请求都读同一个字）
//    B_BUSY_RISE  等 O_wr_busy（2ms 超时 = 启动没被接受）
//    B_BUSY_FALL  等 O_wr_busy 落：
//                   O_wr_error=0 -> blk_commit + blk++
//                   O_wr_error=1 -> blk_rewind 重发同一块（rd_ptr 不动，
//                                   块缓冲保证不会读到下一块）
//                 600ms 超时按失败处理（> sd_write 自己的 500ms 卡 busy 超时）
//
//  ★ 块缓冲（sd_blk_buf）的重置策略 —— 头注释里那条 "I_blk_start 必须单拍"
//    只是问题的一半，另一半是**失败后的残留**：
//    sd_blk_buf 没有 flush 口，而拍照失败（抓帧溢出 / 回读超时 / PLL 失锁 /
//    3 次块重试用尽）时 packer 可能还在吐上一张的旧数据，缓冲里也会剩 0~512 个
//    旧字。下一张照片的 pack_start 只重置 packer 自己，不碰块缓冲 —— 直接开写
//    就会把旧数据当成新 BMP 的头几个字（整张错位）。
//    这里不做 flush 协议，而是**除 S_WRITE 之外把 sd_blk_buf 一直按在复位里**：
//    packer 的旧数据全部被吃掉；S_WRITE 第 1 拍（pack_start 生效那拍）仍按住，
//    第 2 拍才放开 —— 这样 pack_start 生效沿上 packer 可能吐出的那一个残字
//    （它的 word_idx 还没更新）也被复位吃掉，新照片的第 0 个字正好落在 ram[0]。
//    失败拍照期间 sd_ctrl 残留的那一块会往目标 LBA 写垃圾，但该槽位号不会前进，
//    下一张成功时会整段覆盖，该槽位在下次成功覆盖前可能是花图（失败拍照的副作用，可接受）。
//
//  ★ pack_start 与 O_rd_start_tgl 同拍（S_WRITE_WAIT 计满 32 拍）：
//    packer 要先发 27 个 BMP 头字才开始要源字（25MHz 下 1.08us），远长于
//    photo_readback 的 flush（7 个 133MHz 拍 ≈53ns），所以两者同拍安全。
//
//  ★ 复位树：本模块用 S_board_reset_n（25MHz 域，与 UI/SD 同源）；
//    photo_capture / photo_readback 的 AXI 侧用 S_pll_lock & pll_locked & S_board_reset_n。
//    pll_locked 在本模块里 2FF 同步后参与：req_sync 清零、16 拍失锁保护、
//    S_IDLE 的启动条件。两边复位不同源时靠这三条兜底。
//=============================================================================
module photo_fsm #(
    // ★ 唯一随卡变化的常量。32'd34944 由 sim/tools/sd_prep.py 在真卡上算出：
    //   分区起始 2048 + 数据区 (2348 + 2×15210) + (首簇 6 - 2)×32 = 34944
    //   8 个槽位占 LBA 34944..133247（每槽 12288 扇区 = 6MiB）。
    //   **换卡必须重跑 sd_prep.py 重新填**；填 0 会写到分区表上。
    //   tb_photo_fsm 显式传 0，不受这个默认值影响。
    parameter [31:0]  PHOTO_BASE_LBA    = 32'd34944,
    parameter integer SLOT_SECTORS      = 12288,     // 每个槽位 6MB
    parameter integer BLOCKS_PER_PHOTO  = 12151      // 12151*256*2B = 6,221,312B
)(
    input  wire         I_clk,                  // 25MHz
    input  wire         I_rst_n,                // S_board_reset_n
    input  wire         I_pll_locked,           // DDR PLL lock（异步）
    input  wire         I_capture_req_tgl,      // UI 150MHz 域的 toggle
    // ---------------- 回读数据流（25MHz 域）------------------
    input  wire         I_src_valid,
    input  wire [127:0] I_src_data,
    output wire         O_src_ready,
    // ---------------- 抓帧 / 回读控制 ----------------
    output reg          O_cap_start_tgl,
    input  wire         I_cap_dma_done_tgl,     // AXI 域 toggle
    input  wire         I_cap_overflow_tgl,     // 相机域 toggle
    input  wire         I_cap_axi_idle,         // AXI 域电平
    output reg          O_rd_start_tgl,
    // ---------------- SD（SPI 模式）------------------
    input  wire         I_sd_miso,
    output wire         O_sd_clk,
    output wire         O_sd_cs,
    output wire         O_sd_mosi,
    // ---------------- 提示 ----------------
    output wire         O_led,                  // 高有效
    output wire [3:0]   O_status                // one-hot: idle/busy/done/error（寄存输出）
);
    //=========================================================================
    // ★ PHOTO_BASE_LBA 是模块参数，顶层不覆盖 —— 单点真值就在上面那行的默认值。
    //   已用 sim/tools/sd_prep.py 在真卡上算出 32'd34944（8 个槽位连续且首尾
    //   相接，两路解析结果一致）。换一张卡必须重跑脚本重新填。
    //   绝对不能留 0 上板：那会从 LBA 0 起写，覆盖分区表/引导区。
    //=========================================================================

    //=========================================================================
    // 状态编码 / 常数
    //=========================================================================
    localparam [3:0] S_BOOT       = 4'd0,
                     S_SD_INIT    = 4'd1,
                     S_IDLE       = 4'd2,
                     S_CAPTURE    = 4'd3,
                     S_WRITE_WAIT = 4'd4,
                     S_WRITE      = 4'd5,
                     S_DONE       = 4'd6,
                     S_ERROR      = 4'd7;

    localparam [1:0] B_WAIT       = 2'd0,
                     B_START      = 2'd1,
                     B_BUSY_RISE  = 2'd2,
                     B_BUSY_FALL  = 2'd3;

    // O_status：one-hot
    localparam [3:0] ST_IDLE = 4'b0001,
                     ST_BUSY = 4'b0010,
                     ST_DONE = 4'b0100,
                     ST_ERR  = 4'b1000;

    // 25MHz 节拍超时（拍数 = 秒 x 25e6）
    localparam [31:0] T_INIT      = 32'd50_000_000;    // 2 s    SD 初始化
    localparam [31:0] T_CAPTURE   = 32'd7_500_000;     // 300 ms 抓帧
    localparam [31:0] T_PROGRESS  = 32'd50_000_000;    // 2 s    写卡进度看门狗（每块 commit 清零）
    localparam [31:0] T_WRITE_ALL = 32'd750_000_000;   // 30 s   写卡总时长
    localparam [31:0] T_BLK_BUSY  = 32'd15_000_000;    // 600 ms 块 busy
    localparam [31:0] T_BLK_RISE  = 32'd50_000;        // 2 ms   busy 上升
    localparam [31:0] T_DONE      = 32'd25_000_000;    // 1 s    done 显示
    localparam [31:0] T_ERROR     = 32'd50_000_000;    // 2 s    error 显示
    localparam [31:0] T_RD_WAIT   = 32'd32;            // S_WRITE_WAIT 的 32 拍

    // 失败次数上限：SD 初始化 3 次失败；单块写最多重发 3 次（设计文档 §6.3）
    localparam [2:0] SD_FAIL_MAX  = 3'd2;
    localparam [2:0] BLK_RETRY_MAX= 3'd3;

    //=========================================================================
    // 内部网
    //=========================================================================
    wire        pack_wvalid;
    wire [15:0] pack_word;
    wire [9:0]  blk_count;          // 缓冲里可读字数（0..512）
    wire        blk_full;
    wire [15:0] sd_word;
    wire        sd_init_done;
    wire        sd_init_error;
    wire        O_wr_req;           // sd_ctrl 的取数请求
    wire        O_wr_busy;          // sd_ctrl 整块忙
    wire        O_wr_error;         // sd_ctrl 本块失败（下一块启动才清）

    reg         pack_start;         // 单拍：启动 BMP 打包
    reg         init_start_r;       // ★ 电平：初始化期间保持 1
    reg         blk_start;          // 单拍
    reg         blk_commit;         // 单拍
    reg         blk_rewind;         // 单拍
    reg         wr_start_r;         // 单拍：启动一块 SD 写
    reg         blk_hold_rst;       // 块缓冲复位保持（见头注释）

    // 状态机寄存器（先声明，下面例化/同步器都要用）
    reg [3:0]  state;
    reg [1:0]  bstate;
    reg [2:0]  slot;                // 0..7 循环
    reg [15:0] blk;                 // 本张已写块数
    reg [31:0] lba;                 // 当前块绝对扇区
    reg [2:0]  retry;               // 失败计数（SD 初始化 / 单块写共用）
    reg [31:0] timer;               // 状态用计时器（25MHz 拍）
    reg [31:0] wr_timer;            // 块 busy 计时器
    reg [1:0]  boot_cnt;
    reg [3:0]  init_low_cnt;        // I_init_start 拉低的拍数

    //=========================================================================
    // 例化：BMP 打包器 / 512x16 块缓冲 / SD 控制器
    //=========================================================================
    photo_pack #(
        .N_SRC     (518400),        // 480x1080 个 128bit 字
        .PAD_WORDS (229)            // 12151*512/2 - 27 - 518400*6 = 229
    ) u_pack (
        .I_clk        (I_clk),
        .I_rst_n      (I_rst_n),
        .I_start      (pack_start),
        .I_src_valid  (I_src_valid),
        .I_src_data   (I_src_data),
        .O_src_ready  (O_src_ready),
        .O_word_valid (pack_wvalid),
        .O_word       (pack_word),
        .I_word_ready (~blk_full),
        .O_done       ()
    );

    // 512x16 块缓冲：成功 commit 前进 256 字，失败 rewind 重发同一块。
    // 复位由 blk_hold_rst 拉长（见文件头），单独复位是为了丢掉失败拍照的残留。
    sd_blk_buf u_blk (
        .I_clk        (I_clk),
        .I_rst_n      (I_rst_n & ~blk_hold_rst),
        .I_we         (pack_wvalid & ~blk_full),
        .I_din        (pack_word),
        .I_blk_start  (blk_start),
        .I_blk_commit (blk_commit),
        .I_blk_rewind (blk_rewind),
        .O_count      (blk_count),
        .O_full       (blk_full),
        .I_wr_req     (O_wr_req),
        .O_wr_data    (sd_word)
    );

    sd_ctrl #(
        .DIV_FREQ (100)             // 250kHz 初始化时钟
    ) u_sd (
        .I_clk        (I_clk),
        .I_rst_n      (I_rst_n),
        .I_init_start (init_start_r),   // ★ 电平，不是单拍
        .O_init_done  (sd_init_done),
        .O_init_error (sd_init_error),
        .I_wr_start   (wr_start_r),
        .I_wr_lba     (lba),
        .I_wr_data    (sd_word),
        .O_wr_req     (O_wr_req),
        .O_wr_busy    (O_wr_busy),
        .O_wr_error   (O_wr_error),
        .I_sd_miso    (I_sd_miso),
        .O_sd_clk     (O_sd_clk),
        .O_sd_cs      (O_sd_cs),
        .O_sd_mosi    (O_sd_mosi)
    );

    //=========================================================================
    // CDC：pll_locked 同步 + 请求 / toggle 边沿检测
    //=========================================================================
    reg [1:0] pll_locked_sync;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) pll_locked_sync <= 2'b00;
        else          pll_locked_sync <= {pll_locked_sync[0], I_pll_locked};
    end
    wire pll_locked_ok = pll_locked_sync[1];

    // UI 的 O_capture_req 是 150MHz 域的 toggle（电平翻转 = 一次请求）。
    // ★ 2FF 会把「UI 复位时 toggle 从 1 掉回 0」也识别成一次翻转，所以
    //   !pll_locked（同步后）或 S_BOOT 期间强行把 req_sync 清 0 吞掉它；
    //   而且只在 S_IDLE 接受 req_pulse。
    reg [2:0] req_sync;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n || !pll_locked_ok || (state == S_BOOT))
            req_sync <= 3'b000;
        else
            req_sync <= {req_sync[1:0], I_capture_req_tgl};
    end
    wire req_pulse = req_sync[2] ^ req_sync[1];

    // 运行中掉锁再锁回：若电平本来就是 1（上次请求还在），2FF 的 0->1 会伪造一次翻转。
    // 掉锁期间记下当时的电平，恢复后先吞掉它，直到电平真的回到 0 才重新武装。
    reg req_suppress;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)                req_suppress <= 1'b0;
        else if (!pll_locked_ok)     req_suppress <= I_capture_req_tgl;
        else if (!I_capture_req_tgl) req_suppress <= 1'b0;
    end
    wire req_pulse_ok = req_pulse && !req_suppress;

    // 抓帧完成 / 溢出 / 回读完成：三个 toggle 各自 3FF + 边沿
    reg [2:0] capdma_sync;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) capdma_sync <= 3'b000;
        else          capdma_sync <= {capdma_sync[1:0], I_cap_dma_done_tgl};
    end
    wire capdma_pulse = capdma_sync[2] ^ capdma_sync[1];

    reg [2:0] ovf_sync;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) ovf_sync <= 3'b000;
        else          ovf_sync <= {ovf_sync[1:0], I_cap_overflow_tgl};
    end
    wire ovf_pulse = ovf_sync[2] ^ ovf_sync[1];

    // 回读完成（I_rd_done_tgl）不再参与判据：完成条件是块数；读侧是否活着
    // 由下面的写卡进度看门狗判断。

    // AXI 侧静默标志：2FF + 连续 4 拍为 1 才认（源是 AXI 域的组合表达式，
    // 单拍假 idle 跨到 25MHz 域有被采到的风险；4 拍过滤 1~2 拍毛刺）
    reg [1:0] axi_idle_sync;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) axi_idle_sync <= 2'b00;
        else          axi_idle_sync <= {axi_idle_sync[0], I_cap_axi_idle};
    end
    reg [1:0] axi_idle_cnt;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)                axi_idle_cnt <= 2'd0;
        else if (!axi_idle_sync[1])  axi_idle_cnt <= 2'd0;
        else if (axi_idle_cnt != 2'd3) axi_idle_cnt <= axi_idle_cnt + 2'd1;
    end
    wire axi_idle_ok = (axi_idle_cnt == 2'd3);

    // DDR PLL 失锁保护：同步后连续 16 拍为 0 -> S_ERROR。
    // S_BOOT 里本来就在等锁，不计数；S_ERROR 里不重复触发（让 2s 计时走完）。
    reg [4:0] lock_lost_cnt;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)                                          lock_lost_cnt <= 5'd0;
        else if (pll_locked_ok || (state == S_BOOT) ||
                 (state == S_ERROR))                           lock_lost_cnt <= 5'd0;
        else if (lock_lost_cnt != 5'd16)                       lock_lost_cnt <= lock_lost_cnt + 5'd1;
    end
    wire lock_lost = (lock_lost_cnt == 5'd16);

    //=========================================================================
    // LED 闪烁节拍（自由运行）
    //   5Hz：100ms 半周期 = 2,500,000 拍
    //   1Hz：500ms 半周期 = 12,500,000 拍
    //=========================================================================
    reg [21:0] blink5_cnt;
    reg        blink5_r;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            blink5_cnt <= 22'd0;
            blink5_r   <= 1'b0;
        end
        else if (blink5_cnt == 22'd2_499_999) begin
            blink5_cnt <= 22'd0;
            blink5_r   <= ~blink5_r;
        end
        else begin
            blink5_cnt <= blink5_cnt + 22'd1;
        end
    end

    reg [23:0] blink1_cnt;
    reg        blink1_r;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            blink1_cnt <= 24'd0;
            blink1_r   <= 1'b0;
        end
        else if (blink1_cnt == 24'd12_499_999) begin
            blink1_cnt <= 24'd0;
            blink1_r   <= ~blink1_r;
        end
        else begin
            blink1_cnt <= blink1_cnt + 24'd1;
        end
    end

    //=========================================================================
    // 槽位基址：slot 只有 3bit，用常量表代替 32bit 乘法器
    //   （TD 里 32x32 乘法器很贵，而 slot*SLOT_SECTORS 只是 8 个常数）
    //=========================================================================
    reg [31:0] slot_base;
    always @(*) begin
        case (slot)
            3'd0:    slot_base = PHOTO_BASE_LBA;
            3'd1:    slot_base = PHOTO_BASE_LBA + SLOT_SECTORS;
            3'd2:    slot_base = PHOTO_BASE_LBA + 2*SLOT_SECTORS;
            3'd3:    slot_base = PHOTO_BASE_LBA + 3*SLOT_SECTORS;
            3'd4:    slot_base = PHOTO_BASE_LBA + 4*SLOT_SECTORS;
            3'd5:    slot_base = PHOTO_BASE_LBA + 5*SLOT_SECTORS;
            3'd6:    slot_base = PHOTO_BASE_LBA + 6*SLOT_SECTORS;
            default: slot_base = PHOTO_BASE_LBA + 7*SLOT_SECTORS;
        endcase
    end

    //=========================================================================
    // 主状态机 + 块状态机（同一个 always，所有赋值顺序可控）
    //  （寄存器已在文件上半部分统一声明，见 143~153 行）
    //=========================================================================
    // 写卡进度看门狗：每块 commit 清零；2s 没任何块完成 -> 读侧或卡挂了。
    // ★ 不能用「进入 S_WRITE 后 5s 必须看到 rd_done」：整条 DDR→SD 管线只能缓冲
    //   约 17KB（读 FIFO 16KB + 块缓冲 1.4KB），rd_done 必然在整张快写完时才到，
    //   那样会把"卡慢（规格 3~8s/张）"误判成"读侧挂死"。
    reg [31:0] prog_timer;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)              prog_timer <= 32'd0;
        else if (state != S_WRITE) prog_timer <= 32'd0;
        else if (blk_commit)       prog_timer <= 32'd0;
        else if (prog_timer != 32'hFFFF_FFFF) prog_timer <= prog_timer + 32'd1;
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            state           <= S_BOOT;
            bstate          <= B_WAIT;
            slot            <= 3'd0;
            blk             <= 16'd0;
            lba             <= 32'd0;
            retry           <= 3'd0;
            timer           <= 32'd0;
            wr_timer        <= 32'd0;
            boot_cnt        <= 2'd0;
            init_low_cnt    <= 4'd0;
            init_start_r    <= 1'b0;
            pack_start      <= 1'b0;
            blk_start       <= 1'b0;
            blk_commit      <= 1'b0;
            blk_rewind      <= 1'b0;
            wr_start_r      <= 1'b0;
            blk_hold_rst    <= 1'b1;
            O_cap_start_tgl <= 1'b0;
            O_rd_start_tgl  <= 1'b0;
        end
        else begin
            // ---- 脉冲默认单拍 ----
            pack_start <= 1'b0;
            blk_start  <= 1'b0;
            blk_commit <= 1'b0;
            blk_rewind <= 1'b0;
            wr_start_r <= 1'b0;

            // ---- 状态计时器：默认 +1，任何状态切换都显式清 0（写在后面覆盖）----
            timer <= timer + 32'd1;

            // ---- 块缓冲只在 S_WRITE 里放开（含 pack_start 那一拍仍按住）----
            if (state == S_WRITE) blk_hold_rst <= 1'b0;
            else                  blk_hold_rst <= 1'b1;

            // ---- 全局：DDR PLL 失锁 ----
            if (lock_lost && (state != S_BOOT) && (state != S_ERROR)) begin
                state    <= S_ERROR;
                timer    <= 32'd0;
                wr_timer <= 32'd0;
                bstate   <= B_WAIT;
                retry    <= 3'd0;
            end
            else case (state)

            //-----------------------------------------------------------------
            S_BOOT: begin
                // pll_locked 已经过 2FF，这里再稳 2 拍才认
                if (pll_locked_ok) boot_cnt <= boot_cnt + 2'd1;
                else               boot_cnt <= 2'd0;

                if (boot_cnt == 2'd1) begin
                    state        <= S_SD_INIT;
                    timer        <= 32'd0;
                    retry        <= 3'd0;
                    init_low_cnt <= 4'd8;       // 先按住低 9 拍，让 sd_init 从 idle 起
                    init_start_r <= 1'b0;
                    boot_cnt     <= 2'd0;
                end
            end

            //-----------------------------------------------------------------
            S_SD_INIT: begin
                if (init_low_cnt != 4'd0) begin
                    // 拉低 >=2 拍（这里 9 拍）：sd_init 把它当异步复位
                    init_start_r <= 1'b0;
                    init_low_cnt <= init_low_cnt - 4'd1;
                    timer        <= 32'd0;
                end
                else begin
                    init_start_r <= 1'b1;       // ★ 电平：初始化期间保持 1
                    if (sd_init_done) begin
                        state <= S_IDLE;
                        timer <= 32'd0;
                        retry <= 3'd0;
                    end
                    else if (sd_init_error || (timer >= T_INIT)) begin
                        if (retry >= SD_FAIL_MAX) begin
                            // 3 次失败：报错。S_ERROR 2s 后回 S_IDLE，
                            // 那里发现 sd_init_done=0 会再发起一轮（每 2s 一轮）
                            state <= S_ERROR;
                            timer <= 32'd0;
                            retry <= 3'd0;
                        end
                        else begin
                            retry        <= retry + 1'b1;
                            init_low_cnt <= 4'd8;
                            init_start_r <= 1'b0;
                            timer        <= 32'd0;
                        end
                    end
                end
            end

            //-----------------------------------------------------------------
            S_IDLE: begin
                if (!sd_init_done) begin
                    // SD 还没初始化好（可能上一轮 3 次都失败进了 S_ERROR）：
                    // 再发起一轮，别把设备锁死在 S_IDLE
                    state        <= S_SD_INIT;
                    timer        <= 32'd0;
                    retry        <= 3'd0;
                    init_low_cnt <= 4'd8;
                    init_start_r <= 1'b0;
                end
                else if (req_pulse_ok && pll_locked_ok && axi_idle_ok && !O_wr_busy) begin
                    // ★ 必须等 AXI 静默：上一次溢出的残留 burst 会和新帧错位
                    O_cap_start_tgl <= ~O_cap_start_tgl;
                    blk             <= 16'd0;
                    retry           <= 3'd0;
                    bstate          <= B_WAIT;
                            state           <= S_CAPTURE;
                    timer           <= 32'd0;
                end
            end

            //-----------------------------------------------------------------
            S_CAPTURE: begin
                if (ovf_pulse && pll_locked_ok) begin
                    // 溢出帧永远不翻 dma_done，只能靠这个判失败
                    state <= S_ERROR;
                    timer <= 32'd0;
                    retry <= 3'd0;
                end
                else if (capdma_pulse && pll_locked_ok) begin
                    // ★ 等的是 dma_done（最后一个 burst 已被 AXI 接收），
                    //   不是 frame_done
                    state <= S_WRITE_WAIT;
                    timer <= 32'd0;
                end
                else if (timer >= T_CAPTURE) begin
                    state <= S_ERROR;
                    timer <= 32'd0;
                    retry <= 3'd0;
                end
            end

            //-----------------------------------------------------------------
            S_WRITE_WAIT: begin
                // 等 32 拍再启动回读/打包（> photo_readback 的 7 拍 flush）
                if (timer >= T_RD_WAIT) begin
                    pack_start     <= 1'b1;             // 单拍（下一拍被默认清 0）
                    O_rd_start_tgl <= ~O_rd_start_tgl;
                    bstate         <= B_WAIT;
                    state          <= S_WRITE;
                    timer          <= 32'd0;
                end
            end

            //-----------------------------------------------------------------
            S_WRITE: begin
                if (blk >= BLOCKS_PER_PHOTO) begin
                    // 全部块写完：成功，槽位前进
                    slot  <= slot + 1'b1;
                    state <= S_DONE;
                    timer <= 32'd0;
                    retry <= 3'd0;
                end
                else if (prog_timer >= T_PROGRESS) begin
                    // 2s 没有任何块完成 -> 读侧或卡挂了
                    state  <= S_ERROR;
                    timer  <= 32'd0;
                    retry  <= 3'd0;
                    bstate <= B_WAIT;
                end
                else if (timer >= T_WRITE_ALL) begin
                    // 30s 还没写完整张（含等数据的时间）
                    state  <= S_ERROR;
                    timer  <= 32'd0;
                    retry  <= 3'd0;
                    bstate <= B_WAIT;
                end
                else begin
                    case (bstate)
                    //---------------------------------------------------------
                    B_WAIT: begin
                        // ★ !blk_commit：commit 那一拍 blk_count 还是提交前的值
                        //   （rd_ptr 下一拍才前进 256），此时判断会误以为还剩一整块
                        if (!blk_commit && !O_wr_busy && (blk_count >= 10'd256) &&
                            (blk < BLOCKS_PER_PHOTO)) begin
                            lba        <= slot_base + blk;
                            blk_start  <= 1'b1;
                            wr_start_r <= 1'b1;
                            bstate     <= B_START;
                        end
                    end
                    //---------------------------------------------------------
                    B_START: begin
                        blk_start  <= 1'b0;      // ★ 单拍，否则每块都读同一个字
                        wr_start_r <= 1'b0;
                        wr_timer   <= 32'd0;
                        bstate     <= B_BUSY_RISE;
                    end
                    //---------------------------------------------------------
                    B_BUSY_RISE: begin
                        if (O_wr_busy) begin
                            wr_timer <= 32'd0;
                            bstate   <= B_BUSY_FALL;
                        end
                        else if (wr_timer >= T_BLK_RISE) begin
                            // 启动没被 sd_write 接受
                            wr_timer <= 32'd0;
                            if (retry >= BLK_RETRY_MAX) begin
                                state  <= S_ERROR;
                                timer  <= 32'd0;
                                retry  <= 3'd0;
                                bstate <= B_WAIT;
                            end
                            else begin
                                retry      <= retry + 1'b1;
                                blk_rewind <= 1'b1;
                                bstate     <= B_WAIT;
                            end
                        end
                        else wr_timer <= wr_timer + 32'd1;
                    end
                    //---------------------------------------------------------
                    B_BUSY_FALL: begin
                        if (!O_wr_busy) begin
                            // busy 落下 = 本块结束；O_wr_error 保持到下一块启动
                            if (O_wr_error) begin
                                wr_timer <= 32'd0;
                                if (retry >= BLK_RETRY_MAX) begin
                                    state  <= S_ERROR;
                                    timer  <= 32'd0;
                                    retry  <= 3'd0;
                                    bstate <= B_WAIT;
                                end
                                else begin
                                    retry      <= retry + 1'b1;
                                    blk_rewind <= 1'b1;   // 同一块重发（rd_ptr 不动）
                                    bstate     <= B_WAIT;
                                end
                            end
                            else begin
                                blk_commit <= 1'b1;       // rd_ptr += 256
                                blk        <= blk + 16'd1;
                                retry      <= 3'd0;
                                wr_timer   <= 32'd0;
                                bstate     <= B_WAIT;
                            end
                        end
                        else if (wr_timer >= T_BLK_BUSY) begin
                            // ★ 600ms 还没落下 = sd_write 真挂了（它自己 500ms 超时都没救回来）。
                            //   绝不能 rewind 重发：busy 还在高，重发会把正在写的块从头上重读，
                            //   随后把旧块的 busy 当成新块成功去 commit，静默写坏一块还计为成功。
                            wr_timer <= 32'd0;
                            state    <= S_ERROR;
                            timer    <= 32'd0;
                            retry    <= 3'd0;
                            bstate   <= B_WAIT;
                        end
                        else wr_timer <= wr_timer + 32'd1;
                    end
                    //---------------------------------------------------------
                    default: bstate <= B_WAIT;
                    endcase
                end
            end

            //-----------------------------------------------------------------
            S_DONE: begin
                if (timer >= T_DONE) begin
                    state <= S_IDLE;
                    timer <= 32'd0;
                end
            end

            //-----------------------------------------------------------------
            S_ERROR: begin
                if (timer >= T_ERROR) begin
                    state <= S_IDLE;
                    timer <= 32'd0;
                    retry <= 3'd0;
                end
            end

            //-----------------------------------------------------------------
            default: begin
                state <= S_ERROR;
                timer <= 32'd0;
            end
            endcase
        end
    end

    //=========================================================================
    // O_status：one-hot（寄存输出）
    //   忙 = BOOT / SD_INIT / CAPTURE / WRITE_WAIT / WRITE
    //
    // ★ 必须**寄存**，不能组合译码：state 是二进制编码，跳变时多位同时变化，
    //   组合 case 的译码输出会出现几 ns 的中间值（default 把它译成 BUSY，
    //   某些跳变还会瞬态命中 DONE/ERR，例如 S_WRITE=0101 -> S_DONE=0110 的
    //   中间可能经过 0111=S_ERROR）。这个信号要跨到 150MHz 显示域给角标用，
    //   2FF 同步器有概率正好采到那个瞬态 → 状态切换那一帧角标闪一个错色点。
    //   寄存后源端就没有多位译码毛刺了。
    //=========================================================================
    // ★ 复位值必须是 ST_BUSY，不是 ST_IDLE：state 复位后是 S_BOOT（不是 S_IDLE），
    //   组合译码走 default 也是 BUSY。如果这里复位成 IDLE，上电/SD 初始化期间
    //   角标会显示"空闲"，而且 tb 的 `wait (status == IDLE)` 会在初始化完成前
    //   就通过（这正是一次真实的回归失败原因）。
    reg [3:0] status_r;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) status_r <= ST_BUSY;
        else case (state)
            S_IDLE:  status_r <= ST_IDLE;
            S_DONE:  status_r <= ST_DONE;
            S_ERROR: status_r <= ST_ERR;
            default: status_r <= ST_BUSY;
        endcase
    end
    assign O_status = status_r;

    //=========================================================================
    // LED（高有效）
    //   S_IDLE  常亮；S_DONE 常亮 1s；
    //   S_CAPTURE / S_WRITE_WAIT / S_WRITE 5Hz 闪；
    //   S_ERROR 1Hz 闪；S_BOOT 灭；S_SD_INIT 正常灭、初始化报错重试时 1Hz 闪
    //=========================================================================
    reg led_r;
    always @(*) begin
        case (state)
            S_IDLE:                          led_r = 1'b1;
            S_DONE:                          led_r = 1'b1;
            S_CAPTURE,
            S_WRITE_WAIT,
            S_WRITE:                         led_r = blink5_r;
            S_SD_INIT:                       led_r = sd_init_error ? blink1_r : 1'b0;
            S_ERROR:                         led_r = blink1_r;
            default:                         led_r = 1'b0;
        endcase
    end
    assign O_led = led_r;

endmodule
