`timescale 1ns/1ps
//-----------------------------------------------------------------------------
// photo_readback —— 双键拍照：把 DDR 快照区按 burst 读回来，256→128 交给打包器。
//
// 结构：
//   AXI 域 133.33MHz                                   25MHz 域
//   ─────────────────                                  ────────
//   burst 引擎 ──→ FDMA 读口（每次 BURST_WORDS/2 拍 × 256bit）
//        │
//        └──→ sfifo 256→128（16KB）──→ O_src_valid/O_src_data ──→ photo_pack
//
// 硬约束：
//   * 复位释放后**不会**自动读快照：必须收到一次 I_start_tgl 才从 SNAP_BASE 开始
//     读 TOTAL_BURSTS 个 burst（内部 armed 标志，读完自动清）。
//   * 与 ui_dbuf 分时复用 FDMA 读口：每次只发一个 burst、等 I_rbusy 落下再发下一个。
//     O_rareq 一旦发出必须保持到 I_rbusy 拉高，burst 中途绝不撤销（设计文档 §5.5）；
//     本模块的 REQ 状态天然满足。
//   * ★ 复位契约：I_axi_rstn 与 I_sd_rstn 必须**同源、同释放**。本模块的 flush
//     计数器在 AXI 域，fifo_rst 同时吃两个复位；若两者释放不同步，会出现 AXI 侧
//     已经开始搬数、25MHz 侧还在复位（或反过来）的窗口，FIFO 指针/空满标志就不
//     可信。上层复位树必须保证这一点。
//   * O_src_valid / O_src_data 的时序（直连 sfifo 的 show-ahead 读口，
//     SHOW_AHEAD_EN=1、OUTREG_EN="NOREG"）：
//       - O_src_valid = ~empty（组合直出，**不能**再用 I_src_ready 去门控它）；
//       - O_src_valid=1 期间，O_src_data 就是**当前待取的 128bit 字**：
//         低半 = 先写入的（偶数）256bit 字的低半，接着是它的高半，然后下一个
//         256bit 字……即内存里 256bit 字 0 的低半/高半、字 1 的低半/高半……
//       - 消费者在 I_src_ready=1 的那个时钟沿取走一个 128bit 字，FIFO 同沿推进；
//         I_src_ready=0 时 dout 保持不动，不丢字；
//       - O_src_valid=0 时 O_src_data **无定义**（可能是上一张剩下的内容），
//         下游只能在 valid=1 时采样；
//       - 25MHz 侧没有别的逻辑，跨域只有 sfifo 的格雷码指针。
//     ★ start 之后到 fifo_rst 生效之间有 ≤2 个 SD 时钟拍的窗口，O_src_valid 可能
//       还挂着上一次的残留。下游必须在 start 后等 flush 结束（或等 valid 先落再升）
//       才开始消费，否则可能吃掉上一张的最后一个字。
//   * ★ 下一次 start 只能在**上一次回读完全结束**（O_dma_done_tgl 已翻、FDMA 已
//     空闲、O_src_valid 流也被下游取完）之后给。接口没有 idle 输出，靠上层保证。
//     flush 只能覆盖 start 后 8 拍：若在 burst 中途给 start，在途 burst 的后续拍
//     会在 flush_ok 之后落入 FIFO，和新序列混在一起（不做 burst 级中止）。
//     flush 同时会清掉 FIFO 里还没被取走的字，所以必须等下游取完再 start。
//     集成时 photo_pack 的 I_start 与回读 start 同拍也没关系：打包器要先发 27 个
//     BMP 头字才开始要源字，25MHz 下 ≈1.08µs，远长于 start 后 ≤2 个 SD 时钟拍的
//     残留窗口。
//   * O_dma_done_tgl：只表示「最后一个 burst 的最后一拍已经被 AXI 侧（FDMA）接收」，
//     不是 DDR 读完、更不是 SD 写完。翻转时 FIFO 里还可能有若干 128bit 字没被
//     25MHz 侧取走，必须把 O_src_valid 流收完才算整张照片回读完成。
//
// flush guard（沿用 photo_capture / Task 2 已验证的结论）：
//   sfifo 是「异步复位、同步释放」：rst 撤销后内部 wr_rst/rd_rst 还要 2 拍才真正
//   放开。所以 flush 分两段：复位只占 2 拍（flush_cnt=7/6），rst 撤销后再等 6 拍
//   （到 flush_cnt=0 时 flush_ok）才放开请求与写使能。
//   拍数账（flush_cnt=7 版本，start_pulse 后第 1 拍记 c=0；C 用例有对应时序断言）：
//     c=2 fifo_rst 撤销 → c=4 wr_rst 释放（2 拍）→ c=7 flush_ok
//     （flush_ok 比 wr_rst 释放晚 **3 拍**，不是 4 拍）→ c=9 第一个 we。
//     即第一个写使能离 wr_rst 释放有 6 个时钟沿、离 fifo_rst 撤销有 8 个时钟沿。
//   所以可容忍 **≤7 级**同步释放（不是 5 级）：到第 8 级时第一个字能写进 RAM 0 号
//   单元但写指针不推进，第二个字会把它覆盖（整段错位）。不依赖、不假设 sfifo
//   的释放拍数。
//   ★ 真正的失败模式不是「丢一个字」：sfifo 的 wr_en_s 不受 wr_rst 门控
//     （RAM 的 cea 恒为 1、wea=wr_en_s），写指针复位期间 we=1 的每个字都反复写到
//     RAM 的 0/1 号单元、写指针不推进；复位释放后写指针仍从 0 开始 → 这一段字
//     全部丢失、整条流错位（整段静默损坏）。不要退回「flush 4 拍、rst 全程有效」。
//
// ★ 参数约束（不满足会静默丢数据，例化时自己保证）：
//   BURST_WORDS ≥ 2 且必须是偶数（128bit 成对拼 256bit；rsize=0 是非法请求）；
//   BEATS = BURST_WORDS/2 ≤ 96（AL_FULL_NUM=512-96：afull=0 时至少还有 96 个
//   256bit 空位，保证一个 burst 一定装得下 —— 真机 90 拍时改成 512-32 就会在
//   479+90 > 511 处溢出丢字）；
//   TOTAL_BURSTS ≥ 1。
//-----------------------------------------------------------------------------
module photo_readback #(
    parameter integer TOTAL_BURSTS = 2880,
    parameter integer BURST_WORDS  = 180,
    parameter [28:0]  SNAP_BASE    = 29'h0600_0000
)(
    // ---------------- AXI 域（I_axi_clk，133.33MHz）------------------------
    input  wire         I_axi_clk,
    input  wire         I_axi_rstn,     // ★ 必须与 I_sd_rstn 同源、同释放
    input  wire         I_start_tgl,    // toggle（内部 3FF + 边沿）
    output wire [28:0]  O_raddr,
    output wire         O_rareq,
    output wire [15:0]  O_rsize,        // 单位 = 256bit 拍
    input  wire [255:0] I_rdata,
    input  wire         I_rvalid,       // 已掩码，每拍一个 256bit 字
    input  wire         I_rbusy,        // 已掩码
    output reg          O_dma_done_tgl,
    // ---------------- 25MHz 域（I_sd_clk）----------------------------------
    input  wire         I_sd_clk,       // 25MHz
    input  wire         I_sd_rstn,      // ★ 必须与 I_axi_rstn 同源
    output wire         O_src_valid,
    output wire [127:0] O_src_data,
    input  wire         I_src_ready
);
    //-------------------------------------------------------------------------
    // 参数
    //-------------------------------------------------------------------------
    localparam integer BEATS     = BURST_WORDS / 2;             // 256bit 拍 / burst
    localparam integer ADDR_STEP = BURST_WORDS * 16;            // 每 burst 字节步长
    localparam integer CNT_BITS  = $clog2(TOTAL_BURSTS + 1);

    localparam [15:0]         BEATS16  = BEATS;
    localparam [CNT_BITS-1:0] NBURSTS  = TOTAL_BURSTS;
    localparam [28:0]         STEP29   = ADDR_STEP;

    // flush 总长度：2 拍复位 + rst 撤销后 6 拍 guard（flush_cnt 归零才 flush_ok）
    localparam [2:0] FLUSH_LOAD = 3'd7;

    //=========================================================================
    // start toggle 同步到 AXI 域（3FF + 边沿检测）
    //=========================================================================
    reg [2:0] st_sync;
    always @(posedge I_axi_clk or negedge I_axi_rstn)
        if (!I_axi_rstn) st_sync <= 3'b000;
        else             st_sync <= {st_sync[1:0], I_start_tgl};

    wire start_pulse = st_sync[2] ^ st_sync[1];

    //-------------------------------------------------------------------------
    // flush：start 后先把 sfifo 复位掉（丢掉上一次的残留），再留 post-flush guard
    //-------------------------------------------------------------------------
    reg [2:0] flush_cnt;
    always @(posedge I_axi_clk or negedge I_axi_rstn)
        if (!I_axi_rstn)          flush_cnt <= 3'd0;
        else if (start_pulse)     flush_cnt <= FLUSH_LOAD;
        else if (flush_cnt != 0)  flush_cnt <= flush_cnt - 3'd1;

    // ★ 复位只占 2 拍（flush_cnt=7/6），flush_cnt=5 时 fifo_rst 撤销；
    //   flush_ok 到 flush_cnt 归零才放开（start_pulse 后 c=7，比 wr_rst 释放晚 3 拍）。
    //   第一个写使能在 c=9：离 fifo_rst 撤销 8 个时钟沿、离 wr_rst 释放 6 个。
    //   C 用例有对应时序断言（直接测 fifo_rst 撤销 → 第一个 we 的拍数），
    //   把 guard 缩短（FLUSH_LOAD=4、>2）这条断言会立刻 FAIL。
    wire fifo_rst = ~I_axi_rstn | ~I_sd_rstn | (flush_cnt > 3'd5);
    wire flush_ok = (flush_cnt == 3'd0);

    //=========================================================================
    // 跨域 FIFO：256bit（AXI 域）→ 128bit（25MHz 域）
    //   512×256bit = 1024×128bit = 16KB
    //   show-ahead：dout 就是「当前待读的那个 128bit 字」，低半先出。
    //   afull = 已存 ≥ 416 个 256bit 字（还剩 96 个空位），请求条件用它而不是
    //   rdusedw（满时 9bit 计数会回绕）。
    //=========================================================================
    wire         wfull;
    wire         afull;
    wire         empty;
    wire [127:0] dout;

    assign O_src_valid = ~empty;
    assign O_src_data  = dout;

    sfifo #(
        .DATA_WIDTH_W (256),
        .DATA_WIDTH_R (128),
        .ADDR_WIDTH_W (9),          // 512×256b
        .ADDR_WIDTH_R (10),         // 1024×128b
        .AL_FULL_NUM  (512 - 96),
        .AL_EMPTY_NUM (2),
        .SHOW_AHEAD_EN(1'b1),
        .OUTREG_EN    ("NOREG")
    ) u_fifo (
        .rst       (fifo_rst),
        .clkw      (I_axi_clk),
        .we        (I_rvalid & ~wfull & flush_ok),
        .di        (I_rdata),
        .full_flag (wfull),
        .afull     (afull),
        .clkr      (I_sd_clk),
        .re        (I_src_ready & ~empty),
        .dout      (dout),
        .valid     (),
        .empty_flag(empty),
        .aempty    (),
        .wrusedw   (),
        .rdusedw   ()
    );

    //=========================================================================
    // AXI 域：burst 引擎（每次一个 burst，等 busy 落下再发下一个）
    //=========================================================================
    localparam [1:0] R_IDLE = 2'd0,
                     R_REQ  = 2'd1,
                     R_WAIT = 2'd2;

    reg [1:0]          r_st;
    reg [CNT_BITS-1:0] burst_cnt;
    reg [28:0]         addr_r;      // 当前 burst 起始地址（增量维护，不做乘法）
    reg                armed;       // 收到 start 后置 1，最后一个 burst 完成清 0

    assign O_raddr = addr_r;
    assign O_rareq = (r_st == R_REQ);
    assign O_rsize = BEATS16;

    always @(posedge I_axi_clk or negedge I_axi_rstn) begin
        if (!I_axi_rstn) begin
            r_st           <= R_IDLE;
            burst_cnt      <= {CNT_BITS{1'b0}};
            addr_r         <= SNAP_BASE;
            armed          <= 1'b0;
            O_dma_done_tgl <= 1'b0;
        end else if (start_pulse) begin
            // 一次新的回读：burst 计数与地址都从 SNAP_BASE 重新开始（flush 同时展开）
            r_st      <= R_IDLE;
            burst_cnt <= {CNT_BITS{1'b0}};
            addr_r    <= SNAP_BASE;
            armed     <= 1'b1;
        end else begin
            case (r_st)
            // ★ armed 不能省：复位释放后 flush_cnt=0（flush_ok=1），若没有 armed，
            //   引擎会在收到 start 之前就把整块快照读进 FIFO（tb 实测：dut_fl
            //   复位后立刻读完 2 个 burst，第一次 start 又读一遍 → 请求数翻倍）。
            //   复位后不自读的前提是 I_axi_rstn/I_sd_rstn 同源同释放；若 AXI 侧
            //   单独掉电，复位释放可能被 3FF 识别成一次 start —— 靠 flush 和上层
            //   16 拍失锁规则兜底。
            // flush_ok 保证 FIFO 已复位完；!afull 保证 burst 装得下；
            // !I_rbusy 保证上一拍（或上一次）burst 已经结束。
            R_IDLE: if (armed && flush_ok && !I_rbusy && !afull &&
                        (burst_cnt < NBURSTS))
                        r_st <= R_REQ;

            // 保持 O_rareq=1 直到 FDMA 拉 busy（协议要求，中途不撤）
            R_REQ:  if (I_rbusy)
                        r_st <= R_WAIT;

            // busy 持续整个 burst；它落下的那一拍 = 最后一拍已被 AXI 接收
            R_WAIT: if (!I_rbusy) begin
                        burst_cnt <= burst_cnt + 1'b1;
                        // 注意：最后一个 burst 完成后地址还会再加一次，停在
                        // SNAP_BASE + TOTAL_BURSTS*ADDR_STEP（不是基址）。地址只
                        // 用于 burst 请求，下一次 start 会重新装载，不影响功能。
                        addr_r    <= addr_r + STEP29;
                        if (burst_cnt == (NBURSTS - 1'b1)) begin
                            O_dma_done_tgl <= ~O_dma_done_tgl;
                            armed          <= 1'b0;   // 整块读完，等下一次 start
                        end
                        r_st <= R_IDLE;
                    end

            default: r_st <= R_IDLE;
            endcase
        end
    end

endmodule
