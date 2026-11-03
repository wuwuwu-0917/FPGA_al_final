`timescale 1ns/1ps
//-----------------------------------------------------------------------------
// photo_capture —— 双键拍照：旁路 ISP 128bit 字流，抓一整帧写进 DDR 快照区。
//
// 结构（docs/superpowers/specs/2026-10-26-photo-capture-sd-design.md §3/§5）：
//
//   相机域 74.25MHz                                AXI 域 133MHz
//   ─────────────────                              ──────────────
//   I_isp_* ──┬──→ sfifo 128→256 (16KB) ──→ burst 引擎 ──→ FDMA 写口
//             │        （异步 FIFO）          （每次 BURST_WORDS/2 拍 × 256bit）
//             └── 帧首 tuser 对齐 / 字数统计 / FIFO 满检测
//
// 硬约束：
//   * 相机链只有 valid 没有 ready —— 本模块绝不反压。
//   * 一帧 = TOTAL_WORDS 个 128bit 字，正好
//     N_BURSTS = TOTAL_WORDS/BURST_WORDS 次 FDMA。要求整除，且 BURST_WORDS 为偶数。
//   * ★ 复位契约：I_cap_rstn 与 I_axi_rstn 必须**同源、同释放**。本模块的
//     flush_ok/start_word 只看 I_cap_rstn（fifo_rst 也吃 I_axi_rstn），若将来
//     两个复位独立释放，必须先把 I_axi_rstn 同步进相机域再参与 flush_ok，
//     否则可能在 AXI 侧还在复位时就开始抓帧。
//
// ★ 溢出的语义（上层必须按这个来判）：
//   FIFO 满时**写侧立即停止**、O_frame_done_tgl 不翻、O_overflow_tgl 翻一次；
//   但 FIFO 里已经收下的那部分字仍会被 AXI 引擎继续搬进快照区
//   （即快照区内容作废，DDR 里可能留下半帧）。所以：
//     - 判成功必须看 O_dma_done_tgl（最后一个 burst 已写进 DDR，而不是 frame_done）；
//       溢出帧永远不翻 dma_done，判失败看 O_overflow_tgl；
//     - 下一次给 I_start_tgl 之前必须先确认 O_axi_idle==1。O_axi_idle 为 1
//       表示 AXI 侧既没有在途 burst、也不会再发起新的 burst；上层在它保持
//       为 1 时才可以 start，否则 flush 会打断正在搬的那次 burst。
//
// ★ O_dma_done_tgl 只表示「最后一个 burst 的最后一拍已被 AXI 接收」，
//   不表示 DDR 写入完成（写响应/AXI 侧还会晚几拍）。回读是整帧抓完后才做，
//   所以作为「可以开始回读」的提示是够用的。
//
// 与最初参考实现的关键修正（均有 tb 覆盖）：
//   1) 帧首那个 tuser 字本身必须写进 FIFO：只把 in_frame 当写使能会让
//      「起帧那一拍」的字被丢掉，而 wcnt 照加 —— 整帧少一个字，DMA 永远
//      凑不满最后一个 burst。这里用 start_word 把起帧拍也算进写使能。
//   2) 抓帧起点必须等 FIFO flush 结束（flush_ok），否则 start 后复位
//      FIFO 的几拍里来的 tuser 会让帧错位。
//   3) sfifo 是「异步复位、同步释放」：rst 撤销后内部 wr_rst/rd_rst 还要
//      若干拍才真正放开（当前 RTL 2 拍）。所以 flush 分成两段：
//      复位只占 2 拍，rst 撤销后再等 6 拍 flush_ok 才放开写使能（post-flush
//      guard）。这样**不依赖 sfifo 的释放拍数**：复位撤销到 flush_ok 之间
//      有 6 拍，当前释放只用 2 拍，余 4 拍 —— 最多可容忍 5 级同步释放。
//   4) axi_start 后压 AXI_HOLD 拍再允许发 burst：上一帧若溢出放弃，
//      rdusedw 在 flush 后还要几拍才归零，不压这几拍会拿旧计数抢发一个
//      空 burst，把新帧的第一段数据挤到 +BURST_WORDS*16 去。
//-----------------------------------------------------------------------------
module photo_capture #(
    parameter integer TOTAL_WORDS = 518400,        // 一帧 128bit 字数（480×1080）
    parameter integer BURST_WORDS = 180,           // 每次 FDMA 搬的 128bit 字数（偶数）
    parameter [28:0]  SNAP_BASE   = 29'h0600_0000, // DDR 快照区基地址
    // axi_start 后压住的 AXI 拍数。约束：
    //     AXI_HOLD + 3 > 3 * F_axi / F_cap + 2
    // 当前 74.25MHz(cap) / 133.33MHz(axi) 下右边≈7.4，AXI_HOLD=8 余量≈3.6 拍。
    // ★ 换时钟比必须重新算这个数（cap 越慢、axi 越快，需要的拍数越多）。
    parameter integer AXI_HOLD    = 8
)(
    // ---------------- 相机域（S_hs_rx_clk，74.25MHz）------------------
    input  wire         I_cap_clk,
    input  wire         I_cap_rstn,           // ★ 必须与 I_axi_rstn 同源、同释放
    input  wire         I_start_tgl,          // 25MHz 域的 toggle
    input  wire         I_isp_tuser,          // 帧首第 1 字
    input  wire         I_isp_tvalid,
    input  wire [127:0] I_isp_tdata,
    output reg          O_frame_done_tgl,     // 抓满一帧（toggle）
    output reg          O_overflow_tgl,       // FIFO 溢出、本帧判废（toggle）
    // ---------------- AXI 域（O_axi_clk，133MHz）---------------------
    input  wire         I_axi_clk,
    input  wire         I_axi_rstn,           // ★ 必须与 I_cap_rstn 同源、同释放
    output wire [28:0]  O_waddr,
    output wire         O_wareq,
    output wire [15:0]  O_wsize,              // 单位 = 256bit 拍
    output wire [255:0] O_wdata,
    input  wire         I_wbusy,              // 已掩码
    input  wire         I_wvalid,             // 已掩码，每拍一个 256bit 字
    output reg          O_dma_done_tgl,       // 最后一个 burst 的最后一拍已被 AXI 接收（toggle）
    // O_axi_idle = 1 表示 AXI 侧既没有在途 burst、也不会再发起新的 burst；
    // 上层在它保持为 1 时才可以给 I_start_tgl。
    output wire         O_axi_idle,
    // 抓帧进度观测：已发出的 burst 数 ÷16，供在线调试使用
    output wire [7:0]   O_dbg_burst
);
    //-------------------------------------------------------------------------
    // 参数
    //-------------------------------------------------------------------------
    localparam integer BEATS     = BURST_WORDS / 2;             // 256bit 拍 / burst
    localparam integer N_BURSTS  = TOTAL_WORDS / BURST_WORDS;   // 一帧的 burst 数
    localparam integer ADDR_STEP = BURST_WORDS * 16;            // 每 burst 字节步长

    localparam [15:0] BEATS16  = BEATS;
    localparam [15:0] NBURST16 = N_BURSTS;
    localparam [28:0] STEP29   = ADDR_STEP;
    // 8bit 计数器：AXI_HOLD 必须 ≤ 255，超过会被静默截断
    localparam [7:0]  HOLD_C   = AXI_HOLD;

    // sfifo：1024×128b = 512×256b = 16KB
    localparam integer FIFO_AW  = 10;
    localparam integer FIFO_AR  = 9;

    // flush 总长度：2 拍复位 + rst 撤销后 6 拍 guard（第 7 拍才 flush_ok）
    localparam [2:0] FLUSH_LOAD = 3'd7;

    // ★ 参数约束（不满足会静默丢数据，例化时自己保证）：
    //   BURST_WORDS 必须是偶数（128→256 要成对读）；
    //   TOTAL_WORDS 必须能被 BURST_WORDS 整除（否则最后一个 burst 不满）；
    //   BEATS <= 511（sfifo 的 rdusedw 只有 9bit）。

    //=========================================================================
    // 相机域：start toggle 同步 + 单拍边沿
    //=========================================================================
    reg [2:0] st_sync;
    always @(posedge I_cap_clk or negedge I_cap_rstn)
        if (!I_cap_rstn) st_sync <= 3'b000;
        else             st_sync <= {st_sync[1:0], I_start_tgl};

    wire start_pulse = st_sync[2] ^ st_sync[1];

    //-------------------------------------------------------------------------
    // flush：start 后先把 sfifo 复位掉（丢掉上一帧残留），再留一段 guard
    //-------------------------------------------------------------------------
    reg [2:0] flush_cnt;
    always @(posedge I_cap_clk or negedge I_cap_rstn)
        if (!I_cap_rstn)          flush_cnt <= 3'd0;
        else if (start_pulse)     flush_cnt <= FLUSH_LOAD;
        else if (flush_cnt != 0)  flush_cnt <= flush_cnt - 3'd1;

    // ★ 复位只占 2 拍（flush_cnt=7/6），rst 在 flush_cnt 变成 5 时撤销；
    //   但 flush_ok 要等到 flush_cnt 归零（rst 撤销后第 6 拍）才放开。
    //   sfifo 是「异步置位、同步释放」：rst 撤销后内部 wr_rst/rd_rst 还要若干拍
    //   才真正放开（当前 RTL 是 2 拍）。这段 guard 让「第一个可写拍」离复位撤销
    //   有 4 拍余量，不依赖、也不假设 sfifo 的释放拍数（可容忍 ≤5 级释放）。
    wire fifo_rst = ~I_cap_rstn | ~I_axi_rstn | (flush_cnt > 3'd5);
    wire flush_ok = (flush_cnt == 3'd0);   // rst 撤销后第 6 拍才放开

    //-------------------------------------------------------------------------
    // 帧控制 + 字数统计
    //-------------------------------------------------------------------------
    reg        armed;          // 已收到 start，等下一个 tuser
    reg        in_frame;       // 正在抓
    reg [31:0] wcnt;           // 本帧已抓字数

    wire wfull;

    // 起帧拍：tuser 这一拍本身也要写进 FIFO（写使能里带 start_word）
    wire start_word = armed & I_isp_tvalid & I_isp_tuser & ~in_frame & flush_ok;
    wire fifo_we    = I_isp_tvalid & (in_frame | start_word) & ~wfull;

    always @(posedge I_cap_clk or negedge I_cap_rstn) begin
        if (!I_cap_rstn) begin
            armed            <= 1'b0;
            in_frame         <= 1'b0;
            wcnt             <= 32'd0;
            O_frame_done_tgl <= 1'b0;
            O_overflow_tgl   <= 1'b0;
        end else if (start_pulse) begin
            // 新的一次拍照：重新等帧首（flush 已同时展开）
            armed    <= 1'b1;
            in_frame <= 1'b0;
            wcnt     <= 32'd0;
        end else if (!in_frame) begin
            if (start_word) begin
                if (wfull) begin
                    // 理论上 flush 后不可能满；真满了就按溢出处理，绝不抓半帧
                    armed          <= 1'b0;
                    O_overflow_tgl <= ~O_overflow_tgl;
                end else begin
                    in_frame <= 1'b1;
                    wcnt     <= 32'd1;       // 这一拍的字已经写进 FIFO
                end
            end
        end else if (I_isp_tvalid) begin
            if (wfull) begin
                // FIFO 满：写侧立刻停，本帧判废（FIFO 里已有的残留仍会被搬走）
                in_frame       <= 1'b0;
                armed          <= 1'b0;
                O_overflow_tgl <= ~O_overflow_tgl;
            end else begin
                wcnt <= wcnt + 32'd1;
                if (wcnt == (TOTAL_WORDS - 1)) begin
                    in_frame         <= 1'b0;
                    armed            <= 1'b0;
                    O_frame_done_tgl <= ~O_frame_done_tgl;
                end
            end
        end
    end

    //=========================================================================
    // 跨域 FIFO：128bit（相机域）→ 256bit（AXI 域）
    //   show-ahead：dout 就是「当前待读的那个 256bit 字」，低半 = 先写入的
    //   （偶数）128bit 字，高半 = 后写入的（奇数）字。
    //   afull/aempty 不用（溢出判定用 full_flag），AL_FULL_NUM 保持默认。
    //=========================================================================
    wire [8:0]   rdusedw;          // 可读的 256bit 字数
    wire [255:0] fdout;
    wire         fpop;

    sfifo #(
        .DATA_WIDTH_W (128),
        .DATA_WIDTH_R (256),
        .ADDR_WIDTH_W (FIFO_AW),   // 1024×128b
        .ADDR_WIDTH_R (FIFO_AR),   // 512×256b
        .SHOW_AHEAD_EN(1'b1),
        .OUTREG_EN    ("NOREG")
    ) u_fifo (
        .rst        (fifo_rst),
        .clkw       (I_cap_clk),
        .we         (fifo_we),
        .di         (I_isp_tdata),
        .full_flag  (wfull),
        .afull      (),
        .clkr       (I_axi_clk),
        .re         (fpop),
        .dout       (fdout),
        .valid      (),
        .empty_flag (),
        .aempty     (),
        .wrusedw    (),
        .rdusedw    (rdusedw)
    );

    //=========================================================================
    // AXI 域：burst 引擎
    //=========================================================================
    reg [2:0] a_st_sync;
    always @(posedge I_axi_clk or negedge I_axi_rstn)
        if (!I_axi_rstn) a_st_sync <= 3'b000;
        else             a_st_sync <= {a_st_sync[1:0], I_start_tgl};

    wire axi_start = a_st_sync[2] ^ a_st_sync[1];

    // start 后压几拍：等 flush 把 FIFO 两侧指针和 rdusedw 都清干净
    reg [7:0] hold_cnt;
    always @(posedge I_axi_clk or negedge I_axi_rstn)
        if (!I_axi_rstn)         hold_cnt <= HOLD_C;
        else if (axi_start)      hold_cnt <= HOLD_C;
        else if (hold_cnt != 0)  hold_cnt <= hold_cnt - 8'd1;

    localparam [1:0] W_IDLE = 2'd0,
                     W_REQ  = 2'd1,
                     W_WAIT = 2'd2;

    reg [15:0] burst_cnt;
    reg [1:0]  wst;
    reg [28:0] addr_r;             // 当前 burst 的起始地址（增量维护，不做乘法）

    assign O_wareq = (wst == W_REQ);
    assign O_wsize = BEATS16;
    assign O_wdata = fdout;

    // 只有被仲裁器选中且 busy 期间 wvalid 才为 1；FIFO 已按 burst 长度准备好数据
    assign fpop    = I_wvalid;

    assign O_waddr = addr_r;
    assign O_dbg_burst = burst_cnt[15:4];   // 抓帧进度观测

    //=========================================================================
    // ★ 2026-11-03 保底：W_REQ 超时重发
    //   O_wareq 经仲裁器送出后为一拍宽的脉冲。若该拍未被 FDMA 接收，
    //   请求将无法得到响应，而 W_REQ 需等待 I_wbusy 才能推进。
    //   这里设置 64 拍超时：超时则撤销请求、返回 W_IDLE 并自动重发，
    //   保证抓帧通路在总线竞争下仍能完成整帧传输。
    //   正常情况 busy 在 1~2 拍内返回，本逻辑不会被触发。
    //=========================================================================
    localparam [7:0] WRQ_TMO = 8'd64;
    reg [7:0] wrq_tmo;   //synthesis keep
    always @(posedge I_axi_clk or negedge I_axi_rstn)
        if (!I_axi_rstn)              wrq_tmo <= 8'd0;
        else if (axi_start)           wrq_tmo <= 8'd0;
        else if (wst != W_REQ)        wrq_tmo <= 8'd0;
        else if (wrq_tmo != WRQ_TMO)  wrq_tmo <= wrq_tmo + 8'd1;


    // AXI 侧静默：没有在途 burst，也**不会再发起新的 burst**。
    // 后面两个条件不能省：W_IDLE 里只要 rdusedw 还够、burst 还没发完，下一拍
    // 就会进 W_REQ（残留排空期间这种 1 拍窗口出现过 254 次）。上层只有看到
    // 它稳定为 1 时才允许给 I_start_tgl，否则 flush 会撞上刚要发的 burst。
    assign O_axi_idle = (wst == W_IDLE) && !I_wbusy && (hold_cnt == 0) &&
                        ((rdusedw < BEATS16) || (burst_cnt >= NBURST16));

    always @(posedge I_axi_clk or negedge I_axi_rstn) begin
        if (!I_axi_rstn) begin
            burst_cnt      <= 16'd0;
            wst            <= W_IDLE;
            addr_r         <= SNAP_BASE;
            O_dma_done_tgl <= 1'b0;
        end else if (axi_start) begin
            burst_cnt      <= 16'd0;
            wst            <= W_IDLE;
            addr_r         <= SNAP_BASE;
        end else begin
            case (wst)
            W_IDLE: if ((hold_cnt == 0) && !I_wbusy &&
                        (rdusedw >= BEATS16) && (burst_cnt < NBURST16))
                        wst <= W_REQ;

            W_REQ:  if (I_wbusy)                 // FDMA 已锁存地址/长度
                        wst <= W_WAIT;
                    else if (wrq_tmo >= WRQ_TMO) // ★ 请求丢失：撤请求重发
                        wst <= W_IDLE;

            W_WAIT: if (!I_wbusy) begin          // 本 burst 搬完
                        burst_cnt <= burst_cnt + 16'd1;
                        addr_r    <= addr_r + STEP29;
                        if (burst_cnt == (NBURST16 - 16'd1))
                            O_dma_done_tgl <= ~O_dma_done_tgl;
                        wst <= W_IDLE;
                    end

            default: wst <= W_IDLE;
            endcase
        end
    end

endmodule
