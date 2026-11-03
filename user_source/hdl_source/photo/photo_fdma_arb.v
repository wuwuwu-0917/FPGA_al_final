`timescale 1ns/1ps
//-----------------------------------------------------------------------------
// photo_fdma_arb —— ui_dbuf（显示帧缓存）与拍照通道分时复用同一个 FDMA 用户口
//
//   ub（I_ub_*）─┐                        ┌─ FDMA 写口（O_w*, I_wbusy/I_wvalid）
//                ├─ 仲裁（读写独立）──────┤
//   ph（I_ph_*）─┘                        └─ FDMA 读口（O_r*, I_rbusy/I_rvalid）
//
// 真实 uiFDMA.v 的 user 侧时序事实（本模块的全部约束都来自这几条）：
//   1) wareq/rareq 是电平；fdma_wstart = (locked==0 && wareq)，采样后**下一拍**
//      busy 拉高；busy 覆盖整个 burst，最后一拍之后才撤；
//   2) O_fdma_wvalid/rvalid 是每个 256bit 拍的脉冲，uidbuf 直接把它当 FIFO 的
//      pop / write 使能；
//   3) I_fdma_wready/rready 在 uifdma_axi_ddr 内部固定为 1，**不能反压** ——
//      任何一拍漏掉/多算都无重传；
//   4) ★ uidbuf 的 FDMA 状态机只看 busy、不看 valid（uidbuf.v:323-346/469-490）：
//      写 FSM 等 busy=1 才认为请求被接收，等 busy=0 才自增地址/计数。所以把
//      共享的 busy 直接给它，拍照传输期间它会误判「我的请求被接收」→ 整帧错位。
//
// 本模块的做法：
//   * **掩码 busy**：每个客户端只看到属于自己的那次传输的 busy
//       O_ub_wbusy = I_wbusy & ~sel_w，O_ph_wbusy = I_wbusy & sel_w（读侧同）。
//     未被选中的客户端 busy 恒 0 → 状态机停在原地、wareq/rareq 保持、不丢请求。
//   * **掩码 valid**：O_ub_wvalid = I_wvalid & ~sel_w（读侧同）。uidbuf 只用它
//     当 FIFO 的 pop 使能，屏蔽是安全的；但**在某个客户端自己的 burst 期间
//     绝不能屏蔽**（FDMA 内部 ready 恒 1，屏蔽 1 拍就丢 1 拍且无重传）—— 所以
//     sel 只允许在 I_wbusy/I_rbusy 为 0 时改变（见下）。
//   * **切换协议**（读写各自独立）：sel 在 I_clk 域打拍；只在 busy==0 时进入
//     SWITCH。关键点是**决定切换的那一拍就已经把要送给 FDMA 的 req 压 0**
//     —— 由 w_present/r_present 里的 `want==sel` 条件保证；w_dec/r_dec 只是
//     同义的观测信号，专门留给 TB 断言用，**改它们不影响功能**。
//     SWITCH 拍再空转 1 拍，之后才把新授权方的 req 放出去。
//     如果只在 SWITCH 拍压 0（决定拍不压），FDMA 会在决定拍按**旧 sel** 的地址
//     采到一个 burst，等 sel 翻过去后 busy/valid 全掩到错误的客户端 → 数据错位。
//   * **一个 burst 的 req 只出现 1 拍**：与真实 fdma_wstart 的采样方式一致
//     （同一个沿采样，1 拍足够）；busy 期间 req 恒 0，避免 locked 一落就
//     立刻起第二个 burst（那样 arbiter 无法逐 burst 记账）。
//   * 写方向：**拍照优先**。预算（Task 11 联仿按这个核）：
//       - dbuf 写 FIFO 实际是 32KB（2048×128bit；wrapper 的 W_BUFDEPTH=1024 会被
//         uidbuf 的 $clog2 翻倍），按 fit 输出 ≈143MB/s 算，从空到满 ≈224µs；
//       - 拍照侧受 capture FIFO（512×256bit）限流：最多攒 ≈5.7 个 90 拍 burst，
//         稳态每行 480 个 128bit 字 = 2.67 个 burst，AXI 排空约为填充的 8 倍，
//         所以它是「一次一个 burst、burst 之间必然有空档」，不会整帧背靠背；
//         让路一次 ≈1 个 burst（0.7µs）+ 2 拍切换，224µs 余量足够。
//       ★ 这条预算**依赖拍照侧的请求形态**：写方向没有任何 dbuf 饥饿保护
//         （按规格如此）。若将来拍照变成背靠背整帧请求，dbuf 写 FIFO 会被冲爆。
//   * 读方向：**dbuf 优先**（dbuf 读 FIFO 正常请求点只剩 ≈4.7µs，每次只偷
//     1 个 burst ≈0.7µs），但拍照侧饥饿保护会强制让一次：实现取
//     STARVE_MAX=15，即最多等 15 个已完成的 dbuf burst（满足「≤16」），
//     防止拍照回读被无限期饿死。
//   * 读写两个方向**完全独立**（FDMA 的 W/R 引擎独立），互不阻塞。
//
// ★ 真实系统里的耦合（Task 11 系统级联仿重点，L0 仿真证明不了）：
//   ui_dbuf 内部的 WR_S（uidbuf.v:582-621）把写/读请求**串行化且写优先** ——
//   拍照写被挂起时，ub 的读请求根本不会拉高，所以「读方向 dbuf 优先」在这些
//   窗口里不起作用。显示读的额外延迟 ≈ 拍照写等待 + 2 拍切换 + 偶发的强制
//   拍照读 burst。必须用真实 uidbuf + 本仲裁器 + 拍照读写联仿，确认显示读
//   FIFO 不欠载。
//
// 复位：I_rst_n 建议直接接 pll_locked；★ **必须与 FDMA 的 M_AXI_ARESETN
//       同源、同释放**（否则仲裁器可能把 FDMA 复位边界上的残留在途 burst 当成
//       自己发出的）。sel/状态机异步复位；复位期间 O_wareq/O_rareq 强制为 0
//       （不把复位中的请求送进 FDMA）。
//
// ★ 留给上层的约束（本模块不检查）：
//   - 客户端的 wareq/rareq 一旦拉高必须保持到 busy 拉高（burst 中途绝不撤销）；
//   - 客户端在 busy 期间不得改变 addr/size/data（本模块直通，不锁存）；
//   - O_wsize/O_rsize 为 0 是非法请求：真实 uiFDMA 在 wsize=0 时 wleft=0、
//     fdma_wend 永远等不到 1，而 AWLEN 仍按 255 发 → FDMA 会**卡死 / 地址跑飞**。
//-----------------------------------------------------------------------------
module photo_fdma_arb (
    input  wire        I_clk,        // O_axi_clk 133MHz
    input  wire        I_rst_n,      // pll_locked
    // ---------------- uidbuf 侧 ----------------
    input  wire [28:0]  I_ub_waddr,
    input  wire         I_ub_wareq,
    input  wire [15:0]  I_ub_wsize,
    input  wire [255:0] I_ub_wdata,
    output wire         O_ub_wbusy,
    output wire         O_ub_wvalid,
    input  wire [28:0]  I_ub_raddr,
    input  wire         I_ub_rareq,
    input  wire [15:0]  I_ub_rsize,
    output wire         O_ub_rbusy,
    output wire         O_ub_rvalid,
    // ---------------- 拍照侧 ----------------
    input  wire [28:0]  I_ph_waddr,
    input  wire         I_ph_wareq,
    input  wire [15:0]  I_ph_wsize,
    input  wire [255:0] I_ph_wdata,
    output wire         O_ph_wbusy,
    output wire         O_ph_wvalid,
    input  wire [28:0]  I_ph_raddr,
    input  wire         I_ph_rareq,
    input  wire [15:0]  I_ph_rsize,
    output wire         O_ph_rbusy,
    output wire         O_ph_rvalid,
    // ---------------- FDMA 侧 ----------------
    // ★ 本模块只 mux **写数据**（O_wdata）。读数据不 mux（没有 O_rdata 端口）：
    //   顶层把 FDMA 的 fdma_rdata 直接广播给两个客户端，各自用**掩码后的
    //   rvalid**当 FIFO 的写使能（photo_readback 的 u_fifo.we 就是这么接的）。
    output wire [28:0]  O_waddr,
    output wire         O_wareq,
    output wire [15:0]  O_wsize,
    output wire [255:0] O_wdata,
    input  wire         I_wbusy,
    input  wire         I_wvalid,
    output wire [28:0]  O_raddr,
    output wire         O_rareq,
    output wire [15:0]  O_rsize,
    input  wire         I_rbusy,
    input  wire         I_rvalid
);
    //-------------------------------------------------------------------------
    // 公共常量
    //-------------------------------------------------------------------------
    // 读方向：连续这么多个 dbuf burst 之后强制让给拍照侧一次。
    // 设计文档写「16 个已完成 burst」，这里取 15 —— 即拍照侧最多等 15 个
    // 完成的 dbuf burst 就让一次，严格满足「≤16」的要求。
    localparam [4:0] STARVE_MAX = 5'd15;
    // 发出 req 后等 busy 的超时（只在请求方已撤销时允许回到 IDLE）
    localparam [4:0] TMO_MAX    = 5'd31;

    localparam [1:0] S_IDLE = 2'd0,
                     S_SW   = 2'd1,
                     S_RUN  = 2'd2;

    //=========================================================================
    // 写方向：拍照优先
    //=========================================================================
    reg [1:0] w_st;
    reg       sel_w;
    reg       bseen_w;      // 本次 burst 期间见过 busy（支持 LAT>1 的行为模型）
    reg [4:0] tmo_w;

    wire ub_wreq  = I_ub_wareq;
    wire ph_wreq  = I_ph_wareq;
    wire sel_wreq = sel_w ? ph_wreq : ub_wreq;

    // 拍照优先：只要拍照侧要就选它；否则 dbuf；都没人要就保持原选择
    wire want_w = ph_wreq ? 1'b1 : (ub_wreq ? 1'b0 : sel_w);

    // 决定切换的那一拍（**只给 TB 观测**，见文件头）：真正保证这一拍 req=0 的
    // 是下面 w_present 里的 `want_w == sel_w`；改 w_dec 不影响功能。
    wire w_dec = I_rst_n && (w_st == S_IDLE) && (want_w != sel_w);
    // 只有「IDLE + FDMA 空闲 + 已选定」的这一拍才把 req 送出去
    wire w_present = I_rst_n && (w_st == S_IDLE) && (want_w == sel_w) && !I_wbusy;

    wire sw_w  = (w_st == S_SW);
    wire act_w = (w_st == S_RUN);

    assign O_waddr = sel_w ? I_ph_waddr : I_ub_waddr;
    assign O_wsize = sel_w ? I_ph_wsize : I_ub_wsize;
    assign O_wdata = sel_w ? I_ph_wdata : I_ub_wdata;
    assign O_wareq = (w_present && sel_wreq) ? 1'b1 : 1'b0;

    // 掩码：未选中方看到的 busy 恒 0、valid 恒 0
    assign O_ub_wbusy  = I_wbusy  & ~sel_w;
    assign O_ph_wbusy  = I_wbusy  &  sel_w;
    assign O_ub_wvalid = I_wvalid & ~sel_w;
    assign O_ph_wvalid = I_wvalid &  sel_w;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            w_st    <= S_IDLE;
            sel_w   <= 1'b0;        // 复位后默认授权 dbuf
            bseen_w <= 1'b0;
            tmo_w   <= 5'd0;
        end else begin
            case (w_st)
            //-------------------------------------------------------------
            S_IDLE: begin
                if (I_wbusy) begin
                    // FDMA 还在忙（只可能来自复位边界或异常）：不动 sel，等它落下
                    bseen_w <= 1'b1;
                end else if (want_w != sel_w) begin
                    // 本拍 req 已被 w_present/w_dec 压 0，FDMA 采不到任何东西；
                    // 下一拍进 SWITCH 再空转 1 拍，然后才放开新授权
                    sel_w   <= want_w;
                    bseen_w <= 1'b0;
                    tmo_w   <= 5'd0;
                    w_st    <= S_SW;
                end else if (sel_wreq) begin
                    // 本拍 O_wareq=1，FDMA 在这个沿采样 → 下一拍 busy 拉高
                    bseen_w <= 1'b0;
                    tmo_w   <= 5'd0;
                    w_st    <= S_RUN;
                end
            end
            //-------------------------------------------------------------
            S_SW: begin
                w_st <= S_IDLE;         // 1 拍空转，req 恒 0
            end
            //-------------------------------------------------------------
            S_RUN: begin
                if (I_wbusy) begin
                    // busy 覆盖整个 burst；它落下的那一拍 = 最后一拍已被 FDMA 接收
                    bseen_w <= 1'b1;
                    tmo_w   <= 5'd0;
                end else if (bseen_w) begin
                    bseen_w <= 1'b0;
                    w_st    <= S_IDLE;  // burst 正常结束
                end else begin
                    // 还没见过 busy：行为模型 LAT>1 时多等几拍（不改变 LAT=1 时序）。
                    // 兜底：请求方已撤销且 busy 一直不来 → 没有在途 burst，可安全
                    // 回 IDLE；仍在请求时绝不回 IDLE（否则一换 sel 就可能撞上迟到
                    // 的 busy，把别人的 burst 判到自己头上）。
                    if (tmo_w != TMO_MAX) tmo_w <= tmo_w + 5'd1;
                    // ★ 2026-11-03：原来是 (tmo_w == TMO_MAX-1) —— 只有那一拍照逸。
                    //   若那一拍请求方还在请求，tmo_w 就饱和在 TMO_MAX，
                    //   外层条件 tmo_w != TMO_MAX 从此为假，无法再进入该分支，
                    //   状态机将停留在 S_RUN 无法返回。
                    //   改成 >= ，饱和之后只要请求方撤销仍能回 IDLE。
                    if ((tmo_w >= TMO_MAX - 5'd1) && !sel_wreq) begin
                        tmo_w <= 5'd0;
                        w_st  <= S_IDLE;
                    end
                end
            end
            //-------------------------------------------------------------
            default: w_st <= S_IDLE;
            endcase
        end
    end

    //=========================================================================
    // 读方向：dbuf 优先 + 拍照侧饥饿保护
    //=========================================================================
    reg [1:0] r_st;
    reg       sel_r;
    reg       bseen_r;
    reg [4:0] tmo_r;
    reg [4:0] starve_r;     // 已完成、且拍照侧仍在等的 dbuf burst 数

    wire ub_rreq  = I_ub_rareq;
    wire ph_rreq  = I_ph_rareq;
    wire sel_rreq = sel_r ? ph_rreq : ub_rreq;

    wire force_r = (starve_r >= STARVE_MAX) && ph_rreq;
    wire want_r  = force_r ? 1'b1 :
                   (ub_rreq ? 1'b0 : (ph_rreq ? 1'b1 : sel_r));

    // 只给 TB 观测（原因同 w_dec：真正压 req 的是 r_present 的 want_r==sel_r）
    wire r_dec     = I_rst_n && (r_st == S_IDLE) && (want_r != sel_r);
    wire r_present = I_rst_n && (r_st == S_IDLE) && (want_r == sel_r) && !I_rbusy;

    wire sw_r  = (r_st == S_SW);
    wire act_r = (r_st == S_RUN);

    assign O_raddr = sel_r ? I_ph_raddr : I_ub_raddr;
    assign O_rsize = sel_r ? I_ph_rsize : I_ub_rsize;
    assign O_rareq = (r_present && sel_rreq) ? 1'b1 : 1'b0;

    assign O_ub_rbusy  = I_rbusy & ~sel_r;
    assign O_ph_rbusy  = I_rbusy &  sel_r;
    assign O_ub_rvalid = I_rvalid & ~sel_r;
    assign O_ph_rvalid = I_rvalid &  sel_r;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            r_st     <= S_IDLE;
            sel_r    <= 1'b0;       // 复位后默认授权 dbuf
            bseen_r  <= 1'b0;
            tmo_r    <= 5'd0;
            starve_r <= 5'd0;
        end else begin
            case (r_st)
            //-------------------------------------------------------------
            S_IDLE: begin
                if (I_rbusy) begin
                    bseen_r <= 1'b1;
                end else if (want_r != sel_r) begin
                    // ★ 这里**不能**清 starve_r：清掉之后下一拍 ub 的优先级会立刻
                    //   把刚让出去的授权抢回去（ub_rreq && sel_r → 切回 dbuf），
                    //   拍照侧永远等不到。饥饿计数只在「真的服务完拍照侧」时清。
                    sel_r   <= want_r;
                    bseen_r <= 1'b0;
                    tmo_r   <= 5'd0;
                    r_st    <= S_SW;
                end else if (sel_rreq) begin
                    bseen_r <= 1'b0;
                    tmo_r   <= 5'd0;
                    r_st    <= S_RUN;
                end
            end
            //-------------------------------------------------------------
            S_SW: begin
                r_st <= S_IDLE;
            end
            //-------------------------------------------------------------
            S_RUN: begin
                if (I_rbusy) begin
                    bseen_r <= 1'b1;
                    tmo_r   <= 5'd0;
                end else if (bseen_r) begin
                    bseen_r <= 1'b0;
                    r_st    <= S_IDLE;
                    // 饥饿计数：攒满 STARVE_MAX 个 dbuf burst 就强制让一次
                    if (sel_r) begin
                        starve_r <= 5'd0;           // 刚服务完拍照侧
                    end else if (ph_rreq) begin
                        starve_r <= (starve_r >= STARVE_MAX) ? STARVE_MAX
                                                             : (starve_r + 1'b1);
                    end else begin
                        starve_r <= 5'd0;           // 拍照侧没在等，不用记账
                    end
                end else begin
                    if (tmo_r != TMO_MAX) tmo_r <= tmo_r + 5'd1;
                    // ★ 2026-11-03：同写方向，改成 >= 才能从饱和状态逃逸。
                    if ((tmo_r >= TMO_MAX - 5'd1) && !sel_rreq) begin
                        tmo_r <= 5'd0;
                        r_st  <= S_IDLE;
                    end
                end
            end
            //-------------------------------------------------------------
            default: r_st <= S_IDLE;
            endcase
        end
    end

endmodule
