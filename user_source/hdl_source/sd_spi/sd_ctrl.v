//=============================================================================
//  sd_ctrl.v  ---  SD 卡 SPI 控制器顶层（初始化 + 单块写）
//
//  结构：
//    sd_init  : 上电 -> CMD0 -> CMD8 -> CMD55/ACMD41 -> CMD58 -> [CMD16] -> done
//    sd_write : CMD24 单块写（512B），地址换算在 sd_write 内部完成
//
//  ★ I_init_start 契约（电平，不是单拍）：
//      初始化期间必须保持 1；拉低（>=2 个 25MHz 拍，sd_init 内部按异步复位
//      处理）= 中止并复位初始化（回 idle、清 O_init_done/O_init_error）。
//      重试必须先拉低再拉高；初始化成功后拉低会重新走一遍初始化。
//      ★ 写卡期间不要拉低 I_init_start：拉低会清 O_init_done、O_sd_clk 切回
//        250kHz，正在写的块会失败。
//
//  SD 时钟：
//    初始化阶段直接用 sd_init 输出的 250kHz（init_sd_clk = ~div_clk，
//    DIV_FREQ=100 @25MHz），初始化完成后切到 25MHz（~I_clk）。
//    这样分频器只有 sd_init 内部一份，相位 by construction，不重复占 FF。
//
//  用户接口契约：
//    O_wr_req  : 1 拍请求。sd_write 内部在 state2 头字节 bit_cnt==14（请求
//                本块第 1 个字）和 state3 每个字 bit_cnt==14 拉高，2 拍
//                之后才采样 I_wr_data——上游必须用“寄存输出”语义（例如
//                sd_blk_buf 的预取寄存器），show-ahead FIFO 直接接会每块
//                丢第 0 字、整块后移 16bit。每块共 256 个请求。
//    O_wr_busy : 覆盖整个块：命令 + 512B 数据 + CRC + 数据响应 token +
//                卡 busy。busy 落下表示本块结束（错误时提前落下）。
//    O_wr_error: 电平，本块失败（R1!=0 / 坏 token / 超时）。
//                ★ 保持到下一块启动（I_wr_start 被采样）才清：Task 7 的
//                拍照 FSM 应在 O_wr_busy 落下后、发起下一块之前检查它。
//    I_wr_start: 单拍启动，内部与 O_init_done 相与，初始化未完成时忽略；
//                必须在 O_wr_busy=0 时给（busy 期间的启动脉冲会被丢弃）。
//    I_wr_lba  : 写块号 LBA（SDHC/SDXC 直接作为块地址；SDSC 时 sd_write
//                内部换算成字节地址 LBA<<9）。
//
//  复位：sd_ctrl / sd_init / sd_write 共用同一个 I_rst_n（异步、低有效）。
//=============================================================================
module sd_ctrl #(
    parameter integer DIV_FREQ = 100
)(
    input  wire        I_clk,          // 25MHz
    input  wire        I_rst_n,        // 低有效
    input  wire        I_init_start,   // 初始化启动（电平，见上）
    output wire        O_init_done,    // 初始化完成
    output wire        O_init_error,   // 初始化命令超时重试用尽（电平）
    input  wire        I_wr_start,     // 单拍写启动
    input  wire [31:0] I_wr_lba,       // 写块号 LBA
    input  wire [15:0] I_wr_data,      // 写数据
    output wire        O_wr_req,       // 取下一字（1 拍）
    output wire        O_wr_busy,      // 整块写忙
    output wire        O_wr_error,     // 写失败（电平，下一块启动才清）
    input  wire        I_sd_miso,      // SD 卡 MISO
    output wire        O_sd_clk,       // SD 卡 SPI 时钟
    output wire        O_sd_cs,        // SD 卡片选
    output wire        O_sd_mosi       // SD 卡 MOSI
);

//--------------------------------------------------------------------------
// 内部信号
//--------------------------------------------------------------------------
wire       init_sd_clk;      // sd_init 的 250kHz 初始化时钟（= ~div_clk）
wire       init_sd_cs;
wire       init_sd_mosi;
wire       wr_sd_cs;
wire       wr_sd_mosi;
wire       sdhc;             // OCR CCS（未引出，调试可层次化访问）

//--------------------------------------------------------------------------
// SD 时钟：初始化 250kHz（sd_init 内部分频），初始化完成后 25MHz
//--------------------------------------------------------------------------
assign O_sd_clk = O_init_done ? ~I_clk : init_sd_clk;

//--------------------------------------------------------------------------
// CS / MOSI 选择：初始化阶段接 sd_init；初始化完成后接 sd_write；
// 写空闲时 CS=1、MOSI=1
//--------------------------------------------------------------------------
assign O_sd_cs   = (!O_init_done) ? init_sd_cs   :
                   (O_wr_busy)    ? wr_sd_cs     : 1'b1;
assign O_sd_mosi = (!O_init_done) ? init_sd_mosi :
                   (O_wr_busy)    ? wr_sd_mosi   : 1'b1;

//--------------------------------------------------------------------------
// 初始化模块
//--------------------------------------------------------------------------
sd_init #(
    .DIV_FREQ       (DIV_FREQ)
) u_sd_init (
    .clk_ref        (I_clk),
    .rst_n          (I_rst_n),
    .I_start        (I_init_start),

    .sd_miso        (I_sd_miso),
    .sd_clk         (init_sd_clk),
    .sd_cs          (init_sd_cs),
    .sd_mosi        (init_sd_mosi),

    .sd_init_done   (O_init_done),
    .O_sdhc         (sdhc),
    .O_error        (O_init_error)
);

//--------------------------------------------------------------------------
// 写模块（初始化完成前忽略写启动）
//--------------------------------------------------------------------------
sd_write u_sd_write (
    .clk            (I_clk),
    .rst_n          (I_rst_n),

    .sd_miso        (I_sd_miso),
    .sd_cs          (wr_sd_cs),
    .sd_mosi        (wr_sd_mosi),

    .wr_start_en    (I_wr_start & O_init_done),
    .wr_sec_addr    (I_wr_lba),
    .wr_data        (I_wr_data),
    .wr_busy        (O_wr_busy),
    .wr_req         (O_wr_req),

    .I_sdhc         (sdhc),
    .O_error        (O_wr_error),
    .O_r1           ()
);

endmodule
