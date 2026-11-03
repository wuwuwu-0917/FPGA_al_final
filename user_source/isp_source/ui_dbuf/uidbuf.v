/*******************************MILIANKE*******************************
*Company : MiLianKe Electronic Technology Co., Ltd.
*WebSite:https://www.milianke.com
*TechWeb:https://www.uisrc.com
*tmall-shop:https://milianke.tmall.com
*jd-shop:https://milianke.jd.com
*taobao-shop1: https://milianke.taobao.com
*Create Date: 2021/10/15
*File Name: uidbuf.v
*Description: 
*Declaration:
*The reference demo provided by Milianke is only used for learning. 
*We cannot ensure that the demo itself is free of bugs, so users 
*should be responsible for the technical problems and consequences
*caused by the use of their own products.
*Copyright: Copyright (c) MiLianKe
*All rights reserved.
*Revision: 
*--2.0 20211015
*--3.0 20220310
*--3.1 20220910
*--4.0 20250610
*--4.1 20250713
*--4.2 20250827
*Signal description
*1) I_ input
*2) O_ output
*3) IO_ input output
*4) S_ system internal signal
*5) _n activ low
*6) _dg debug signal 
*7) _r delay or register
*8) _s state mechine
*********************************************************************/

/*********uidbuf(fdma data buffer controller)基于FDMA信号时序的缓存控制器***********
--以下是米联客设计的uidbuf(fdma data buffer controller)基于FDMA信号时序的缓存控制器
--1.代码简洁，占用极少逻辑资源，代码结构清晰，逻辑设计严谨
--2.读写通道独立，FIFO大小，数据位宽可以灵活设置，适合用于基于RGB时序的视频数据或者数据流传输
--3.从native接口改为支持stream接口协议
--4.FIFO复位同步机制修改，当检测到tuser，并且FIFO非空的情况，需要复位FIFO，进行同步。
--5.增加了读写通道的总裁避免过长时间占用AXI总线，导致读或者写通道无法获取到访问权限
--4.0.改进了tuser,empty,同步以tusert进行，同步优化设计不需要wtlast信号,等信号跨时钟域处理
--4.2.增加VS同步功能，读通道可以用VS同步或者不用VS同步，可以对数据量的情况无需产生tuser，改进了写通道的数据同步机制
*********************************************************************/

module ui_dbuf#(
parameter  integer                      AXI_DATA_WIDTH = 128	, //AXI总线数据位宽
parameter  integer                      AXI_ADDR_WIDTH = 32		, //AXI总线地址位宽

parameter  integer                      ENABLE_VSYNC   = 1		,

parameter  integer                      W_BUFDEPTH     = 2048	, //写通道AXI设置FIFO缓存大小
parameter  integer                      W_DATAWIDTH    = 32		, //写通道AXI设置数据位宽大小
parameter  [AXI_ADDR_WIDTH -1'b1: 0]    W_BASEADDR     = 0		, //写通道设置内存起始地址
parameter  integer                      W_DSIZEBITS    = 24		, //写通道设置缓存数据的增量地址大小，用于FDMA DBUF 计算帧缓存起始地址
parameter  integer                      W_XSIZE        = 1920	, //写通道设置X方向的数据大小，代表了每次FDMA 传输的数据长度
parameter  integer                      W_XSTRIDE      = 1920	, //写通道设置X方向的Stride值，主要用于图形缓存应用
parameter  integer                      W_YSIZE        = 1080	, //写通道设置Y方向值，代表了进行了多少次XSIZE传输
parameter  integer                      W_XDIV         = 2		, //写通道对X方向数据拆分为XDIV次传输，减少FIFO的使用
parameter  integer                      W_BUFSIZE      = 3		, //写通道设置帧缓存大小，目前最大支持128帧，可以修改参数支持更缓存数

parameter  integer                      R_BUFDEPTH     = 2048	, //读通道AXI设置FIFO缓存大小
parameter  integer                      R_DATAWIDTH    = 32		, //读通道AXI设置数据位宽大小
parameter  [AXI_ADDR_WIDTH -1'b1: 0]    R_BASEADDR     = 0		, //读通道设置内存起始地址
parameter  integer                      R_DSIZEBITS    = 24		, //读通道设置缓存数据的增量地址大小，用于FDMA DBUF 计算帧缓存起始地址
parameter  integer                      R_XSIZE        = 1920	, //读通道设置X方向的数据大小，代表了每次FDMA 传输的数据长度
parameter  integer                      R_XSTRIDE      = 1920	, //读通道设置X方向的Stride值，主要用于图形缓存应用
parameter  integer                      R_YSIZE        = 1080	, //读通道设置Y方向值，代表了进行了多少次XSIZE传输
parameter  integer                      R_XDIV         = 2		, //读通道对X方向数据拆分为XDIV次传输，减少FIFO的使用
parameter  integer                      R_BUFSIZE      = 3 		  //读通道设置帧缓存大小，目前最大支持128帧，可以修改参数支持更缓存数
) 
(
input   wire                            I_ui_clk		,//和FDMA AXI总线时钟一致
input   wire                            I_ui_rstn		,//和FDMA AXI复位一致
//sensor input -W_FIFO--------------	
input   wire                            I_W_en			,
input   wire                            I_W_wclk		, 
input   wire                            I_W_tuser		,  
input   wire                            I_W_tvalid		,//数据有效使能
input   wire [W_DATAWIDTH -1 : 0]       I_W_tdata		,//用户写数据
input   wire                            I_W_tlast		,
output  wire                            O_W_tready		,
output  reg  [7 : 0]                    O_W_sync_cnt	,//写通道BUF帧同步输出
input   wire [7 : 0]                    I_W_buf			,// 写通道BUF帧同步输入
// 写 FIFO 水位监测：统计写 FIFO 接近满时的写入次数。
// 本模块 O_W_tready 恒为 1（上游无反压能力），该计数用于观测写链路压力，
// 只做统计，不改变任何行为。
output  reg  [15:0]                     O_wdrop_cnt	,


//----------fdma signals write-------       
output  wire [AXI_ADDR_WIDTH- 1:0]      O_fdma_waddr	,//FDMA写通道地址
output  wire                            O_fdma_wareq	,//FDMA写通道请求
output  wire [15  :0]                   O_fdma_wsize	,//FDMA写通道一次FDMA的传输大小                                 
input   wire                            I_fdma_wbusy	,//synthesis keep//FDMA处于BUSY状态，AXI总线正在写操作  
output  wire [AXI_DATA_WIDTH - 1:0]     O_fdma_wdata	,//FDMA写数据
input   wire                            I_fdma_wvalid	,//FDMA 写有效
output  wire                            O_fdma_wready	,//FDMA写准备好，用户可以写数据
output  reg [7 : 0]                     O_fdma_wbuf		,//FDMA的写帧缓存号输出
output  wire                            O_fdma_wirq		,//FDMA一次写完成的数据传输完成后，产生中断。   
//----------fdma signals read-------  
input   wire                            I_R_en			, 
input   wire                            I_Rsync_en		,//功能预留，尚未开发
input   wire                            I_R_rclk		,  
input   wire                            I_R_tready		,  
output  wire                            O_R_tuser		,
output  wire                            O_R_tvalid		,
output  wire [R_DATAWIDTH- 1 : 0]       O_R_tdata		,//用户读数据
output  wire                            O_R_vrst		,
output  wire                            O_R_tlast		,  
output  reg  [7 : 0] 					O_R_sync_cnt 	,//读通道BUF帧同步输出
input   wire [7 : 0] 					I_R_buf			,//写通道BUF帧同步输入

output  wire [AXI_ADDR_WIDTH-1'b1: 0]   O_fdma_raddr	, // FDMA读通道地址
output  wire                            O_fdma_rareq	, // FDMA读通道请求
output  wire [15: 0]                    O_fdma_rsize	, // FDMA读通道一次FDMA的传输大小                                     
input   wire                            I_fdma_rbusy	, // FDMA处于BUSY状态，AXI总线正在读操作     
input   wire [AXI_DATA_WIDTH-1'b1:0]    I_fdma_rdata	, // FDMA读数据
input   wire                            I_fdma_rvalid	, // FDMA 读有效
output  wire                            O_fdma_rready	, // FDMA读准备好，用户可以读数据
output  reg  [7  :0]                    O_fdma_rbuf 	, // FDMA的读帧缓存号输出
output  wire                            O_fdma_rirq 	  // FDMA一次读完成的数据传输完成后，产生中断
);

  // 计算Log2
function integer clog2;
    input integer value;
    begin
      for (clog2 = 0; value > 0; clog2 = clog2 + 1) value = value >> 1;
    end
endfunction


  //FDMA读写状态机的状态值，一般4个状态值即可 
localparam    S_IDLE  = 2'd0;
localparam    S_RST   = 2'd1;
localparam    S_DATA1 = 2'd2;
localparam    S_DATA2 = 2'd3;

localparam    WFIFO_DEPTH           = W_BUFDEPTH;  //写通道FIFO深度
localparam    W_WR_DATA_COUNT_WIDTH = clog2(WFIFO_DEPTH);  //计算FIFO的写通道位宽
localparam    W_RD_DATA_COUNT_WIDTH = clog2(
      WFIFO_DEPTH * W_DATAWIDTH / AXI_DATA_WIDTH
);  //clog2(WFIFO_DEPTH/(AXI_DATA_WIDTH/W_DATAWIDTH))+1;

localparam    WYBUF_SIZE = (W_BUFSIZE - 1'b1);  //写通道需要完成多少次XSIZE操作
localparam    WY_BURST_TIMES       = (W_YSIZE*W_XDIV);//写通道需要完成的FDMA burst 操作次数，XDIV用于把XSIZE分解多次传输
localparam    FDMA_WX_BURST        = (W_XSIZE*W_DATAWIDTH/AXI_DATA_WIDTH)/W_XDIV; //FDMA BURST 一次的大小
localparam    WX_BURST_ADDR_INC    = (W_XSIZE*(W_DATAWIDTH/8))/W_XDIV; //FDMA每次burst之后的地址增加
localparam    WX_LAST_ADDR_INC     = (W_XSTRIDE-W_XSIZE)*(W_DATAWIDTH/8) + WX_BURST_ADDR_INC; //根据stride值计算出来最后一次地址



wire                              W_empty;
  
wire[W_RD_DATA_COUNT_WIDTH-1 : 0] W_rcnt;
reg [W_RD_DATA_COUNT_WIDTH-1 : 0] W_rcnt_r1,W_rcnt_r2;
reg [8  : 0]                      wrst_cnt;
reg                               wfifo_rst_done;
reg                               wfifo_rst_lock;

//延迟1拍，同时对
reg                               W_tuser_r3,W_tuser_r2, W_tuser_r1;
reg                               W_tvalid_r1;
reg                               W_tlast_r1,W_tlast_r2;
reg [W_DATAWIDTH -1 : 0]          W_tdata_r1;

//------------------------------------------------------------------------------
// 【本工程添加，非厂商原码】把原本写在下面（约 269 行 / 397 行）的 reg 声明前移到这里。
// 厂商原码在这些信号**首次使用之后**才声明它们（W_MS 在 214 行用、269 行才声明），
// 靠的是 Verilog 的隐式线网规则。TD 的综合器容忍这种前向引用，
// ModelSim 的 vlog 会报 vlog-2730 Undefined variable / vlog-2388 already declared
// 而编不过。这里只是把声明挪到使用之前，语义完全不变。
//------------------------------------------------------------------------------
reg [1 : 0]              W_MS                = 0;
reg [W_DSIZEBITS-1'b1:0] W_addr              = 0;
reg [15:0]               W_bcnt              = 0;
reg                      W_REQ               = 0;
reg [5 : 0]              wirq_dly_cnt        = 0;
reg [3 : 0]              wdiv_cnt            = 0;
reg                      O_fdma_wareq_r      = 1'b0; //synthesis keep
reg                      W_FIFO_Rst          = 0;
reg [7 : 0]              O_fdma_wbufn;
reg [7 : 0]              O_fdma_rbufn;
always @(posedge I_W_wclk) begin
              W_tuser_r1        <= I_W_tuser;
              W_tuser_r2        <= W_tuser_r1;
              W_tlast_r1        <= I_W_tlast;
              W_tlast_r2        <= W_tlast_r1;
              W_tvalid_r1       <= I_W_tvalid;
              W_tdata_r1        <= I_W_tdata;          
              W_tuser_r3        <= W_tuser_r2|W_tuser_r1|I_W_tuser;
end  

//计算XSIZE，本代码用于调试时候debug使用 
//reg[11 : 0]   w_xcnt;
//always @(posedge I_W_wclk ) begin
//  if(wfifo_rst_done == 0)                   
//              w_xcnt            <= 0;
//  else if({W_tuser_r2, W_tuser_r1} == 2'b01)  
//              w_xcnt            <= 0;
//  else if({W_tlast_r2 & W_tlast_r1} == 2'b01)           
//              w_xcnt            <= 0;
//  else if(W_tvalid_r1)                       
//              w_xcnt            <= w_xcnt + 1'b1;
//end

//reg[11 : 0]   w_ycnt;
//always @(posedge I_W_wclk ) begin
//  if(wfifo_rst_done == 0)                   
//              w_ycnt            <= 0;
//  else if (~W_tlast_r2 & W_tlast_r1 ) begin //同步I_W_tlast计数同步
//    if (w_ycnt == W_YSIZE - 1)              
//              w_ycnt            <= 0;
//    else                                    
//              w_ycnt            <= w_ycnt + 1'b1;  //?帧计数器
//  end
//end


//FIFO ?同步复位计数器
always @(posedge I_W_wclk or negedge I_ui_rstn) begin
  if (~I_ui_rstn) begin
              wrst_cnt          <= 0;
              wfifo_rst_done    <= 0;
              wfifo_rst_lock    <= 0;
  end
  else if(I_W_tuser & (W_empty == 1'b0)) begin //同步丢失，复位FIFO 重新同步
              wfifo_rst_lock    <= 1;
              wfifo_rst_done    <= 0;
              wrst_cnt          <= 0;
  end 
  else if(wfifo_rst_lock & (wrst_cnt[8] == 0))  //对FIFO复位
              wrst_cnt          <= wrst_cnt + 1'b1;
  else if(W_MS == S_IDLE & I_W_tuser)begin      //完成FIFO 复位和同步
              wrst_cnt          <= 0;
              wfifo_rst_done    <= 1;
              wfifo_rst_lock    <= 0;
  end
end

//FIFO 复位
wire   wfifo_rst  = (wrst_cnt >10) & (wrst_cnt <80) ;//帧同步期间，复位40个I_W_wclk的周期

reg [1:0]     W_MS_r = 0;
always @(posedge I_ui_clk) W_MS_r <= W_MS;

  //每次FDMA DBUF 完成一帧数据传输后，产生中断，这个中断持续60个周期的uiclk,这里的延迟必须足够ZYNQ IP核识别到这个中断
always @(posedge I_ui_clk) begin
    if (I_ui_rstn == 1'b0) begin
              wirq_dly_cnt      <= 6'd0;
              O_fdma_wbuf       <= 0;
    end else if ((W_MS_r == S_DATA2) && (W_MS == S_IDLE)) begin
              wirq_dly_cnt      <= 60;
              O_fdma_wbuf       <= O_fdma_wbufn;
    end else if (wirq_dly_cnt > 0)
              wirq_dly_cnt      <= wirq_dly_cnt - 1'b1;
end

assign O_fdma_wareq = O_fdma_wareq_r;

//帧同步，对于视频有效
reg           wfifo_rst_done_r1 = 1'b0, wfifo_rst_done_r2 = 1'b0, wfifo_rst_done_r3 = 1'b0;
always @(posedge I_ui_clk) begin
              wfifo_rst_done_r1 <= wfifo_rst_done;
              wfifo_rst_done_r2 <= wfifo_rst_done_r1;
end
//写状态机帧同步
reg           wfs_r1,wfs_r2,wfs_r3,wfs_r4;
always @(posedge I_ui_clk) begin
              wfs_r1            <= W_tuser_r3;
              wfs_r2            <= wfs_r1;
              wfs_r3            <= wfs_r2;
              wfs_r4            <= wfs_r3;
end

wire            w_fs;
assign          w_fs              = {wfs_r4,wfs_r3} == 2'b10 ? 1 : 0;

//读通道的数据FIFO，采用了原语调用xpm_fifo_async fifo，当FIFO存储空间有足够空余，满足一次FDMA的burst即可发出请求
always @(posedge I_ui_clk)begin
              W_rcnt_r1           <= W_rcnt;
              W_rcnt_r2           <= W_rcnt_r1;
end

always @(posedge I_ui_clk) W_REQ <= (W_rcnt_r2 > FDMA_WX_BURST - 1);

assign        O_fdma_wready = 1'b1;

// 上面这些 reg 的声明已前移到文件开头的声明区（见那里的【本工程添加】说明）

assign        O_fdma_wsize        = FDMA_WX_BURST;
assign        O_fdma_wirq         = (wirq_dly_cnt > 0);
assign        O_fdma_waddr        = W_BASEADDR + {O_fdma_wbufn,W_addr};//由于FPGA逻辑做乘法比较复杂，因此通过设置高位地址实现缓存设置


//写通道状态机，采用4个状态值描述
always @(posedge I_ui_clk) begin
  if (!I_ui_rstn) begin
              W_MS              <= S_IDLE;
              W_FIFO_Rst        <= 0;
              W_addr            <= 0;
              O_W_sync_cnt      <= 0;
              W_bcnt            <= 0;
              wdiv_cnt          <= 0;
              O_fdma_wbufn      <= 0;
    end else begin
      case (W_MS)
        S_IDLE: begin
              W_addr            <= 0;
              W_bcnt            <= 0;
              wdiv_cnt          <= 0;
          if((I_W_en & (wfifo_rst_done_r2 & w_fs) )) begin //帧同步，对于非视频数据一般常量为1
		    W_MS              <= S_RST;
            if((O_W_sync_cnt == WYBUF_SIZE && O_fdma_rbufn == 0) ||(O_W_sync_cnt + 1 == O_fdma_rbufn))
              O_W_sync_cnt  <=  O_W_sync_cnt;/////////////////////////////////////////////////////////////////////////////////写指针控制
            else  if (O_W_sync_cnt < WYBUF_SIZE)  //输出帧同步计数器
              O_W_sync_cnt <= O_W_sync_cnt + 1'b1;
            else O_W_sync_cnt <= 0;
          end
        end
        S_RST:begin//帧同步，对于非视频数据直接跳过,对于视频数据，会同步每一帧，并且复位数据FIFO
              O_fdma_wbufn      <= I_W_buf;
              W_MS              <= S_DATA1;
        end
        S_DATA1: begin  //发送写FDMA请求
          if (wfifo_rst_done_r2 == 1'b0) 
              W_MS              <= S_IDLE;
          else if (I_fdma_wbusy == 1'b1) begin
              W_MS              <= S_DATA2;
          end
        end
        S_DATA2: begin  //写有效数据
          if (I_fdma_wbusy == 1'b0) begin
            if (W_bcnt == WY_BURST_TIMES - 1'b1)  //判断是否传输完毕
              W_MS              <= S_IDLE;
            else begin
              if(wdiv_cnt < W_XDIV - 1'b1)begin//如果对XSIZE做了分次传输，一个XSIZE也需要XDIV次FDMA完成传输
                W_addr          <= W_addr + WX_BURST_ADDR_INC;  //计算地址增量
                wdiv_cnt        <= wdiv_cnt + 1'b1;
              end else begin
                W_addr          <= W_addr + WX_LAST_ADDR_INC; //计算最后一次地址增量，最后一次地址根据stride 计算
                wdiv_cnt        <= 0;
              end
                W_bcnt          <= W_bcnt + 1'b1;
                W_MS            <= S_DATA1;
            end
          end
        end
        default: W_MS           <= S_IDLE;
      endcase
    end
end
    
wire          W_full_pro;
wire          O_W_full;
assign        O_W_tready = 1'b1;

//写FIFO使能
wire   wfifo_wen  = W_tvalid_r1 & wfifo_rst_done;

// ★ 2026-11-01 诊断：写 FIFO 满时的写入会被 sfifo 直接丢弃，数一下有多少次。
//   纯观测，不改变任何行为。
//   ★ 2026-11-01 改成「每帧统计 + 峰值慢衰减」。
//   原来是累计计数，一超过 255 就永远卡在 255，完全没有信息量。
//   现在：每帧清 cur_drop 并把上一帧的计数抬进 O_wdrop_cnt 做峰值保持，
//   之后每帧衰减 1 → 一次爆发可以在屏上停留最多 255 帧（约 4 秒），
//   足够看清是"哪一步操作"触发的。
reg [15:0] cur_drop;
always @(posedge I_W_wclk or negedge I_ui_rstn) begin
    if (!I_ui_rstn) begin
        cur_drop    <= 16'd0;
        O_wdrop_cnt <= 16'd0;
    end else if (I_W_tuser) begin
        if (cur_drop > O_wdrop_cnt)          O_wdrop_cnt <= cur_drop;
        else if (O_wdrop_cnt != 16'd0)       O_wdrop_cnt <= O_wdrop_cnt - 16'd1;
        cur_drop <= 16'd0;
    end
    // 只统计"这一路本该在写"时的溢出。I_W_en=0 的那一路是有意停写的，
    // 上游还在往里灌，FIFO 必然满——那不是故障，别污染计数器。
    else if (wfifo_wen & O_W_full & I_W_en) begin
        cur_drop <= cur_drop + 16'd1;
    end
end


sfifo #( 
.DATA_WIDTH_W(W_DATAWIDTH), 
.DATA_WIDTH_R(AXI_DATA_WIDTH), 
.ADDR_WIDTH_W(W_WR_DATA_COUNT_WIDTH), 
.ADDR_WIDTH_R(W_RD_DATA_COUNT_WIDTH), 
.AL_FULL_NUM(WFIFO_DEPTH-2), 
.AL_EMPTY_NUM(2), 
.SHOW_AHEAD_EN(1'b1) , 
.OUTREG_EN ("NOREG")
) wdbuf_fifo (
.rst          ((I_ui_rstn == 1'b0) | wfifo_rst  ),
.clkw         (I_W_wclk                         ),
.we           (wfifo_wen                        ),
.di           (W_tdata_r1                       ),
.full_flag    (O_W_full                         ),
.afull        (W_full_pro                       ),
.clkr         (I_ui_clk                         ),
.re           (I_fdma_wvalid                    ),
.dout         (O_fdma_wdata                     ),
.empty_flag   (W_empty                          ),
.rdusedw      (W_rcnt                           )
);

//generate  if(ENABLE_READ == 1)begin : FDMA_READ// 通过设置通道使能，可以优化代码的利用率
localparam RYBUF_SIZE = (R_BUFSIZE - 1'b1);  //读通道需要完成多少次XSIZE操作
localparam RY_BURST_TIMES             = (R_YSIZE*R_XDIV); //读通道需要完成的FDMA burst 操作次数，XDIV用于把XSIZE分解多次传输
localparam FDMA_RX_BURST              = (R_XSIZE*R_DATAWIDTH/AXI_DATA_WIDTH)/R_XDIV; //FDMA BURST 一次的大小
localparam RX_BURST_ADDR_INC          = (R_XSIZE*(R_DATAWIDTH/8))/R_XDIV; //FDMA每次burst之后的地址增加
localparam RX_LAST_ADDR_INC           = (R_XSTRIDE-R_XSIZE)*(R_DATAWIDTH/8) + RX_BURST_ADDR_INC; //根据stride值计算出来最后一次地址

localparam RFIFO_DEPTH                = R_BUFDEPTH*R_DATAWIDTH/AXI_DATA_WIDTH;//R_BUFDEPTH/(AXI_DATA_WIDTH/R_DATAWIDTH);
localparam R_WR_DATA_COUNT_WIDTH      = clog2(RFIFO_DEPTH);  //读通道FIFO 输入部分深度
localparam R_RD_DATA_COUNT_WIDTH      = clog2(R_BUFDEPTH);  //写通道FIFO输出部分深度


assign O_fdma_rready = 1'b1;
reg                                     O_fdma_rareq_r = 1'b0;
reg  [                         1 : 0]   R_MS = 0;//synthesis keep
reg  [            R_DSIZEBITS-1'b1:0]   R_addr = 0;
reg  [                          15:0]   R_bcnt = 0;
wire [R_WR_DATA_COUNT_WIDTH-1'b1 : 0]   R_wcnt;
reg  [R_WR_DATA_COUNT_WIDTH-1'b1 : 0]   R_wcnt_r1,R_wcnt_r2;
reg                                     R_REQ = 0;
reg  [                         5 : 0]   rirq_dly_cnt = 0;
reg  [                         3 : 0]   rdiv_cnt = 0;
reg  [                         7 : 0]   rrst_cnt = 0;
// O_fdma_rbufn 的声明已前移到文件开头的声明区（见那里的【本工程添加】说明）
assign    O_fdma_rsize                 = FDMA_RX_BURST;
assign    O_fdma_rirq                  = (rirq_dly_cnt > 0);

assign    O_fdma_raddr                 = R_BASEADDR + {O_fdma_rbufn,R_addr};//由于FPGA逻辑做乘法比较复杂，因此通过设置高位地址实现缓存设置

reg  [1 :0] R_MS_r = 0;

always @(posedge I_ui_clk)      R_MS_r <= R_MS;

  //每次FDMA DBUF 完成一帧数据传输后，产生中断，这个中断持续60个周期的uiclk,这里的延迟必须足够ZYNQ IP核识别到这个中断
always @(posedge I_ui_clk) begin
  if (I_ui_rstn == 1'b0) begin
              rirq_dly_cnt        <= 6'd0;
              O_fdma_rbuf         <= 0;
  end else if ((R_MS_r == S_DATA2) && (R_MS == S_IDLE)) begin
              rirq_dly_cnt        <= 60;
              O_fdma_rbuf         <= O_fdma_rbufn;
  end else if (rirq_dly_cnt > 0) 
              rirq_dly_cnt        <= rirq_dly_cnt - 1'b1;
end

assign        O_fdma_rareq         = O_fdma_rareq_r;

reg rfifo_rst;
reg rfs_r1,rfs_r2,rfs_r3,rfs_r4;

always @(posedge I_ui_clk) begin
              rfs_r1            <= I_R_en;
              rfs_r2            <= rfs_r1;
              rfs_r3            <= rfs_r2;
              rfs_r4            <= rfs_r3;
end

//读通道状态机，采用4个状态值描述
always @(posedge I_ui_clk) begin
  if (!I_ui_rstn) begin
              R_MS              <= S_IDLE;
              R_addr            <= 0;
              O_R_sync_cnt      <= 0;
              R_bcnt            <= 0;
              rrst_cnt          <= 0;
              rdiv_cnt          <= 0;
              O_fdma_rbufn      <= 0;
  end else begin
    case (R_MS) //帧同步，对于非视频数据一般常量为1
    S_IDLE: begin
              R_addr          <= 0;
              R_bcnt          <= 0;
              rrst_cnt        <= 0;
              rdiv_cnt        <= 0;
      if({rfs_r4,rfs_r3} == 2'b10 | (ENABLE_VSYNC == 0)) begin
              R_MS <= S_RST;
        if(O_R_sync_cnt < RYBUF_SIZE) //输出帧同步计数器，当需要用读通道做帧同步的时候使用
              O_R_sync_cnt    <= O_R_sync_cnt + 1'b1;
        else  O_R_sync_cnt    <= 0;
      end

    end
    S_RST: begin  //读操作没有帧同步，所以必须保证开始就是能同步
 		          O_fdma_rbufn    <= I_R_buf;
              R_MS            <= S_DATA1;
    end
    S_DATA1: begin
      if (I_fdma_rbusy == 1'b1) begin
              R_MS            <= S_DATA2;
      end
    end
    S_DATA2: begin  //写有效数据
      if (I_fdma_rbusy == 1'b0) begin
        if (R_bcnt == RY_BURST_TIMES - 1'b1)  //判断是否传输完毕
              R_MS            <= S_IDLE;
        else begin
          if(rdiv_cnt < R_XDIV - 1'b1)begin//如果对XSIZE做了分次传输，一个XSIZE也需要XDIV次FDMA完成传输
              R_addr          <= R_addr + RX_BURST_ADDR_INC;  //计算地址增量
              rdiv_cnt        <= rdiv_cnt + 1'b1;
          end else begin
              R_addr          <= R_addr + RX_LAST_ADDR_INC; //计算最后一次地址增量，最后一次地址根据stride 计算
              rdiv_cnt        <= 0;
          end
              R_bcnt          <= R_bcnt + 1'b1;
              R_MS            <= S_DATA1;
        end
      end
    end
    default:  R_MS            <= S_IDLE;
    endcase
  end
end

  //读通道的数据FIFO，采用了原语调用xpm_fifo_async fifo，当FIFO存储空间有足够空余，满足一次FDMA的burst即可发出请求
wire          R_empty;  //synthesis keep
wire          R_tvalid;  //synthesis keep
assign        R_tvalid        = ~R_empty;

always @(posedge I_ui_clk) begin
              R_wcnt_r1       <= R_wcnt;
              R_wcnt_r2       <= R_wcnt_r1;
end
always @(posedge I_ui_clk) R_REQ <= (R_wcnt_r2 < FDMA_RX_BURST - 1 );

assign        O_R_tvalid      = ~R_empty &(I_R_tready);


//以下代码用于参数tuser 和 tlast，对于读通道，数据一旦出错，输出错误不可纠错，一般内部通道除非硬件问题，也不会出错
reg           r_tuser_lock;  //synthesis keep
// reg r_tuser;
reg           r_tlast;  //synthesis keep
reg [15:0]    r_xcnt;  //synthesis keep
reg           fram_end;  //synthesis keep
reg [15:0]    r_ycnt;  //synthesis keep

assign        O_R_tuser = r_tuser_lock & O_R_tvalid;
assign        O_R_tlast = (r_xcnt == R_XSIZE - 1) & O_R_tvalid;

always @(posedge I_R_rclk or negedge I_ui_rstn) begin
  if (~I_ui_rstn) 
              r_xcnt          <= 0;
  else if (O_R_tvalid) begin
    if (r_xcnt == R_XSIZE - 1) 
              r_xcnt          <= 0;
    else      
              r_xcnt          <= r_xcnt + 1'b1;
  end
end

always @(posedge I_R_rclk or negedge I_ui_rstn) begin
  if (~I_ui_rstn) begin
              r_tuser_lock    <= 1'b1;
              r_ycnt          <= 0;
  end else begin
    if (O_R_tlast) begin
      if (r_ycnt == R_YSIZE - 1) begin
              r_tuser_lock    <= 1'b1;
              r_ycnt          <= 0;
      end 
      else 
              r_ycnt          <= r_ycnt + 1'b1;
    end 
    else if (O_R_tuser)
              r_tuser_lock    <= 1'b0;
  end
end

reg [20:0]    dl_cnt;
assign        O_R_vrst        = ~dl_cnt[20];
always @(posedge I_R_rclk or negedge I_ui_rstn) begin
  if(~I_ui_rstn)
              dl_cnt          <= 0;
  else if(~dl_cnt[20])
               dl_cnt         <= dl_cnt + 1'b1;
end


sfifo #( 
.DATA_WIDTH_W(AXI_DATA_WIDTH), 
.DATA_WIDTH_R(R_DATAWIDTH), 
.ADDR_WIDTH_W(R_WR_DATA_COUNT_WIDTH), 
.ADDR_WIDTH_R(R_RD_DATA_COUNT_WIDTH), 
.AL_FULL_NUM(RFIFO_DEPTH-2), 
.AL_EMPTY_NUM(2), 
.SHOW_AHEAD_EN(1'b1) , 
.OUTREG_EN ("NOREG")
) rdbuf_fifo (
.rst       (I_ui_rstn == 1'b0             ),
.clkw      (I_ui_clk                      ),
.we        (I_fdma_rvalid                 ),
.di        (I_fdma_rdata                  ),
.wrusedw   (R_wcnt                        ),
.clkr      (I_R_rclk                      ),
.re        (O_R_tvalid                    ),
.dout      (O_R_tdata                     ),
.empty_flag(R_empty                       )
);


reg [1:0]WR_S;//synthesis keep

always @(posedge I_ui_clk) begin
  if(!I_ui_rstn)begin
              WR_S            <= 2'd0;
              O_fdma_wareq_r  <= 1'd0;
              O_fdma_rareq_r  <= 1'd0;
  end
  else begin
    case(WR_S)
    0:begin
      if(I_fdma_wbusy == 1'b0 && W_REQ && W_MS == S_DATA1)begin //优先执行写
              O_fdma_wareq_r  <= 1'b1;
              WR_S            <= 2'd1;
      end  
      else if(I_fdma_rbusy == 1'b0 && R_REQ  && R_MS == S_DATA1)begin//如果写完成后再执行读
              O_fdma_rareq_r  <= 1'b1;  
              WR_S            <= 2'd2;
      end
    end
    1:begin
      if(I_fdma_wbusy == 1'b1) begin //等待写完成
              O_fdma_wareq_r  <= 1'b0;
              WR_S            <= 3;
      end
    end          
    2:begin
      if(I_fdma_rbusy == 1'b1) begin //等待读完成
              O_fdma_rareq_r  <= 1'b0;
              WR_S            <= 3;
      end            
    end
    3:begin
      if(I_fdma_wbusy==0&&I_fdma_rbusy==0)
              WR_S            <= 0;
      end
    default:  WR_S            <= 0;
    endcase
    end
end  





endmodule

