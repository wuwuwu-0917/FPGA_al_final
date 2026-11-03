
`define DRAM_BYTE_NUM 4
`define ALC_MC_32B
`define AXI
//`define STG 
`define APP_ADDR_WIDTH   27
`define PHOTO_BADGE              // 关掉它可整块综合掉右上角拍照角标



module design_top_wrapper (
    /*
        input 25MHz system clock
    */
    input wire       I_sys_clk_25m,


    /*
        anPH1A SW2 / SW3: active-low UI keys (K4 / P5)
    */
    input wire       I_key_in,
    input wire       I_key_next_n,

    /*
        camera (IMX415) iic configure interface
    */
    inout wire       IO_cam_scl,
    inout wire       IO_cam_sda,

    /*
        camera MCLK(INCK) 与复位 XCLR
    */
	output wire      O_cam_clk_27m,

	output wire      O_cam_rst_n,

	inout wire       IO_led,

    /*
        SD card SPI（anPH1A TF 座：P17/R21/P22/P21）
    */
    input  wire      I_sd_miso,
    output wire      O_sd_clk,
    output wire      O_sd_cs,
    output wire      O_sd_mosi,

    ///hdmi tmds channel signal N
	output [2:0]O_hdmi_tx_p, //HDMI输出数据P端
	output [2:0]O_hdmi_tx_n,  //HDMI输出数据N端
    output O_hdmi_clk_p,     //HDMI输出时钟P端
	output O_hdmi_clk_n,     //HDMI输出时钟N端
    
    
    output wire 	 fan_pwm

);
	
	wire        S_sys_clk;
    wire        S_hdmi_clk;
    wire		S_serial_clk;
    wire 		I_ddr_clk;
 

    wire        S_clk_70m;

    // 相机黑电平：原由软核经 APB 写入（复位默认值就是 0），软核删除后改为常量。
    // 阶段 3b 之后这个减法会被 ISP 链里的 ui_BLC 取代。
    localparam [9:0] CAM_BLACK_LEVEL = 10'd0;

    wire        S_configure_load_en;
    wire        S_configure_done;                   
    wire[13:0]  S_HTOTAL;                           
    wire[13:0]  S_HSA;                              
    wire[13:0]  S_HFP;                              
    wire[13:0]  S_HBP;                              
    wire[13:0]  S_HACTIVE;                          
    wire[13:0]  S_VTOTAL;                           
    wire[13:0]  S_VSA;                              
    wire[13:0]  S_VFP;                              
    wire[13:0]  S_VBP;                              
    wire[13:0]  S_VACTIVE;                          
    wire        S_hdcp_en;                          
    wire        S_video_test_source_en;                         
    wire[6:0]   S_video_VIC;                               
    wire[1:0]   S_pixel_color_deep;                        
    wire[2:0]   S_video_format;                            
    wire[1:0]   S_pixel_num_per_clock;                 
    wire        S_data_over_sample_en;    
    wire[19:0]  S_audio_clock_regeneration_CTS;            
    wire[19:0]  S_audio_clock_regeneration_N;               
    wire[2:0]   S_audio_channel_number;                    
    wire[2:0]   S_audio_sample_rate;                       

    wire        S_serdes_config_load_en;                 
    wire[1:0]   S_serdes_data_width_sel;                 
    wire[2:0]   S_serdes_tx_rate;                        
    wire[23:0]  S_serdes_ref_clk_freq;                   
    wire[15:0]  S_serdes_line_rate;                      
    wire        S_serdes_mpll_sel;                       
    wire[1:0]   S_serdes_mpll_tx_clk_div;                
    wire[7:0]   S_serdes_mpll_multiplier;                
    wire        S_serdes_config_done;      

    wire        S_serdes_configure_load_en;         
 
	wire        S_pll_lock;

    // Independent power-on reset frees both physical keys for the OSD.
    // Initialized FFs are supported by TD (as in the board's fan_ctrl demo).
    // Hold the PLL and camera in reset for 4096 reference clocks (~164 us).
    reg [11:0] S_por_count = 12'd0;
    wire S_board_reset_n = &S_por_count;
    always @(posedge I_sys_clk_25m) begin
        if (!S_board_reset_n) S_por_count <= S_por_count + 1'b1;
    end


    wire        S_hs_rx_clk;           //synthesis keep  
    wire        S_hs_rx_valid;         //synthesis keep  
    wire[31:0]  S_hs_rx_data;          //synthesis keep
 
    wire        S_csi_frame_start;  //synthesis keep      
    wire        S_csi_frame_end;  //synthesis keep         
    wire        S_csi_valid;  //synthesis keep           
    wire[31:0]  S_csi_data;          
 
 
    wire        S_rgb888_frame_start;
    wire        S_rgb888_frame_end;
    wire        S_rgb888_valid;      
    wire[95:0]  S_rgb888_data;        

    wire        S_awb_frame_start;    //synthesis keep
    wire        S_awb_frame_end;      //synthesis keep
    wire        S_awb_valid;          //synthesis keep
    wire[95:0]  S_awb_data;           //synthesis keep

    wire        S_mipi_rx_error;         //synthesis keep

    wire        S_vi_256b_frame_start;   //synthesis keep
    wire        S_vi_256b_valid;         //synthesis keep
    wire[255:0] S_vi_256b_data;       

    wire        S_video_out_rd_busy;
    wire        S_video_in_wr_busy; 
    wire[1:0]   S_video_out_rp;     

    wire        S_ddr_user_wr_en;        //synthesis keep
    wire        S_ddr_user_rd_en;        //synthesis keep
    wire[27:0]  S_ddr_user_addr;         //synthesis keep
    wire[255:0] S_ddr_user_wr_data;      //synthesis keep
    wire        S_ddr_user_ready;        //synthesis keep
    wire        S_ddr_user_rd_valid;     //synthesis keep
    wire[255:0] S_ddr_user_rd_data;      //synthesis keep

    wire        S_vi_ddr_wr_en;   
    wire[27:0]  S_vi_ddr_wr_addr; 
    wire[255:0] S_vi_ddr_wr_data;
    wire        S_vi_ddr_wr_ready;

    wire        S_vo_ddr_rd_en;   
    wire[27:0]  S_vo_ddr_rd_addr; 
    wire        S_vo_ddr_rd_ready;

    wire        S_video_source_vsync;
    wire        S_video_source_user; 
    wire        S_video_source_valid;
    wire        S_video_source_last; 

    wire        S_vdieo_rd_en;  
    wire[95:0]  S_video_rd_data;

    wire        S_mixer_user; 
    wire        S_mixer_valid;  
    wire        S_mixer_last;   
    wire[23:0] S_mixer_data;   

    reg[9:0]    S_hdmi_config_cnt;
    reg         S_serdes_config_done_1d;

    wire        S_ddr_clk;       //synthesis keep

    wire        S_init_calib_complete;  
    wire[27:0]  S_mc_app_addr;          
    wire[2:0]   S_mc_app_cmd;           
    wire        S_mc_app_en;            
    wire[255:0] S_mc_app_wdf_data;      
    wire        S_mc_app_wdf_end;       
    wire[31:0]  S_mc_app_wdf_mask;      
    wire        S_mc_app_wdf_wren;      
    wire[255:0] S_mc_app_rd_data;       
    wire        S_mc_app_rd_data_end;   
    wire        S_mc_app_rd_data_valid; 
    wire        S_mc_app_rdy;           
    wire        S_mc_app_wdf_rdy;       

    //  MIPI lane 错误标志，来自 DPHY。同样必须带 mark_debug 才看得到波形
	(* KEEP = "TRUE" *) wire[3:0]   S_lane_error;//synthesis keep

    //=========================================================================
    //  IMX415 硬件 I2C 配置引擎相关信号（本次新增）
    //  对应文件：imx415_power_seq.v / imx415_cfg_fsm.v / imx415_cfg_rom.v
    //            / imx415_i2c_master.v
    //=========================================================================
    wire        S_cam_clk_27m;            // PLL C0 原始输出（未门控）
    reg  [1:0]  S_mclk_en_sync;           // MCLK 门控使能的 2 级同步（27MHz 下降沿域）
    wire        S_cam_mclk_en;            // 来自 power_seq（25MHz 域）
    wire        S_cam_xclr_n;             // 相机复位，低有效
    wire        S_cam_cfg_start_25m;      // power_seq 给出的启动电平（25MHz 域）
    wire        S_cam_cfg_start;          // 同步到 70MHz 域之后

    wire [7:0]  S_cfg_rom_index;
    wire [23:0] S_cfg_rom_data;
    wire        S_hw_i2c_start;
    wire [6:0]  S_hw_i2c_dev;
    wire [15:0] S_hw_i2c_addr;
    wire [7:0]  S_hw_i2c_data;
    wire        S_hw_i2c_busy;
    wire        S_hw_i2c_done;
    wire        S_hw_i2c_nack;
    wire        S_hw_i2c_scl_oe;
    wire        S_hw_i2c_sda_oe;

    //-------------------------------------------------------------------------
    //  下面这几个是给 ChipWatcher 看的调试量。
    //
    //  ★ 2026-10-28：像素滤镜版 ERAM 超预算
    //    （design 268 + ChipWatcher 7 = 275 > 272）。
    //    最终 demo 关闭 ChipWatcher：这里去掉 (* mark_debug *) 属性，保留 KEEP，
    //    并删除/重命名 phy_1/cw。需要调试时再把 mark_debug 加回来，
    //    并先做 E/F→4px 等 ERAM 降级给调试缓冲腾地方。
    //    （原来 TD 的 insert_debugger 只收带 mark_debug 的信号；
    //      只写 //synthesis keep 不会出现在波形窗口。）
    //-------------------------------------------------------------------------
    (* KEEP = "TRUE" *) wire        S_cam_cfg_done;    //synthesis keep
    (* KEEP = "TRUE" *) wire [7:0]  S_cam_err_cnt;     //synthesis keep
    (* KEEP = "TRUE" *) wire [3:0]  S_cam_scan_result; //synthesis keep
    (* KEEP = "TRUE" *) wire [6:0]  S_cam_dev_used;    //synthesis keep
    (* KEEP = "TRUE" *) wire [2:0]  S_cam_fsm_state;   //synthesis keep
    (* KEEP = "TRUE" *) wire [2:0]  S_cam_pwr_state;   //synthesis keep

    
    wire  S_fifo_empty;
    wire  S_fifo_rd_valid;
    wire  S_fifo_frame_start;
    wire[9:0]  S_fifo_rd_num;
    wire[9:0]  S_fifo_wr_num;

    wire [23:0]  S_fifo_rd_data;
    reg         S_fifo_rd_en;
    
    wire 			S_csi_rx_vsync0;//synthesis keep
	wire 			S_csi_rx_hsync0;//synthesis keep

    wire [11:0] S_delay_ms10;   //synthesis keep
    wire        S_delay_valid;  //synthesis keep
    
    wire   vtc_rstn ;
    wire   vtc_clk;
    wire   vtc_vs;//synthesis keep
    wire   vtc_hs;//synthesis keep
    //wire   vtc_user;//synthesis keep
    wire   vtc_de_valid;//synthesis keep
    wire   S_R_vrst; // DDR read-side frame alignment resets VTC only.
    //wire   vtc_last;////synthesis keep
   
    wire S_rst;
    wire [7 :0]	rgb_r ,rgb_g ,rgb_b;// 定义寄存器保存图像的颜色数据
    assign vtc_clk = S_hdmi_clk; // 内部像素时钟
    // 上电用 PLL LOCK 复位；另外叠加 ui_dbuf 读侧的 VS 重同步脉冲
    // （参考工程就是把 O_R_vrst 接去复位 VTC 与 HDMI，让显示帧与读缓存切换对齐）
    // ★ 2026-11-03：VTC 是**同步复位**（uivtc.v 里 always @(posedge clk) if(!rstn)），
    //   而复位源里的 S_pll_lock 与 vtc_clk 异步 —— 复位**撤销**会踩在
    //   恢复/移除（recovery/removal）窗口上，FF 可能进亚稳态，VTC 计数器
    //   起在未定义状态，影响视频时序输出的稳定性。
    //   改成标准做法：异步置位、同步撤销（只把撤销推迟 3 拍 ≈20ns）。
    wire vtc_rstn_raw = S_pll_lock & ~S_R_vrst;
    reg  [2:0] vtc_rstn_sync;   //synthesis keep
    always @(posedge vtc_clk or negedge vtc_rstn_raw)
        if (!vtc_rstn_raw) vtc_rstn_sync <= 3'b000;
        else               vtc_rstn_sync <= {vtc_rstn_sync[1:0], 1'b1};
    assign vtc_rstn = vtc_rstn_sync[2];
    assign S_rst = ~S_pll_lock;


    wire [31:0] ud_rdata_0;//synthesis keep   // 1456 dbuf 读出的 32bit 像素
    wire [31:0] ud_rdata_1;//synthesis keep   // 1920 dbuf 读出的 32bit 像素
    wire        S_ui_visible;
    wire        S_ui_hidden;
    wire        S_hdmi_pixel_valid_0, S_hdmi_pixel_valid_1;
    wire [23:0] S_cam_rgb;
    wire        S_cam_pixel_valid;
    wire        S_vis_ready;
    wire [7:0]  wbuf_sync_0, wbuf_sync_1;
    // 两路 dbuf 的写 FIFO 状态监测（纯观测，不影响功能）
    wire [15:0] S_wdrop0, S_wdrop1;
    wire [7:0]  rbuf_sync_0, rbuf_sync_1;
    wire        S_r_en_0, S_r_en_1, S_r_ready_0, S_r_ready_1;
    wire        v0_rbusy, v1_rbusy, v0_rvalid, v1_rvalid;
    wire [28:0] v0_waddr, v1_waddr, vid_waddr;
    wire        v0_wareq, v1_wareq, vid_wareq;
    wire [15:0] v0_wsize, v1_wsize, vid_wsize;
    wire [255:0] v0_wdata, v1_wdata, vid_wdata;
    wire        v0_wbusy, v1_wbusy, vid_wbusy, v0_wvalid, v1_wvalid, vid_wvalid;
    wire [28:0] v0_raddr, v1_raddr, act_raddr;
    wire        v0_rareq, v1_rareq, act_rareq;
    wire [15:0] v0_rsize, v1_rsize, act_rsize;
    wire        act_rbusy, act_rvalid;
    reg  [1:0]  hidden_axi_sync;

    // ISP 链输出（ui_isp_top -> ui_dbuf 写侧，128bit = 4 像素 x {8'h00,R,G,B}）
    wire         S_isp_tuser, S_isp_tvalid, S_isp_tlast;
    wire [127:0] S_isp_tdata;//synthesis keep
    wire         S_fit_tuser, S_fit_tvalid, S_fit_tlast;
    wire [127:0] S_fit_tdata;

    // ISP 链输入（ui_image_correction -> ui_isp_top，40bit = 4 x RAW10，P0 在 bit[9:0]）
    wire [39:0]  S_raw_tdata;//synthesis keep
    wire         S_raw_tvalid, S_raw_tlast, S_raw_tuser;
    wire [9:0]   S_raw_tdest;

    // ui_raw10_unpacket_4lane 输出（40bit RAW10）
    wire [39:0]  S_r10_data;//synthesis keep
    wire         S_r10_valid, S_r10_fs, S_r10_fe;
    wire [15:0] S_corr_pad_cnt;
    wire [15:0] S_corr_miss_cnt;
    wire [15:0] S_corr_gap_max;
    wire [15:0] S_corr_gap_min;

    ui_fit_scaler u_ui_fit_scaler (
        .I_clk(S_hs_rx_clk), .I_rst_n(S_pll_lock),
        .I_tuser(S_isp_tuser), .I_tvalid(S_isp_tvalid),
        .I_tdata(S_isp_tdata), .I_tlast(S_isp_tlast),
        .O_tuser(S_fit_tuser), .O_tvalid(S_fit_tvalid),
        .O_tdata(S_fit_tdata), .O_tlast(S_fit_tlast)
    );
    

	PLL U_PLL(
  		.refclk   ( I_sys_clk_25m          ),
  		.lock     ( S_pll_lock             ),
        .reset	  (	~S_board_reset_n	   ),
 
  		.clk0_out ( S_cam_clk_27m                ),   // 改为内部网，出来后经门控再送引脚
  		.clk1_out ( S_sys_clk              ),
        .clk2_out ( S_clk_70m              ),
        .clk3_out ( S_hdmi_clk       ),  //像素时钟
        .clk4_out (	S_serial_clk			   ),//HDMI IO的serdes 时钟 5倍的像素时钟
        .clk5_out (	I_ddr_clk			   )
	);

    //=========================================================================
    //  相机复位 XCLR
    //
    //  原先恒接 1'b1 —— 也就是说原设计**从未给传感器做过 system clear**。
    //  现在由 imx415_power_seq 产生：PLL 锁定后先拉低 >=500ns，再拉高。
    //  注意 pin.adc 上这个脚是 PULLTYPE=NONE，FPGA 配置期间是悬空的。
    //=========================================================================
	assign O_cam_rst_n = S_cam_xclr_n;

    //=========================================================================
    //  LED：软核已删除，改由"相机配置完成"直接点亮
    //  （imx415_cfg_fsm 的 O_cfg_done，见下面的配置引擎例化）
    //=========================================================================
    // LED 的 assign 移到下方 photo_fsm 之后（改成高有效、由拍照状态机驱动）

    //=========================================================================
    //  I2C 引脚：由硬件配置引擎独占驱动
    //
    //  软核已删除（连同它那份跑不动的 SC2210 8KB 机器码固件），
    //  所以这里不再需要任何"软核/硬件谁驱动总线"的仲裁，直接由
    //  imx415_i2c_master 的开漏输出驱动即可。
    //=========================================================================
    assign IO_cam_scl = S_hw_i2c_scl_oe ? 1'b0 : 1'bz;

    assign IO_cam_sda = S_hw_i2c_sda_oe ? 1'b0 : 1'bz;

    ///fan
    assign fan_pwm = 1'b1;

    //=========================================================================
    //  已删除：RISC-V 软核（soft_ps_wrapper）与 APB 用户寄存器（apd_to_register）
    //
    //  理由：软核跑的是 SC2210 的 8KB 机器码固件（TCM0_INITFILE =
    //  SC2210_HDMI_1080P.bin.mif），没有源码、改不动，而 IMX415 的寄存器初始化
    //  现在完全由 imx415_cfg_fsm + imx415_cfg_rom 这套硬件引擎负责。
    //  它原本经 APB 提供 AWB 增益 / idelay / 黑电平 / TPG 使能，实测这几项
    //  在顶层要么被硬接成常量、要么是死代码，所以删除后功能没有实际损失：
    //     AWB 增益   -> 由 ISP 链里的 ui_awb 自动统计（阶段 3b 接入）
    //     idelay     -> 本来就是 localparam IDELAY_* 常量
    //     黑电平     -> 复位默认值就是 0，改成上面的 CAM_BLACK_LEVEL 常量
    //     TPG 使能   -> 死代码（S_hdmi_tpg_en 在顶层无人使用，uitpg.v 也没进编译表）
    //  一并删除的还有 UART/JTAG 端口与它们在 pin.adc 里的约束。
    //  好处：省掉约 1MB 的门级网表、一个 32KB TCM 的 ERAM 簇和一个 GCLK，
    //        对后续 layout 的拥塞与时序都是净收益。
    //=========================================================================


localparam  AXI_ADDR_WIDTH = `APP_ADDR_WIDTH +2;
localparam  AXI_DATA_WIDTH = `DRAM_BYTE_NUM*8*8;

    // --- FDMA 三组网：f_* = FDMA 侧，fdma_* = uidbuf 侧，ph_* = 拍照侧 ---
    wire [AXI_ADDR_WIDTH-1:0] f_fdma_waddr, f_fdma_raddr;
    wire        f_fdma_wareq, f_fdma_rareq, f_fdma_wbusy, f_fdma_rbusy;
    wire [15:0] f_fdma_wsize, f_fdma_rsize;
    wire [AXI_DATA_WIDTH-1:0] f_fdma_wdata, f_fdma_rdata;
    wire        f_fdma_wvalid, f_fdma_rvalid;

    wire [AXI_ADDR_WIDTH-1:0] ph_fdma_waddr, ph_fdma_raddr;
    wire        ph_fdma_wareq, ph_fdma_rareq, ph_fdma_wbusy, ph_fdma_rbusy;
    wire [15:0] ph_fdma_wsize, ph_fdma_rsize;
    wire [AXI_DATA_WIDTH-1:0] ph_fdma_wdata;
    wire        ph_fdma_wvalid, ph_fdma_rvalid;

wire                             O_axi_clk;
wire                             pll_locked;

    // 拍照系统的 AXI/SD 侧统一复位：含 S_pll_lock（FDMA 的 I_sys_rstn 就是它），
    // 保证拍照侧复位释放不早于 FDMA；回读模块要求两个复位同源同释放，这里用同一根。
    wire photo_axi_rst_n = S_pll_lock & pll_locked & S_board_reset_n;

//例化APP接口转AXI接口
uifdma_axi_ddr#
(
.SIMULATION      ("TRUE")   ,//"TRUE"   
.APP_ADDR_WIDTH  (`APP_ADDR_WIDTH)     ,  // 128 x 16bit                
.AXI_ADDR_WIDTH  (AXI_ADDR_WIDTH)     ,                                     
.AXI_DATA_WIDTH  (`DRAM_BYTE_NUM*8*8) ,
.APP_DATA_WIDTH  (`DRAM_BYTE_NUM*8*8) ,
.APP_MASK_WIDTH  (`DRAM_BYTE_NUM * 8)  
)
u_uifdma_axi_ddr
(
.I_ddr_clk                       ( I_sys_clk_25m    ),
.I_sys_rstn                      ( ~S_rst         ),
.O_ddr_pll_locked                ( pll_locked   ),

// DDR signals
.ddr_reset_n                     ( ddr_reset_n  ),
.ddr_addr                        ( ddr_addr     ),
.ddr_ba                          ( ddr_ba       ),
.ddr_ck_p                        ( ddr_ck_p     ),
.ddr_ck_n                        ( ddr_ck_n     ),
.ddr_cke                         ( ddr_cke      ),
.ddr_cs_n                        ( ddr_cs_n     ),
.ddr_ras_n                       ( ddr_ras_n    ),
.ddr_cas_n                       ( ddr_cas_n    ),
.ddr_we_n                        ( ddr_we_n     ),
.ddr_odt                         ( ddr_odt      ),
.ddr_dqs_p                       ( ddr_dqs_p    ),
.ddr_dqs_n                       ( ddr_dqs_n    ),
.ddr_dq                          ( ddr_dq       ),
.ddr_dm                          ( ddr_dm       ),

.O_uart_txd                      (O_uart_txd    ),
.I_uart_rxd                      (I_uart_rxd    ),

.O_axi_clk                       (O_axi_clk     ),

.I_fdma_waddr                    (f_fdma_waddr)  ,// FDMA 写通道地址（仲裁器输出）
.I_fdma_wareq                    (f_fdma_wareq)  ,// FDMA 写通道请求
.I_fdma_wsize                    (f_fdma_wsize)  ,// FDMA 写通道一次传输大小
.O_fdma_wbusy                    (f_fdma_wbusy)  ,// FDMA BUSY（写）
.I_fdma_wdata                    (f_fdma_wdata)  ,// FDMA 写数据
.O_fdma_wvalid                   (f_fdma_wvalid) ,// FDMA 写有效
.I_fdma_wready                   (1'b1)         ,// FDMA 写准备好
.I_fdma_raddr                    (f_fdma_raddr)  ,// FDMA 读通道地址
.I_fdma_rareq                    (f_fdma_rareq)  ,// FDMA 读通道请求
.I_fdma_rsize                    (f_fdma_rsize)  ,// FDMA 读通道一次传输大小
.O_fdma_rbusy                    (f_fdma_rbusy)  ,// FDMA BUSY（读）
.O_fdma_rdata                    (f_fdma_rdata)  ,// FDMA 读数据
.O_fdma_rvalid                   (f_fdma_rvalid) ,// FDMA 读有效
.I_fdma_rready                   (1'b1)          // FDMA 读准备好
);


 

    // 模式在 AXI 域 2FF 同步；150MHz 域的 I_R_en/I_R_tready 直接用 S_ui_hidden。
    assign S_ui_hidden = ~S_ui_visible;
    wire hidden_axi = hidden_axi_sync[1];
    always @(posedge O_axi_clk or negedge photo_axi_rst_n) begin
        if (!photo_axi_rst_n) hidden_axi_sync <= 2'b00;
        else hidden_axi_sync <= {hidden_axi_sync[0], S_ui_hidden};
    end

    // ★ 2026-11-01 只写正在显示的那一路 dbuf。
    //   原来两路都硬接 I_W_en=1'b1：缩小页面时 dbuf1 仍在写全分辨率
    //   （约 497 MB/s），全屏时 dbuf0 仍在写（约 286 MB/s），属于无谓的带宽开销。
    //   写侧又永远不反压（uidbuf 里 O_W_tready=1'b1），FIFO 一满就静默丢像素
    //   → 整行左移，丢多少随帧变 → 画面错位而且每帧在抖。
    //   暖机：复位后约 80 ms 内两路都写，保证切过去的缓冲区里是有效画面，
    //   而不是 DDR 里的随机内容。
    reg [24:0] wr_warm;
    wire       wr_warm_done = wr_warm[24];
    always @(posedge O_axi_clk or negedge photo_axi_rst_n) begin
        if (!photo_axi_rst_n)      wr_warm <= 25'd0;
        else if (!wr_warm_done)    wr_warm <= wr_warm + 25'd1;
    end
    wire S_wr0_en = (!hidden_axi) | (!wr_warm_done);
    wire S_wr1_en = ( hidden_axi) | (!wr_warm_done);

    //=========================================================================
    //  帧缓存控制器：dbuf0 = 1456×819（可见），dbuf1 = 1920×1080（全屏）
    //
    //  两路都持续写；DDR 三帧。dbuf1 的 1920 三帧放在 dbuf0 第一个 32MB
    //  槽位的空隙里（5MB 起，每帧 8.294MB，最后一帧止于 30.07MB），
    //  不增加 DDR 容量，也不碰拍照区 0x0600_0000。
    //=========================================================================
    ui_dbuf #(
    .AXI_DATA_WIDTH (AXI_DATA_WIDTH),
    .AXI_ADDR_WIDTH (AXI_ADDR_WIDTH),
    .ENABLE_VSYNC   (1),
    .W_BUFDEPTH     (256),
    .W_DATAWIDTH    (128),
    .W_BASEADDR     (0),
    .W_DSIZEBITS    (25),
    .W_XSIZE        (364),
    .W_XSTRIDE      (364),
    .W_YSIZE        (819),
    .W_XDIV         (2),
    .W_BUFSIZE      (3),

    .R_BUFDEPTH     (1024),
    .R_DATAWIDTH    (32),
    .R_BASEADDR     (0),
    .R_DSIZEBITS    (25),
    .R_XSIZE        (1456),
    .R_XSTRIDE      (1456),
    .R_YSIZE        (819),
    .R_XDIV         (2),
    .R_BUFSIZE      (3)
    )
    uidbuf_u0
    (
    .I_ui_clk      (O_axi_clk),
    .I_ui_rstn     (pll_locked),

    .I_W_en        (S_wr0_en),   // ★ 只在该路被显示时才写（见上方 S_wr0_en）
    .I_W_wclk      (S_hs_rx_clk),
    .I_W_tuser     (S_fit_tuser),
    .I_W_tvalid    (S_fit_tvalid),
    .I_W_tdata     (S_fit_tdata),
    .I_W_tlast     (S_fit_tlast),
    .O_W_tready    (),
    .O_W_sync_cnt  (wbuf_sync_0),
    .O_wdrop_cnt   (S_wdrop0),
    .I_W_buf       (wbuf_sync_0),

    .I_R_en        (S_r_en_0),
    .I_R_rclk      (S_hdmi_clk),
    .I_R_tready    (S_r_ready_0),
    .O_R_tuser     (),
    .O_R_tvalid    (S_hdmi_pixel_valid_0),
    .O_R_tdata     (ud_rdata_0),
    .O_R_tlast     (),
    .O_R_sync_cnt  (),
    .I_R_buf       (rbuf_sync_0),
    .O_R_vrst      (S_R_vrst),

    .O_fdma_waddr  (v0_waddr),
    .O_fdma_wareq  (v0_wareq),
    .O_fdma_wsize  (v0_wsize),
    .I_fdma_wbusy  (v0_wbusy),
    .O_fdma_wdata  (v0_wdata),
    .I_fdma_wvalid (v0_wvalid),
    .O_fdma_wready (),
    .O_fdma_raddr  (v0_raddr),
    .O_fdma_rareq  (v0_rareq),
    .O_fdma_rsize  (v0_rsize),
    .I_fdma_rbusy  (v0_rbusy),
    .I_fdma_rdata  (f_fdma_rdata),
    .I_fdma_rvalid (v0_rvalid),
    .O_fdma_rready (),
    .O_fdma_wbuf   (),
    .O_fdma_wirq   (),
    .O_fdma_rbuf   (),
    .O_fdma_rirq   ()
    );

    ui_dbuf #(
    .AXI_DATA_WIDTH (AXI_DATA_WIDTH),
    .AXI_ADDR_WIDTH (AXI_ADDR_WIDTH),
    .ENABLE_VSYNC   (1),
    .W_BUFDEPTH     (512),
    .W_DATAWIDTH    (128),
    .W_BASEADDR     (29'h0050_0000),
    .W_DSIZEBITS    (23),
    .W_XSIZE        (480),
    .W_XSTRIDE      (480),
    .W_YSIZE        (1080),
    .W_XDIV         (2),
    .W_BUFSIZE      (3),

    .R_BUFDEPTH     (1024),
    .R_DATAWIDTH    (32),
    .R_BASEADDR     (29'h0050_0000),
    .R_DSIZEBITS    (23),
    .R_XSIZE        (1920),
    .R_XSTRIDE      (1920),
    .R_YSIZE        (1080),
    .R_XDIV         (4),
    .R_BUFSIZE      (3)
    )
    uidbuf_u1
    (
    .I_ui_clk      (O_axi_clk),
    .I_ui_rstn     (pll_locked),

    .I_W_en        (S_wr1_en),   // ★ 只在该路被显示时才写（见上方 S_wr1_en）
    .I_W_wclk      (S_hs_rx_clk),
    .I_W_tuser     (S_isp_tuser),
    .I_W_tvalid    (S_isp_tvalid),
    .I_W_tdata     (S_isp_tdata),
    .I_W_tlast     (S_isp_tlast),
    .O_W_tready    (),
    .O_W_sync_cnt  (wbuf_sync_1),
    .O_wdrop_cnt   (S_wdrop1),
    .I_W_buf       (wbuf_sync_1),

    .I_R_en        (S_r_en_1),
    .I_R_rclk      (S_hdmi_clk),
    .I_R_tready    (S_r_ready_1),
    .O_R_tuser     (),
    .O_R_tvalid    (S_hdmi_pixel_valid_1),
    .O_R_tdata     (ud_rdata_1),
    .O_R_tlast     (),
    .O_R_sync_cnt  (),
    .I_R_buf       (rbuf_sync_1),
    .O_R_vrst      (),

    .O_fdma_waddr  (v1_waddr),
    .O_fdma_wareq  (v1_wareq),
    .O_fdma_wsize  (v1_wsize),
    .I_fdma_wbusy  (v1_wbusy),
    .O_fdma_wdata  (v1_wdata),
    .I_fdma_wvalid (v1_wvalid),
    .O_fdma_wready (),
    .O_fdma_raddr  (v1_raddr),
    .O_fdma_rareq  (v1_rareq),
    .O_fdma_rsize  (v1_rsize),
    .I_fdma_rbusy  (v1_rbusy),
    .I_fdma_rdata  (f_fdma_rdata),
    .I_fdma_rvalid (v1_rvalid),
    .O_fdma_rready (),
    .O_fdma_wbuf   (),
    .O_fdma_wirq   (),
    .O_fdma_rbuf   (),
    .O_fdma_rirq   ()
    );

    // 3 帧缓存，读延迟写 1 帧（两路各一组）
    uisetvbuf#(.BUF_DELAY(1), .BUF_LENTH(3)) uisetvbuf_0 (
    .bufn_i(wbuf_sync_0),
    .bufn_o(rbuf_sync_0)
    );

    uisetvbuf#(.BUF_DELAY(1), .BUF_LENTH(3)) uisetvbuf_1 (
    .bufn_i(wbuf_sync_1),
    .bufn_o(rbuf_sync_1)
    );

    //=========================================================================
    //  拍照存 SD 卡：仲裁器 + 抓帧 + 回读 + 总状态机
    //
    //  photo_fsm 全在 25MHz 域；抓帧/回读跨相机域与 AXI 域；SD 四脚在 25MHz 域。
    //  FDMA 读写各自分时复用：写侧拍照优先，读侧 dbuf 优先 + 饥饿保护。
    //=========================================================================
    wire photo_cap_start_tgl, photo_rd_start_tgl;
    wire photo_cap_dma_done_tgl, photo_cap_overflow_tgl, photo_cap_axi_idle;
    wire photo_src_valid, photo_src_ready;
    wire [127:0] photo_src_data;
    wire photo_led;
    wire [3:0] photo_status;            // 25MHz one-hot，去 150MHz 角标（photo_st_s2）
    wire S_capture_req;

    photo_fdma_arb u_photo_arb (
        .I_clk(O_axi_clk), .I_rst_n(photo_axi_rst_n),
        .I_ub_waddr(vid_waddr), .I_ub_wareq(vid_wareq), .I_ub_wsize(vid_wsize),
        .I_ub_wdata(vid_wdata), .O_ub_wbusy(vid_wbusy), .O_ub_wvalid(vid_wvalid),
        .I_ub_raddr(act_raddr), .I_ub_rareq(act_rareq), .I_ub_rsize(act_rsize),
        .O_ub_rbusy(act_rbusy), .O_ub_rvalid(act_rvalid),
        .I_ph_waddr(ph_fdma_waddr), .I_ph_wareq(ph_fdma_wareq), .I_ph_wsize(ph_fdma_wsize),
        .I_ph_wdata(ph_fdma_wdata), .O_ph_wbusy(ph_fdma_wbusy), .O_ph_wvalid(ph_fdma_wvalid),
        .I_ph_raddr(ph_fdma_raddr), .I_ph_rareq(ph_fdma_rareq), .I_ph_rsize(ph_fdma_rsize),
        .O_ph_rbusy(ph_fdma_rbusy), .O_ph_rvalid(ph_fdma_rvalid),
        .O_waddr(f_fdma_waddr), .O_wareq(f_fdma_wareq), .O_wsize(f_fdma_wsize),
        .O_wdata(f_fdma_wdata), .I_wbusy(f_fdma_wbusy), .I_wvalid(f_fdma_wvalid),
        .O_raddr(f_fdma_raddr), .O_rareq(f_fdma_rareq), .O_rsize(f_fdma_rsize),
        .I_rbusy(f_fdma_rbusy), .I_rvalid(f_fdma_rvalid)
    );

    //=========================================================================
    //  视频写仲裁（1456 dbuf / 1920 dbuf 轮询）+ 读侧模式 mux
    //=========================================================================
    video_fdma_arb u_video_arb (
        .I_clk(O_axi_clk), .I_rst_n(photo_axi_rst_n),
        .I_v0_waddr(v0_waddr), .I_v0_wareq(v0_wareq), .I_v0_wsize(v0_wsize), .I_v0_wdata(v0_wdata),
        .O_v0_wbusy(v0_wbusy), .O_v0_wvalid(v0_wvalid),
        .I_v1_waddr(v1_waddr), .I_v1_wareq(v1_wareq), .I_v1_wsize(v1_wsize), .I_v1_wdata(v1_wdata),
        .O_v1_wbusy(v1_wbusy), .O_v1_wvalid(v1_wvalid),
        .O_waddr(vid_waddr), .O_wareq(vid_wareq), .O_wsize(vid_wsize), .O_wdata(vid_wdata),
        .I_wbusy(vid_wbusy), .I_wvalid(vid_wvalid)
    );

    assign act_raddr = hidden_axi ? v1_raddr : v0_raddr;
    assign act_rareq = hidden_axi ? v1_rareq : v0_rareq;
    assign act_rsize = hidden_axi ? v1_rsize : v0_rsize;
    assign v0_rbusy  = hidden_axi ? 1'b0 : act_rbusy;
    assign v1_rbusy  = hidden_axi ? act_rbusy : 1'b0;
    assign v0_rvalid = hidden_axi ? 1'b0 : act_rvalid;
    assign v1_rvalid = hidden_axi ? act_rvalid : 1'b0;

    assign S_r_en_0    = S_ui_hidden ? 1'b0 : vtc_vs;
    assign S_r_en_1    = S_ui_hidden ? vtc_vs : 1'b0;
    assign S_r_ready_0 = S_ui_hidden ? 1'b0 : S_vis_ready;
    assign S_r_ready_1 = S_ui_hidden ? vtc_de_valid : 1'b0;

    assign S_cam_rgb = S_ui_hidden ? ud_rdata_1[23:0] : ud_rdata_0[23:0];
    assign S_cam_pixel_valid = S_ui_hidden ? S_hdmi_pixel_valid_1 : S_hdmi_pixel_valid_0;

    ui_fit_read_gate u_ui_fit_read_gate (
        .I_clk(S_hdmi_clk), .I_rst_n(S_pll_lock),
        .I_vs(vtc_vs), .I_de(vtc_de_valid), .O_ready(S_vis_ready)
    );


    // 拍照抓帧进度观测接口：把 burst 计数同步到显示域，供在线调试观测。
    // 该通路为只读观测，不参与拍照与显示的任何功能判断。
    wire [7:0] photo_dbg_burst;
    reg  [7:0] dbg_s1, dbg_s2;   //synthesis keep

    photo_capture u_photo_capture (
        // 相机侧复位含 pll_locked：保证它不会比 AXI 侧更早释放；
        // 单侧掉锁由 photo_fsm 的「失锁 16 拍 -> S_ERROR」兜底
        .I_cap_clk(S_hs_rx_clk), .I_cap_rstn(S_pll_lock & pll_locked),
        .I_start_tgl(photo_cap_start_tgl),
        .I_isp_tuser(S_isp_tuser), .I_isp_tvalid(S_isp_tvalid), .I_isp_tdata(S_isp_tdata),
        .O_frame_done_tgl(),            // 故意不接：成功看 dma_done，失败看 overflow
        .O_overflow_tgl(photo_cap_overflow_tgl),
        .I_axi_clk(O_axi_clk), .I_axi_rstn(photo_axi_rst_n),
        .O_waddr(ph_fdma_waddr), .O_wareq(ph_fdma_wareq),
        .O_wsize(ph_fdma_wsize), .O_wdata(ph_fdma_wdata),
        .I_wbusy(ph_fdma_wbusy), .I_wvalid(ph_fdma_wvalid),
        .O_dma_done_tgl(photo_cap_dma_done_tgl),
        .O_axi_idle(photo_cap_axi_idle),
        .O_dbg_burst(photo_dbg_burst)   // 抓帧进度观测
    );

    photo_readback u_photo_readback (
        .I_axi_clk(O_axi_clk), .I_axi_rstn(photo_axi_rst_n),
        .I_start_tgl(photo_rd_start_tgl),
        .O_raddr(ph_fdma_raddr), .O_rareq(ph_fdma_rareq), .O_rsize(ph_fdma_rsize),
        .I_rdata(f_fdma_rdata), .I_rvalid(ph_fdma_rvalid), .I_rbusy(ph_fdma_rbusy),
        .O_dma_done_tgl(),              // 不参与判据：块数才是完成条件（见 photo_fsm 的进度看门狗）
        .I_sd_clk(I_sys_clk_25m), .I_sd_rstn(photo_axi_rst_n),
        .O_src_valid(photo_src_valid), .O_src_data(photo_src_data),
        .I_src_ready(photo_src_ready)
    );

    photo_fsm #(
        // ★ PHOTO_BASE_LBA 是 photo_fsm.v 的参数默认值（单点真值，Task 10 填）
        .SLOT_SECTORS     (12288),
        .BLOCKS_PER_PHOTO (12151)
    ) u_photo_fsm (
        .I_clk(I_sys_clk_25m), .I_rst_n(S_board_reset_n), .I_pll_locked(pll_locked),
        .I_capture_req_tgl(S_capture_req),
        .I_src_valid(photo_src_valid), .I_src_data(photo_src_data), .O_src_ready(photo_src_ready),
        .O_cap_start_tgl(photo_cap_start_tgl),
        .I_cap_dma_done_tgl(photo_cap_dma_done_tgl),
        .I_cap_overflow_tgl(photo_cap_overflow_tgl),
        .I_cap_axi_idle(photo_cap_axi_idle),
        .O_rd_start_tgl(photo_rd_start_tgl),
        .I_sd_miso(I_sd_miso), .O_sd_clk(O_sd_clk), .O_sd_cs(O_sd_cs), .O_sd_mosi(O_sd_mosi),
        .O_led(photo_led), .O_status(photo_status)
    );

    // LED 高电平点亮（原理图 P14：VDD3.3V→2.2K→LED→MMBT3904，基极经 4.7K 接 T6）
    assign IO_led = S_cam_cfg_done ? (photo_led ? 1'b1 : 1'b0) : 1'b0;


    //=========================================================================
    //  读侧：DDR RGB -> 独立两键 UI/OSD -> HDMI
    //
    //  删掉的是旧链路的四拍拆像素逻辑（ud_rdata_cnt / ud_rdata_d1 /
    //  vtc_de_valid_d1 / ud_rfifo_en）+ raw8_2_rgb888（3x3 去马赛克）+
    //  awb（增益被硬接成 0）+ bound_cut（边界裁剪）。
    //
    //  原因：整条 ISP 链现在搬到了 DDR **写侧**（ui_isp_top 在 ui_dbuf 之前），
    //  存进 DDR 的已经是 4 像素/拍的 RGB32，读侧按原始 VTC 节奏取像素，
    //  然后叠加 src/hdmi_filter_ui.v 的 OSD。UI 不改变 DDR 的读时序。
    //  这也顺带消掉了旧读侧按 4 拍串行化的时序 hack。
    //=========================================================================

    uivtc#
    (
    .H_ActiveSize       (1920), //视频时间参数,行视频信号，一行有效(需要显示的部分)像素所占的时钟数，一个时钟对应一个有效像素
    .H_SyncStart        (1920+88), //视频时间参数,行同步开始，即多少时钟数后开始产生行同步信号 
    .H_SyncEnd          (1920+88+44),//视频时间参数,行同步结束，即多少时钟数后停止产生行同步信号，之后就是行有效数据部分
    .H_FrameSize        (1920+88+44+148), //视频时间参数,行视频信号，一行视频信号总计占用的时钟数

    .V_ActiveSize       (1080),//视频时间参数,场视频信号，一帧图像所占用的有效(需要显示的部分)行数量，通常说的视频分辨率即H_ActiveSize*V_ActiveSize
    .V_SyncStart        (1080+4),//视频时间参数,场同步开始，即多少行数后开始产生场同步信号 
    .V_SyncEnd          (1080+4+5), //视频时间参数,场同步结束，多少行后停止产生长同步信号  
    .V_FrameSize        (1080+4+5+36) //视频时间参数,场视频信号，一帧视频信号总计占用的行数量       
    )
    uivtc_inst
    (
    .I_vtc_clk(vtc_clk),  //系统时钟 
    .I_vtc_rstn(vtc_rstn),//系统复位
    .O_vtc_vs  (vtc_vs),//场同步输出
    .O_vtc_hs  (vtc_hs),//行同步输出
    .O_vtc_de_valid(vtc_de_valid)//视频数据有效
    );

    // Dedicated OSD: raw VTC DE tracks coordinates even when DDR has no pixel.
    // DDR read pacing still uses the original VTC signals; never delayed OSD DE.
    wire [23:0] S_ui_rgb;
    wire S_ui_vs, S_ui_hs, S_ui_de;

    // 帧率：滤镜域实测值 -> 显示域（两级同步）
    wire [7:0] S_isp_fps;
    reg  [7:0] fps_s1, fps_s2;
    wire [2:0] S_ui_mode;
    wire [7:0] S_ui_strength, S_ui_param1, S_ui_param2, S_ui_smooth, S_ui_split;
    wire [7:0] S_ui_he;
    reg  [7:0] f_he_s1, f_he_s2;
    wire S_ui_compare;
    wire [3:0] S_ui_focus;
    wire S_ui_page;
    wire [7:0] S_ae_mean, S_ae_peak;
    wire       S_ae_valid;
    wire [7:0] S_ae_gain;
    wire [7:0] S_ae_nack_cnt, S_ae_wr_cnt;
    reg [1:0] S_ui_reset_sync;
    always @(posedge S_hdmi_clk or negedge S_pll_lock) begin
        if (!S_pll_lock) S_ui_reset_sync <= 2'b00;
        else S_ui_reset_sync <= {S_ui_reset_sync[0], 1'b1};
    end

    hdmi_filter_ui #(.CLK_HZ(150000000)) u_hdmi_filter_ui (
        .I_clk(S_hdmi_clk), .I_rst_n(S_ui_reset_sync[1]),
        .I_key_n({I_key_next_n, I_key_in}),
        .I_vs(vtc_vs), .I_hs(vtc_hs), .I_de(vtc_de_valid),
        .I_pixel_valid(S_cam_pixel_valid), .I_rgb(S_cam_rgb),
        .I_fps(fps_s2),    // ★ OSD 显示真实帧率
        .I_ae_mean(S_ae_mean),
        .I_ae_gain(S_ae_gain),
        .I_ae_nack(S_ae_nack_cnt),
        .I_delay_ms10(S_delay_ms10), .I_delay_valid(S_delay_valid),
        .O_vs(S_ui_vs), .O_hs(S_ui_hs), .O_de(S_ui_de), .O_rgb(S_ui_rgb),
        .O_mode(S_ui_mode), .O_strength(S_ui_strength),
        .O_param1(S_ui_param1), .O_param2(S_ui_param2), .O_smooth(S_ui_smooth),
        .O_he_strength(S_ui_he),
        .O_compare_en(S_ui_compare), .O_split(S_ui_split),
        .O_menu_visible(S_ui_visible), .O_focus(S_ui_focus), .O_page(S_ui_page),
        .O_capture_req(S_capture_req)
    );

    //=========================================================================
    //  拍照状态角标：25MHz one-hot -> 150MHz（逐位 2FF），叠在 UI 输出之后
    //  整块（含下面的 2FF 同步器）都在 `ifdef PHOTO_BADGE 内：宏关掉时
    //  photo_status 也没有消费者，综合能把整条 25MHz 状态通路一起裁掉。
    //=========================================================================
    // hs 不进 hdmi_tx（hdmi_tx.v 的 RGB 通路只有 vs/de/data 三个输入），
    // 所以这里不引出 S_ui_hs_b —— 引了也是一根死线，还会让 ifdef 两支
    // 的"延迟不同"看起来有意义（其实没人读）。
    wire [23:0] S_ui_rgb_b;
    wire        S_ui_vs_b, S_ui_de_b;
`ifdef PHOTO_BADGE
    reg [3:0] photo_st_s1, photo_st_s2;
    always @(posedge S_hdmi_clk or negedge S_ui_reset_sync[1]) begin
        if (!S_ui_reset_sync[1]) begin photo_st_s1 <= 4'd0; photo_st_s2 <= 4'd0; end
        else begin photo_st_s1 <= photo_status; photo_st_s2 <= photo_st_s1; end
    end

    photo_badge #(.BLINK_BIT(25)) u_photo_badge (
        .I_clk(S_hdmi_clk), .I_rst_n(S_ui_reset_sync[1]),
        .I_status(photo_st_s2),
        .I_full(~S_ui_visible),
        .I_vs(S_ui_vs), .I_hs(S_ui_hs), .I_de(S_ui_de), .I_rgb(S_ui_rgb),
        .O_vs(S_ui_vs_b), .O_hs(), .O_de(S_ui_de_b), .O_rgb(S_ui_rgb_b)
    );
`else
    assign S_ui_rgb_b = S_ui_rgb;
    assign S_ui_vs_b  = S_ui_vs;
    assign S_ui_de_b  = S_ui_de;
`endif

    //hdmi 输出IP
    hdmi_tx#(
    //HDMI视频参数设置       
    .H_ActiveSize       (1920), //视频时间参数,行视频信号，一行有效(需要显示的部分)像素所占的时钟数，一个时钟对应一个有效像素
    .H_SyncStart        (1920+88), //视频时间参数,行同步开始，即多少时钟数后开始产生行同步信号 
    .H_SyncEnd          (1920+88+44),//视频时间参数,行同步结束，即多少时钟数后停止产生行同步信号，之后就是行有效数据部分
    .H_FrameSize        (1920+88+44+148), //视频时间参数,行视频信号，一行视频信号总计占用的时钟数

    .V_ActiveSize       (1080),//视频时间参数,场视频信号，一帧图像所占用的有效(需要显示的部分)行数量，通常说的视频分辨率即H_ActiveSize*V_ActiveSize
    .V_SyncStart        (1080+4),//视频时间参数,场同步开始，即多少行数后开始产生场同步信号 
    .V_SyncEnd          (1080+4+5), //视频时间参数,场同步结束，多少行后停止产生长同步信号  
    .V_FrameSize        (1080+4+5+36),  //视频时间参数,场视频信号，一帧视频信号总计占用的行数量               
        
    .VIDEO_VIC          ( 16       ),
    .VIDEO_TPG          ( "Disable"),//设置disable，用户数据驱动HDMI接口，否则设置eable产生内部测试图形
    .VIDEO_FORMAT       ( "RGB444" )//设置输入数据格式为RGB格式
    )u_hdmi_tx
    (
    .I_pixel_clk        ( S_hdmi_clk           ),//像素时钟
    .I_serial_clk       ( S_serial_clk           ),//串行发送时钟
    //  ★ 这里**只接 S_rst（上电复位）**，不接 S_R_vrst。
    //    S_R_vrst 来自 ui_dbuf 里像素域的帧延迟计数器（dl_cnt），若拿它复位 HDMI，
    //    就会造出一条 "像素域计数器 -> 750MHz 串行域输出寄存器复位脚" 的跨域
    //    recovery 路径，预算只有 750MHz 的一个周期 1.333ns，实测 −449ps 收不住
    //    （见 phy_1/final_timing.rpt 的 S_hdmi_clk -> S_serial_clk 组）。
    //    HDMI 只做 TMDS 编码与串并转换，没有需要按帧对齐的内部状态，
    //    帧同步由 VTC 负责（下面 vtc_rstn 那条），所以这里不需要它。
    .I_rst              ( S_rst              ),//异步复位信号，高电平有效（仅上电）

    .I_video_rgb_enable (1'b1                ),//是否使能RGB输入接口，设置1使能，否则采用stream video时序接口  
    .I_video_in_vs      (S_ui_vs_b           ),//OSD + 拍照角标，同步与 RGB 同延迟
    .I_video_in_de      (S_ui_de_b           ),//full active raster, black fallback on DDR underflow
    .I_video_in_data    (S_ui_rgb_b          ),//RGB with two-key OSD + 拍照角标

    .O_hdmi_clk_p       ( O_hdmi_clk_p       ),//HDMI时钟通道
    .O_hdmi_tx_p        ( O_hdmi_tx_p        )//HDMI数据通道
    );


    //=========================================================================
    //  MIPI RAW10 解包 + 裁剪 + 完整 ISP 链（写在 DDR 之前）
    //
    //  数据流：csi_unpacket -> ui_raw10_unpacket_4lane(5:4 齿轮箱, 40bit)
    //        -> ui_image_correction(裁到 480 字/行 x 1080 行)
    //        -> ui_isp_top(整条 ISP) -> ui_dbuf
    //
    //  ★ 为什么换掉原来的 raw10_unpacket + video_cuttting：
    //    新解包是"5 个输入字出 4 个输出字"的齿轮箱，每 5 拍里有 1 拍 valid 空档；
    //    老的 video_cuttting 用 valid 的下降沿数行，会被这个空档骗到，行数跑飞。
    //    ui_image_correction 用"连续 10 拍无 valid"判行尾，正好吃得住这个空档。
    //  ★ 裁剪窗口：丢每行第 1 个 40bit 字（4 像素，偶数 -> Bayer 列相位不变）、
    //    丢第 1 行（帧起始后的空隙让 v_cnt 先自增一次 -> 行相位不变）。
    //    每行恰好 480 字 = 1920 像素，已由 tb_isp_chain.v 与 tb_write_path.v
    //    双重钉死（每帧 518400 个 128bit 字）。
    //=========================================================================
    ui_raw10_unpacket_4lane u_raw10_unpacket(
        .I_clk              ( S_hs_rx_clk        ),

        .I_rst_n            ( ~S_rst             ),

        .I_csi_frame_start  ( S_csi_frame_start  ),
        .I_csi_frame_end    ( S_csi_frame_end    ),
        .I_csi_valid        ( S_csi_valid        ),
        .I_csi_data         ( S_csi_data         ),

        .O_raw10_frame_start( S_r10_fs           ),
        .O_raw10_frame_end  ( S_r10_fe           ),
        .O_raw10_valid      ( S_r10_valid        ),
        .O_raw10_data       ( S_r10_data         )
    );

    ui_image_correction #(
        .DATA_WIDTH         (40),
        .TDEST_WIDTH        (10)
    ) u_image_correction (
        .I_clk              ( S_hs_rx_clk        ),
        .I_rst_n            ( ~S_rst             ),

        .I_raw_data         ( S_r10_data         ),
        .I_raw_valid        ( S_r10_valid        ),
        .I_raw_frame_start  ( S_r10_fs           ),
        .I_raw_frame_end    ( S_r10_fe           ),
        .I_line_start       ( S_csi_rx_hsync0     ),

        .O_raw_tdata        ( S_raw_tdata        ),
        .O_raw_tlast        ( S_raw_tlast        ),
        .O_raw_tdest        ( S_raw_tdest        ),
        .O_raw_tvalid       ( S_raw_tvalid       ),
        .O_raw_tuser        ( S_raw_tuser        ),
        .O_raw_tready       ( 1'b1               ),
        .O_pad_cnt          ( S_corr_pad_cnt     ),
        .O_miss_cnt         ( S_corr_miss_cnt    ),
        .O_gap_max          ( S_corr_gap_max     ),
        .O_gap_min          ( S_corr_gap_min     )
    );

    //  ★ ui_isp_top 的端口名是反的：O_tready 是输入、I_tready 是（模块内部从未驱动
    //    的）输出。参考工程把 .O_tready 接在一条未驱动的线上（= z），这里必须显式拉高；
    //    没用的 I_tdest 一并接 0，避免末级 x 传播。
    wire [2:0] S_filt_mode;
    wire [7:0] S_filt_p1, S_filt_p2, S_filt_p3, S_filt_p4, S_filt_p5;
    wire       S_filt_cmp;
    wire [7:0] S_filt_split;

    ui_isp_top u_isp_top (
        .I_clk              ( S_hs_rx_clk        ),
        .I_rst_n            ( ~S_rst             ),

        .I_tlast            ( S_raw_tlast        ),
        .I_tuser            ( S_raw_tuser        ),
        .I_tdata            ( S_raw_tdata        ),
        .I_tvalid           ( S_raw_tvalid       ),
        .I_tdest            ( S_raw_tdest        ),
        .O_tready           ( 1'b1               ),

        // ★ 滤镜域（PLL C1 = 75MHz）与运行时参数（已同步到相机域）
        .I_clk_flt          ( S_sys_clk          ),
        .I_mode             ( S_filt_mode        ),
        .I_param1           ( S_filt_p1          ),
        .I_param2           ( S_filt_p2          ),
        .I_param3           ( S_filt_p3          ),
        .I_param4           ( S_filt_p4          ),
        .I_param5           ( S_filt_p5          ),
        .I_compare          ( S_filt_cmp         ),
        .I_split            ( S_filt_split       ),
        .I_he_strength      ( f_he_s2            ),
        .O_ae_mean          ( S_ae_mean          ),
        .O_ae_peak          ( S_ae_peak          ),
        .O_ae_valid         ( S_ae_valid         ),

        .O_tdata            ( S_isp_tdata        ),
        .O_tlast            ( S_isp_tlast        ),
        .O_tuser            ( S_isp_tuser        ),
        .O_tvalid           ( S_isp_tvalid       ),
        .I_tready           (                    ),
        .O_fps              ( S_isp_fps          )
    );

    //=========================================================================
    //  ★ 2026-10-30 自动曝光第一阶段：测光（只测不控）
    //
    //  2026-10-31 起抽头移到 HE 之前（ui_isp_top 内部、saturation 输出），
    //  输出由 ui_isp_top 的 O_ae_mean/O_ae_peak/O_ae_valid 引出。
    //=========================================================================
    //=========================================================================
    //  帧率跨域：滤镜域 75MHz -> 显示域 150MHz
    //
    //  O_fps 每 1 秒才刷新一次，是标准的慢变量，两级同步足够。
    //=========================================================================
    //  OSD 头部的「帧率」显示滤镜域实测帧率（fps_s2）。
    always @(posedge S_hdmi_clk or negedge S_ui_reset_sync[1]) begin
        if (!S_ui_reset_sync[1]) begin
            fps_s1 <= 8'd0;
            fps_s2 <= 8'd0;
            dbg_s1 <= 8'd0;
            dbg_s2 <= 8'd0;
        end
        else begin
            fps_s1 <= S_isp_fps;
            fps_s2 <= fps_s1;
            dbg_s1 <= photo_dbg_burst;   // 抓帧进度同步到显示域
            dbg_s2 <= dbg_s1;            //synthesis keep
        end
    end

    //=========================================================================
    //  端到端延迟监测：帧进入（ISP/滤镜输出）到该帧在 HDMI 上显示
    //
    //  wbuf/rbuf 是 DDR 三缓冲的写/读缓冲号，时间戳按 buffer 号对应；
    //  I_cam_tuser 是 74.24MHz 域的单拍脉冲，wbuf 是 133MHz 域的慢变量，
    //  模块内部统一同步到 150MHz。当前只监测，尚未接 OSD。
    //=========================================================================
    delay_meter u_delay_meter (
        .I_clk       ( S_hdmi_clk         ),
        .I_rst_n     ( S_ui_reset_sync[1] ),
        .I_cam_tuser ( S_fit_tuser        ),
        .I_wbuf      ( wbuf_sync_0        ),
        .I_vs        ( vtc_vs             ),
        .I_rbuf      ( rbuf_sync_0        ),
        .O_delay_ms10( S_delay_ms10       ),
        .O_valid     ( S_delay_valid      )
    );

    //=========================================================================
    //  UI 参数跨域：S_hdmi_clk(150MHz) -> S_hs_rx_clk(74.25MHz)
    //
    //  UI 侧本来就在 VS 上升沿提交参数，是帧级慢变量，两级同步足够。
    //  模式映射：FPGA_al 0 原图/1 铅笔/2 浮雕/3 漫画/4 水墨
    //          -> 滤镜内部 0 原图 / 1 漫画 / 2 浮雕 / 3 铅笔
    //=========================================================================
    reg [2:0] f_mode_s1, f_mode_s2;
    reg [7:0] f_p1_s1, f_p1_s2, f_p2_s1, f_p2_s2, f_p3_s1, f_p3_s2;
    reg [7:0] f_p4_s1, f_p4_s2, f_p5_s1, f_p5_s2;
    reg       f_cmp_s1, f_cmp_s2;
    reg [7:0] f_spl_s1, f_spl_s2;

    always @(posedge S_hs_rx_clk or negedge S_board_reset_n) begin
        if (!S_board_reset_n) begin
            f_mode_s1 <= 3'd0; f_mode_s2 <= 3'd0;
            f_p1_s1 <= 8'd75;  f_p1_s2 <= 8'd75;
            f_p2_s1 <= 8'd6;   f_p2_s2 <= 8'd6;
            f_p3_s1 <= 8'd8;   f_p3_s2 <= 8'd8;
            f_p4_s1 <= 8'd20;  f_p4_s2 <= 8'd20;
            f_p5_s1 <= 8'd20;  f_p5_s2 <= 8'd20;
            f_cmp_s1 <= 1'b0;  f_cmp_s2 <= 1'b0;
            f_spl_s1 <= 8'd50; f_spl_s2 <= 8'd50;
            f_he_s1  <= 8'd60; f_he_s2  <= 8'd60;
        end
        else begin
            f_mode_s1 <= S_ui_mode;     f_mode_s2 <= f_mode_s1;
            f_p1_s1   <= S_ui_strength; f_p1_s2   <= f_p1_s1;
            f_p2_s1   <= S_ui_param1;   f_p2_s2   <= f_p2_s1;
            f_p3_s1   <= S_ui_param2;   f_p3_s2   <= f_p3_s1;
            f_p4_s1   <= S_ui_smooth;   f_p4_s2   <= f_p4_s1;
            f_p5_s1   <= 8'd20;         f_p5_s2   <= f_p5_s1;
            f_cmp_s1  <= S_ui_compare;  f_cmp_s2  <= f_cmp_s1;
            f_spl_s1  <= S_ui_split;    f_spl_s2  <= f_spl_s1;
            f_he_s1   <= S_ui_he;       f_he_s2   <= f_he_s1;
        end
    end

    // 5 模式 -> 内部滤镜号
    // ★ 2026-10-29：UI 顺序改为 原图 / 像素 / 浮雕 / 铅笔 / 漫画。
    //   必须与 src/hdmi_filter_ui.v 的 mode_to_filt() 完全一致，改一处就要改两处。
    assign S_filt_mode = (f_mode_s2 == 3'd1) ? 3'd4 :   // 像素（原水墨槽位）
                         (f_mode_s2 == 3'd2) ? 3'd2 :   // 浮雕
                         (f_mode_s2 == 3'd3) ? 3'd3 :   // 铅笔
                         (f_mode_s2 == 3'd4) ? 3'd1 :   // 漫画
                                               3'd0 ;    // 原图
    assign S_filt_p1    = f_p1_s2;
    assign S_filt_p2    = f_p2_s2;
    assign S_filt_p3    = f_p3_s2;
    assign S_filt_p4    = f_p4_s2;
    assign S_filt_p5    = f_p5_s2;
    assign S_filt_cmp   = f_cmp_s2;
    assign S_filt_split = f_spl_s2;

    //=========================================================================
    //  MIPI D-PHY 采样点（idelay）
    //
    //  综合期固定采样延迟，当前 IMX415 profile 使用 594 Mbps/lane。
    //
    //  ★ 为什么不接回 APB：apd_to_register 确实产生 S_data_lane*_idelay /
    //    S_clk_lane_idelay，但要写那些寄存器得靠软核固件，而固件是 8KB 机器码
    //    改不动。所以做成综合期常量：扫一次改一次常量、重建，约 10~20 分钟一轮。
    //    先全 0 试，不出图再逐个加大。
    //
    //  APUG076：512 级，每级约 5~8ps。
    //=========================================================================
    //  修正版 IMX415 工程的采样点：时钟 lane 192、数据 lane 20。
    //  APUG076：512 级，每级约 5~8ps。
    localparam [8:0] IDELAY_CLK = 9'd192;
    localparam [8:0] IDELAY_D0  = 9'd20;
    localparam [8:0] IDELAY_D1  = 9'd20;
    localparam [8:0] IDELAY_D2  = 9'd20;
    localparam [8:0] IDELAY_D3  = 9'd20;

    //=========================================================================
    //  IMX415 上电时序 + 硬件 I2C 配置引擎（本次新增）
    //
    //  数据流：
    //    imx415_power_seq（25MHz 域，PLL 之外）
    //        ├─ O_xclr_n   -> O_sc2210_rst_n                       (system clear)
    //        ├─ O_mclk_en  -> 门控 PLL C0 -> O_cam_clk_27m      (INCK)
    //        └─ O_cfg_start -> 2 级同步到 70MHz -> cfg_fsm
    //    imx415_cfg_fsm（70MHz 域）
    //        ├─ imx415_cfg_rom   : 201 条 {addr, data}
    //        └─ imx415_i2c_master -> IO_cam_scl / IO_cam_sda
    //=========================================================================

    imx415_power_seq #(
        .CLK_FREQ    ( 25000000 ),
        .XCLR_LOW_US ( 2        ),
        .INCK_DLY_US ( 3        ),
        .PWR_WAIT_MS ( 24       ),
        .GATE_MCLK   ( 1        )
    ) u_imx415_power_seq (
        .I_clk       ( I_sys_clk_25m       ),
        .I_rst_n     ( S_board_reset_n     ),   // independent POR; keys now operate the UI
        .I_pll_lock  ( S_pll_lock          ),
        .O_xclr_n    ( S_cam_xclr_n        ),
        .O_mclk_en   ( S_cam_mclk_en       ),
        .O_cfg_start ( S_cam_cfg_start_25m ),
        .O_state     ( S_cam_pwr_state     )
    );

    //---- 启动电平：25MHz 域 -> 70MHz 域（2 级同步器；两个时钟在 sdc 里已是异步组）----
    reg [1:0] S_cfg_start_sync;
    always @(posedge S_clk_70m or negedge S_pll_lock) begin
        if(!S_pll_lock) S_cfg_start_sync <= 2'b00;
        else            S_cfg_start_sync <= {S_cfg_start_sync[0], S_cam_cfg_start_25m};
    end
    assign S_cam_cfg_start = S_cfg_start_sync[1];

    //---- 相机 MCLK（INCK）门控 ----
    //  索尼手册 p84 第 3 条："The master clock input after setting the XCLR pin
    //  to High level" —— INCK 必须在 XCLR 拉高之后才给。
    //  PLL C0 一锁定就一直在跑，所以这里做门控：使能在 27MHz 的**下降沿**打拍，
    //  门控值只在时钟低电平期间变化，不会产生毛刺或窄脉冲。
    always @(negedge S_cam_clk_27m)
        S_mclk_en_sync <= {S_mclk_en_sync[0], S_cam_mclk_en};

    assign O_cam_clk_27m = S_cam_clk_27m & S_mclk_en_sync[1];

    //---- 配置表 ROM ----
    imx415_cfg_rom u_imx415_cfg_rom (
        .I_index ( S_cfg_rom_index ),
        .O_data  ( S_cfg_rom_data  )
    );

    //=========================================================================
    //  自动曝光（2026-10-30）：控制环 + I2C 写仲裁
    //    cfg 阶段：cfg_fsm 独占总线（S_cfg_i2c_* -> S_hw_i2c_*）
    //    cfg 之后：AE 有写请求时接管，平时总线仍归 cfg（cfg 此时已空闲）
    //=========================================================================
    wire        S_cfg_i2c_start;
    wire [6:0]  S_cfg_i2c_dev;
    wire [15:0] S_cfg_i2c_addr;
    wire [7:0]  S_cfg_i2c_data;

    // 约 60ms 一次的更新节拍（70MHz）
    reg [21:0] S_ae_tick_cnt;
    reg        S_ae_tick;
    always @(posedge S_clk_70m or negedge S_pll_lock) begin
        if (!S_pll_lock) begin S_ae_tick_cnt <= 22'd0; S_ae_tick <= 1'b0; end
        else begin
            S_ae_tick_cnt <= S_ae_tick_cnt + 22'd1;
            S_ae_tick <= (S_ae_tick_cnt == 22'd0);
        end
    end

    // 亮度值 75MHz -> 70MHz（每帧才变一次，两级同步足够）
    reg [7:0] S_ae_mean_s1, S_ae_mean_s2;
    always @(posedge S_clk_70m or negedge S_pll_lock) begin
        if (!S_pll_lock) begin S_ae_mean_s1 <= 8'd0; S_ae_mean_s2 <= 8'd0; end
        else begin S_ae_mean_s1 <= S_ae_mean; S_ae_mean_s2 <= S_ae_mean_s1; end
    end

    wire       S_ae_upd;
    // AE 正常闭环。AE_FORCE_EN=1 是诊断用的扫掠模式（已在板上验证通过），
    //   需要复测 I2C 通路时改回 1。
    ui_ae_ctrl #(.AE_FORCE_EN(0)) u_ui_ae_ctrl (
        .I_clk         ( S_clk_70m      ),
        .I_rst_n       ( ~S_rst         ),
        .I_enable      ( 1'b1           ),
        .I_mean        ( S_ae_mean_s2   ),
        .I_frame_pulse ( S_ae_tick      ),
        .O_gain        ( S_ae_gain      ),
        .O_upd         ( S_ae_upd       ),
        .O_err         (                )
    );

    wire        S_ae_i2c_start, S_ae_i2c_active;
    wire [6:0]  S_ae_i2c_dev;
    wire [15:0] S_ae_i2c_addr;
    wire [7:0]  S_ae_i2c_data;
    ui_ae_i2c_wr u_ui_ae_i2c_wr (
        .I_clk    ( S_clk_70m        ),
        .I_rst_n  ( ~S_rst           ),
        .I_allow  ( S_cam_cfg_done   ),
        .I_dev    ( S_cam_dev_used   ),
        .I_gain   ( S_ae_gain        ),
        .I_upd    ( S_ae_upd         ),
        .O_start  ( S_ae_i2c_start   ),
        .O_dev    ( S_ae_i2c_dev     ),
        .O_addr   ( S_ae_i2c_addr    ),
        .O_data   ( S_ae_i2c_data    ),
        .I_busy   ( S_hw_i2c_busy    ),
        .I_done   ( S_hw_i2c_done    ),
        .I_nack   ( S_hw_i2c_nack    ),
        .O_active ( S_ae_i2c_active  ),
        .O_nack_cnt ( S_ae_nack_cnt  ),
        .O_wr_cnt   ( S_ae_wr_cnt    )
    );

    assign S_hw_i2c_start = S_ae_i2c_active ? S_ae_i2c_start : S_cfg_i2c_start;
    assign S_hw_i2c_dev   = S_ae_i2c_active ? S_ae_i2c_dev   : S_cfg_i2c_dev;
    assign S_hw_i2c_addr  = S_ae_i2c_active ? S_ae_i2c_addr  : S_cfg_i2c_addr;
    assign S_hw_i2c_data  = S_ae_i2c_active ? S_ae_i2c_data  : S_cfg_i2c_data;

    //---- 位级 I2C 主机（SCL ~100kHz）----
    imx415_i2c_master #(
        .CLK_FREQ        ( 71428571 ),
        .SCL_FREQ        ( 100000   ),
        .SCL_STRETCH_CHK ( 0        )
    ) u_imx415_i2c_master (
        .I_clk      ( S_clk_70m       ),
        .I_rst_n    ( ~S_rst          ),
        .I_start    ( S_hw_i2c_start  ),
        .I_dev_addr ( S_hw_i2c_dev    ),
        .I_reg_addr ( S_hw_i2c_addr   ),
        .I_reg_data ( S_hw_i2c_data   ),
        .O_busy     ( S_hw_i2c_busy   ),
        .O_done     ( S_hw_i2c_done   ),
        .O_nack     ( S_hw_i2c_nack   ),
        .O_timeout  (                 ),
        .O_scl_oe   ( S_hw_i2c_scl_oe ),
        .O_sda_oe   ( S_hw_i2c_sda_oe ),
        .I_scl_i    ( IO_cam_scl   ),
        .I_sda_i    ( IO_cam_sda   )
    );

    //---- 配置状态机（含 4 地址自动扫描）----
    imx415_cfg_fsm #(
        .CLK_FREQ ( 71428571 ),
        .GAP_MS   ( 24       ),
        .STAB_MS  ( 24       ),
        .ROM_SIZE ( 201      ),
        .SCAN_EN  ( 1        ),
        .DEV_ADDR ( 7'h1A    )
    ) u_imx415_cfg_fsm (
        .I_clk         ( S_clk_70m         ),
        .I_rst_n       ( ~S_rst            ),
        .I_start       ( S_cam_cfg_start   ),
        .O_rom_index   ( S_cfg_rom_index   ),
        .I_rom_data    ( S_cfg_rom_data    ),
        .O_i2c_start   ( S_cfg_i2c_start   ),
        .O_i2c_dev     ( S_cfg_i2c_dev     ),
        .O_i2c_addr    ( S_cfg_i2c_addr    ),
        .O_i2c_data    ( S_cfg_i2c_data    ),
        .I_i2c_busy    ( S_hw_i2c_busy     ),
        .I_i2c_done    ( S_hw_i2c_done     ),
        .I_i2c_nack    ( S_hw_i2c_nack     ),
        .O_cfg_done    ( S_cam_cfg_done    ),
        .O_err_cnt     ( S_cam_err_cnt     ),
        .O_scan_result ( S_cam_scan_result ),
        .O_dev_used    ( S_cam_dev_used    ),
        .O_state       ( S_cam_fsm_state   )
    );

    csi_unpacket #(
        .CHECK_WC (1),
        .EXP_WC   (16'd2430)
    ) u_csi_unpacket(
        .I_clk             ( S_hs_rx_clk       ),
        .I_rst_n           ( ~S_rst            ),

        .I_hs_valid        ( S_hs_rx_valid     ),
        .I_hs_data         ( S_hs_rx_data      ),

        .O_csi_frame_start ( S_csi_frame_start ),
        .O_csi_frame_end   ( S_csi_frame_end   ),
        .O_csi_rx_vsync0	( S_csi_rx_vsync0	),
    	.O_csi_rx_hsync0	( S_csi_rx_hsync0	),
        .O_csi_valid       ( S_csi_valid       ),
        .O_csi_data        ( S_csi_data        )
    );


    mipi_dphy_rx_ph1a_mipiio_wrapper#(
        .LANE_NUM              ( 4 ),
        .BYTE_NUM              ( 1 )
    )u_mipi_dphy_rx_ph1a_mipiio_wrapper(
        .I_lp_clk              ( S_clk_70m       ), 
        .I_rst                 ( S_rst           ),
 
        .I_clk_lane_in_delay   ( IDELAY_CLK      ),
        .I_data_lane0_in_delay ( IDELAY_D0       ),
        .I_data_lane1_in_delay ( IDELAY_D1       ),
        .I_data_lane2_in_delay ( IDELAY_D2       ),
        .I_data_lane3_in_delay ( IDELAY_D3       ),

        //  MIPI 数据 lane 的 P/N 反相，高有效，一位一条 lane：
        //      [0]=Lane0(板座15/14) [1]=Lane1(20/19) [2]=Lane2(23/22) [3]=Lane3(29/28)
        //
        //  ★【2026-10-12 上板实测确认】本板这一位必须是 4'b1101，**不能改成 4'b0000**：
        //    实测 4'b1101 能出图（虽然色彩有错），改成 4'b0000 直接**完全不出图**。
        //    （参考工程中的值为 4'b0000，本板实测无法出图，故采用 4'b1101。
        //      本板座子到 PHY 的 lane 走线就是这样接的，与参考工程那块板不同。）
        //    位含义：[0]=Lane0(板座15/14) [1]=Lane1(20/19) [2]=Lane2(23/22) [3]=Lane3(29/28)
        //    当前 4'b1101 = Lane0 / Lane2 / Lane3 反相，Lane1 不反。
        //  ★ 注意：本端口只有 LANE_NUM(=4) 位，管不到时钟 lane；
        //    时钟 P/N 若反了只能在板/转接板上对调 CKP/CKN。
        .I_lane_invert         ( 4'b1101         ),

        .O_hs_rx_clk           ( S_hs_rx_clk     ),
        .O_hs_rx_valid         ( S_hs_rx_valid   ),
        .O_hs_rx_data          ( S_hs_rx_data    ),

        .O_lp_rx_lane0_p       (  ),
        .O_lp_rx_lane0_n       (  ),

        .I_lp_tx_en            ( 1'b0            ),
        .I_lp_tx_lane0_p       ( 1'b0            ),
        .I_lp_tx_lane0_n       ( 1'b0            ),

        .O_lane_error          ( S_lane_error    )
    );

    
endmodule

`timescale 1ns/1ps
//-----------------------------------------------------------------------------
// video_fdma_arb —— 两路视频写（1456 dbuf / 1920 dbuf）轮询仲裁
//
// 与 photo_fdma_arb 相同的 FDMA 用户侧协议：
//   * wareq 是电平；FDMA 在采样后的下一拍拉高 busy，覆盖整个 burst；
//   * wvalid 是每拍脉冲，客户端只把它当 FIFO pop 使能；
//   * 未选中客户端看到 busy=0、valid=0，状态机停在原地、不丢请求；
//   * 只在 busy=0 时切换 sel；决定拍就把送给 FDMA 的 req 压 0，SWITCH 拍
//     再空转 1 拍，之后才放新授权方的请求；
//   * 两路都请求时轮询（last 记录上一次服务方，下一次优先另一路）。
//-----------------------------------------------------------------------------
module video_fdma_arb (
    input  wire         I_clk,
    input  wire         I_rst_n,
    // client 0: 1456 dbuf
    input  wire [28:0]  I_v0_waddr,
    input  wire         I_v0_wareq,
    input  wire [15:0]  I_v0_wsize,
    input  wire [255:0] I_v0_wdata,
    output wire         O_v0_wbusy,
    output wire         O_v0_wvalid,
    // client 1: 1920 dbuf
    input  wire [28:0]  I_v1_waddr,
    input  wire         I_v1_wareq,
    input  wire [15:0]  I_v1_wsize,
    input  wire [255:0] I_v1_wdata,
    output wire         O_v1_wbusy,
    output wire         O_v1_wvalid,
    // slave: photo_fdma_arb 的 ub 写口
    output wire [28:0]  O_waddr,
    output wire         O_wareq,
    output wire [15:0]  O_wsize,
    output wire [255:0] O_wdata,
    input  wire         I_wbusy,
    input  wire         I_wvalid
);
    localparam [1:0] S_IDLE = 2'd0,
                     S_SW   = 2'd1,
                     S_RUN  = 2'd2,
                     S_REQ  = 2'd3;
    // 发出 req 后等 busy 的超时；只在请求方已撤销时允许回 IDLE。
    localparam [4:0] TMO_MAX = 5'd31;

    reg [1:0] st;
    reg       sel;
    reg       last;      // 1 = 上一次服务的是 v1，下一次优先 v0
    reg       bseen;
    reg [4:0] tmo;

    wire v0req = I_v0_wareq;
    wire v1req = I_v1_wareq;
    wire selreq = sel ? v1req : v0req;

    // 两路都请求时选上轮没服务的那路；只有一路时选它；否则保持。
    wire want = (v0req && v1req) ? ~last : (v0req ? 1'b0 : (v1req ? 1'b1 : sel));
    assign O_waddr = sel ? I_v1_waddr : I_v0_waddr;
    assign O_wsize = sel ? I_v1_wsize : I_v0_wsize;
    assign O_wdata = sel ? I_v1_wdata : I_v0_wdata;
    // photo_fdma_arb 在服务拍照时会掩掉 ub 的 busy/req；请求必须保持到
    // FDMA 真正接受（busy 拉高），不能只发一拍，否则拍照后写通道会卡死。
    assign O_wareq = I_rst_n && (((st == S_IDLE) && (want == sel) && selreq) ||
                                 ((st == S_REQ) && selreq));

    assign O_v0_wbusy  = I_wbusy  & ~sel;
    assign O_v1_wbusy  = I_wbusy  &  sel;
    assign O_v0_wvalid = I_wvalid & ~sel;
    assign O_v1_wvalid = I_wvalid &  sel;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            st    <= S_IDLE;
            sel   <= 1'b0;
            last  <= 1'b0;
            bseen <= 1'b0;
            tmo   <= 5'd0;
        end else begin
            case (st)
                S_IDLE: begin
                    if (I_wbusy) begin
                        bseen <= 1'b1;
                    end else if (want != sel) begin
                        // 本拍 req 已被 present 压 0；下一拍 SWITCH 空转后再放开
                        sel   <= want;
                        bseen <= 1'b0;
                        tmo   <= 5'd0;
                        st    <= S_SW;
                    end else if (selreq) begin
                        bseen <= 1'b0;
                        tmo   <= 5'd0;
                        st    <= S_REQ;
                    end
                end
                S_REQ: begin
                    if (I_wbusy) begin
                        bseen <= 1'b1;
                        tmo   <= 5'd0;
                        st    <= S_RUN;
                    end else if (!selreq) begin
                        tmo <= 5'd0;
                        st  <= S_IDLE;
                    end else if (tmo != TMO_MAX) begin
                        tmo <= tmo + 1'b1;
                    end
                end
                S_SW: begin
                    st <= S_IDLE;
                end
                S_RUN: begin
                    if (I_wbusy) begin
                        bseen <= 1'b1;
                        tmo   <= 5'd0;
                    end else if (bseen) begin
                        bseen <= 1'b0;
                        last  <= sel;
                        st    <= S_IDLE;
                    end else if (tmo != TMO_MAX) begin
                        tmo <= tmo + 1'b1;
                        if ((tmo == TMO_MAX - 1'b1) && !selreq) begin
                            tmo <= 5'd0;
                            st  <= S_IDLE;
                        end
                    end
                end
                default: st <= S_IDLE;
            endcase
        end
    end
endmodule
