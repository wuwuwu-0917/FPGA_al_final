
/*******************************MILIANKE*******************************
*Company : MiLianKe Electronic Technology Co., Ltd.
*WebSite:https://www.milianke.com
*TechWeb:https://www.uisrc.com
*tmall-shop:https://milianke.tmall.com
*jd-shop:https://milianke.jd.com
*taobao-shop1: https://milianke.taobao.com
*Create Date: 2024/10/23
*Module Name:
*File Name:
*Description: 
*The reference demo provided by Milianke is only used for learning. 
*We cannot ensure that the demo itself is free of bugs, so users 
*should be responsible for the technical problems and consequences
*caused by the use of their own products.
*Copyright: Copyright (c) MiLianKe
*All rights reserved.
*Revision: 1.0
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
/*********ui_isp_top ********************************************
--米联客ISP图像算法包含有黑电平矫正(BLC),去噪(Denoising),去马赛克(Demosaicing),白平衡(wab),
伽马矫正(Gamma),对比度调节(Contrast),亮度调节(Brightness),饱和度调节(Saturation)锐化(Sharpening)
--版本号1.0  
*****************************************************************/
module ui_isp_top (
    input 			I_clk,
    input 			I_rst_n			,
    input 			I_tlast			,
    input 			I_tuser			,
    input [39:0] 	I_tdata			,
    input 			I_tvalid		,
    input [9:0] 	I_tdest			,
    input 			O_tready		,
    // ★ 2026-10-21 滤镜域（75MHz）与运行时参数
    input 			I_clk_flt		,
    input [2:0] 	I_mode			,	// 0 原图 / 1 漫画 / 2 浮雕 / 3 铅笔
    input [7:0] 	I_param1		,
    input [7:0] 	I_param2		,
    input [7:0] 	I_param3		,
    input [7:0] 	I_param4		,
    input [7:0] 	I_param5		,
    input 			I_compare		,
    input [7:0] 	I_split			,
    input [7:0] 	I_he_strength	,	// 直方图均衡强度 0..100
    output [7:0] 	O_ae_mean		,	// HE 之前测光：每帧平均亮度
    output [7:0] 	O_ae_peak		,	// HE 之前测光：每帧峰值亮度
    output 			O_ae_valid		,
    output [127:0] 	O_tdata			,
    output 			O_tlast			,
    output 			O_tuser			,
    output 			O_tvalid		,
    output 			I_tready		,
    output [7:0] 	O_fps			// 滤镜域实测帧率（帧/秒），OSD 显示用
);

    wire         	BLC_O_tlast;  
    wire         	BLC_O_tuser;  
    wire [39:0] 	BLC_O_tdata;  
    wire         	BLC_O_tvalid;  
    wire [9:0] 		BLC_O_tdest;  
    wire         	BLC_O_tready;  
    
    wire [9:0] 		black_level_offset_r0;
    wire [9:0] 		black_level_offset_r1;
    wire [9:0] 		black_level_offset_r2;
    wire [9:0] 		black_level_offset_r3;
    
    
    wire         	m_aixs_tvalid;  
    wire [127:0] 	m_aixs_tdata;  
    wire         	m_aixs_tuser;  
    wire         	m_aixs_tlast;  
    wire         	m_aixs_tready;  
    wire [ 95:0] 	m_aixs_tdata_96;
    
    wire         	awb_O_tlast;  
    wire         	awb_O_tuser;  
    wire [ 95:0] 	awb_O_tdata;  
    wire         	awb_O_tvalid; 
    wire         	awb_O_tready; 
	  
    wire         	gamma_O_tlast;  
    wire         	gamma_O_tuser;  
    wire [ 95:0] 	gamma_O_tdata;  
    wire         	gamma_O_tvalid;  
    wire         	gamma_O_tready;  
	  
    wire         	cut_O_tlast;
    wire         	cut_O_tuser;
    wire [ 95:0] 	cut_O_tdata;	
    wire         	cut_O_tvalid;
    wire         	cut_O_tready;
	  
    wire         	contrast_O_tlast;
    wire         	contrast_O_tuser;
    wire [ 95:0] 	contrast_O_tdata;
    wire         	contrast_O_tvalid;
    wire         	contrast_O_tready;
	  
    wire         	Brightness_O_tlast;
    wire         	Brightness_O_tuser;
    wire [ 95:0] 	Brightness_O_tdata;
    wire         	Brightness_O_tvalid;
    wire         	Brightness_O_tready;
	  
    wire         	Saturation_O_tlast;
    wire         	Saturation_O_tuser;
    wire [ 95:0] 	Saturation_O_tdata;
    wire         	Saturation_O_tvalid;
    wire         	Saturation_O_tready;
    
    wire [127:0] 	Saturation_O_tdata_128;

    // 滤镜域输出（相机域恢复后）
    wire [ 95:0] 	com_O_tdata;
    wire         	com_O_tvalid;
    wire         	com_O_tlast;
    wire         	com_O_tuser;

	//assign m_aixs_tready = O_tready;
	//assign O_tdata  = m_aixs_tdata;
	//assign O_tlast  = m_aixs_tlast    ;
	//assign O_tuser  = m_aixs_tuser    ;
	//assign O_tvalid = m_aixs_tvalid   ;

	assign Saturation_O_tready = O_tready	;
	assign O_tdata  = Saturation_O_tdata_128;
	assign O_tlast  = com_O_tlast           ;
	assign O_tuser  = com_O_tuser           ;
	assign O_tvalid = com_O_tvalid          ;


ui_BLC #(
    .Black_level_offset_r0	(50),
    .Black_level_offset_r1	(50),
    .Black_level_offset_r2	(50),
    .Black_level_offset_r3	(50)
) X_ui_BLC (
    .I_clk   		(I_clk				),
    .I_rst_n 		(I_rst_n 			),
    .I_tlast 		(I_tlast 			),
    .I_tuser 		(I_tuser 			),
    .I_tdata 		(I_tdata 			),
    .I_tvalid		(I_tvalid			),
    .I_tdest 		(I_tdest 			),
    .I_tready		(					),
	
    .O_tlast 		(BLC_O_tlast		),
    .O_tuser 		(BLC_O_tuser		),
    .O_tdata 		(BLC_O_tdata		),
    .O_tvalid		(BLC_O_tvalid		),
    .O_tdest 		(BLC_O_tdest		),
    .O_tready		(BLC_O_tready		)
);

ui_top_demosaic #(
    .IMG_HEIGHT          (1080			),  // 图像高度
    .IMG_WIDTH           (1920			),   // 图像宽度
    .data_complete_delay (50  			),
    .BAYER_MODE          ("RGGB"		)
)
X_ui_top_demosaic
(
    .I_clk   		(I_clk				),  // 时钟信号
    .I_rst_n 		(I_rst_n			),  // 复位信号，低有效
    .I_axi4s_tdata 	(BLC_O_tdata		),  // AXI4-Stream视频数据
    .I_axi4s_tdest 	(BLC_O_tdest		),
    .I_axi4s_tlast 	(BLC_O_tlast		),  // 行结束信号
    .I_axi4s_tvalid	(BLC_O_tvalid		),  // 数据有效信号
    .I_axi4s_tuser 	(BLC_O_tuser		),  // 帧开始信号
    .I_axi4s_tready	(BLC_O_tready		),  // 从模块准备好接受数据
    .O_tlast  		(m_aixs_tlast		),  // 输出行结束信号
    .O_tuser  		(m_aixs_tuser		),  // 输出帧开始信号
    .O_tdata  		(m_aixs_tdata		),  // 输出数据
    .O_tvalid 		(m_aixs_tvalid		),  // 输出数据有效信号
    .O_tready   	(m_aixs_tready		)   // 输出数据准备好信号
);

ui_data128_96 X_ui_data128_96(
    .I_tdata		(m_aixs_tdata		),
    .O_tdata		(m_aixs_tdata_96	)
);

ui_awb #(
    .IMG_HEIGHT		(1080				),
    .IMG_WIDTH		(1920				)
) X_ui_awb (	
    .I_clk   		(I_clk				),
    .I_rst_n 		(I_rst_n			),
    .I_tlast 		(m_aixs_tlast		),
    .I_tuser 		(m_aixs_tuser		),
    .I_tdata 		(m_aixs_tdata_96	),
    .I_tvalid		(m_aixs_tvalid		),
    .I_tready		(m_aixs_tready		),
    .O_tlast 		(awb_O_tlast 		),
    .O_tuser 		(awb_O_tuser 		),
    .O_tdata 		(awb_O_tdata 		),
    .O_tvalid		(awb_O_tvalid		),
    .O_tready		(awb_O_tready		)
);

ui_image_cut #(
    .IMG_WIDTH		(1920				),
    .IMG_HEIGHT		(1080				),
    .DATA_WIDTH		(96					),
    .SKIP_ROWS_top	(0					),
    .SKIP_ROWS_bottom(0					),
    .SKIP_COLS_left	(1					),
    .SKIP_COLS_right(0					)
) X_ui_image_cut 	
(	
    .I_clk   		(I_clk				),
    .I_rst_n 		(I_rst_n			),
    .I_tlast 		(awb_O_tlast		),
    .I_tuser 		(awb_O_tuser		),
    .I_tdata 		(awb_O_tdata		),
    .I_tvalid		(awb_O_tvalid		),
    .I_tready		(awb_O_tready		),
    .O_tlast 		(cut_O_tlast		),
    .O_tuser 		(cut_O_tuser		),
    .O_tdata 		(cut_O_tdata		),
    .O_tvalid		(cut_O_tvalid		),
    .O_tready		(cut_O_tready		)
);

ui_gamma #(
    .GAMMA_10x		(22					)
) X_ui_gamma 	
(	
    .I_clk   		(I_clk				),
    .I_rst_n 		(I_rst_n			),
    .I_tlast 		(cut_O_tlast		),
    .I_tuser 		(cut_O_tuser		),
    .I_tdata 		(cut_O_tdata		),
    .I_tvalid		(cut_O_tvalid		),
    .I_tready		(cut_O_tready		),
    .O_tlast 		(gamma_O_tlast		),
    .O_tuser 		(gamma_O_tuser		),
    .O_tdata 		(gamma_O_tdata		),
    .O_tvalid		(gamma_O_tvalid		),
    .O_tready		(gamma_O_tready		)
);

ui_Contrast_adj #(
    .contrast_level	(140				) 
) X_ui_Contrast_adj (
    .I_clk   		(I_clk				),
    .I_rst_n 		(I_rst_n			),
    .I_tlast 		(gamma_O_tlast		),
    .I_tuser 		(gamma_O_tuser		),
    .I_tdata 		(gamma_O_tdata		),
    .I_tvalid		(gamma_O_tvalid		),
    .I_tready		(gamma_O_tready		),
    .O_tlast 		(contrast_O_tlast	),
    .O_tuser 		(contrast_O_tuser	),
    .O_tdata 		(contrast_O_tdata	),
    .O_tvalid		(contrast_O_tvalid	),
    .O_tready		(contrast_O_tready	)
);

ui_Brightness_adj #(
    .BRIGHTNESS_ADD  (5					),
    .BRIGHTNESS_MINUS(0					)
) X_ui_Brightness_adjustment 
(
    .I_clk   		(I_clk				),
    .I_rst_n 		(I_rst_n			),
    .I_tlast 		(contrast_O_tlast	),
    .I_tuser 		(contrast_O_tuser	),
    .I_tdata 		(contrast_O_tdata	),
    .I_tvalid		(contrast_O_tvalid	),
    .I_tready		(contrast_O_tready	),
    .O_tlast 		(Brightness_O_tlast	),
    .O_tuser 		(Brightness_O_tuser	),
    .O_tdata 		(Brightness_O_tdata	),
    .O_tvalid		(Brightness_O_tvalid),
    .O_tready		(Brightness_O_tready)
);

ui_Saturation_adj #(
    .ADJUST_VAL		(130				)
) X_ui_Saturation_adj (
    .I_clk   		(I_clk				),
    .I_rst_n 		(I_rst_n			),
    .I_tlast 		(Brightness_O_tlast	),
    .I_tuser 		(Brightness_O_tuser	),
    .I_tdata 		(Brightness_O_tdata	),
    .I_tvalid		(Brightness_O_tvalid),
    .I_tready		(Brightness_O_tready),
    .O_tlast 		(Saturation_O_tlast ),
    .O_tuser 		(Saturation_O_tuser ),
    .O_tdata 		(Saturation_O_tdata ),
    .O_tvalid		(Saturation_O_tvalid),
    .O_tready		(Saturation_O_tready)
);

//=============================================================================
//  直方图均衡：饱和度之后、滤镜域之前
//  AE 测光抽头同时移到这里（HE 输入），避免未来控制环与 HE 互相追。
//=============================================================================
wire [95:0] he_O_tdata;
wire        he_O_tvalid, he_O_tlast, he_O_tuser;
wire        he_lut_we;
wire [7:0]  he_lut_addr, he_lut_data;
wire        he_lut_done;   // 第一张查表建好后拉高；未就绪时 ui_he_apply 旁路

ui_he_stats u_he_stats (
    .I_clk      (I_clk),
    .I_rst_n    (I_rst_n),
    .I_tdata    (Saturation_O_tdata),
    .I_tvalid   (Saturation_O_tvalid),
    .I_tlast    (Saturation_O_tlast),
    .I_tuser    (Saturation_O_tuser),
    .I_strength (I_he_strength),
    .O_lut_we   (he_lut_we),
    .O_lut_addr (he_lut_addr),
    .O_lut_data (he_lut_data),
    .O_lut_done (he_lut_done)
);

ui_he_apply u_he_apply (
    .I_clk      (I_clk),
    .I_rst_n    (I_rst_n),
    .I_tdata    (Saturation_O_tdata),
    .I_tvalid   (Saturation_O_tvalid),
    .I_tlast    (Saturation_O_tlast),
    .I_tuser    (Saturation_O_tuser),
    .I_lut_we   (he_lut_we),
    .I_lut_addr (he_lut_addr),
    .I_lut_data (he_lut_data),
    .I_lut_done (he_lut_done),
    .O_tdata    (he_O_tdata),
    .O_tvalid   (he_O_tvalid),
    .O_tlast    (he_O_tlast),
    .O_tuser    (he_O_tuser)
);

ui_ae_meter u_ae_meter (
    .I_clk    (I_clk),
    .I_rst_n  (I_rst_n),
    .I_tdata  (Saturation_O_tdata),
    .I_tvalid (Saturation_O_tvalid),
    .I_tlast  (Saturation_O_tlast),
    .I_tuser  (Saturation_O_tuser),
    .O_mean   (O_ae_mean),
    .O_peak   (O_ae_peak),
    .O_valid  (O_ae_valid)
);

//=============================================================================
//  滤镜域（2026-10-21 移植：clean 的漫画/铅笔 + lvjing 的浮雕，四路并联）
//
//  插在饱和度之后、96->128 位宽转换之前 —— 和 clean_new 的插入点完全一致。
//  内部自带 相机域 <-> 75MHz 异步 FIFO，本模块只需额外提供 I_clk_flt。
//=============================================================================
ui_filter_domain_new #(
    .PENCIL_RADIUS (16   ),
    .COMIC_RADIUS  (16   ),
    .COMIC_SHARP   (1    ),
    .IMG_WIDTH     (1920 ),
    .IMG_HEIGHT    (1080 ),
    .FIFO_AW       (9    )
) u_filter_domain (
    .I_clk			(I_clk				),
    .I_clk_flt		(I_clk_flt			),
    .I_rst_n		(I_rst_n			),
    .I_tdata		(he_O_tdata			),
    .I_tvalid		(he_O_tvalid		),
    .I_tlast		(he_O_tlast			),
    .I_tuser		(he_O_tuser			),
    .I_mode			(I_mode				),
    .I_param1		(I_param1			),
    .I_param2		(I_param2			),
    .I_param3		(I_param3			),
    .I_param4		(I_param4			),
    .I_param5		(I_param5			),
    .I_compare		(I_compare			),
    .I_split		(I_split			),
    .O_tdata		(com_O_tdata		),
    .O_tvalid		(com_O_tvalid		),
    .O_tlast		(com_O_tlast		),
    .O_tuser		(com_O_tuser		),
    .O_fps			(O_fps				)
);

ui_data96_128 X_ui_data96_128 (
    .I_tdata		(com_O_tdata		  ),
    .O_tdata		(Saturation_O_tdata_128)
);

endmodule
