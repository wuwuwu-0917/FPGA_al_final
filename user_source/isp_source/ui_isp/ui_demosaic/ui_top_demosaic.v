
/*******************************MILIANKE*******************************
*Company : MiLianKe Electronic Technology Co., Ltd.
*WebSite:https://www.milianke.com
*TechWeb:https://www.uisrc.com
*tmall-shop:https://milianke.tmall.com
*jd-shop:https://milianke.jd.com
*taobao-shop1: https://milianke.taobao.com
*Create Date: 2023/03/23
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
/**********************ui_top_demosaic ******************************
该模块主要是用于3x3矩阵的形成，以及对图像边界像素的处理
在原基础上优化了时序，把原来的一些未完全对齐的信号进行对齐。
--版本号1.0  
*****************************************************************/

module ui_top_demosaic #(
    parameter IMG_HEIGHT          = 1080,  // 图像高度
    parameter IMG_WIDTH           = 1920,  // 图像宽度
    parameter data_complete_delay = 50  ,
    parameter BAYER_MODE          = "BGGR"
)
(
    input           I_clk   		,  // 时钟信号
    input           I_rst_n 		,  // 复位信号，低有效

    input [39:0]    I_axi4s_tdata 	,  // AXI4-Stream视频数据
    input [9:0]     I_axi4s_tdest 	,
    input           I_axi4s_tlast 	,  // 行结束信号
    input           I_axi4s_tvalid	,  // 数据有效信号
    input           I_axi4s_tuser 	,  // 帧开始信号
    output          I_axi4s_tready	,  // 从模块准备好接受数据

    output          O_tlast  		,  // 输出行结束信号
    output          O_tuser  		,  // 输出帧开始信号
    output [127:0]  O_tdata  		,  // 输出数据
    output          O_tvalid 		,  // 输出数据有效信号
    input           O_tready 		   // 输出数据准备好信号
);


wire 		demosaic_tlast ; 
wire 		demosaic_tuser ; 
wire 		demosaic_tvalid; 
wire 		demosaic_tready; 

wire        bayer_ypos      ;
wire [95:0] matrix_last_line;
wire [95:0] matrix_cur_line ;
wire [95:0] matrix_next_line;

ui_raw_matrix_3x3_buffer #(
    .IMG_HEIGHT         (IMG_HEIGHT         )	,  // 图像高度
    .IMG_WIDTH          (IMG_WIDTH          )	,  // 图像宽度
    .data_complete_delay(data_complete_delay)
)X_ui_raw_matrix_3x3_buffer 
(
   .I_clk    				(I_clk   )				,  // 时钟信号
   .I_rst_n  				(I_rst_n )				,  // 复位信号，低有效

   .I_axi4s_video_tdata 	(I_axi4s_tdata )	,  // AXI4-Stream视频数据
   .I_axi4s_video_tdest 	(I_axi4s_tdest )	,
   .I_axi4s_video_tlast 	(I_axi4s_tlast )	,  // 行结束信号
   .I_axi4s_video_tvalid	(I_axi4s_tvalid)	,  // 数据有效信号
   .I_axi4s_video_tuser 	(I_axi4s_tuser )	,  // 帧开始信号
   .I_axi4s_video_tready	(I_axi4s_tready)	,  // 从模块准备好接受数据

   .O_tlast         		(demosaic_tlast )		,   // 行结束信号输出
   .O_tuser         		(demosaic_tuser )		,   // 帧开始信号输出
   .O_tvalid        		(demosaic_tvalid)		,   // 数据有效信号输出
   .O_tready        		(demosaic_tready)		,   // 上一级模块准备好接受数据
	
   .bayer_ypos      		(bayer_ypos      )		,   // Bayer滤波器行位置
   .matrix_last_line		(matrix_last_line)		,   // 上一行3x3矩阵数据
   .matrix_cur_line 		(matrix_cur_line )		,   // 当前行3x3矩阵数据
   .matrix_next_line		(matrix_next_line)		    // 下一行3x3矩阵数据
);

ui_bilinear_interpolation #(
    .BAYER_MODE 		(BAYER_MODE)
	)
X_ui_bilinear_interpolation(
	.I_clk            	(I_clk   )			,
	.I_rst_n          	(I_rst_n )			,
	
	.I_tlast      		(demosaic_tlast )  	,
	.I_tuser      		(demosaic_tuser )  	,
	.I_tvalid     		(demosaic_tvalid)  	,
	.I_tready     		(demosaic_tready)  	,
	
	.bayer_ypos       	(bayer_ypos       )	,
	.matrix_last_line 	(matrix_last_line )	,
	.matrix_cur_line  	(matrix_cur_line  )	,
	.matrix_next_line 	(matrix_next_line )	,
	
	.O_tlast 			(O_tlast )			,
	.O_tuser 			(O_tuser )			,
	.O_tdata 			(O_tdata )			,
	.O_tvalid			(O_tvalid)			,
	.O_tready			(O_tready)			

);



endmodule