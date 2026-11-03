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
/*********ui_mipi_to_raw_converter ******************************
该模块主要是用于3x3矩阵的形成，以及对图像边界像素的处理
在原基础上优化了时序，把原来的一些未完全对齐的信号进行对齐。
--版本号1.0  
*****************************************************************/

module ui_mipi_to_raw_converter 
(
    input           	I_clk   	,
    input           	I_rst_n 	,
	
    input   [39:0]  	I_raw_tdata ,
    input   [ 9:0]  	I_raw_tdest ,
    input           	I_raw_tlast ,
    input           	I_raw_tvalid,
    input           	I_raw_tuser ,
    output          	I_raw_tready,
		
    // 输出RAW10数据	
    output  [31:0]  	O_raw_tdata ,
    output          	O_raw_tlast ,
    output          	O_raw_tvalid,
    output          	O_raw_tuser ,
    input           	O_raw_tready
);
    reg            			I_tlast_r0;
    reg            			I_tuser_r0;
			
    reg            			I_tvalid_r0;
    reg    [31:0]  			raw_data_r0;
	
	assign  O_raw_tlast  = 	I_tlast_r0;   
    assign  O_raw_tuser  = 	I_tuser_r0;   
    assign  O_raw_tvalid = 	I_tvalid_r0;
	
    assign  O_raw_tdata  = 	raw_data_r0;
		
    assign  I_raw_tready = 	O_raw_tready;

    ///此处对于每个RAW10取高8位是因为高位所包含的颜色数据更多。
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            raw_data_r0 <= 0;
        else if(I_raw_tvalid)begin
            raw_data_r0 <= {I_raw_tdata[9:2], I_raw_tdata[19:12], I_raw_tdata[29:22], I_raw_tdata[39:32]};
        end 
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)begin
            {I_tlast_r0,I_tuser_r0,I_tvalid_r0} <= 0;
        end
        else begin
            I_tlast_r0  <= I_raw_tlast;
            I_tuser_r0  <= I_raw_tuser;
            I_tvalid_r0 <= I_raw_tvalid;
        end
    end
    

endmodule


