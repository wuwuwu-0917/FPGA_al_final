
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
/*********ui_gamma ********************************************
伽马矫正(Gamma)
--版本号1.0
*****************************************************************/
module ui_gamma #(
    parameter [4:0] GAMMA_10x = 22 // 伽玛值的十倍，默认值为22，即伽玛值为2.2
)
(
    input                   I_clk  		, // 时钟信号
    input                   I_rst_n		, // 复位信号，低电平有效

    input                   I_tlast     , // 输入数据通道的最后一个有效信号
    input                   I_tuser     , // 输入数据通道的第一个有效信号
    input [95:0]            I_tdata     , // 输入数据，总共96位，每24位表示一个像素的RGB数据
    input                   I_tvalid    , // 输入数据有效信号
    output                  I_tready    , // 输入数据准备好信号

    output                  O_tlast     , // 输出数据通道的最后一个有效信号
    output                  O_tuser     , // 输出数据通道的第一个有效信号
    output [95:0]           O_tdata     , // 输出数据，总共96位，每24位表示一个像素的RGB数据
    output                  O_tvalid    , // 输出数据有效信号
    input                   O_tready      // 输出数据准备好信号
);

/*****************************************************************
                        将输入的96位数据分解为4个像素的RGB888格式                                        
*****************************************************************/

    wire [23:0] I_tdata_r1 = I_tdata[23:0] ;     // 第1个像素的数据
    wire [23:0] I_tdata_r2 = I_tdata[47:24];     // 第2个像素的数据
    wire [23:0] I_tdata_r3 = I_tdata[71:48];     // 第3个像素的数据
    wire [23:0] I_tdata_r4 = I_tdata[95:72];     // 第4个像素的数据

    wire [7:0]  rgb888_r[3:0];                   // 定义数组，存储4个像素的R分量
    wire [7:0]  rgb888_g[3:0];                   // 定义数组，存储4个像素的G分量
    wire [7:0]  rgb888_b[3:0];                   // 定义数组，存储4个像素的B分量

    // 提取第1个像素的RGB分量
    assign   rgb888_r[0] = I_tdata_r1[16+:8];    // R分量，高8位
    assign   rgb888_g[0] = I_tdata_r1[8+:8] ;    // G分量，中8位
    assign   rgb888_b[0] = I_tdata_r1[0+:8] ;    // B分量，低8位

    // 提取第2个像素的RGB分量
    assign   rgb888_r[1] = I_tdata_r2[16+:8];
    assign   rgb888_g[1] = I_tdata_r2[8+:8] ;
    assign   rgb888_b[1] = I_tdata_r2[0+:8] ;

    // 提取第3个像素的RGB分量
    assign   rgb888_r[2] = I_tdata_r3[16+:8];
    assign   rgb888_g[2] = I_tdata_r3[8+:8] ;
    assign   rgb888_b[2] = I_tdata_r3[0+:8] ;

    // 提取第4个像素的RGB分量
    assign   rgb888_r[3] = I_tdata_r4[16+:8];
    assign   rgb888_g[3] = I_tdata_r4[8+:8] ;
    assign   rgb888_b[3] = I_tdata_r4[0+:8] ;

/*****************************************************************
                        定义中间信号                     
*****************************************************************/

    // 定义用于存储查找表(LUT)输出的信号，针对不同的伽玛值
    wire [11:0] O_LUT_2_6_data_r[3:0];
    wire [11:0] O_LUT_2_6_data_g[3:0];
    wire [11:0] O_LUT_2_6_data_b[3:0];
    
    wire [11:0] O_LUT_2_4_data_r[3:0];
    wire [11:0] O_LUT_2_4_data_g[3:0];
    wire [11:0] O_LUT_2_4_data_b[3:0];

    wire [11:0] O_LUT_2_2_data_r[3:0];
    wire [11:0] O_LUT_2_2_data_g[3:0];
    wire [11:0] O_LUT_2_2_data_b[3:0];

    wire [11:0] O_LUT_2_0_data_r[3:0];
    wire [11:0] O_LUT_2_0_data_g[3:0];
    wire [11:0] O_LUT_2_0_data_b[3:0];

    wire [11:0] O_LUT_1_8_data_r[3:0];
    wire [11:0] O_LUT_1_8_data_g[3:0];
    wire [11:0] O_LUT_1_8_data_b[3:0];

    wire [11:0] O_LUT_1_6_data_r[3:0];
    wire [11:0] O_LUT_1_6_data_g[3:0];
    wire [11:0] O_LUT_1_6_data_b[3:0];
    
    wire [11:0] O_LUT_1_4_data_r[3:0];
    wire [11:0] O_LUT_1_4_data_g[3:0];
    wire [11:0] O_LUT_1_4_data_b[3:0];
    
    wire [11:0] O_LUT_1_2_data_r[3:0];
    wire [11:0] O_LUT_1_2_data_g[3:0];
    wire [11:0] O_LUT_1_2_data_b[3:0];

    wire [11:0] O_LUT_0_8_data_r[3:0];
    wire [11:0] O_LUT_0_8_data_g[3:0];
    wire [11:0] O_LUT_0_8_data_b[3:0];

    // 定义新的RGB值
    wire  [7:0]  R_new [3:0];
    wire  [7:0]  G_new [3:0];
    wire  [7:0]  B_new [3:0];

/*****************************************************************
                        定义寄存器信号                     
*****************************************************************/

    // 用于存储选择后的LUT输出数据
    reg   [11:0]  acc_rgb888_r[3:0];
    reg   [11:0]  acc_rgb888_g[3:0];
    reg   [11:0]  acc_rgb888_b[3:0];

    // 用于同步输入的tlast、tuser和tvalid信号
    reg   [3:0]   I_tlast_r ;
    reg   [3:0]   I_tuser_r ;
    reg   [3:0]   I_tvalid_r;

    // 用于反归一化的中间变量
    reg   [20:0] acc_rgb888_r_x0[3:0];
    reg   [20:0] acc_rgb888_g_x0[3:0];
    reg   [20:0] acc_rgb888_b_x0[3:0];

    reg   [20:0] acc_rgb888_r_x1[3:0];
    reg   [20:0] acc_rgb888_g_x1[3:0];
    reg   [20:0] acc_rgb888_b_x1[3:0];

    reg   [7:0]  acc_rgb888_r_x2[3:0];
    reg   [7:0]  acc_rgb888_g_x2[3:0];
    reg   [7:0]  acc_rgb888_b_x2[3:0];

/*****************************************************************
                           伽玛查找表模块实例化                                        
*****************************************************************/
    genvar i;
    generate
        for(i = 0; i < 4; i = i + 1)
        begin: LUT
            // 针对不同的伽玛值，实例化对应的LUT模块

            // 伽玛值为2.6的LUT
            ui_lut_2_6 X_ui_lut_2_6_R(
                .I_clk         (I_clk     ),
                .I_rst_n       (I_rst_n   ),
                .I_LUT_2_6_data(rgb888_r[i]),
                .O_LUT_2_6_data(O_LUT_2_6_data_r[i])
            );

            ui_lut_2_6 X_ui_lut_2_6_G(
                .I_clk         (I_clk     ),
                .I_rst_n       (I_rst_n   ),
                .I_LUT_2_6_data(rgb888_g[i]),
                .O_LUT_2_6_data(O_LUT_2_6_data_g[i])
            );

            ui_lut_2_6 X_ui_lut_2_6_B(
                .I_clk         (I_clk     ),
                .I_rst_n       (I_rst_n   ),
                .I_LUT_2_6_data(rgb888_b[i]),
                .O_LUT_2_6_data(O_LUT_2_6_data_b[i])
            );

            // 伽玛值为2.4的LUT
            ui_lut_2_4 X_ui_lut_2_4_R(
                .I_clk         (I_clk     ),
                .I_rst_n       (I_rst_n   ),
                .I_LUT_2_4_data(rgb888_r[i]),
                .O_LUT_2_4_data(O_LUT_2_4_data_r[i])
            );

            ui_lut_2_4 X_ui_lut_2_4_G(
                .I_clk         (I_clk     ),
                .I_rst_n       (I_rst_n   ),
                .I_LUT_2_4_data(rgb888_g[i]),
                .O_LUT_2_4_data(O_LUT_2_4_data_g[i])
            );

            ui_lut_2_4 X_ui_lut_2_4_B(
                .I_clk         (I_clk     ),
                .I_rst_n       (I_rst_n   ),
                .I_LUT_2_4_data(rgb888_b[i]),
                .O_LUT_2_4_data(O_LUT_2_4_data_b[i])
            );

            // 伽玛值为2.2的LUT
            ui_lut_2_2 X_ui_lut_2_2_R(
                .I_clk         (I_clk     ),
                .I_rst_n       (I_rst_n   ),
                .I_LUT_2_2_data(rgb888_r[i]),
                .O_LUT_2_2_data(O_LUT_2_2_data_r[i])
            );

            ui_lut_2_2 X_ui_lut_2_2_G(
                .I_clk         (I_clk     ),
                .I_rst_n       (I_rst_n   ),
                .I_LUT_2_2_data(rgb888_g[i]),
                .O_LUT_2_2_data(O_LUT_2_2_data_g[i])
            );

            ui_lut_2_2 X_ui_lut_2_2_B(
                .I_clk         (I_clk     ),
                .I_rst_n       (I_rst_n   ),
                .I_LUT_2_2_data(rgb888_b[i]),
                .O_LUT_2_2_data(O_LUT_2_2_data_b[i])
            );

            // 伽玛值为2.0的LUT
            ui_lut_2_0 X_ui_lut_2_0_R(
                .I_clk         (I_clk     ),
                .I_rst_n       (I_rst_n   ),
                .I_LUT_2_0_data(rgb888_r[i]),
                .O_LUT_2_0_data(O_LUT_2_0_data_r[i])
            );

            ui_lut_2_0 X_ui_lut_2_0_G(
                .I_clk         (I_clk     ),
                .I_rst_n       (I_rst_n   ),
                .I_LUT_2_0_data(rgb888_g[i]),
                .O_LUT_2_0_data(O_LUT_2_0_data_g[i])
            );

            ui_lut_2_0 X_ui_lut_2_0_B(
                .I_clk         (I_clk     ),
                .I_rst_n       (I_rst_n   ),
                .I_LUT_2_0_data(rgb888_b[i]),
                .O_LUT_2_0_data(O_LUT_2_0_data_b[i])
            );

            // 伽玛值为1.8的LUT
            ui_lut_1_8 X_ui_lut_1_8_R(
                .I_clk         (I_clk     ),
                .I_rst_n       (I_rst_n   ),
                .I_LUT_1_8_data(rgb888_r[i]),
                .O_LUT_1_8_data(O_LUT_1_8_data_r[i])
            );

            ui_lut_1_8 X_ui_lut_1_8_G(
                .I_clk         (I_clk     ),
                .I_rst_n       (I_rst_n   ),
                .I_LUT_1_8_data(rgb888_g[i]),
                .O_LUT_1_8_data(O_LUT_1_8_data_g[i])
            );

            ui_lut_1_8 X_ui_lut_1_8_B(
                .I_clk         (I_clk     ),
                .I_rst_n       (I_rst_n   ),
                .I_LUT_1_8_data(rgb888_b[i]),
                .O_LUT_1_8_data(O_LUT_1_8_data_b[i])
            );

            // 伽玛值为1.6的LUT
            ui_lut_1_6 X_ui_lut_1_6_R(
                .I_clk         (I_clk     ),
                .I_rst_n       (I_rst_n   ),
                .I_LUT_1_6_data(rgb888_r[i]),
                .O_LUT_1_6_data(O_LUT_1_6_data_r[i])
            );

            ui_lut_1_6 X_ui_lut_1_6_G(
                .I_clk         (I_clk     ),
                .I_rst_n       (I_rst_n   ),
                .I_LUT_1_6_data(rgb888_g[i]),
                .O_LUT_1_6_data(O_LUT_1_6_data_g[i])
            );

            ui_lut_1_6 X_ui_lut_1_6_B(
                .I_clk         (I_clk     ),
                .I_rst_n       (I_rst_n   ),
                .I_LUT_1_6_data(rgb888_b[i]),
                .O_LUT_1_6_data(O_LUT_1_6_data_b[i])
            );

            // 伽玛值为1.4的LUT
            ui_lut_1_4 X_ui_lut_1_4_R(
                .I_clk         (I_clk     ),
                .I_rst_n       (I_rst_n   ),
                .I_LUT_1_4_data(rgb888_r[i]),
                .O_LUT_1_4_data(O_LUT_1_4_data_r[i])
            );

            ui_lut_1_4 X_ui_lut_1_4_G(
                .I_clk         (I_clk     ),
                .I_rst_n       (I_rst_n   ),
                .I_LUT_1_4_data(rgb888_g[i]),
                .O_LUT_1_4_data(O_LUT_1_4_data_g[i])
            );

            ui_lut_1_4 X_ui_lut_1_4_B(
                .I_clk         (I_clk     ),
                .I_rst_n       (I_rst_n   ),
                .I_LUT_1_4_data(rgb888_b[i]),
                .O_LUT_1_4_data(O_LUT_1_4_data_b[i])
            );

            // 伽玛值为1.2的LUT
            ui_lut_1_2 X_ui_lut_1_2_R(
                .I_clk         (I_clk     ),
                .I_rst_n       (I_rst_n   ),
                .I_LUT_1_2_data(rgb888_r[i]),
                .O_LUT_1_2_data(O_LUT_1_2_data_r[i])
            );

            ui_lut_1_2 X_ui_lut_1_2_G(
                .I_clk         (I_clk     ),
                .I_rst_n       (I_rst_n   ),
                .I_LUT_1_2_data(rgb888_g[i]),
                .O_LUT_1_2_data(O_LUT_1_2_data_g[i])
            );

            ui_lut_1_2 X_ui_lut_1_2_B(
                .I_clk         (I_clk     ),
                .I_rst_n       (I_rst_n   ),
                .I_LUT_1_2_data(rgb888_b[i]),
                .O_LUT_1_2_data(O_LUT_1_2_data_b[i])
            );

            // 伽玛值为0.8的LUT
            ui_lut_0_8 X_ui_lut_0_8_R(
                .I_clk         (I_clk     ),
                .I_rst_n       (I_rst_n   ),
                .I_LUT_0_8_data(rgb888_r[i]),
                .O_LUT_0_8_data(O_LUT_0_8_data_r[i])
            );

            ui_lut_0_8 X_ui_lut_0_8_G(
                .I_clk         (I_clk     ),
                .I_rst_n       (I_rst_n   ),
                .I_LUT_0_8_data(rgb888_g[i]),
                .O_LUT_0_8_data(O_LUT_0_8_data_g[i])
            );

            ui_lut_0_8 X_ui_lut_0_8_B(
                .I_clk         (I_clk     ),
                .I_rst_n       (I_rst_n   ),
                .I_LUT_0_8_data(rgb888_b[i]),
                .O_LUT_0_8_data(O_LUT_0_8_data_b[i])
            );

/*****************************************************************
                           选择对应的伽玛校正值                                      
*****************************************************************/

            // 根据参数GAMMA_10x选择对应的伽玛LUT输出
            always @(posedge I_clk or negedge I_rst_n)begin 
                if(!I_rst_n)begin
                    acc_rgb888_r[i] <= 12'd0;
                    acc_rgb888_g[i] <= 12'd0;
                    acc_rgb888_b[i] <= 12'd0;
                end
                else begin
                    case (GAMMA_10x)
                        5'd26:begin 
                           acc_rgb888_r[i] <= O_LUT_2_6_data_r[i];
                           acc_rgb888_g[i] <= O_LUT_2_6_data_g[i];
                           acc_rgb888_b[i] <= O_LUT_2_6_data_b[i];
                        end
                        5'd24:begin 
                           acc_rgb888_r[i] <= O_LUT_2_4_data_r[i];
                           acc_rgb888_g[i] <= O_LUT_2_4_data_g[i];
                           acc_rgb888_b[i] <= O_LUT_2_4_data_b[i];
                        end
                        5'd22:begin 
                           acc_rgb888_r[i] <= O_LUT_2_2_data_r[i];
                           acc_rgb888_g[i] <= O_LUT_2_2_data_g[i];
                           acc_rgb888_b[i] <= O_LUT_2_2_data_b[i];
                        end
                        5'd20:begin
                           acc_rgb888_r[i] <= O_LUT_2_0_data_r[i];
                           acc_rgb888_g[i] <= O_LUT_2_0_data_g[i];
                           acc_rgb888_b[i] <= O_LUT_2_0_data_b[i];
                        end
                        5'd18:begin
                           acc_rgb888_r[i] <= O_LUT_1_8_data_r[i];
                           acc_rgb888_g[i] <= O_LUT_1_8_data_g[i];
                           acc_rgb888_b[i] <= O_LUT_1_8_data_b[i];
                        end
                        5'd16:begin
                           acc_rgb888_r[i] <= O_LUT_1_6_data_r[i];
                           acc_rgb888_g[i] <= O_LUT_1_6_data_g[i];
                           acc_rgb888_b[i] <= O_LUT_1_6_data_b[i];
                        end
                        5'd14:begin
                           acc_rgb888_r[i] <= O_LUT_1_4_data_r[i];
                           acc_rgb888_g[i] <= O_LUT_1_4_data_g[i];
                           acc_rgb888_b[i] <= O_LUT_1_4_data_b[i];
                        end
                        5'd12:begin
                           acc_rgb888_r[i] <= O_LUT_1_2_data_r[i];
                           acc_rgb888_g[i] <= O_LUT_1_2_data_g[i];
                           acc_rgb888_b[i] <= O_LUT_1_2_data_b[i];
                        end
                        5'd8 :begin
                           acc_rgb888_r[i] <= O_LUT_0_8_data_r[i];
                           acc_rgb888_g[i] <= O_LUT_0_8_data_g[i];
                           acc_rgb888_b[i] <= O_LUT_0_8_data_b[i];
                        end
                        default:begin 
                           acc_rgb888_r[i] <= O_LUT_2_2_data_r[i];
                           acc_rgb888_g[i] <= O_LUT_2_2_data_g[i];
                           acc_rgb888_b[i] <= O_LUT_2_2_data_b[i];
                        end
                    endcase
                end
            end

/*****************************************************************
                           反归一化处理                                      
*****************************************************************/

            // 第一级流水线，将数据左移8位
            always @(posedge I_clk or negedge I_rst_n)begin 
                if(!I_rst_n)begin
                    acc_rgb888_r_x0[i] <= 0;
                    acc_rgb888_g_x0[i] <= 0;
                    acc_rgb888_b_x0[i] <= 0;
                end
                else if(I_tvalid_r[0])begin
                    acc_rgb888_r_x0[i] <= acc_rgb888_r[i] << 8;
                    acc_rgb888_g_x0[i] <= acc_rgb888_g[i] << 8;
                    acc_rgb888_b_x0[i] <= acc_rgb888_b[i] << 8;
                end
                else begin
                    acc_rgb888_r_x0[i] <= acc_rgb888_r_x0[i] ;
                    acc_rgb888_g_x0[i] <= acc_rgb888_g_x0[i] ;
                    acc_rgb888_b_x0[i] <= acc_rgb888_b_x0[i] ;
                end
            end
        
            // 第二级流水线，减去2048，实现反归一化
            always @(posedge I_clk or negedge I_rst_n)begin 
                if(!I_rst_n)begin
                    acc_rgb888_r_x1[i] <= 0;
                    acc_rgb888_g_x1[i] <= 0;
                    acc_rgb888_b_x1[i] <= 0;
                end
                else if(I_tvalid_r[1])begin
                    acc_rgb888_r_x1[i] <= acc_rgb888_r_x0[i] - 2048;
                    acc_rgb888_g_x1[i] <= acc_rgb888_g_x0[i] - 2048;
                    acc_rgb888_b_x1[i] <= acc_rgb888_b_x0[i] - 2048;
                end
                else begin
                    acc_rgb888_r_x1[i] <= acc_rgb888_r_x1[i];
                    acc_rgb888_g_x1[i] <= acc_rgb888_g_x1[i];
                    acc_rgb888_b_x1[i] <= acc_rgb888_b_x1[i];
                end
            end
    
            // 第三级流水线，限制RGB值的范围，防止溢出
            always @(posedge I_clk or negedge I_rst_n)begin 
                if(!I_rst_n)begin
                    acc_rgb888_r_x2[i] <= 0;
                    acc_rgb888_g_x2[i] <= 0;
                    acc_rgb888_b_x2[i] <= 0;
                end
                else if(I_tvalid_r[2])begin
                    acc_rgb888_r_x2[i] <= (acc_rgb888_r_x1[i][20]) ? 8'd255 : (acc_rgb888_r_x1[i][19:12]);
                    acc_rgb888_g_x2[i] <= (acc_rgb888_g_x1[i][20]) ? 8'd255 : (acc_rgb888_g_x1[i][19:12]);
                    acc_rgb888_b_x2[i] <= (acc_rgb888_b_x1[i][20]) ? 8'd255 : (acc_rgb888_b_x1[i][19:12]);
                end
                else begin
                    acc_rgb888_r_x2[i] <= acc_rgb888_r_x2[i];
                    acc_rgb888_g_x2[i] <= acc_rgb888_g_x2[i];
                    acc_rgb888_b_x2[i] <= acc_rgb888_b_x2[i];
                end
            end

/*****************************************************************
                            获取新的RGB值                                      
*****************************************************************/

            // 将反归一化并限制后的值赋给新的RGB值
            assign R_new[i] = acc_rgb888_r_x2[i];
            assign G_new[i] = acc_rgb888_g_x2[i];
            assign B_new[i] = acc_rgb888_b_x2[i];

        end
    endgenerate

/*****************************************************************
                            同步信号处理                                      
*****************************************************************/

    // 对输入的控制信号进行同步，延迟4个时钟周期，与数据对齐
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            {I_tlast_r,I_tuser_r,I_tvalid_r} <= 0;
        else begin
            I_tlast_r <= {I_tlast_r[2:0],I_tlast};
            I_tuser_r <= {I_tuser_r[2:0],I_tuser};
            I_tvalid_r <= {I_tvalid_r[2:0],I_tvalid};
        end
    end

    // 将同步后的控制信号赋给输出
    assign  O_tlast  = I_tlast_r[3];   
    assign  O_tuser  = I_tuser_r[3];   
    assign  O_tvalid = I_tvalid_r[3];

    // 将新的RGB值组合成输出数据
    assign O_tdata = {R_new[3],G_new[3],B_new[3],
                      R_new[2],G_new[2],B_new[2],
                      R_new[1],G_new[1],B_new[1],
                      R_new[0],G_new[0],B_new[0]};

    // 将输入的准备好信号直接连接到输出的准备好信号
    assign  I_tready = O_tready;

endmodule
