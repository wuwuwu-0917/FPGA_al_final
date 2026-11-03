
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
/*********ui_Saturation_adj ********************************************
--饱和度调节(Saturation)
    
    Y = 0.2989*R + 0.5870*G + 0.1140*B;
    R_new = -Y * value + R * (1+value);
    G_new = -Y * value + G * (1+value);
    B_new = -Y * value + B * (1+value);
    value = (-1~1) 
--版本号1.0  
*****************************************************************/


module ui_Saturation_adj#(
parameter [8:0]  ADJUST_VAL = 128 // 饱和度调节参数，9位，默认值为128
)  
(
    input                   I_clk    ,    // 输入时钟信号
    input                   I_rst_n  ,    // 输入复位信号，低电平有效

    input                   I_tlast  ,  // 行结束信号
    input                   I_tuser  ,  // 帧开始信号
    input [95:0]            I_tdata  ,  // 输入数据，总共包含4个像素的RGB值，每个像素24位，4*24=96位
    input                   I_tvalid ,  // 输入数据有效信号
    output                  I_tready ,  // 输入数据准备好信号

    output                  O_tlast  ,  // 输出行结束信号
    output                  O_tuser  ,  // 输出帧开始信号
    output [95:0]           O_tdata  ,  // 输出数据
    output                  O_tvalid ,  // 输出数据有效信号
    input                   O_tready    // 输出数据准备好信号
);

/*****************************************************************
                        将输入的96位数据切分为4个24位的像素，以便后续处理                                        
*****************************************************************/
    wire [23:0] I_tdata_r1 = I_tdata[23:0] ;    // 第1个像素数据
    wire [23:0] I_tdata_r2 = I_tdata[47:24];    // 第2个像素数据
    wire [23:0] I_tdata_r3 = I_tdata[71:48];    // 第3个像素数据
    wire [23:0] I_tdata_r4 = I_tdata[95:72];    // 第4个像素数据

    // 定义RGB888格式的数组，存储每个像素的R、G、B分量
    wire [7:0]  rgb888_r[3:0];
    wire [7:0]  rgb888_g[3:0];
    wire [7:0]  rgb888_b[3:0];

    // 从每个像素中提取R、G、B分量，注意位宽和位置
    assign   rgb888_r[0] = I_tdata_r1[16+:8];   // 第1个像素的R分量
    assign   rgb888_g[0] = I_tdata_r1[8+:8] ;   // 第1个像素的G分量
    assign   rgb888_b[0] = I_tdata_r1[0+:8] ;   // 第1个像素的B分量

    assign   rgb888_r[1] = I_tdata_r2[16+:8];   // 第2个像素的R分量
    assign   rgb888_g[1] = I_tdata_r2[8+:8] ;   // 第2个像素的G分量
    assign   rgb888_b[1] = I_tdata_r2[0+:8] ;   // 第2个像素的B分量

    assign   rgb888_r[2] = I_tdata_r3[16+:8];   // 第3个像素的R分量
    assign   rgb888_g[2] = I_tdata_r3[8+:8] ;   // 第3个像素的G分量
    assign   rgb888_b[2] = I_tdata_r3[0+:8] ;   // 第3个像素的B分量

    assign   rgb888_r[3] = I_tdata_r4[16+:8];   // 第4个像素的R分量
    assign   rgb888_g[3] = I_tdata_r4[8+:8] ;   // 第4个像素的G分量
    assign   rgb888_b[3] = I_tdata_r4[0+:8] ;   // 第4个像素的B分量

/*****************************************************************
                        信号声明部分                                         
*****************************************************************/
    // 定义调整值数组，每个像素一个调整值
    wire [8:0]   adjust_val[3:0];

    // 将参数ADJUST_VAL赋值给每个像素的调整值
    assign adjust_val[0] = ADJUST_VAL;
    assign adjust_val[1] = ADJUST_VAL;
    assign adjust_val[2] = ADJUST_VAL;
    assign adjust_val[3] = ADJUST_VAL;
    
    // 定义用于同步的寄存器，延迟输入的控制信号
    reg  [3:0]   I_tlast_r ;
    reg  [3:0]   I_tuser_r ;
    reg  [3:0]   I_tvalid_r;

    // 定义用于计算亮度Y的中间变量
    reg  [16:0]  Y_R_m[3:0];
    reg  [17:0]  Y_G_m[3:0];
    reg  [14:0]  Y_B_m[3:0];

    // 定义用于存储第一级流水线的RGB值
    reg  [7:0]   rgb888_r_r0[3:0];
    reg  [7:0]   rgb888_g_r0[3:0];
    reg  [7:0]   rgb888_b_r0[3:0];
    reg  [8:0]   RGB_C[3:0];
    
    // 定义Y的计算结果
    wire [17:0]  Y_w[3:0];
    reg  [7:0]   Y[3:0];

    // 定义用于存储第二级流水线的RGB值
    reg  [7:0]   rgb888_r_r1[3:0];
    reg  [7:0]   rgb888_g_r1[3:0];
    reg  [7:0]   rgb888_b_r1[3:0];

    // 定义Y_C的符号位和绝对值，用于判断调整值的正负
    reg          Y_C_sign[3:0];
    reg  [7:0]   Y_C_abs[3:0];

    // 定义用于存储第三级流水线的中间结果
    reg  [16:0]   Y_m[3:0];

    reg  [16:0]   rgb888_r_r2[3:0];
    reg  [16:0]   rgb888_g_r2[3:0];
    reg  [16:0]   rgb888_b_r2[3:0];

    // 定义Y_m_s等用于符号扩展的变量
    wire [18:0] Y_m_s[3:0];
    wire [18:0] Y_R_m_s[3:0];
    wire [18:0] Y_G_m_s[3:0];
    wire [18:0] Y_B_m_s[3:0];

    // 定义最终新的RGB值
    reg  [7:0]  R_new[3:0];
    reg  [7:0]  G_new[3:0];
    reg  [7:0]  B_new[3:0];
    

    // 定义用于亮度计算的常数系数
    localparam C0 = 9'd306;  // 0.299*1024，用于R分量
    localparam C1 = 10'd601; // 0.587*1024，用于G分量
    localparam C2 = 7'd117;  // 0.114*1024，用于B分量

/*****************************************************************
                    计算部分，实现饱和度调整                                         
*****************************************************************/

    genvar i;
    generate
        for(i = 0; i < 4; i = i + 1)
        begin: computing
        // 第一级流水线，计算Y的各部分并保存RGB值
        // Y=0.299*R+0.587*G+0.114*B
        always @(posedge I_clk or negedge I_rst_n) begin
            if(!I_rst_n) begin
                {Y_R_m[i], Y_G_m[i], Y_B_m[i]} <= 0;
                {rgb888_r_r0[i], rgb888_g_r0[i], rgb888_b_r0[i]} <= 0;
                RGB_C[i] <= 0;
            end 
            else if(I_tvalid)begin
                Y_R_m[i] <= rgb888_r[i]*C0; // R分量乘以系数C0
                Y_G_m[i] <= rgb888_g[i]*C1; // G分量乘以系数C1
                Y_B_m[i] <= rgb888_b[i]*C2; // B分量乘以系数C2
                {rgb888_r_r0[i], rgb888_g_r0[i], rgb888_b_r0[i]} <= {rgb888_r[i], rgb888_g[i], rgb888_b[i]};
                RGB_C[i] <= 255 + adjust_val[i]; // 计算RGB_C，用于后续计算
            end
        end
        // 将Y_R_m、Y_G_m、Y_B_m相加，得到Y_w
        assign Y_w[i] = Y_R_m[i] + Y_G_m[i] + Y_B_m[i];
    
        // 第二级流水线，对Y进行截取，并同步RGB值和调整值的符号与绝对值
        always @(posedge I_clk or negedge I_rst_n) begin
            if(!I_rst_n) begin
                Y[i] <= 0;
                {rgb888_r_r1[i], rgb888_g_r1[i], rgb888_b_r1[i]} <= 0;
                {Y_C_sign[i], Y_C_abs[i]} <= 0;
            end 
            else if(I_tvalid_r[0])begin
                Y[i] <= Y_w[i][17:10]; // 截取Y_w的高位，得到Y
                {rgb888_r_r1[i], rgb888_g_r1[i], rgb888_b_r1[i]} <= {rgb888_r_r0[i], rgb888_g_r0[i], rgb888_b_r0[i]};
                Y_C_sign[i] <= adjust_val[i][8]; // 判断调整值的符号位，最高位为符号
                Y_C_abs[i] <= adjust_val[i][8] ? (~adjust_val[i][7:0] + 1) : adjust_val[i];// 如果为负数，取补码+1，否则直接取值
            end
        end   
    
        // 第三级流水线，计算Y_m和RGB的中间值
        always @(posedge I_clk or negedge I_rst_n) begin
            if(!I_rst_n) begin
                Y_m[i] <= 0;
                {rgb888_r_r2[i], rgb888_g_r2[i], rgb888_b_r2[i]} <= 0;
            end 
            else if(I_tvalid_r[1])begin
                Y_m[i] <= Y[i]*Y_C_abs[i]; // Y乘以调整值的绝对值
                rgb888_r_r2[i] <= rgb888_r_r1[i]*RGB_C[i]; // R分量乘以RGB_C
                rgb888_g_r2[i] <= rgb888_g_r1[i]*RGB_C[i]; // G分量乘以RGB_C
                rgb888_b_r2[i] <= rgb888_b_r1[i]*RGB_C[i]; // B分量乘以RGB_C
            end
        end
        
        // 根据调整值的符号位，对Y_m取反或保持不变
        assign Y_m_s[i] = (~Y_C_sign[i]) ? (~{2'b0, Y_m[i]} + 1) : {2'b0, Y_m[i]};
    
        // 计算新的RGB分量
        assign Y_R_m_s[i] = Y_m_s[i] + rgb888_r_r2[i];
        assign Y_G_m_s[i] = Y_m_s[i] + rgb888_g_r2[i];
        assign Y_B_m_s[i] = Y_m_s[i] + rgb888_b_r2[i];
    
        // 第四级流水线，对计算结果进行饱和处理，防止溢出，并得到最终的RGB值
        always @(posedge I_clk or negedge I_rst_n) begin
            if(!I_rst_n) begin
                {R_new[i], G_new[i], B_new[i]} <= 0;
            end 
            else if(I_tvalid_r[2])begin
                // 判断结果是否为负数或溢出，进行裁剪
                R_new[i] <= Y_R_m_s[i][18] ? 0 : Y_R_m_s[i][17:16] > 0 ? 255 : Y_R_m_s[i][15:8];
                G_new[i] <= Y_G_m_s[i][18] ? 0 : Y_G_m_s[i][17:16] > 0 ? 255 : Y_G_m_s[i][15:8];
                B_new[i] <= Y_B_m_s[i][18] ? 0 : Y_B_m_s[i][17:16] > 0 ? 255 : Y_B_m_s[i][15:8];
            end
        end       
        end
    endgenerate
    
/*****************************************************************
                    同步输入的控制信号，使数据和控制信号对齐                                         
*****************************************************************/

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) 
            {I_tlast_r,I_tuser_r,I_tvalid_r} <= 0;
        else begin
            I_tlast_r <= {I_tlast_r[2:0],I_tlast};    // 同步I_tlast信号
            I_tuser_r <= {I_tuser_r[2:0],I_tuser};    // 同步I_tuser信号
            I_tvalid_r <= {I_tvalid_r[2:0],I_tvalid}; // 同步I_tvalid信号
        end
    end

    // 输出同步的控制信号
    assign  O_tlast  = I_tlast_r[3];   
    assign  O_tuser  = I_tuser_r[3];   
    assign  O_tvalid = I_tvalid_r[3];

    // 将新的RGB值组合成输出数据
    assign  O_tdata = {R_new[3],G_new[3],B_new[3],
                       R_new[2],G_new[2],B_new[2],
                       R_new[1],G_new[1],B_new[1],
                       R_new[0],G_new[0],B_new[0]};
    // 如果需要将RGB888转换为RGB565，可以使用以下代码进行转换
    // assign O_st_data = {R_new[7-:5],G_new[7-:6],B_new[7-:5]};
    // 输入就绪信号直接连接输出就绪信号
    assign  I_tready = O_tready;

endmodule
