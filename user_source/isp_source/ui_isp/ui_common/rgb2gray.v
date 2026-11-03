//=============================================================================
//  rgb2gray.v --- RGB888 转灰度（4 像素/拍，96bit -> 32bit）
//
//  来源：FPGA_lvjing/rtl/rgb2gray.v（原样搬入，只删了与本次集成无关的注释）
//
//  【公式】gray = (77*R + 150*G + 29*B) >> 8
//      系数和 = 77 + 150 + 29 = 256：
//        1) R=G=B=v 时结果恰好等于 v（灰轴恒等）；
//        2) 累加最大 256*255 = 65280，16bit 装得下，右移 8 位必落 0..255，
//           不需要钳位。
//
//  【位序】像素 p 在 I_rgb[p*24 +: 24] = {R[23:16], G[15:8], B[7:0]}
//          像素 p 的灰度在 O_gray[p*8 +: 8]
//
//  【延迟】纯组合，0 拍。
//=============================================================================
`timescale 1ns/1ps

module rgb2gray #(
    parameter integer PIX_PER_WORD = 4,
    parameter integer DATA_WIDTH   = PIX_PER_WORD * 24,   // 96
    parameter integer GRAY_WIDTH   = PIX_PER_WORD * 8     // 32
)(
    input  wire [DATA_WIDTH-1:0] I_rgb,
    output wire [GRAY_WIDTH-1:0] O_gray
);

    genvar p;
    generate
        for (p = 0; p < PIX_PER_WORD; p = p + 1) begin : g_px

            // 低位是 B、高位是 R —— 写成 r = I_rgb[p*24 +: 8] 会把系数对调
            wire [7:0]  b = I_rgb[p*24    +: 8];
            wire [7:0]  g = I_rgb[p*24+8  +: 8];
            wire [7:0]  r = I_rgb[p*24+16 +: 8];

            // 先零扩展到 16bit 再移位：移位结果位宽只跟随左操作数
            wire [15:0] r16 = {8'h00, r};
            wire [15:0] g16 = {8'h00, g};
            wire [15:0] b16 = {8'h00, b};

            // 77R + 150G + 29B，最大 65280，16bit 不溢出
            wire [15:0] acc = (r16 << 6) + (r16 << 3) + (r16 << 2) + r16
                            + (g16 << 7) + (g16 << 4) + (g16 << 2) + (g16 << 1)
                            + (b16 << 4) + (b16 << 3) + (b16 << 2) + b16;

            assign O_gray[p*8 +: 8] = acc[15:8];   // 等价于 >> 8
        end
    endgenerate

endmodule
