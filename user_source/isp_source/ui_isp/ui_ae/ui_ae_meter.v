//=============================================================================
//  ui_ae_meter.v --- 自动曝光测光（第一阶段：只测，不控）
//
//  【用途】统计每一帧的画面亮度，供 AE 控制环和 OSD 显示使用。
//
//  【输入】ISP 链输出的 4 像素/字 RGB888 流（滤镜域时钟，空档不算）。
//          像素位序同 ui_filter_domain：p*24+16 = R, +8 = G, +0 = B。
//
//  【输出】
//      O_mean   本帧平均亮度 0..255
//      O_peak   本帧最大亮度 0..255（用来看是否过曝）
//      O_valid  每帧一个脉冲（帧首更新后才有效）
//
//  【为什么不做真除法】
//      每一帧的像素数固定 1920×1080 = 2,073,600 ≈ 2^21 × 0.9888。
//      用 sum >> 21 近似相除，误差约 1.1%——对"显示亮度"和 AE 的
//      目标值 ±5% 死区来说完全够用，还省掉一个 40/22 位的除法器。
//      如果将来帧尺寸变了，改 MEAN_SHIFT 这一个参数即可。
//
//  【亮度公式】luma = (R + 2G + B) >> 2
//      标准 BT.601 是全精度 77/150/29，这里要乘 3 次；用 (R+2G+B)/4 只需要
//      加法，误差对测光无影响（测光看的是相对变化）。
//=============================================================================
`timescale 1ns/1ps

module ui_ae_meter #(
    parameter integer MEAN_SHIFT = 21      // log2(每帧像素数)
)(
    input  wire        I_clk,
    input  wire        I_rst_n,

    input  wire [95:0] I_tdata,            // 4 像素 × RGB888
    input  wire        I_tvalid,
    input  wire        I_tlast,
    input  wire        I_tuser,            // 帧首

    output reg  [7:0]  O_mean,
    output reg  [7:0]  O_peak,
    output reg         O_valid
);

    //-------------------------------------------------------------------------
    // 一个字的 4 个像素亮度求和（每个 0..255，合计 0..1020）
    //-------------------------------------------------------------------------
    function [9:0] pix_luma;
        input [23:0] p;
        reg   [9:0]  r, g, b;
        begin
            r = {2'b00, p[23:16]};
            g = {2'b00, p[15:8]};
            b = {2'b00, p[7:0]};
            // (R + 2G + B) >> 2
            pix_luma = (r + {g[8:0], 1'b0} + b) >> 2;
        end
    endfunction

    wire [9:0] l0 = pix_luma(I_tdata[23:0]);
    wire [9:0] l1 = pix_luma(I_tdata[47:24]);
    wire [9:0] l2 = pix_luma(I_tdata[71:48]);
    wire [9:0] l3 = pix_luma(I_tdata[95:72]);
    wire [11:0] wsum = l0 + l1 + l2 + l3;      // 0..1020

    //-------------------------------------------------------------------------
    // 帧内累加
    //-------------------------------------------------------------------------
    reg [31:0] acc;        // ≤ 255*2073600 = 5.29e8，32 位够
    reg [7:0]  peak;

    wire [11:0] peak_cand = {2'b00, l0} ;
    wire [9:0]  wmax = (l0 > l1 ? l0 : l1) > (l2 > l3 ? l2 : l3)
                       ? (l0 > l1 ? l0 : l1) : (l2 > l3 ? l2 : l3);

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            acc   <= 32'd0;
            peak  <= 8'd0;
            O_mean  <= 8'd0;
            O_peak  <= 8'd0;
            O_valid <= 1'b0;
        end
        else begin
            O_valid <= 1'b0;

            if (I_tvalid && I_tuser) begin
                // 帧首：把上一帧的结果锁出去，然后重新开始累加
                O_mean  <= (acc >> MEAN_SHIFT) > 32'd255 ? 8'd255
                                                         : acc[MEAN_SHIFT +: 8];
                O_peak  <= peak;
                O_valid <= 1'b1;
                acc     <= wsum;
                peak    <= wmax;
            end
            else if (I_tvalid) begin
                acc  <= acc + wsum;
                if (wmax > peak) peak <= wmax;
            end
        end
    end

    wire unused_ok = |{I_tlast, peak_cand};

endmodule
