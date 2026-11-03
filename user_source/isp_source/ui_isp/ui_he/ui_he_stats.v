`timescale 1ns / 1ps
//=============================================================================
//  ui_he_stats.v --- histogram / CDF / strength-mixed LUT builder
//
//  Runs in the 74.24 MHz ISP domain.  One frame is IMG_W x IMG_H, 4 pixels
//  per word.  tuser = frame start, tlast = line end.
//
//  64 bins (Y>>2), four histograms (one per pixel lane), merged at frame end.
//  he[i] = (cdf[i] * NORM_K) >> NORM_S, LUT[i] = i + (he[i]-i)*strength/100.
//=============================================================================
module ui_he_stats #(
    parameter integer IMG_W = 1920,
    parameter integer IMG_H = 1080
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire [95:0] I_tdata,
    input  wire        I_tvalid,
    input  wire        I_tlast,
    input  wire        I_tuser,
    input  wire [7:0]  I_strength,

    output reg         O_lut_we,
    output reg  [7:0]  O_lut_addr,
    output reg  [7:0]  O_lut_data,
    output reg         O_lut_done
);

    localparam integer WPL    = IMG_W / 4;
    localparam integer TOTAL  = IMG_W * IMG_H;
    localparam integer NORM_K = (255 * 1048576) / TOTAL;  // 129 for 1080p
    localparam integer NORM_S = 20;

    wire [31:0] gray;
    rgb2gray u_gray (.I_rgb(I_tdata), .O_gray(gray));

    wire [5:0] bin0 = gray[7:2];
    wire [5:0] bin1 = gray[15:10];
    wire [5:0] bin2 = gray[23:18];
    wire [5:0] bin3 = gray[31:26];

    reg [15:0] hist0 [0:63];
    reg [15:0] hist1 [0:63];
    reg [15:0] hist2 [0:63];
    reg [15:0] hist3 [0:63];
    reg [7:0]  he_mem [0:63];

    reg [10:0] line_cnt;
    reg [2:0]  state;
    reg [5:0]  bin_idx;
    reg [7:0]  lut_idx;
    reg [20:0] cdf;
    reg [7:0]  strength_s;

    localparam S_IDLE = 3'd0, S_MERGE = 3'd1, S_LUT = 3'd2, S_CLEAR = 3'd3,
               S_TAIL = 3'd4, S_OFF = 3'd5;

    // ★ 2026-10-31 保中位数（=保整体亮度）：
    //   普通直方图均衡会把中位数硬拉到 128，等于给画面加了一个"自动提亮"，
    //   和前面的传感器自动曝光正面打架——AE 把输入亮度稳到 125，HE 又整体推高，
    //   用户调哪个旋钮都不对。这里在建表时整体减掉 he[mb]-mb*4，
    //   让中位灰度映射回它自己，HE 只加局部对比，不改整体亮度。
    reg [5:0]  med_idx;      // 中位 bin（CDF 第一次越过 N/2 的位置）
    reg        med_found;
    reg signed [8:0] off_r;  // he[med_idx] - med_idx*4，范围 -255..255

    wire [17:0] bin_sum = {2'b00, hist0[bin_idx]} + {2'b00, hist1[bin_idx]}
                        + {2'b00, hist2[bin_idx]} + {2'b00, hist3[bin_idx]};
    wire [20:0] cdf_next = cdf + {3'b000, bin_sum};

    function [7:0] clamp_lut;
        input [7:0] base;
        input signed [17:0] mix;
        reg signed [17:0] q;
        reg signed [17:0] v;
        begin
            q = mix / 100;               // constant divisor, synthesizes small
            v = $signed({1'b0, base}) + q;
            if (v < 0) clamp_lut = 0;
            else if (v > 255) clamp_lut = 255;
            else clamp_lut = v[7:0];
        end
    endfunction

    // line count (tlast is line end)
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            line_cnt <= 0;
        end else begin
            if (I_tuser) line_cnt <= 0;
            else if (I_tvalid && I_tlast) line_cnt <= line_cnt + 1'b1;
        end
    end

    // histogram read-modify-write (4 lanes); S_CLEAR zeroes one bin per cycle
    always @(posedge I_clk) begin
        if (state == S_IDLE && I_tvalid) begin
            hist0[bin0] <= hist0[bin0] + 1'b1;
            hist1[bin1] <= hist1[bin1] + 1'b1;
            hist2[bin2] <= hist2[bin2] + 1'b1;
            hist3[bin3] <= hist3[bin3] + 1'b1;
        end else if (state == S_CLEAR) begin
            hist0[bin_idx] <= 16'd0;
            hist1[bin_idx] <= 16'd0;
            hist2[bin_idx] <= 16'd0;
            hist3[bin_idx] <= 16'd0;
        end
    end

    // sequencer
    integer j;
    reg signed [10:0] diff;    // he[bin]-Y-off_r，最坏 -510..510，11 位
    reg signed [17:0] mix_r;   // 流水寄存器：上一项的 mix
    reg [7:0]         idx_r;   // 流水寄存器：上一项的 lut_idx

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            // power-up clear: zero all 4x64 histogram entries first
            state <= S_CLEAR;
            bin_idx <= 0; lut_idx <= 0; cdf <= 0;
            strength_s <= 0;
            med_idx <= 6'd0; med_found <= 1'b0; off_r <= 9'sd0;
            O_lut_we <= 0; O_lut_addr <= 0; O_lut_data <= 0; O_lut_done <= 0;
            for (j = 0; j < 64; j = j + 1) he_mem[j] <= 0;
        end else begin
            O_lut_we   <= 1'b0;
            O_lut_done <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (I_tvalid && I_tlast && (line_cnt == IMG_H-1)) begin
                        state <= S_MERGE;
                        bin_idx <= 0;
                        cdf <= 0;
                        med_found <= 1'b0;
                        med_idx   <= 6'd0;
                        strength_s <= I_strength;
                    end
                end

                S_MERGE: begin
                    cdf <= cdf_next;
                    he_mem[bin_idx] <= (cdf_next * NORM_K) >> NORM_S;
                    // 中位 bin：CDF 第一次越过一半像素的位置
                    if (!med_found && cdf_next >= (TOTAL/2)) begin
                        med_found <= 1'b1;
                        med_idx   <= bin_idx;
                    end
                    if (bin_idx == 63) begin
                        state <= S_OFF;
                        if (!med_found) med_idx <= 6'd63;
                    end else begin
                        bin_idx <= bin_idx + 1'b1;
                    end
                end

                S_OFF: begin
                    // 中位数偏移量：让中位灰度经过 LUT 后仍等于它自己
                    off_r   <= $signed({1'b0, he_mem[med_idx]})
                             - $signed({1'b0, med_idx, 2'b00});
                    lut_idx <= 8'd0;
                    state   <= S_LUT;
                end

                // ★ 2026-10-31：原来"读 he_mem -> 乘 strength -> 除 100 -> 加 -> 限幅"
                //   全压在一拍里，综合出 23 级逻辑 / 14.03ns，是本次 P&R 里
                //   clk1(74.24MHz) 唯一的时序违例（-755ps，8 个端点）。
                //   拆成"算下一项 / 写上一项"两拍流水：吞吐不变（仍是一片一项），
                //   关键路径只剩 clamp_lut 这一段。
                S_LUT: begin
                    if (lut_idx != 8'd0) begin
                        O_lut_we   <= 1'b1;
                        O_lut_addr <= idx_r;
                        O_lut_data <= clamp_lut(idx_r, mix_r);
                    end
                    diff  = ($signed({1'b0, he_mem[lut_idx[7:2]]})
                             - $signed({1'b0, lut_idx})) - off_r;   // 减掉中位偏移 -> 保亮度
                    mix_r <= diff * $signed({1'b0, strength_s});
                    idx_r <= lut_idx;
                    if (lut_idx == 8'd255) state <= S_TAIL;
                    else                    lut_idx <= lut_idx + 1'b1;
                end

                S_TAIL: begin
                    // 补写最后一项（lut_idx=255）
                    O_lut_we   <= 1'b1;
                    O_lut_addr <= idx_r;
                    O_lut_data <= clamp_lut(idx_r, mix_r);
                    state   <= S_CLEAR;
                    bin_idx <= 0;
                end

                S_CLEAR: begin
                    if (bin_idx == 63) begin
                        state <= S_IDLE;
                        O_lut_done <= 1'b1;
                    end else begin
                        bin_idx <= bin_idx + 1'b1;
                    end
                end
            endcase
        end
    end
endmodule
