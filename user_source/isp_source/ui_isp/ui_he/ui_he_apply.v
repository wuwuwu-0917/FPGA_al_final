`timescale 1ns / 1ps
//=============================================================================
//  ui_he_apply.v --- histogram-equalization lookup + RGB offset
//
//  Two 256x8 ERAM banks, contents duplicated.  During active pixels both
//  ports of each bank read (4 reads per cycle for 4 pixels).  During the
//  blanking-time table update both ports write the same address/data.
//
//  Y' = LUT[Y]; delta = Y' - Y; R/G/B each get delta and clamp to 0..255.
//  latency = 2 cycles.
//=============================================================================
module ui_he_apply (
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire [95:0] I_tdata,
    input  wire        I_tvalid,
    input  wire        I_tlast,
    input  wire        I_tuser,
    input  wire        I_lut_we,
    input  wire [7:0]  I_lut_addr,
    input  wire [7:0]  I_lut_data,
    // ★ 2026-10-31 失效保护：查表还没建好（上电第一帧，或统计状态机
    //   因为任何原因没跑完）时直接旁路，画面保持原样，绝不会被压黑。
    input  wire        I_lut_done,
    output reg  [95:0] O_tdata,
    output reg         O_tvalid,
    output reg         O_tlast,
    output reg         O_tuser
);

    wire [31:0] gray;
    rgb2gray u_gray (.I_rgb(I_tdata), .O_gray(gray));

    // ★ 2026-10-31 ERAM 吃紧（合并隐藏 UI 后 277/272）：查表只有 4x256x8 = 8Kb，
    //   改为**分布式 RAM**，省 2 块 ERAM，代价约 128 个 LUT（LUT 有余量）。
    //   原来的双口 ERAM 一拍读 4 个像素；分布式每个阵列只有 1 个读口，
    //   所以存 4 份（内容完全相同，写时一起写），各负责一个像素通道。
    (* ram_style = "dram" *) reg [7:0] lut_g0 [0:255];
    (* ram_style = "dram" *) reg [7:0] lut_g1 [0:255];
    (* ram_style = "dram" *) reg [7:0] lut_g2 [0:255];
    (* ram_style = "dram" *) reg [7:0] lut_g3 [0:255];

    reg [7:0] doa0, dob0, doa1, dob1;

    always @(posedge I_clk) begin
        if (I_lut_we) begin
            lut_g0[I_lut_addr] <= I_lut_data;
            lut_g1[I_lut_addr] <= I_lut_data;
            lut_g2[I_lut_addr] <= I_lut_data;
            lut_g3[I_lut_addr] <= I_lut_data;
        end
        doa0 <= lut_g0[gray[7:0]];
        dob0 <= lut_g1[gray[15:8]];
        doa1 <= lut_g2[gray[23:16]];
        dob1 <= lut_g3[gray[31:24]];
    end

    reg [7:0] y0_d, y1_d, y2_d, y3_d;
    reg [95:0] pix_d;
    reg vld_d, lst_d, usr_d;
    reg lut_ready;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) lut_ready <= 1'b0;
        else if (I_lut_done) lut_ready <= 1'b1;
    end

    function [23:0] apply_px;
        input [23:0] px;
        input [7:0]  y_new;
        input [7:0]  y_old;
        reg signed [8:0]  d;
        reg signed [9:0]  r, g, b;
        begin
            d = $signed({1'b0, y_new}) - $signed({1'b0, y_old});
            r = $signed({1'b0, px[23:16]}) + d;
            g = $signed({1'b0, px[15:8]})  + d;
            b = $signed({1'b0, px[7:0]})   + d;
            if (r < 0) r = 0; else if (r > 255) r = 255;
            if (g < 0) g = 0; else if (g > 255) g = 255;
            if (b < 0) b = 0; else if (b > 255) b = 255;
            apply_px = {r[7:0], g[7:0], b[7:0]};
        end
    endfunction

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            y0_d <= 0; y1_d <= 0; y2_d <= 0; y3_d <= 0;
            pix_d <= 0; vld_d <= 0; lst_d <= 0; usr_d <= 0;
            O_tdata <= 0; O_tvalid <= 0; O_tlast <= 0; O_tuser <= 0;
        end else begin
            y0_d <= gray[7:0];
            y1_d <= gray[15:8];
            y2_d <= gray[23:16];
            y3_d <= gray[31:24];
            pix_d <= I_tdata;
            vld_d <= I_tvalid;
            lst_d <= I_tlast;
            usr_d <= I_tuser;

            // 查表未就绪 -> 原样输出（旁路），就绪后走均衡
            if (!lut_ready)
                O_tdata <= pix_d;
            else
                O_tdata <= {apply_px(pix_d[95:72], dob1, y3_d),
                            apply_px(pix_d[71:48], doa1, y2_d),
                            apply_px(pix_d[47:24], dob0, y1_d),
                            apply_px(pix_d[23:0],  doa0, y0_d)};
            O_tvalid <= vld_d;
            O_tlast  <= lst_d;
            O_tuser  <= usr_d;
        end
    end
endmodule
