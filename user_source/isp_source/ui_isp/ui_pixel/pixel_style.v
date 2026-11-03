//=============================================================================
//  pixel_style.v --- 像素风滤镜（D/E/F 三档），主工程接口版
//
//  算法来源：D:\VS Code Project\FPGA_test\xiangsu\rtl\pixel_style_opt.v
//  算法、色板、抖动、描边、rounding 常数一律不变；本文件只做：
//    1) Verilog-2001（去掉二维数组）
//    2) 主工程接口：I_style 2bit（0=D 4px lab32 / 1=E 8px DB32 / 2=F 8px PICO8）
//    3) tuser 帧首复位行/帧计数，并按有效拍推进（主工程每行有 478 拍空档）
//    4) 行宽/帧高参数化（主工程 480 字/行、1080 行）
//
//  ⚠️ 本版仍是**行为级**（数组异步读），用于先和 xiangsu 参考输出逐像素对拍。
//     正式并主 RTL 前，由后续步骤把 hs4/vsum4/ring4/group_buf/map/palette
//     改成显式 ERAM + 同步读（见 2026-10-28-pixel-filter-integration-design.md §3.3）。
//
//  延迟（有效拍）：8 行 + 1 拍输出寄存器；同步 BRAM 化后还会加 2~3 拍，
//  由 ui_filter_domain_new 的 ALIGN_DELAY 吸收。
//=============================================================================
`timescale 1ns/1ps

module pixel_style #(
    parameter integer WORDS_PER_LINE  = 480,
    parameter integer LINES_PER_FRAME = 1080,
    parameter integer RGB_W           = 96
)(
    input  wire                  I_clk,
    input  wire                  I_rst_n,

    input  wire [RGB_W-1:0]      I_tdata,
    input  wire                  I_tvalid,
    input  wire                  I_tlast,
    input  wire                  I_tuser,
    input  wire [1:0]            I_style,     // 0=D(4px lab32) 1=E(8px DB32) 2=F(8px PICO8)

    // 表初始化写口（仿真用；正式版改成 ERAM INIT_FILE）
    input  wire                  I_map_we,
    input  wire [11:0]           I_map_addr,
    input  wire [7:0]            I_map_dat,
    input  wire                  I_pal_we,
    input  wire [9:0]            I_pal_addr,
    input  wire [23:0]           I_pal_dat,

    output reg  [RGB_W-1:0]      O_tdata,
    output reg                   O_tvalid,
    output reg                   O_tlast,
    output reg                   O_tuser
);

    localparam integer LAST_W = WORDS_PER_LINE - 1;
    localparam integer LAST_L = LINES_PER_FRAME - 1;
    localparam integer HALF_W = WORDS_PER_LINE / 2;     // 240

    //------------------------------------------------------------------
    // 表
    //------------------------------------------------------------------
    reg [7:0]  map_ram [0:4095];
    reg [23:0] pal_ram [0:767];
    always @(posedge I_clk) begin
        if (I_map_we) map_ram[I_map_addr] <= I_map_dat;
        if (I_pal_we) pal_ram[I_pal_addr] <= I_pal_dat;
    end

    //------------------------------------------------------------------
    // 输入计数 / tuser 帧首复位
    //   当前拍的行/字索引用 lnum/wnum（tuser 时强制 0），
    //   寄存器在时钟沿更新；这样 tuser 那一拍就已经按第 0 行第 0 字处理。
    //------------------------------------------------------------------
    reg [15:0] wcnt, rcnt;
    wire first = I_tvalid && I_tuser;
    wire [15:0] lnum = first ? 16'd0 : rcnt;
    wire [15:0] wnum = first ? 16'd0 : wcnt;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            wcnt <= 16'd0;
            rcnt <= 16'd0;
        end else if (I_tvalid) begin
            if (first) begin
                // 当前拍就是第 0 字；下一拍必须是第 1 字。
                // 这里若写 0，整帧会晚一个字（4 像素），和 line_window_3x3 的约定一致。
                wcnt <= 16'd1;
                rcnt <= 16'd0;
            end else if (wcnt == LAST_W) begin
                wcnt <= 16'd0;
                rcnt <= (rcnt == LAST_L) ? 16'd0 : (rcnt + 16'd1);
            end else begin
                wcnt <= wcnt + 16'd1;
            end
        end
    end

    // 风格在帧首锁存，整帧不变
    reg [1:0] style_f;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)      style_f <= 2'd0;
        else if (first)    style_f <= I_style;
    end

    wire       block8 = (style_f != 2'd0);   // D=4px，E/F=8px
    wire [1:0] pal_sel = style_f;

    wire [15:0] drcnt = (lnum >= 16'd8) ? (lnum - 16'd8)
                                        : (lnum + LINES_PER_FRAME - 16'd8);
    wire [15:0] by4   = drcnt >> 2;
    wire [15:0] by8   = drcnt >> 3;

    //------------------------------------------------------------------
    // 输入像素 / 4 像素水平
    //------------------------------------------------------------------
    wire [7:0] r0 = I_tdata[23:16], g0 = I_tdata[15:8],  b0 = I_tdata[7:0];
    wire [7:0] r1 = I_tdata[47:40], g1 = I_tdata[39:32], b1 = I_tdata[31:24];
    wire [7:0] r2 = I_tdata[71:64], g2 = I_tdata[63:56], b2 = I_tdata[55:48];
    wire [7:0] r3 = I_tdata[95:88], g3 = I_tdata[87:80], b3 = I_tdata[79:72];

    wire [9:0] hr = r0 + r1 + r2 + r3;
    wire [9:0] hg = g0 + g1 + g2 + g3;
    wire [9:0] hb = b0 + b1 + b2 + b3;

    //------------------------------------------------------------------
    // 4px 滚动和 / 4×4 块色
    //------------------------------------------------------------------
    reg [29:0] hs4   [0:4*WORDS_PER_LINE-1];
    reg [35:0] vsum4 [0:WORDS_PER_LINE-1];

    wire [10:0] a4   = lnum[1:0] * WORDS_PER_LINE + wnum[8:0];
    wire [29:0] old4 = hs4[a4];
    wire [35:0] cur4 = vsum4[wnum[8:0]];

    wire [11:0] vr4 = (lnum == 16'd0) ? {2'b0, hr} :
                      (lnum <  16'd4) ? (cur4[35:24] + {2'b0, hr}) :
                      (cur4[35:24] + {2'b0, hr} - {2'b0, old4[29:20]});
    wire [11:0] vg4 = (lnum == 16'd0) ? {2'b0, hg} :
                      (lnum <  16'd4) ? (cur4[23:12] + {2'b0, hg}) :
                      (cur4[23:12] + {2'b0, hg} - {2'b0, old4[19:10]});
    wire [11:0] vb4 = (lnum == 16'd0) ? {2'b0, hb} :
                      (lnum <  16'd4) ? (cur4[11:0]  + {2'b0, hb}) :
                      (cur4[11:0]  + {2'b0, hb} - {2'b0, old4[9:0]});

    wire [12:0] vr4p = {1'b0, vr4} + 13'd8;
    wire [12:0] vg4p = {1'b0, vg4} + 13'd8;
    wire [12:0] vb4p = {1'b0, vb4} + 13'd8;
    wire [23:0] blk4 = {vr4p[11:4], vg4p[11:4], vb4p[11:4]};

    always @(posedge I_clk) begin
        if (I_tvalid) begin
            hs4[a4]   <= {hr, hg, hb};
            vsum4[wnum[8:0]] <= {vr4, vg4, vb4};
        end
    end

    //------------------------------------------------------------------
    // tile 行 0..3 环（4 个一维数组，避免二维数组）
    //------------------------------------------------------------------
    reg [1:0] tile4;
    wire tile4_end = I_tvalid && (wnum == LAST_W) && (lnum[1:0] == 2'd3);
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)      tile4 <= 2'd0;
        else if (first)    tile4 <= 2'd0;
        else if (tile4_end) tile4 <= tile4 + 2'd1;
    end

    reg [23:0] ring4_0 [0:WORDS_PER_LINE-1];
    reg [23:0] ring4_1 [0:WORDS_PER_LINE-1];
    reg [23:0] ring4_2 [0:WORDS_PER_LINE-1];
    reg [23:0] ring4_3 [0:WORDS_PER_LINE-1];

    always @(posedge I_clk) begin
        if (I_tvalid && (lnum[1:0] == 2'd3)) begin
            case (tile4)
                2'd0: ring4_0[wnum[8:0]] <= blk4;
                2'd1: ring4_1[wnum[8:0]] <= blk4;
                2'd2: ring4_2[wnum[8:0]] <= blk4;
                default: ring4_3[wnum[8:0]] <= blk4;
            endcase
        end
    end

    // 输出 T-2；up=T-3, down=T-1
    wire [1:0] t4_cur  = tile4 + 2'd2;
    wire [1:0] t4_up   = tile4 + 2'd1;
    wire [1:0] t4_dn   = tile4 + 2'd3;
    wire [1:0] t4_prev = tile4 + 2'd3;   // 8px 分组用：Q-1

    wire [8:0] w4r = (wnum == LAST_W) ? wnum[8:0] : (wnum[8:0] + 9'd1);
    wire [8:0] w4p = (wnum == 16'd0) ? (WORDS_PER_LINE - 1) : (wnum[8:0] - 9'd1);

    // 4px 读（4 个数组按 tile 选择）
    reg [23:0] c4_cur, c4_raw, c4_up, c4_dn, gprev0, gprev1;
    always @(*) begin
        case (t4_cur)
            2'd0: begin c4_cur = ring4_0[wnum[8:0]]; c4_raw = ring4_0[w4r]; end
            2'd1: begin c4_cur = ring4_1[wnum[8:0]]; c4_raw = ring4_1[w4r]; end
            2'd2: begin c4_cur = ring4_2[wnum[8:0]]; c4_raw = ring4_2[w4r]; end
            default: begin c4_cur = ring4_3[wnum[8:0]]; c4_raw = ring4_3[w4r]; end
        endcase
        case (t4_up)
            2'd0: c4_up = ring4_0[wnum[8:0]];
            2'd1: c4_up = ring4_1[wnum[8:0]];
            2'd2: c4_up = ring4_2[wnum[8:0]];
            default: c4_up = ring4_3[wnum[8:0]];
        endcase
        case (t4_dn)
            2'd0: c4_dn = ring4_0[wnum[8:0]];
            2'd1: c4_dn = ring4_1[wnum[8:0]];
            2'd2: c4_dn = ring4_2[wnum[8:0]];
            default: c4_dn = ring4_3[wnum[8:0]];
        endcase
        case (t4_prev)
            2'd0: begin gprev0 = ring4_0[w4p]; gprev1 = ring4_0[wnum[8:0]]; end
            2'd1: begin gprev0 = ring4_1[w4p]; gprev1 = ring4_1[wnum[8:0]]; end
            2'd2: begin gprev0 = ring4_2[w4p]; gprev1 = ring4_2[wnum[8:0]]; end
            default: begin gprev0 = ring4_3[w4p]; gprev1 = ring4_3[wnum[8:0]]; end
        endcase
    end

    reg [23:0] c4_left;
    always @(posedge I_clk) if (I_tvalid) c4_left <= c4_cur;
    wire [23:0] c4_l = (wnum == 16'd0) ? c4_cur : c4_left;
    wire [23:0] c4_r = (wnum == LAST_W) ? c4_cur : c4_raw;

    wire [23:0] c4_u = (by4 == 16'd0) ? c4_cur : c4_up;
    wire [23:0] c4_d = (by4 == (LINES_PER_FRAME/4 - 1)) ? c4_cur : c4_dn;

    function [7:0] lum8;
        input [23:0] c;
        begin
            lum8 = (77 * c[23:16] + 150 * c[15:8] + 29 * c[7:0]) >> 8;
        end
    endfunction

    function edge_hi;
        input [7:0] a, b;
        input [7:0] thr;
        begin
            edge_hi = ((a > b) && ((a - b) > thr));
        end
    endfunction

    wire [7:0] lum4c = lum8(c4_cur);
    wire [7:0] lum4l = lum8(c4_l);
    wire [7:0] lum4r = lum8(c4_r);
    wire [7:0] lum4u = lum8(c4_u);
    wire [7:0] lum4d = lum8(c4_d);

    wire m4 = edge_hi(lum4c, lum4l, 8'd50) | edge_hi(lum4c, lum4r, 8'd50) |
              edge_hi(lum4c, lum4u, 8'd50) | edge_hi(lum4c, lum4d, 8'd50);

    wire [7:0] o4r = m4 ? ((c4_cur[23:16] * 141) >> 8) : c4_cur[23:16];
    wire [7:0] o4g = m4 ? ((c4_cur[15:8]  * 141) >> 8) : c4_cur[15:8];
    wire [7:0] o4b = m4 ? ((c4_cur[7:0]   * 141) >> 8) : c4_cur[7:0];
    wire [23:0] c4_out = {o4r, o4g, o4b};

    //------------------------------------------------------------------
    // 8px 分组：2×2 个 4×4 块色 → 2 行 group 缓存
    //------------------------------------------------------------------
    reg [23:0] blk4_prev;
    always @(posedge I_clk) if (I_tvalid) blk4_prev <= blk4;

    wire [9:0] gc_r = {2'b0, blk4_prev[23:16]} + {2'b0, blk4[23:16]} +
                      {2'b0, gprev0[23:16]}     + {2'b0, gprev1[23:16]} + 10'd2;
    wire [9:0] gc_g = {2'b0, blk4_prev[15:8]}  + {2'b0, blk4[15:8]} +
                      {2'b0, gprev0[15:8]}      + {2'b0, gprev1[15:8]} + 10'd2;
    wire [9:0] gc_b = {2'b0, blk4_prev[7:0]}   + {2'b0, blk4[7:0]} +
                      {2'b0, gprev0[7:0]}       + {2'b0, gprev1[7:0]} + 10'd2;
    wire [23:0] gcol = {gc_r[9:2], gc_g[9:2], gc_b[9:2]};

    reg [23:0] group_buf [0:2*HALF_W-1];
    wire group_active = block8 && I_tvalid && lnum[2] && wnum[0];
    wire [7:0] gj = wnum[8:1];
    wire       gslot_w = lnum[3];
    // 展平索引 = slot*HALF_W + gj；不能用 {slot,gj}（会变成 slot*256+gj，越界）
    wire [8:0] gidx_w = (gslot_w ? HALF_W : 0) + gj;

    always @(posedge I_clk) begin
        if (group_active)
            group_buf[gidx_w] <= gcol;
    end

    // 8px 输出（左右描边）
    wire [7:0]  gj_r = (gj == HALF_W-1) ? gj : (gj + 8'd1);
    wire        gslot_r = drcnt[3];
    wire [8:0]  gidx_r  = (gslot_r ? HALF_W : 0) + gj;
    wire [8:0]  gidx_rr = (gslot_r ? HALF_W : 0) + gj_r;
    wire [23:0] g_cur = group_buf[gidx_r];
    wire [23:0] g_raw = group_buf[gidx_rr];

    reg [23:0] g_left;
    always @(posedge I_clk) if (I_tvalid && wnum[0]) g_left <= g_cur;
    wire [23:0] g_l = (gj == 8'd0) ? g_cur : g_left;
    wire [23:0] g_r = (gj == HALF_W-1) ? g_cur : g_raw;

    wire [7:0] lum8c = lum8(g_cur);
    wire [7:0] lum8l = lum8(g_l);
    wire [7:0] lum8r = lum8(g_r);

    wire m8 = edge_hi(lum8c, lum8l, 8'd60) | edge_hi(lum8c, lum8r, 8'd60);

    wire [7:0] o8r = m8 ? ((g_cur[23:16] * 141) >> 8) : g_cur[23:16];
    wire [7:0] o8g = m8 ? ((g_cur[15:8]  * 141) >> 8) : g_cur[15:8];
    wire [7:0] o8b = m8 ? ((g_cur[7:0]   * 141) >> 8) : g_cur[7:0];
    wire [23:0] c8_out = {o8r, o8g, o8b};

    //------------------------------------------------------------------
    // 选路 + 抖动 + 查表
    //------------------------------------------------------------------
    wire [23:0] csel = block8 ? c8_out : c4_out;
    wire [15:0] bx   = block8 ? {1'b0, wnum[15:1]} : wnum;
    wire [15:0] by   = block8 ? by8 : by4;

    function signed [7:0] dith8f;
        input [1:0] y, x;
        begin
            case ({y, x})
                4'd0: dith8f = -4; 4'd1: dith8f =  0; 4'd2: dith8f = -3; 4'd3: dith8f =  1;
                4'd4: dith8f =  2; 4'd5: dith8f = -2; 4'd6: dith8f =  3; 4'd7: dith8f = -1;
                4'd8: dith8f = -2; 4'd9: dith8f =  2; 4'd10: dith8f = -3; 4'd11: dith8f = 1;
                4'd12: dith8f = 4; 4'd13: dith8f = 0; 4'd14: dith8f =  3; 4'd15: dith8f = -1;
                default: dith8f = 0;
            endcase
        end
    endfunction

    function signed [7:0] dith12f;
        input [1:0] y, x;
        begin
            case ({y, x})
                4'd0: dith12f = -6; 4'd1: dith12f =  0; 4'd2: dith12f = -4; 4'd3: dith12f =  2;
                4'd4: dith12f =  3; 4'd5: dith12f = -3; 4'd6: dith12f =  5; 4'd7: dith12f = -1;
                4'd8: dith12f = -3; 4'd9: dith12f =  3; 4'd10: dith12f = -5; 4'd11: dith12f = 1;
                4'd12: dith12f = 6; 4'd13: dith12f = 0; 4'd14: dith12f =  4; 4'd15: dith12f = -2;
                default: dith12f = 0;
            endcase
        end
    endfunction

    wire signed [7:0] doff = (pal_sel == 2'd2) ? dith12f(by[1:0], bx[1:0])
                                               : dith8f (by[1:0], bx[1:0]);

    wire signed [9:0] srr = $signed({2'b0, csel[23:16]}) + $signed(doff);
    wire signed [9:0] sgg = $signed({2'b0, csel[15:8] }) + $signed(doff);
    wire signed [9:0] sbb = $signed({2'b0, csel[7:0]  }) + $signed(doff);

    wire [7:0] dr = (srr < 0) ? 8'd0 : (srr > 255) ? 8'd255 : srr[7:0];
    wire [7:0] dg = (sgg < 0) ? 8'd0 : (sgg > 255) ? 8'd255 : sgg[7:0];
    wire [7:0] db = (sbb < 0) ? 8'd0 : (sbb > 255) ? 8'd255 : sbb[7:0];

    wire [11:0] map_addr = {dr[7:4], dg[7:4], db[7:4]};
    wire [7:0]  pal_idx  = map_ram[map_addr];
    wire [23:0] pal_col  = pal_ram[{pal_sel, pal_idx}];

    //------------------------------------------------------------------
    // 输出 / 计数器生成同步
    //------------------------------------------------------------------
    wire vld = I_tvalid && (lnum >= 16'd8);

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_tdata  <= {RGB_W{1'b0}};
            O_tvalid <= 1'b0;
            O_tlast  <= 1'b0;
            O_tuser  <= 1'b0;
        end else begin
            O_tdata  <= {pal_col, pal_col, pal_col, pal_col};
            O_tvalid <= vld;
            O_tlast  <= vld && (wnum == LAST_W);
            O_tuser  <= vld && (wnum == 16'd0) && (drcnt == 16'd0);
        end
    end

    // 源文件接口里的 tlast 不参与运算，显式消费
    wire unused_ok = I_tlast;

endmodule
