//=============================================================================
//  pixel_style_bram.v --- 像素风滤镜（D/E/F）的 ERAM 版
//
//  算法与 pixel_style.v（xiangsu/pixel_style_opt.v 的移植）完全一致；
//  本文件只把存储改成同步读的 ERAM，并把数据路径按读延迟重定时。
//
//  存储（每块都是 1 读 + 1 写，TD 可推 ERAM）：
//    hs4        4×480×30   滚动水平 4 行和
//    vsum4      480×36      4 行垂直和
//    ring4_0..3 480×24 ×4   tile 行块色环
//    group_b0/1 240×24 ×2   8px group 缓存（两个 slot 分开，读写不同 slot）
//    map/palette            用 pixel_rom（PH1_LOGIC_ERAM + INIT_FILE）
//
//  流水（每个 vld 拍推进一级）：
//    s0 输入/发读地址
//    s1 读 hs4/vsum → 算 vsum/blk4 → 写 hs4/vsum/ring；发 ring 读
//    s2 ring 数据 → 算 gcol/写 group；发 c4_raw 读
//    s3 c4_raw → 算 4px 描边；发 g_cur 读
//    s4 g_cur → 发 g_raw 读
//    s5 g_raw → 算 8px 描边/选路/抖动/发 map 读
//    s6 map 数据 → 发 palette 读
//    s7 palette 数据 → 输出寄存器
//
//  ⚠️ 延迟：8 行 + 7 级流水。ui_filter_domain_new 里的 PIXEL_LAT 必须同步。
//=============================================================================
`timescale 1ns/1ps

module pixel_style_bram #(
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
    input  wire [1:0]            I_style,

    output reg  [RGB_W-1:0]      O_tdata,
    output reg                   O_tvalid,
    output reg                   O_tlast,
    output reg                   O_tuser
);

    localparam integer LAST_W = WORDS_PER_LINE - 1;
    localparam integer LAST_L = LINES_PER_FRAME - 1;
    localparam integer HALF_W = WORDS_PER_LINE / 2;

    //------------------------------------------------------------------
    // 输入计数 / 风格锁存
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
                wcnt <= 16'd1;     // 当前拍是第 0 字，下一拍第 1 字
                rcnt <= 16'd0;
            end else if (wcnt == LAST_W) begin
                wcnt <= 16'd0;
                rcnt <= (rcnt == LAST_L) ? 16'd0 : (rcnt + 16'd1);
            end else begin
                wcnt <= wcnt + 16'd1;
            end
        end
    end

    reg [1:0] style_f;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)   style_f <= 2'd0;
        else if (first) style_f <= I_style;
    end
    wire       block8 = (style_f != 2'd0);
    wire [1:0] pal_f  = style_f;

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

    // ★ 2026-10-28 上板回归：常量乘法会被 TD 推成 DSP。
    //   本工程 DSP 已到 92%，像素核不应再占 DSP；480 = 512 - 32，用移位加减。
    wire [10:0] a4 = ({9'd0, lnum[1:0]} << 9) - ({9'd0, lnum[1:0]} << 5)
                   + {3'd0, wnum[8:0]};

    //------------------------------------------------------------------
    // 存储
    //------------------------------------------------------------------
    (* ram_style = "block" *) reg [29:0] hs4   [0:4*WORDS_PER_LINE-1];  /* fehdl force_ram=1, ram_style="bram" */
    (* ram_style = "block" *) reg [35:0] vsum4 [0:WORDS_PER_LINE-1];    /* fehdl force_ram=1, ram_style="bram" */
    (* ram_style = "block" *) reg [23:0] ring4_0 [0:WORDS_PER_LINE-1];  /* fehdl force_ram=1, ram_style="bram" */
    (* ram_style = "block" *) reg [23:0] ring4_1 [0:WORDS_PER_LINE-1];  /* fehdl force_ram=1, ram_style="bram" */
    (* ram_style = "block" *) reg [23:0] ring4_2 [0:WORDS_PER_LINE-1];  /* fehdl force_ram=1, ram_style="bram" */
    (* ram_style = "block" *) reg [23:0] ring4_3 [0:WORDS_PER_LINE-1];  /* fehdl force_ram=1, ram_style="bram" */
    (* ram_style = "block" *) reg [23:0] group_b0 [0:HALF_W-1];         /* fehdl force_ram=1, ram_style="bram" */
    (* ram_style = "block" *) reg [23:0] group_b1 [0:HALF_W-1];         /* fehdl force_ram=1, ram_style="bram" */

    // 每个 ring 行的最后一字（w=0 时 gprev0 用；避免第二读口）
    reg [23:0] ring_last0, ring_last1, ring_last2, ring_last3;

    //------------------------------------------------------------------
    // s0 -> s1：读地址在 s0 发，数据在 s1 用；计数/数据打一拍
    //------------------------------------------------------------------
    reg [29:0] old4_q;
    reg [35:0] cur4_q;
    always @(posedge I_clk) begin
        old4_q <= hs4[a4];
        cur4_q <= vsum4[wnum[8:0]];
    end

    reg [9:0]  hr_s1, hg_s1, hb_s1;
    reg [15:0] lnum_s1, wnum_s1;
    reg [10:0] a4_s1;
    reg        vld_s1;

    always @(posedge I_clk) begin
        vld_s1 <= I_tvalid;
        if (I_tvalid) begin
            hr_s1 <= hr; hg_s1 <= hg; hb_s1 <= hb;
            lnum_s1 <= lnum; wnum_s1 <= wnum;
            a4_s1 <= a4;
        end
    end

    // s1 求和
    wire [11:0] vr4_1 = (lnum_s1 == 16'd0) ? {2'b0, hr_s1} :
                        (lnum_s1 <  16'd4) ? (cur4_q[35:24] + {2'b0, hr_s1}) :
                        (cur4_q[35:24] + {2'b0, hr_s1} - {2'b0, old4_q[29:20]});
    wire [11:0] vg4_1 = (lnum_s1 == 16'd0) ? {2'b0, hg_s1} :
                        (lnum_s1 <  16'd4) ? (cur4_q[23:12] + {2'b0, hg_s1}) :
                        (cur4_q[23:12] + {2'b0, hg_s1} - {2'b0, old4_q[19:10]});
    wire [11:0] vb4_1 = (lnum_s1 == 16'd0) ? {2'b0, hb_s1} :
                        (lnum_s1 <  16'd4) ? (cur4_q[11:0]  + {2'b0, hb_s1}) :
                        (cur4_q[11:0]  + {2'b0, hb_s1} - {2'b0, old4_q[9:0]});

    wire [12:0] vr4p_1 = {1'b0, vr4_1} + 13'd8;
    wire [12:0] vg4p_1 = {1'b0, vg4_1} + 13'd8;
    wire [12:0] vb4p_1 = {1'b0, vb4_1} + 13'd8;
    wire [23:0] blk4_1 = {vr4p_1[11:4], vg4p_1[11:4], vb4p_1[11:4]};

    always @(posedge I_clk) if (I_tvalid) begin
        hs4[a4_s1] <= {hr_s1, hg_s1, hb_s1};
        vsum4[wnum_s1[8:0]] <= {vr4_1, vg4_1, vb4_1};
    end

    wire [1:0] tile_w1 = lnum_s1[3:2];
    wire ring_we = I_tvalid && (lnum_s1[1:0] == 2'd3);

    // ★ 2026-10-28：每个数组一个独立的简单写使能。
    //   原来的 case 写法会让 TD 认为 4 个数组共用一套条件写口，
    //   BRAM 推断失败（SYN-5105），退回 DRAM，白白多花 960 个 LUT。
    always @(posedge I_clk) begin
        if (ring_we && (tile_w1 == 2'd0)) ring4_0[wnum_s1[8:0]] <= blk4_1;
        if (ring_we && (tile_w1 == 2'd1)) ring4_1[wnum_s1[8:0]] <= blk4_1;
        if (ring_we && (tile_w1 == 2'd2)) ring4_2[wnum_s1[8:0]] <= blk4_1;
        if (ring_we && (tile_w1 == 2'd3)) ring4_3[wnum_s1[8:0]] <= blk4_1;
        if (ring_we && (wnum_s1 == LAST_W)) begin
            if (tile_w1 == 2'd0)      ring_last0 <= blk4_1;
            else if (tile_w1 == 2'd1) ring_last1 <= blk4_1;
            else if (tile_w1 == 2'd2) ring_last2 <= blk4_1;
            else                      ring_last3 <= blk4_1;
        end
    end

    // s1 发 ring 读：t4_cur/up/dn/prev 由 s1 行号算
    wire [1:0] t2_cur  = lnum_s1[3:2] + 2'd2;
    wire [1:0] t2_up   = lnum_s1[3:2] + 2'd1;
    wire [1:0] t2_dn   = lnum_s1[3:2] + 2'd3;
    wire [1:0] t2_prev = lnum_s1[3:2] + 2'd3;

    // ★ 2026-10-28：4 个数组无条件各读一次，再在寄存器后做 4:1 选择。
    //   原来的“case 选数组再读”让 TD 把每个读口推成独立 DRAM（960 个 LUT）。
    //   这里读地址对所有数组相同，每个数组只有 1 读 + 1 写，BRAM 可推断。
    reg [23:0] ring_rd0, ring_rd1, ring_rd2, ring_rd3;
    always @(posedge I_clk) begin
        ring_rd0 <= ring4_0[wnum_s1[8:0]];
        ring_rd1 <= ring4_1[wnum_s1[8:0]];
        ring_rd2 <= ring4_2[wnum_s1[8:0]];
        ring_rd3 <= ring4_3[wnum_s1[8:0]];
    end

    function [23:0] sel_ring;
        input [1:0]  s;
        input [23:0] a, b, c, d;
        begin
            case (s)
                2'd0: sel_ring = a;
                2'd1: sel_ring = b;
                2'd2: sel_ring = c;
                default: sel_ring = d;
            endcase
        end
    endfunction

    wire [23:0] c4_cur_q = sel_ring(t2_cur,  ring_rd0, ring_rd1, ring_rd2, ring_rd3);
    wire [23:0] c4_up_q  = sel_ring(t2_up,   ring_rd0, ring_rd1, ring_rd2, ring_rd3);
    wire [23:0] c4_dn_q  = sel_ring(t2_dn,   ring_rd0, ring_rd1, ring_rd2, ring_rd3);
    wire [23:0] gprev1_q = sel_ring(t2_prev, ring_rd0, ring_rd1, ring_rd2, ring_rd3);

    // s1 -> s2 计数 / blk4
    reg [15:0] lnum_s2, wnum_s2;
    reg        vld_s2;
    reg [23:0] blk4_s2, blk4_prev_s2;

    always @(posedge I_clk) begin
        vld_s2 <= vld_s1;
        if (I_tvalid) begin
            lnum_s2 <= lnum_s1; wnum_s2 <= wnum_s1;
            blk4_s2 <= blk4_1;
            blk4_prev_s2 <= blk4_s2;
        end
    end

    //------------------------------------------------------------------
    // s2：ring 数据 → gcol / 写 group；发 c4_raw 读
    //------------------------------------------------------------------
    reg [23:0] gprev1_prev_q;
    always @(posedge I_clk) if (I_tvalid) gprev1_prev_q <= gprev1_q;

    wire [23:0] gprev0_s2 = (wnum_s2 == 16'd0) ?
        ((t2_prev == 2'd0) ? ring_last0 :
         (t2_prev == 2'd1) ? ring_last1 :
         (t2_prev == 2'd2) ? ring_last2 : ring_last3) : gprev1_prev_q;

    wire [9:0] gc_r_s2 = {2'b0, blk4_prev_s2[23:16]} + {2'b0, blk4_s2[23:16]} +
                         {2'b0, gprev0_s2[23:16]}     + {2'b0, gprev1_q[23:16]} + 10'd2;
    wire [9:0] gc_g_s2 = {2'b0, blk4_prev_s2[15:8]}  + {2'b0, blk4_s2[15:8]} +
                         {2'b0, gprev0_s2[15:8]}      + {2'b0, gprev1_q[15:8]} + 10'd2;
    wire [9:0] gc_b_s2 = {2'b0, blk4_prev_s2[7:0]}   + {2'b0, blk4_s2[7:0]} +
                         {2'b0, gprev0_s2[7:0]}       + {2'b0, gprev1_q[7:0]} + 10'd2;
    wire [23:0] gcol_s2 = {gc_r_s2[9:2], gc_g_s2[9:2], gc_b_s2[9:2]};

    wire group_active = block8 && I_tvalid && lnum_s2[2] && wnum_s2[0];
    wire [7:0] gj_s2 = wnum_s2[8:1];
    wire       gslot_w2 = lnum_s2[3];

    always @(posedge I_clk) if (group_active) begin
        if (gslot_w2) group_b1[gj_s2] <= gcol_s2;
        else          group_b0[gj_s2] <= gcol_s2;
    end

    // c4_raw 就是“下一个字的 c4_cur”。
    //   s2 的 wraw2 = wnum_s2+1，而同一拍 s1 的 wnum_s1 也等于 wnum_s2+1
    //   （同一行内），数组选择也相同；行尾那一拍 c4_raw 在 s3 被 clamp 到
    //   c4_cur_h，根本不使用。所以直接复用 c4_cur_q，省掉第二个读口。
    wire [23:0] c4_raw_q = c4_cur_q;

    // s2 -> s3 计数 / c4 数据 / c4_left
    reg [15:0] lnum_s3, wnum_s3;
    reg        vld_s3;
    reg [23:0] c4_cur_h, c4_up_h, c4_dn_h, c4_left_q;

    always @(posedge I_clk) begin
        vld_s3 <= vld_s2;
        if (I_tvalid) begin
            lnum_s3 <= lnum_s2; wnum_s3 <= wnum_s2;
            c4_cur_h <= c4_cur_q; c4_up_h <= c4_up_q; c4_dn_h <= c4_dn_q;
            // 非阻塞赋值右侧取旧值：c4_cur_h 在这里还是上一字的 cur
            c4_left_q <= c4_cur_h;
        end
    end

    //------------------------------------------------------------------
    // s3：4px 描边；发 g_cur 读
    //------------------------------------------------------------------
    wire [15:0] drcnt_s3 = (lnum_s3 >= 16'd8) ? (lnum_s3 - 16'd8)
                                               : (lnum_s3 + LINES_PER_FRAME - 16'd8);
    wire [15:0] by4_s3 = drcnt_s3 >> 2;

    wire [23:0] c4_l_s3 = (wnum_s3 == 16'd0) ? c4_cur_h : c4_left_q;
    wire [23:0] c4_r_s3 = (wnum_s3 == LAST_W) ? c4_cur_h : c4_raw_q;
    wire [23:0] c4_u_s3 = (by4_s3 == 16'd0) ? c4_cur_h : c4_up_h;
    wire [23:0] c4_d_s3 = (by4_s3 == (LINES_PER_FRAME/4 - 1)) ? c4_cur_h : c4_dn_h;

    // ★ 2026-10-28：77/150/29 用移位加，别让 TD 推 DSP。
    //   77=64+8+4+1，150=128+16+4+2，29=16+8+4+1；结果与乘法完全一致。
    function [7:0] lum8;
        input [23:0] c;
        reg [15:0] r, g, b;
        begin
            r = ({8'd0, c[23:16]} << 6) + ({8'd0, c[23:16]} << 3)
              + ({8'd0, c[23:16]} << 2) + {8'd0, c[23:16]};
            g = ({8'd0, c[15:8]}  << 7) + ({8'd0, c[15:8]}  << 4)
              + ({8'd0, c[15:8]}  << 2) + ({8'd0, c[15:8]}  << 1);
            b = ({8'd0, c[7:0]}   << 4) + ({8'd0, c[7:0]}   << 3)
              + ({8'd0, c[7:0]}   << 2) + {8'd0, c[7:0]};
            lum8 = (r + g + b) >> 8;
        end
    endfunction

    // ★ 2026-10-28：x*141>>8 用移位加（141=128+8+4+1），与乘法逐位一致，0 DSP。
    function [7:0] dim141;
        input [7:0] x;
        reg [15:0] t;
        begin
            t = ({8'd0, x} << 7) + ({8'd0, x} << 3)
              + ({8'd0, x} << 2) + {8'd0, x};
            dim141 = t[15:8];
        end
    endfunction

    function edge_hi;
        input [7:0] a, b;
        input [7:0] thr;
        begin
            edge_hi = ((a > b) && ((a - b) > thr));
        end
    endfunction

    wire m4_s3 = edge_hi(lum8(c4_cur_h), lum8(c4_l_s3), 8'd50) |
                 edge_hi(lum8(c4_cur_h), lum8(c4_r_s3), 8'd50) |
                 edge_hi(lum8(c4_cur_h), lum8(c4_u_s3), 8'd50) |
                 edge_hi(lum8(c4_cur_h), lum8(c4_d_s3), 8'd50);

    // ⚠️ 必须用 8bit 中间 wire：直接写 {a*141>>8, b*141>>8, c*141>>8}
    // 会让每个表达式变成 32bit，拼接成 96bit 再截 24bit，R/G 全被丢掉。
    wire [7:0] o4r_s3 = dim141(c4_cur_h[23:16]);
    wire [7:0] o4g_s3 = dim141(c4_cur_h[15:8] );
    wire [7:0] o4b_s3 = dim141(c4_cur_h[7:0]  );
    wire [23:0] c4_out_s3 = m4_s3 ? {o4r_s3, o4g_s3, o4b_s3} : c4_cur_h;

    // g_cur 读
    wire       gslot_r3 = drcnt_s3[3];
    wire [7:0] gj_s3 = wnum_s3[8:1];
    reg [23:0] g_cur_q;
    always @(posedge I_clk) begin
        if (gslot_r3) g_cur_q <= group_b1[gj_s3];
        else          g_cur_q <= group_b0[gj_s3];
    end

    // s3 -> s4 计数
    reg [15:0] lnum_s4, wnum_s4;
    reg        vld_s4;
    always @(posedge I_clk) begin
        vld_s4 <= vld_s3;
        if (I_tvalid) begin lnum_s4 <= lnum_s3; wnum_s4 <= wnum_s3; end
    end

    // c4_out 从 s3 到 s5 要经过两级流水（s4、s5），不能只用一个寄存器：
    // 单寄存器在 s5 时已经被更晚的字覆盖，会和 dither 的 wnum_s5 错开。
    reg [23:0] c4_out_s4, c4_out_s5;
    always @(posedge I_clk) begin
        if (I_tvalid) c4_out_s4 <= c4_out_s3;
        if (I_tvalid) c4_out_s5 <= c4_out_s4;
    end

    //------------------------------------------------------------------
    // s4：g_cur 数据；发 g_raw 读
    //------------------------------------------------------------------
    wire [15:0] drcnt_s4 = (lnum_s4 >= 16'd8) ? (lnum_s4 - 16'd8)
                                               : (lnum_s4 + LINES_PER_FRAME - 16'd8);
    wire       gslot_r4 = drcnt_s4[3];
    // s4 当前字比 s5 输出字晚 1 个字：g_raw 要取输出字 gj+1 的组色
    // wnum_s4 是输出字+1；对偶数输出字，直接 >>1 是当前组，需要 +1
    wire [7:0] gj_base4 = (wnum_s4 == 16'd0) ? 8'd0 : ((wnum_s4 - 16'd1) >> 1);
    wire [7:0] gj_r4 = (gj_base4 == HALF_W-1) ? gj_base4 : (gj_base4 + 8'd1);

    reg [23:0] g_raw_q;
    reg [23:0] g_cur_h, g_left_q;
    always @(posedge I_clk) begin
        if (gslot_r4) g_raw_q <= group_b1[gj_r4];
        else          g_raw_q <= group_b0[gj_r4];
        if (I_tvalid) begin
            g_cur_h <= g_cur_q;
        end
    end

    // s4 -> s5 计数
    reg [15:0] lnum_s5, wnum_s5;
    reg        vld_s5;
    always @(posedge I_clk) begin
        vld_s5 <= vld_s4;
        if (I_tvalid) begin lnum_s5 <= lnum_s4; wnum_s5 <= wnum_s4; end
    end

    //------------------------------------------------------------------
    // s5：8px 描边 / 选路 / 抖动 / map 地址
    //------------------------------------------------------------------
    wire [15:0] drcnt_s5 = (lnum_s5 >= 16'd8) ? (lnum_s5 - 16'd8)
                                               : (lnum_s5 + LINES_PER_FRAME - 16'd8);
    wire [15:0] by8_s5 = drcnt_s5 >> 3;
    wire [15:0] by4_s5 = drcnt_s5 >> 2;

    wire [7:0] gj_s5 = wnum_s5[8:1];
    // 源实现是在奇数字上寄存 g_cur，当前字用旧值；这里同样在 s5 寄存
    always @(posedge I_clk) if (I_tvalid && wnum_s5[0]) g_left_q <= g_cur_h;
    wire [23:0] g_l_s5 = (gj_s5 == 8'd0) ? g_cur_h : g_left_q;
    wire [23:0] g_r_s5 = (gj_s5 == HALF_W-1) ? g_cur_h : g_raw_q;

    wire m8_s5 = edge_hi(lum8(g_cur_h), lum8(g_l_s5), 8'd60) |
                 edge_hi(lum8(g_cur_h), lum8(g_r_s5), 8'd60);

    wire [7:0] o8r_s5 = dim141(g_cur_h[23:16]);
    wire [7:0] o8g_s5 = dim141(g_cur_h[15:8] );
    wire [7:0] o8b_s5 = dim141(g_cur_h[7:0]  );
    wire [23:0] c8_out_s5 = m8_s5 ? {o8r_s5, o8g_s5, o8b_s5} : g_cur_h;

    wire [23:0] csel = block8 ? c8_out_s5 : c4_out_s5;
    wire [15:0] bx   = block8 ? {1'b0, wnum_s5[15:1]} : wnum_s5;
    wire [15:0] by   = block8 ? by8_s5 : by4_s5;

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

    wire signed [7:0] doff = (pal_f == 2'd2) ? dith12f(by[1:0], bx[1:0])
                                             : dith8f (by[1:0], bx[1:0]);

    wire signed [9:0] srr = $signed({2'b0, csel[23:16]}) + $signed(doff);
    wire signed [9:0] sgg = $signed({2'b0, csel[15:8] }) + $signed(doff);
    wire signed [9:0] sbb = $signed({2'b0, csel[7:0]  }) + $signed(doff);

    wire [7:0] dr = (srr < 0) ? 8'd0 : (srr > 255) ? 8'd255 : srr[7:0];
    wire [7:0] dg = (sgg < 0) ? 8'd0 : (sgg > 255) ? 8'd255 : sgg[7:0];
    wire [7:0] db = (sbb < 0) ? 8'd0 : (sbb > 255) ? 8'd255 : sbb[7:0];

    wire [11:0] map_addr = {dr[7:4], dg[7:4], db[7:4]};
    reg  [11:0] map_addr_q;
    always @(posedge I_clk) if (I_tvalid) map_addr_q <= map_addr;

    // s5 -> s6 计数
    reg [15:0] lnum_s6, wnum_s6;
    reg        vld_s6;
    always @(posedge I_clk) begin
        vld_s6 <= vld_s5;
        if (I_tvalid) begin lnum_s6 <= lnum_s5; wnum_s6 <= wnum_s5; end
    end

    //------------------------------------------------------------------
    // s6：map 数据 -> palette 地址
    //------------------------------------------------------------------
    wire [7:0] map_q;
    pixel_rom #(
        .DATA_WIDTH (8),
        .ADDR_WIDTH (12),
        .DEPTH      (4096),
        .INIT_FILE  ("pixel_map.dath")
    ) u_map (
        .I_clk (I_clk), .I_addr(map_addr_q), .O_data(map_q)
    );

    wire [9:0] pal_addr = {pal_f, map_q};
    reg  [9:0] pal_addr_q;
    always @(posedge I_clk) if (I_tvalid) pal_addr_q <= pal_addr;

    // s6 -> s7 计数
    reg [15:0] lnum_s7, wnum_s7;
    reg        vld_s7;
    always @(posedge I_clk) begin
        vld_s7 <= vld_s6;
        if (I_tvalid) begin lnum_s7 <= lnum_s6; wnum_s7 <= wnum_s6; end
    end

    //------------------------------------------------------------------
    // s7：palette 数据 -> 输出
    //------------------------------------------------------------------
    wire [23:0] pal_q;
    pixel_rom #(
        .DATA_WIDTH (24),
        .ADDR_WIDTH (10),
        .DEPTH      (768),
        .INIT_FILE  ("pixel_palette.dath")
    ) u_pal (
        .I_clk (I_clk), .I_addr(pal_addr_q), .O_data(pal_q)
    );

    // pal_q 在 s7 边沿比 s7 计数**晚 2 个字**（map/palette 两级都在地址寄存器
    // 之后出数）。所以把 valid/tuser/tlast 再延迟 2 个有效拍，和 pal_q 对齐。
    // 不能只改 lnum_s7/wnum_s7 的比较值：tlast 会跨行，必须延迟信号本身。
    reg tuser_s7, tlast_s7, vld_s7_r;
    always @(posedge I_clk) begin
        tuser_s7 <= I_tvalid && (wnum_s7 == 16'd0) && (lnum_s7 == 16'd8);
        tlast_s7 <= I_tvalid && (wnum_s7 == LAST_W);
        vld_s7_r <= I_tvalid;
    end

    reg tuser_d1, tuser_d2, tuser_d3, tlast_d1, tlast_d2, tlast_d3, vld_d1, vld_d2, vld_d3;
    always @(posedge I_clk) if (I_tvalid) begin
        tuser_d1 <= tuser_s7; tuser_d2 <= tuser_d1; tuser_d3 <= tuser_d2;
        tlast_d1 <= tlast_s7; tlast_d2 <= tlast_d1; tlast_d3 <= tlast_d2;
        vld_d1   <= vld_s7_r; vld_d2   <= vld_d1; vld_d3   <= vld_d2;
    end

    // 第一帧帧首之前不输出（和 pixel_style.v 的 lnum>=8 门控一致），
    // 否则 s7 流水会在 tuser 前多吐 8 个预热字。
    reg frame_active;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)          frame_active <= 1'b0;
        else if (tuser_d1)     frame_active <= 1'b1;
    end
    wire frame_active_now = frame_active | tuser_d1;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_tdata  <= {RGB_W{1'b0}};
            O_tvalid <= 1'b0;
            O_tlast  <= 1'b0;
            O_tuser  <= 1'b0;
        end else begin
            O_tdata  <= {pal_q, pal_q, pal_q, pal_q};
            O_tvalid <= I_tvalid && frame_active_now;
            O_tlast  <= I_tvalid && tlast_d1;
            O_tuser  <= I_tvalid && tuser_d1;
        end
    end

    wire unused_ok = I_tlast;

endmodule
