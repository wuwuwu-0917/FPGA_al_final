`timescale 1ns / 1ps
//=============================================================================
//  delay_meter.v --- end-to-end frame latency monitor
//
//  All logic runs in the HDMI 150 MHz domain.  Two slow/asynchronous inputs
//  are synchronized in:
//    I_cam_tuser : single-cycle pulse in the 74.24 MHz camera domain
//    I_wbuf      : DDR buffer number from the 133 MHz AXI domain
//
//  One event is recorded per written frame, tagged with the buffer number the
//  frame goes into.  At VS the displayed buffer's timestamp is looked up and
//  its age is converted to 0.1 ms units.  Repeated displays of the same
//  buffer hold the previous reading; a stale buffer invalidates the display.
//=============================================================================
module delay_meter #(
    parameter [31:0] INVALID_TICKS = 32'd15_000_000, // 100 ms at 150 MHz
    parameter [31:0] DIVISOR       = 32'd15_000      // ticks per 0.1 ms
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_cam_tuser,
    input  wire [7:0]  I_wbuf,
    input  wire        I_vs,
    input  wire [7:0]  I_rbuf,
    output reg  [11:0] O_delay_ms10,
    output reg         O_valid
);

    //-------------------------------------------------------------------------
    // tuser: two-flop sync plus rising-edge detect.
    //-------------------------------------------------------------------------
    reg tu_s1, tu_s2, tu_s3;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            tu_s1 <= 1'b0; tu_s2 <= 1'b0; tu_s3 <= 1'b0;
        end else begin
            tu_s1 <= I_cam_tuser;
            tu_s2 <= tu_s1;
            tu_s3 <= tu_s2;
        end
    end
    wire tu_pulse = tu_s2 & ~tu_s3;

    //-------------------------------------------------------------------------
    // wbuf: two-flop sync plus a one-cycle stability filter.  The buffer
    // number can change between non-adjacent values (1 -> 2 flips two bits),
    // so a transient synchronized value is ignored until it holds.
    //-------------------------------------------------------------------------
    reg [7:0] wb_s1, wb_s2, wb_stable, wb_stable_d;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            wb_s1 <= 8'd0; wb_s2 <= 8'd0;
            wb_stable <= 8'd0; wb_stable_d <= 8'd0;
        end else begin
            wb_s1       <= I_wbuf;
            wb_s2       <= wb_s1;
            wb_stable   <= wb_s2;
            wb_stable_d <= wb_stable;
        end
    end

    //-------------------------------------------------------------------------
    // Free-running timestamp counter.
    //-------------------------------------------------------------------------
    reg [31:0] ts_counter;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) ts_counter <= 32'd0;
        else          ts_counter <= ts_counter + 32'd1;
    end

    //-------------------------------------------------------------------------
    // Frame event: tuser arms the detector, the next stable wbuf change fires.
    //-------------------------------------------------------------------------
    reg armed;
    reg [7:0]  wb_prev;
    reg [31:0] ts_in  [0:2];
    reg [7:0]  cnt_in [0:2];
    reg [7:0]  evt_cnt;
    integer k;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            armed   <= 1'b0;
            wb_prev <= 8'd0;
            evt_cnt <= 8'd0;
            for (k = 0; k < 3; k = k + 1) begin
                ts_in[k]  <= 32'd0;
                cnt_in[k] <= 8'd0;
            end
        end else begin
            if (tu_pulse)
                armed <= 1'b1;
            if (armed && (wb_stable == wb_stable_d) && (wb_stable != wb_prev)) begin
                armed                <= 1'b0;
                wb_prev              <= wb_stable;
                ts_in[wb_stable[1:0]]  <= ts_counter;
                cnt_in[wb_stable[1:0]] <= evt_cnt + 8'd1;
                evt_cnt              <= evt_cnt + 8'd1;
            end
        end
    end

    //-------------------------------------------------------------------------
    // Display side: at VS, use the timestamp of the buffer being displayed.
    //-------------------------------------------------------------------------
    reg vs_d;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) vs_d <= 1'b0;
        else          vs_d <= I_vs;
    end
    wire vs_rise = I_vs & ~vs_d;

    reg [31:0] ts_out;
    reg [31:0] delta;
    reg [31:0] div_dividend;
    reg        div_start;
    reg        div_done;
    reg [31:0] div_quotient;

    wire [7:0]  evt_age = evt_cnt - cnt_in[I_rbuf[1:0]];
    wire        evt_ok  = (evt_age != 8'd0) && (evt_age <= 8'd3);
    wire [31:0] age     = ts_counter - ts_in[I_rbuf[1:0]];

    // ★ 2026-10-31 读数改为「每帧都采 + 一阶低通」：
    //   原来只在**显示的 buffer 号变化时**才重新算（I_rbuf != rbuf_last），
    //   画面重复显示同一个 buffer 的那几帧数字是冻住的，换 buffer 才跳一下，
    //   看起来就是一格一格地爬。现在每个 VS 都取一次样，再用 α=1/8 的
    //   一阶低通压掉帧间 ±1 帧的抖动，读数稳定在 30 ms 附近。
    reg [31:0] ema;
    reg [5:0]  stale_cnt;
    wire [31:0] ema_next = ema - (ema >> 3) + (age >> 3);

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            ts_out       <= 32'd0;
            delta        <= 32'd0;
            ema          <= 32'd0;
            stale_cnt    <= 6'd0;
            div_dividend <= 32'd0;
            div_start    <= 1'b0;
            O_delay_ms10 <= 12'd0;
            O_valid      <= 1'b0;
        end else begin
            div_start <= 1'b0;
            if (vs_rise) begin
                ts_out <= ts_counter;
                if (!evt_ok || (age > INVALID_TICKS)) begin
                    // 显示的 buffer 已经不新鲜：连续 64 帧都这样才熄灭，
                    // 偶发一帧的抖动不至于让读数闪成 "--"。
                    if (stale_cnt != 6'h3f) stale_cnt <= stale_cnt + 6'd1;
                    else                    O_valid   <= 1'b0;
                end else begin
                    stale_cnt    <= 6'd0;
                    delta        <= age;
                    ema          <= (ema == 32'd0) ? age : ema_next;
                    div_dividend <= (ema == 32'd0) ? age : ema_next;
                    div_start    <= 1'b1;
                end
            end
            if (div_done) begin
                O_delay_ms10 <= div_quotient[11:0];
                O_valid      <= 1'b1;
            end
        end
    end

    //-------------------------------------------------------------------------
    // Constant-divisor divide: one pass per displayed frame, 32 cycles.
    // quotient = dividend / DIVISOR  (0.1 ms units)
    //-------------------------------------------------------------------------
    reg [4:0]  div_bit;
    reg [31:0] div_sr;
    reg [31:0] div_rem;
    reg [31:0] div_q;
    reg        div_busy;
    reg [31:0] rem_next;
    reg [31:0] q_next;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            div_bit      <= 5'd0;
            div_sr       <= 32'd0;
            div_rem      <= 32'd0;
            div_q        <= 32'd0;
            div_busy     <= 1'b0;
            div_done     <= 1'b0;
            div_quotient <= 32'd0;
        end else begin
            div_done <= 1'b0;
            if (div_start) begin
                div_sr   <= div_dividend;
                div_rem  <= 32'd0;
                div_q    <= 32'd0;
                div_bit  <= 5'd32;
                div_busy <= 1'b1;
            end else if (div_busy) begin
                rem_next = {div_rem[30:0], div_sr[31]};
                q_next   = {div_q[30:0], 1'b0};
                if (rem_next >= DIVISOR) begin
                    rem_next = rem_next - DIVISOR;
                    q_next[0] = 1'b1;
                end
                div_rem <= rem_next;
                div_q   <= q_next;
                div_sr  <= {div_sr[30:0], 1'b0};
                div_bit <= div_bit - 5'd1;
                if (div_bit == 5'd1) begin
                    div_busy     <= 1'b0;
                    div_done     <= 1'b1;
                    div_quotient <= q_next;
                end
            end
        end
    end
endmodule
