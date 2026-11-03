// Exact 91/120 nearest-neighbour fit for a 1920x1080 four-pixel ISP stream.
// The resulting 1456x819 RGB frame is stored in DDR at native capture timing.
module ui_fit_scaler #(
    parameter integer SRC_WIDTH = 1920,
    parameter integer SRC_HEIGHT = 1080,
    parameter integer DST_WIDTH = 1456,
    parameter integer DST_HEIGHT = 819,
    parameter integer SCALE_NUM = 91,
    parameter integer SCALE_DEN = 120
)(
    input wire I_clk,
    input wire I_rst_n,
    input wire I_tuser,
    input wire I_tvalid,
    input wire [127:0] I_tdata,
    input wire I_tlast,
    output reg O_tuser,
    output reg O_tvalid,
    output reg [127:0] O_tdata,
    output reg O_tlast
);
    reg [11:0] src_x;
    reg [10:0] src_y;
    reg [10:0] dst_y;
    reg [11:0] next_src_x;
    reg [10:0] next_src_y;
    reg [6:0] rem_x, rem_y;
    reg [10:0] dst_x;
    reg [2:0] pack_count;
    reg [127:0] pack_data;

    integer lane;
    integer x_base_work, y_work, next_x_work, next_y_work;
    integer rem_x_work, rem_y_work, dst_x_work, dst_y_work, pack_count_work;
    integer advance_work;
    reg [127:0] pack_data_work;
    reg row_selected_work;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            src_x <= 0; src_y <= 0; dst_y <= 0; next_src_x <= 0; next_src_y <= 0;
            rem_x <= 0; rem_y <= 0; dst_x <= 0; pack_count <= 0; pack_data <= 0;
            O_tuser <= 0; O_tvalid <= 0; O_tdata <= 0; O_tlast <= 0;
        end else begin
            O_tuser <= 0;
            O_tvalid <= 0;
            O_tlast <= 0;
            if (I_tvalid) begin
                x_base_work = I_tuser ? 0 : src_x;
                y_work = I_tuser ? 0 : src_y;
                dst_y_work = I_tuser ? 0 : dst_y;
                next_x_work = I_tuser ? 0 : next_src_x;
                next_y_work = I_tuser ? 0 : next_src_y;
                rem_x_work = I_tuser ? 0 : rem_x;
                rem_y_work = I_tuser ? 0 : rem_y;
                dst_x_work = I_tuser ? 0 : dst_x;
                pack_count_work = I_tuser ? 0 : pack_count;
                pack_data_work = I_tuser ? 128'd0 : pack_data;
                row_selected_work = (y_work == next_y_work) &&
                                    (y_work < SRC_HEIGHT) && (dst_y_work < DST_HEIGHT);

                for (lane = 0; lane < 4; lane = lane + 1) begin
                    if (row_selected_work && (x_base_work + lane < SRC_WIDTH) &&
                        (x_base_work + lane == next_x_work) && (dst_x_work < DST_WIDTH)) begin
                        // Fixed four-lane packing avoids a variable part-select
                        // on the camera clock path.
                        case (pack_count_work)
                            0: pack_data_work[31:0]   = I_tdata[lane*32 +: 32];
                            1: pack_data_work[63:32]  = I_tdata[lane*32 +: 32];
                            2: pack_data_work[95:64]  = I_tdata[lane*32 +: 32];
                            default: pack_data_work[127:96] = I_tdata[lane*32 +: 32];
                        endcase
                        pack_count_work = pack_count_work + 1;
                        dst_x_work = dst_x_work + 1;
                        advance_work = rem_x_work + SCALE_DEN;
                        // For the fixed 91/120 fit, (rem + 120) is in
                        // [120,210], so the quotient is only 1 or 2.
                        if (advance_work >= (SCALE_NUM * 2)) begin
                            next_x_work = next_x_work + 2;
                            rem_x_work = advance_work - (SCALE_NUM * 2);
                        end else begin
                            next_x_work = next_x_work + 1;
                            rem_x_work = advance_work - SCALE_NUM;
                        end
                        if (pack_count_work == 4) begin
                            O_tdata <= pack_data_work;
                            O_tvalid <= 1;
                            O_tlast <= (dst_x_work == DST_WIDTH);
                            O_tuser <= (dst_x_work == 4) && (y_work == 0);
                            pack_count_work = 0;
                        end
                    end
                end

                dst_x <= dst_x_work;
                pack_count <= pack_count_work;
                pack_data <= pack_data_work;
                next_src_x <= next_x_work;
                rem_x <= rem_x_work;
                dst_y <= dst_y_work;
                if (I_tlast) begin
                    src_x <= 0;
                    dst_x <= 0;
                    pack_count <= 0;
                    pack_data <= 0;
                    next_src_x <= 0;
                    rem_x <= 0;
                    src_y <= y_work + 1;
                    if (row_selected_work && (y_work < SRC_HEIGHT)) begin
                        dst_y_work = dst_y_work + 1;
                        advance_work = rem_y_work + SCALE_DEN;
                        if (advance_work >= (SCALE_NUM * 2)) begin
                            next_y_work = next_y_work + 2;
                            rem_y_work = advance_work - (SCALE_NUM * 2);
                        end else begin
                            next_y_work = next_y_work + 1;
                            rem_y_work = advance_work - SCALE_NUM;
                        end
                    end
                    next_src_y <= next_y_work;
                    rem_y <= rem_y_work;
                    dst_y <= dst_y_work;
                end else begin
                    src_x <= x_base_work + 4;
                    src_y <= y_work;
                    next_src_y <= next_y_work;
                    rem_y <= rem_y_work;
                end
            end
        end
    end
endmodule

// Match the UI's active-raster counters and read the fitted frame only in its
// exact 16:9 viewport: x=[0,1456), y=[78,897).
module ui_fit_read_gate (
    input wire I_clk,
    input wire I_rst_n,
    input wire I_vs,
    input wire I_de,
    output wire O_ready
);
    reg [11:0] x_count, y_count;
    reg de_previous;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            x_count <= 0;
            y_count <= 0;
            de_previous <= 0;
        end else begin
            de_previous <= I_de;
            if (I_vs) begin
                x_count <= 0;
                y_count <= 0;
            end else if (I_de) begin
                x_count <= x_count + 1'b1;
            end else if (de_previous) begin
                x_count <= 0;
                y_count <= y_count + 1'b1;
            end
        end
    end

    assign O_ready = I_de && x_count < 1456 && y_count >= 78 && y_count < 897;
endmodule
