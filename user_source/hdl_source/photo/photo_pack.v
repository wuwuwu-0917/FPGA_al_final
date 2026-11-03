// 把 4px/128bit 的 ISP 字流打包成 BMP 16bit 字流。
// 源字 4 个 lane：lane L = {8'h00, R, G, B}（B 在最低字节）。
// BMP 每像素 3 字节 B,G,R，行宽 1920*3=5760（4 的倍数，无需行填充）。
module photo_pack #(
    parameter integer N_SRC     = 518400,      // 源 128bit 字数
    parameter integer PAD_WORDS = 229          // 末块补零字数
)(
    input  wire         I_clk,
    input  wire         I_rst_n,
    input  wire         I_start,          // 单拍启动（契约见下）
    input  wire         I_src_valid,
    input  wire [127:0] I_src_data,
    output wire         O_src_ready,      // 取走一个源字
    output wire         O_word_valid,
    output wire [15:0]  O_word,
    input  wire         I_word_ready,
    output wire         O_done
);
    // ---- 接口契约（Task 7 集成注意） --------------------------------------
    // I_start : 单拍启动，只能在「上游 FIFO 已 flush、下游空闲」时给；
    //           打包中途拉高会丢弃已拍源字并从头重发 27 字 BMP 头。
    // O_done  : 电平，保持到下一次 I_start。只表示「所有 16bit 字都已被
    //           下游接收（O_word_valid & I_word_ready）」，不表示已写进 SD 卡；
    //           必须等 16bit FIFO 排空、SD 块写完后再报拍照成功。
    // ----------------------------------------------------------------------
    localparam integer HDR_WORDS   = 27;       // 54 字节 BMP 头（固定）
    localparam integer PIX_WORDS   = N_SRC * 6;
    localparam integer TOTAL_WORDS = HDR_WORDS + PIX_WORDS + PAD_WORDS;
    localparam integer IDX_BITS    = $clog2(TOTAL_WORDS + 1);

    reg [IDX_BITS-1:0] word_idx;
    reg [2:0]  pix_sub;
    reg [127:0] src_hold;
    reg        src_hold_v;
    reg        started;              // I_start 之后才出字；复位/未启动时 valid=0

    wire in_hdr  = (word_idx < HDR_WORDS);
    wire in_pix  = (word_idx >= HDR_WORDS) && (word_idx < HDR_WORDS + PIX_WORDS);
    wire in_pad  = (word_idx >= HDR_WORDS + PIX_WORDS);
    wire need_src = in_pix && (pix_sub == 3'd0) && !src_hold_v;
    assign O_src_ready = need_src;
    assign O_done      = started && (word_idx == TOTAL_WORDS);

    reg [15:0] word_mux;
    always @(*) begin
        if (in_hdr) begin
            // 54 字节 BMP 头（按传输顺序每 2 字节拼成一个 16bit 字，高字节先发）
            case (word_idx)
                 0: word_mux = 16'h424D;  // 'B','M'
                 1: word_mux = 16'h36EC;  // bfSize = 6,220,854
                 2: word_mux = 16'h5E00;
                 3: word_mux = 16'h0000;  // bfReserved1
                 4: word_mux = 16'h0000;  // bfReserved2
                 5: word_mux = 16'h3600;  // bfOffBits = 54
                 6: word_mux = 16'h0000;
                 7: word_mux = 16'h2800;  // biSize = 40
                 8: word_mux = 16'h0000;
                 9: word_mux = 16'h8007;  // biWidth = 1920
                10: word_mux = 16'h0000;
                11: word_mux = 16'hC8FB;  // biHeight = -1080
                12: word_mux = 16'hFFFF;
                13: word_mux = 16'h0100;  // biPlanes = 1
                14: word_mux = 16'h1800;  // biBitCount = 24
                15: word_mux = 16'h0000;  // biCompression = 0
                16: word_mux = 16'h0000;
                17: word_mux = 16'h00EC;  // biSizeImage = 6,220,800
                18: word_mux = 16'h5E00;
                19: word_mux = 16'h130B;  // biXPelsPerMeter = 2835
                20: word_mux = 16'h0000;
                21: word_mux = 16'h130B;  // biYPelsPerMeter = 2835
                22: word_mux = 16'h0000;
                23: word_mux = 16'h0000;  // biClrUsed = 0
                24: word_mux = 16'h0000;
                25: word_mux = 16'h0000;  // biClrImportant = 0
                26: word_mux = 16'h0000;
                default: word_mux = 16'h0000;
            endcase
        end else if (in_pix) begin
            case (pix_sub)
                3'd0: word_mux = {src_hold[7:0],   src_hold[15:8]};    // B0,G0
                3'd1: word_mux = {src_hold[23:16], src_hold[39:32]};   // R0,B1
                3'd2: word_mux = {src_hold[47:40], src_hold[55:48]};   // G1,R1
                3'd3: word_mux = {src_hold[71:64], src_hold[79:72]};   // B2,G2
                3'd4: word_mux = {src_hold[87:80], src_hold[103:96]};  // R2,B3
                default: word_mux = {src_hold[111:104], src_hold[119:112]}; // G3,R3
            endcase
        end
        else word_mux = 16'h0000;   // in_pad 补零
    end
    assign O_word = word_mux;

    // 像素阶段必须等源字到手才 valid；头和补零阶段直接 valid。
    // 未启动或已发满 TOTAL_WORDS 时 valid=0，避免复位期间/发完后继续吐字。
    assign O_word_valid = started && (word_idx < TOTAL_WORDS) &&
                          (in_hdr | (in_pix & src_hold_v) | in_pad);

    wire word_accept = O_word_valid & I_word_ready;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            started <= 1'b0;
            word_idx <= 0; pix_sub <= 0; src_hold_v <= 1'b0; src_hold <= 128'd0;
        end else if (I_start) begin
            started <= 1'b1;
            word_idx <= 0; pix_sub <= 0; src_hold_v <= 1'b0; src_hold <= 128'd0;
        end else begin
            if (need_src && I_src_valid) begin
                src_hold   <= I_src_data;
                src_hold_v <= 1'b1;
            end
            if (word_accept) begin
                word_idx <= word_idx + 1'b1;
                if (in_pix) begin
                    if (pix_sub == 3'd5) begin pix_sub <= 0; src_hold_v <= 1'b0; end
                    else pix_sub <= pix_sub + 1'b1;
                end
            end
        end
    end
endmodule
