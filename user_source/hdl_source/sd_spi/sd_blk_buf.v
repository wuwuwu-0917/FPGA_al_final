//=============================================================================
//  sd_blk_buf.v  ---  拍照块缓冲（512 x 16bit，浅双口 RAM + 预取寄存器）
//
//  用途：packer 写入 16bit 字，sd_write 以 256 字为一块读出。
//
//  为什么需要预取寄存器（C2）：
//    sd_write 在 O_wr_req 之后第 2 拍才采样 wr_data（块头 bit_cnt==14 /
//    每个字 bit_cnt==14 拉高，还有 2 拍才把数据锁进 wr_data_t）。工程里的
//    sfifo 是 show-ahead 语义（re 当拍就推进读指针），直接接会让每块丢第 0
//    字、整块后移 16bit。本模块在每个 I_wr_req 当拍读 RAM、下一拍把数据送
//    到 O_wr_data（寄存输出），正好落在 sd_write 的 2 拍采样窗口里。
//
//  指针：
//    wr_ptr / rd_ptr 为 AW+1 位（深度 512 -> 10bit，mod 1024），RAM 地址取
//    低 AW 位。O_count = wr_ptr - rd_ptr（0..512）。
//    I_blk_start  : blk_rd <= rd_ptr（本块从 rd_ptr 开始）
//    I_blk_commit : rd_ptr += BLK_WORDS（本块成功，消耗 256 字）
//    I_blk_rewind : blk_rd <= rd_ptr（本块失败，rd_ptr 不动，重发同一块）
//  深度固定 512 字（一块的余量）。
//
//  ★ I_blk_start 契约：必须在第一个 I_wr_req 之前给出，且块间保持低
//    （单拍脉冲最好）。它在高电平期间会每个时钟都把 blk_rd 重装成 rd_ptr，
//    若一直拉高，读指针不会推进，sd_write 会取到重复数据。
//
//  注：BLK_WORDS 固定 256（深度固定 512，AW 固定 9）；O_count 固定 10bit。
//      改 BLK_WORDS 需要同步改深度/指针宽度/O_count。
//=============================================================================
module sd_blk_buf #(
    parameter integer BLK_WORDS = 256
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_we,          // packer 写使能
    input  wire [15:0] I_din,         // packer 写数据
    input  wire        I_blk_start,   // 脉冲：读指针从 rd_ptr 开始
    input  wire        I_blk_commit,  // 脉冲：本块成功，rd_ptr += BLK_WORDS
    input  wire        I_blk_rewind,  // 脉冲：本块失败，读指针回到 rd_ptr
    output wire [9:0]  O_count,       // 可读字数（0..512）
    output wire        O_full,
    input  wire        I_wr_req,      // sd_write 的 256 次取数请求
    output reg  [15:0] O_wr_data      // 预取数据（寄存输出）
);

    // 深度固定 512（不用 $clog2，避免 TD 6.2 对 $clog2 的不确定性）
    localparam integer DEPTH = 512;
    localparam integer AW    = 9;

    reg [15:0] ram [0:DEPTH-1];
    reg [AW:0] wr_ptr;      // 10bit
    reg [AW:0] rd_ptr;      // 10bit
    reg [AW:0] blk_rd;      // 10bit

    assign O_count = wr_ptr - rd_ptr;
    assign O_full  = (O_count >= DEPTH);

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            wr_ptr    <= {(AW+1){1'b0}};
            rd_ptr    <= {(AW+1){1'b0}};
            blk_rd    <= {(AW+1){1'b0}};
            O_wr_data <= 16'd0;
        end
        else begin
            // 写侧
            if (I_we && !O_full) begin
                ram[wr_ptr[AW-1:0]] <= I_din;
                wr_ptr <= wr_ptr + 1'b1;
            end
            // 读侧预取：I_wr_req 当拍读当前字，下一拍出现在 O_wr_data
            if (I_wr_req)
                O_wr_data <= ram[blk_rd[AW-1:0]];
            // 块内读指针
            if (I_blk_start)
                blk_rd <= rd_ptr;
            else if (I_blk_rewind)
                blk_rd <= rd_ptr;          // 重发同一块
            else if (I_wr_req)
                blk_rd <= blk_rd + 1'b1;
            // 块提交
            if (I_blk_commit)
                rd_ptr <= rd_ptr + BLK_WORDS;
        end
    end

endmodule
