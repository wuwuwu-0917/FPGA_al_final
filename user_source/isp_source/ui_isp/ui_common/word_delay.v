//=============================================================================
//  word_delay.v --- 按"字"计数的环形延时（AXI-Stream 直通，含同步信号）
//
//  来源：FPGA_lvjing/rtl/word_delay.v
//  相对原文件的改动：
//    1) 删掉 `initial $display(...)`（综合不需要，TD 也不喜欢 initial）
//    2) $clog2 换成自带的 clog2_f（TD 6.2 对 $clog2 支持不稳）
//    3) 存储阵列加 `/* fehdl force_ram=1 */` 注释，保证 TD 推成 ERAM 而不是触发器
//
//  【用途】把 blur_gray 和 line_window_3x3 的时间轴对齐（见 pencil_front.v）
//
//  【实现】一块 RAM 当环形缓冲：
//      写地址 wp 每拍 +1；读地址取 wp+1（下一个要写的格）。
//      mem[wp+1] 上一次被写是在 N-1 拍之前 —— 正好是"N 拍之前的字"。
//      取 wp+1 而不是 wp，是为了把"寄存读"那一拍补回来，让总延时正好等于 N。
//
//  【同步信号】不另做移位寄存器 —— valid/last/user 拼在数据后面一起存进 RAM。
//
//  【暖机门控】复位后前 N 拍读到的 RAM 格还没写过（是 X）。
//    所以用计数器 gate 住输出：没到 N 拍就强行输出 0（用 mux，不用 &，否则 X 会漏）。
//
//  【延迟】正好 N 个"有效拍"（不是 N 拍，空档不算）。⚠️ 要求 N >= 2。
//=============================================================================
`timescale 1ns/1ps

module word_delay #(
    parameter integer W = 96,      // 数据位宽
    parameter integer N = 7680     // 延时多少个字（>= 2）
)(
    input  wire           I_clk,
    input  wire           I_rst_n,

    input  wire [W-1:0]   I_data,
    input  wire           I_valid,
    input  wire           I_last,
    input  wire           I_user,

    output wire [W-1:0]   O_data,
    output wire           O_valid,
    output wire           O_last,
    output wire           O_user
);

    //-------------------------------------------------------------------------
    // 本地 clog2（移位法，纯常数在综合期算掉）
    //-------------------------------------------------------------------------
    function integer clog2_f;
        input integer v;
        integer i;
        begin
            clog2_f = 0;
            for (i = v - 1; i > 0; i = i >> 1)
                clog2_f = clog2_f + 1;
        end
    endfunction

    localparam integer AW = (N <= 1) ? 1 : clog2_f(N);
    localparam integer RW = W + 3;

    reg [RW-1:0] mem [0:N-1];  /* fehdl force_ram=1, ram_style="bram" */
    reg [AW-1:0] wp;
    reg [AW:0]   cnt;                       // 有效拍计数，到 N 为止

    wire [AW-1:0] wp_next = (wp == N-1) ? {AW{1'b0}} : (wp + 1'b1);
    wire [AW-1:0] rp      = wp_next;        // 读"下一个要写的格"（见文件头）
    wire          ready   = (cnt >= N);     // 暖机完成

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            wp  <= {AW{1'b0}};
            cnt <= {(AW+1){1'b0}};
        end else if (I_valid) begin
            wp <= wp_next;
            if (!ready) cnt <= cnt + 1'b1;
        end
    end

    reg [RW-1:0] rd;

    //-----------------------------------------------------------------------
    // ★ 读也必须跟着 valid 走（原版是每拍都读）
    //
    //   本工程的视频流**不是每拍都有字**：每行 480 个有效字之后有 ~478 拍空档
    //   （传感器 bursts 完一行就歇着）。如果读寄存器每拍都更新，空档期 wp 冻住、
    //   rp 冻住，rd 会一直读到同一格 -> O_valid 冻在高电平，下游会收到成倍的
    //   假数据（实测 ispchain 每帧 1200366 字 vs 应有的 518400）。
    //
    //   改成"只在有效拍读"之后：rd 与 wp/rp 完全同步，第 k 个有效输入拍对应的
    //   输出出现在第 k+N 个有效拍上，空档期输出也自然为 0。
    //-----------------------------------------------------------------------
    always @(posedge I_clk) begin
        if (I_valid) begin
            mem[wp] <= {I_data, I_user, I_last, I_valid};
            rd      <= mem[rp];
        end
    end

    // 暖机没完成时强行输出 0（用 mux，不用 &，否则 X 会漏出去）
    //   输出做成组合，不要再打一拍 —— 否则延时变成 N+1 拍
    //
    //   ⚠️ O_valid 必须再 & I_valid：rd 在空档期是保持值，光看 rd[0] 会一直为 1
    //      （实测 ispchain 每帧 1200366 字 vs 应有的 518400）
    wire out_ok = ready & I_valid;

    //  ★★★ 2026-10-18 晚：O_data **绝对不能**用 out_ok / I_valid 门控！
    //
    //   这条是"全屏规则黑点"的根因。时序是：
    //      · rd 只在 I_valid 时更新，空档期保持"上一拍该输出的那个字"
    //      · 下游 comic_v11 是在 vld_d1（= I_tvalid 打一拍）上采样的
    //      · 空档后的第一拍：I_valid=0 但 vld_d1=1  → 正好在**这一拍**采样
    //   所以只要把 O_data 门成 0，下游就会把 0 当成真实像素写进 g_d_s / G_1..4，
    //   于是 C−g 突然变成 C−0 > 12 → ink 打满 → 该像素变成黑点。
    //
    //   直连（480 连续 + 478 空档）时每个空档只在行末，每行只错 1 个字；
    //   ui_filter_domain 的跨域 FIFO 把空档打散到行中间之后，每行错几十个字，
    //   实测 24% 的字错乱 —— 就是上板看到的满屏黑点。
    //
    //   正确做法：O_data 只在**暖机未完成**时清 0，之后一直给出 rd；
    //             O_valid 保留 & I_valid（空档期不宣告有效，字不会丢 ——
    //             rd 在空档期不更新，下一个有效拍仍然输出同一个字）。
    assign O_data  = ready ? rd[RW-1:3] : {W{1'b0}};
    assign O_valid = out_ok & rd[0];
    assign O_last  = out_ok & rd[1];
    assign O_user  = out_ok & rd[2];

endmodule
