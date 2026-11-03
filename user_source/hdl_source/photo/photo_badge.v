`timescale 1ns/1ps
//-----------------------------------------------------------------------------
// photo_badge —— 1920x1080 显示域右上角 16x16 拍照状态角标
//
// 位置：x ∈ [1400,1415]、y ∈ [96,111]（预览区内，不压参数面板）。
// 状态 one-hot（photo_fsm.O_status）：
//   idle  -> 不画
//   busy  -> 橙色 2Hz 闪
//   done  -> 绿色常亮
//   error -> 红色 1Hz 闪
//
// 时序：in_rect/rgb 先寄存一拍，输出再寄存一拍（共 **2 拍**），
//       三根同步与 RGB 走同一条链、完全对齐；HDMI 只看相对关系，2 拍无影响。
// 由上层用 `ifdef PHOTO_BADGE 包住例化，出问题可整块综合掉。
//
// ★ 灭相是**纯黑方块**（覆盖预览图），不是"透出底图"。要改成透视闪烁，
//   把 hit_r 改成 in_rect && blink[...] 即可，其它不用动。
// ★ 资源口径（别按"只加 27 个 FF"估算）：约 3 级 LUT（4 个比较器 + 与门）
//   + 约 130 个 FF（xcnt12 + ycnt11 + de_d1 + blink27 + hit/col/rgb
//   + O_* 输出寄存器），显示域原有约 1458 FF，占比约 +9%。
// ★ BLINK_BIT 的取值范围受 tb 约束：一帧约 18 万拍，值太大（>15）时
//   "一帧内必然覆盖两个相位"就不再成立，闪烁断言会变成概率通过。
//-----------------------------------------------------------------------------
module photo_badge #(
    parameter integer BLINK_BIT = 25      // 150MHz 下 [25]≈2.2Hz、[26]≈1.1Hz；tb 用小值
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire [3:0]  I_status,          // one-hot: [0]idle [1]busy [2]done [3]error
    input  wire        I_full,            // ★ 1 = 全屏（无 UI），角标改画在全画面右上角
    input  wire        I_vs, I_hs, I_de,
    input  wire [23:0] I_rgb,
    output reg         O_vs, O_hs, O_de,
    output reg  [23:0] O_rgb
);
    localparam [23:0] C_ORANGE = 24'hFF8000;
    localparam [23:0] C_GREEN  = 24'h00C000;
    localparam [23:0] C_RED    = 24'hFF0000;

    // x/y 计数（与 UI 输出同步）
    reg [11:0] xcnt;
    reg [10:0] ycnt;
    reg        de_d;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            xcnt <= 12'd0; ycnt <= 11'd0; de_d <= 1'b0;
        end else begin
            de_d <= I_de;
            if (I_vs)          begin xcnt <= 12'd0; ycnt <= 11'd0; end
            else if (I_de)     xcnt <= xcnt + 12'd1;
            else if (de_d)     begin xcnt <= 12'd0; ycnt <= ycnt + 11'd1; end
        end
    end

    // 闪烁
    reg [BLINK_BIT+1:0] blink;
    always @(posedge I_clk or negedge I_rst_n)
        if (!I_rst_n) blink <= 0;
        else          blink <= blink + 1'b1;

    // 只有 busy/done/error 才画；idle 时整块透明（矩形内保持原像素）
    wire badge_on = I_status[1] | I_status[2] | I_status[3];
    // ★ 2026-11-01 两套坐标：
    //   可见 UI：预览区是 x<1456 / y∈[78,897)，角标放 (1400,96) 右上角；
    //   全屏：画面铺满 1920x1080、顶栏没了，所以贴到 (1888,24)。
    wire [11:0] bx0 = I_full ? 12'd1888 : 12'd1400;
    wire [10:0] by0 = I_full ? 11'd24   : 11'd96;
    wire in_rect = I_de && badge_on &&
                   (xcnt >= bx0) && (xcnt < bx0 + 12'd16) &&
                   (ycnt >= by0) && (ycnt < by0 + 11'd16);
    wire [23:0] badge_col = I_status[1] ? (blink[BLINK_BIT]   ? C_ORANGE : 24'h000000) :
                            I_status[2] ? C_GREEN :
                            I_status[3] ? (blink[BLINK_BIT+1] ? C_RED    : 24'h000000) :
                                          24'h000000;

    // 两级寄存：rgb / 命中 / 颜色 / 同步一起延迟，保持对齐（共 2 拍）
    reg        hit_r;
    reg [23:0] col_r, rgb_r;
    reg        vs_r, hs_r, de_r;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            hit_r <= 1'b0; col_r <= 24'd0; rgb_r <= 24'd0;
            vs_r <= 1'b0; hs_r <= 1'b0; de_r <= 1'b0;
            O_rgb <= 24'd0; O_vs <= 1'b0; O_hs <= 1'b0; O_de <= 1'b0;
        end else begin
            hit_r <= in_rect;
            col_r <= badge_col;
            rgb_r <= I_rgb;
            vs_r  <= I_vs; hs_r <= I_hs; de_r <= I_de;
            O_rgb <= hit_r ? col_r : rgb_r;
            O_vs  <= vs_r; O_hs <= hs_r; O_de <= de_r;
        end
    end
endmodule
