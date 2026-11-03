//=============================================================================
//  ui_ae_ctrl.v --- 自动曝光控制环（传感器模拟增益版，第二阶段）
//
//  【策略】只调 IMX415 的**模拟增益**寄存器 0x3060，不碰曝光时间 SHS。
//    理由：
//      1) 增益的单调性没有歧义——码值越大越亮，可以直接闭环；
//         SHS 是"帧长 − 曝光行数"，方向反了好几条街，没数据手册不敢乱写。
//      2) 亮场景降增益 → 传感器不过曝 → 细节真的能救回来，这正是
//         "对准亮物体要压低亮度"要的效果（ISP 数字增益做不到）。
//      3) 暗场景提增益 → 变亮，噪点是物理代价，无法避免。
//    下一步要接曝光时间时，只要在这里多算一个 SHS 值即可，控制环不动。
//
//  【寄存器】IMX415（与 imx415_cfg_rom.v 里已有的写法一致）：
//      0x3060  模拟增益码 0x25 = 上电默认（约 11 dB）
//      0x3061  fine gain，本模块不动
//    增益码步进 0.3 dB：0x00 = 0 dB，每 +1 码 +0.3 dB。
//
//  【控制律】每 UPDATE_DIV 帧更新一次：
//      err = target − mean
//      |err| <= DEADBAND            → 不动（防止在目标附近来回抖）
//      err > 0（偏暗）              → gain += STEP（最多到 GAIN_MAX）
//      err < 0（偏亮）              → gain -= STEP（最少到 GAIN_MIN）
//    步进限幅保证一次最多动 STEP 个码，画面不会一跳一跳。
//
//  【为什么不用除法/对数】
//    只用整数比较和加减，没有除法器、没有对数近似，DSP 占用为 0。
//    收敛速度靠 STEP 和 UPDATE_DIV 一起调，上板改这两个参数即可。
//=============================================================================
`timescale 1ns/1ps

module ui_ae_ctrl #(
    parameter integer TARGET_MEAN = 125,   // 目标平均亮度 0..255（2026-10-31 由 110 上调）
    parameter integer DEADBAND    = 6,     // ±死区，进了就不动
    parameter [7:0] GAIN_INIT = 8'h25, // 上电默认增益码（与 cfg_rom 一致）
    parameter [7:0] GAIN_MIN  = 8'h00, // ★ 放宽下限
    parameter [7:0] GAIN_MAX  = 8'h60, // ★ 放宽上限
    parameter [7:0] STEP      = 8'd4,     // 每次更新的增益码步进（★ 2→4）
    parameter integer UPDATE_DIV  = 1,     // 每 N 帧更新一次（★ 2026-10-31: 5→1，加快收敛）
    // ★ 测试开关：置 1 时忽略闭环，增益在 LO/HI 之间每 ~2 秒来回扫，
    //   用来判断"I2C 写到底有没有落到传感器上"——画面应随之一亮一暗。
    parameter integer AE_FORCE_EN = 0,
    parameter [7:0]   FORCE_LO    = 8'h00,
    parameter [7:0]   FORCE_HI    = 8'h60
)(
    input  wire        I_clk,          // 配置时钟域（和 I2C 同一个）
    input  wire        I_rst_n,
    input  wire        I_enable,       // 1 = 闭环使能；0 = 保持默认增益

    input  wire [7:0]  I_mean,         // 实测平均亮度（已跨到本时钟域）
    input  wire        I_frame_pulse,  // 每帧一个脉冲（已跨域）

    output reg  [7:0]  O_gain,         // 当前增益码
    output reg         O_upd,          // 增益变化脉冲，I2C 侧据此写寄存器
    output wire [7:0]  O_err          // 最近一次误差（调试/OSD 用）
);

    reg [7:0] div_cnt;
    reg [6:0] sweep_cnt;      // 测试扫掠用
    reg       sweep_hi;
    // ★ 2026-10-31：不用有符号比较（无符号 reg 和负数比较会两边都成立，
    //   导致增益只加不减）。改成对 8 位亮度直接做无符号上下限比较。
    reg [7:0] mean_lat;

    assign O_err = mean_lat;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            div_cnt <= 8'd0;
            sweep_cnt <= 7'd0;
            sweep_hi <= 1'b0;
            O_gain  <= (AE_FORCE_EN != 0) ? FORCE_LO : GAIN_INIT;
            O_upd   <= 1'b0;
            mean_lat<= 8'd0;
        end
        else begin
            O_upd <= 1'b0;
            mean_lat <= I_mean;

            if (AE_FORCE_EN != 0) begin
                // 测试模式：每 16 个更新节拍（约 1 秒）在 LO / HI 之间切换。
                // 画面能看到一明一暗的周期变化，就说明写**确实落到传感器**上了。
                if (I_frame_pulse) begin
                    if (sweep_cnt >= 7'd15) begin
                        sweep_cnt <= 7'd0;
                        sweep_hi  <= ~sweep_hi;
                        O_gain    <= sweep_hi ? FORCE_LO : FORCE_HI;
                        O_upd     <= 1'b1;
                    end
                    else sweep_cnt <= sweep_cnt + 7'd1;
                end
            end
            else if (!I_enable) begin
                // 关闭时缓慢回到默认增益，避免下次打开时突变
                if (O_gain > GAIN_INIT) begin O_gain <= O_gain - 1'b1; O_upd <= 1'b1; end
                else if (O_gain < GAIN_INIT) begin O_gain <= O_gain + 1'b1; O_upd <= 1'b1; end
                div_cnt <= 8'd0;
            end
            else if (I_frame_pulse) begin
                if (div_cnt >= UPDATE_DIV - 1) begin
                    div_cnt <= 8'd0;
                    // 偏暗：mean < TARGET − DEADBAND（无符号比较）
                    if (I_mean < TARGET_MEAN - DEADBAND) begin
                        // 偏暗：加增益
                        if (O_gain < GAIN_MAX) begin
                            O_gain <= (O_gain + STEP > GAIN_MAX) ? GAIN_MAX
                                                                 : O_gain + STEP;
                            O_upd <= 1'b1;
                        end
                    end
                    else if (I_mean > TARGET_MEAN + DEADBAND) begin
                        // 偏亮：减增益
                        if (O_gain > GAIN_MIN) begin
                            O_gain <= (O_gain < GAIN_MIN + STEP) ? GAIN_MIN
                                                                 : O_gain - STEP;
                            O_upd <= 1'b1;
                        end
                    end
                end
                else begin
                    div_cnt <= div_cnt + 1'b1;
                end
            end
        end
    end

endmodule
