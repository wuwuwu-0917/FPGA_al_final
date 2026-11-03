//=============================================================================
//  imx415_power_seq.v  ---  IMX415 上电 / 复位时序器
//
//  索尼手册 IMX415-AAQR-C p84「Power-on and Power-off Sequence」原文要求：
//
//    1. 电源按 DVDD(1.1V) -> OVDD(1.8V) -> AVDD(2.9V) 顺序上升（板级，本模块无关）
//    2. 所有电源升完之后，XCLR 保持低电平 >= 500ns
//    3. 把 XCLR 拉高产生 system clear。
//       **"The master clock input after setting the XCLR pin to High level."**
//       —— INCK 必须 **在 XCLR 拉高之后** 才给。
//    4. system clear 之后才做寄存器通信。
//
//  手册 p84 时序表：
//       TLOW : AVDD 上升      -> Clear OFF        >= 500 ns
//       T3   : Clear OFF      -> INCK 上升        >= 1   us
//       T4   : Clear OFF      -> 通信开始         >= 20  us
//    （"Clear OFF" = XCLR 的上升沿）
//
//  ---------------------------------------------------------------------------
//  本模块时钟域为**板级 25MHz 直连时钟 I_sys_clk_25m**，不是 PLL 产物。
//  这样做的原因：XCLR 和 MCLK 使能必须是"PLL 之外"的逻辑才能保证先后顺序
//  —— 如果放在 PLL 输出的时钟域里，PLL 一停就什么都做不了，
//     而且 MCLK 本身就是 PLL 的 C0 输出，用它自己的时钟域去门控它自己会绕圈。
//
//  输出：
//      O_xclr_n     -> 顶层 O_sc2210_rst_n（低有效）
//      O_mclk_en    -> 顶层用来门控 27MHz 相机 MCLK（见 GATE_MCLK）
//      O_cfg_start  -> 给 imx415_cfg_fsm 的 I_start（跨到 70MHz 域，用 2 级同步器）
//
//  ---------------------------------------------------------------------------
//  ★ GATE_MCLK（默认 1）—— 相对原计划的一处改动，请知悉
//
//    原计划 PLAN_IMX415_BRINGUP.md 的"决策 B"写的是「不门控 MCLK，先按 INCK 一直
//    在跑做，若上板不出图再回头补」。但手册 p84 第 3 条是**明文要求** INCK 必须在
//    XCLR 拉高之后才给，而本工程的 MCLK 是 PLL C0 直出、PLL 一锁就一直在跑
//    —— 也就是说原设计**确实违反了这条规格**。
//
//    该项配置出错会导致无图像输出且难以定位，而实现代价仅为一个与门，故按正确方式处理：
//      - O_mclk_en 默认为 0，XCLR 拉高后 T3(>=1us) 才置 1；
//      - 顶层在 27MHz 时钟的**下降沿**打拍后再与，得到无毛刺的门控时钟
//        （使能只在时钟低电平期间变化，不会产生窄脉冲）。
//
//    若上板后怀疑是门控引入的问题，把 GATE_MCLK 置 0 即可回到"INCK 常开"，
//    其余逻辑完全不变。
//=============================================================================
`timescale 1ns / 1ps

module imx415_power_seq #(
    parameter integer CLK_FREQ     = 25000000,  // I_clk 频率(Hz)
    parameter integer XCLR_LOW_US  = 2,         // XCLR 低电平时长(us)，手册要求 >= 0.5us
    parameter integer INCK_DLY_US  = 3,         // Clear OFF -> INCK 上升(us)，手册要求 >= 1us
    parameter integer PWR_WAIT_MS  = 24,        // PLL 锁定后等待(ms)，留足电源/稳压器稳定
    parameter integer GATE_MCLK    = 1          // 1 = 由 O_mclk_en 门控相机 MCLK
)(
    input  wire       I_clk,        // 板级 25MHz
    input  wire       I_rst_n,      // 低有效（顶层建议接 I_key_in）
    input  wire       I_pll_lock,   // PLL 锁定指示

    output reg        O_xclr_n,     // 相机复位，低有效
    output reg        O_mclk_en,    // 相机 MCLK 使能（GATE_MCLK=0 时恒 1）
    output reg        O_cfg_start,  // 配置开始（常高电平，给 cfg_fsm）
    output reg  [2:0] O_state       // 调试用
);

    //=========================================================================
    //  常量
    //=========================================================================
    localparam [2:0] ST_IDLE     = 3'd0,   // 等 PLL 锁定
                     ST_XCLR_LOW = 3'd1,   // XCLR 拉低
                     ST_MCLK_DLY = 3'd2,   // XCLR 拉高后等 T3，再放 MCLK
                     ST_PWR_WAIT = 3'd3,   // 等电源/稳压器稳定
                     ST_DONE     = 3'd4;

    localparam integer XCLR_LOW_CYC = (CLK_FREQ / 1000000) * XCLR_LOW_US;
    localparam integer MCLK_DLY_CYC = (CLK_FREQ / 1000000) * INCK_DLY_US;
    localparam integer PWR_WAIT_CYC = (CLK_FREQ / 1000) * PWR_WAIT_MS;

    //=========================================================================
    //  状态
    //=========================================================================
    reg [2:0]  S_state;
    reg [31:0] S_cnt;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_state     <= ST_IDLE;
            S_cnt       <= 32'd0;
            O_xclr_n    <= 1'b0;    // 复位期间 XCLR 保持低 —— 相机一直处于 system clear
            O_mclk_en   <= (GATE_MCLK == 0) ? 1'b1 : 1'b0;
            O_cfg_start <= 1'b0;
            O_state     <= ST_IDLE;
        end else begin
            O_state <= S_state;

            case (S_state)

            //-------------------------------------------------------------
            // 等 PLL 锁定。之前 XCLR 一直为低、MCLK 关闭。
            ST_IDLE: begin
                S_cnt <= 32'd0;
                if (I_pll_lock)
                    S_state <= ST_XCLR_LOW;
            end

            //-------------------------------------------------------------
            // XCLR 保持低 >= 500ns（手册第 2 条 / TLOW）
            ST_XCLR_LOW: begin
                O_xclr_n <= 1'b0;
                if (S_cnt >= (XCLR_LOW_CYC - 1)) begin
                    S_cnt   <= 32'd0;
                    S_state <= ST_MCLK_DLY;
                end else
                    S_cnt <= S_cnt + 32'd1;
            end

            //-------------------------------------------------------------
            // XCLR 拉高 = system clear 完成；等 T3(>=1us) 之后再放 INCK
            // （手册第 3 条："The master clock input after setting the XCLR pin to High level"）
            ST_MCLK_DLY: begin
                O_xclr_n <= 1'b1;
                if (S_cnt >= (MCLK_DLY_CYC - 1)) begin
                    S_cnt     <= 32'd0;
                    O_mclk_en <= 1'b1;
                    S_state   <= ST_PWR_WAIT;
                end else
                    S_cnt <= S_cnt + 32'd1;
            end

            //-------------------------------------------------------------
            // 等电源/内部稳压器稳定。
            // 这一段同时覆盖了手册 T4（Clear OFF -> 通信开始 >= 20us）和
            // p87 流程里上电后的 24ms 稳定期，纯保守、只有一次，代价可忽略。
            ST_PWR_WAIT: begin
                O_xclr_n  <= 1'b1;
                O_mclk_en <= 1'b1;
                if (S_cnt >= (PWR_WAIT_CYC - 1)) begin
                    S_cnt       <= 32'd0;
                    O_cfg_start <= 1'b1;
                    S_state     <= ST_DONE;
                end else
                    S_cnt <= S_cnt + 32'd1;
            end

            //-------------------------------------------------------------
            ST_DONE: begin
                O_xclr_n    <= 1'b1;
                O_mclk_en   <= 1'b1;
                O_cfg_start <= 1'b1;
            end

            default: S_state <= ST_IDLE;
            endcase
        end
    end

endmodule
