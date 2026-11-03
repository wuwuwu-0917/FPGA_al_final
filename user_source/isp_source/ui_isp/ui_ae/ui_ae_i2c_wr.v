//=============================================================================
//  ui_ae_i2c_wr.v --- AE 的 I2C 写事务生成器（只写一个寄存器）
//
//  用法：给一个增益码 + 更新脉冲，本模块向 IMX415 的 0x3060 写一次。
//        和 imx415_cfg_fsm 共用同一个 imx415_i2c_master，由顶层仲裁：
//        配置完成前（I_allow=0）本模块完全不动总线。
//
//  【为什么要超时】i2c_master 的 start 是电平/脉冲握手，若因任何原因
//    没被响应，FSM 会永远卡在 WAIT。加一个计数器兜底，超时就放弃这次写，
//    下一次更新再试，绝不阻塞。
//=============================================================================
`timescale 1ns/1ps

module ui_ae_i2c_wr #(
    parameter [6:0]  DEV      = 7'h1A,
    // ★ 2026-10-31 查 IMX415 手册确认：
    //   GAIN_PCG_0 [8:0] @ 0x3090h，0.0~72.0dB / 0.3dB 步进，
    //   "The same setting is applied in all colors"（全局增益）。
    //   原来写的 0x3060 在寄存器表里查不到，写了不报错但完全无效。
    parameter [15:0] REG_ADDR = 16'h3090,
    parameter integer TIMEOUT = 16'd60000   // 70MHz 下约 0.86ms，够一次写
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_allow,        // 配置完成才允许占用总线
    // ★ 2026-10-31：从机地址**必须用 cfg_fsm 扫描到的实际地址**，
    //   写死 7'h1A 在地址不符时会被 NACK，表现为"增益在跳但画面不变"。
    input  wire [6:0]  I_dev,          // 实际使用的从机地址（O_dev_used）
    input  wire [7:0]  I_gain,         // 要写的增益码
    input  wire        I_upd,          // 更新脉冲

    output reg         O_start,
    output reg  [6:0]  O_dev,
    output reg  [15:0] O_addr,
    output reg  [7:0]  O_data,
    input  wire        I_busy,
    input  wire        I_done,
    input  wire        I_nack,        // 传感器回 NACK：写没落地的最直接证据
    output wire        O_active,       // 1 = 正在占用 I2C
    output reg  [7:0]  O_nack_cnt,     // NACK 计数（饱和）
    output reg  [7:0]  O_wr_cnt        // ★ 实际发出的写事务次数（饱和），
                                       //   0 = 一次都没发（allow 没起来），
                                       //   递增但画面不变 = 发出去了但无效
);

    localparam [1:0] IDLE = 2'd0, WAIT = 2'd1;

    reg [1:0]  st;
    reg [15:0] tmo;

    assign O_active = (st != IDLE);

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            st      <= IDLE;
            O_start <= 1'b0;
            O_dev   <= I_dev;
            O_addr  <= REG_ADDR;
            O_data  <= 8'd0;
            tmo     <= 16'd0;
            O_nack_cnt <= 8'd0;
            O_wr_cnt   <= 8'd0;
        end
        else begin
            O_start <= 1'b0;
            case (st)
                IDLE: begin
                    tmo <= 16'd0;
                    if (I_upd && I_allow) begin
                        O_dev   <= I_dev;
                        O_addr  <= REG_ADDR;
                        O_data  <= I_gain;
                        O_start <= 1'b1;
                        if (O_wr_cnt != 8'hFF) O_wr_cnt <= O_wr_cnt + 8'd1;
                        st      <= WAIT;
                    end
                end
                WAIT: begin
                    if (I_done) begin
                        if (I_nack && O_nack_cnt != 8'hFF) O_nack_cnt <= O_nack_cnt + 8'd1;
                        st <= IDLE;
                    end
                    else if (tmo >= TIMEOUT) begin
                        st <= IDLE;          // 超时放弃，下次再试
                    end
                    else begin
                        tmo <= tmo + 16'd1;
                    end
                end
                default: st <= IDLE;
            endcase
        end
    end

endmodule
