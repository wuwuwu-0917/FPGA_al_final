//=============================================================================
//  imx415_i2c_master.v  ---  IMX415 寄存器写专用 I2C 主机（位级，开漏）
//
//  一次事务固定 4 字节：
//      S | dev+W | A | addr[15:8] | A | addr[7:0] | A | data[7:0] | A | P
//  与索尼手册 p28 及米联客 uii2c(WMEN_LEN=4) 完全一致。
//
//  设计要点
//  ---------
//  1) 开漏输出：O_scl_oe / O_sda_oe 为高时表示"拉低总线"，为低时释放（靠外部上拉）。
//     顶层接法：
//         assign IO_sc2210_scl = O_scl_oe ? 1'b0 : 1'bz;
//         assign IO_sc2210_sda = O_sda_oe ? 1'b0 : 1'bz;
//
//  2) 时基为"四分之一 SCL 周期"，每个 bit 走 4 个相位：
//         phase0 : SCL 低 —— 建立 SDA
//         phase1 : SCL 低 —— 保持
//         phase2 : SCL 高 —— 上升沿，从机采样
//         phase3 : SCL 高 —— 保持，本模块在此采样从机的 ACK
//     于是 SCL 一个完整周期 = 4*QUARTER 拍。
//
//     QUARTER = CLK_FREQ / (4 * SCL_FREQ)
//     本工程 CLK_FREQ = 71.428571MHz(S_clk_70m), SCL_FREQ = 100kHz
//         -> QUARTER = (71428571 + 200000) / 400000 = 179
//         -> 实际 SCL = 71428571 / (4*179) = 99.76 kHz
//
//  3) 时钟拉伸：SCL_STRETCH_CHK = 0（默认）时不检查，行为与米联客 uii2c 等价，
//     不引入任何对回读通路的依赖 —— 上板首选。
//     置 1 时会等待 I_scl_i 真正变高才推进，并带超时保护（O_timeout）。
//     注：本工程 SCL 经板上 TXS0104/TXS0108 电平转换，回读是否可靠需实测，
//         所以默认关闭，避免"配置流程卡死"这种新故障模式。
//
//  4) 本模块只实现"写"。IMX415 初始化不需要读。
//     从机地址由 I_dev_addr 在运行时给出 —— 这样上层 FSM 可以先扫描 4 个候选地址
//     （0x1A/0x10/0x36/0x37，由模组内部 SLAMODE 引脚决定，连接器上量不到）。
//
//  时钟域：单时钟，无 CDC。复位低有效。
//=============================================================================
`timescale 1ns / 1ps

module imx415_i2c_master #(
    parameter integer CLK_FREQ        = 71428571,   // I_clk 频率(Hz)
    parameter integer SCL_FREQ        = 100000,     // 目标 SCL 频率(Hz)
    parameter integer SCL_STRETCH_CHK = 0,          // 1 = 检查 SCL 时钟拉伸
    parameter integer TIMEOUT_Q       = 4096        // 拉伸等待上限(四分之一周期数)
)(
    input  wire        I_clk,
    input  wire        I_rst_n,

    // ---- 命令接口 ----
    input  wire        I_start,      // 单拍脉冲：发起一次写事务
    input  wire [6:0]  I_dev_addr,   // 7bit 从机地址（事务期间必须保持稳定）
    input  wire [15:0] I_reg_addr,   // 寄存器地址（16bit）
    input  wire [7:0]  I_reg_data,   // 寄存器数据（8bit）

    // ---- 状态 ----
    output reg         O_busy,       // 事务进行中
    output reg         O_done,       // 完成脉冲（1 拍）
    output reg         O_nack,       // 与 O_done 同拍：本次事务出现过 NACK
    output reg         O_timeout,    // 与 O_done 同拍：SCL 拉伸超时被中止

    // ---- 开漏总线 ----
    output reg         O_scl_oe,     // 1 = 拉低 SCL
    output reg         O_sda_oe,     // 1 = 拉低 SDA
    input  wire        I_scl_i,      // SCL 回读（仅 SCL_STRETCH_CHK=1 时使用）
    input  wire        I_sda_i       // SDA 回读（ACK 采样）
);

    //=========================================================================
    //  参数与常量
    //=========================================================================
    localparam integer QUARTER = (CLK_FREQ + 2*SCL_FREQ) / (4*SCL_FREQ);
    localparam integer NTICK   = (QUARTER < 2) ? 2 : QUARTER;

    localparam [1:0] ST_IDLE  = 2'd0,
                     ST_START = 2'd1,
                     ST_WRITE = 2'd2,
                     ST_STOP  = 2'd3;

    localparam [3:0] ACK_BIT = 4'd8;   // bit_idx = 8 表示 ACK 位

    //=========================================================================
    //  状态寄存器
    //=========================================================================
    reg [1:0]  S_st;
    reg [1:0]  S_phase;       // 0..3
    reg [15:0] S_tick;        // 0..NTICK-1
    reg [1:0]  S_byte_idx;    // 0..3（0=dev+W, 1=addrH, 2=addrL, 3=data）
    reg [3:0]  S_bit_idx;     // 0..7 数据位，8 = ACK
    reg        S_nack_any;
    reg        S_aborted;

    reg [15:0] S_to_cnt;      // 拉伸等待计数

    //=========================================================================
    //  当前要发送的字节（组合选择，不用移位寄存器，省掉载入时序问题）
    //=========================================================================
    reg [7:0] S_txbyte;
    always @(*) begin
        case (S_byte_idx)
            2'd0   : S_txbyte = {I_dev_addr, 1'b0}; // 从机地址 + 写标志
            2'd1   : S_txbyte = I_reg_addr[15:8];
            2'd2   : S_txbyte = I_reg_addr[7:0];
            default: S_txbyte = I_reg_data;
        endcase
    end

    // 数据位：MSB 先出
    wire S_bit_val = S_txbyte[3'd7 - S_bit_idx[2:0]];

    //=========================================================================
    //  开漏驱动（纯组合，由 S_st / S_phase 决定）
    //=========================================================================
    always @(*) begin
        O_scl_oe = 1'b0;
        O_sda_oe = 1'b0;
        case (S_st)
            ST_START: begin
                // phase0: SCL=1,SDA=1(总线空闲)  phase1: SDA↓  phase2/3: SCL↓
                O_scl_oe = (S_phase >= 2'd2);
                O_sda_oe = (S_phase >= 2'd1);
            end
            ST_WRITE: begin
                O_scl_oe = (S_phase < 2'd2);            // 前半周期拉低 SCL
                if (S_bit_idx == ACK_BIT)
                    O_sda_oe = 1'b0;                    // ACK 位：主机释放 SDA
                else
                    O_sda_oe = ~S_bit_val;              // 数据位：0 才拉低
            end
            ST_STOP: begin
                // phase0: SCL=0,SDA=0  phase1: SCL↑  phase2: SDA↑ = STOP
                O_scl_oe = (S_phase == 2'd0);
                O_sda_oe = (S_phase <  2'd2);
            end
            default: begin
                O_scl_oe = 1'b0;                        // IDLE：全部释放
                O_sda_oe = 1'b0;
            end
        endcase
    end

    //=========================================================================
    //  时基 & 推进条件
    //=========================================================================
    wire S_tick_last = (S_tick >= (NTICK - 1));

    // 允许推进：拉伸检查关闭时永远允许；打开时要求 SCL 已经被拉低或已实际变高
    wire S_adv_ok;
    generate
        if (SCL_STRETCH_CHK == 0) begin : g_no_chk
            assign S_adv_ok = 1'b1;
        end else begin : g_chk
            assign S_adv_ok = O_scl_oe | I_scl_i;
        end
    endgenerate

    wire S_can_adv = S_tick_last & S_adv_ok;

    // 超时：仅在"该推进却卡住"时累加
    wire S_waiting = S_tick_last & (~S_adv_ok) & (S_st != ST_IDLE);
    wire S_to_hit  = S_waiting & (S_to_cnt >= (TIMEOUT_Q - 1));

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)
            S_tick <= 16'd0;
        else if (S_tick_last) begin
            if (S_adv_ok)
                S_tick <= 16'd0;
            // else: 保持 —— 等待被拉伸的 SCL
        end else
            S_tick <= S_tick + 16'd1;
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)
            S_to_cnt <= 16'd0;
        else if (!S_waiting)
            S_to_cnt <= 16'd0;
        else if (!S_to_hit)
            S_to_cnt <= S_to_cnt + 16'd1;
    end

    //=========================================================================
    //  事务起点 / 终点
    //=========================================================================
    wire S_start_latch = (S_st == ST_IDLE) & I_start;
    wire S_stop_end    = S_can_adv & (S_st == ST_STOP) & (S_phase == 2'd3);
    wire S_abort_now   = S_to_hit;

    //=========================================================================
    //  相位 / 字节 / 位 计数器
    //=========================================================================
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_st       <= ST_IDLE;
            S_phase    <= 2'd0;
            S_byte_idx <= 2'd0;
            S_bit_idx  <= 4'd0;
        end else if (S_abort_now) begin
            S_st       <= ST_IDLE;
            S_phase    <= 2'd0;
        end else if (S_start_latch) begin
            S_st       <= ST_START;
            S_phase    <= 2'd0;
            S_byte_idx <= 2'd0;
            S_bit_idx  <= 4'd0;
        end else if (S_can_adv && (S_st != ST_IDLE)) begin
            if (S_phase != 2'd3)
                S_phase <= S_phase + 2'd1;
            else begin
                S_phase <= 2'd0;
                case (S_st)
                    ST_START: begin
                        S_st       <= ST_WRITE;
                        S_byte_idx <= 2'd0;
                        S_bit_idx  <= 4'd0;
                    end
                    ST_WRITE: begin
                        if (S_bit_idx == ACK_BIT) begin
                            S_bit_idx <= 4'd0;
                            if (S_byte_idx == 2'd3)
                                S_st <= ST_STOP;        // 第 4 字节 ACK 之后收尾
                            else
                                S_byte_idx <= S_byte_idx + 2'd1;
                        end else
                            S_bit_idx <= S_bit_idx + 4'd1;
                    end
                    ST_STOP:
                        S_st <= ST_IDLE;
                    default:
                        ;
                endcase
            end
        end
    end

    //=========================================================================
    //  ACK 采样：ACK 位的 phase3 末尾（SCL 仍为高、SDA 稳定）
    //=========================================================================
    wire S_ack_sample = S_can_adv & (S_st == ST_WRITE) &
                        (S_bit_idx == ACK_BIT) & (S_phase == 2'd3);

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)
            S_nack_any <= 1'b0;
        else if (S_start_latch)
            S_nack_any <= 1'b0;
        else if (S_ack_sample)
            S_nack_any <= S_nack_any | I_sda_i;   // SDA 仍为高 = 从机没应答 = NACK
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)
            S_aborted <= 1'b0;
        else if (S_start_latch)
            S_aborted <= 1'b0;
        else if (S_abort_now)
            S_aborted <= 1'b1;
    end

    //=========================================================================
    //  对外状态输出
    //=========================================================================
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_busy    <= 1'b0;
            O_done    <= 1'b0;
            O_nack    <= 1'b0;
            O_timeout <= 1'b0;
        end else begin
            O_busy    <= (S_start_latch | O_busy) & ~(S_stop_end | S_abort_now);
            O_done    <= S_stop_end | S_abort_now;
            if (S_stop_end) begin
                O_nack    <= S_nack_any;
                O_timeout <= 1'b0;
            end else if (S_abort_now) begin
                O_nack    <= 1'b0;
                O_timeout <= 1'b1;
            end
        end
    end

endmodule
