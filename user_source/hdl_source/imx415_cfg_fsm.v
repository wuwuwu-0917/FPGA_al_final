//=============================================================================
//  imx415_cfg_fsm.v  ---  IMX415 上电配置状态机
//
//  对应索尼手册 p84(上电时序) + p87(Setting Flow, Slave Mode)：
//
//      Power-on
//        -> XCLR 低 >=500ns -> XCLR 高(=System clear)   [由 imx415_power_seq.v 负责]
//        -> INCK 必须在 XCLR 拉高之后才给                [同上]
//        -> 通信开始 (Clear OFF -> >=20us)               [同上, 给出 I_start]
//        -> 写 201 条配置表(第 0 条即 STANDBY=1 进待机)
//        -> 间隔 >=24ms
//        -> 写 0x3000=0x00 (STANDBY 取消) + 0x3002=0x00 (XMSTA 主机开始)
//        -> 稳压等待 >=24ms   (手册 TSYNC: Standby OFF -> 24ms)
//        -> O_cfg_done = 1
//
//  ★ 关于 0x3000 / 0x3002 这两条为什么不在 ROM 里：
//      这样 imx415_cfg_rom.v 就能与米联客 uics500reg.v **逐条 1:1 对应**，
//      便于用脚本核对 201 条一个不多一个不少。这两条由本模块在第二段产生。
//
//  ★ 从机地址自动扫描（SCAN_EN=1，默认）
//      IMX415 的 7bit I2C 地址由模组内部 SLAMODE0/SLAMODE1 引脚决定（手册 p26），
//      连接器上量不到，有 0x1A / 0x10 / 0x36 / 0x37 四种可能。
//      本模块上电后先对这 4 个地址各做一次写探测（写 0x3000=0x01，
//      即使命中也是我们本来就要做的"进待机"，无害），ACK 结果记在 O_scan_result，
//      然后自动选用第一个应答的地址跑完整张表。
//      若一个都没应答，回退到 DEV_ADDR 并继续跑完整张表
//      —— 此时 O_scan_result = 0 且 O_err_cnt 会涨到 201，一眼能看出问题。
//
//  实现说明：整个状态机放在**一个 always 块**里（含事务的 issue/wait 子相位），
//           避免两个进程互相等待造成的重复发起竞争。
//
//  时钟域：单时钟（建议 S_clk_70m, 71.428571MHz），无 CDC。复位低有效。
//=============================================================================
`timescale 1ns / 1ps

module imx415_cfg_fsm #(
    parameter integer CLK_FREQ  = 71428571,   // I_clk 频率(Hz)，用于算延时
    parameter integer GAP_MS    = 24,         // 第一段 -> 第二段 的间隔(ms)
    parameter integer STAB_MS   = 24,         // 第二段 -> cfg_done 的稳压等待(ms)
    parameter integer ROM_SIZE  = 201,        // 配置表条数
    parameter integer SCAN_EN   = 1,          // 1 = 先扫描 4 个候选从机地址
    parameter [6:0]   DEV_ADDR  = 7'h1A       // SCAN_EN=0 时直接用这个地址
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_start,       // 高电平即可（power_seq 给的是常高电平）

    // ---- 配置表 ROM ----
    output wire [7:0]  O_rom_index,
    input  wire [23:0] I_rom_data,    // { addr[15:0], value[7:0] }

    // ---- I2C 主机 ----
    output reg         O_i2c_start,
    output reg  [6:0]  O_i2c_dev,
    output reg  [15:0] O_i2c_addr,
    output reg  [7:0]  O_i2c_data,
    input  wire        I_i2c_busy,
    input  wire        I_i2c_done,
    input  wire        I_i2c_nack,

    // ---- 状态 / 调试 ----
    output reg         O_cfg_done,
    output reg  [7:0]  O_err_cnt,     // 收到 NACK 的事务数
    output reg  [3:0]  O_scan_result, // [i]=1 -> 第 i 个候选地址有 ACK
    output reg  [6:0]  O_dev_used,    // 实际使用的从机地址，便于 ChipWatcher 观察
    output reg  [2:0]  O_state        // 主状态机当前状态（见下方 localparam）
);

    //=========================================================================
    //  常量
    //=========================================================================
    localparam [2:0] ST_IDLE  = 3'd0,   // 等待启动
                     ST_SCAN  = 3'd1,   // 扫描 4 个候选从机地址
                     ST_TABLE = 3'd2,   // 第一段：写 201 条配置表
                     ST_GAP   = 3'd3,   // 段间延时 24ms
                     ST_SEG2  = 3'd4,   // 第二段：取消待机 2 条
                     ST_STAB  = 3'd5,   // 稳压等待 24ms
                     ST_DONE  = 3'd6;

    localparam [0:0] SUB_ISSUE = 1'b0,  // 发起一次 I2C 写事务
                     SUB_WAIT  = 1'b1;  // 等这次事务结束

    localparam integer SEG2_COUNT = 2;
    localparam integer SCAN_COUNT = 4;

    // 手册 p26 的四个候选从机地址
    localparam [6:0] CAND0 = 7'h1A,
                     CAND1 = 7'h10,
                     CAND2 = 7'h36,
                     CAND3 = 7'h37;

    localparam integer GAP_CYCLES  = (CLK_FREQ / 1000) * GAP_MS;
    localparam integer STAB_CYCLES = (CLK_FREQ / 1000) * STAB_MS;

    //=========================================================================
    //  寄存器
    //=========================================================================
    reg [2:0]  S_state;
    reg        S_sub;
    reg [7:0]  S_idx;
    reg [31:0] S_dly_cnt;

    //=========================================================================
    //  ROM 索引：直接跟随 S_idx（组合），因此 I_rom_data 在同一拍有效
    //=========================================================================
    assign O_rom_index = S_idx;

    //=========================================================================
    //  本次要写的 {addr, data}
    //=========================================================================
    reg [23:0] S_wr_word;
    always @(*) begin
        case (S_state)
            ST_SEG2 : S_wr_word = (S_idx == 8'd0) ? 24'h300000 : 24'h300200;
            ST_SCAN : S_wr_word = 24'h300001;   // 探测写 STANDBY=1（命中即我们想要的状态）
            default : S_wr_word = I_rom_data;   // ST_TABLE
        endcase
    end

    // 扫描阶段逐个换地址
    reg [6:0] S_cand;
    always @(*) begin
        case (S_idx[1:0])
            2'd0   : S_cand = CAND0;
            2'd1   : S_cand = CAND1;
            2'd2   : S_cand = CAND2;
            default: S_cand = CAND3;
        endcase
    end

    // 由扫描结果选定地址（扫描尚未出结果时先回退到 DEV_ADDR）
    reg [6:0] S_dev_sel;
    always @(*) begin
        if (SCAN_EN == 0)          S_dev_sel = DEV_ADDR;
        else if (O_scan_result[0]) S_dev_sel = CAND0;
        else if (O_scan_result[1]) S_dev_sel = CAND1;
        else if (O_scan_result[2]) S_dev_sel = CAND2;
        else if (O_scan_result[3]) S_dev_sel = CAND3;
        else                       S_dev_sel = DEV_ADDR;   // 一个都没应答时的回退
    end

    // 当前阶段应该用哪个地址
    wire [6:0] S_dev_now = (S_state == ST_SCAN) ? S_cand : S_dev_sel;

    //=========================================================================
    //  主状态机（单进程，含事务 issue/wait 子相位）
    //=========================================================================
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_state       <= ST_IDLE;
            S_sub         <= SUB_ISSUE;
            S_idx         <= 8'd0;
            S_dly_cnt     <= 32'd0;
            O_i2c_start   <= 1'b0;
            O_i2c_dev     <= DEV_ADDR;
            O_i2c_addr    <= 16'd0;
            O_i2c_data    <= 8'd0;
            O_cfg_done    <= 1'b0;
            O_err_cnt     <= 8'd0;
            O_scan_result <= 4'd0;
            O_dev_used    <= DEV_ADDR;
            O_state       <= ST_IDLE;
        end else begin
            O_state <= S_state;

            case (S_state)

            //-------------------------------------------------------------
            ST_IDLE: begin
                O_i2c_start <= 1'b0;
                if (I_start) begin
                    O_cfg_done    <= 1'b0;
                    O_err_cnt     <= 8'd0;
                    O_scan_result <= 4'd0;
                    O_dev_used    <= DEV_ADDR;
                    S_idx         <= 8'd0;
                    S_sub         <= SUB_ISSUE;
                    S_state       <= (SCAN_EN != 0) ? ST_SCAN : ST_TABLE;
                end
            end

            //-------------------------------------------------------------
            // 扫描 4 个候选地址：各发一次 {0x3000, 0x01}
            ST_SCAN: begin
                O_dev_used <= S_cand;
                case (S_sub)
                SUB_ISSUE: begin
                    if (!I_i2c_busy) begin
                        O_i2c_dev   <= S_cand;
                        O_i2c_addr  <= S_wr_word[23:8];
                        O_i2c_data  <= S_wr_word[7:0];
                        O_i2c_start <= 1'b1;
                        S_sub       <= SUB_WAIT;
                    end
                end
                SUB_WAIT: begin
                    O_i2c_start <= 1'b0;
                    if (I_i2c_done) begin
                        S_sub <= SUB_ISSUE;
                        // 有 ACK 才算命中（I_i2c_nack=1 表示从机没应答）
                        O_scan_result[S_idx[1:0]] <= ~I_i2c_nack;
                        if (S_idx == (SCAN_COUNT - 1)) begin
                            S_idx   <= 8'd0;
                            S_state <= ST_TABLE;
                        end else
                            S_idx <= S_idx + 8'd1;
                    end
                end
                endcase
            end

            //-------------------------------------------------------------
            // 第一段：写 ROM 里的 201 条
            ST_TABLE: begin
                O_dev_used <= S_dev_sel;
                case (S_sub)
                SUB_ISSUE: begin
                    if (!I_i2c_busy) begin
                        O_i2c_dev   <= S_dev_sel;
                        O_i2c_addr  <= S_wr_word[23:8];
                        O_i2c_data  <= S_wr_word[7:0];
                        O_i2c_start <= 1'b1;
                        S_sub       <= SUB_WAIT;
                    end
                end
                SUB_WAIT: begin
                    O_i2c_start <= 1'b0;
                    if (I_i2c_done) begin
                        S_sub <= SUB_ISSUE;
                        if (I_i2c_nack) O_err_cnt <= O_err_cnt + 8'd1;
                        if (S_idx == (ROM_SIZE - 1)) begin
                            S_idx     <= 8'd0;
                            S_dly_cnt <= 32'd0;
                            S_state   <= ST_GAP;
                        end else
                            S_idx <= S_idx + 8'd1;
                    end
                end
                endcase
            end

            //-------------------------------------------------------------
            // 段间间隔
            ST_GAP: begin
                O_i2c_start <= 1'b0;
                if (S_dly_cnt >= (GAP_CYCLES - 1)) begin
                    S_dly_cnt <= 32'd0;
                    S_idx     <= 8'd0;
                    S_sub     <= SUB_ISSUE;
                    S_state   <= ST_SEG2;
                end else
                    S_dly_cnt <= S_dly_cnt + 32'd1;
            end

            //-------------------------------------------------------------
            // 第二段：0x3000=0x00(待机取消) + 0x3002=0x00(主机模式开始)
            ST_SEG2: begin
                O_dev_used <= S_dev_sel;
                case (S_sub)
                SUB_ISSUE: begin
                    if (!I_i2c_busy) begin
                        O_i2c_dev   <= S_dev_sel;
                        O_i2c_addr  <= S_wr_word[23:8];
                        O_i2c_data  <= S_wr_word[7:0];
                        O_i2c_start <= 1'b1;
                        S_sub       <= SUB_WAIT;
                    end
                end
                SUB_WAIT: begin
                    O_i2c_start <= 1'b0;
                    if (I_i2c_done) begin
                        S_sub <= SUB_ISSUE;
                        if (I_i2c_nack) O_err_cnt <= O_err_cnt + 8'd1;
                        if (S_idx == (SEG2_COUNT - 1)) begin
                            S_dly_cnt <= 32'd0;
                            S_state   <= ST_STAB;
                        end else
                            S_idx <= S_idx + 8'd1;
                    end
                end
                endcase
            end

            //-------------------------------------------------------------
            // 稳压等待（手册 TSYNC: Standby OFF -> 24ms）
            ST_STAB: begin
                O_i2c_start <= 1'b0;
                if (S_dly_cnt >= (STAB_CYCLES - 1)) begin
                    S_dly_cnt  <= 32'd0;
                    O_dev_used <= S_dev_sel;
                    O_cfg_done <= 1'b1;
                    S_state    <= ST_DONE;
                end else
                    S_dly_cnt <= S_dly_cnt + 32'd1;
            end

            //-------------------------------------------------------------
            ST_DONE: begin
                O_i2c_start <= 1'b0;
                O_dev_used  <= S_dev_sel;
                // 停在 DONE，直到复位。cfg_done 保持高电平供 ChipWatcher 观察。
            end

            default: S_state <= ST_IDLE;
            endcase
        end
    end

endmodule
