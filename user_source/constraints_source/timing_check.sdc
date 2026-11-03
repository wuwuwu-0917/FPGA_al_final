#create_clock -period 10 -name ddr_clk -waveform {0 5} [get_nets I_ddr_clk]
create_clock -name clk_25m -period 40 -waveform {0 20} [get_ports {I_sys_clk_25m}]
 
#-----------------------------------------------------------------------------
# hs_rx_clk = MIPIIO 的 byte clock，来自恢复出来的 HS 时钟。
# D-PHY 关系：byte_clk = 线速率 / 8
#     SC2210 时代 :  period 20 ->  50.0 MHz -> 400 Mbps/lane
#     IMX415 修正版 : period 13.47 -> 74.25 MHz -> 594 Mbps/lane
#                     （594/8 = 74.25 MHz，理论周期 13.468ns）
#
# ★ 这是全工程唯一一处对 MIPI 线速率的时序假设，也是本次改造风险最高的一环：
#   hs_rx_clk 域使用修正版 IMX415 的 74.25MHz byte clock。
#   若综合后时序不收敛，可先检查 hs_rx_clk 与传感器配置是否均为 74.25MHz。
#   修正版传感器配置与该时钟周期保持一致。
#-----------------------------------------------------------------------------
create_clock -name hs_rx_clk -period 13.47 -waveform {0 6.735} [get_pins {u_mipi_dphy_rx_ph1a_mipiio_wrapper/u_ph1a_mipiio_wrapper/u_PH1_PHY_MIPIIO.hsrx_byteclk_to_fabric}]
 
#------------------------------------------------------------------------------
# 150MHz HDMI 像素时钟 / 750MHz HDMI 串行时钟
#
# 【2026-10-12 修正】原来这两行写的是 create_clock ... [get_nets {S_hdmi_clk}]，
# 但综合后网表里没有叫 S_hdmi_clk / S_serial_clk 的 net，工具静默忽略，
# 于是下面 set_clock_groups 里 get_clocks {S_hdmi_clk} 拿到空集 ——
# 150MHz 域没被排除出异步组，DDR rfifo 里的格雷码 CDC 被当成同域路径去算，
# 报出 -1.060ns 的假违例（占当时 19 个 setup 违例中的 17 个）。
# 现改为与 cpu_clk_70m / cam_clk_27m 相同的 rename_clock 写法（同源于 PLL）。
#
# 注意：S_hdmi_clk 与 S_serial_clk 是 1:5 同步关系，必须留在**同一个 group**，
#       让 TMDS 编码器 -> 串行器装载寄存器 那条真实跨域路径继续被检查
#       （实测 Setup -0.039ns / Hold -0.114ns）。
#       参考工程 18_fpga_isp 把两者声明为互相异步（等于 waive 掉这条路径），
#       本工程不采用该做法。
#------------------------------------------------------------------------------
#create_clock -name I_ddr_clk -period 10 -waveform {0 5} [get_nets {I_ddr_clk}]
 
rename_clock -name {cpu_clk_70m} -source [get_ports {I_sys_clk_25m}] -master_clock {clk_25m} [get_pins {U_PLL/pll_inst.clkc[2]}]
rename_clock -name {cam_clk_27m} -source [get_ports {I_sys_clk_25m}] -master_clock {clk_25m} [get_pins {U_PLL/pll_inst.clkc[0]}]
rename_clock -name {S_hdmi_clk}   -source [get_ports {I_sys_clk_25m}] -master_clock {clk_25m} [get_pins {U_PLL/pll_inst.clkc[3]}]
rename_clock -name {S_serial_clk} -source [get_ports {I_sys_clk_25m}] -master_clock {clk_25m} [get_pins {U_PLL/pll_inst.clkc[4]}]
# ★ 2026-10-21 滤镜域：PLL C1 = 75MHz（CLKC1_DIV=20），独立于相机域 / 显示域
rename_clock -name {S_filt_clk}   -source [get_ports {I_sys_clk_25m}] -master_clock {clk_25m} [get_pins {U_PLL/pll_inst.clkc[1]}]

derive_pll_clocks


set_false_path -from [get_regs {vtc_vs}] -to [get_regs {u_uifdma_axi_ddr/u_ddr_phy/u_clk/hctrl_clk_bufg}]

# 【2026-10-12 修正】-exclusive 改 -asynchronous（这些时钟是同时运行的，不是互斥的）。
#
# ★ 2026-10-28：S_hdmi_clk 和 S_serial_clk 拆成两个异步 group。
#   原因：21:36 那版 build（加了像素滤镜）报
#       S_hdmi_clk -> S_serial_clk  hold -0.152ns / 19 端点
#   而这条路径的 launch/capture 全在加密的 hdmi_phy_wrapper 硬宏内部
#   （S_odd_even_reg -> S_load_en_reg / S_shift_data_reg），fabric 侧无法改；
#   厂商参考工程 18_fpga_isp 也是把 pclkx1/pclkx5 声明成异步 group。
#   这里按参考工程 waive 这条内部路径，先验证“无信号”是否由它引起。
#   ⚠️ 这是约束 waiver，不是物理修复：若上板恢复信号但出现随机黑屏/花屏，
#      说明 PHY 内部 hold 仍有物理风险，应回退成下面注释的同一 group，
#      改用多 seed / 降像素时钟（55Hz）。
#
# 原始（1:5 同组，继续检查）：
set_clock_groups -asynchronous -group [get_clocks {S_hdmi_clk S_serial_clk}] -group [get_clocks { clk_25m }] -group [get_clocks { cpu_clk_70m }] -group [get_clocks { cam_clk_27m }] -group [get_clocks { hs_rx_clk }] -group [get_clocks {S_filt_clk}] -group [get_clocks {u_uifdma_axi_ddr/u_ddr_phy/hctrl_clk}]
# set_clock_groups -asynchronous -group [get_clocks {S_hdmi_clk}] -group [get_clocks {S_serial_clk}] -group [get_clocks { clk_25m }] -group [get_clocks { cpu_clk_70m }] -group [get_clocks { cam_clk_27m }] -group [get_clocks { hs_rx_clk }] -group [get_clocks {S_filt_clk}] -group [get_clocks {u_uifdma_axi_ddr/u_ddr_phy/hctrl_clk}]

