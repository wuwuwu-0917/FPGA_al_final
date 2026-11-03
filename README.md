# 可调实时艺术滤镜相机

基于安路 PH1A90 FPGA 的实时图像处理与交互实现。
本目录包含最终版本的 **Verilog 源码** 与对应的 **FPGA 比特流文件**。

---

## 一、比特流

```
td_project/Verimake_camerasc2210_mipi_ddr_hdmi_Runs/best_result/
    Verimake_camerasc2210_mipi_ddr_hdmi.bit
```

| 项        | 值                                                                   |
| --------- | -------------------------------------------------------------------- |
| 大小      | 4,850,318 字节                                                       |
| sha256    | `d79dcf3f48c942cb0a55a0087f55434eda7f1efe7a048b8c2853b3e3ee010a57` |
| 目标器件  | 安路 PH1A90SBG484（speed-2）                                         |
| 开发工具  | Tang Dynasty 6.2.1                                                   |
| 布局种子  | place seed 4                                                         |
| Setup WNS | +7 ps，0 违例                                                        |
| Hold WNS  | +20 ps，0 违例                                                       |
| 资源占用  | 48,234 slices（74.99%），268/272 ERAM，193/240 DSP                   |

烧写后接 HDMI 显示器与 5V 电源即可离线运行，无需 PC、无需操作系统。

---

## 二、功能

### 1. 五路实时艺术滤镜

全部在 FPGA 内实时处理，不依赖 PC 或云端算力。

| 模式 | 可调参数                                                 |
| ---- | -------------------------------------------------------- |
| 原图 | 直通                                                     |
| 像素 | 强度；风格 D（4px/lab32）/ E（8px/DB32）/ F（8px/PICO8） |
| 浮雕 | 强度；深度；光照方向                                     |
| 铅笔 | 强度；边缘阈值；色彩强度                                 |
| 漫画 | 强度；描边强度；墨线阈值                                 |

- 每种滤镜支持 **0–100 强度混合**，可在原图与滤镜之间连续过渡
- 五路共用 3×3 窗口与 16 行模糊前端，**切换模式时画面不跳变**
- 支持 **原图/滤镜分割对比**，分割线位置 0–100 可调

### 2. 画质处理

- IMX415 传感器采集，4-lane MIPI CSI-2，RAW10
- 完整 ISP 链：黑电平校正 → 去马赛克 → 自动白平衡 → Gamma 2.2
  → 对比度 → 亮度 → 饱和度
- **传感器级自动曝光**：目标平均亮度 125，死区 ±6
- **保亮度直方图均衡**：64 bin，强度 0–100 可调

### 3. 交互与显示

- **双键操作**：板上 K4 / P5 完成模式切换、进入参数页、调节参数、恢复默认，
  支持单击、双击、长按与组合键，无需 PC、鼠标或触控屏
- **九宫格构图辅助线**：三分线 + 中心十字准星，1px 白线
- **隐藏式原生 1080p 全屏**：长按进入无遮挡全屏，长按恢复
- **屏幕信息**：实测帧率、端到端延迟、AE 两行读数（亮度/增益）、拍照状态角标
- HDMI 直接输出 1920×1080，VTC 约 60.6 Hz

### 4. 拍照存储

- 一键将当前画面保存为 **1920×1080 BMP** 并写入 TF 卡
- SD SPI 模式，8 个 6MB 槽位循环使用
- 拍照过程中与显示通路共享 DDR 带宽，带请求超时重发与仲裁保护，
  保证抓帧通路在带宽竞争下可靠完成

---

## 三、技术特点

1. **五路滤镜共用前端 + 统一对齐点**：原图、像素、浮雕、铅笔、漫画共用 3×3 窗口
   与 16 行模糊，四路对齐到统一有效拍，像素路经前端匹配延迟汇合，
   实现运行时无跳变切换与原图分割对比。
2. **可见/全屏双帧缓存**：可见模式 1456×819，全屏 1920×1080，各三帧缓存；
   上电暖机后只写当前显示的一路，减少无效 DDR 带宽。
3. **AE 与 HE 解耦**：测光放在 HE 之前，HE 只加局部对比不改整体亮度，
   避免两个自动环互相追。
4. **资源受限下的优化**：像素滤镜全程用移位加替代常量乘法，实现 0 DSP；
   HE 查表、字模阴影、Logo 使用分布式 RAM；OSD 几何判断提前一拍预计算，
   保证 150MHz 时序收敛。

---

## 四、工程结构

目录结构与 Tang Dynasty 生成的工程一致，可直接用 TD 打开或命令行构建。

```
src/
    hdmi_filter_ui.v            两键 UI、OSD 绘制、构图辅助线、AE 读数、帧率与延迟显示
user_source/
    hdl_source/                 顶层、相机配置、CSI 解包、视频时序、DDR/HDMI/SD、拍照子系统
        design_top_wrapper.v    顶层：时钟、复位与各子系统互联
        photo/                  拍照：状态机、抓帧、回读、BMP 打包、FDMA 仲裁、状态角标
        sd_spi/                 SD 卡 SPI 主机
        uivtc.v                 视频时序发生器
    isp_source/
        ui_isp/                 ISP 链 + 75MHz 滤镜域 + 漫画滤镜
        ui_dbuf/                DDR 帧缓存控制器与同步 FIFO
    constraints_source/         引脚约束 pin.adc、时序约束 timing.sdc
    ip_source/                  DDR、PLL、MIPI、HDMI 等厂商 IP 配置

td_project/
    Verimake_camerasc2210_mipi_ddr_hdmi.al               TD 工程文件
    Verimake_camerasc2210_mipi_ddr_hdmi_devicesetting.cfg 器件设置
    Verimake_camerasc2210_mipi_ddr_hdmi_Runs/
        syn_1/                    综合运行目录
            settings.cfg                                    综合设置
            Verimake_camerasc2210_mipi_ddr_hdmi.prj          工程文件（源文件清单）
            Verimake_camerasc2210_mipi_ddr_hdmi_gate.db      综合网表（布线输入）
            gate.qor                                        综合时序与面积报告
        phy_1/                    布局布线运行目录（place seed 4）
            settings.cfg                                    布线设置
            route.qor                                       时序报告
            Verimake_camerasc2210_mipi_ddr_hdmi_phy.area    资源报告
            ...                                             其余为布线过程记录
        best_result/
            Verimake_camerasc2210_mipi_ddr_hdmi.bit          最终比特流
```

共 **121 个 `.v` 文件**。

**时钟划分**：相机/ISP 域约 74MHz，滤镜域 75MHz，AXI 域约 133MHz，
显示域 150MHz，HDMI 串行 750MHz。跨域信号使用异步 FIFO、两级同步和 toggle 握手。

---

## 五、重新构建

工程文件已包含在内，可直接构建。

```sh
# 1) 综合（约 6 分钟）
cd td_project/Verimake_camerasc2210_mipi_ddr_hdmi_Runs/syn_1
D:/Software/TD/bin/td_commands_prompt.exe D:/Software/TD/doc/scripts/DefaultFlow.tcl
#    产物：Verimake_camerasc2210_mipi_ddr_hdmi_gate.db

# 2) 布线：在 TD 中打开 td_project/Verimake_camerasc2210_mipi_ddr_hdmi.al，
#    选择综合得到的 gate.db 作为输入，place seed 设为 4，执行布局布线。
```
