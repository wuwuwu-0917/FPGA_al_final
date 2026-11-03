
/*******************************MILIANKE*******************************
*Company : MiLianKe Electronic Technology Co., Ltd.
*WebSite:https://www.milianke.com
*TechWeb:https://www.uisrc.com
*tmall-shop:https://milianke.tmall.com
*jd-shop:https://milianke.jd.com
*taobao-shop1: https://milianke.taobao.com
*Create Date: 2023/03/23
*Module Name:
*File Name:
*Description: 
*The reference demo provided by Milianke is only used for learning. 
*We cannot ensure that the demo itself is free of bugs, so users 
*should be responsible for the technical problems and consequences
*caused by the use of their own products.
*Copyright: Copyright (c) MiLianKe
*All rights reserved.
*Revision: 1.0
*Signal description
*1) I_ input
*2) O_ output
*3) IO_ input output
*4) S_ system internal signal
*5) _n activ low
*6) _dg debug signal 
*7) _r delay or register
*8) _s state mechine
*********************************************************************/
/*********ui_image_correction 消减图像的多余的***********
--版本号1.0
*****************************************************************/
//=============================================================================
//  【本工程改造，非厂商原码】在米联客原始版基础上加了三件事：
//    1) 行边界改用 csi_unpacket 的 O_csi_rx_hsync0（真行同步），不再数 valid 空隙
//    2) 漏行补齐（replay）：行间隔异常时重放上一行（取 last-1，奇偶才正确）
//       => 去马赛克的 bayer_ypos 不会因丢行而整片翻相（满屏彩条）
//    3) 帧首丢行补偿：用"帧起始短包 -> 第一个行包头"的拍数量出帧首丢了几行
//       => 修"每隔几帧整帧变粉/绿"
//
//  ★ 全部时间常量按 **594 Mbps/lane、hs_rx_clk = 74.25MHz** 标定：
//        1H = 542 拍，行周期 2H = 1084 拍
//    改链路速率时，本文件里所有 542 / 1600 / 2700 / 1300 / 9000 / 2170 都要同步改。
//=============================================================================

module ui_image_correction #(
    parameter DATA_WIDTH   = 40,
    parameter TDEST_WIDTH  = 10
)(
    input                          		I_clk   			,
    input                          		I_rst_n 			,

    input    [DATA_WIDTH - 1:0]       	I_raw_data			,
    input                             	I_raw_valid 		,
    input                             	I_raw_frame_start	,
    input                             	I_raw_frame_end  	,

    // 【2026-10-13 新增】真行同步：来自 csi_unpacket 的 O_csi_rx_hsync0
    //   （每个 RAW10 长包的包头处 1 拍脉冲）。用它替代"数 valid 空隙"判行尾，
    //   理由见下面 last 的说明。
    input                             	I_line_start     	,
    // 输出RAW10数据	
    output     [DATA_WIDTH - 1:0]     	O_raw_tdata 		,
    output                            	O_raw_tlast 		,
    output reg [TDEST_WIDTH - 1:0]    	O_raw_tdest 		,
    output                            	O_raw_tvalid		,
    output                            	O_raw_tuser 		,
    input                             	O_raw_tready		,

    //  【本工程添加】长期可观测量（每帧在帧起始处清零）
    //      O_pad_cnt  —— 本帧补了几行（补行触发次数）
    //      O_miss_cnt —— 本帧"该补没补"的行数，**应当恒为 0**
    //  判读：miss_cnt != 0 ⇒ 那几行没补上 ⇒ 去马赛克行相位在那一行翻一次 ⇒ 一条粗彩条；
    //        pad_cnt 每帧约等于丢包数，可以拿来跟 D-PHY 的丢包对账。
    output     [15:0]                   O_pad_cnt			,
    output     [15:0]                   O_miss_cnt			,

    //  【本工程添加】行间隔极值（每帧清零）—— 判断补行是"真漏包"还是"误触发"
    //      正常行间隔 = 1626 拍；出现 3252/4878 才是真的漏了 1/2 行。
    //      若 gap_max 只有 1700~2100 却也触发了补行 ⇒ 门限定低了（误补）。
    output     [15:0]                   O_gap_max			,
    output     [15:0]                   O_gap_min
);

    reg                        I_raw_tvalid_d;
    reg    [DATA_WIDTH - 1:0]  I_raw_tdata_r;
    reg    [14:0]              h_cnt,v_cnt;
    wire                       V_valid,H_valid;
    wire                       tuser;

    //-------------------------------------------------------------------------
    //  【本工程添加，非厂商原码】补行（replay）用的信号，**必须先声明后使用**。
    //  厂商原码有"先用后声明"的地方，靠 Verilog 的隐式线网规则；TD 容忍，
    //  但 ModelSim 的 vlog 会报 vlog-2730 Undefined variable 而编不过。
    //  语义详见文件末尾"S_rep_*"那一段的注释。
    //-------------------------------------------------------------------------
    localparam integer MEM_DEPTH = 1024;      // 行缓存深度（2 个 480 字的分区）
    localparam integer MEM_AW    = 10;        // = log2(1024)

    reg  [1:0]              S_rms      = 2'd0;      // 补行状态机
    reg  [8:0]              S_rep_cnt  = 9'd0;      // 补行内计数 0..479
    reg                     S_rbuf     = 1'b0;      // 补行读的是哪个分区
    reg                     S_wsel     = 1'b0;      // 当前行写哪个分区
    //  （原来这里有个 S_gap_th 门限阶梯，2026-10-14 晚已删除 —— 触发点改成
    //    由 S_row_gap 直接推算，见文件末尾 S_owed 那一段。）
    reg  [3:0]              S_rep_num  = 4'd0;      // 本行间隔内已补了几行
    //  ★★★ 2026-10-14 深夜：**本帧是否已开始输出**。
    //  必须从**偶数的 v_cnt** 才开始（见文件末尾"帧首相位"那一段）。
    reg                     S_started  = 1'b0;
    reg                     S_rep_arm  = 1'b0;      // 帧内使能（帧首/帧尾自动关闭）
    wire                    S_replaying;            // 正在补行
    wire                    S_rep_go;               // 本拍启动一次补行
    wire [MEM_AW-1:0]       S_mem_raddr;
    wire [MEM_AW-1:0]       S_mem_waddr;
    wire                    S_mem_we;
    wire [DATA_WIDTH-1:0]   S_rep_data;
	
	assign H_valid 		= (h_cnt < 481) && (h_cnt > 0);
    
    //  【2026-10-13 同步修改】v_cnt 的计数时机从"行尾空隙"改成了"行首包头"，
    //    所以下面所有与 v_cnt 比较的门限都 +1，才能保持"丢掉第 1 行"这个原意不变
    //    （丢第 1 行是为了保住 Bayer 行相位，不能改）。
    //  ★★ 2026-10-13 15:3x：**必须过滤掉"假帧起始"** —— 这是本轮的关键修复
    //
    //  上板实测：每帧只数到 ~1050 个长包（应约 1096），而且
    //      line_min 恒 1082、line_period 恒 1083、line_max 恒 3252、gap_max 恒 626、
    //      S_mon_lane_err = 0、S_mon_xhs_cnt = 4566(=VMAX，证实时间模型正确)。
    //  这些一起排除了 D-PHY 丢包与并包，剩下的只有一种可能：
    //      **统计窗口被切碎了** —— 本模块的 v_cnt 被中途清零。
    //
    //  而 v_cnt 只由 I_raw_frame_start 清零，它来自 csi_unpacket 的
    //  O_csi_frame_start = "某个包的 DI 字节 == 0x00"。CSI-2 里 DI==0x00 就是
    //  帧起始**短包**，但**任何一个包头字节被采成 0x00 的上升沿也会命中**，
    //  包括行尾噪声、短包误判等。这种"假帧起始"每帧只要来一两个，
    //  就会：① 把统计窗口从中途切开 -> 行数读数偏小且每帧不同（992/1014/1053）；
    //        ② 让 v_cnt 归零 -> 行奇偶被打乱 -> **彩带**；
    //        ③ 让帧对齐错位 -> **竖直抖动**。
    //  一个机制同时解释三个症状，且它一直躲在诊断盲区里（原来的 S_mon_fs_cnt
    //  是在 tuser 处同时清零+锁存，读出来恒为 1，等于白装）。
    //
    //  判据：**真的帧起始前面一定有场消隐**（VMAX=4566H 里读 1096 行，
    //  消隐约 2374H ≈ 1.29M 拍）；而假的帧起始是夹在行流里的，
    //  离上一个行包头最多 3H ≈ 3252 拍。两者差三个数量级，取 8192 拍做门限。
    //
    //  复位后 S_no_line_cnt 预置成饱和值 -> 上电后的第一个帧起始一定放行。
    //=========================================================================
    reg [16:0] S_no_line_cnt = 17'h1FFFF;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)         S_no_line_cnt <= 17'h1FFFF;   // 复位后放行第一个
        else if(I_line_start) S_no_line_cnt <= 17'd0;      // 每个行包头清零
        else if(S_no_line_cnt != 17'h1FFFF)
            S_no_line_cnt <= S_no_line_cnt + 17'd1;
    end

    localparam [16:0] FS_MIN_GAP = 17'd8192;
    //  ★★★ 2026-10-13 23:0x：**门限已撤除**。
    //
    //  这个门限当初是为"假帧起始"加的（怕行流中间的噪声被当成帧起始，把 v_cnt 中途清零）。
    //  但后来加的 `S_mon_fs_rate`（1 秒窗口内的帧起始脉冲数）实测一直是 **28~29**，
    //  而 VMAX=4566 → 30fps ⇒ **根本没有多余/虚假的帧起始** ⇒ 这个门限从一开始就不需要。
    //
    //  22:54 上板实测：测量窗口变成**两帧**（frame_period 精确 2x、xvs_cnt=2、xhs_cnt=9132=2x4566），
    //  而 FS 包本身正常（fs_rate=29/秒）⇒ 说明 `ui_image_correction` **把一半的帧起始拒掉了**，
    //  只剩下这一个门限能造成这个后果 ⇒ 撤除。
    //
    //  保留 S_no_line_cnt 与 FS_MIN_GAP 的声明不动（零成本），
    //  万一以后真发现有假帧起始，把下面这行改回
    //      assign S_frame_start_ok = I_raw_frame_start & (S_no_line_cnt > FS_MIN_GAP);
    //  即可恢复门限。
    wire S_frame_start_ok = I_raw_frame_start;

    //  ★ 2026-10-13 20:5x 修正：**tuser 不能再判 `v_cnt == 2`**。
    //  v_cnt 现在会在漏行处一次跳 2~3 步（见下面 S_vstep），会**跳过 2** ——
    //  上板实测的后果是 tuser 一次都不触发，ISP 收不到帧起始，画面彻底碎掉
    //  （S_mon_frame_lines / rx_lines / frame_words 全读 0，raw_min/max 停在初值）。
    //  改成"本帧第一次进入 v_cnt >= 2 的那一行"，逐帧只发一次。
    reg S_user_sent;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)              S_user_sent <= 1'b0;
        else if(S_frame_start_ok) S_user_sent <= 1'b0;
        else if(tuser)            S_user_sent <= 1'b1;
    end

    //  ★ 2026-10-14 深夜：加 `V_valid` —— 现在"本帧是否已开始"由 S_started 决定
    //    （必须从偶数 v_cnt 开始），光判 v_cnt >= 2 已经不够了。
    assign tuser 		= (h_cnt == 1) && (v_cnt >= 2) && (~S_user_sent) && I_raw_tvalid_d && V_valid;
    
    //  ★★★ 2026-10-14：输出三件套加一层**补行覆盖**（详见文件末尾 S_rep_* 那段）。
    //  补行期间：tvalid 连续 480 拍、tlast 落在第 480 拍、tdata 来自行缓存。
    assign O_raw_tdata  = S_replaying ? S_rep_data
                                      : (V_valid && H_valid ? I_raw_tdata_r : 40'd0);
    assign I_raw_tready = O_raw_tready;
    //  行缓存是**异步读**（PH1_LOGIC_ERAM 用 REGMODE_B = "NOREG"，与厂商去马赛克一致），
    //  所以补行的第 1 拍就有数据 —— 480 拍连续 tvalid，tlast 落在第 480 拍（S_rep_cnt == 479）。
    assign O_raw_tvalid = S_replaying ? 1'b1
                                      : (H_valid && V_valid && I_raw_tvalid_d);
    assign O_raw_tlast  = S_replaying ? (S_rep_cnt == 9'd479)
                                      : (V_valid && (h_cnt == 480) && I_raw_tvalid_d);
    assign O_raw_tuser  = tuser;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n )begin
            I_raw_tvalid_d <= 0;
            I_raw_tdata_r  <= 0;
        end
        else begin
            I_raw_tvalid_d <= I_raw_valid;
            I_raw_tdata_r  <= I_raw_data;
        end
    end
    
    reg  [16:0] cnt;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n )begin
            cnt <= 0;
        end
        else if(I_raw_valid)
            cnt <= 0;
        else if(!I_raw_valid)begin
            cnt <= cnt + 1;
        end
    end

    //=========================================================================
    //  【2026-10-13 修复】行边界：由"数 valid 空隙"改为"包驱动"
    //
    //  原来：wire last = (cnt == 10);   // 连续 10 拍 valid 为空才算行尾
    //
    //  问题：传感器两个长包**背靠背**时（valid 不拉低），
    //        ui_raw10_unpacket_4lane 的 5:4 齿轮箱相位不复位 -> 两包之间毫无空隙
    //        -> 这里看不到行边界 -> **两行并成一行**。
    //        上板实测：收到 1063 个行包，却只输出 1006 行（每帧丢几十行）。
    //
    //  修复：改用 csi_unpacket 的 O_csi_rx_hsync0 —— 它是"每个 RAW10 长包
    //        包头处 1 拍脉冲"，是真正的行同步，不依赖包间是否有空隙。
    //        这里延迟几拍对齐到齿轮箱的输出（标定值见仿真 tb_isp_chain）。
    //
    //  注意：原来的 cnt 计数器保留未删（不影响功能），便于对照。
    //=========================================================================
    reg [3:0] S_ls_sr = 4'd0;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) S_ls_sr <= 4'd0;
        else         S_ls_sr <= {S_ls_sr[2:0], I_line_start};
    end

    wire last = S_ls_sr[2];      // 延迟 3 拍；若行数与仿真不符，改这个下标

    //=========================================================================


    //=========================================================================
    //  ★★ 2026-10-13 20:1x：**按行间隔检测"漏行"，让 v_cnt 与传感器的行号保持一致**
    //
    //  现象：上板实测仍有满屏彩带，但**彩条变粗、数量变少**，而且数量正好跟
    //        "漏行数量的减少"成比例（点完时钟延时 192 之后 rx_lines 1042 -> 1060）。
    //  机理：D-PHY 每帧还会漏掉约 20~35 个包。**漏掉的包不会产生 hsync0**，
    //        所以 v_cnt 不为它加 1 —— 从第一次漏行起，我们的 v_cnt 就比传感器的
    //        行号少了 1，之后每漏一次再少 1。
    //        而 ISP 的去马赛克是靠"从 tuser 起数了多少行"来定 RGGB 行相位的
    //        ⇒ **每漏一次相位就翻一次 ⇒ 满屏彩带**（彩带条数 ≈ 漏行次数）。
    //
    //  修法：量相邻两个行边界之间的拍数
    //        正常 = 2H = 1084 拍（594Mbps 下 hs_rx_clk = 74.25MHz）
    //        漏 1 个包 -> ~2168 拍；漏 2 个 -> ~3252 拍（实测 line_max 一直是 3250 ✓）
    //        ⇒ 间隔超过门限时 **v_cnt 一次多走几步**，把丢掉的行号补回来。
    //        ⇒ v_cnt 重新等于传感器的行号 ⇒ ISP 看到的相位回到正确值。
    //
    //  代价：输出行数会比 1080 少二十来行（约 2%），画面底部会有一条很窄的接缝
    //        （显示上一帧残留）。这比"满屏彩带"好得多。若还嫌它，下一步可以在
    //        这些空位上补零行（不需要行缓存）。
    //
    //  ★ 门限是按 594Mbps 定的；若改回 891Mbps（正常间隔 1626 拍），这两个数要一起改。
    //=========================================================================
    reg [15:0] S_row_gap = 16'd0;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)                   S_row_gap <= 16'd0;
        else if(last)                  S_row_gap <= 16'd0;
        else if(S_row_gap != 16'hFFFF) S_row_gap <= S_row_gap + 16'd1;
    end

    //  594Mbps：hs_rx_clk = 74.25MHz，1H = 542 拍，行间隔 2H = 1084 拍
    //    漏 1 行 2168，漏 2 行 3252
    //    GAP_1PKT 取 1084 与 2168 之间，GAP_2PKT 取 2168 与 3252 之间
    localparam [15:0] GAP_1PKT = 16'd1600;
    localparam [15:0] GAP_2PKT = 16'd2700;

    //=========================================================================
    //  ★★★ 2026-10-16：**帧首丢行的补偿** —— 修"每隔几帧整帧变粉/绿"
    //
    //  【问题】`v_cnt` = "传感器行号 + 1" 这个关系靠 `S_vstep` 维持，但**帧首那次
    //    `S_vstep` 被强制成 1**（帧首前面是场消隐，那段空隙量不出丢了几行）。
    //    **只要传感器第 0 行没到**（它没发出来 / PHY 漏收），帧首第一次 `last`
    //    其实是**传感器第 1 行**，而 `v_cnt` 只加到 1
    //    ⇒ 从此 `v_cnt` = "传感器行号 + 0" ⇒ **整帧差一个行相位** ⇒ 整帧颜色反。
    //
    //  【仿真坐实】tb_isp_chain case 9（把传感器第 0 行丢掉）：
    //      不丢行时 tuser 落在 tb_row=1；丢掉第 0 行后落在 tb_row=2 —— **差 1（奇数）⇒ 相位翻** ✓
    //    ★ 注意：这种错误**读 `v_cnt` 是看不出来的**（两次都读 2，偶数），
    //      必须拿"tuser 落在传感器第几行"来判 —— 上一轮那条判据就是因此瞎掉的。
    //
    //  【修法】用「帧起始短包 → 第一个行包头」之间的**拍数**量出帧首丢了几行：
    //    这段间隔完全由传感器决定（VMAX 与读出区都不变），一行不丢时是个常数；
    //    丢 N 行，它就长 N 个行周期。
    //    基准值（一行不丢时该是多少）**自校准**：取历史最小值 ——
    //      帧首丢行是 ~2.6% 的概率，多帧里总有没丢的；
    //      再加一个"±2 个行周期"的窗口，防止被毛刺毒化。
    //    ⇒ 帧首那次 `last` 让 `v_cnt` 多跳 `1 + 丢的行数`，
    //      于是 `v_cnt` = "传感器行号 + 1" 重新成立 ✓
    //=========================================================================
    //  ★ 计数器必须够宽：真机的"帧起始短包 -> 第一个行包头"约 **1.28M 拍**（场消隐），
    //    原来的 18 位（26 万）会直接饱和 —— 仿真里没暴露是因为 tb 的帧首几乎没空隙。
    localparam [20:0] FS_GAP_INIT = 21'h1FFFFF;
    localparam [20:0] FS_GAP_WIN  = 21'd2170;      // 校准窗口：±2 个行周期（2 x 1084）

    reg [20:0] S_fs_gap   = 21'd0;    // 帧起始短包 -> 第一个行包头 的拍数
    reg [20:0] S_fs_min   = FS_GAP_INIT;
    reg        S_fs_done  = 1'b0;     // 本帧已经量到了第一个行包头
    reg        S_fs_first = 1'b0;     // 本帧还没见过 last

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_fs_gap   <= 21'd0;
            S_fs_done  <= 1'b0;
            S_fs_first <= 1'b0;
        end
        else if(S_frame_start_ok) begin
            S_fs_gap   <= 21'd0;
            S_fs_done  <= 1'b0;
            S_fs_first <= 1'b1;
        end
        else begin
            if(!S_fs_done && (S_fs_gap != FS_GAP_INIT)) S_fs_gap <= S_fs_gap + 21'd1;
            if(last && S_fs_first) begin
                S_fs_done  <= 1'b1;
                S_fs_first <= 1'b0;
            end
        end
    end

    //  基准自校准：只在"接近当前基准"时才更新，防止被毛刺毒化。
    //  ★ 第一次必须无条件学进来 —— 否则哨兵值自己就把第一次学习挡掉了
    //    （一开始就是这么错的：`S_fs_min - S_fs_gap` = 26 万，远超窗口，永远学不进）。
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) S_fs_min <= FS_GAP_INIT;
        else if(S_fs_done) begin
            if((S_fs_gap < S_fs_min) &&
               ((S_fs_min == FS_GAP_INIT) || ((S_fs_min - S_fs_gap) < FS_GAP_WIN)))
                S_fs_min <= S_fs_gap;
        end
    end

    //  帧首丢的行数 ≈ 多出来的拍数 / 行周期
    //  594Mbps 下行周期 1084 ≈ 1024，用 >>10 近似即可（判定边界离真值 400+ 拍）
    wire [20:0] S_fs_excess = (S_fs_gap > S_fs_min) ? (S_fs_gap - S_fs_min) : 21'd0;
    wire [20:0] S_fs_rows_w = (S_fs_excess + 21'd542) >> 10;
    wire [15:0] S_fs_rows   = (S_fs_rows_w > 21'd15) ? 16'd15 : S_fs_rows_w[15:0];

    //  本拍 S_row_gap 还保持着"刚过去的那个间隔"（它在 last 处才清零）
    //  ★ 2026-10-13 20:5x 修正：**每帧第一个 last 必须只走 1 步**。
    //  原因：S_row_gap 里此刻还留着上一帧那 1.4M 拍的场消隐，会被判成"漏了很多"，
    //  于是 v_cnt 从 0 直接跳到 3 —— 而帧首本来就是 v_cnt=0，
    //  这样第一行就错位了（相位错 2 行）。
    //  v_cnt == 0 时一定是帧的第一行（帧起始处 v_cnt 被清零），强制走 1 步。
    wire [1:0] S_vstep = (v_cnt == 0) ? 2'd1 :
                         (S_row_gap > GAP_2PKT) ? 2'd3 :
                         (S_row_gap > GAP_1PKT) ? 2'd2 : 2'd1;

    // 计算行数
    //  ★ 2026-10-16：**帧首那一次要补上"帧首丢的行"**（见上面"帧首丢行的补偿"）——
    //    否则只要传感器第 0 行没到，`v_cnt` 从此就比传感器行号少 1，
    //    整帧的行相位会反（上板表现就是"每隔几帧一张全屏粉/绿帧"）。
    wire [15:0] S_step_now = (v_cnt == 0) ? (16'd1 + S_fs_rows)   // 帧首：1 + 帧首丢的行数
                                          : {14'd0, S_vstep};     // 其余：常规漏行补偿

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n || S_frame_start_ok)
            v_cnt <= 0;
        else if(last)
            v_cnt  <=  v_cnt + S_step_now;
    end

    //  【2026-10-13 同步修改】门限 +1（见上面 tuser 处的说明）：
    //    原来丢 v_cnt=0 那一行，现在等价于丢 v_cnt=1 那一行。
    //
    //  ★★★ 2026-10-13 22:5x：**把输出行数钉死成 1024 行**（原来是 1080，见下面说明）
    //
    //  为什么必须钉死：`ui_dbuf` 的写侧按 burst 计数、读侧由 VTC 按**固定 1080 行**读。
    //    实测每帧只交付 ~1032 行（传感器发了 ~1096，D-PHY 漏掉几十个包）⇒ 写侧每帧
    //    只写完约 95% ⇒ **读写指针赛跑 ⇒ 画面被撕成横向错位的带子，而且位置每帧移动**
    //    （灰度判别实验证实是"亮度/数据层"问题，不是颜色问题）。
    //    参考工程那边能收满 1080 行，所以不会出现这个现象。
    //
    //  取 1024 的理由：实测最少也有 1032 行（时钟延时定案 192 之后），留 8 行余量；
    //    而且是 2 的幂、是 480 的整数倍关系简单，burst 计数好算。
    //    代价：显示器 1080 行里约 56 行是重复内容 —— 但那是**固定位置**的，
    //    不是满屏乱跳的彩条。
    //
    //  ★ 配套改动（缺一不可）：
    //      design_top_wrapper.v 里 ui_dbuf 的 W_YSIZE 和 R_YSIZE 都改成 1024
    //      （写侧 1024*2 = 2048 burst，读侧同样 1024 行，两侧永远同步）
    //
    //  ★ 若以后 D-PHY 的漏包率降下来、能稳定收满 1082 行以上，
    //    可以把这里改回 `(v_cnt > 1) && (v_cnt < 1082)` 并把两个 YSIZE 改回 1080。
    //=========================================================================
    //  ★★★ 2026-10-13 23:2x：**输出行数必须用"已输出行数"来限，不能用 v_cnt。**
    //
    //  23:14 上板实测 frame_lines = 990（不是 1024），原因很清楚：
    //      v_cnt 是**传感器行号**（漏行时会一次跳 2~3 步把丢失的行号补回来），
    //      所以 v_cnt 走得比"实际收到的行数"快。
    //      实测收到 1061 个包、漏行补偿多走约 35 步 ⇒ v_cnt 走到 1096，
    //      而 `v_cnt < 1026` 这个上限意味着只输出到 v_cnt=1025 ——
    //      **但那时只收到了 990 行** ⇒ 输出 990 行。
    //  两个"各自正确"的逻辑互相抵消了。
    //
    //  修法：**另设一个只数"已输出行数"的计数器**，输出窗口按它来：
    //      v_cnt 仍然保持"传感器行号"（行相位靠它）
    //      S_out_cnt 只数输出行数（总行数靠它）
    //  ⇒ 两者职责分开，互不干扰。
    //  ★★★ 2026-10-14：**从 1024 改回 1080**
    //
    //  为什么现在可以改回来：补行（见文件末尾 S_rep_* 那一段）让每帧**真的**产出
    //    约 1117 行（= 实收 ~1050 + 补 ~67），足以填满 1080，不必再靠"少给行数"来
    //    避开 ui_dbuf 的读写赛跑。
    //  为什么必须一起改：**`W_YSIZE == OUT_ROWS == R_YSIZE` 是不变量**
    //    （ui_dbuf 写侧一帧只写 W_YSIZE 行、读侧只读 R_YSIZE 行；读得比写得多
    //     就会读到 DDR 上**从没写过的地址**，那是真乱码，不是"保持上一行"）。
    localparam [15:0] OUT_ROWS = 16'd1080;

    reg [15:0] S_out_cnt;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n || S_frame_start_ok)              S_out_cnt <= 16'd0;
        // Count completed output rows so the first enabled row cannot escape the 1080-row limit.
        else if(O_raw_tlast)                           S_out_cnt <= S_out_cnt + 16'd1;  // 仅统计实际送出的完整行
    end

    //=========================================================================
    //  ★★★ 2026-10-14 深夜：**帧首相位** —— 修"几帧之间出现一张全屏粉/绿的帧"
    //
    //  现象（上板实测）：画面几何完全正常、也没有彩条，但**每隔几帧整帧变成
    //      粉紫色或粉绿色**（同一场景的两种相位）。这就是**整帧 R/B 反相**。
    //
    //  机理：
    //    去马赛克的 `bayer_ypos = !v_cnt[0]`（它自己的行计数器）假定
    //      **它的第 0 行 = 传感器的奇数行（R/G 行）**。
    //    而无损情况下，本模块吐出的第一行正好是**传感器第 1 行**（奇数）✓。
    //
    //    **但只要第 1 行（或第 0 行）丢了**：S_vstep 会让 v_cnt 从 1 直接跳到 3
    //      （跳过 2），于是 V_valid / tuser 都落在 v_cnt=3 那一行 ——
    //      而那是**传感器第 2 行（偶数）** ⇒ **第一行就差了 1 个相位
    //      ⇒ 整帧每一行都错 ⇒ 全屏粉/绿。**
    //    概率：每帧约 21 个异常包分布在 1103 行里，落在最前 2 行的概率约 3.8%，
    //      30fps 下每秒约一次 —— 与实测"几帧之间出现一次"完全吻合。
    //
    //  修法：**输出必须从偶数的 v_cnt 开始。**
    //    若第 1 行丢了（v_cnt 跳到 3，奇数），就把第 2 行也跳过，等 v_cnt 到 4
    //    再开始 ⇒ 第一行变成传感器第 3 行（奇数）⇒ **相位正确** ✓。
    //    代价：画面整体下移 1 行 —— 看不出来，但比整帧变色好得多。
    //
    //  为什么不去"补"第 1 行：那个位置需要的是一个**奇数行的替身**，
    //    而此刻我们手上只有第 0 行（偶数）—— 补上去奇偶还是错的。
    //    ⇒ 帧首这一次只能靠"跳过一行"把相位对齐，这也是唯一可行的办法。
    //=========================================================================
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n || S_frame_start_ok) S_started <= 1'b0;
        else if((v_cnt > 1) && (v_cnt[0] == 1'b0)) S_started <= 1'b1;
    end

    assign V_valid = S_started && (S_out_cnt < OUT_ROWS);

    //=========================================================================
    //  ★★★ 2026-10-14：**漏行补齐（replay）—— 彩条的最终修复**
    //
    //  【为什么是这个修法】2026-10-14 本机仿真坐实（tb_isp_chain case 8）：
    //      ui_raw_matrix_3x3_buffer.v:293-298  v_cnt 每个输出行 +1，**只被 tuser 复位**
    //      ui_raw_matrix_3x3_buffer.v:145      bayer_ypos = !v_cnt[0] & ch_valid
    //      ui_bilinear_interpolation.v:316-329 bayer_ypos 直接选 R/B 的插值权重
    //  ⇒ bayer_ypos 完全由「tuser 之后**实际输出了多少行**」决定，
    //    与上面那个 v_cnt / S_vstep **毫无关系**（S_vstep 只改变"哪几行被输出"，
    //    不改变"总共输出多少行"；自从 V_valid 改用 S_out_cnt 之后它连前者也不影响了）。
    //  ⇒ 上游每丢一行，下游就少数一行 ⇒ **从那一刻起 bayer_ypos 全部反相**
    //    ⇒ R/B 插值权重整片搞反 ⇒ 满屏横向彩条；丢点位置每帧不同 ⇒ 彩条跳来跳去。
    //  仿真证据：丢第 500 行 ⇒ 第 501 行翻转，此后 525/925 次采样全部偏离基准；
    //            对照组（不丢行）零翻转。
    //
    //  ⇒ 唯一能把相位拨回去的办法：**真的多输出一行数据**。
    //    补行号没用（下游不读它），改 ui_dbuf 的读写指针更没用（伤害发生在它上游）。
    //
    //  【怎么补】把"刚输出完的那一行"同时写进行缓存；当行间隔越过阈值时，
    //    趁空隙把这一行**原样重放一遍**（480 拍，tvalid 连续、tlast 落在第 480 拍）。
    //    重放出来的行对下游就是一行普通数据 ⇒ 下游行计数跟着 +1 ⇒ 相位复位。
    //
    //  【时序余量】正常行间隔 1084 拍，连丢 N 行 ⇒ (N+1)*1084 拍。
    //    第 j 次补行触发点 = 1600 + 1100*(j-1)，占 481 拍；
    //    要求 1600 + 1100*(N-1) + 481 <= 1084*(N+1)  ⇒  N <= 6。
    //    实测 line_max 一直是 3 倍间隔（连丢 2 行），余量充足，故 S_rep_num 上限取 7。
    //
    //  【行缓存】480 字 x 40bit x 2 个分区（乒乓），写成简单双口让 TD 推断 BRAM。
    //    写：随真实行输出同步写入当前分区。
    //    读：补行时读"刚写完的那个分区"（S_rbuf 在触发瞬间锁存 S_wsel）。
    //    两个分区保证"补行还没读完、下一行已经开写"这个边界也安全。
    //=========================================================================
    localparam [8:0]  REP_LAST  = 9'd479;    // 补行最后一拍（tvalid 0..479 共 480 拍）

    //  ★★★ 2026-10-14 晚：**触发点不再用一组固定门限，改成从"行间隔"直接推算。**
    //
    //  为什么改（上板实测的教训）：
    //      固定的门限阶梯（1600 / 2700 / 3800 ...）只能覆盖"门限所列出的那几种间隔"。
    //      而且原来的 S_rep_arm 用 I_raw_frame_end 解除使能 —— 而每帧有约 19 个包的
    //      包头字节被 PHY 采错（S_mon_head_bad=19），**其中若有谁被采成 0x01 就是"帧结束"**，
    //      会在帧中间把补行使能关掉 ⇒ 那之后的第一次漏行就补不上 ⇒
    //      **相位从此翻一次 ⇒ 画面上就是一条粗彩条**。实测正是"只剩几条粗条纹"。
    //
    //  新算法（自洽，不需要标定）：
    //      行间隔 = (漏行数 + 1) x 1083
    //      step   = round(行间隔 / 1083) ~= (S_row_gap + 542) >> 10
    //      ⇒ 此刻「本间隔内本该已经补了 step-1 行」
    //      ⇒ 欠账 S_owed = max(0, (step-1) - S_rep_num)
    //  用 1024 近似 1083：1083 -> (1625)>>10 = 1、2166 -> 2708>>10 = 2、3249 -> 3791>>10 = 3，
    //  判定边界在 1506 / 2530 / 3554，离真值（1083/2166/3249）都有 400 拍以上余量。
    //  (gap + 542) >> 10 ≈ (gap + 1H) / 1024 ≈ (gap + 1H) / 2H
    //    真值 1084 / 2168 / 3252 -> 1 / 2 / 3；判定边界 1506 / 2530 / 3554，余量 400+ 拍
    wire [15:0] S_gap_step = (S_row_gap + 16'd542) >> 10;
    wire [15:0] S_gap_need = (S_gap_step > 16'd1) ? (S_gap_step - 16'd1) : 16'd0;
    wire [15:0] S_owed     = (S_gap_need > {12'd0, S_rep_num})
                           ? (S_gap_need - {12'd0, S_rep_num}) : 16'd0;

    assign S_replaying = (S_rms == 2'd1);

    //  帧内使能：**只在帧首关闭**，帧首那次 last 之后一直开着。
    //  ★ 2026-10-14：**不再用 I_raw_frame_end 解除** —— 见上面那段说明，
    //    包头被采错的包有可能被当成 FE，那会在帧中间把补行关掉。
    //    帧尾那段场消隐改用"间隔上限"排除（见 S_rep_go 里的 S_row_gap < 9000）。
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)                  S_rep_arm <= 1'b0;
        else if(S_frame_start_ok)     S_rep_arm <= 1'b0;
        else if(last && (v_cnt != 0)) S_rep_arm <= 1'b1;   // 帧首那次 last 之后才允许
    end

    assign S_rep_go = (S_rms == 2'd0)
                   && (S_owed != 16'd0)
                   && (S_row_gap > 16'd1300)    // 上一行的数据先走完
                   && (S_row_gap < 16'd9000)    // 超过约 8 个行周期就是场消隐，不是漏行
                   && (S_rep_num < 4'd7)        // 一次间隔最多补 7 行
                   && (S_out_cnt < OUT_ROWS)
                   && S_rep_arm
                   && S_user_sent               // 帧起始一定已经发过（不许抢在 tuser 之前补）
                   && (~last);                  // last 那一拍 S_row_gap 还是旧值，必须排除

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_rms     <= 2'd0;
            S_rep_cnt <= 9'd0;
            S_rbuf    <= 1'b0;
            S_rep_num <= 4'd0;
        end
        else begin
            case(S_rms)
            2'd0: begin                              // 空闲
                S_rep_cnt <= 9'd0;
                if(S_rep_go) begin
                    S_rms     <= 2'd1;
                    //  ★★★ 2026-10-14 深夜：**补的是 last-1 行，不是 last 行。**
                    //
                    //  为什么：被补的那个空位是**奇数行/偶数行**已经定死了 ——
                    //      丢的是 last+1 行，它的奇偶**必然与 last 相反**。
                    //    而 bayer_ypos 是按"这一行应该是奇还是偶"来取 R/B 插值权重的，
                    //    所以拿 last 行（同奇偶）去补 ⇒ **那一行权重必反 ⇒ 一条细彩线**
                    //    （实测正好是 21 条红/蓝细横线 = S_mon_pad_cnt = 21）。
                    //    拿 last-1 行（**相反奇偶**）去补 ⇒ 权重正确 ⇒ 看不出来 ✓
                    //
                    //  为什么不需要额外存储：行缓存本来就是乒乓两个分区 ——
                    //    `S_wsel` 是 last 行所在的分区，`~S_wsel` 就是 last-1 行所在的分区，
                    //    而它此刻还没被覆盖（下一行还没来）。**白拿。**
                    //
                    //  `^ S_rep_num[0]`：同一个间隔里连补多行时要交替取 last-1 / last，
                    //    这样每一行补上去的奇偶都对（连丢 N 行时第 j 个补 last-N+j 行）。
                    S_rbuf    <= ~S_wsel ^ S_rep_num[0];
                    S_rep_num <= S_rep_num + 4'd1;
                end
            end
            2'd1: begin                              // 重放中
                if(S_rep_cnt == REP_LAST) S_rms <= 2'd0;
                S_rep_cnt <= S_rep_cnt + 9'd1;
            end
            default: S_rms <= 2'd0;
            endcase

            //  行首：本间隔的补行计数归零（欠账由上面的 S_owed 每拍重算，不需要额外状态）
            if(last) S_rep_num <= 4'd0;
        end
    end

    //-------------------------------------------------------------------------
    //  补行计数（长期可观测量，每帧在帧起始处清零）
    //-------------------------------------------------------------------------
    reg [15:0] S_pad_cnt  = 16'd0;
    reg [15:0] S_miss_cnt = 16'd0;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n || S_frame_start_ok) begin
            S_pad_cnt  <= 16'd0;
            S_miss_cnt <= 16'd0;
        end
        else begin
            if(S_rep_go) S_pad_cnt <= S_pad_cnt + 16'd1;
            //  行首结算：这个间隔里"本该补"的减去"实际补的"
            if(last && (S_gap_need > {12'd0, S_rep_num}))
                S_miss_cnt <= S_miss_cnt + (S_gap_need - {12'd0, S_rep_num});
        end
    end

    assign O_pad_cnt  = S_pad_cnt;
    assign O_miss_cnt = S_miss_cnt;

    //-------------------------------------------------------------------------
    //  行间隔极值（每帧清零；帧首那次 last 的间隔是场消隐，排除）
    //-------------------------------------------------------------------------
    reg [15:0] S_gap_max = 16'd0;
    reg [15:0] S_gap_min = 16'hFFFF;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n || S_frame_start_ok) begin
            S_gap_max <= 16'd0;
            S_gap_min <= 16'hFFFF;
        end
        else if(last && (v_cnt != 0)) begin
            if(S_row_gap > S_gap_max) S_gap_max <= S_row_gap;
            if(S_row_gap < S_gap_min) S_gap_min <= S_row_gap;
        end
    end

    assign O_gap_max = S_gap_max;
    assign O_gap_min = S_gap_min;

    //  写分区：每行头翻转（本行写进新分区，上一行留在旧分区里供补行读）
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n || S_frame_start_ok) S_wsel <= 1'b0;
        else if(last)                    S_wsel <= ~S_wsel;
    end

    //  行缓存写：与真实行的输出完全同步（写进去的就是发出去的那 480 个字）
    assign S_mem_we    = I_raw_tvalid_d && V_valid && H_valid;
    assign S_mem_waddr = {S_wsel, (h_cnt[8:0] - 9'd1)};

    //  行缓存读：补行期间顺序读（B 口是**异步读**，地址与输出同拍）
    assign S_mem_raddr = {S_rbuf, S_rep_cnt};

    //  行缓存本体：**显式例化安路 BRAM 原语**，与工程里 ui_ram_demosaic /
    //  ui_ram_zhenghe / ui_ram_awb_delay_signal 用的是同一个原语。
    //  ★ 为什么不靠"让 TD 自动推断"：这个工程里所有 RAM 都是显式原语；
    //    1024x40bit = 40960 bit，万一被推断成分布式 RAM 会吃掉四万多个查找表。
    //  MODE = "DP"（真双口）：A 口只写、B 口只读，两个地址互不相干。
    //  REGMODE_B = "NOREG"：B 口异步读 —— 与厂商去马赛克的用法一致（已验证）。
    PH1_LOGIC_ERAM #(
        .DATA_WIDTH_A          (DATA_WIDTH       ),
        .DATA_WIDTH_B          (DATA_WIDTH       ),
        .ADDR_WIDTH_A          (MEM_AW           ),
        .ADDR_WIDTH_B          (MEM_AW           ),
        .DATA_DEPTH_A          (MEM_DEPTH        ),
        .DATA_DEPTH_B          (MEM_DEPTH        ),
        .MODE                  ("DP"             ),
        .REGMODE_A             ("NOREG"          ),
        .REGMODE_B             ("NOREG"          ),
        .WRITEMODE_A           ("NORMAL"         ),
        .WRITEMODE_B           ("READBEFOREWRITE"),
        .IMPLEMENT             ("20K(FAST)"      ),
        .ECC_ENCODE            ("DISABLE"        ),
        .ECC_DECODE            ("DISABLE"        ),
        .CLKMODE               ("SYNC"           ),
        .SSROVERCE             ("DISABLE"        ),
        .OREGSET_A             ("RESET"          ),
        .OREGSET_B             ("RESET"          ),
        .RESETMODE_A           ("ASYNC"          ),
        .RESETMODE_B           ("ASYNC"          ),
        .ASYNC_RESET_RELEASE_A ("ASYNC"          ),
        .ASYNC_RESET_RELEASE_B ("ASYNC"          ),
        .INIT_FILE             ("NONE"           ),
        .FILL_ALL              ("NONE"           )
    ) S_line_mem (
        .dia            (I_raw_tdata_r      ),
        .dib            ({DATA_WIDTH{1'b0}} ),
        .addra          (S_mem_waddr        ),
        .addrb          (S_mem_raddr        ),
        .cea            (1'b1               ),
        .ceb            (1'b1               ),
        .ocea           (1'b0               ),
        .oceb           (1'b0               ),
        .clka           (I_clk              ),
        .clkb           (I_clk              ),
        .wea            (S_mem_we           ),
        .web            (1'b0               ),
        .bea            (1'b0               ),
        .beb            (1'b0               ),
        .rsta           (1'b0               ),
        .rstb           (1'b0               ),
        .doa            (                   ),
        .dob            (S_rep_data         ),
        .ecc_sbiterr    (                   ),
        .ecc_dbiterr    (                   ),
        .ecc_sbiterrinj ('d0                ),
        .ecc_dbiterrinj ('d0                )
    );

    // 计算每行像素个数
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n || S_frame_start_ok)
            h_cnt <= 0;
        else if(last)
            h_cnt <= 0;
        else if(I_raw_tvalid_d)
            h_cnt <=  h_cnt + 1;
    end


//    cwc2 cwc2_inst
//  (
//  .probe0  (v_cnt       ),
//  .probe1  (h_cnt      ),
//  .probe2  (tuser  ),
//  .probe3  (cnt    ),
//  .probe4  (O_raw_tlast        ),
//  .probe5  (O_raw_tvalid         ),
//  .probe6  (O_raw_tdata ),
//  .clk(I_clk)
//  );
endmodule
