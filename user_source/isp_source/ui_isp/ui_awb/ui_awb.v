/*****************************************************************
Company : MiLianKe Electronic Technology Co., Ltd.
WebSite:https://www.milianke.com
TechWeb:https://www.uisrc.com
tmall-shop:https://milianke.tmall.com
jd-shop:https://milianke.jd.com
taobao-shop: https://milianke.taobao.com
Description: 
The reference demo provided by Milianke is only used for learning. 
We cannot ensure that the demo itself is free of bugs, so users 
should be responsible for the technical problems and consequences
caused by the use of their own products.
@Author      :   XiaoQingquan 
@Time        :   2024/10 
version:     :   
@Description :   在原基础上删去了一些赘余的代码，并且优化了时序，加入BRAM IP核，
                 让从除法器出来的每一个数据都可以对上相应的像素数据进行增益，使
                 得它的适用行更加广泛了。
*****************************************************************/
module ui_awb  #(   
    parameter IMG_HEIGHT = 1080,
    parameter IMG_WIDTH  = 1920
) 
(
    input                   I_clk  ,
    input                   I_rst_n,


    input                   I_tlast  ,
    input                   I_tuser  ,
    input [95:0]            I_tdata  ,
    input                   I_tvalid , 
    output                  I_tready ,

    output                  O_tlast  ,
    output                  O_tuser  ,
    output [95:0]           O_tdata  ,
    output                  O_tvalid ,
    input                   O_tready
);


/*****************************************************************
                          中间信号                                        
*****************************************************************/

reg         I_tvalid_r0,I_tvalid_r1;          // 输入有效信号的寄存
reg  [95:0] I_tdata_d0,I_tdata_d1,I_tdata_d2; // 输入数据的寄存

// 累加RGB通道的数值，用于计算平均值
reg  [31:0]         sum_r;
reg  [31:0]         sum_g;
reg  [31:0]         sum_b;

wire  [95:0]        data_d0 ;     // 延时后的数据
wire                valid_d0;     // 延时后的有效信号
wire                I_tuser_r0;   // 延时后的帧开始信号

reg                 I_tuser_r1;   // 帧开始信号的寄存

reg   [95:0]        data_d1 ;     // 再次延时后的数据
reg                 valid_d1;     // 再次延时后的有效信号
reg   [9:0]         rgb_sum;//每个像素点RGB的和
localparam IMG_WIDTH_4x = (IMG_WIDTH >> 2);  // 图像宽度除以4，得到每个数据块的数量

/*****************************************************************
                           平均值计算                                        
*****************************************************************/

// 对输入数据和有效信号进行寄存，形成流水线
always @(posedge I_clk or negedge I_rst_n) begin
    if (!I_rst_n) begin
        I_tdata_d0  <= 0;
        I_tdata_d1  <= 0;
        I_tdata_d2  <= 0;
        {I_tvalid_r0,I_tvalid_r1} <= 0;
    end else begin
        I_tdata_d0  <= I_tdata   ;
        I_tdata_d1  <= I_tdata_d0;
        I_tdata_d2  <= I_tdata_d1;
        I_tvalid_r0 <= I_tvalid;
        I_tvalid_r1 <= I_tvalid_r0;
    end
end


// 将输入数据拆分为4个像素的RGB分量
wire [7:0] rgb888_r_d;
wire [7:0] rgb888_g_d;
wire [7:0] rgb888_b_d;

assign  rgb888_r_d = I_tdata[16+:8];
assign  rgb888_g_d = I_tdata[8+:8];
assign  rgb888_b_d = I_tdata[0+:8];


// 将输入数据拆分为4个像素的RGB分量
wire [7:0] rgb888_r_d0;
wire [7:0] rgb888_g_d0;
wire [7:0] rgb888_b_d0;

assign  rgb888_r_d0 = I_tdata_d0[16+:8];
assign  rgb888_g_d0 = I_tdata_d0[8+:8];
assign  rgb888_b_d0 = I_tdata_d0[0+:8];

//  【本工程添加】R/B 增益的声明**前移到这里** —— 因为下面"迭代白平衡"的统计量
//  需要在**校正后**的像素上做判断，要用到这两个寄存器的当前值。
//  ModelSim 的 vlog 要求先声明后使用（厂商原码把这些声明放在很后面）。
reg  [19:0]   in_r_r1 = 20'h10000;
reg  [19:0]   in_b_r1 = 20'h10000;

//=============================================================================
//  ★★★ 2026-10-16：**白平衡统计窗口 —— 只统计"本来就是灰色"的像素**
//
//  【为什么改】上板实测两组数（这是决定性的）：
//      白墙（应该补 ~1.0）        ⇒ `gain_r` = **1.075**  ✓ 完全正确
//      房间场景（补完却一片品红）⇒ `gain_r` = **1.732**  ✗ 补过头了
//  ⇒ **`ui_awb` 本身没坏**（白墙那组说明它会算对），坏的是**判据**：
//
//      `in_r_r1 = sum_g / sum_r` 用的是**整幅画面所有像素的平均值** ——
//      这就是"灰色世界"假设：**它假定画面的平均色应该是灰的**。
//      可真实场景里往往有绿色的东西（灯光、植物、屏幕、衣服），
//      于是一补就把**平均色**拉成灰 ⇒ **画面里本来不绿的部分被推成品红** ✓
//
//  【正确做法】白平衡的参照物是**中性色的物体**（白墙、灰纸、灰桌面），
//    不是"整幅画面的平均"。所以只累加**接近中性**的像素：
//        ① 三个通道都没削顶（颜色信息还在）
//        ② 够亮（暗部信噪比差，会把比值往 1 拉）
//        ③ **通道跨度相对亮度足够小**（≈ 接近灰）
//    ③ 用 `跨度 < (最大值 >> 2)` 近似"相对跨度 < 25%"，不需要除法器。
//
//  ★ 白墙那组数正好是这一条的自检：白墙上几乎全部像素都接近中性
//    ⇒ 判据不变 ⇒ 增益应当仍然算出 ~1.07 ✓
//
//  ★★ 但"在原始图上挑灰像素"仍有自指偏差（见下一条修正），所以这一版的
//      `S_awb_max/min/span/use` 全部改到**校正后**的像素上算 —— 下面的最小值/跨度
//      计算已挪到 `S_awb_c_r/g/b` 之后。
//=============================================================================
//=============================================================================
//  ★★★ 2026-10-16 晚第三次修正：**迭代白平衡 —— 统计量改到"已校正的图"上算**
//
//  【为什么必须改】前两版都是在**原始图**上挑中性像素，那有**自指偏差**：
//      画面本来就偏绿时，真正中性的白墙此刻看起来并不中性，反而被筛掉
//      ⇒ 样本只剩"碰巧此刻就是灰"的一小撮（实测 2.8% = 34054/120万），不代表整体
//      ⇒ 算出 `gain_r` = 1.14，而画面实际需要的要大得多 ⇒ 补不够 ⇒ 还是绿的。
//
//  【正确做法】对**增益校正之后**的像素来判"是不是中性"、来求比值：
//      设真实需要的增益是 k、当前增益是 g，
//      校正后 `sum_r' = g·ΣR`、`sum_g' = ΣG`
//      ⇒ 测出的比值 = ΣG/(g·ΣR) = **k/g**
//
//      于是增益更新必须改成**相乘**而不是"赋值"：
//          赋值：g ← k/g      ⇒ 不动点在 g = √k   ✗ 只能走到一半
//          **相乘：g ← g·(k/g) = k**              ✓ **一步就是正确答案**
//      ⇒ 加上限速之后，增益会平滑收敛到 k 并停住 ✓
//
//  【为什么不会振荡】统计是**每帧**做一次，而限速让增益一帧最多动 1/48（<2%），
//      所以一帧之内增益几乎是常数 ⇒ 这是个阻尼反馈环 ✓
//
//  【自检】灰度场景（R=G=B）下：比值 = 1 ⇒ 增益 ×1 不变 ⇒ 仍然停在 1.0 ✓
//=============================================================================
//  （`in_r_r1` / `in_b_r1` 的声明已前移到文件开头的声明区，见那里的说明）
wire [27:0] S_awb_r_prod = in_r_r1 * rgb888_r_d0;      // 20bit(Q16) x 8bit
wire [27:0] S_awb_b_prod = in_b_r1 * rgb888_b_d0;
wire [7:0]  S_awb_c_r = S_awb_r_prod[27:24] ? 8'hFF : S_awb_r_prod[23:16]; // 与输出级同样削顶
wire [7:0]  S_awb_c_b = S_awb_b_prod[27:24] ? 8'hFF : S_awb_b_prod[23:16];
wire [7:0]  S_awb_c_g = rgb888_g_d0;

wire [7:0] S_awb_max = (S_awb_c_r > S_awb_c_g)
                     ? ((S_awb_c_r > S_awb_c_b) ? S_awb_c_r : S_awb_c_b)
                     : ((S_awb_c_g > S_awb_c_b) ? S_awb_c_g : S_awb_c_b);
wire [7:0] S_awb_min = (S_awb_c_r < S_awb_c_g)
                     ? ((S_awb_c_r < S_awb_c_b) ? S_awb_c_r : S_awb_c_b)
                     : ((S_awb_c_g < S_awb_c_b) ? S_awb_c_g : S_awb_c_b);
wire [7:0] S_awb_span = S_awb_max - S_awb_min;

//  ① 校正后没有削顶（削顶了颜色信息就没了）
//  ② 够亮（暗部信噪比差，会把比值往 1 拉）
//  ③ **接近中性**：通道跨度 < 亮度的一半
wire S_awb_use = (S_awb_c_r != 8'hFF) &&
                 (S_awb_c_g != 8'hFF) &&
                 (S_awb_c_b != 8'hFF) &&
                 (S_awb_max > 8'd40) &&
                 (S_awb_span < (S_awb_max >> 1));



    ///削弱白色
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            rgb_sum <= 0;
        end
        else if(I_tvalid)begin
            rgb_sum <= rgb888_r_d + rgb888_g_d + rgb888_b_d;
        end
    end
    
    wire flag_frame_r;
    reg [31:0] sum_r_d ;
    reg [31:0] sum_g_d ;
    reg [31:0] sum_b_d ;
    //=========================================================================
    //  ★★★ 2026-10-16：**统计量改成"每帧重新累加"**
    //
    //  厂商原码是一对累加器 + 一个 5 帧计数器：
    //      `sum_r_d` 自己累加，`frame_cnt == 4` 时复位；
    //      `sum_r` 在 `flag_frame_r`（frame_cnt 变 1）那一刻**被 sum_r_d 整体覆盖**，
    //              其余时间自己累加。
    //  **两个累加器的累加窗口不同 ⇒ 它们"总和的比值"本来就不一样 ⇒
    //    每次覆盖那一刻，`sum_g/sum_r` 会整体跳一下。**
    //
    //  上板实测（决定性证据）：`S_mon_awb_step`（帧间增益跳变量的历史最大值）
    //      = **44041 / 49124 = 0.67~0.75 倍** —— 增益每隔几帧整整跳 0.7 倍。
    //      R、B 同时被抬高 ~1.5 倍 ⇒ 整帧偏品红；掉回 ~1.0 ⇒ 整帧偏绿
    //      ⇒ **就是"粉绿屏闪"** ✓
    //
    //  改成每帧重新累加：一帧约 120 万个像素，32 位累加器完全够（不会溢出），
    //    比值是"整个上一帧"的统计，本身就稳 ✓
    //  （`sum_r_d` / `frame_cnt` / `flag_frame_r` 那一段保留但**已不再参与计算**。）
    //=========================================================================
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n || I_tuser) begin          // ★ 每帧重新开始累加
            sum_r <= 1;
            sum_g <= 1;
            sum_b <= 1;
        end else if (I_tvalid_r0 && S_awb_use) begin
            //  ★ 2026-10-16：累加的是**增益校正后**的值（迭代白平衡的关键，见上面那段说明）
            sum_r <= sum_r + S_awb_c_r;
            sum_g <= sum_g + S_awb_c_g;
            sum_b <= sum_b + S_awb_c_b;
        end
    end

    reg [3:0]  frame_cnt;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n ) begin
            sum_r_d <= 1;
            sum_g_d <= 1;
            sum_b_d <= 1;
        end else if(frame_cnt==4)begin
            sum_r_d <= 1;
            sum_g_d <= 1;
            sum_b_d <= 1;
        end else if(I_tvalid_r0 && S_awb_use)begin
            sum_r_d <= sum_r_d + rgb888_r_d0;
            sum_g_d <= sum_g_d + rgb888_g_d0;
            sum_b_d <= sum_b_d + rgb888_b_d0;
        end 
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n ) begin
            frame_cnt <= 0;
        end else if(frame_cnt==4)begin
            frame_cnt <= 0;
        end else if(I_tuser)begin
            frame_cnt <= frame_cnt + 1;
        end
    end

    reg  flag_frame;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n ) begin
            flag_frame <= 0;
        end else if(frame_cnt==1)begin
            flag_frame <= 1;
        end else begin
            flag_frame <= 0;
        end
    end


    reg  flag_frame_d;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n ) begin
            flag_frame_d <= 0;
        end else begin
            flag_frame_d <= flag_frame;
        end
    end

    assign flag_frame_r = (flag_frame == 1) && (flag_frame_d == 0);



  /*****************************************************************
                           计算增益                                       
*****************************************************************/

  wire [47:0] dout_R;
  wire [47:0] dout_B;

  wire done;

  wire [47:0] in_r_x1;
  wire [47:0] in_b_x1;
  assign in_r_x1 = dout_R[47:0];
  assign in_b_x1 = dout_B[47:0];

    

/*****************************************************************
                   IP 调用                                    
*****************************************************************/
    //为了保证合理的延迟，这里使用了除法器


      divider div_gen_G_R_r1 (
          .clk(I_clk),  // input wire aclk
          .rst(!I_rst_n),
          .start(I_tvalid_r1),
          .denominator(sum_r),  // input wire [31 : 0] s_axis_divisor_tdata
          .numerator({sum_g,16'd0}),  // input wire [47 : 0] s_axis_dividend_tdata
          .quotient(dout_R),  // output wire [79 : 0] m_axis_dout_tdata
          .done(done)
      );

      divider divide_G_B_r1 (
          .clk(I_clk),  // input wire aclk
          .rst(!I_rst_n),
          .start(I_tvalid_r1),
          .denominator(sum_b),  // input wire [31 : 0] s_axis_divisor_tdata
          .numerator({sum_g,16'd0}),  // input wire [47 : 0] s_axis_dividend_tdata
          .quotient(dout_B),  // output wire [79 : 0] m_axis_dout_tdata
          .done()
      );

     
    ui_signal_delay #(
        .IMG_HEIGHT   (IMG_HEIGHT),
        .IMG_WIDTH    (IMG_WIDTH ),
        .DATA_WIDTH   (96  ),
        .DELAY_CYCLE  (50)
    )X_ui_signal_delay_d (
        /*input                       */ .I_clk  (I_clk  ),
        /*input                       */ .I_rst_n(I_rst_n),
        /*input                       */ .I_tuser(I_tuser),  //synthesis keep 
        /*input  wire                 */ .I_valid(I_tvalid_r1),  //synthesis keep 
        /*input  wire [DATA_WIDTH-1:0]*/ .I_data (I_tdata_d1 ),   //synthesis keep 
        /*output wire                 */ .O_valid(valid_d0   ),  //synthesis keep 
        /*output wire [DATA_WIDTH-1:0]*/ .O_data (data_d0    ),   //synthesis keep 
        /*output wire                 */ .O_tuser(I_tuser_r0 )
    );

    

/*****************************************************************
                            ????????                                      
*****************************************************************/

//  `in_r_r1` / `in_b_r1` 的声明已前移到文件开头的声明区（见那里的【本工程添加】说明）


// 计算增益，防止增益过大溢出，限制在0xFFFFF
//  ★★★ 2026-10-15：**原来的条件里带 `|| I_tuser`，每帧都把增益清零回 1.0 —— 删掉。**
//
//  为什么（上板实测"每隔几帧整帧变粉/变绿"的根因）：
//      `in_r_r1 = sum_g / sum_r`，而这里接的除法器是用 `I_tvalid_r1` **每拍启动一次**
//      （见下面 div_gen_G_R_r1），也就是说增益是**跟着累计和持续收敛**的，
//      并不是"一帧只算一次"。
//      可原来每个 `I_tuser` 都把它清回 `20'h10000` = **1.0** ⇒
//          增益 = 1.0            ⇒ **没做白平衡 ⇒ 整帧偏绿**（Bayer 原始色就是偏绿的）
//          增益 = 收敛后的比值   ⇒ **整帧偏品红**
//      ⇒ **两种颜色交替出现 = 整帧的粉/绿闪。**
//      而且 `sum_r/sum_g` 是跨 5 帧（frame_cnt 0..4）才清一次的，
//      累计和很大 ⇒ 比值本身变化很慢 ⇒ 只要不清零，增益在一帧之内几乎不变 ✓
//
//  删掉 `|| I_tuser` 之后：增益变成"持续的运行估计"，上电后很快收敛并稳定下来，
//      不再有帧周期的往复摆动 ✓
//=============================================================================
//  ★★★ 2026-10-16：**增益改成"限速跟踪"**
//
//  为什么光修统计量还不够：统计量再稳，只要它偶尔抖一下，增益就跟着整帧跳。
//    而白平衡增益本来就该是**慢变**的（它反映场景光照，不会帧间突变）。
//
//  做法：每个 `I_tuser` 采一次目标值（此刻 `sum_r/sum_g` 正好累计完**上一整帧**），
//    增益每帧最多朝目标挪 `SLEW_STEP`，够近就直接到位。
//    ⇒ 从 1.0 走到 1.6 要约 29 帧（≈1 秒），肉眼看不出来 ✓
//    ⇒ **无论统计量多吵，画面都不会闪** ✓
//
//  ★ 这是"设计"而不是"掩盖症状"：AWB 增益的更新率本来就该被限住。
//=============================================================================
localparam [19:0] SLEW_STEP = 20'd1365;   // ≈ 0x10000 的 1/48 ≈ 每帧 0.021 倍

//=============================================================================
//  ★★★ 2026-10-16 **调试开关：白平衡旁路**
//
//  = 1 时把 R/B 增益钉死成 1.0（完全不做白平衡），用来一刀切开两件事：
//      * 若画面变成"稳定但偏绿"的原始色  ⇒ 上游（去马赛克/BLC/齿轮箱）是好的，
//        AWB 在"补过头"，问题在 AWB 的判据/增益上
//      * 若画面仍然是品红 / 仍在闪    ⇒ 问题在 AWB **上游**，得往去马赛克那边查
//
//  判据同时看读数：旁路时 `S_mon_awb_step` 应当恒为 0（增益不动 ⇒ 不闪）。
//
//  ★ 2026-10-16 复现结论：**旁路实验已经把 AWB 从"屏闪"里洗清了**（帧首丢行才是元凶，
//     已修好）。现在改为查**颜色**：白墙实测原始输出只"略带绿"（G/R ≈ 1.1），
//     而旁路前在房间场景读到 `gain_r` = 1.89 —— 差太多，必须分开是哪种：
//         白墙上增益 ≈ 1.0~1.1  ⇒ AWB 正确，是"灰色世界假设在彩色场景失效" ⇒ 换判据
//         白墙上增益仍 ≈ 1.9    ⇒ AWB 的统计量本身坏了 ⇒ 查 ui_awb 内部
//  ⇒ **本版置 0（启用 AWB）**，用白墙场景读增益来分辨。
//=============================================================================

reg [19:0] S_tgt_r = 20'h10000;
reg [19:0] S_tgt_b = 20'h10000;

wire [19:0] S_div_r_now = |in_r_x1[47:20] ? 20'hfffff : in_r_x1[19:0];
wire [19:0] S_div_b_now = |in_b_x1[47:20] ? 20'hfffff : in_b_x1[19:0];

//  ★ 相乘更新：新增益 = 当前增益 × 测出的比值（两个都是 Q16 定点）
//      Q16 × Q16 = Q32 ⇒ 取 [35:16] 就是 Q16 的结果；
//      [39:36] 非零说明超过 16.0 ⇒ 饱和。
wire [39:0] S_mul_r = in_r_r1 * S_div_r_now;
wire [39:0] S_mul_b = in_b_r1 * S_div_b_now;
wire [19:0] S_new_r = (|S_mul_r[39:36]) ? 20'hFFFFF : S_mul_r[35:16];
wire [19:0] S_new_b = (|S_mul_b[39:36]) ? 20'hFFFFF : S_mul_b[35:16];

//  ★ 样本数守卫：本帧通过筛选的像素太少时**保持上一次的目标** ——
//    否则"这一帧几乎没有中性色"会把增益一路拉回 1.0（画面会突然偏绿）。
reg [31:0] S_awb_n = 32'd0;
always @(posedge I_clk or negedge I_rst_n) begin
    if(!I_rst_n || I_tuser)           S_awb_n <= 32'd0;
    else if(I_tvalid_r0 && S_awb_use) S_awb_n <= S_awb_n + 32'd1;
end


always @(posedge I_clk or negedge I_rst_n) begin
    if(!I_rst_n) begin
        S_tgt_r <= 20'h10000;
        S_tgt_b <= 20'h10000;
    end
    else if(I_tuser) begin            // ★ 每帧采一次目标（此刻是上一帧的完整统计）
        //  ★★★ 2026-10-16：**增益更新改成"相乘"** —— 迭代白平衡的关键一步。
        //    统计量是在"已校正的图"上算的，所以测出的比值 = k/g（k 是正确答案）。
        //      赋值：g ← k/g  ⇒ 不动点在 √k，只能走到一半 ✗
        //      相乘：g ← g·(k/g) = k ⇒ **一步就是正确答案** ✓
        if(S_awb_n > 32'd2000) begin  // 样本够多才更新（2000/120万 ≈ 0.17%，门槛很低）
            S_tgt_r <= S_new_r;
            S_tgt_b <= S_new_b;
        end
    end
end

wire        S_r_up   = (S_tgt_r > in_r_r1);
wire        S_r_dn   = (S_tgt_r < in_r_r1);
wire [19:0] S_r_diff = S_r_up ? (S_tgt_r - in_r_r1) : (in_r_r1 - S_tgt_r);
wire        S_b_up   = (S_tgt_b > in_b_r1);
wire        S_b_dn   = (S_tgt_b < in_b_r1);
wire [19:0] S_b_diff = S_b_up ? (S_tgt_b - in_b_r1) : (in_b_r1 - S_tgt_b);

always @(posedge I_clk or negedge I_rst_n)begin
    if(!I_rst_n)begin
        in_r_r1 <= 20'h10000;
        in_b_r1 <= 20'h10000;
    end
    else if(I_tuser)begin             // ★ 每帧只动一步
        if      (S_r_diff <= SLEW_STEP) in_r_r1 <= S_tgt_r;
        else if (S_r_up)                in_r_r1 <= in_r_r1 + SLEW_STEP;
        else if (S_r_dn)                in_r_r1 <= in_r_r1 - SLEW_STEP;

        if      (S_b_diff <= SLEW_STEP) in_b_r1 <= S_tgt_b;
        else if (S_b_up)                in_b_r1 <= in_b_r1 + SLEW_STEP;
        else if (S_b_dn)                in_b_r1 <= in_b_r1 - SLEW_STEP;
    end
end


    ////信号同步
    always @(posedge I_clk or negedge I_rst_n) begin
      if (!I_rst_n) begin
          valid_d1 <= 0;
          data_d1  <= 0;
          I_tuser_r1 <= 0;
      end else begin
          valid_d1 <= valid_d0;
          data_d1  <= data_d0 ;
          I_tuser_r1 <= I_tuser_r0;
      end
    end
/*****************************************************************
                           ????????                                       
*****************************************************************/

    reg [27:0] R_new_r1;
    reg [27:0] G_new_r1;
    reg [27:0] B_new_r1;

    reg [27:0] R_new_r2;
    reg [27:0] G_new_r2;
    reg [27:0] B_new_r2;

    reg [27:0] R_new_r3;
    reg [27:0] G_new_r3;
    reg [27:0] B_new_r3;

    reg [27:0] R_new_r4;
    reg [27:0] G_new_r4;
    reg [27:0] B_new_r4;


    wire   [7:0] rgb888_r_d1[0:3];
    wire   [7:0] rgb888_g_d1[0:3];
    wire   [7:0] rgb888_b_d1[0:3];

    assign  rgb888_r_d1[0] = data_d1[16+:8];
    assign  rgb888_g_d1[0] = data_d1[8+:8];
    assign  rgb888_b_d1[0] = data_d1[0+:8];

    assign  rgb888_r_d1[1] = data_d1[40+:8];
    assign  rgb888_g_d1[1] = data_d1[32+:8];
    assign  rgb888_b_d1[1] = data_d1[24+:8];

    assign  rgb888_r_d1[2] = data_d1[64+:8];
    assign  rgb888_g_d1[2] = data_d1[56+:8];
    assign  rgb888_b_d1[2] = data_d1[48+:8];

    assign  rgb888_r_d1[3] = data_d1[88+:8];
    assign  rgb888_g_d1[3] = data_d1[80+:8];
    assign  rgb888_b_d1[3] = data_d1[72+:8];
// 计算新的RGB值，应用增益
always @(posedge I_clk or negedge I_rst_n)begin 
    if(!I_rst_n)begin
        R_new_r1 <= 0;
        G_new_r1 <= 0;
        B_new_r1 <= 0;

        R_new_r2 <= 0;
        G_new_r2 <= 0;
        B_new_r2 <= 0;

        R_new_r3 <= 0;
        G_new_r3 <= 0;
        B_new_r3 <= 0;

        R_new_r4 <= 0;
        G_new_r4 <= 0;
        B_new_r4 <= 0;
    end
    else if(valid_d1) begin
        R_new_r1 <= in_r_r1 * rgb888_r_d1[0];
        G_new_r1 <=           rgb888_g_d1[0];
        B_new_r1 <= in_b_r1 * rgb888_b_d1[0];

        R_new_r2 <= in_r_r1 * rgb888_r_d1[1];
        G_new_r2 <=           rgb888_g_d1[1];
        B_new_r2 <= in_b_r1 * rgb888_b_d1[1];

        R_new_r3 <= in_r_r1 * rgb888_r_d1[2];
        G_new_r3 <=           rgb888_g_d1[2];
        B_new_r3 <= in_b_r1 * rgb888_b_d1[2];

        R_new_r4 <= in_r_r1 * rgb888_r_d1[3];
        G_new_r4 <=           rgb888_g_d1[3];
        B_new_r4 <= in_b_r1 * rgb888_b_d1[3];
    end
end

/*****************************************************************
                            ??????????                                      
*****************************************************************/

    wire [7:0] R_new_x1;
    wire [7:0] G_new_x1;
    wire [7:0] B_new_x1;

    wire [7:0] R_new_x2;
    wire [7:0] G_new_x2;
    wire [7:0] B_new_x2;

    wire [7:0] R_new_x3;
    wire [7:0] G_new_x3;
    wire [7:0] B_new_x3;

    wire [7:0] R_new_x4;
    wire [7:0] G_new_x4;
    wire [7:0] B_new_x4;

    assign R_new_x1 = (R_new_r1[27:24])?255:R_new_r1[23:16];
    assign G_new_x1 =  G_new_r1;
    assign B_new_x1 = (B_new_r1[27:24])?255:B_new_r1[23:16];

    assign R_new_x2 = (R_new_r2[27:24])?255:R_new_r2[23:16];
    assign G_new_x2 =  G_new_r2;
    assign B_new_x2 = (B_new_r2[27:24])?255:B_new_r2[23:16];

    assign R_new_x3 = (R_new_r3[27:24])?255:R_new_r3[23:16];
    assign G_new_x3 =  G_new_r3;
    assign B_new_x3 = (B_new_r3[27:24])?255:B_new_r3[23:16];

    assign R_new_x4 = (R_new_r4[27:24])?255:R_new_r4[23:16];
    assign G_new_x4 =  G_new_r4;
    assign B_new_x4 = (B_new_r4[27:24])?255:B_new_r4[23:16];
/*****************************************************************
                           ??????                                       
*****************************************************************/

    reg valid_d2,I_tuser_r2;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)begin
            valid_d2 <= 0;
            I_tuser_r2 <= 0;
        end
        else begin
            valid_d2 <= valid_d1;
            I_tuser_r2 <= I_tuser_r1;
        end
    end


    assign  O_tlast  = (valid_d1 == 0) && (valid_d2 == 1);   
    assign  O_tuser  = I_tuser_r2 ;   
    assign  O_tvalid = valid_d2;

    assign  O_tdata = O_tvalid?{R_new_x4,G_new_x4,B_new_x4,
                       R_new_x3,G_new_x3,B_new_x3,
                       R_new_x2,G_new_x2,B_new_x2,
                       R_new_x1,G_new_x1,B_new_x1}:0;
    // assign O_t_data = {R_new[7-:5],G_new[7-:6],B_new[7-:5]};
    assign  I_tready = O_tready;
 
endmodule



