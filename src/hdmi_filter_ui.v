// 1920x1080 RGB OSD and streaming pencil, emboss, and comic filters.
// K4: mode previous / enter parameter page; parameter -1/-10 / next parameter.
// P5: mode next; parameter +1/+10 / return to mode selection.
// K4+P5 held together for COMBO_MS (400 ms) = one photo request (O_capture_req
// toggles); COMBO_MS must stay below LONG_PRESS_MS.
// Input VS is active-high blanking sync. Config commits at its rising edge.
// RGB, DE, HS, VS have the same ten-cycle latency. Track raw DE, not FIFO valid.
// Chinese glyph ROM is embedded below: no external asset is needed by RTL.
module hdmi_filter_ui #(
    parameter integer CLK_HZ = 150000000,
    parameter integer DEBOUNCE_MS = 10,
    parameter integer LONG_PRESS_MS = 700,
    parameter integer DOUBLE_CLICK_MS = 350,
    parameter integer COMBO_MS = 400
)(
    input wire I_clk, I_rst_n,
    input wire [1:0] I_key_n,
    input wire I_vs, I_hs, I_de, I_pixel_valid,
    input wire [23:0] I_rgb,
    input wire [7:0]  I_fps,          // 相机实测帧率（来自 ui_filter_domain_new.O_fps）
    // ★ 2026-10-30 自动曝光测光值（来自 ui_ae_meter，滤镜域 75MHz，
    //   本模块在 150MHz 域，内部做两级同步。每帧才更新一次，是慢变量。
    input wire [7:0]  I_ae_mean,      // 本帧平均亮度 0..255
    input wire [7:0]  I_ae_gain,      // 当前模拟增益码（用于判断 AE 在不在动）
    input wire [7:0]  I_ae_nack,      // I2C 写被 NACK 的次数（>0 时增益显示红色）
    input wire [11:0] I_delay_ms10,   // 端到端延迟，0.1ms 单位（来自 delay_meter）
    input wire        I_delay_valid,  // 0 = 显示 "--"
    output reg O_vs, O_hs, O_de,
    output reg [23:0] O_rgb,
    output reg [2:0] O_mode,
    output reg [7:0] O_strength, O_param1, O_param2, O_smooth,
    output reg [7:0] O_he_strength,   // 直方图均衡强度 0..100（原图页 focus1）
    output reg O_compare_en,
    output reg [7:0] O_split,
    output reg O_menu_visible,
    output reg [3:0] O_focus,
    output reg O_page,
    output reg O_capture_req      // 两键同按 COMBO_MS 触发一次拍照；电平翻转，每次翻转=一次请求
);
    localparam integer DB_CYCLES = ((CLK_HZ/1000)*DEBOUNCE_MS > 0) ? (CLK_HZ/1000)*DEBOUNCE_MS : 1;
    localparam integer LONG_CYCLES = ((CLK_HZ/1000)*LONG_PRESS_MS > 0) ? (CLK_HZ/1000)*LONG_PRESS_MS : 1;
    localparam integer DOUBLE_CYCLES = ((CLK_HZ/1000)*DOUBLE_CLICK_MS > 0) ? (CLK_HZ/1000)*DOUBLE_CLICK_MS : 1;
    localparam integer DB_BITS = (DB_CYCLES > 1) ? $clog2(DB_CYCLES+1) : 1;
    localparam integer HOLD_BITS = (LONG_CYCLES > 1) ? $clog2(LONG_CYCLES+1) : 1;
    localparam integer DOUBLE_BITS = (DOUBLE_CYCLES > 1) ? $clog2(DOUBLE_CYCLES+1) : 1;
    localparam integer COMBO_CYCLES = ((CLK_HZ/1000)*COMBO_MS > 0) ? (CLK_HZ/1000)*COMBO_MS : 1;
    localparam integer COMBO_BITS = (COMBO_CYCLES > 1) ? $clog2(COMBO_CYCLES+1) : 1;
    reg [1:0] key_meta, key_sync, key_stable, key_prev, long_used;
    reg [1:0] key_short, key_long;
    reg [1:0] key_single_event, key_double_event, key_long_event;
    reg [1:0] click_pending;
    reg [DB_BITS-1:0] debounce_count [0:1];
    reg [HOLD_BITS-1:0] hold_count [0:1];
    reg [DOUBLE_BITS-1:0] click_count [0:1];
    // 两键同按拍照：必须声明在下面按键 always 之前，否则 both_down 会被当成隐式线网。
    reg combo_seen;
    reg [COMBO_BITS-1:0] combo_count;
    reg [1:0] combo_release;
    wire both_down = ~key_stable[0] & ~key_stable[1];
    integer k;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            key_meta <= 2'b11; key_sync <= 2'b11;
            key_stable <= 2'b11; key_prev <= 2'b11;
            key_short <= 0; key_long <= 0; long_used <= 0;
            for (k=0;k<2;k=k+1) begin debounce_count[k]<=0; hold_count[k]<=0; end
        end else begin
            key_meta <= I_key_n; key_sync <= key_meta;
            key_prev <= key_stable; key_short <= 0; key_long <= 0;
            for (k=0;k<2;k=k+1) begin
                if (key_sync[k] == key_stable[k]) debounce_count[k] <= 0;
                else if (debounce_count[k] == DB_CYCLES-1) begin
                    key_stable[k] <= key_sync[k]; debounce_count[k] <= 0;
                end else debounce_count[k] <= debounce_count[k] + 1'b1;
                if (!key_stable[k]) begin
                    // 组合键手势归组合键所有：两键按住期间、以及两键全松之前
                    // （combo_seen）都不计长按。否则"按住K4 -> 再按P5拍照 ->
                    // 松开P5"会让 K4 续接冻结的 hold_count 触发长按、顺带进参数页。
                    if (!long_used[k] && !both_down && !combo_seen) begin
                        if (hold_count[k] == LONG_CYCLES-1) begin
                            key_long[k] <= 1; long_used[k] <= 1;
                        end else hold_count[k] <= hold_count[k] + 1'b1;
                    end
                end else begin
                    // combo_seen 覆盖整段组合键（按下到完全松开 2 拍后），
                    // 松开沿不会漏出短按，也就不会变成 +/-1 或模式切换。
                    if (!key_prev[k] && !long_used[k] && !combo_seen) key_short[k] <= 1;
                    hold_count[k] <= 0; long_used[k] <= 0;
                end
            end
        end
    end

    // A released click is held briefly so a second release can become a double click.
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            key_single_event <= 0; key_double_event <= 0; key_long_event <= 0;
            click_pending <= 0;
            for (k=0;k<2;k=k+1) click_count[k] <= 0;
        end else begin
            key_single_event <= 0; key_double_event <= 0; key_long_event <= 0;
            for (k=0;k<2;k=k+1) begin
                if (key_long[k]) begin
                    // A long press owns the release sequence.  Do not turn a
                    // pending click into an extra +/-1 edit on the same edge.
                    key_long_event[k] <= 1;
                    click_pending[k] <= 0;
                    click_count[k] <= 0;
                end else if (key_short[k]) begin
                    if (click_pending[k] && click_count[k] != 0) begin
                        key_double_event[k] <= 1;
                        click_pending[k] <= 0;
                        click_count[k] <= 0;
                    end else if (!click_pending[k]) begin
                        click_pending[k] <= 1;
                        click_count[k] <= DOUBLE_CYCLES;
                    end else begin
                        key_single_event[k] <= 1;
                        click_count[k] <= DOUBLE_CYCLES;
                    end
                end else if (click_pending[k]) begin
                    if (click_count[k] == 0) begin
                        key_single_event[k] <= 1;
                        click_pending[k] <= 0;
                    end else click_count[k] <= click_count[k] - 1'b1;
                end
            end
        end
    end

    // 两键同按（都稳定按下）保持 COMBO_CYCLES 拍 -> 翻转 O_capture_req。
    // 一次按住只产生一次；必须两键都松开并稳定 2 拍后才允许下一次。
    // 已确认、非 bug 的行为（别"修"）：
    //   1) combo 之前就已经在双击窗口里的单击不会被取消。默认
    //      DOUBLE_CLICK_MS=350 < COMBO_MS=400，那次单击总在拍照之前落地。
    //   2) 不足 COMBO_MS 的双键按压不拍照；松开时两键各留一次单击。
    //      两个单击同拍到达选模式页时，FSM 的两次赋值读的是同一个旧
    //      mode_pending，是后写生效（P5 的 +1 覆盖 K4 的 -1），不是相抵；
    //      净效果是前进一个模式。别按"相抵"去改逻辑。
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            combo_seen <= 0; combo_count <= 0;
            combo_release <= 0; O_capture_req <= 0;
        end else begin
            if (both_down) begin
                combo_release <= 0;
                if (!combo_seen) begin
                    if (combo_count == COMBO_CYCLES-1) begin
                        combo_seen  <= 1'b1;
                        combo_count <= 0;
                        O_capture_req <= ~O_capture_req;
                    end else combo_count <= combo_count + 1'b1;
                end
            end else begin
                combo_count <= 0;
                if (combo_seen) begin
                    if (key_stable == 2'b11) begin
                        if (combo_release == 2'd1) begin combo_seen <= 0; combo_release <= 0; end
                        else combo_release <= combo_release + 1'b1;
                    end else combo_release <= 0;
                end
            end
        end
    end

    reg [2:0] mode_pending;
    reg       ae_disp_on;   // 曝光显示开关（原图页切换，切滤镜保持）
    // ★ 2026-11-01 构图辅助线（九宫格三分线 + 中心十字准星）。
    //   纯 LUT：只是几个坐标比较，不占 ERAM，也不碰视频通路。
    //   开关：模式页或全屏下**双击 K4**（参数页的双击是步进 10，不冲突）
    reg       grid_on;
    reg [3:0] focus_pending;
    reg page_pending;
    reg ui_hidden;                // 1 = OSD hidden, full-screen camera
    reg [7:0] strengths [0:4];
    reg [7:0] he_strength_pending;   // global HE strength, adjustable on the original page
    reg [7:0] primary [0:4];
    reg [7:0] secondary [0:4];
    reg [7:0] smoothing [0:4];
    // Comparison is a global display option, while filter values remain per mode.
    reg compare_pending;
    reg [7:0] split_pending;
    reg vs_previous;
    integer m;
    // 单击步进：默认 5。范围小的参数保持 1——
    //   浮雕光照方向 0..3、漫画墨线阈值 0..10，跳 5 会一格跨过好几个有效档。
    function [3:0] single_step;
        input [3:0] focus;
        input [2:0] mode;
        begin
            // ★ 2026-10-29：这里传进来的是 **滤镜号**（mode_to_filt）。
            //   3=铅笔 4=像素：铅笔 focus1 = 边缘阈值，像素 focus1 = 风格 0/1/2；
            //   2=浮雕 1=漫画 4=像素：浮雕方向、漫画墨线阈值仍在 focus2。
            if (focus == 4'd1 && (mode == 3'd3 || mode == 3'd4))
                single_step = 4'd1;
            else if (focus == 4'd2 && (mode == 3'd2 || mode == 3'd1 || mode == 3'd4))
                single_step = 4'd1;
            else
                single_step = 4'd5;
        end
    endfunction
    //-------------------------------------------------------------------------
    // ★ 2026-10-29：滤镜顺序改为 原图 / 像素 / 浮雕 / 铅笔 / 漫画。
    //
    //   下面三个默认值函数、以及 strengths/primary/secondary/smoothing 四个
    //   数组，全部按 **UI 位置** 索引（0=原图 1=像素 2=浮雕 3=铅笔 4=漫画）。
    //   而 single_step / next_focus / adjust_parameter 里判断"哪个滤镜"的
    //   分支必须用 **滤镜号**（mode_to_filt），两者已分离，别再混用。
    //-------------------------------------------------------------------------
    // UI 位置 -> 内部滤镜号（与 design_top_wrapper.v 的 S_filt_mode 映射一致）
    //   0 原图 -> 0 raw        1 像素 -> 4 pixel      2 浮雕 -> 2 emboss
    //   3 铅笔 -> 3 pencil     4 漫画 -> 1 comic
    function [2:0] mode_to_filt;
        input [2:0] pos;
        begin case(pos)
            3'd1:mode_to_filt=3'd4;
            3'd2:mode_to_filt=3'd2;
            3'd3:mode_to_filt=3'd3;
            3'd4:mode_to_filt=3'd1;
            default:mode_to_filt=3'd0;
        endcase end
    endfunction
    // UI 位置 -> **旧顺序下的位置**。字模表里的标签 id 是按旧顺序编号的
    // （10=原图 11=铅笔 12=浮雕 13=漫画 14=像素；参数行 2x/3x 同理），
    // 所以凡是取标签 id / 查字模的地方都必须用这个函数，不能用 mode_to_filt。
    //   新位置:  0 原图  1 像素  2 浮雕  3 铅笔  4 漫画
    //   旧位置:  0       4       2       1       3
    function [2:0] mode_to_oldpos;
        input [2:0] pos;
        begin case(pos)
            3'd1:mode_to_oldpos=3'd4;
            3'd2:mode_to_oldpos=3'd2;
            3'd3:mode_to_oldpos=3'd1;
            3'd4:mode_to_oldpos=3'd3;
            default:mode_to_oldpos=3'd0;
        endcase end
    endfunction
    function [7:0] default_primary;
        input [2:0] pos;
        // 按 UI 位置：
        //   pos1 像素 = 风格模式（占 primary 位）默认 0 = D 档
        //   pos2 浮雕 = 浮雕深度 75
        //   pos3 铅笔 = 已不显示的旧值，保留 60 仅兼容
        //   pos4 漫画 = 描边强度 80（UI 0..100 -> INK_GAIN 0..25，80 正好是 20）
        begin case(pos)
            3'd1:default_primary=8'd0;
            3'd2:default_primary=8'd75;
            3'd3:default_primary=8'd60;
            3'd4:default_primary=8'd80;
            default:default_primary=8'd0;
        endcase end
    endfunction
    function [7:0] default_strength;
        input [2:0] pos;
        begin
            // 按 UI 位置：原图 75 / 像素 70 / 浮雕 60 / 铅笔 75 / 漫画 90
            case(pos)
                3'd1:default_strength=8'd70;
                3'd2:default_strength=8'd60;
                3'd4:default_strength=8'd90;
                default:default_strength=8'd75;
            endcase
        end
    endfunction
    function [7:0] default_secondary;
        input [2:0] pos;
        begin case(pos)
            3'd1:default_secondary=8'd0;    // 像素：只用风格行
            3'd2:default_secondary=8'd1;    // 浮雕光照方向 0..3
            3'd3:default_secondary=8'd20;   // 铅笔边缘阈值
            // 漫画墨线阈值只保留前 10 个有效档位（0..10）
            3'd4:default_secondary=8'd3;
            default:default_secondary=8'd0;
        endcase end
    endfunction
    function [7:0] bounded_step;
        input integer value, amount, minimum, maximum;
        input decrease;
        integer adjusted;
        begin
            adjusted=value;
            if(decrease) adjusted=adjusted-amount;
            else adjusted=adjusted+amount;
            if(adjusted<minimum) adjusted=minimum;
            else if(adjusted>maximum) adjusted=maximum;
            bounded_step=adjusted;
        end
    endfunction
    function [3:0] next_focus;
        input [3:0] focus;
        input [2:0] mode;
        begin
            // 0..2 are filter parameters; 4/5/6 are compare, split, reset.
            // ★ 2026-10-24：漫画的"平滑强度"滑块撤掉了（固定成 clean_new 值），
            //   focus==3 不再出现，漫画和其他滤镜一样 0->1->2->4->5->6->0。
            if(mode==0) begin
                // 原图：强度 -> 均衡强度(focus1) -> 构图辅助(focus2)
                //       -> 对比 -> 分割 -> 恢复默认
                // ★ 2026-11-01：原来 focus 2 是跳过的，现在用来放构图辅助线开关。
                if(focus>=6) next_focus=0;
                else if(focus==0) next_focus=1;
                else if(focus==1) next_focus=2;
                else if(focus==2) next_focus=4;
                else if(focus>=4) next_focus=focus+1'b1;
                else next_focus=4;
            end else if(mode==1) begin
                // 铅笔：强度 -> 边缘阈值(focus1) -> 色彩强度(focus2)
                //       -> 对比(focus4) -> 分割(focus5) -> 恢复默认(focus6)
                if(focus>=6) next_focus=0;
                else if(focus==2) next_focus=4;
                else next_focus=focus+1'b1;
            end else if(mode==4) begin
                // 像素：强度 -> 风格(focus1) -> 对比(focus4)
                //       -> 分割(focus5) -> 恢复默认(focus6)
                if(focus>=6) next_focus=0;
                else if(focus==1) next_focus=4;
                else if(focus>=4) next_focus=(focus>=6)?0:focus+1'b1;
                else next_focus=focus+1'b1;
            end else begin
                if(focus>=6) next_focus=0;
                else if(focus==2) next_focus=4;
                else if(focus>=4) next_focus=(focus>=6)?0:focus+1'b1;
                else next_focus=focus+1'b1;
            end
        end
    endfunction
    function [2:0] mode_step;
        input [2:0] mode;
        input decrease;
        begin
            if(decrease) mode_step=(mode==0)?4:mode-1'b1;
            else mode_step=(mode==4)?0:mode+1'b1;
        end
    endfunction
    task adjust_parameter;
        input [3:0] focus;
        input [7:0] amount;
        input decrease;
        reg [2:0] filt;         // 当前 UI 位置对应的滤镜号（判断分支用）
        begin
            filt = mode_to_filt(mode_pending);
            case(focus)
                0:begin
                    // ★ 2026-10-30：「原图」页的 focus0 不再是滤镜强度，
                    //   改成"曝光显示"开关——按下即翻转，P5/K4 都行。
                    // ★ 2026-11-01 原图页 focus0 现在是「构图辅助」开关
                    if(mode_pending==3'd0)
                        grid_on <= ~grid_on;
                    else
                        strengths[mode_pending]<=bounded_step(strengths[mode_pending],amount,0,100,decrease);
                end
                // 铅笔：focus1 = 边缘阈值（secondary）；其余模式 focus1 = primary
                1:begin
                    if(filt==3'd0)          // ★ 原图：focus1 = 「曝光显示」开关
                        ae_disp_on <= ~ae_disp_on;
                    else if(filt==3'd3)     // 铅笔：焦点在"边缘阈值"（secondary）
                        secondary[mode_pending]<=bounded_step(secondary[mode_pending],amount,0,100,decrease);
                    else if(filt==3'd4)     // 像素风格 0=D / 1=E / 2=F
                        primary[mode_pending]<=bounded_step(primary[mode_pending],amount,0,2,decrease);
                    else                    // 浮雕：浮雕深度
                        primary[mode_pending]<=bounded_step(primary[mode_pending],amount,0,100,decrease);
                end
                // 铅笔：focus2 = 色彩强度（smoothing）；其余模式保持原语义
                2:begin
                    // ★ 2026-11-01 原图页 focus2 = 均衡强度（全局）
                    if(mode_pending==3'd0)
                        he_strength_pending<=bounded_step(he_strength_pending,amount,0,100,decrease);
                    else if(filt==3'd3)
                        smoothing[mode_pending]<=bounded_step(smoothing[mode_pending],amount,0,100,decrease);
                    else case(filt)
                        // 像素模式只有一行，focus2 不会到达；保留 0..2 以防万一
                        3'd4:secondary[mode_pending]<=bounded_step(secondary[mode_pending],amount,0,2,decrease);
                        3'd2:secondary[mode_pending]<=bounded_step(secondary[mode_pending],amount,0,3,decrease);
                        // 墨线阈值只保留前 10 个有效档位（0..10）。
                        3'd1:secondary[mode_pending]<=bounded_step(secondary[mode_pending],amount,0,10,decrease);
                        default:begin end
                    endcase
                end
                4:begin
                    // The comparison row is a binary setting. Any positive edit
                    // selects the requested side and keeps the two-button UI deterministic.
                    if(decrease) compare_pending<=1'b0;
                    else compare_pending<=1'b1;
                end
                5:split_pending<=bounded_step(split_pending,amount,0,100,decrease);
                6:begin
                    if(!decrease) begin
                        // 按 UI 位置：0=原图 1=像素 2=浮雕 3=铅笔 4=漫画
                        strengths[0]<=75; strengths[1]<=70; strengths[2]<=60; strengths[3]<=75; strengths[4]<=90;
                        primary[0]<=0;  primary[1]<=0;  primary[2]<=75; primary[3]<=60; primary[4]<=80;
                        secondary[0]<=0; secondary[1]<=0; secondary[2]<=1; secondary[3]<=20; secondary[4]<=3;
                        smoothing[0]<=40; smoothing[1]<=40; smoothing[2]<=40; smoothing[3]<=70; smoothing[4]<=36;
                        he_strength_pending<=8'd60;
                        compare_pending<=1'b0; split_pending<=8'd50;
                    end
                end
                default:begin end
            endcase
        end
    endtask
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            mode_pending<=0; focus_pending<=0;page_pending<=0;
            vs_previous<=0;O_mode<=0;O_strength<=75;O_param1<=0;O_param2<=0;O_smooth<=40;
            he_strength_pending<=8'd60;O_he_strength<=8'd60;
            compare_pending<=0;split_pending<=50;ae_disp_on<=1'b0;grid_on<=1'b0;
            O_compare_en<=0;O_split<=50;O_menu_visible<=1;O_focus<=0;O_page<=0;
            ui_hidden<=1'b0;
            for(m=0;m<5;m=m+1) begin
                strengths[m]<=default_strength(m);primary[m]<=default_primary(m);secondary[m]<=default_secondary(m);
                // 按 UI 位置：pos3=铅笔色彩强度（默认 70），pos4=漫画平滑（已删除，保留 36）。
                smoothing[m]<=(m==3)?8'd70:((m==4)?8'd36:8'd40);
            end
        end else begin
            vs_previous<=I_vs;
            if(!page_pending) begin
                // Mode selection follows the physical two-button specification:
                // a released click moves one mode, while a long K4 enters the
                // selected mode's parameter page. Double-clicks are ignored here.
                if(!ui_hidden) begin
                    if(key_single_event[0]) begin mode_pending<=mode_step(mode_pending,1'b1);focus_pending<=0;end
                    if(key_single_event[1]) begin mode_pending<=mode_step(mode_pending,1'b0);focus_pending<=0;end
                    if(key_long_event[0]) begin page_pending<=1;focus_pending<=0;end
                    // Long P5 on the mode page was a no-op; use it to hide the UI.
                    if(key_long_event[1]) begin ui_hidden<=1'b1;focus_pending<=0;end
                end else begin
                    // Full screen: clicks still switch filters, but there is no
                    // parameter page and no edit path.
                    if(key_single_event[0]) begin mode_pending<=mode_step(mode_pending,1'b1);focus_pending<=0;end
                    if(key_single_event[1]) begin mode_pending<=mode_step(mode_pending,1'b0);focus_pending<=0;end
                    if(key_long_event[0]) begin ui_hidden<=1'b0;page_pending<=1'b0;focus_pending<=0;end
                end
                // ★ 模式页 / 全屏：双击 K4 切换构图辅助线。参数页的双击是
                //   「步进 10」，所以这个手势只放在 !page_pending 里。
                if(key_double_event[0]) grid_on<=~grid_on;
            end else begin
                if(key_single_event[0]) adjust_parameter(focus_pending,single_step(focus_pending,mode_to_filt(mode_pending)),1'b1);
                if(key_double_event[0]) adjust_parameter(focus_pending,10,1'b1);
                if(key_single_event[1]) adjust_parameter(focus_pending,single_step(focus_pending,mode_to_filt(mode_pending)),1'b0);
                if(key_double_event[1]) adjust_parameter(focus_pending,10,1'b0);
                if(key_long_event[0]) focus_pending<=next_focus(focus_pending,mode_to_filt(mode_pending));
                if(key_long_event[1]) begin page_pending<=0;focus_pending<=0;end
            end
            if(I_vs && !vs_previous) begin
                O_mode<=mode_pending;O_strength<=strengths[mode_pending];O_param1<=primary[mode_pending];
                O_param2<=secondary[mode_pending];O_smooth<=smoothing[mode_pending];
                O_he_strength<=he_strength_pending;
                O_compare_en<=compare_pending;O_split<=split_pending;O_menu_visible<=~ui_hidden;O_focus<=focus_pending;O_page<=page_pending;
            end
        end
    end

    // Frame metadata: exact decimal glyphs and pixel bounds, no pixel-rate division.
    function [11:0] decimal_glyphs;
        input [7:0] value;
        begin case(value)
            8'd0:decimal_glyphs=12'hff0;
            8'd1:decimal_glyphs=12'hff1;
            8'd2:decimal_glyphs=12'hff2;
            8'd3:decimal_glyphs=12'hff3;
            8'd4:decimal_glyphs=12'hff4;
            8'd5:decimal_glyphs=12'hff5;
            8'd6:decimal_glyphs=12'hff6;
            8'd7:decimal_glyphs=12'hff7;
            8'd8:decimal_glyphs=12'hff8;
            8'd9:decimal_glyphs=12'hff9;
            8'd10:decimal_glyphs=12'hf10;
            8'd11:decimal_glyphs=12'hf11;
            8'd12:decimal_glyphs=12'hf12;
            8'd13:decimal_glyphs=12'hf13;
            8'd14:decimal_glyphs=12'hf14;
            8'd15:decimal_glyphs=12'hf15;
            8'd16:decimal_glyphs=12'hf16;
            8'd17:decimal_glyphs=12'hf17;
            8'd18:decimal_glyphs=12'hf18;
            8'd19:decimal_glyphs=12'hf19;
            8'd20:decimal_glyphs=12'hf20;
            8'd21:decimal_glyphs=12'hf21;
            8'd22:decimal_glyphs=12'hf22;
            8'd23:decimal_glyphs=12'hf23;
            8'd24:decimal_glyphs=12'hf24;
            8'd25:decimal_glyphs=12'hf25;
            8'd26:decimal_glyphs=12'hf26;
            8'd27:decimal_glyphs=12'hf27;
            8'd28:decimal_glyphs=12'hf28;
            8'd29:decimal_glyphs=12'hf29;
            8'd30:decimal_glyphs=12'hf30;
            8'd31:decimal_glyphs=12'hf31;
            8'd32:decimal_glyphs=12'hf32;
            8'd33:decimal_glyphs=12'hf33;
            8'd34:decimal_glyphs=12'hf34;
            8'd35:decimal_glyphs=12'hf35;
            8'd36:decimal_glyphs=12'hf36;
            8'd37:decimal_glyphs=12'hf37;
            8'd38:decimal_glyphs=12'hf38;
            8'd39:decimal_glyphs=12'hf39;
            8'd40:decimal_glyphs=12'hf40;
            8'd41:decimal_glyphs=12'hf41;
            8'd42:decimal_glyphs=12'hf42;
            8'd43:decimal_glyphs=12'hf43;
            8'd44:decimal_glyphs=12'hf44;
            8'd45:decimal_glyphs=12'hf45;
            8'd46:decimal_glyphs=12'hf46;
            8'd47:decimal_glyphs=12'hf47;
            8'd48:decimal_glyphs=12'hf48;
            8'd49:decimal_glyphs=12'hf49;
            8'd50:decimal_glyphs=12'hf50;
            8'd51:decimal_glyphs=12'hf51;
            8'd52:decimal_glyphs=12'hf52;
            8'd53:decimal_glyphs=12'hf53;
            8'd54:decimal_glyphs=12'hf54;
            8'd55:decimal_glyphs=12'hf55;
            8'd56:decimal_glyphs=12'hf56;
            8'd57:decimal_glyphs=12'hf57;
            8'd58:decimal_glyphs=12'hf58;
            8'd59:decimal_glyphs=12'hf59;
            8'd60:decimal_glyphs=12'hf60;
            8'd61:decimal_glyphs=12'hf61;
            8'd62:decimal_glyphs=12'hf62;
            8'd63:decimal_glyphs=12'hf63;
            8'd64:decimal_glyphs=12'hf64;
            8'd65:decimal_glyphs=12'hf65;
            8'd66:decimal_glyphs=12'hf66;
            8'd67:decimal_glyphs=12'hf67;
            8'd68:decimal_glyphs=12'hf68;
            8'd69:decimal_glyphs=12'hf69;
            8'd70:decimal_glyphs=12'hf70;
            8'd71:decimal_glyphs=12'hf71;
            8'd72:decimal_glyphs=12'hf72;
            8'd73:decimal_glyphs=12'hf73;
            8'd74:decimal_glyphs=12'hf74;
            8'd75:decimal_glyphs=12'hf75;
            8'd76:decimal_glyphs=12'hf76;
            8'd77:decimal_glyphs=12'hf77;
            8'd78:decimal_glyphs=12'hf78;
            8'd79:decimal_glyphs=12'hf79;
            8'd80:decimal_glyphs=12'hf80;
            8'd81:decimal_glyphs=12'hf81;
            8'd82:decimal_glyphs=12'hf82;
            8'd83:decimal_glyphs=12'hf83;
            8'd84:decimal_glyphs=12'hf84;
            8'd85:decimal_glyphs=12'hf85;
            8'd86:decimal_glyphs=12'hf86;
            8'd87:decimal_glyphs=12'hf87;
            8'd88:decimal_glyphs=12'hf88;
            8'd89:decimal_glyphs=12'hf89;
            8'd90:decimal_glyphs=12'hf90;
            8'd91:decimal_glyphs=12'hf91;
            8'd92:decimal_glyphs=12'hf92;
            8'd93:decimal_glyphs=12'hf93;
            8'd94:decimal_glyphs=12'hf94;
            8'd95:decimal_glyphs=12'hf95;
            8'd96:decimal_glyphs=12'hf96;
            8'd97:decimal_glyphs=12'hf97;
            8'd98:decimal_glyphs=12'hf98;
            8'd99:decimal_glyphs=12'hf99;
            8'd100:decimal_glyphs=12'h100;
            8'd101:decimal_glyphs=12'h101;
            8'd102:decimal_glyphs=12'h102;
            8'd103:decimal_glyphs=12'h103;
            8'd104:decimal_glyphs=12'h104;
            8'd105:decimal_glyphs=12'h105;
            8'd106:decimal_glyphs=12'h106;
            8'd107:decimal_glyphs=12'h107;
            8'd108:decimal_glyphs=12'h108;
            8'd109:decimal_glyphs=12'h109;
            8'd110:decimal_glyphs=12'h110;
            8'd111:decimal_glyphs=12'h111;
            8'd112:decimal_glyphs=12'h112;
            8'd113:decimal_glyphs=12'h113;
            8'd114:decimal_glyphs=12'h114;
            8'd115:decimal_glyphs=12'h115;
            8'd116:decimal_glyphs=12'h116;
            8'd117:decimal_glyphs=12'h117;
            8'd118:decimal_glyphs=12'h118;
            8'd119:decimal_glyphs=12'h119;
            8'd120:decimal_glyphs=12'h120;
            8'd121:decimal_glyphs=12'h121;
            8'd122:decimal_glyphs=12'h122;
            8'd123:decimal_glyphs=12'h123;
            8'd124:decimal_glyphs=12'h124;
            8'd125:decimal_glyphs=12'h125;
            8'd126:decimal_glyphs=12'h126;
            8'd127:decimal_glyphs=12'h127;
            8'd128:decimal_glyphs=12'h128;
            8'd129:decimal_glyphs=12'h129;
            8'd130:decimal_glyphs=12'h130;
            8'd131:decimal_glyphs=12'h131;
            8'd132:decimal_glyphs=12'h132;
            8'd133:decimal_glyphs=12'h133;
            8'd134:decimal_glyphs=12'h134;
            8'd135:decimal_glyphs=12'h135;
            8'd136:decimal_glyphs=12'h136;
            8'd137:decimal_glyphs=12'h137;
            8'd138:decimal_glyphs=12'h138;
            8'd139:decimal_glyphs=12'h139;
            8'd140:decimal_glyphs=12'h140;
            8'd141:decimal_glyphs=12'h141;
            8'd142:decimal_glyphs=12'h142;
            8'd143:decimal_glyphs=12'h143;
            8'd144:decimal_glyphs=12'h144;
            8'd145:decimal_glyphs=12'h145;
            8'd146:decimal_glyphs=12'h146;
            8'd147:decimal_glyphs=12'h147;
            8'd148:decimal_glyphs=12'h148;
            8'd149:decimal_glyphs=12'h149;
            8'd150:decimal_glyphs=12'h150;
            8'd151:decimal_glyphs=12'h151;
            8'd152:decimal_glyphs=12'h152;
            8'd153:decimal_glyphs=12'h153;
            8'd154:decimal_glyphs=12'h154;
            8'd155:decimal_glyphs=12'h155;
            8'd156:decimal_glyphs=12'h156;
            8'd157:decimal_glyphs=12'h157;
            8'd158:decimal_glyphs=12'h158;
            8'd159:decimal_glyphs=12'h159;
            8'd160:decimal_glyphs=12'h160;
            8'd161:decimal_glyphs=12'h161;
            8'd162:decimal_glyphs=12'h162;
            8'd163:decimal_glyphs=12'h163;
            8'd164:decimal_glyphs=12'h164;
            8'd165:decimal_glyphs=12'h165;
            8'd166:decimal_glyphs=12'h166;
            8'd167:decimal_glyphs=12'h167;
            8'd168:decimal_glyphs=12'h168;
            8'd169:decimal_glyphs=12'h169;
            8'd170:decimal_glyphs=12'h170;
            8'd171:decimal_glyphs=12'h171;
            8'd172:decimal_glyphs=12'h172;
            8'd173:decimal_glyphs=12'h173;
            8'd174:decimal_glyphs=12'h174;
            8'd175:decimal_glyphs=12'h175;
            8'd176:decimal_glyphs=12'h176;
            8'd177:decimal_glyphs=12'h177;
            8'd178:decimal_glyphs=12'h178;
            8'd179:decimal_glyphs=12'h179;
            8'd180:decimal_glyphs=12'h180;
            8'd181:decimal_glyphs=12'h181;
            8'd182:decimal_glyphs=12'h182;
            8'd183:decimal_glyphs=12'h183;
            8'd184:decimal_glyphs=12'h184;
            8'd185:decimal_glyphs=12'h185;
            8'd186:decimal_glyphs=12'h186;
            8'd187:decimal_glyphs=12'h187;
            8'd188:decimal_glyphs=12'h188;
            8'd189:decimal_glyphs=12'h189;
            8'd190:decimal_glyphs=12'h190;
            8'd191:decimal_glyphs=12'h191;
            8'd192:decimal_glyphs=12'h192;
            8'd193:decimal_glyphs=12'h193;
            8'd194:decimal_glyphs=12'h194;
            8'd195:decimal_glyphs=12'h195;
            8'd196:decimal_glyphs=12'h196;
            8'd197:decimal_glyphs=12'h197;
            8'd198:decimal_glyphs=12'h198;
            8'd199:decimal_glyphs=12'h199;
            8'd200:decimal_glyphs=12'h200;
            8'd201:decimal_glyphs=12'h201;
            8'd202:decimal_glyphs=12'h202;
            8'd203:decimal_glyphs=12'h203;
            8'd204:decimal_glyphs=12'h204;
            8'd205:decimal_glyphs=12'h205;
            8'd206:decimal_glyphs=12'h206;
            8'd207:decimal_glyphs=12'h207;
            8'd208:decimal_glyphs=12'h208;
            8'd209:decimal_glyphs=12'h209;
            8'd210:decimal_glyphs=12'h210;
            8'd211:decimal_glyphs=12'h211;
            8'd212:decimal_glyphs=12'h212;
            8'd213:decimal_glyphs=12'h213;
            8'd214:decimal_glyphs=12'h214;
            8'd215:decimal_glyphs=12'h215;
            8'd216:decimal_glyphs=12'h216;
            8'd217:decimal_glyphs=12'h217;
            8'd218:decimal_glyphs=12'h218;
            8'd219:decimal_glyphs=12'h219;
            8'd220:decimal_glyphs=12'h220;
            8'd221:decimal_glyphs=12'h221;
            8'd222:decimal_glyphs=12'h222;
            8'd223:decimal_glyphs=12'h223;
            8'd224:decimal_glyphs=12'h224;
            8'd225:decimal_glyphs=12'h225;
            8'd226:decimal_glyphs=12'h226;
            8'd227:decimal_glyphs=12'h227;
            8'd228:decimal_glyphs=12'h228;
            8'd229:decimal_glyphs=12'h229;
            8'd230:decimal_glyphs=12'h230;
            8'd231:decimal_glyphs=12'h231;
            8'd232:decimal_glyphs=12'h232;
            8'd233:decimal_glyphs=12'h233;
            8'd234:decimal_glyphs=12'h234;
            8'd235:decimal_glyphs=12'h235;
            8'd236:decimal_glyphs=12'h236;
            8'd237:decimal_glyphs=12'h237;
            8'd238:decimal_glyphs=12'h238;
            8'd239:decimal_glyphs=12'h239;
            8'd240:decimal_glyphs=12'h240;
            8'd241:decimal_glyphs=12'h241;
            8'd242:decimal_glyphs=12'h242;
            8'd243:decimal_glyphs=12'h243;
            8'd244:decimal_glyphs=12'h244;
            8'd245:decimal_glyphs=12'h245;
            8'd246:decimal_glyphs=12'h246;
            8'd247:decimal_glyphs=12'h247;
            8'd248:decimal_glyphs=12'h248;
            8'd249:decimal_glyphs=12'h249;
            8'd250:decimal_glyphs=12'h250;
            8'd251:decimal_glyphs=12'h251;
            8'd252:decimal_glyphs=12'h252;
            8'd253:decimal_glyphs=12'h253;
            8'd254:decimal_glyphs=12'h254;
            8'd255:decimal_glyphs=12'h255;
            default:decimal_glyphs=12'hfff;
        endcase end
    endfunction
    function [1:0] value_first_digit;
        input [7:0] value;
        begin
            if(value>=8'd100) value_first_digit=2'd0;
            else if(value>=8'd10) value_first_digit=2'd1;
            else value_first_digit=2'd2;
        end
    endfunction
    function [7:0] value_display_width;
        input [7:0] value;
        input percent;
        reg [1:0] first_digit;
        begin
            first_digit=value_first_digit(value);
            value_display_width=((3-first_digit)*16)+(percent?32:0);
        end
    endfunction
    function [7:0] digit_glyph;
        input [3:0] digit;
        begin
            digit_glyph=(digit==4'hf)?8'hff:(8'd1+digit);
        end
    endfunction
    function [23:0] value_glyphs;
        input [11:0] digits;
        begin
            value_glyphs={digit_glyph(digits[11:8]),digit_glyph(digits[7:4]),digit_glyph(digits[3:0])};
        end
    endfunction
    function [10:0] percent_bar_end;
        input [7:0] value;
        reg [15:0] scaled_delta;
        reg [15:0] value_ext;
        begin
            // 362/100 is represented as 29/8.  The small fixed-point error
            // is below one pixel and avoids a synthesized divider.
            value_ext=value;
            scaled_delta=(((value_ext<<4)+(value_ext<<3)+(value_ext<<2)+value_ext)+4)>>3;
            if(scaled_delta>362) scaled_delta=362;
            percent_bar_end=1506+scaled_delta;
        end
    endfunction
    function [10:0] ranged_bar_end;
        input [2:0] mode;
        input [3:0] focus;
        input [7:0] value;
        reg [10:0] scaled;
        reg [15:0] value_ext;
        begin
            scaled=1506;
            value_ext=value;
            if(focus==1 && mode==4) begin
                // 像素风格 0=D / 1=E / 2=F -> 三个刻度
                case(value)
                    8'd0: ranged_bar_end=1506;
                    8'd1: ranged_bar_end=1687;
                    default: ranged_bar_end=1868;
                endcase
            end else if(focus!=2) begin
                ranged_bar_end=percent_bar_end(value);
            end else begin
                case(mode)
                    1: begin
                        // 铅笔边缘阈值 0..100 -> 进度条 0..362
                        ranged_bar_end=percent_bar_end(value);
                    end
                    2: begin
                        case(value)
                            8'd0: ranged_bar_end=1506;
                            8'd1: ranged_bar_end=1627;
                            8'd2: ranged_bar_end=1747;
                            default: ranged_bar_end=1868;
                        endcase
                    end
                    3: begin
                        // 墨线阈值 0..10 -> 滑块条 0..362（100% 满格）
                        ranged_bar_end=percent_bar_end(value*10);
                    end
                    default: ranged_bar_end=percent_bar_end(value);
                endcase
            end
        end
    endfunction
    function [10:0] split_position;
        input [7:0] value;
        reg [17:0] scaled;
        reg [17:0] value_ext;
        begin
            // 1456/100 is represented as 233/16; the maximum error is one
            // pixel while every 1% setting remains monotonic.
            value_ext=value;
            scaled=((value_ext<<7)+(value_ext<<6)+(value_ext<<5)+(value_ext<<3)+value_ext+8)>>4;
            if(scaled>1456) scaled=1456;
            split_position=scaled;
        end
    endfunction
    reg [23:0] strength_glyphs, primary_glyphs, secondary_glyphs, smooth_glyphs, split_glyphs;
    reg [1:0] strength_first_digit, primary_first_digit, secondary_first_digit, smooth_first_digit, split_first_digit;
    reg [7:0] strength_text_width, primary_text_width, secondary_text_width, smooth_text_width, split_text_width;
    // 第 3 行（y=331）的数值命中区：原图页画 he_strength，其余模式画 smooth，
    // 取两者较大宽度即可同时覆盖。
    wire [7:0] val_w_row2 = (primary_text_width > smooth_text_width) ? primary_text_width : smooth_text_width;
    // 自动曝光测光显示用的字形/宽度（和 strength_glyphs 同一套预算节奏）
    reg [23:0] ae_mean_glyphs, ae_gain_glyphs;
    reg [1:0]  ae_mean_first, ae_gain_first;
    reg [7:0]  ae_mean_width, ae_gain_width;
    reg [10:0] strength_end, primary_end, secondary_end, smooth_end, split_end, split_left, split_right;
    // ★ 2026-10-24：帧率数字的格式化（decimal_glyphs 256 项译码 + digit_glyph +
    //   宽度计算）原来直接放在像素级 always @* 里，估算 13~18 级 LUT，是 150MHz 域
    //   新增的第二条深组合链。改成和 strength_glyphs 一样在 VS 沿预算好，
    //   像素路径只做查表。
    reg [23:0] fps_glyphs_r;
    reg [1:0]  fps_first_r;
    reg [7:0]  fps_width_r;
    // End-to-end delay readout: 2 integer digits + '.' + 1 decimal digit.
    // Formatting happens in the same VS budget cycle as the other readouts.
    reg [11:0] delay_s;
    reg        delay_valid_s;
    reg [31:0] delay_glyphs_r;
    // 2026-10-27: these values used to be computed inside the single I_vs edge
    // cycle, which put a 12-level chain (value*10 through the DSP, then four
    // shifted adds and a saturate) on the clk3 critical path.  Split across
    // four cycles inside vblank instead: cycle 0 latches the parameters, 1
    // computes the bar ends, 2 the digit glyphs, 3 the text widths and the
    // split position.  Vblank is ~570k cycles long, so four cycles are invisible.
    reg [1:0] budget_step;
    reg [2:0] mode_s;
    reg [7:0] strength_s, primary_s, secondary_s, smooth_s, split_s, fps_s;
    // ---- 自动曝光测光显示（★ 2026-10-30）----
    //   ae_*_d 是两级同步（滤镜域 -> 150MHz 域），ae_*_s 在帧首锁存。
    reg [7:0] ae_mean_d, ae_gain_d;
    reg [7:0] ae_mean_s, ae_gain_s;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            ae_mean_d <= 8'd0; ae_gain_d <= 8'd0;
        end else begin
            ae_mean_d <= I_ae_mean;
            ae_gain_d <= I_ae_gain;
        end
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            strength_glyphs<=24'hff0806;primary_glyphs<=24'hffff01;secondary_glyphs<=24'hffff01;
            smooth_glyphs<=24'hff0501;split_glyphs<=24'hff0601;
            strength_first_digit<=1;primary_first_digit<=2;secondary_first_digit<=2;
            smooth_first_digit<=1;split_first_digit<=1;
            strength_text_width<=64;primary_text_width<=48;secondary_text_width<=16;
            smooth_text_width<=64;split_text_width<=64;
            strength_end<=11'd1782;primary_end<=11'd1512;secondary_end<=11'd1512;smooth_end<=11'd1656;split_end<=11'd1692;
            split_left<=11'd727;split_right<=11'd729;
            fps_glyphs_r<=24'hff0806;fps_first_r<=2'd2;fps_width_r<=8'd32;
            ae_gain_glyphs<=24'hff0806;ae_gain_first<=2'd2;ae_gain_width<=8'd32;
            ae_mean_glyphs<=24'hff0806;ae_mean_first<=2'd2;ae_mean_width<=8'd32;
            budget_step<=2'd0;
            mode_s<=0;strength_s<=0;primary_s<=0;secondary_s<=0;
            smooth_s<=0;split_s<=0;fps_s<=0;
        end else begin
            if(I_vs && !vs_previous) begin
                budget_step<=2'd1;
                mode_s<=mode_pending;
                strength_s<=strengths[mode_pending];
                primary_s<=primary[mode_pending];
                secondary_s<=secondary[mode_pending];
                smooth_s<=smoothing[mode_pending];
                split_s<=split_pending;
                fps_s<=I_fps;
                ae_mean_s<=ae_mean_d;
                ae_gain_s<=ae_gain_d;
            end else if(budget_step!=2'd0 && budget_step!=2'd3) begin
                budget_step<=budget_step+2'd1;
            end else begin
                budget_step<=2'd0;
            end

            // Cycle 1: progress-bar end positions.
            if(budget_step==2'd1) begin
                strength_end<=percent_bar_end(strength_s);
                // 像素模式的风格行需要 0/1/2 三档；其他模式 focus1 仍等价于 percent
                primary_end<=ranged_bar_end(mode_to_oldpos(mode_s),1,primary_s);
                if(mode_to_filt(mode_s)==3'd0)
                    primary_end<=percent_bar_end(he_strength_pending);
                secondary_end<=ranged_bar_end(mode_to_oldpos(mode_s),2,secondary_s);
                smooth_end<=percent_bar_end(smooth_s);
                split_end<=percent_bar_end(split_s);
            end

            // Cycle 2: digit glyphs and first-digit indices.
            if(budget_step==2'd2) begin
                fps_glyphs_r<=value_glyphs(decimal_glyphs(fps_s));
                fps_first_r<=value_first_digit(fps_s);
                ae_gain_glyphs<=value_glyphs(decimal_glyphs(ae_gain_s));
                ae_gain_first<=value_first_digit(ae_gain_s);
                ae_mean_glyphs<=value_glyphs(decimal_glyphs(ae_mean_s));
                ae_mean_first<=value_first_digit(ae_mean_s);
                strength_glyphs<=value_glyphs(decimal_glyphs(strength_s));
                primary_glyphs<=value_glyphs(decimal_glyphs(primary_s));
                secondary_glyphs<=value_glyphs(decimal_glyphs(secondary_s));
                smooth_glyphs<=value_glyphs(decimal_glyphs(smooth_s));
                split_glyphs<=value_glyphs(decimal_glyphs(split_s));
                strength_first_digit<=value_first_digit(strength_s);
                primary_first_digit<=value_first_digit(primary_s);
                if(mode_to_filt(mode_s)==3'd0) begin
                    primary_glyphs<=value_glyphs(decimal_glyphs(he_strength_pending));
                    primary_first_digit<=value_first_digit(he_strength_pending);
                end
                secondary_first_digit<=value_first_digit(secondary_s);
                smooth_first_digit<=value_first_digit(smooth_s);
                split_first_digit<=value_first_digit(split_s);
            end

            // Cycle 3: text widths and the compare split position.
            if(budget_step==2'd3) begin
                fps_width_r<=value_display_width(fps_s,1'b0);
                ae_gain_width<=value_display_width(ae_gain_s,1'b0);
                ae_mean_width<=value_display_width(ae_mean_s,1'b0);
                strength_text_width<=value_display_width(strength_s,1'b1);
                // 像素风格的 0/1/2 不带百分号；其他模式仍带
                primary_text_width<=value_display_width(primary_s, mode_to_filt(mode_s)!=3'd4);
                if(mode_to_filt(mode_s)==3'd0)
                    primary_text_width<=value_display_width(he_strength_pending,1'b1);
                secondary_text_width<=value_display_width(secondary_s,1'b0);
                smooth_text_width<=value_display_width(smooth_s,1'b1);
                split_text_width<=value_display_width(split_s,1'b1);
                if(split_position(split_s)==0) begin
                    split_left<=11'd0;
                    split_right<=11'd1;
                end else if(split_position(split_s)>=1456) begin
                    split_left<=11'd1455;
                    split_right<=11'd1456;
                end else begin
                    split_left<=split_position(split_s)-11'd1;
                    split_right<=split_position(split_s)+11'd1;
                end
            end
        end
    end

    // Coordinates refer to raw active pixels, independent of RGB/FIFO availability.
    reg [11:0] x_count, y_count;
    reg de_previous;
    // x_count is cleared on the falling DE edge, before the next active line.
    // Use it directly: a DE-to-coordinate mux would fan out across the whole OSD.
    wire [11:0] pixel_x = x_count;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin x_count<=0;y_count<=0;de_previous<=0;end
        else begin
            de_previous<=I_de;
            if(I_vs) begin x_count<=0;y_count<=0;end
            else if(I_de) x_count<=pixel_x+1'b1;
            else if(de_previous) begin x_count<=0;y_count<=y_count+1'b1;end
        end
    end

    // The comic filter uses one RGB line store over the 1456x819 camera
    // viewport. Neighborhood and arithmetic stages are registered separately
    // to keep the 150 MHz pixel path within one cycle.
    localparam integer PREVIEW_WIDTH = 1456;
    // ★ 2026-11-01：这里原来是 comic_line[0:1455]（24bit ≈ 34.9 kbit），
    //   正好占 2 块 ERAM。它属于一条**死滤镜流水线**：唯一终点
    //   effect_rgb_d8 没有任何读者（滤镜已经搬到写侧的 ui_filter_domain）。
    //   综合工具不会剪掉被写过的 RAM，所以这 2 块 ERAM 一直白占。
    //   这里退化成单寄存器——整条链是死的，存什么都到不了输出，
    //   验收判据是 9 张 PPM 与改前逐字节一致。
    reg [23:0] comic_line_px;
    reg [23:0] line_rgb_d1, upper_left_rgb_d1, upper_left2_rgb_d1;
    reg [23:0] left_rgb_d1, left2_rgb_d1, camera_rgb_d1;
    reg [11:0] x_d1, y_d1;
    reg hs_d1, vs_d1, de_d1, preview_d1;
    reg [23:0] comic_left_d2, comic_left2_d2, comic_up_d2;
    reg [23:0] comic_upleft_d2, comic_upleft2_d2;
    reg [7:0] gray_left_in_d2, gray_up_in_d2;
    reg [7:0] emboss_cur_gray_d2, emboss_left_gray_d2;
    reg [7:0] emboss_up_gray_d2, emboss_upleft_gray_d2;
    reg [23:0] camera_rgb_d2;
    reg [11:0] x_d2, y_d2;
    reg hs_d2, vs_d2, de_d2, preview_d2;
    reg [2:0] mode_d2;
    reg [7:0] strength_d2, param1_d2, param2_d2, smooth_amount_d2;
    reg [23:0] camera_rgb_d3, average_rgb_d3, emboss_rgb_d3;
    reg [11:0] x_d3, y_d3;
    reg hs_d3, vs_d3, de_d3, preview_d3;
    reg [2:0] mode_d3;
    reg [7:0] strength_d3, param1_d3, param2_d3, smooth_amount_d3;
    reg [7:0] gray_left_d3, gray_up_d3;
    reg [23:0] camera_rgb_d4, smooth_rgb_d4, emboss_rgb_d4;
    reg [11:0] x_d4, y_d4;
    reg hs_d4, vs_d4, de_d4, preview_d4;
    reg [2:0] mode_d4;
    reg [7:0] strength_d4, param1_d4, param2_d4;
    reg [7:0] gray_left_d4, gray_up_d4;
    reg [23:0] camera_rgb_d5, smooth_rgb_d5, emboss_rgb_d5;
    reg [11:0] x_d5, y_d5;
    reg hs_d5, vs_d5, de_d5, preview_d5;
    reg [2:0] mode_d5;
    reg [7:0] strength_d5, param1_d5, param2_d5;
    reg [7:0] gray_current_d5, gray_left_d5, gray_up_d5;
    reg [8:0] horizontal_difference_d6, vertical_difference_d6;
    reg edge_line_previous_d6;
    reg [23:0] camera_rgb_d6, smooth_rgb_d6, emboss_rgb_d6;
    reg [11:0] x_d6, y_d6;
    reg hs_d6, vs_d6, de_d6, preview_d6;
    reg [2:0] mode_d6;
    reg [7:0] strength_d6, param1_d6, param2_d6, gray_current_d6;
    reg [9:0] edge_strength_d6;
    reg edge_from_d6;
    // A one-bit line store and two horizontal history bits turn the sparse
    // gradient response into a continuous ink stroke.  The line store keeps
    // the previous-row contour so horizontal object boundaries are not lost
    // when the color quantizer flattens both sides to the same tone.
    reg edge_hist1_d7, edge_hist2_d7;
    reg edge_line_px;      // 同上：原来是 edge_line[0:1455] 的死行缓存
    reg [9:0] comic_edge_threshold;
    reg comic_edge_from_d6;
    reg [23:0] comic_left, comic_left2, comic_up, comic_upleft, comic_upleft2;
    // Keep the comic ink as a one-pixel strong-contour mask.  Expanding weak
    // responses through the line store made wood grain and tabletop scratches
    // look like object outlines.
    wire comic_edge_outline_d6 = comic_edge_from_d6;
    reg [23:0] camera_rgb_d7, comic_rgb_d7;
    reg comic_edge_d7;
    reg [11:0] x_d7, y_d7;
    reg hs_d7, vs_d7, de_d7;
    reg [7:0] blend_strength_d7;
    reg [2:0] mode_d7;
    reg [23:0] camera_rgb_d8, effect_rgb_d8;
    // Camera pixel must advance with the OSD so the render block (which runs
    // on d9) composites the filtered image of the same pixel position.
    reg [23:0] camera_rgb_d9;
    reg [11:0] x_d8, y_d8;
    // y_content = y_d8 - 78 is computed one stage early so the geometry
    // pre-compute stage does not spend a logic level on the subtraction.
    reg [11:0] y_content_d8;
    reg hs_d8, vs_d8, de_d8;
    reg [2:0] mode_d8;
    reg [23:0] last_line_rgb, last_left_rgb;

    function [7:0] rgb_luma;
        input [23:0] rgb;
        begin
            // Shift-only Rec.601 approximation.  The former three constant
            // multipliers were replicated for every edge tap and dominated
            // the 150 MHz pixel path after the new mode branches were added.
            rgb_luma = (rgb[23:16]>>2) + (rgb[15:8]>>1) + (rgb[7:0]>>2);
        end
    endfunction

    function [7:0] emboss_gray;
        input [23:0] rgb;
        reg [15:0] r, g, b;
        reg [15:0] acc;
        begin
            r = {8'd0,rgb[23:16]};
            g = {8'd0,rgb[15:8]};
            b = {8'd0,rgb[7:0]};
            acc = (r<<6)+(r<<3)+(r<<2)+r
                + (g<<7)+(g<<4)+(g<<2)+(g<<1)
                + (b<<4)+(b<<3)+(b<<2)+b;
            emboss_gray = acc[15:8];
        end
    endfunction

    function [7:0] clamp_emboss;
        input signed [13:0] value;
        begin
            if(value < 0) clamp_emboss=8'd0;
            else if(value > 14'sd255) clamp_emboss=8'd255;
            else clamp_emboss=value[7:0];
        end
    endfunction

    function [7:0] abs_diff8;
        input [7:0] a, b;
        begin
            abs_diff8 = (a>=b) ? (a-b) : (b-a);
        end
    endfunction

    // Eight fixed blend points keep strength=0 as an exact pass-through,
    // strength=50 as the calibrated default, and strength=100 as the full
    // filtered target.  The shift/add weights make the control visibly useful
    // without inserting a general multiplier in the pixel path.
    function [7:0] blend_fast_channel;
        input [7:0] original, target, amount;
        reg [11:0] sum;
        begin
            if(amount==0) blend_fast_channel=original;
            else if(amount>=8'd88) blend_fast_channel=target;
            else if(amount>=8'd75) begin
                sum=({2'd0,original})+({2'd0,target}<<3)+8'd4;
                blend_fast_channel=sum[10:3];
            end else if(amount>=8'd63) begin
                sum=({2'd0,original}<<1)+({2'd0,target}<<2)+({2'd0,target})+8'd4;
                blend_fast_channel=sum[10:3];
            end else if(amount>=8'd50) begin
                sum=({2'd0,original}<<2)+({2'd0,target}<<2)+8'd4;
                blend_fast_channel=sum[10:3];
            end else if(amount>=8'd38) begin
                sum=({2'd0,original}<<2)+({2'd0,original})+({2'd0,target}<<1)+({2'd0,target})+8'd4;
                blend_fast_channel=sum[10:3];
            end else if(amount>=8'd25) begin
                sum=({2'd0,original}<<2)+({2'd0,original}<<1)+({2'd0,target})+8'd4;
                blend_fast_channel=sum[10:3];
            end else if(amount>=8'd13) begin
                sum=({2'd0,original}<<2)+({2'd0,original}<<1)+({2'd0,original})+{2'd0,target}+8'd4;
                blend_fast_channel=sum[10:3];
            end else begin
                sum=({2'd0,original}<<3)+{2'd0,target}+8'd4;
                blend_fast_channel=sum[10:3];
            end
        end
    endfunction

    // Fixed threshold/ROM posterizer.  The previous multiply/divide implementation
    // made a long pixel-rate arithmetic path and amplified fine wood grain.  The
    // input high nibble selects a nearby palette entry; this is LUT friendly and
    // keeps every supported level count deterministic in hardware.
    function [7:0] posterize_channel;
        input [7:0] value;
        input [7:0] levels;
        begin
            case(levels)
                8'd4: begin case(value[7:4]
                    ) 4'd0,4'd1,4'd2:posterize_channel=8'd0;
                      4'd3,4'd4,4'd5,4'd6,4'd7:posterize_channel=8'd85;
                      4'd8,4'd9,4'd10,4'd11,4'd12:posterize_channel=8'd170;
                      default:posterize_channel=8'd255; endcase end
                8'd6: begin case(value[7:4]
                    ) 4'd0,4'd1:posterize_channel=8'd0;
                      4'd2,4'd3,4'd4:posterize_channel=8'd51;
                      4'd5,4'd6,4'd7:posterize_channel=8'd102;
                      4'd8,4'd9,4'd10:posterize_channel=8'd153;
                      4'd11,4'd12,4'd13:posterize_channel=8'd204;
                      default:posterize_channel=8'd255; endcase end
                8'd8: begin case(value[7:4]
                    ) 4'd0,4'd1:posterize_channel=8'd0;
                      4'd2,4'd3,4'd4:posterize_channel=8'd51;
                      4'd5,4'd6,4'd7:posterize_channel=8'd102;
                      4'd8,4'd9,4'd10:posterize_channel=8'd153;
                      4'd11,4'd12,4'd13:posterize_channel=8'd204;
                      default:posterize_channel=8'd255; endcase end
                8'd12: begin case(value[7:4]
                    ) 4'd0:posterize_channel=8'd0;
                      4'd1:posterize_channel=8'd23;
                      4'd2,4'd3:posterize_channel=8'd46;
                      4'd4:posterize_channel=8'd70;
                      4'd5,4'd6:posterize_channel=8'd93;
                      4'd7:posterize_channel=8'd116;
                      4'd8:posterize_channel=8'd139;
                      4'd9,4'd10:posterize_channel=8'd162;
                      4'd11:posterize_channel=8'd185;
                      4'd12,4'd13:posterize_channel=8'd209;
                      4'd14:posterize_channel=8'd232;
                      default:posterize_channel=8'd255; endcase end
                default: begin case(value[7:4]
                    ) 4'd0:posterize_channel=8'd0;
                      4'd1:posterize_channel=8'd17;
                      4'd2:posterize_channel=8'd34;
                      4'd3:posterize_channel=8'd51;
                      4'd4:posterize_channel=8'd68;
                      4'd5:posterize_channel=8'd85;
                      4'd6:posterize_channel=8'd102;
                      4'd7:posterize_channel=8'd119;
                      4'd8:posterize_channel=8'd136;
                      4'd9:posterize_channel=8'd153;
                      4'd10:posterize_channel=8'd170;
                      4'd11:posterize_channel=8'd187;
                      4'd12:posterize_channel=8'd204;
                      4'd13:posterize_channel=8'd221;
                      4'd14:posterize_channel=8'd238;
                      default:posterize_channel=8'd255; endcase end
            endcase
        end
    endfunction

    // Shift one channel by the quantized luminance delta.  Quantizing a
    // shared luminance value keeps hue and saturation stable; independently
    // quantizing R/G/B was the source of the dirty colour speckle in the
    // previous comic result.
    function [7:0] tone_channel;
        input [7:0] channel_value, luminance_value, quantized_luminance;
        reg signed [10:0] delta;
        reg signed [10:0] shifted;
        begin
            delta = $signed({1'b0,quantized_luminance}) - $signed({1'b0,luminance_value});
            shifted = $signed({1'b0,channel_value}) + delta;
            if(shifted < 0) tone_channel=8'd0;
            else if(shifted > 255) tone_channel=8'd255;
            else tone_channel=shifted[7:0];
        end
    endfunction

    wire [23:0] camera_rgb_in = I_pixel_valid ? I_rgb : 24'h000000;
    wire [7:0] emboss_cur_gray = emboss_cur_gray_d2;
    wire [7:0] emboss_up_gray = emboss_up_gray_d2;
    wire [7:0] emboss_left_gray = emboss_left_gray_d2;
    wire [7:0] emboss_upleft_gray = emboss_upleft_gray_d2;
    reg signed [8:0] emboss_diff;
    always @* begin
        // Four causal 2x2 diagonal pairings keep the lvjing emboss response
        // usable in this blanking-aware single-pixel stream.
        case(O_param2[1:0])
            2'd0: emboss_diff = $signed({1'b0,emboss_upleft_gray}) - $signed({1'b0,emboss_cur_gray});
            2'd1: emboss_diff = $signed({1'b0,emboss_up_gray}) - $signed({1'b0,emboss_left_gray});
            2'd2: emboss_diff = $signed({1'b0,emboss_cur_gray}) - $signed({1'b0,emboss_upleft_gray});
            default: emboss_diff = $signed({1'b0,emboss_left_gray}) - $signed({1'b0,emboss_up_gray});
        endcase
    end
    wire [3:0] emboss_gain = (O_param1 > 8'd100) ? 4'd12 : O_param1[6:3];
    wire signed [12:0] emboss_product = emboss_diff * $signed({1'b0,emboss_gain});
    wire signed [13:0] emboss_level = (emboss_product >>> 2) + 14'sd128;
    wire [7:0] emboss_value = clamp_emboss(emboss_level);
    wire [23:0] emboss_rgb = {emboss_value,emboss_value,emboss_value};
    wire preview_pixel_in = I_de && pixel_x<PREVIEW_WIDTH &&
                            y_count>=12'd78 && y_count<12'd897;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            line_rgb_d1<=0;upper_left_rgb_d1<=0;upper_left2_rgb_d1<=0;
            left_rgb_d1<=0;left2_rgb_d1<=0;camera_rgb_d1<=0;
            x_d1<=0;y_d1<=0;hs_d1<=0;vs_d1<=0;de_d1<=0;preview_d1<=0;
            last_line_rgb<=0;last_left_rgb<=0;
        end else begin
            // The upscaler output is valid on every active pixel; the repeat
            // lines of hidden mode have I_pixel_valid=0 but must not go black.
            camera_rgb_d1<=I_rgb;
            x_d1<=pixel_x;y_d1<=y_count;
            hs_d1<=I_hs;vs_d1<=I_vs;de_d1<=I_de;
            preview_d1<=preview_pixel_in;
            if(preview_pixel_in) begin
                line_rgb_d1 <= (y_count==12'd78) ? camera_rgb_in : comic_line_px;
                // Three-tap causal horizontal window on both current and prior
                // rows. Replication at the first two columns avoids a dark seam.
                upper_left_rgb_d1 <= (y_count==12'd78 || pixel_x==0) ? camera_rgb_in : line_rgb_d1;
                upper_left2_rgb_d1 <= (y_count==12'd78 || pixel_x<12'd2) ? camera_rgb_in : upper_left_rgb_d1;
                left_rgb_d1 <= (pixel_x==0) ? camera_rgb_in : last_line_rgb;
                left2_rgb_d1 <= (pixel_x<12'd2) ? camera_rgb_in : last_left_rgb;
                comic_line_px <= camera_rgb_in;
                last_left_rgb <= last_line_rgb;
                last_line_rgb <= camera_rgb_in;
            end else begin
                line_rgb_d1<=0;upper_left_rgb_d1<=0;upper_left2_rgb_d1<=0;
                left_rgb_d1<=0;left2_rgb_d1<=0;
                if(I_de && pixel_x==0) begin last_line_rgb<=0;last_left_rgb<=0;end
            end
        end
    end

    reg [23:0] smooth_rgb, average_rgb;
    reg [11:0] average_r, average_g, average_b;
    reg [9:0] edge_threshold;
    reg [7:0] gray_current, gray_left, gray_up;
    reg [8:0] gray_left_sum, gray_up_sum;
    reg [7:0] smooth_r, smooth_g, smooth_b;
    reg [7:0] pencil_base_d6;
    reg pencil_hatch_d6;
    reg [8:0] horizontal_difference, vertical_difference;
    always @* begin
        // Comic-only range gating: reject neighbor taps across strong luma
        // transitions, approximating the range term in a bilateral filter.
        // Pencil keeps the original un-gated smoothing kernel unchanged.
        comic_left=left_rgb_d1;
        comic_left2=left2_rgb_d1;
        comic_up=line_rgb_d1;
        comic_upleft=upper_left_rgb_d1;
        comic_upleft2=upper_left2_rgb_d1;
        if(O_mode==3) begin
            if(abs_diff8(rgb_luma(camera_rgb_d1),rgb_luma(left_rgb_d1))>8'd24) comic_left=camera_rgb_d1;
            if(abs_diff8(rgb_luma(camera_rgb_d1),rgb_luma(left2_rgb_d1))>8'd32) comic_left2=camera_rgb_d1;
            if(abs_diff8(rgb_luma(camera_rgb_d1),rgb_luma(line_rgb_d1))>8'd24) comic_up=camera_rgb_d1;
            if(abs_diff8(rgb_luma(camera_rgb_d1),rgb_luma(upper_left_rgb_d1))>8'd24) comic_upleft=camera_rgb_d1;
            if(abs_diff8(rgb_luma(camera_rgb_d1),rgb_luma(upper_left2_rgb_d1))>8'd32) comic_upleft2=camera_rgb_d1;
        end
        // 3x2 separable causal Gaussian: [1 2 1] horizontally and
        // [1 1] vertically. It removes tabletop grain with only two extra
        // pixel registers and keeps the one-line streaming memory unchanged.
        // [1 2 1] horizontally on the current and previous rows.  The six
        // taps sum to eight, so [10:3] is the normalized (rounded) average.
        average_r=({4'd0,camera_rgb_d2[23:16]})+({4'd0,comic_left_d2[23:16]}<<1)+{4'd0,comic_left2_d2[23:16]}
                 +({4'd0,comic_up_d2[23:16]})+({4'd0,comic_upleft_d2[23:16]}<<1)+{4'd0,comic_upleft2_d2[23:16]}+12'd4;
        average_g=({4'd0,camera_rgb_d2[15:8]})+({4'd0,comic_left_d2[15:8]}<<1)+{4'd0,comic_left2_d2[15:8]}
                 +({4'd0,comic_up_d2[15:8]})+({4'd0,comic_upleft_d2[15:8]}<<1)+{4'd0,comic_upleft2_d2[15:8]}+12'd4;
        average_b=({4'd0,camera_rgb_d2[7:0]})+({4'd0,comic_left_d2[7:0]}<<1)+{4'd0,comic_left2_d2[7:0]}
                 +({4'd0,comic_up_d2[7:0]})+({4'd0,comic_upleft_d2[7:0]}<<1)+{4'd0,comic_upleft2_d2[7:0]}+12'd4;
        average_rgb={average_r[10:3],average_g[10:3],average_b[10:3]};
        smooth_r=blend_fast_channel(camera_rgb_d3[23:16],average_rgb_d3[23:16],smooth_amount_d3);
        smooth_g=blend_fast_channel(camera_rgb_d3[15:8],average_rgb_d3[15:8],smooth_amount_d3);
        smooth_b=blend_fast_channel(camera_rgb_d3[7:0],average_rgb_d3[7:0],smooth_amount_d3);
        smooth_rgb={smooth_r,smooth_g,smooth_b};
        // Detect contours from the smoothed center pixel.  Using the raw camera
        // luma here outlined every tabletop grain and sensor speckle.
        // Compare against low-pass neighbours from the same causal window.
        // Mixing a smoothed centre with raw neighbours was creating isolated
        // black specks on wood grain and wire bundles.
        gray_left_sum={1'b0,rgb_luma(left_rgb_d1)}+{1'b0,rgb_luma(upper_left_rgb_d1)};
        gray_up_sum={1'b0,rgb_luma(line_rgb_d1)}+{1'b0,rgb_luma(upper_left_rgb_d1)};
        gray_left=gray_left_sum[8:1];
        gray_up=gray_up_sum[8:1];
        // Keep the default threshold high enough for broad object boundaries
        // on ordinary camera scenes.  The user can lower it from the UI for
        // stronger outlines; an overly low fixed threshold turns table grain
        // and ceiling texture into large black regions.
        // Comic uses a slightly lower seed than pencil: the edge-aware
        // smoothing has already removed isolated sensor/wood texture, while
        // the connected weak branch keeps the contour continuous.
        gray_current=rgb_luma(smooth_rgb_d4);
        horizontal_difference=(gray_current_d5>=gray_left_d5)?
            gray_current_d5-gray_left_d5:gray_left_d5-gray_current_d5;
        vertical_difference=(gray_current_d5>=gray_up_d5)?
            gray_current_d5-gray_up_d5:gray_up_d5-gray_current_d5;
        edge_threshold=(param1_d6>=8'd96)?10'd24:10'd96-(param1_d6>>1);
        comic_edge_threshold=(param1_d6>=8'd96)?10'd20:10'd52-(param1_d6>>1);
        edge_strength_d6=(horizontal_difference_d6>=vertical_difference_d6)?
            horizontal_difference_d6:vertical_difference_d6;
        edge_from_d6=preview_d6 && (edge_strength_d6>=edge_threshold);
        comic_edge_from_d6=preview_d6 &&
            ((edge_strength_d6 >= (comic_edge_threshold+10'd16)) ||
             ((edge_strength_d6 >= (comic_edge_threshold+10'd6)) &&
              ((edge_line_previous_d6 && y_d6>12'd78 && x_d6<PREVIEW_WIDTH) ||
               edge_hist1_d7 || edge_hist2_d7)));
        pencil_base_d6=8'd248-
                       (((8'd255-gray_current_d6)>>1)+
                        ((8'd255-gray_current_d6)>>2));
        pencil_hatch_d6=(gray_current_d6<8'd172) &&
                        ((((x_d6[3:0]+y_d6[3:0])&4'h3)==0) ||
                         (((x_d6[3:0]-y_d6[3:0])&4'h7)==0));
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            camera_rgb_d2<=0;comic_left_d2<=0;comic_left2_d2<=0;comic_up_d2<=0;
            comic_upleft_d2<=0;comic_upleft2_d2<=0;x_d2<=0;y_d2<=0;
            hs_d2<=0;vs_d2<=0;de_d2<=0;preview_d2<=0;
            mode_d2<=0;strength_d2<=0;param1_d2<=0;param2_d2<=0;smooth_amount_d2<=0;
            gray_left_in_d2<=0;gray_up_in_d2<=0;
            emboss_cur_gray_d2<=0;emboss_left_gray_d2<=0;
            emboss_up_gray_d2<=0;emboss_upleft_gray_d2<=0;
        end else begin
            camera_rgb_d2<=camera_rgb_d1;
            comic_left_d2<=comic_left;comic_left2_d2<=comic_left2;
            comic_up_d2<=comic_up;comic_upleft_d2<=comic_upleft;
            comic_upleft2_d2<=comic_upleft2;
            x_d2<=x_d1;y_d2<=y_d1;hs_d2<=hs_d1;vs_d2<=vs_d1;de_d2<=de_d1;
            preview_d2<=preview_d1;
            mode_d2<=O_mode;strength_d2<=O_strength;param1_d2<=O_param1;
            param2_d2<=O_param2;smooth_amount_d2<=O_smooth;
            gray_left_in_d2<=gray_left;gray_up_in_d2<=gray_up;
            emboss_cur_gray_d2<=emboss_gray(camera_rgb_d1);
            emboss_left_gray_d2<=emboss_gray(left_rgb_d1);
            emboss_up_gray_d2<=emboss_gray(line_rgb_d1);
            emboss_upleft_gray_d2<=emboss_gray(upper_left_rgb_d1);
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            average_rgb_d3<=0;emboss_rgb_d3<=0;
            camera_rgb_d3<=0;x_d3<=0;y_d3<=0;
            hs_d3<=0;vs_d3<=0;de_d3<=0;preview_d3<=0;
            mode_d3<=0;strength_d3<=0;param1_d3<=0;param2_d3<=0;smooth_amount_d3<=0;
            gray_left_d3<=0;gray_up_d3<=0;
        end else begin
            average_rgb_d3<=average_rgb;
            emboss_rgb_d3<=emboss_rgb;
            camera_rgb_d3<=camera_rgb_d2;
            x_d3<=x_d2;y_d3<=y_d2;hs_d3<=hs_d2;vs_d3<=vs_d2;
            de_d3<=de_d2;preview_d3<=preview_d2;
            mode_d3<=mode_d2;strength_d3<=strength_d2;param1_d3<=param1_d2;
            param2_d3<=param2_d2;smooth_amount_d3<=smooth_amount_d2;
            gray_left_d3<=gray_left_in_d2;gray_up_d3<=gray_up_in_d2;
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            camera_rgb_d4<=0;smooth_rgb_d4<=0;emboss_rgb_d4<=0;
            x_d4<=0;y_d4<=0;hs_d4<=0;vs_d4<=0;de_d4<=0;preview_d4<=0;
            mode_d4<=0;strength_d4<=0;param1_d4<=0;param2_d4<=0;
            gray_left_d4<=0;gray_up_d4<=0;
        end else begin
            camera_rgb_d4<=camera_rgb_d3;smooth_rgb_d4<=smooth_rgb;emboss_rgb_d4<=emboss_rgb_d3;
            x_d4<=x_d3;y_d4<=y_d3;hs_d4<=hs_d3;vs_d4<=vs_d3;de_d4<=de_d3;preview_d4<=preview_d3;
            mode_d4<=mode_d3;strength_d4<=strength_d3;param1_d4<=param1_d3;param2_d4<=param2_d3;
            gray_left_d4<=gray_left_d3;gray_up_d4<=gray_up_d3;
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            camera_rgb_d5<=0;smooth_rgb_d5<=0;emboss_rgb_d5<=0;
            gray_current_d5<=0;gray_left_d5<=0;gray_up_d5<=0;
            x_d5<=0;y_d5<=0;hs_d5<=0;vs_d5<=0;de_d5<=0;preview_d5<=0;
            mode_d5<=0;strength_d5<=0;param1_d5<=0;param2_d5<=0;
        end else begin
            camera_rgb_d5<=camera_rgb_d4;smooth_rgb_d5<=smooth_rgb_d4;emboss_rgb_d5<=emboss_rgb_d4;
            gray_current_d5<=gray_current;gray_left_d5<=gray_left_d4;gray_up_d5<=gray_up_d4;
            x_d5<=x_d4;y_d5<=y_d4;hs_d5<=hs_d4;vs_d5<=vs_d4;de_d5<=de_d4;preview_d5<=preview_d4;
            mode_d5<=mode_d4;strength_d5<=strength_d4;param1_d5<=param1_d4;param2_d5<=param2_d4;
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            horizontal_difference_d6<=0;vertical_difference_d6<=0;edge_line_previous_d6<=0;
            camera_rgb_d6<=0;smooth_rgb_d6<=0;emboss_rgb_d6<=0;gray_current_d6<=0;
            x_d6<=0;y_d6<=0;hs_d6<=0;vs_d6<=0;de_d6<=0;preview_d6<=0;
            mode_d6<=0;strength_d6<=0;param1_d6<=0;param2_d6<=0;
        end else begin
            horizontal_difference_d6<=(x_d5==0 || !preview_d5)?9'd0:horizontal_difference;
            vertical_difference_d6<=(y_d5<=12'd78 || !preview_d5)?9'd0:vertical_difference;
            edge_line_previous_d6<=(preview_d5 && y_d5>12'd78 && x_d5<PREVIEW_WIDTH)?edge_line_px:1'b0;
            camera_rgb_d6<=camera_rgb_d5;smooth_rgb_d6<=smooth_rgb_d5;emboss_rgb_d6<=emboss_rgb_d5;
            gray_current_d6<=gray_current_d5;
            x_d6<=x_d5;y_d6<=y_d5;hs_d6<=hs_d5;vs_d6<=vs_d5;de_d6<=de_d5;preview_d6<=preview_d5;
            mode_d6<=mode_d5;strength_d6<=strength_d5;param1_d6<=param1_d5;param2_d6<=param2_d5;
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            camera_rgb_d7<=0;comic_rgb_d7<=0;comic_edge_d7<=0;blend_strength_d7<=0;
            x_d7<=0;y_d7<=0;hs_d7<=0;vs_d7<=0;de_d7<=0;mode_d7<=0;
            edge_hist1_d7<=0;edge_hist2_d7<=0;
        end else begin
            camera_rgb_d7<=camera_rgb_d6;
            if(!preview_d6) comic_rgb_d7<=24'h000000;
            else if(mode_d6==1) begin
                // Pencil: warm paper, sparse deterministic hatch in shadows,
                // and a darker shared contour from the smoothed gradient.
                if(edge_from_d6) comic_rgb_d7<=24'h4a403c;
                else if(pencil_hatch_d6) begin
                    comic_rgb_d7<={pencil_base_d6-8'd18,
                                   pencil_base_d6-8'd20,
                                   pencil_base_d6-8'd24};
                end else begin
                    comic_rgb_d7<={pencil_base_d6+8'd4,
                                   pencil_base_d6+8'd2,
                                   pencil_base_d6};
                end
            end else if(mode_d6==2) comic_rgb_d7<=emboss_rgb_d6;
            else if(comic_edge_outline_d6) comic_rgb_d7<=24'h101018;
            else begin
                // Quantize each channel after the edge-aware smoothing.  The
                // earlier shared-luma shift preserved nearly all source
                // texture, so the result looked like the original with a
                // slight tint instead of clean comic colour blocks.  These
                // ROM mappings are channel-local and synthesis friendly;
                // the black contour remains a separate branch.
                comic_rgb_d7<={posterize_channel(smooth_rgb_d6[23:16],param2_d6),
                                posterize_channel(smooth_rgb_d6[15:8],param2_d6),
                                posterize_channel(smooth_rgb_d6[7:0],param2_d6)};
            end
            comic_edge_d7<=(mode_d6==3) && comic_edge_outline_d6;
            if((mode_d6==3) && comic_edge_outline_d6)
                blend_strength_d7<=(strength_d6==0)?8'd0:
                                   ((strength_d6>=50)?8'd100:(strength_d6<<1));
            else blend_strength_d7<=strength_d6;
            mode_d7<=mode_d6;
            x_d7<=x_d6;y_d7<=y_d6;hs_d7<=hs_d6;vs_d7<=vs_d6;de_d7<=de_d6;
            if(!preview_d6 || x_d6==0) begin
                edge_hist1_d7<=comic_edge_from_d6;edge_hist2_d7<=1'b0;
            end else begin
                edge_hist1_d7<=comic_edge_from_d6;edge_hist2_d7<=edge_hist1_d7;
            end
            if(preview_d6 && x_d6<PREVIEW_WIDTH)
                edge_line_px<=comic_edge_from_d6;
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            camera_rgb_d8<=0;effect_rgb_d8<=0;x_d8<=0;y_d8<=0;y_content_d8<=0;
            hs_d8<=0;vs_d8<=0;de_d8<=0;mode_d8<=0;
        end else begin
            camera_rgb_d8<=camera_rgb_d7;
            effect_rgb_d8<={blend_fast_channel(camera_rgb_d7[23:16],comic_rgb_d7[23:16],blend_strength_d7),
                            blend_fast_channel(camera_rgb_d7[15:8],comic_rgb_d7[15:8],blend_strength_d7),
                            blend_fast_channel(camera_rgb_d7[7:0],comic_rgb_d7[7:0],blend_strength_d7)};
            x_d8<=x_d7;y_d8<=y_d7;hs_d8<=hs_d7;vs_d8<=vs_d7;de_d8<=de_d7;
            y_content_d8<=y_d7-12'd78;
            mode_d8<=mode_d7;
        end
    end

    localparam [23:0] PANEL=24'h12202b, CANVAS=24'h101a24, INK=24'heaf1f5;
    localparam [23:0] MUTED=24'h91a5b4, ACCENT=24'h00b5b9, SELECT=24'h1c303d;
    localparam [23:0] TRACK=24'h344754, BORDER=24'h344754, WHITE=24'hffffff;
    localparam [23:0] FOCUS=24'hffbd36;

    // Header logo (Anlogic icon, no wordmark).  Generated by
    // sim/tools/build_ui_logo.py -- do not hand-edit between the markers.
    localparam [11:0] LOGO_X0 = 12'd15;
    localparam [11:0] LOGO_Y0 = 12'd14;
    localparam [23:0] LOGO_INK = 24'hf4f494;
    // LOGO_GENERATED_BEGIN
    localparam integer LOGO_W = 72;
    localparam integer LOGO_H = 50;
    localparam integer LOGO_ROWS = 64;
    // 4608 bit，同样改分布式，给 ERAM 让路
    (* ram_style = "dram" *) reg [71:0] logo_rows [0:63];
    initial begin
        logo_rows[0] = 72'h0000013fffffffffff;
        logo_rows[1] = 72'h0000033ffffffffffe;
        logo_rows[2] = 72'h0000039ffffffffffc;
        logo_rows[3] = 72'h000007800000000000;
        logo_rows[4] = 72'h000007c00000000000;
        logo_rows[5] = 72'h00000fcffffffffff8;
        logo_rows[6] = 72'h00000fe7fffffffff8;
        logo_rows[7] = 72'h00001fe7fffffffff0;
        logo_rows[8] = 72'h00001ff00000000000;
        logo_rows[9] = 72'h00003ff00000000000;
        logo_rows[10] = 72'h00003ff80000000000;
        logo_rows[11] = 72'h00007ff9ffffffffc0;
        logo_rows[12] = 72'h00007ffcffffffffc0;
        logo_rows[13] = 72'h0000fffcffffffff80;
        logo_rows[14] = 72'h0000fffe0000000000;
        logo_rows[15] = 72'h0001fffe0000000000;
        logo_rows[16] = 72'h0001ffff3ffffffe00;
        logo_rows[17] = 72'h0003ffff3ffffffe00;
        logo_rows[18] = 72'h0003ffff9ffffffc00;
        logo_rows[19] = 72'h0007ffff8000000000;
        logo_rows[20] = 72'h0007ffffc000000000;
        logo_rows[21] = 72'h000fffffc400001000;
        logo_rows[22] = 72'h000ffcffeffffff800;
        logo_rows[23] = 72'h001ffcffe7fffff000;
        logo_rows[24] = 72'h001ff87ff3ffffe000;
        logo_rows[25] = 72'h003ff87ff000000000;
        logo_rows[26] = 72'h003ff03ff800000000;
        logo_rows[27] = 72'h007ff03ff9ffffc000;
        logo_rows[28] = 72'h007fe01ffcffff8000;
        logo_rows[29] = 72'h00ffe01ffcffff8000;
        logo_rows[30] = 72'h00ffc00ffe00000000;
        logo_rows[31] = 72'h01ffc00ffe00000000;
        logo_rows[32] = 72'h01ff8007ff3ffe0000;
        logo_rows[33] = 72'h03ff8007ff3ffe0000;
        logo_rows[34] = 72'h03ff0003ff9ffc0000;
        logo_rows[35] = 72'h07ff0001ff80000000;
        logo_rows[36] = 72'h07fe0001ffc0000000;
        logo_rows[37] = 72'h0fffffc0ffe0000000;
        logo_rows[38] = 72'h0fffffc0ffeff00000;
        logo_rows[39] = 72'h1fffffe07ff7f00000;
        logo_rows[40] = 72'h1fffffe07ff3e00000;
        logo_rows[41] = 72'h3ffffff03ff8000000;
        logo_rows[42] = 72'h3ffffff03ff8000000;
        logo_rows[43] = 72'h7ffffff81ffdc00000;
        logo_rows[44] = 72'hfffffff81ffd800000;
        logo_rows[45] = 72'hfffffffc0ffe800000;
        logo_rows[46] = 72'h000000000000000000;
        logo_rows[47] = 72'h000000000000000000;
        logo_rows[48] = 72'h000000000000000000;
        logo_rows[49] = 72'h000000000000000000;
        logo_rows[50] = 72'h000000000000000000;
        logo_rows[51] = 72'h000000000000000000;
        logo_rows[52] = 72'h000000000000000000;
        logo_rows[53] = 72'h000000000000000000;
        logo_rows[54] = 72'h000000000000000000;
        logo_rows[55] = 72'h000000000000000000;
        logo_rows[56] = 72'h000000000000000000;
        logo_rows[57] = 72'h000000000000000000;
        logo_rows[58] = 72'h000000000000000000;
        logo_rows[59] = 72'h000000000000000000;
        logo_rows[60] = 72'h000000000000000000;
        logo_rows[61] = 72'h000000000000000000;
        logo_rows[62] = 72'h000000000000000000;
        logo_rows[63] = 72'h000000000000000000;
    end
    // LOGO_GENERATED_END

    // FONT_GENERATED_BEGIN
    localparam integer GLYPH_DIGIT_ZERO = 1;
    localparam integer GLYPH_PERCENT = 11;
    localparam integer GLYPH_DOT = 12;
    localparam integer GLYPH_DASH = 13;
    reg [31:0] font_rows [0:3615];
    initial begin
        font_rows[0] = 32'h00000000;
        font_rows[1] = 32'h00000000;
        font_rows[2] = 32'h00000000;
        font_rows[3] = 32'h00000000;
        font_rows[4] = 32'h00000000;
        font_rows[5] = 32'h00000000;
        font_rows[6] = 32'h00000000;
        font_rows[7] = 32'h00000000;
        font_rows[8] = 32'h00000000;
        font_rows[9] = 32'h00000000;
        font_rows[10] = 32'h00000000;
        font_rows[11] = 32'h00000000;
        font_rows[12] = 32'h00000000;
        font_rows[13] = 32'h00000000;
        font_rows[14] = 32'h00000000;
        font_rows[15] = 32'h00000000;
        font_rows[16] = 32'h00000000;
        font_rows[17] = 32'h00000000;
        font_rows[18] = 32'h00000000;
        font_rows[19] = 32'h00000000;
        font_rows[20] = 32'h00000000;
        font_rows[21] = 32'h00000000;
        font_rows[22] = 32'h00000000;
        font_rows[23] = 32'h00000000;
        font_rows[24] = 32'h00000000;
        font_rows[25] = 32'h00000000;
        font_rows[26] = 32'h00000000;
        font_rows[27] = 32'h00000000;
        font_rows[28] = 32'h00000000;
        font_rows[29] = 32'h00000000;
        font_rows[30] = 32'h00000000;
        font_rows[31] = 32'h00000000;
        font_rows[32] = 32'h00000000;
        font_rows[33] = 32'h00000000;
        font_rows[34] = 32'h00000000;
        font_rows[35] = 32'h00000000;
        font_rows[36] = 32'h00000000;
        font_rows[37] = 32'h0003e000;
        font_rows[38] = 32'h000ff800;
        font_rows[39] = 32'h001e7c00;
        font_rows[40] = 32'h001c1c00;
        font_rows[41] = 32'h00381e00;
        font_rows[42] = 32'h00380e00;
        font_rows[43] = 32'h00380e00;
        font_rows[44] = 32'h00700e00;
        font_rows[45] = 32'h00700e00;
        font_rows[46] = 32'h00700e00;
        font_rows[47] = 32'h00700e00;
        font_rows[48] = 32'h00700e00;
        font_rows[49] = 32'h00700e00;
        font_rows[50] = 32'h00700e00;
        font_rows[51] = 32'h00700e00;
        font_rows[52] = 32'h00380e00;
        font_rows[53] = 32'h00381c00;
        font_rows[54] = 32'h003c1c00;
        font_rows[55] = 32'h001e7800;
        font_rows[56] = 32'h000ff000;
        font_rows[57] = 32'h0007e000;
        font_rows[58] = 32'h00000000;
        font_rows[59] = 32'h00000000;
        font_rows[60] = 32'h00000000;
        font_rows[61] = 32'h00000000;
        font_rows[62] = 32'h00000000;
        font_rows[63] = 32'h00000000;
        font_rows[64] = 32'h00000000;
        font_rows[65] = 32'h00000000;
        font_rows[66] = 32'h00000000;
        font_rows[67] = 32'h00000000;
        font_rows[68] = 32'h00000000;
        font_rows[69] = 32'h0000e000;
        font_rows[70] = 32'h0001e000;
        font_rows[71] = 32'h000fe000;
        font_rows[72] = 32'h001fe000;
        font_rows[73] = 32'h001ce000;
        font_rows[74] = 32'h0010e000;
        font_rows[75] = 32'h0000e000;
        font_rows[76] = 32'h0000e000;
        font_rows[77] = 32'h0000e000;
        font_rows[78] = 32'h0000e000;
        font_rows[79] = 32'h0000e000;
        font_rows[80] = 32'h0000e000;
        font_rows[81] = 32'h0000e000;
        font_rows[82] = 32'h0000e000;
        font_rows[83] = 32'h0000e000;
        font_rows[84] = 32'h0000e000;
        font_rows[85] = 32'h0000e000;
        font_rows[86] = 32'h0000e000;
        font_rows[87] = 32'h0000e000;
        font_rows[88] = 32'h0000e000;
        font_rows[89] = 32'h0000e000;
        font_rows[90] = 32'h00000000;
        font_rows[91] = 32'h00000000;
        font_rows[92] = 32'h00000000;
        font_rows[93] = 32'h00000000;
        font_rows[94] = 32'h00000000;
        font_rows[95] = 32'h00000000;
        font_rows[96] = 32'h00000000;
        font_rows[97] = 32'h00000000;
        font_rows[98] = 32'h00000000;
        font_rows[99] = 32'h00000000;
        font_rows[100] = 32'h00000000;
        font_rows[101] = 32'h0007e000;
        font_rows[102] = 32'h001ff800;
        font_rows[103] = 32'h003e7c00;
        font_rows[104] = 32'h00301c00;
        font_rows[105] = 32'h00001c00;
        font_rows[106] = 32'h00000c00;
        font_rows[107] = 32'h00001c00;
        font_rows[108] = 32'h00001c00;
        font_rows[109] = 32'h00001c00;
        font_rows[110] = 32'h00003800;
        font_rows[111] = 32'h00007800;
        font_rows[112] = 32'h0001f000;
        font_rows[113] = 32'h0003c000;
        font_rows[114] = 32'h00078000;
        font_rows[115] = 32'h000e0000;
        font_rows[116] = 32'h001c0000;
        font_rows[117] = 32'h00380000;
        font_rows[118] = 32'h00380000;
        font_rows[119] = 32'h003ffc00;
        font_rows[120] = 32'h003ffe00;
        font_rows[121] = 32'h007ffe00;
        font_rows[122] = 32'h00000000;
        font_rows[123] = 32'h00000000;
        font_rows[124] = 32'h00000000;
        font_rows[125] = 32'h00000000;
        font_rows[126] = 32'h00000000;
        font_rows[127] = 32'h00000000;
        font_rows[128] = 32'h00000000;
        font_rows[129] = 32'h00000000;
        font_rows[130] = 32'h00000000;
        font_rows[131] = 32'h00000000;
        font_rows[132] = 32'h00000000;
        font_rows[133] = 32'h0007e000;
        font_rows[134] = 32'h001ff000;
        font_rows[135] = 32'h001cf800;
        font_rows[136] = 32'h00103c00;
        font_rows[137] = 32'h00001c00;
        font_rows[138] = 32'h00001c00;
        font_rows[139] = 32'h00001c00;
        font_rows[140] = 32'h00003800;
        font_rows[141] = 32'h00007000;
        font_rows[142] = 32'h000fe000;
        font_rows[143] = 32'h000ff000;
        font_rows[144] = 32'h0000f800;
        font_rows[145] = 32'h00003c00;
        font_rows[146] = 32'h00001c00;
        font_rows[147] = 32'h00001c00;
        font_rows[148] = 32'h00001c00;
        font_rows[149] = 32'h00001c00;
        font_rows[150] = 32'h00203c00;
        font_rows[151] = 32'h0038f800;
        font_rows[152] = 32'h003ff000;
        font_rows[153] = 32'h001fc000;
        font_rows[154] = 32'h00000000;
        font_rows[155] = 32'h00000000;
        font_rows[156] = 32'h00000000;
        font_rows[157] = 32'h00000000;
        font_rows[158] = 32'h00000000;
        font_rows[159] = 32'h00000000;
        font_rows[160] = 32'h00000000;
        font_rows[161] = 32'h00000000;
        font_rows[162] = 32'h00000000;
        font_rows[163] = 32'h00000000;
        font_rows[164] = 32'h00000000;
        font_rows[165] = 32'h00003800;
        font_rows[166] = 32'h00007800;
        font_rows[167] = 32'h0000f800;
        font_rows[168] = 32'h0000f800;
        font_rows[169] = 32'h0001f800;
        font_rows[170] = 32'h0001b800;
        font_rows[171] = 32'h0003b800;
        font_rows[172] = 32'h00073800;
        font_rows[173] = 32'h00063800;
        font_rows[174] = 32'h000e3800;
        font_rows[175] = 32'h001c3800;
        font_rows[176] = 32'h00383800;
        font_rows[177] = 32'h00783800;
        font_rows[178] = 32'h00703800;
        font_rows[179] = 32'h00ffff00;
        font_rows[180] = 32'h00ffff00;
        font_rows[181] = 32'h00003800;
        font_rows[182] = 32'h00003800;
        font_rows[183] = 32'h00003800;
        font_rows[184] = 32'h00003800;
        font_rows[185] = 32'h00003800;
        font_rows[186] = 32'h00000000;
        font_rows[187] = 32'h00000000;
        font_rows[188] = 32'h00000000;
        font_rows[189] = 32'h00000000;
        font_rows[190] = 32'h00000000;
        font_rows[191] = 32'h00000000;
        font_rows[192] = 32'h00000000;
        font_rows[193] = 32'h00000000;
        font_rows[194] = 32'h00000000;
        font_rows[195] = 32'h00000000;
        font_rows[196] = 32'h00000000;
        font_rows[197] = 32'h001ff800;
        font_rows[198] = 32'h001ff800;
        font_rows[199] = 32'h001c0000;
        font_rows[200] = 32'h001c0000;
        font_rows[201] = 32'h001c0000;
        font_rows[202] = 32'h001c0000;
        font_rows[203] = 32'h001c0000;
        font_rows[204] = 32'h00180000;
        font_rows[205] = 32'h001fe000;
        font_rows[206] = 32'h001ff000;
        font_rows[207] = 32'h0011f800;
        font_rows[208] = 32'h00003c00;
        font_rows[209] = 32'h00001c00;
        font_rows[210] = 32'h00001c00;
        font_rows[211] = 32'h00000e00;
        font_rows[212] = 32'h00001c00;
        font_rows[213] = 32'h00001c00;
        font_rows[214] = 32'h00003c00;
        font_rows[215] = 32'h0038f800;
        font_rows[216] = 32'h003ff000;
        font_rows[217] = 32'h001fc000;
        font_rows[218] = 32'h00000000;
        font_rows[219] = 32'h00000000;
        font_rows[220] = 32'h00000000;
        font_rows[221] = 32'h00000000;
        font_rows[222] = 32'h00000000;
        font_rows[223] = 32'h00000000;
        font_rows[224] = 32'h00000000;
        font_rows[225] = 32'h00000000;
        font_rows[226] = 32'h00000000;
        font_rows[227] = 32'h00000000;
        font_rows[228] = 32'h00000000;
        font_rows[229] = 32'h0001f800;
        font_rows[230] = 32'h0003fc00;
        font_rows[231] = 32'h00078c00;
        font_rows[232] = 32'h000e0000;
        font_rows[233] = 32'h001c0000;
        font_rows[234] = 32'h001c0000;
        font_rows[235] = 32'h00380000;
        font_rows[236] = 32'h00380000;
        font_rows[237] = 32'h0039f000;
        font_rows[238] = 32'h003ff800;
        font_rows[239] = 32'h003e3c00;
        font_rows[240] = 32'h003c1e00;
        font_rows[241] = 32'h00380e00;
        font_rows[242] = 32'h00380e00;
        font_rows[243] = 32'h00380e00;
        font_rows[244] = 32'h00380e00;
        font_rows[245] = 32'h00380e00;
        font_rows[246] = 32'h001c1e00;
        font_rows[247] = 32'h001f3c00;
        font_rows[248] = 32'h000ff800;
        font_rows[249] = 32'h0003e000;
        font_rows[250] = 32'h00000000;
        font_rows[251] = 32'h00000000;
        font_rows[252] = 32'h00000000;
        font_rows[253] = 32'h00000000;
        font_rows[254] = 32'h00000000;
        font_rows[255] = 32'h00000000;
        font_rows[256] = 32'h00000000;
        font_rows[257] = 32'h00000000;
        font_rows[258] = 32'h00000000;
        font_rows[259] = 32'h00000000;
        font_rows[260] = 32'h00000000;
        font_rows[261] = 32'h007ffe00;
        font_rows[262] = 32'h007ffe00;
        font_rows[263] = 32'h00001e00;
        font_rows[264] = 32'h00001c00;
        font_rows[265] = 32'h00001c00;
        font_rows[266] = 32'h00003800;
        font_rows[267] = 32'h00003000;
        font_rows[268] = 32'h00007000;
        font_rows[269] = 32'h00007000;
        font_rows[270] = 32'h0000e000;
        font_rows[271] = 32'h0000e000;
        font_rows[272] = 32'h0001c000;
        font_rows[273] = 32'h0001c000;
        font_rows[274] = 32'h00038000;
        font_rows[275] = 32'h00038000;
        font_rows[276] = 32'h00038000;
        font_rows[277] = 32'h00070000;
        font_rows[278] = 32'h00070000;
        font_rows[279] = 32'h00070000;
        font_rows[280] = 32'h00070000;
        font_rows[281] = 32'h00070000;
        font_rows[282] = 32'h00000000;
        font_rows[283] = 32'h00000000;
        font_rows[284] = 32'h00000000;
        font_rows[285] = 32'h00000000;
        font_rows[286] = 32'h00000000;
        font_rows[287] = 32'h00000000;
        font_rows[288] = 32'h00000000;
        font_rows[289] = 32'h00000000;
        font_rows[290] = 32'h00000000;
        font_rows[291] = 32'h00000000;
        font_rows[292] = 32'h00000000;
        font_rows[293] = 32'h0007e000;
        font_rows[294] = 32'h000ff800;
        font_rows[295] = 32'h001e7800;
        font_rows[296] = 32'h003c1c00;
        font_rows[297] = 32'h00381c00;
        font_rows[298] = 32'h00380c00;
        font_rows[299] = 32'h00381c00;
        font_rows[300] = 32'h00181c00;
        font_rows[301] = 32'h000e3800;
        font_rows[302] = 32'h0007f000;
        font_rows[303] = 32'h000ff800;
        font_rows[304] = 32'h001e7c00;
        font_rows[305] = 32'h00381e00;
        font_rows[306] = 32'h00700e00;
        font_rows[307] = 32'h00700e00;
        font_rows[308] = 32'h00700e00;
        font_rows[309] = 32'h00700e00;
        font_rows[310] = 32'h00381e00;
        font_rows[311] = 32'h003e7c00;
        font_rows[312] = 32'h001ff800;
        font_rows[313] = 32'h0007e000;
        font_rows[314] = 32'h00000000;
        font_rows[315] = 32'h00000000;
        font_rows[316] = 32'h00000000;
        font_rows[317] = 32'h00000000;
        font_rows[318] = 32'h00000000;
        font_rows[319] = 32'h00000000;
        font_rows[320] = 32'h00000000;
        font_rows[321] = 32'h00000000;
        font_rows[322] = 32'h00000000;
        font_rows[323] = 32'h00000000;
        font_rows[324] = 32'h00000000;
        font_rows[325] = 32'h0007e000;
        font_rows[326] = 32'h000ff000;
        font_rows[327] = 32'h001e7800;
        font_rows[328] = 32'h00381c00;
        font_rows[329] = 32'h00381c00;
        font_rows[330] = 32'h00700e00;
        font_rows[331] = 32'h00700e00;
        font_rows[332] = 32'h00700e00;
        font_rows[333] = 32'h00780e00;
        font_rows[334] = 32'h00381e00;
        font_rows[335] = 32'h003c3e00;
        font_rows[336] = 32'h001ffe00;
        font_rows[337] = 32'h000fee00;
        font_rows[338] = 32'h00000e00;
        font_rows[339] = 32'h00000c00;
        font_rows[340] = 32'h00001c00;
        font_rows[341] = 32'h00001c00;
        font_rows[342] = 32'h00003800;
        font_rows[343] = 32'h0038f000;
        font_rows[344] = 32'h003fe000;
        font_rows[345] = 32'h001fc000;
        font_rows[346] = 32'h00000000;
        font_rows[347] = 32'h00000000;
        font_rows[348] = 32'h00000000;
        font_rows[349] = 32'h00000000;
        font_rows[350] = 32'h00000000;
        font_rows[351] = 32'h00000000;
        font_rows[352] = 32'h00000000;
        font_rows[353] = 32'h00000000;
        font_rows[354] = 32'h00000000;
        font_rows[355] = 32'h00000000;
        font_rows[356] = 32'h00000000;
        font_rows[357] = 32'h01f00600;
        font_rows[358] = 32'h07f80c00;
        font_rows[359] = 32'h061c1c00;
        font_rows[360] = 32'h0e0c1800;
        font_rows[361] = 32'h0e0c3800;
        font_rows[362] = 32'h0c0c7000;
        font_rows[363] = 32'h0e0c6000;
        font_rows[364] = 32'h0e1ce000;
        font_rows[365] = 32'h0739c000;
        font_rows[366] = 32'h03f98000;
        font_rows[367] = 32'h01e39f00;
        font_rows[368] = 32'h00073fc0;
        font_rows[369] = 32'h000631c0;
        font_rows[370] = 32'h000e70e0;
        font_rows[371] = 32'h000c60e0;
        font_rows[372] = 32'h00186060;
        font_rows[373] = 32'h003860e0;
        font_rows[374] = 32'h003060e0;
        font_rows[375] = 32'h006071c0;
        font_rows[376] = 32'h00e03f80;
        font_rows[377] = 32'h00c01f00;
        font_rows[378] = 32'h00000000;
        font_rows[379] = 32'h00000000;
        font_rows[380] = 32'h00000000;
        font_rows[381] = 32'h00000000;
        font_rows[382] = 32'h00000000;
        font_rows[383] = 32'h00000000;
        font_rows[384] = 32'h00000000;
        font_rows[385] = 32'h00000000;
        font_rows[386] = 32'h00000000;
        font_rows[387] = 32'h00000000;
        font_rows[388] = 32'h00000000;
        font_rows[389] = 32'h00000000;
        font_rows[390] = 32'h00000000;
        font_rows[391] = 32'h00000000;
        font_rows[392] = 32'h00000000;
        font_rows[393] = 32'h00000000;
        font_rows[394] = 32'h00000000;
        font_rows[395] = 32'h00000000;
        font_rows[396] = 32'h00000000;
        font_rows[397] = 32'h00000000;
        font_rows[398] = 32'h00010000;
        font_rows[399] = 32'h00038000;
        font_rows[400] = 32'h00038000;
        font_rows[401] = 32'h00038000;
        font_rows[402] = 32'h00000000;
        font_rows[403] = 32'h00000000;
        font_rows[404] = 32'h00000000;
        font_rows[405] = 32'h00000000;
        font_rows[406] = 32'h00000000;
        font_rows[407] = 32'h00000000;
        font_rows[408] = 32'h00000000;
        font_rows[409] = 32'h00000000;
        font_rows[410] = 32'h00000000;
        font_rows[411] = 32'h00000000;
        font_rows[412] = 32'h00000000;
        font_rows[413] = 32'h00000000;
        font_rows[414] = 32'h00000000;
        font_rows[415] = 32'h00000000;
        font_rows[416] = 32'h00000000;
        font_rows[417] = 32'h00000000;
        font_rows[418] = 32'h00000000;
        font_rows[419] = 32'h00000000;
        font_rows[420] = 32'h00000000;
        font_rows[421] = 32'h00000000;
        font_rows[422] = 32'h00000000;
        font_rows[423] = 32'h00000000;
        font_rows[424] = 32'h00000000;
        font_rows[425] = 32'h00000000;
        font_rows[426] = 32'h00000000;
        font_rows[427] = 32'h000ff000;
        font_rows[428] = 32'h000ff000;
        font_rows[429] = 32'h00000000;
        font_rows[430] = 32'h00000000;
        font_rows[431] = 32'h00000000;
        font_rows[432] = 32'h00000000;
        font_rows[433] = 32'h00000000;
        font_rows[434] = 32'h00000000;
        font_rows[435] = 32'h00000000;
        font_rows[436] = 32'h00000000;
        font_rows[437] = 32'h00000000;
        font_rows[438] = 32'h00000000;
        font_rows[439] = 32'h00000000;
        font_rows[440] = 32'h00000000;
        font_rows[441] = 32'h00000000;
        font_rows[442] = 32'h00000000;
        font_rows[443] = 32'h00000000;
        font_rows[444] = 32'h00000000;
        font_rows[445] = 32'h00000000;
        font_rows[446] = 32'h00000000;
        font_rows[447] = 32'h00000000;
        font_rows[448] = 32'h00000000;
        font_rows[449] = 32'h00000000;
        font_rows[450] = 32'h00000000;
        font_rows[451] = 32'h00018000;
        font_rows[452] = 32'h0003c000;
        font_rows[453] = 32'h0001c000;
        font_rows[454] = 32'h1ffffff8;
        font_rows[455] = 32'h1ffffff8;
        font_rows[456] = 32'h1c000038;
        font_rows[457] = 32'h1c407038;
        font_rows[458] = 32'h1cf07038;
        font_rows[459] = 32'h00787000;
        font_rows[460] = 32'h001c7000;
        font_rows[461] = 32'h030e7000;
        font_rows[462] = 32'h03c47000;
        font_rows[463] = 32'h01e07000;
        font_rows[464] = 32'h00786000;
        font_rows[465] = 32'h00106000;
        font_rows[466] = 32'h00006000;
        font_rows[467] = 32'h1ffffff8;
        font_rows[468] = 32'h1ffffff8;
        font_rows[469] = 32'h0001c000;
        font_rows[470] = 32'h0003e000;
        font_rows[471] = 32'h0007f800;
        font_rows[472] = 32'h001e3e00;
        font_rows[473] = 32'h00fc0fc0;
        font_rows[474] = 32'h07f003f0;
        font_rows[475] = 32'h1fc000f8;
        font_rows[476] = 32'h1c000010;
        font_rows[477] = 32'h00000000;
        font_rows[478] = 32'h00000000;
        font_rows[479] = 32'h00000000;
        font_rows[480] = 32'h00000000;
        font_rows[481] = 32'h00000000;
        font_rows[482] = 32'h00000000;
        font_rows[483] = 32'h000001c0;
        font_rows[484] = 32'h000001c0;
        font_rows[485] = 32'h000001c0;
        font_rows[486] = 32'h1ff801c0;
        font_rows[487] = 32'h1ff801c0;
        font_rows[488] = 32'h1c1801c0;
        font_rows[489] = 32'h1c1bfffc;
        font_rows[490] = 32'h1c1bfffc;
        font_rows[491] = 32'h1c1801c0;
        font_rows[492] = 32'h1c1801c0;
        font_rows[493] = 32'h1c1881c0;
        font_rows[494] = 32'h1ff9c1c0;
        font_rows[495] = 32'h1ff8c1c0;
        font_rows[496] = 32'h1c18e1c0;
        font_rows[497] = 32'h1c1871c0;
        font_rows[498] = 32'h1c1871c0;
        font_rows[499] = 32'h1c1821c0;
        font_rows[500] = 32'h1c1801c0;
        font_rows[501] = 32'h1c1801c0;
        font_rows[502] = 32'h1c1801c0;
        font_rows[503] = 32'h1ff801c0;
        font_rows[504] = 32'h1ff801c0;
        font_rows[505] = 32'h1c1801c0;
        font_rows[506] = 32'h1c187f80;
        font_rows[507] = 32'h00007f80;
        font_rows[508] = 32'h00000000;
        font_rows[509] = 32'h00000000;
        font_rows[510] = 32'h00000000;
        font_rows[511] = 32'h00000000;
        font_rows[512] = 32'h00000000;
        font_rows[513] = 32'h00000000;
        font_rows[514] = 32'h00300c00;
        font_rows[515] = 32'h00300c00;
        font_rows[516] = 32'h00300c00;
        font_rows[517] = 32'h00300c00;
        font_rows[518] = 32'h3ffffffc;
        font_rows[519] = 32'h3ffffffc;
        font_rows[520] = 32'h00300c00;
        font_rows[521] = 32'h00300c00;
        font_rows[522] = 32'h00300c00;
        font_rows[523] = 32'h00000000;
        font_rows[524] = 32'h00000000;
        font_rows[525] = 32'h07ffff80;
        font_rows[526] = 32'h07ffff80;
        font_rows[527] = 32'h00001f00;
        font_rows[528] = 32'h00007c00;
        font_rows[529] = 32'h0001f000;
        font_rows[530] = 32'h0007c000;
        font_rows[531] = 32'h001f0000;
        font_rows[532] = 32'h003e0000;
        font_rows[533] = 32'h00f80018;
        font_rows[534] = 32'h01e00038;
        font_rows[535] = 32'h03c00038;
        font_rows[536] = 32'h07000038;
        font_rows[537] = 32'h07000038;
        font_rows[538] = 32'h070000f0;
        font_rows[539] = 32'h07fffff0;
        font_rows[540] = 32'h01ffffc0;
        font_rows[541] = 32'h00000000;
        font_rows[542] = 32'h00000000;
        font_rows[543] = 32'h00000000;
        font_rows[544] = 32'h00000000;
        font_rows[545] = 32'h00000000;
        font_rows[546] = 32'h00000000;
        font_rows[547] = 32'h00018400;
        font_rows[548] = 32'h00018f00;
        font_rows[549] = 32'h00018780;
        font_rows[550] = 32'h000183c0;
        font_rows[551] = 32'h00018080;
        font_rows[552] = 32'h00018000;
        font_rows[553] = 32'h1ffffff8;
        font_rows[554] = 32'h1ffffff8;
        font_rows[555] = 32'h0007e000;
        font_rows[556] = 32'h0007e000;
        font_rows[557] = 32'h000db000;
        font_rows[558] = 32'h000db000;
        font_rows[559] = 32'h00199800;
        font_rows[560] = 32'h00399c00;
        font_rows[561] = 32'h00718c00;
        font_rows[562] = 32'h00e18600;
        font_rows[563] = 32'h01c18700;
        font_rows[564] = 32'h03c18380;
        font_rows[565] = 32'h078181c0;
        font_rows[566] = 32'h0f0180f0;
        font_rows[567] = 32'h1e018078;
        font_rows[568] = 32'h3c01803c;
        font_rows[569] = 32'h10018018;
        font_rows[570] = 32'h00018000;
        font_rows[571] = 32'h00018000;
        font_rows[572] = 32'h00018000;
        font_rows[573] = 32'h00000000;
        font_rows[574] = 32'h00000000;
        font_rows[575] = 32'h00000000;
        font_rows[576] = 32'h00000000;
        font_rows[577] = 32'h00000000;
        font_rows[578] = 32'h0c007000;
        font_rows[579] = 32'h0e007000;
        font_rows[580] = 32'h07007ff0;
        font_rows[581] = 32'h03807ff0;
        font_rows[582] = 32'h01807000;
        font_rows[583] = 32'h003ffff8;
        font_rows[584] = 32'h003ffff8;
        font_rows[585] = 32'h18307018;
        font_rows[586] = 32'h3c307038;
        font_rows[587] = 32'h1e307030;
        font_rows[588] = 32'h0f307f80;
        font_rows[589] = 32'h0737ffb0;
        font_rows[590] = 32'h02337030;
        font_rows[591] = 32'h00307070;
        font_rows[592] = 32'h00303fe0;
        font_rows[593] = 32'h00301fc0;
        font_rows[594] = 32'h06301800;
        font_rows[595] = 32'h07301c00;
        font_rows[596] = 32'h07306c30;
        font_rows[597] = 32'h06766e30;
        font_rows[598] = 32'h06776438;
        font_rows[599] = 32'h0e6660b8;
        font_rows[600] = 32'h0eee60d8;
        font_rows[601] = 32'h0cec61dc;
        font_rows[602] = 32'h1dcc61d8;
        font_rows[603] = 32'h1d807f80;
        font_rows[604] = 32'h0d803f00;
        font_rows[605] = 32'h00000000;
        font_rows[606] = 32'h00000000;
        font_rows[607] = 32'h00000000;
        font_rows[608] = 32'h00000000;
        font_rows[609] = 32'h00000000;
        font_rows[610] = 32'h00000000;
        font_rows[611] = 32'h07001000;
        font_rows[612] = 32'h07003800;
        font_rows[613] = 32'h06001800;
        font_rows[614] = 32'h060ffff0;
        font_rows[615] = 32'h0feffff0;
        font_rows[616] = 32'h0fe18180;
        font_rows[617] = 32'h1c018380;
        font_rows[618] = 32'h1801c300;
        font_rows[619] = 32'h381ffff8;
        font_rows[620] = 32'h1ffffff8;
        font_rows[621] = 32'h0fe00000;
        font_rows[622] = 32'h03000000;
        font_rows[623] = 32'h030fffe0;
        font_rows[624] = 32'h030fffe0;
        font_rows[625] = 32'h3ffe0060;
        font_rows[626] = 32'h1fefffe0;
        font_rows[627] = 32'h030fffe0;
        font_rows[628] = 32'h030e0060;
        font_rows[629] = 32'h030fffe0;
        font_rows[630] = 32'h030fffe0;
        font_rows[631] = 32'h0324c608;
        font_rows[632] = 32'h0370c61c;
        font_rows[633] = 32'h03e1c618;
        font_rows[634] = 32'h03c78618;
        font_rows[635] = 32'h073f07f8;
        font_rows[636] = 32'h021803f0;
        font_rows[637] = 32'h00000000;
        font_rows[638] = 32'h00000000;
        font_rows[639] = 32'h00000000;
        font_rows[640] = 32'h00000000;
        font_rows[641] = 32'h00000000;
        font_rows[642] = 32'h01800000;
        font_rows[643] = 32'h01800000;
        font_rows[644] = 32'h0183fff0;
        font_rows[645] = 32'h0183fff0;
        font_rows[646] = 32'h01830030;
        font_rows[647] = 32'h01830030;
        font_rows[648] = 32'h1ffb0030;
        font_rows[649] = 32'h1ffb0030;
        font_rows[650] = 32'h01830030;
        font_rows[651] = 32'h0383fff0;
        font_rows[652] = 32'h0383fff0;
        font_rows[653] = 32'h07e30030;
        font_rows[654] = 32'h07f30030;
        font_rows[655] = 32'h0fbb0030;
        font_rows[656] = 32'h0d9b0030;
        font_rows[657] = 32'h1d9b0030;
        font_rows[658] = 32'h3983fff0;
        font_rows[659] = 32'h1983fff0;
        font_rows[660] = 32'h11830030;
        font_rows[661] = 32'h01830030;
        font_rows[662] = 32'h01830030;
        font_rows[663] = 32'h01830030;
        font_rows[664] = 32'h01830030;
        font_rows[665] = 32'h0183fff0;
        font_rows[666] = 32'h0183fff0;
        font_rows[667] = 32'h01830030;
        font_rows[668] = 32'h01830030;
        font_rows[669] = 32'h00000000;
        font_rows[670] = 32'h00000000;
        font_rows[671] = 32'h00000000;
        font_rows[672] = 32'h00000000;
        font_rows[673] = 32'h00000000;
        font_rows[674] = 32'h00000000;
        font_rows[675] = 32'h01800000;
        font_rows[676] = 32'h01800000;
        font_rows[677] = 32'h0183ff80;
        font_rows[678] = 32'h0183ff80;
        font_rows[679] = 32'h01838380;
        font_rows[680] = 32'h3ffb8380;
        font_rows[681] = 32'h1ffb8380;
        font_rows[682] = 32'h03838380;
        font_rows[683] = 32'h03838380;
        font_rows[684] = 32'h03838380;
        font_rows[685] = 32'h07e38380;
        font_rows[686] = 32'h07e38380;
        font_rows[687] = 32'h07f38380;
        font_rows[688] = 32'h0dbb8380;
        font_rows[689] = 32'h0d9b8380;
        font_rows[690] = 32'h19838380;
        font_rows[691] = 32'h39838380;
        font_rows[692] = 32'h11830388;
        font_rows[693] = 32'h1183038c;
        font_rows[694] = 32'h0187038c;
        font_rows[695] = 32'h0187039c;
        font_rows[696] = 32'h0186039c;
        font_rows[697] = 32'h018e0398;
        font_rows[698] = 32'h019c01f8;
        font_rows[699] = 32'h01b801f0;
        font_rows[700] = 32'h01980000;
        font_rows[701] = 32'h00000000;
        font_rows[702] = 32'h00000000;
        font_rows[703] = 32'h00000000;
        font_rows[704] = 32'h00000000;
        font_rows[705] = 32'h00000000;
        font_rows[706] = 32'h00000000;
        font_rows[707] = 32'h00000000;
        font_rows[708] = 32'h00000000;
        font_rows[709] = 32'h00000000;
        font_rows[710] = 32'h00000000;
        font_rows[711] = 32'h00000000;
        font_rows[712] = 32'h00200800;
        font_rows[713] = 32'h00701c00;
        font_rows[714] = 32'h00383800;
        font_rows[715] = 32'h001c7000;
        font_rows[716] = 32'h000ee000;
        font_rows[717] = 32'h0007c000;
        font_rows[718] = 32'h00038000;
        font_rows[719] = 32'h0007c000;
        font_rows[720] = 32'h000ee000;
        font_rows[721] = 32'h001c7000;
        font_rows[722] = 32'h00383800;
        font_rows[723] = 32'h00701c00;
        font_rows[724] = 32'h00200800;
        font_rows[725] = 32'h00000000;
        font_rows[726] = 32'h00000000;
        font_rows[727] = 32'h00000000;
        font_rows[728] = 32'h00000000;
        font_rows[729] = 32'h00000000;
        font_rows[730] = 32'h00000000;
        font_rows[731] = 32'h00000000;
        font_rows[732] = 32'h00000000;
        font_rows[733] = 32'h00000000;
        font_rows[734] = 32'h00000000;
        font_rows[735] = 32'h00000000;
        font_rows[736] = 32'h00000000;
        font_rows[737] = 32'h00000000;
        font_rows[738] = 32'h00000000;
        font_rows[739] = 32'h01800c00;
        font_rows[740] = 32'h01800c00;
        font_rows[741] = 32'h01800c00;
        font_rows[742] = 32'h01800ffc;
        font_rows[743] = 32'h01800ffc;
        font_rows[744] = 32'h0ff80c00;
        font_rows[745] = 32'h0ff80c00;
        font_rows[746] = 32'h0d980c00;
        font_rows[747] = 32'h0d99fff0;
        font_rows[748] = 32'h0d99fff0;
        font_rows[749] = 32'h0d998030;
        font_rows[750] = 32'h0d998e30;
        font_rows[751] = 32'h0d998e30;
        font_rows[752] = 32'h0d998e30;
        font_rows[753] = 32'h0d998c30;
        font_rows[754] = 32'h0d998c30;
        font_rows[755] = 32'h0d998c30;
        font_rows[756] = 32'h0df98c30;
        font_rows[757] = 32'h0df18c30;
        font_rows[758] = 32'h01801e00;
        font_rows[759] = 32'h01803f80;
        font_rows[760] = 32'h018073c0;
        font_rows[761] = 32'h0183e0f0;
        font_rows[762] = 32'h018f8038;
        font_rows[763] = 32'h01860018;
        font_rows[764] = 32'h00000000;
        font_rows[765] = 32'h00000000;
        font_rows[766] = 32'h00000000;
        font_rows[767] = 32'h00000000;
        font_rows[768] = 32'h00000000;
        font_rows[769] = 32'h00000000;
        font_rows[770] = 32'h00000000;
        font_rows[771] = 32'h00010000;
        font_rows[772] = 32'h00038000;
        font_rows[773] = 32'h0001c000;
        font_rows[774] = 32'h1ffffff8;
        font_rows[775] = 32'h1ffffff8;
        font_rows[776] = 32'h00030000;
        font_rows[777] = 32'h0c070020;
        font_rows[778] = 32'h0e0e3070;
        font_rows[779] = 32'h071c78e0;
        font_rows[780] = 32'h03bff3c0;
        font_rows[781] = 32'h013fe180;
        font_rows[782] = 32'h0003c000;
        font_rows[783] = 32'h00c73100;
        font_rows[784] = 32'h03ce3bc0;
        font_rows[785] = 32'h0f9c18f0;
        font_rows[786] = 32'h1e3ffc38;
        font_rows[787] = 32'h183fce18;
        font_rows[788] = 32'h00018400;
        font_rows[789] = 32'h00018000;
        font_rows[790] = 32'h3ffffffc;
        font_rows[791] = 32'h1ffffffc;
        font_rows[792] = 32'h00018000;
        font_rows[793] = 32'h00018000;
        font_rows[794] = 32'h00018000;
        font_rows[795] = 32'h00018000;
        font_rows[796] = 32'h00018000;
        font_rows[797] = 32'h00000000;
        font_rows[798] = 32'h00000000;
        font_rows[799] = 32'h00000000;
        font_rows[800] = 32'h00000000;
        font_rows[801] = 32'h00000000;
        font_rows[802] = 32'h00000000;
        font_rows[803] = 32'h00060000;
        font_rows[804] = 32'h001e1800;
        font_rows[805] = 32'h003c3c00;
        font_rows[806] = 32'h00701e00;
        font_rows[807] = 32'h01e00700;
        font_rows[808] = 32'h03ffffc0;
        font_rows[809] = 32'h03ffffc0;
        font_rows[810] = 32'h000600c0;
        font_rows[811] = 32'h000e0000;
        font_rows[812] = 32'h1ffffff8;
        font_rows[813] = 32'h1ffffff8;
        font_rows[814] = 32'h00701c00;
        font_rows[815] = 32'h00e0cf00;
        font_rows[816] = 32'h03c1e380;
        font_rows[817] = 32'h0f0f81f0;
        font_rows[818] = 32'h3e3e1878;
        font_rows[819] = 32'h19f87c18;
        font_rows[820] = 32'h00c1f000;
        font_rows[821] = 32'h000fc100;
        font_rows[822] = 32'h00ff0780;
        font_rows[823] = 32'h03f81e00;
        font_rows[824] = 32'h01c0fc00;
        font_rows[825] = 32'h0007e000;
        font_rows[826] = 32'h007f8000;
        font_rows[827] = 32'h03fc0000;
        font_rows[828] = 32'h03c00000;
        font_rows[829] = 32'h00000000;
        font_rows[830] = 32'h00000000;
        font_rows[831] = 32'h00000000;
        font_rows[832] = 32'h00000000;
        font_rows[833] = 32'h00000000;
        font_rows[834] = 32'h00600c00;
        font_rows[835] = 32'h0c610c00;
        font_rows[836] = 32'h0e638c00;
        font_rows[837] = 32'h07671c00;
        font_rows[838] = 32'h02661800;
        font_rows[839] = 32'h00601ffc;
        font_rows[840] = 32'h1fffbffc;
        font_rows[841] = 32'h1fffb070;
        font_rows[842] = 32'h00e03070;
        font_rows[843] = 32'h03f87870;
        font_rows[844] = 32'h077ef860;
        font_rows[845] = 32'h0e67d860;
        font_rows[846] = 32'h3c635860;
        font_rows[847] = 32'h10c018e0;
        font_rows[848] = 32'h01c01ce0;
        font_rows[849] = 32'h01800cc0;
        font_rows[850] = 32'h1fff0dc0;
        font_rows[851] = 32'h1fff0f80;
        font_rows[852] = 32'h07070780;
        font_rows[853] = 32'h0e0e0780;
        font_rows[854] = 32'h079c0780;
        font_rows[855] = 32'h01f80fc0;
        font_rows[856] = 32'h00fc1de0;
        font_rows[857] = 32'h01ff38f0;
        font_rows[858] = 32'h0f867078;
        font_rows[859] = 32'h3f00e03c;
        font_rows[860] = 32'h1800c008;
        font_rows[861] = 32'h00000000;
        font_rows[862] = 32'h00000000;
        font_rows[863] = 32'h00000000;
        font_rows[864] = 32'h00000000;
        font_rows[865] = 32'h00000000;
        font_rows[866] = 32'h04000000;
        font_rows[867] = 32'h0e1ffff0;
        font_rows[868] = 32'h071ffff0;
        font_rows[869] = 32'h039c0030;
        font_rows[870] = 32'h01dc1830;
        font_rows[871] = 32'h009c1830;
        font_rows[872] = 32'h001dffb0;
        font_rows[873] = 32'h001dffb0;
        font_rows[874] = 32'h3f1c1830;
        font_rows[875] = 32'h3f1c1830;
        font_rows[876] = 32'h071ffff0;
        font_rows[877] = 32'h071fffb0;
        font_rows[878] = 32'h071c0030;
        font_rows[879] = 32'h071c0030;
        font_rows[880] = 32'h0719ff30;
        font_rows[881] = 32'h0719ff30;
        font_rows[882] = 32'h0739c330;
        font_rows[883] = 32'h0779c330;
        font_rows[884] = 32'h07f9c330;
        font_rows[885] = 32'h07d9c330;
        font_rows[886] = 32'h0739ff30;
        font_rows[887] = 32'h0631ff30;
        font_rows[888] = 32'h04318030;
        font_rows[889] = 32'h00700030;
        font_rows[890] = 32'h00e007f0;
        font_rows[891] = 32'h00c007e0;
        font_rows[892] = 32'h00c00000;
        font_rows[893] = 32'h00000000;
        font_rows[894] = 32'h00000000;
        font_rows[895] = 32'h00000000;
        font_rows[896] = 32'h00000000;
        font_rows[897] = 32'h00000000;
        font_rows[898] = 32'h00300c00;
        font_rows[899] = 32'h00300c00;
        font_rows[900] = 32'h00300c00;
        font_rows[901] = 32'h00300c00;
        font_rows[902] = 32'h3ffffffc;
        font_rows[903] = 32'h3ffffffc;
        font_rows[904] = 32'h00300c00;
        font_rows[905] = 32'h00300c00;
        font_rows[906] = 32'h00300c00;
        font_rows[907] = 32'h00000000;
        font_rows[908] = 32'h00000000;
        font_rows[909] = 32'h0fffffe0;
        font_rows[910] = 32'h0fffffe0;
        font_rows[911] = 32'h001800e0;
        font_rows[912] = 32'h001800c0;
        font_rows[913] = 32'h001800c0;
        font_rows[914] = 32'h001800c0;
        font_rows[915] = 32'h001800c0;
        font_rows[916] = 32'h001800c0;
        font_rows[917] = 32'h001800c0;
        font_rows[918] = 32'h001801c0;
        font_rows[919] = 32'h001801c0;
        font_rows[920] = 32'h00187f80;
        font_rows[921] = 32'h00187f00;
        font_rows[922] = 32'h00180000;
        font_rows[923] = 32'h00180000;
        font_rows[924] = 32'h00180000;
        font_rows[925] = 32'h00180000;
        font_rows[926] = 32'h00000000;
        font_rows[927] = 32'h00000000;
        font_rows[928] = 32'h00000000;
        font_rows[929] = 32'h00000000;
        font_rows[930] = 32'h00000000;
        font_rows[931] = 32'h1fe3ffe0;
        font_rows[932] = 32'h1fe3ffe0;
        font_rows[933] = 32'h00630060;
        font_rows[934] = 32'h00630060;
        font_rows[935] = 32'h00630060;
        font_rows[936] = 32'h0063ffe0;
        font_rows[937] = 32'h0fe3ffe0;
        font_rows[938] = 32'h0fe31c60;
        font_rows[939] = 32'h1c601c00;
        font_rows[940] = 32'h1c001c00;
        font_rows[941] = 32'h1807fff0;
        font_rows[942] = 32'h1807fff0;
        font_rows[943] = 32'h1fe61c30;
        font_rows[944] = 32'h1fe61c30;
        font_rows[945] = 32'h00661c30;
        font_rows[946] = 32'h0067fff0;
        font_rows[947] = 32'h0067fff0;
        font_rows[948] = 32'h00661c00;
        font_rows[949] = 32'h00e01c60;
        font_rows[950] = 32'h00e01c70;
        font_rows[951] = 32'h00e01c30;
        font_rows[952] = 32'h00c7fff8;
        font_rows[953] = 32'h1fdffff8;
        font_rows[954] = 32'h0f8fc01c;
        font_rows[955] = 32'h00000018;
        font_rows[956] = 32'h00000000;
        font_rows[957] = 32'h00000000;
        font_rows[958] = 32'h00000000;
        font_rows[959] = 32'h00000000;
        font_rows[960] = 32'h00000000;
        font_rows[961] = 32'h00000000;
        font_rows[962] = 32'h0000c000;
        font_rows[963] = 32'h0001c000;
        font_rows[964] = 32'h0000e000;
        font_rows[965] = 32'h07fffff8;
        font_rows[966] = 32'h07fffff8;
        font_rows[967] = 32'h06000000;
        font_rows[968] = 32'h060c0300;
        font_rows[969] = 32'h060c0300;
        font_rows[970] = 32'h07fffff8;
        font_rows[971] = 32'h07fffff8;
        font_rows[972] = 32'h060c0300;
        font_rows[973] = 32'h060c0300;
        font_rows[974] = 32'h060fff00;
        font_rows[975] = 32'h060fff00;
        font_rows[976] = 32'h060c0300;
        font_rows[977] = 32'h06000000;
        font_rows[978] = 32'h067fffc0;
        font_rows[979] = 32'h067fffc0;
        font_rows[980] = 32'h061c0380;
        font_rows[981] = 32'h060e0700;
        font_rows[982] = 32'h0e079e00;
        font_rows[983] = 32'h0c03fc00;
        font_rows[984] = 32'h0c01f800;
        font_rows[985] = 32'h1c07fe00;
        font_rows[986] = 32'h187f1fc0;
        font_rows[987] = 32'h39fc03f8;
        font_rows[988] = 32'h11c00078;
        font_rows[989] = 32'h00000000;
        font_rows[990] = 32'h00000000;
        font_rows[991] = 32'h00000000;
        font_rows[992] = 32'h00000000;
        font_rows[993] = 32'h00000000;
        font_rows[994] = 32'h00000000;
        font_rows[995] = 32'h07fffff8;
        font_rows[996] = 32'h07fffff8;
        font_rows[997] = 32'h0700e000;
        font_rows[998] = 32'h0700c000;
        font_rows[999] = 32'h0700c000;
        font_rows[1000] = 32'h073fffe0;
        font_rows[1001] = 32'h073fffe0;
        font_rows[1002] = 32'h07380060;
        font_rows[1003] = 32'h07380060;
        font_rows[1004] = 32'h073fffe0;
        font_rows[1005] = 32'h073fffe0;
        font_rows[1006] = 32'h07380060;
        font_rows[1007] = 32'h07380060;
        font_rows[1008] = 32'h073fffe0;
        font_rows[1009] = 32'h063fffe0;
        font_rows[1010] = 32'h06003000;
        font_rows[1011] = 32'h060c3100;
        font_rows[1012] = 32'h060e3380;
        font_rows[1013] = 32'h0e1c31c0;
        font_rows[1014] = 32'h0e3830e0;
        font_rows[1015] = 32'h0c703070;
        font_rows[1016] = 32'h1cf07038;
        font_rows[1017] = 32'h19e7f038;
        font_rows[1018] = 32'h38c7e010;
        font_rows[1019] = 32'h10000000;
        font_rows[1020] = 32'h00000000;
        font_rows[1021] = 32'h00000000;
        font_rows[1022] = 32'h00000000;
        font_rows[1023] = 32'h00000000;
        font_rows[1024] = 32'h00000000;
        font_rows[1025] = 32'h00000000;
        font_rows[1026] = 32'h00000000;
        font_rows[1027] = 32'h0ffffff0;
        font_rows[1028] = 32'h0ffffff0;
        font_rows[1029] = 32'h0c040030;
        font_rows[1030] = 32'h0c0e0030;
        font_rows[1031] = 32'h0c1c0030;
        font_rows[1032] = 32'h0c3fff30;
        font_rows[1033] = 32'h0c7ffe30;
        font_rows[1034] = 32'h0cf80e30;
        font_rows[1035] = 32'h0dcc3c30;
        font_rows[1036] = 32'h0c87f030;
        font_rows[1037] = 32'h0c03e030;
        font_rows[1038] = 32'h0c3ffe30;
        font_rows[1039] = 32'h0dfc1fb0;
        font_rows[1040] = 32'h0dcf83b0;
        font_rows[1041] = 32'h0c0ff830;
        font_rows[1042] = 32'h0c00f030;
        font_rows[1043] = 32'h0c000030;
        font_rows[1044] = 32'h0c7c0030;
        font_rows[1045] = 32'h0c7ff830;
        font_rows[1046] = 32'h0c0ffc30;
        font_rows[1047] = 32'h0c003c30;
        font_rows[1048] = 32'h0c000030;
        font_rows[1049] = 32'h0ffffff0;
        font_rows[1050] = 32'h0ffffff0;
        font_rows[1051] = 32'h0c000030;
        font_rows[1052] = 32'h00000000;
        font_rows[1053] = 32'h00000000;
        font_rows[1054] = 32'h00000000;
        font_rows[1055] = 32'h00000000;
        font_rows[1056] = 32'h00000000;
        font_rows[1057] = 32'h00000000;
        font_rows[1058] = 32'h00000000;
        font_rows[1059] = 32'h000000c0;
        font_rows[1060] = 32'h000000c0;
        font_rows[1061] = 32'h000000c0;
        font_rows[1062] = 32'h1ffc00c0;
        font_rows[1063] = 32'h1ffc00c0;
        font_rows[1064] = 32'h001c00c0;
        font_rows[1065] = 32'h001dfffc;
        font_rows[1066] = 32'h0c1dfffc;
        font_rows[1067] = 32'h0c1800e0;
        font_rows[1068] = 32'h0e1800c0;
        font_rows[1069] = 32'h0738c0c0;
        font_rows[1070] = 32'h03b8c0c0;
        font_rows[1071] = 32'h03f0e0c0;
        font_rows[1072] = 32'h01f070c0;
        font_rows[1073] = 32'h00e030c0;
        font_rows[1074] = 32'h00f038c0;
        font_rows[1075] = 32'h00f010c0;
        font_rows[1076] = 32'h01f800c0;
        font_rows[1077] = 32'h039c00c0;
        font_rows[1078] = 32'h071e00c0;
        font_rows[1079] = 32'h0f0e00c0;
        font_rows[1080] = 32'h1e0401c0;
        font_rows[1081] = 32'h3c003fc0;
        font_rows[1082] = 32'h10003f80;
        font_rows[1083] = 32'h00001000;
        font_rows[1084] = 32'h00000000;
        font_rows[1085] = 32'h00000000;
        font_rows[1086] = 32'h00000000;
        font_rows[1087] = 32'h00000000;
        font_rows[1088] = 32'h00000000;
        font_rows[1089] = 32'h00000000;
        font_rows[1090] = 32'h00000000;
        font_rows[1091] = 32'h0000e000;
        font_rows[1092] = 32'h0e00e000;
        font_rows[1093] = 32'h0e00e000;
        font_rows[1094] = 32'h0e00e000;
        font_rows[1095] = 32'h0e00e010;
        font_rows[1096] = 32'h0e00e038;
        font_rows[1097] = 32'h0e00e070;
        font_rows[1098] = 32'h0e00e1e0;
        font_rows[1099] = 32'h0e00e3c0;
        font_rows[1100] = 32'h0ffce780;
        font_rows[1101] = 32'h0ffcef00;
        font_rows[1102] = 32'h0e00fc00;
        font_rows[1103] = 32'h0e00f800;
        font_rows[1104] = 32'h0e00f000;
        font_rows[1105] = 32'h0e00e000;
        font_rows[1106] = 32'h0e00e000;
        font_rows[1107] = 32'h0e00e000;
        font_rows[1108] = 32'h0e00e018;
        font_rows[1109] = 32'h0e00e01c;
        font_rows[1110] = 32'h0e0ce018;
        font_rows[1111] = 32'h0e3ce018;
        font_rows[1112] = 32'h0ffce018;
        font_rows[1113] = 32'h0fe0e038;
        font_rows[1114] = 32'h0f80fff0;
        font_rows[1115] = 32'h04007ff0;
        font_rows[1116] = 32'h00000000;
        font_rows[1117] = 32'h00000000;
        font_rows[1118] = 32'h00000000;
        font_rows[1119] = 32'h00000000;
        font_rows[1120] = 32'h00000000;
        font_rows[1121] = 32'h00000000;
        font_rows[1122] = 32'h00000000;
        font_rows[1123] = 32'h0ffffff0;
        font_rows[1124] = 32'h0ffffff0;
        font_rows[1125] = 32'h00700e00;
        font_rows[1126] = 32'h00300e00;
        font_rows[1127] = 32'h00300e00;
        font_rows[1128] = 32'h00300e00;
        font_rows[1129] = 32'h00300e00;
        font_rows[1130] = 32'h00300e00;
        font_rows[1131] = 32'h00300e00;
        font_rows[1132] = 32'h00300e00;
        font_rows[1133] = 32'h3ffffffc;
        font_rows[1134] = 32'h3ffffffc;
        font_rows[1135] = 32'h00700e00;
        font_rows[1136] = 32'h00700e00;
        font_rows[1137] = 32'h00700e00;
        font_rows[1138] = 32'h00700e00;
        font_rows[1139] = 32'h00700e00;
        font_rows[1140] = 32'h00600e00;
        font_rows[1141] = 32'h00e00e00;
        font_rows[1142] = 32'h00e00e00;
        font_rows[1143] = 32'h01c00e00;
        font_rows[1144] = 32'h03800e00;
        font_rows[1145] = 32'h0f000e00;
        font_rows[1146] = 32'h1e000e00;
        font_rows[1147] = 32'h08000e00;
        font_rows[1148] = 32'h00000000;
        font_rows[1149] = 32'h00000000;
        font_rows[1150] = 32'h00000000;
        font_rows[1151] = 32'h00000000;
        font_rows[1152] = 32'h00000000;
        font_rows[1153] = 32'h00000000;
        font_rows[1154] = 32'h00000000;
        font_rows[1155] = 32'h00c00600;
        font_rows[1156] = 32'h01e00700;
        font_rows[1157] = 32'h00f00e00;
        font_rows[1158] = 32'h00701c00;
        font_rows[1159] = 32'h00381c00;
        font_rows[1160] = 32'h00103800;
        font_rows[1161] = 32'h07fffff0;
        font_rows[1162] = 32'h07fffff0;
        font_rows[1163] = 32'h00018000;
        font_rows[1164] = 32'h00018000;
        font_rows[1165] = 32'h00018000;
        font_rows[1166] = 32'h00018000;
        font_rows[1167] = 32'h00018000;
        font_rows[1168] = 32'h1ffffff8;
        font_rows[1169] = 32'h1ffffff8;
        font_rows[1170] = 32'h0003c000;
        font_rows[1171] = 32'h0003c000;
        font_rows[1172] = 32'h0007e000;
        font_rows[1173] = 32'h000e7000;
        font_rows[1174] = 32'h001c3800;
        font_rows[1175] = 32'h003c1c00;
        font_rows[1176] = 32'h00780f00;
        font_rows[1177] = 32'h01e007c0;
        font_rows[1178] = 32'h0fc003f0;
        font_rows[1179] = 32'h1f0000f8;
        font_rows[1180] = 32'h0c000030;
        font_rows[1181] = 32'h00000000;
        font_rows[1182] = 32'h00000000;
        font_rows[1183] = 32'h00000000;
        font_rows[1184] = 32'h00000000;
        font_rows[1185] = 32'h00000000;
        font_rows[1186] = 32'h00000000;
        font_rows[1187] = 32'h00183000;
        font_rows[1188] = 32'h001c7000;
        font_rows[1189] = 32'h00383800;
        font_rows[1190] = 32'h00303800;
        font_rows[1191] = 32'h00701c00;
        font_rows[1192] = 32'h00e00e00;
        font_rows[1193] = 32'h01c00e00;
        font_rows[1194] = 32'h03c00780;
        font_rows[1195] = 32'h078003c0;
        font_rows[1196] = 32'h0f0001e0;
        font_rows[1197] = 32'h1e0000f8;
        font_rows[1198] = 32'h3c00003c;
        font_rows[1199] = 32'h1bffff18;
        font_rows[1200] = 32'h03ffff00;
        font_rows[1201] = 32'h001c0300;
        font_rows[1202] = 32'h00180300;
        font_rows[1203] = 32'h00180300;
        font_rows[1204] = 32'h00380300;
        font_rows[1205] = 32'h00380700;
        font_rows[1206] = 32'h00700700;
        font_rows[1207] = 32'h00e00700;
        font_rows[1208] = 32'h01e00e00;
        font_rows[1209] = 32'h0783fe00;
        font_rows[1210] = 32'h1f03fc00;
        font_rows[1211] = 32'h0c000000;
        font_rows[1212] = 32'h00000000;
        font_rows[1213] = 32'h00000000;
        font_rows[1214] = 32'h00000000;
        font_rows[1215] = 32'h00000000;
        font_rows[1216] = 32'h00000000;
        font_rows[1217] = 32'h00000000;
        font_rows[1218] = 32'h00600038;
        font_rows[1219] = 32'h00700038;
        font_rows[1220] = 32'h00300038;
        font_rows[1221] = 32'h1fffe638;
        font_rows[1222] = 32'h1fffe638;
        font_rows[1223] = 32'h18306638;
        font_rows[1224] = 32'h00300638;
        font_rows[1225] = 32'h0fffe638;
        font_rows[1226] = 32'h0fffc638;
        font_rows[1227] = 32'h00300638;
        font_rows[1228] = 32'h07ffc638;
        font_rows[1229] = 32'h07ffc638;
        font_rows[1230] = 32'h00300638;
        font_rows[1231] = 32'h1fffe638;
        font_rows[1232] = 32'h1fffe638;
        font_rows[1233] = 32'h00300638;
        font_rows[1234] = 32'h00300638;
        font_rows[1235] = 32'h00000638;
        font_rows[1236] = 32'h0fffc638;
        font_rows[1237] = 32'h0fffc638;
        font_rows[1238] = 32'h0e00c038;
        font_rows[1239] = 32'h0e00c038;
        font_rows[1240] = 32'h0e00c038;
        font_rows[1241] = 32'h0fffc030;
        font_rows[1242] = 32'h0fffc7f0;
        font_rows[1243] = 32'h0e00c7f0;
        font_rows[1244] = 32'h00000000;
        font_rows[1245] = 32'h00000000;
        font_rows[1246] = 32'h00000000;
        font_rows[1247] = 32'h00000000;
        font_rows[1248] = 32'h00000000;
        font_rows[1249] = 32'h00000000;
        font_rows[1250] = 32'h00000000;
        font_rows[1251] = 32'h00c03000;
        font_rows[1252] = 32'h00c03800;
        font_rows[1253] = 32'h01c03c00;
        font_rows[1254] = 32'h01c01800;
        font_rows[1255] = 32'h01bffff8;
        font_rows[1256] = 32'h03bffff8;
        font_rows[1257] = 32'h03000000;
        font_rows[1258] = 32'h07000000;
        font_rows[1259] = 32'h070c0060;
        font_rows[1260] = 32'h0f0e00e0;
        font_rows[1261] = 32'h0f0600e0;
        font_rows[1262] = 32'h1f0700e0;
        font_rows[1263] = 32'h3b0700c0;
        font_rows[1264] = 32'h130301c0;
        font_rows[1265] = 32'h130381c0;
        font_rows[1266] = 32'h03038180;
        font_rows[1267] = 32'h03018380;
        font_rows[1268] = 32'h0301c380;
        font_rows[1269] = 32'h0301c300;
        font_rows[1270] = 32'h0300c700;
        font_rows[1271] = 32'h03000600;
        font_rows[1272] = 32'h03000600;
        font_rows[1273] = 32'h033ffffc;
        font_rows[1274] = 32'h033ffffc;
        font_rows[1275] = 32'h03000000;
        font_rows[1276] = 32'h00000000;
        font_rows[1277] = 32'h00000000;
        font_rows[1278] = 32'h00000000;
        font_rows[1279] = 32'h00000000;
        font_rows[1280] = 32'h00000000;
        font_rows[1281] = 32'h00000000;
        font_rows[1282] = 32'h00000000;
        font_rows[1283] = 32'h07ffffe0;
        font_rows[1284] = 32'h07ffffe0;
        font_rows[1285] = 32'h060c3060;
        font_rows[1286] = 32'h060c3060;
        font_rows[1287] = 32'h07ffffe0;
        font_rows[1288] = 32'h07ffffe0;
        font_rows[1289] = 32'h00018000;
        font_rows[1290] = 32'h00018000;
        font_rows[1291] = 32'h1ffffff8;
        font_rows[1292] = 32'h1ffffff8;
        font_rows[1293] = 32'h00018000;
        font_rows[1294] = 32'h03ffffc0;
        font_rows[1295] = 32'h03ffffc0;
        font_rows[1296] = 32'h038000c0;
        font_rows[1297] = 32'h03ffffc0;
        font_rows[1298] = 32'h03ffffc0;
        font_rows[1299] = 32'h038000c0;
        font_rows[1300] = 32'h03ffffc0;
        font_rows[1301] = 32'h03ffffc0;
        font_rows[1302] = 32'h038000c0;
        font_rows[1303] = 32'h03ffffc0;
        font_rows[1304] = 32'h03ffffc0;
        font_rows[1305] = 32'h038000c0;
        font_rows[1306] = 32'h1ffffff8;
        font_rows[1307] = 32'h3ffffffc;
        font_rows[1308] = 32'h00000000;
        font_rows[1309] = 32'h00000000;
        font_rows[1310] = 32'h00000000;
        font_rows[1311] = 32'h00000000;
        font_rows[1312] = 32'h00000000;
        font_rows[1313] = 32'h00000000;
        font_rows[1314] = 32'h03000000;
        font_rows[1315] = 32'h03000000;
        font_rows[1316] = 32'h0700ffc0;
        font_rows[1317] = 32'h0600ffc0;
        font_rows[1318] = 32'h07f8c0c0;
        font_rows[1319] = 32'h0ff8c0c0;
        font_rows[1320] = 32'h0c00c0c0;
        font_rows[1321] = 32'h0c01c0c0;
        font_rows[1322] = 32'h1801c0c0;
        font_rows[1323] = 32'h180180e0;
        font_rows[1324] = 32'h3ff380fc;
        font_rows[1325] = 32'h17f7007c;
        font_rows[1326] = 32'h03870000;
        font_rows[1327] = 32'h03820000;
        font_rows[1328] = 32'h03800000;
        font_rows[1329] = 32'h3ffbfff0;
        font_rows[1330] = 32'h1ffbfff0;
        font_rows[1331] = 32'h03830030;
        font_rows[1332] = 32'h03830030;
        font_rows[1333] = 32'h03830030;
        font_rows[1334] = 32'h03830030;
        font_rows[1335] = 32'h038b0030;
        font_rows[1336] = 32'h039b0030;
        font_rows[1337] = 32'h03fb0030;
        font_rows[1338] = 32'h03e3fff0;
        font_rows[1339] = 32'h03c3fff0;
        font_rows[1340] = 32'h03830030;
        font_rows[1341] = 32'h01030030;
        font_rows[1342] = 32'h00000000;
        font_rows[1343] = 32'h00000000;
        font_rows[1344] = 32'h00000000;
        font_rows[1345] = 32'h00000000;
        font_rows[1346] = 32'h00000000;
        font_rows[1347] = 32'h01c01800;
        font_rows[1348] = 32'h03803800;
        font_rows[1349] = 32'h03ffbff8;
        font_rows[1350] = 32'h07fffff8;
        font_rows[1351] = 32'h0e20e200;
        font_rows[1352] = 32'h1c70c700;
        font_rows[1353] = 32'h3839c380;
        font_rows[1354] = 32'h10108100;
        font_rows[1355] = 32'h0001ffc0;
        font_rows[1356] = 32'h0fffffe0;
        font_rows[1357] = 32'h07ffc000;
        font_rows[1358] = 32'h00070000;
        font_rows[1359] = 32'h00070000;
        font_rows[1360] = 32'h0007ffc0;
        font_rows[1361] = 32'h0fffffc0;
        font_rows[1362] = 32'h0fff0000;
        font_rows[1363] = 32'h00070000;
        font_rows[1364] = 32'h000707f8;
        font_rows[1365] = 32'h00fffff8;
        font_rows[1366] = 32'h1ffffe00;
        font_rows[1367] = 32'h1fff0030;
        font_rows[1368] = 32'h00070030;
        font_rows[1369] = 32'h00030030;
        font_rows[1370] = 32'h00030070;
        font_rows[1371] = 32'h0003ffe0;
        font_rows[1372] = 32'h0001ffc0;
        font_rows[1373] = 32'h00000000;
        font_rows[1374] = 32'h00000000;
        font_rows[1375] = 32'h00000000;
        font_rows[1376] = 32'h00000000;
        font_rows[1377] = 32'h00000000;
        font_rows[1378] = 32'h00000000;
        font_rows[1379] = 32'h0c000ff0;
        font_rows[1380] = 32'h0e3ffff0;
        font_rows[1381] = 32'h073ffe00;
        font_rows[1382] = 32'h03804060;
        font_rows[1383] = 32'h0110e0e0;
        font_rows[1384] = 32'h001860e0;
        font_rows[1385] = 32'h001c71c0;
        font_rows[1386] = 32'h180e3380;
        font_rows[1387] = 32'h3c043100;
        font_rows[1388] = 32'h0e000000;
        font_rows[1389] = 32'h071fffe0;
        font_rows[1390] = 32'h021fffe0;
        font_rows[1391] = 32'h000001e0;
        font_rows[1392] = 32'h00000780;
        font_rows[1393] = 32'h06001e00;
        font_rows[1394] = 32'h07001c00;
        font_rows[1395] = 32'h067ffffc;
        font_rows[1396] = 32'h067ffffc;
        font_rows[1397] = 32'h0e001800;
        font_rows[1398] = 32'h0e001800;
        font_rows[1399] = 32'h0e001800;
        font_rows[1400] = 32'h0c001800;
        font_rows[1401] = 32'h1c003800;
        font_rows[1402] = 32'h1c07f800;
        font_rows[1403] = 32'h1c07f000;
        font_rows[1404] = 32'h00000000;
        font_rows[1405] = 32'h00000000;
        font_rows[1406] = 32'h00000000;
        font_rows[1407] = 32'h00000000;
        font_rows[1408] = 32'h00000000;
        font_rows[1409] = 32'h00000000;
        font_rows[1410] = 32'h00001980;
        font_rows[1411] = 32'h00001980;
        font_rows[1412] = 32'h0fff19c0;
        font_rows[1413] = 32'h0fff18c0;
        font_rows[1414] = 32'h080330c0;
        font_rows[1415] = 32'h08633ff8;
        font_rows[1416] = 32'h08633ff8;
        font_rows[1417] = 32'h0bfb7180;
        font_rows[1418] = 32'h0bfb7180;
        font_rows[1419] = 32'h0863f180;
        font_rows[1420] = 32'h0863f180;
        font_rows[1421] = 32'h0ffffff8;
        font_rows[1422] = 32'h0fff3ff8;
        font_rows[1423] = 32'h08033180;
        font_rows[1424] = 32'h08033180;
        font_rows[1425] = 32'h09fb3180;
        font_rows[1426] = 32'h09fb3180;
        font_rows[1427] = 32'h091b3ff8;
        font_rows[1428] = 32'h191b3ff8;
        font_rows[1429] = 32'h191b3180;
        font_rows[1430] = 32'h19fb3180;
        font_rows[1431] = 32'h19fb3180;
        font_rows[1432] = 32'h19033180;
        font_rows[1433] = 32'h18033ffc;
        font_rows[1434] = 32'h38033ffc;
        font_rows[1435] = 32'h303e3000;
        font_rows[1436] = 32'h103c3000;
        font_rows[1437] = 32'h00000000;
        font_rows[1438] = 32'h00000000;
        font_rows[1439] = 32'h00000000;
        font_rows[1440] = 32'h00000000;
        font_rows[1441] = 32'h00000000;
        font_rows[1442] = 32'h00000000;
        font_rows[1443] = 32'h0c000000;
        font_rows[1444] = 32'h0e1fffe0;
        font_rows[1445] = 32'h079fffe0;
        font_rows[1446] = 32'h03980060;
        font_rows[1447] = 32'h019fffe0;
        font_rows[1448] = 32'h001fffe0;
        font_rows[1449] = 32'h00180060;
        font_rows[1450] = 32'h181fffe0;
        font_rows[1451] = 32'h1c1fffe0;
        font_rows[1452] = 32'h0e000000;
        font_rows[1453] = 32'h073ffff0;
        font_rows[1454] = 32'h023ffff0;
        font_rows[1455] = 32'h00318630;
        font_rows[1456] = 32'h00318630;
        font_rows[1457] = 32'h003ffff0;
        font_rows[1458] = 32'h033ffff0;
        font_rows[1459] = 32'h03000000;
        font_rows[1460] = 32'h071fffe0;
        font_rows[1461] = 32'h071fffc0;
        font_rows[1462] = 32'h060701c0;
        font_rows[1463] = 32'h06038780;
        font_rows[1464] = 32'h0e01fe00;
        font_rows[1465] = 32'h0e00fc00;
        font_rows[1466] = 32'h0c07ffc0;
        font_rows[1467] = 32'h1cff07fc;
        font_rows[1468] = 32'h0cf00078;
        font_rows[1469] = 32'h00000000;
        font_rows[1470] = 32'h00000000;
        font_rows[1471] = 32'h00000000;
        font_rows[1472] = 32'h00000000;
        font_rows[1473] = 32'h00000000;
        font_rows[1474] = 32'h00000000;
        font_rows[1475] = 32'h00000000;
        font_rows[1476] = 32'h1ffffff8;
        font_rows[1477] = 32'h1ffffff8;
        font_rows[1478] = 32'h00000000;
        font_rows[1479] = 32'h00000000;
        font_rows[1480] = 32'h0e7ffe70;
        font_rows[1481] = 32'h0e7ffe70;
        font_rows[1482] = 32'h0e618e70;
        font_rows[1483] = 32'h0e618e70;
        font_rows[1484] = 32'h0e618e70;
        font_rows[1485] = 32'h0e618e70;
        font_rows[1486] = 32'h0e7ffe70;
        font_rows[1487] = 32'h0e7ffe70;
        font_rows[1488] = 32'h0e618e70;
        font_rows[1489] = 32'h0e618e70;
        font_rows[1490] = 32'h0e618e70;
        font_rows[1491] = 32'h0e7ffe70;
        font_rows[1492] = 32'h0e7ffe70;
        font_rows[1493] = 32'h0e600e70;
        font_rows[1494] = 32'h0e600e70;
        font_rows[1495] = 32'h0e000070;
        font_rows[1496] = 32'h0e000070;
        font_rows[1497] = 32'h0ffffff0;
        font_rows[1498] = 32'h0ffffff0;
        font_rows[1499] = 32'h00000070;
        font_rows[1500] = 32'h00000000;
        font_rows[1501] = 32'h00000000;
        font_rows[1502] = 32'h00000000;
        font_rows[1503] = 32'h00000000;
        font_rows[1504] = 32'h00000000;
        font_rows[1505] = 32'h00000000;
        font_rows[1506] = 32'h00000000;
        font_rows[1507] = 32'h00c1c000;
        font_rows[1508] = 32'h00c18000;
        font_rows[1509] = 32'h01c3ff00;
        font_rows[1510] = 32'h0187ff00;
        font_rows[1511] = 32'h038e0700;
        font_rows[1512] = 32'h031c0e00;
        font_rows[1513] = 32'h073ffff0;
        font_rows[1514] = 32'h077ffff0;
        font_rows[1515] = 32'h0f3c1830;
        font_rows[1516] = 32'h0f1c3030;
        font_rows[1517] = 32'h1f1ffff0;
        font_rows[1518] = 32'h3f1ffff0;
        font_rows[1519] = 32'h1301e000;
        font_rows[1520] = 32'h1303f020;
        font_rows[1521] = 32'h031f30f0;
        font_rows[1522] = 32'h037cfbc0;
        font_rows[1523] = 32'h0331df80;
        font_rows[1524] = 32'h03079d80;
        font_rows[1525] = 32'h033e3cc0;
        font_rows[1526] = 32'h0378fce0;
        font_rows[1527] = 32'h0303cc70;
        font_rows[1528] = 32'h030f0c38;
        font_rows[1529] = 32'h037c1c10;
        font_rows[1530] = 32'h0331f800;
        font_rows[1531] = 32'h0301f000;
        font_rows[1532] = 32'h00000000;
        font_rows[1533] = 32'h00000000;
        font_rows[1534] = 32'h00000000;
        font_rows[1535] = 32'h00000000;
        font_rows[1536] = 32'h00000000;
        font_rows[1537] = 32'h00000000;
        font_rows[1538] = 32'h00018000;
        font_rows[1539] = 32'h00018000;
        font_rows[1540] = 32'h0ffffff0;
        font_rows[1541] = 32'h0ffffff0;
        font_rows[1542] = 32'h00018000;
        font_rows[1543] = 32'h07ffffc0;
        font_rows[1544] = 32'h07ffffc0;
        font_rows[1545] = 32'h00018000;
        font_rows[1546] = 32'h1ffffff8;
        font_rows[1547] = 32'h1ffffff8;
        font_rows[1548] = 32'h00380e00;
        font_rows[1549] = 32'h00f0fe00;
        font_rows[1550] = 32'h03fff800;
        font_rows[1551] = 32'h03ffe200;
        font_rows[1552] = 32'h039f8700;
        font_rows[1553] = 32'h007c0180;
        font_rows[1554] = 32'h03f1ffc0;
        font_rows[1555] = 32'h07ffffe0;
        font_rows[1556] = 32'h03f1c070;
        font_rows[1557] = 32'h0001c020;
        font_rows[1558] = 32'h0041c400;
        font_rows[1559] = 32'h01e1cf00;
        font_rows[1560] = 32'h03c1c7c0;
        font_rows[1561] = 32'h0f8181e0;
        font_rows[1562] = 32'h1e3f8070;
        font_rows[1563] = 32'h081f0020;
        font_rows[1564] = 32'h00000000;
        font_rows[1565] = 32'h00000000;
        font_rows[1566] = 32'h00000000;
        font_rows[1567] = 32'h00000000;
        font_rows[1568] = 32'h00000000;
        font_rows[1569] = 32'h00000000;
        font_rows[1570] = 32'h00000000;
        font_rows[1571] = 32'h00000000;
        font_rows[1572] = 32'h0ffffff8;
        font_rows[1573] = 32'h0ffffff8;
        font_rows[1574] = 32'h0000e000;
        font_rows[1575] = 32'h0000e000;
        font_rows[1576] = 32'h0000e000;
        font_rows[1577] = 32'h0000e000;
        font_rows[1578] = 32'h0180e000;
        font_rows[1579] = 32'h0180e000;
        font_rows[1580] = 32'h0180e000;
        font_rows[1581] = 32'h0180fff0;
        font_rows[1582] = 32'h0180fff0;
        font_rows[1583] = 32'h0180e000;
        font_rows[1584] = 32'h0180e000;
        font_rows[1585] = 32'h0180e000;
        font_rows[1586] = 32'h0180e000;
        font_rows[1587] = 32'h0180e000;
        font_rows[1588] = 32'h0180e000;
        font_rows[1589] = 32'h0180e000;
        font_rows[1590] = 32'h0180e000;
        font_rows[1591] = 32'h0180e000;
        font_rows[1592] = 32'h0180e000;
        font_rows[1593] = 32'h3ffffffc;
        font_rows[1594] = 32'h3ffffffc;
        font_rows[1595] = 32'h00000000;
        font_rows[1596] = 32'h00000000;
        font_rows[1597] = 32'h00000000;
        font_rows[1598] = 32'h00000000;
        font_rows[1599] = 32'h00000000;
        font_rows[1600] = 32'h00000000;
        font_rows[1601] = 32'h00000000;
        font_rows[1602] = 32'h010180c0;
        font_rows[1603] = 32'h03c181c0;
        font_rows[1604] = 32'h01e18380;
        font_rows[1605] = 32'h00e18700;
        font_rows[1606] = 32'h00618600;
        font_rows[1607] = 32'h1ffffff8;
        font_rows[1608] = 32'h1ffffff8;
        font_rows[1609] = 32'h1c000038;
        font_rows[1610] = 32'h1dffffb8;
        font_rows[1611] = 32'h01ffff80;
        font_rows[1612] = 32'h01c00380;
        font_rows[1613] = 32'h01c00380;
        font_rows[1614] = 32'h01c00380;
        font_rows[1615] = 32'h01ffff80;
        font_rows[1616] = 32'h01ffff80;
        font_rows[1617] = 32'h01c18380;
        font_rows[1618] = 32'h00018000;
        font_rows[1619] = 32'h07ffffe0;
        font_rows[1620] = 32'h07ffffe0;
        font_rows[1621] = 32'h06018060;
        font_rows[1622] = 32'h06018060;
        font_rows[1623] = 32'h06018060;
        font_rows[1624] = 32'h060180e0;
        font_rows[1625] = 32'h06018fe0;
        font_rows[1626] = 32'h06018fc0;
        font_rows[1627] = 32'h00018000;
        font_rows[1628] = 32'h00018000;
        font_rows[1629] = 32'h00000000;
        font_rows[1630] = 32'h00000000;
        font_rows[1631] = 32'h00000000;
        font_rows[1632] = 32'h00000000;
        font_rows[1633] = 32'h00000000;
        font_rows[1634] = 32'h00000000;
        font_rows[1635] = 32'h00000000;
        font_rows[1636] = 32'h1ffffff8;
        font_rows[1637] = 32'h1ffffff8;
        font_rows[1638] = 32'h00038000;
        font_rows[1639] = 32'h00070000;
        font_rows[1640] = 32'h00070000;
        font_rows[1641] = 32'h0ffffff0;
        font_rows[1642] = 32'h0ffffff0;
        font_rows[1643] = 32'h0e181870;
        font_rows[1644] = 32'h0e181870;
        font_rows[1645] = 32'h0e181870;
        font_rows[1646] = 32'h0e181870;
        font_rows[1647] = 32'h0e1ff870;
        font_rows[1648] = 32'h0e1ff870;
        font_rows[1649] = 32'h0e181870;
        font_rows[1650] = 32'h0e181870;
        font_rows[1651] = 32'h0e181870;
        font_rows[1652] = 32'h0e1ff870;
        font_rows[1653] = 32'h0e1ff870;
        font_rows[1654] = 32'h0e181870;
        font_rows[1655] = 32'h0e181870;
        font_rows[1656] = 32'h0e181870;
        font_rows[1657] = 32'h0ffffff0;
        font_rows[1658] = 32'h0ffffff0;
        font_rows[1659] = 32'h0e000070;
        font_rows[1660] = 32'h00000000;
        font_rows[1661] = 32'h00000000;
        font_rows[1662] = 32'h00000000;
        font_rows[1663] = 32'h00000000;
        font_rows[1664] = 32'h00000000;
        font_rows[1665] = 32'h00000000;
        font_rows[1666] = 32'h06000600;
        font_rows[1667] = 32'h06000600;
        font_rows[1668] = 32'h06000600;
        font_rows[1669] = 32'h0e1f7ff0;
        font_rows[1670] = 32'h0fdf7ff0;
        font_rows[1671] = 32'h0fc30630;
        font_rows[1672] = 32'h18060630;
        font_rows[1673] = 32'h1806fff8;
        font_rows[1674] = 32'h380cfff8;
        font_rows[1675] = 32'h1fcc0630;
        font_rows[1676] = 32'h0fd80630;
        font_rows[1677] = 32'h06187ff0;
        font_rows[1678] = 32'h061f7ff0;
        font_rows[1679] = 32'h061f0620;
        font_rows[1680] = 32'h3fe30600;
        font_rows[1681] = 32'h1fe37ff0;
        font_rows[1682] = 32'h06337fe0;
        font_rows[1683] = 32'h06330600;
        font_rows[1684] = 32'h061e0600;
        font_rows[1685] = 32'h061efff0;
        font_rows[1686] = 32'h064efff0;
        font_rows[1687] = 32'h06ce0600;
        font_rows[1688] = 32'h07ce0600;
        font_rows[1689] = 32'h079f8600;
        font_rows[1690] = 32'h0719e000;
        font_rows[1691] = 32'h0630fffc;
        font_rows[1692] = 32'h00703ff8;
        font_rows[1693] = 32'h00200000;
        font_rows[1694] = 32'h00000000;
        font_rows[1695] = 32'h00000000;
        font_rows[1696] = 32'h00000000;
        font_rows[1697] = 32'h00000000;
        font_rows[1698] = 32'h00000000;
        font_rows[1699] = 32'h03000000;
        font_rows[1700] = 32'h03000000;
        font_rows[1701] = 32'h030ffff0;
        font_rows[1702] = 32'h030ffff0;
        font_rows[1703] = 32'h03007030;
        font_rows[1704] = 32'h03006030;
        font_rows[1705] = 32'h03006030;
        font_rows[1706] = 32'h03fc6030;
        font_rows[1707] = 32'h3ffc6030;
        font_rows[1708] = 32'h3fc06030;
        font_rows[1709] = 32'h03006030;
        font_rows[1710] = 32'h03006030;
        font_rows[1711] = 32'h03006070;
        font_rows[1712] = 32'h03006070;
        font_rows[1713] = 32'h0300e070;
        font_rows[1714] = 32'h0304e070;
        font_rows[1715] = 32'h030ce070;
        font_rows[1716] = 32'h033cc070;
        font_rows[1717] = 32'h0379c070;
        font_rows[1718] = 32'h03e1c070;
        font_rows[1719] = 32'h07c38070;
        font_rows[1720] = 32'h07870060;
        font_rows[1721] = 32'h020f20e0;
        font_rows[1722] = 32'h001e3fe0;
        font_rows[1723] = 32'h003c3fc0;
        font_rows[1724] = 32'h00180000;
        font_rows[1725] = 32'h00000000;
        font_rows[1726] = 32'h00000000;
        font_rows[1727] = 32'h00000000;
        font_rows[1728] = 32'h00000000;
        font_rows[1729] = 32'h00000000;
        font_rows[1730] = 32'h0300e000;
        font_rows[1731] = 32'h0300c000;
        font_rows[1732] = 32'h0301c000;
        font_rows[1733] = 32'h0301ffc0;
        font_rows[1734] = 32'h0303ffc0;
        font_rows[1735] = 32'h3fe70180;
        font_rows[1736] = 32'h1fe60380;
        font_rows[1737] = 32'h030e0700;
        font_rows[1738] = 32'h031c0e00;
        font_rows[1739] = 32'h030ffff0;
        font_rows[1740] = 32'h030ffff0;
        font_rows[1741] = 32'h030e1870;
        font_rows[1742] = 32'h036e1870;
        font_rows[1743] = 32'h07ee1870;
        font_rows[1744] = 32'h1fce1870;
        font_rows[1745] = 32'h3f0e1870;
        font_rows[1746] = 32'h130e1870;
        font_rows[1747] = 32'h033ffffc;
        font_rows[1748] = 32'h033ffffc;
        font_rows[1749] = 32'h03003c00;
        font_rows[1750] = 32'h03007c00;
        font_rows[1751] = 32'h0300e600;
        font_rows[1752] = 32'h0301c700;
        font_rows[1753] = 32'h030783c0;
        font_rows[1754] = 32'h071f01f0;
        font_rows[1755] = 32'h3f7c0078;
        font_rows[1756] = 32'h3e300030;
        font_rows[1757] = 32'h00000000;
        font_rows[1758] = 32'h00000000;
        font_rows[1759] = 32'h00000000;
        font_rows[1760] = 32'h00000000;
        font_rows[1761] = 32'h00000000;
        font_rows[1762] = 32'h03038000;
        font_rows[1763] = 32'h03038000;
        font_rows[1764] = 32'h03038000;
        font_rows[1765] = 32'h03038000;
        font_rows[1766] = 32'h031ffff8;
        font_rows[1767] = 32'h031ffff8;
        font_rows[1768] = 32'h03c30000;
        font_rows[1769] = 32'h1be30c00;
        font_rows[1770] = 32'h1b770c00;
        font_rows[1771] = 32'h1b270c00;
        font_rows[1772] = 32'h1b060c00;
        font_rows[1773] = 32'h1b060c00;
        font_rows[1774] = 32'h3b0f8c10;
        font_rows[1775] = 32'h130f8c38;
        font_rows[1776] = 32'h030dce70;
        font_rows[1777] = 32'h031cde60;
        font_rows[1778] = 32'h0318fee0;
        font_rows[1779] = 32'h03385f40;
        font_rows[1780] = 32'h03703f00;
        font_rows[1781] = 32'h03703b00;
        font_rows[1782] = 32'h03e07380;
        font_rows[1783] = 32'h034071c0;
        font_rows[1784] = 32'h0300e1c0;
        font_rows[1785] = 32'h0301c0e0;
        font_rows[1786] = 32'h03078078;
        font_rows[1787] = 32'h030f0038;
        font_rows[1788] = 32'h03060018;
        font_rows[1789] = 32'h00000000;
        font_rows[1790] = 32'h00000000;
        font_rows[1791] = 32'h00000000;
        font_rows[1792] = 32'h00000000;
        font_rows[1793] = 32'h00000000;
        font_rows[1794] = 32'h00000000;
        font_rows[1795] = 32'h00e00000;
        font_rows[1796] = 32'h00c00000;
        font_rows[1797] = 32'h01fffff8;
        font_rows[1798] = 32'h03fffff8;
        font_rows[1799] = 32'h07000000;
        font_rows[1800] = 32'h07ffffe0;
        font_rows[1801] = 32'h1fffffe0;
        font_rows[1802] = 32'h1b8000e0;
        font_rows[1803] = 32'h13ffffe0;
        font_rows[1804] = 32'h03ffffe0;
        font_rows[1805] = 32'h038000e0;
        font_rows[1806] = 32'h038000e0;
        font_rows[1807] = 32'h03ffffe0;
        font_rows[1808] = 32'h03ffffe0;
        font_rows[1809] = 32'h03b800e0;
        font_rows[1810] = 32'h00700000;
        font_rows[1811] = 32'h00ffffe0;
        font_rows[1812] = 32'h01ffffc0;
        font_rows[1813] = 32'h03f00380;
        font_rows[1814] = 32'h0f1c0f00;
        font_rows[1815] = 32'h1e0f3c00;
        font_rows[1816] = 32'h1803f800;
        font_rows[1817] = 32'h000fff80;
        font_rows[1818] = 32'h00fe1ff8;
        font_rows[1819] = 32'h0ff001f8;
        font_rows[1820] = 32'h07000000;
        font_rows[1821] = 32'h00000000;
        font_rows[1822] = 32'h00000000;
        font_rows[1823] = 32'h00000000;
        font_rows[1824] = 32'h00000000;
        font_rows[1825] = 32'h00000000;
        font_rows[1826] = 32'h00000000;
        font_rows[1827] = 32'h00000600;
        font_rows[1828] = 32'h0fff0680;
        font_rows[1829] = 32'h0fff06e0;
        font_rows[1830] = 32'h0c630660;
        font_rows[1831] = 32'h0e6f0630;
        font_rows[1832] = 32'h0f6f0630;
        font_rows[1833] = 32'h0f7b0600;
        font_rows[1834] = 32'h0d7b7ff8;
        font_rows[1835] = 32'h0c637ff8;
        font_rows[1836] = 32'h0fff0600;
        font_rows[1837] = 32'h0fff0600;
        font_rows[1838] = 32'h00600e00;
        font_rows[1839] = 32'h00600f00;
        font_rows[1840] = 32'h1fff0f00;
        font_rows[1841] = 32'h0fff0f00;
        font_rows[1842] = 32'h00600d80;
        font_rows[1843] = 32'h007f9d80;
        font_rows[1844] = 32'h1fff99c0;
        font_rows[1845] = 32'h1fc638c0;
        font_rows[1846] = 32'h003730e0;
        font_rows[1847] = 32'h0db37060;
        font_rows[1848] = 32'h0d9be070;
        font_rows[1849] = 32'h1999c038;
        font_rows[1850] = 32'h1881c018;
        font_rows[1851] = 32'h10018010;
        font_rows[1852] = 32'h00000000;
        font_rows[1853] = 32'h00000000;
        font_rows[1854] = 32'h00000000;
        font_rows[1855] = 32'h00000000;
        font_rows[1856] = 32'h00000000;
        font_rows[1857] = 32'h00000000;
        font_rows[1858] = 32'h00000000;
        font_rows[1859] = 32'h02003800;
        font_rows[1860] = 32'h07003800;
        font_rows[1861] = 32'h03803800;
        font_rows[1862] = 32'h03c03800;
        font_rows[1863] = 32'h01c03800;
        font_rows[1864] = 32'h00e03800;
        font_rows[1865] = 32'h00403800;
        font_rows[1866] = 32'h00003800;
        font_rows[1867] = 32'h00003800;
        font_rows[1868] = 32'h3f003800;
        font_rows[1869] = 32'h3f003800;
        font_rows[1870] = 32'h03003c00;
        font_rows[1871] = 32'h03003c00;
        font_rows[1872] = 32'h03007c00;
        font_rows[1873] = 32'h03007c00;
        font_rows[1874] = 32'h03006600;
        font_rows[1875] = 32'h0308e600;
        font_rows[1876] = 32'h0318e700;
        font_rows[1877] = 32'h0339c300;
        font_rows[1878] = 32'h03f1c380;
        font_rows[1879] = 32'h03e381c0;
        font_rows[1880] = 32'h03c701e0;
        font_rows[1881] = 32'h070e00f0;
        font_rows[1882] = 32'h021c0078;
        font_rows[1883] = 32'h00380038;
        font_rows[1884] = 32'h00100018;
        font_rows[1885] = 32'h00000000;
        font_rows[1886] = 32'h00000000;
        font_rows[1887] = 32'h00000000;
        font_rows[1888] = 32'h00000000;
        font_rows[1889] = 32'h00000000;
        font_rows[1890] = 32'h0300c380;
        font_rows[1891] = 32'h0300c380;
        font_rows[1892] = 32'h0300c380;
        font_rows[1893] = 32'h031ffff8;
        font_rows[1894] = 32'h030ffff8;
        font_rows[1895] = 32'h0300c380;
        font_rows[1896] = 32'h0300c380;
        font_rows[1897] = 32'h3ff00000;
        font_rows[1898] = 32'h1ff7fff0;
        font_rows[1899] = 32'h0307fff0;
        font_rows[1900] = 32'h07060070;
        font_rows[1901] = 32'h07c60070;
        font_rows[1902] = 32'h07e7fff0;
        font_rows[1903] = 32'h0fe7fff0;
        font_rows[1904] = 32'h0f760070;
        font_rows[1905] = 32'h1f360070;
        font_rows[1906] = 32'h1b07fff0;
        font_rows[1907] = 32'h3b07fff0;
        font_rows[1908] = 32'h33001800;
        font_rows[1909] = 32'h13001800;
        font_rows[1910] = 32'h031ffff8;
        font_rows[1911] = 32'h031ffff8;
        font_rows[1912] = 32'h03003e00;
        font_rows[1913] = 32'h03006700;
        font_rows[1914] = 32'h0301c380;
        font_rows[1915] = 32'h030781e0;
        font_rows[1916] = 32'h033f0078;
        font_rows[1917] = 32'h03180018;
        font_rows[1918] = 32'h00000000;
        font_rows[1919] = 32'h00000000;
        font_rows[1920] = 32'h00000000;
        font_rows[1921] = 32'h00000000;
        font_rows[1922] = 32'h00000000;
        font_rows[1923] = 32'h00007300;
        font_rows[1924] = 32'h000073c0;
        font_rows[1925] = 32'h000071e0;
        font_rows[1926] = 32'h00007040;
        font_rows[1927] = 32'h00007000;
        font_rows[1928] = 32'h3ffffff8;
        font_rows[1929] = 32'h3ffffff8;
        font_rows[1930] = 32'h00003000;
        font_rows[1931] = 32'h00003000;
        font_rows[1932] = 32'h00003000;
        font_rows[1933] = 32'h00003000;
        font_rows[1934] = 32'h1fffb000;
        font_rows[1935] = 32'h1fffb800;
        font_rows[1936] = 32'h00603800;
        font_rows[1937] = 32'h00603800;
        font_rows[1938] = 32'h00601800;
        font_rows[1939] = 32'h00601800;
        font_rows[1940] = 32'h00601c00;
        font_rows[1941] = 32'h00601c18;
        font_rows[1942] = 32'h0067cc1c;
        font_rows[1943] = 32'h00ffce18;
        font_rows[1944] = 32'h1fff0718;
        font_rows[1945] = 32'h1fe007f8;
        font_rows[1946] = 32'h1c0003f0;
        font_rows[1947] = 32'h000000f0;
        font_rows[1948] = 32'h00000000;
        font_rows[1949] = 32'h00000000;
        font_rows[1950] = 32'h00000000;
        font_rows[1951] = 32'h00000000;
        font_rows[1952] = 32'h00000000;
        font_rows[1953] = 32'h00000000;
        font_rows[1954] = 32'h00000000;
        font_rows[1955] = 32'h04001800;
        font_rows[1956] = 32'h0e071800;
        font_rows[1957] = 32'h07071800;
        font_rows[1958] = 32'h07061800;
        font_rows[1959] = 32'h038ffff0;
        font_rows[1960] = 32'h018ffff0;
        font_rows[1961] = 32'h019c1800;
        font_rows[1962] = 32'h00181800;
        font_rows[1963] = 32'h00181800;
        font_rows[1964] = 32'h3f001800;
        font_rows[1965] = 32'h3f3ffff8;
        font_rows[1966] = 32'h033ffff8;
        font_rows[1967] = 32'h03018600;
        font_rows[1968] = 32'h03018600;
        font_rows[1969] = 32'h03018600;
        font_rows[1970] = 32'h03038618;
        font_rows[1971] = 32'h03038618;
        font_rows[1972] = 32'h03070618;
        font_rows[1973] = 32'h031e0638;
        font_rows[1974] = 32'h033c07f8;
        font_rows[1975] = 32'h071803f0;
        font_rows[1976] = 32'h0f800000;
        font_rows[1977] = 32'h1df00000;
        font_rows[1978] = 32'h187ffff8;
        font_rows[1979] = 32'h101ffff8;
        font_rows[1980] = 32'h00000000;
        font_rows[1981] = 32'h00000000;
        font_rows[1982] = 32'h00000000;
        font_rows[1983] = 32'h00000000;
        font_rows[1984] = 32'h00000000;
        font_rows[1985] = 32'h00000000;
        font_rows[1986] = 32'h00000000;
        font_rows[1987] = 32'h03000000;
        font_rows[1988] = 32'h030ffff0;
        font_rows[1989] = 32'h030ffff0;
        font_rows[1990] = 32'h03030060;
        font_rows[1991] = 32'h030180e0;
        font_rows[1992] = 32'h3ff1c1c0;
        font_rows[1993] = 32'h1ff0e780;
        font_rows[1994] = 32'h03007f00;
        font_rows[1995] = 32'h03007f00;
        font_rows[1996] = 32'h0307ffe0;
        font_rows[1997] = 32'h031fc1f8;
        font_rows[1998] = 32'h030e1838;
        font_rows[1999] = 32'h03e01800;
        font_rows[2000] = 32'h1fe01800;
        font_rows[2001] = 32'h3fcffff0;
        font_rows[2002] = 32'h3f0ffff0;
        font_rows[2003] = 32'h03001800;
        font_rows[2004] = 32'h03001800;
        font_rows[2005] = 32'h03001800;
        font_rows[2006] = 32'h031ffffc;
        font_rows[2007] = 32'h031ffffc;
        font_rows[2008] = 32'h03001800;
        font_rows[2009] = 32'h03001800;
        font_rows[2010] = 32'h1f001800;
        font_rows[2011] = 32'h1e001800;
        font_rows[2012] = 32'h00000000;
        font_rows[2013] = 32'h00000000;
        font_rows[2014] = 32'h00000000;
        font_rows[2015] = 32'h00000000;
        font_rows[2016] = 32'h00000000;
        font_rows[2017] = 32'h00000000;
        font_rows[2018] = 32'h00000000;
        font_rows[2019] = 32'h00018000;
        font_rows[2020] = 32'h02018040;
        font_rows[2021] = 32'h070180e0;
        font_rows[2022] = 32'h03c181c0;
        font_rows[2023] = 32'h01c18380;
        font_rows[2024] = 32'h00e18700;
        font_rows[2025] = 32'h00718e00;
        font_rows[2026] = 32'h00218400;
        font_rows[2027] = 32'h00018000;
        font_rows[2028] = 32'h0ffffff0;
        font_rows[2029] = 32'h0ffffff0;
        font_rows[2030] = 32'h00000070;
        font_rows[2031] = 32'h00000070;
        font_rows[2032] = 32'h00000070;
        font_rows[2033] = 32'h00000070;
        font_rows[2034] = 32'h07fffff0;
        font_rows[2035] = 32'h07fffff0;
        font_rows[2036] = 32'h00000070;
        font_rows[2037] = 32'h00000070;
        font_rows[2038] = 32'h00000070;
        font_rows[2039] = 32'h00000070;
        font_rows[2040] = 32'h00000070;
        font_rows[2041] = 32'h1ffffff0;
        font_rows[2042] = 32'h1ffffff0;
        font_rows[2043] = 32'h00000070;
        font_rows[2044] = 32'h00000070;
        font_rows[2045] = 32'h00000000;
        font_rows[2046] = 32'h00000000;
        font_rows[2047] = 32'h00000000;
        font_rows[2048] = 32'h00000000;
        font_rows[2049] = 32'h00000000;
        font_rows[2050] = 32'h00000000;
        font_rows[2051] = 32'h00300400;
        font_rows[2052] = 32'h00700e00;
        font_rows[2053] = 32'h00381c00;
        font_rows[2054] = 32'h00181800;
        font_rows[2055] = 32'h1ffffffc;
        font_rows[2056] = 32'h1ffffff8;
        font_rows[2057] = 32'h00000000;
        font_rows[2058] = 32'h00000000;
        font_rows[2059] = 32'h0fff8c70;
        font_rows[2060] = 32'h0fff8c70;
        font_rows[2061] = 32'h0e038c70;
        font_rows[2062] = 32'h0e038c70;
        font_rows[2063] = 32'h0fff8c70;
        font_rows[2064] = 32'h0fff8c70;
        font_rows[2065] = 32'h0e038c70;
        font_rows[2066] = 32'h0e038c70;
        font_rows[2067] = 32'h0e038c70;
        font_rows[2068] = 32'h0fff8c70;
        font_rows[2069] = 32'h0fff8c70;
        font_rows[2070] = 32'h0e038c70;
        font_rows[2071] = 32'h0e038070;
        font_rows[2072] = 32'h0e038070;
        font_rows[2073] = 32'h0e038070;
        font_rows[2074] = 32'h0e3f0fe0;
        font_rows[2075] = 32'h0e3f0fc0;
        font_rows[2076] = 32'h00000000;
        font_rows[2077] = 32'h00000000;
        font_rows[2078] = 32'h00000000;
        font_rows[2079] = 32'h00000000;
        font_rows[2080] = 32'h00000000;
        font_rows[2081] = 32'h00000000;
        font_rows[2082] = 32'h01c00000;
        font_rows[2083] = 32'h01c00080;
        font_rows[2084] = 32'h01c003c0;
        font_rows[2085] = 32'h01c00780;
        font_rows[2086] = 32'h01c01e00;
        font_rows[2087] = 32'h01c07c00;
        font_rows[2088] = 32'h01c1f000;
        font_rows[2089] = 32'h01c7c000;
        font_rows[2090] = 32'h01cf0000;
        font_rows[2091] = 32'h01cc0000;
        font_rows[2092] = 32'h01c00000;
        font_rows[2093] = 32'h1ffffff8;
        font_rows[2094] = 32'h1ffffff8;
        font_rows[2095] = 32'h01c1c000;
        font_rows[2096] = 32'h01c1c000;
        font_rows[2097] = 32'h01c0c000;
        font_rows[2098] = 32'h01c0e000;
        font_rows[2099] = 32'h01c07000;
        font_rows[2100] = 32'h01c03800;
        font_rows[2101] = 32'h01c03c00;
        font_rows[2102] = 32'h01c01e00;
        font_rows[2103] = 32'h01c18f80;
        font_rows[2104] = 32'h01cf83c0;
        font_rows[2105] = 32'h01ff01f8;
        font_rows[2106] = 32'h01fc0078;
        font_rows[2107] = 32'h01e00018;
        font_rows[2108] = 32'h00800000;
        font_rows[2109] = 32'h00000000;
        font_rows[2110] = 32'h00000000;
        font_rows[2111] = 32'h00000000;
        font_rows[2112] = 32'h00000000;
        font_rows[2113] = 32'h00000000;
        font_rows[2114] = 32'h03001000;
        font_rows[2115] = 32'h03003800;
        font_rows[2116] = 32'h03001c00;
        font_rows[2117] = 32'h03001c00;
        font_rows[2118] = 32'h030ffff8;
        font_rows[2119] = 32'h1feffff8;
        font_rows[2120] = 32'h1fec0038;
        font_rows[2121] = 32'h030c6038;
        font_rows[2122] = 32'h03007038;
        font_rows[2123] = 32'h03006000;
        font_rows[2124] = 32'h0300e000;
        font_rows[2125] = 32'h0360e000;
        font_rows[2126] = 32'h03fffffc;
        font_rows[2127] = 32'h07dffffc;
        font_rows[2128] = 32'h1f0181c0;
        font_rows[2129] = 32'h3f038180;
        font_rows[2130] = 32'h13070380;
        font_rows[2131] = 32'h03070380;
        font_rows[2132] = 32'h0307c700;
        font_rows[2133] = 32'h0300fe00;
        font_rows[2134] = 32'h03003e00;
        font_rows[2135] = 32'h03007f80;
        font_rows[2136] = 32'h0301f3c0;
        font_rows[2137] = 32'h030fc1f0;
        font_rows[2138] = 32'h1f3f0078;
        font_rows[2139] = 32'h1e380010;
        font_rows[2140] = 32'h00000000;
        font_rows[2141] = 32'h00000000;
        font_rows[2142] = 32'h00000000;
        font_rows[2143] = 32'h00000000;
        font_rows[2144] = 32'h00000000;
        font_rows[2145] = 32'h00000000;
        font_rows[2146] = 32'h00000000;
        font_rows[2147] = 32'h04038300;
        font_rows[2148] = 32'h0e038300;
        font_rows[2149] = 32'h06038300;
        font_rows[2150] = 32'h07038300;
        font_rows[2151] = 32'h03838300;
        font_rows[2152] = 32'h039ffff0;
        font_rows[2153] = 32'h011ffff0;
        font_rows[2154] = 32'h00038300;
        font_rows[2155] = 32'h00038300;
        font_rows[2156] = 32'h3f038300;
        font_rows[2157] = 32'h3f038300;
        font_rows[2158] = 32'h03038300;
        font_rows[2159] = 32'h033ffff8;
        font_rows[2160] = 32'h033ffff8;
        font_rows[2161] = 32'h03030300;
        font_rows[2162] = 32'h03030300;
        font_rows[2163] = 32'h03070300;
        font_rows[2164] = 32'h03060300;
        font_rows[2165] = 32'h030e0300;
        font_rows[2166] = 32'h031c0300;
        font_rows[2167] = 32'h073c0300;
        font_rows[2168] = 32'h0f900300;
        font_rows[2169] = 32'h1de00000;
        font_rows[2170] = 32'h187ffff8;
        font_rows[2171] = 32'h103ffff8;
        font_rows[2172] = 32'h00000000;
        font_rows[2173] = 32'h00000000;
        font_rows[2174] = 32'h00000000;
        font_rows[2175] = 32'h00000000;
        font_rows[2176] = 32'h00000000;
        font_rows[2177] = 32'h00000000;
        font_rows[2178] = 32'h00000000;
        font_rows[2179] = 32'h00180000;
        font_rows[2180] = 32'h003c0000;
        font_rows[2181] = 32'h001e0000;
        font_rows[2182] = 32'h000e0000;
        font_rows[2183] = 32'h00070000;
        font_rows[2184] = 32'h00038000;
        font_rows[2185] = 32'h00038000;
        font_rows[2186] = 32'h0003c000;
        font_rows[2187] = 32'h0007c000;
        font_rows[2188] = 32'h0007e000;
        font_rows[2189] = 32'h00066000;
        font_rows[2190] = 32'h000e7000;
        font_rows[2191] = 32'h000e3000;
        font_rows[2192] = 32'h001c3800;
        font_rows[2193] = 32'h001c1800;
        font_rows[2194] = 32'h00381c00;
        font_rows[2195] = 32'h00700e00;
        font_rows[2196] = 32'h00700e00;
        font_rows[2197] = 32'h00e00700;
        font_rows[2198] = 32'h01c00380;
        font_rows[2199] = 32'h038001c0;
        font_rows[2200] = 32'h070001e0;
        font_rows[2201] = 32'h1e0000f8;
        font_rows[2202] = 32'h1c000078;
        font_rows[2203] = 32'h18000010;
        font_rows[2204] = 32'h00000000;
        font_rows[2205] = 32'h00000000;
        font_rows[2206] = 32'h00000000;
        font_rows[2207] = 32'h00000000;
        font_rows[2208] = 32'h00000000;
        font_rows[2209] = 32'h00000000;
        font_rows[2210] = 32'h00000000;
        font_rows[2211] = 32'h040003f0;
        font_rows[2212] = 32'h0e0ffff8;
        font_rows[2213] = 32'h070fff80;
        font_rows[2214] = 32'h038c0000;
        font_rows[2215] = 32'h038c0000;
        font_rows[2216] = 32'h018c0000;
        font_rows[2217] = 32'h000c0000;
        font_rows[2218] = 32'h000ffff0;
        font_rows[2219] = 32'h000ffff0;
        font_rows[2220] = 32'h3f0c0060;
        font_rows[2221] = 32'h3f0c80e0;
        font_rows[2222] = 32'h030dc0e0;
        font_rows[2223] = 32'h030cf1c0;
        font_rows[2224] = 32'h030c7b80;
        font_rows[2225] = 32'h031c3f80;
        font_rows[2226] = 32'h031c0f00;
        font_rows[2227] = 32'h03181f80;
        font_rows[2228] = 32'h03383fc0;
        font_rows[2229] = 32'h0370f1e0;
        font_rows[2230] = 32'h0373e0f0;
        font_rows[2231] = 32'h07278078;
        font_rows[2232] = 32'h0f860030;
        font_rows[2233] = 32'h1de00000;
        font_rows[2234] = 32'h18fffff8;
        font_rows[2235] = 32'h103ffff8;
        font_rows[2236] = 32'h00000000;
        font_rows[2237] = 32'h00000000;
        font_rows[2238] = 32'h00000000;
        font_rows[2239] = 32'h00000000;
        font_rows[2240] = 32'h00000000;
        font_rows[2241] = 32'h00000000;
        font_rows[2242] = 32'h00000000;
        font_rows[2243] = 32'h0ffffff0;
        font_rows[2244] = 32'h0ffffff0;
        font_rows[2245] = 32'h0e000030;
        font_rows[2246] = 32'h0e000030;
        font_rows[2247] = 32'h0e000030;
        font_rows[2248] = 32'h0e000030;
        font_rows[2249] = 32'h0e3ffc30;
        font_rows[2250] = 32'h0e3ffc30;
        font_rows[2251] = 32'h0e300c30;
        font_rows[2252] = 32'h0e300c30;
        font_rows[2253] = 32'h0e300c30;
        font_rows[2254] = 32'h0e300c30;
        font_rows[2255] = 32'h0e300c30;
        font_rows[2256] = 32'h0e300c30;
        font_rows[2257] = 32'h0e300c30;
        font_rows[2258] = 32'h0e3ffc30;
        font_rows[2259] = 32'h0e3ffc30;
        font_rows[2260] = 32'h0e300c30;
        font_rows[2261] = 32'h0e300c30;
        font_rows[2262] = 32'h0e000030;
        font_rows[2263] = 32'h0e000030;
        font_rows[2264] = 32'h0e000030;
        font_rows[2265] = 32'h0ffffff0;
        font_rows[2266] = 32'h0ffffff0;
        font_rows[2267] = 32'h0e000030;
        font_rows[2268] = 32'h0e000030;
        font_rows[2269] = 32'h00000000;
        font_rows[2270] = 32'h00000000;
        font_rows[2271] = 32'h00000000;
        font_rows[2272] = 32'h00000000;
        font_rows[2273] = 32'h00000000;
        font_rows[2274] = 32'h00000000;
        font_rows[2275] = 32'h01806300;
        font_rows[2276] = 32'h01c06380;
        font_rows[2277] = 32'h038061e0;
        font_rows[2278] = 32'h03006040;
        font_rows[2279] = 32'h07006000;
        font_rows[2280] = 32'h062070f8;
        font_rows[2281] = 32'h0e71fff8;
        font_rows[2282] = 32'h0c7fff80;
        font_rows[2283] = 32'h18eff000;
        font_rows[2284] = 32'h3fc07000;
        font_rows[2285] = 32'h1fc070f8;
        font_rows[2286] = 32'h0181fffc;
        font_rows[2287] = 32'h038fffc0;
        font_rows[2288] = 32'h070ff000;
        font_rows[2289] = 32'h06003020;
        font_rows[2290] = 32'h0e703870;
        font_rows[2291] = 32'h1ff039e0;
        font_rows[2292] = 32'h1f801bc0;
        font_rows[2293] = 32'h1c001f80;
        font_rows[2294] = 32'h00f01e18;
        font_rows[2295] = 32'h0ff07e1c;
        font_rows[2296] = 32'h1fc1fe18;
        font_rows[2297] = 32'h1c07c7b8;
        font_rows[2298] = 32'h001f03f8;
        font_rows[2299] = 32'h001c00f0;
        font_rows[2300] = 32'h00000000;
        font_rows[2301] = 32'h00000000;
        font_rows[2302] = 32'h00000000;
        font_rows[2303] = 32'h00000000;
        font_rows[2304] = 32'h00000000;
        font_rows[2305] = 32'h00000000;
        font_rows[2306] = 32'h001c0000;
        font_rows[2307] = 32'h00380000;
        font_rows[2308] = 32'h00700000;
        font_rows[2309] = 32'h007fffc0;
        font_rows[2310] = 32'h00ffff80;
        font_rows[2311] = 32'h01f00380;
        font_rows[2312] = 32'h03b80f00;
        font_rows[2313] = 32'h071c1e00;
        font_rows[2314] = 32'h0e0f7800;
        font_rows[2315] = 32'h0407f000;
        font_rows[2316] = 32'h0007f000;
        font_rows[2317] = 32'h003fffe0;
        font_rows[2318] = 32'h03fc0ff8;
        font_rows[2319] = 32'h1fc180f8;
        font_rows[2320] = 32'h1c018000;
        font_rows[2321] = 32'h00018000;
        font_rows[2322] = 32'h07ffffe0;
        font_rows[2323] = 32'h07ffffe0;
        font_rows[2324] = 32'h00018000;
        font_rows[2325] = 32'h00618600;
        font_rows[2326] = 32'h00f18f00;
        font_rows[2327] = 32'h03c183c0;
        font_rows[2328] = 32'h078180f0;
        font_rows[2329] = 32'h1e018078;
        font_rows[2330] = 32'h0c3f8010;
        font_rows[2331] = 32'h003f8000;
        font_rows[2332] = 32'h00000000;
        font_rows[2333] = 32'h00000000;
        font_rows[2334] = 32'h00000000;
        font_rows[2335] = 32'h00000000;
        font_rows[2336] = 32'h00000000;
        font_rows[2337] = 32'h00000000;
        font_rows[2338] = 32'h00000000;
        font_rows[2339] = 32'h0c000000;
        font_rows[2340] = 32'h0f1ffff8;
        font_rows[2341] = 32'h079ffff8;
        font_rows[2342] = 32'h01d80038;
        font_rows[2343] = 32'h00980038;
        font_rows[2344] = 32'h0018c638;
        font_rows[2345] = 32'h0001c780;
        font_rows[2346] = 32'h180783c0;
        font_rows[2347] = 32'h1c0f00e0;
        font_rows[2348] = 32'h0f1c3870;
        font_rows[2349] = 32'h07983830;
        font_rows[2350] = 32'h01003800;
        font_rows[2351] = 32'h003ffff8;
        font_rows[2352] = 32'h003ffff8;
        font_rows[2353] = 32'h03007c00;
        font_rows[2354] = 32'h0300fc00;
        font_rows[2355] = 32'h0701fe00;
        font_rows[2356] = 32'h0703bb00;
        font_rows[2357] = 32'h070739c0;
        font_rows[2358] = 32'h060e38e0;
        font_rows[2359] = 32'h0e3c3878;
        font_rows[2360] = 32'h0e783838;
        font_rows[2361] = 32'h1c203818;
        font_rows[2362] = 32'h1c003800;
        font_rows[2363] = 32'h0c003800;
        font_rows[2364] = 32'h00000000;
        font_rows[2365] = 32'h00000000;
        font_rows[2366] = 32'h00000000;
        font_rows[2367] = 32'h00000000;
        font_rows[2368] = 32'h00000000;
        font_rows[2369] = 32'h00000000;
        font_rows[2370] = 32'h0301c380;
        font_rows[2371] = 32'h0301c380;
        font_rows[2372] = 32'h0301c380;
        font_rows[2373] = 32'h031ffffc;
        font_rows[2374] = 32'h031ffffc;
        font_rows[2375] = 32'h3fe1c380;
        font_rows[2376] = 32'h1fe1c380;
        font_rows[2377] = 32'h0301c380;
        font_rows[2378] = 32'h03000000;
        font_rows[2379] = 32'h030ffff0;
        font_rows[2380] = 32'h030ffff0;
        font_rows[2381] = 32'h03ec1830;
        font_rows[2382] = 32'h07ec1830;
        font_rows[2383] = 32'h1f8c1830;
        font_rows[2384] = 32'h3f0c1830;
        font_rows[2385] = 32'h130c1830;
        font_rows[2386] = 32'h030ffff0;
        font_rows[2387] = 32'h030ffff0;
        font_rows[2388] = 32'h030c1830;
        font_rows[2389] = 32'h030c1830;
        font_rows[2390] = 32'h030c1830;
        font_rows[2391] = 32'h030c1830;
        font_rows[2392] = 32'h030c1830;
        font_rows[2393] = 32'h030ffff0;
        font_rows[2394] = 32'h3f0ffff0;
        font_rows[2395] = 32'h1e0c0030;
        font_rows[2396] = 32'h00000000;
        font_rows[2397] = 32'h00000000;
        font_rows[2398] = 32'h00000000;
        font_rows[2399] = 32'h00000000;
        font_rows[2400] = 32'h00000000;
        font_rows[2401] = 32'h00000000;
        font_rows[2402] = 32'h00000000;
        font_rows[2403] = 32'h0400e000;
        font_rows[2404] = 32'h0e00e000;
        font_rows[2405] = 32'h0e00e000;
        font_rows[2406] = 32'h0700e000;
        font_rows[2407] = 32'h0380e000;
        font_rows[2408] = 32'h039ffff0;
        font_rows[2409] = 32'h011ffff0;
        font_rows[2410] = 32'h0000e070;
        font_rows[2411] = 32'h0000e070;
        font_rows[2412] = 32'h0000e070;
        font_rows[2413] = 32'h3f00c070;
        font_rows[2414] = 32'h3f00c070;
        font_rows[2415] = 32'h0301c070;
        font_rows[2416] = 32'h0301c070;
        font_rows[2417] = 32'h0301c070;
        font_rows[2418] = 32'h03038070;
        font_rows[2419] = 32'h03038060;
        font_rows[2420] = 32'h03070060;
        font_rows[2421] = 32'h030f0060;
        font_rows[2422] = 32'h031e00e0;
        font_rows[2423] = 32'h033c3fe0;
        font_rows[2424] = 32'h07b81fc0;
        font_rows[2425] = 32'h0ff00000;
        font_rows[2426] = 32'h1cf80000;
        font_rows[2427] = 32'h187ffff8;
        font_rows[2428] = 32'h101ffff8;
        font_rows[2429] = 32'h00000000;
        font_rows[2430] = 32'h00000000;
        font_rows[2431] = 32'h00000000;
        font_rows[2432] = 32'h00000000;
        font_rows[2433] = 32'h00000000;
        font_rows[2434] = 32'h00000000;
        font_rows[2435] = 32'h07ffff80;
        font_rows[2436] = 32'h07ffff80;
        font_rows[2437] = 32'h06000180;
        font_rows[2438] = 32'h06001980;
        font_rows[2439] = 32'h06203980;
        font_rows[2440] = 32'h06703180;
        font_rows[2441] = 32'h06387180;
        font_rows[2442] = 32'h06386180;
        font_rows[2443] = 32'h061ce180;
        font_rows[2444] = 32'h060ec180;
        font_rows[2445] = 32'h0607c180;
        font_rows[2446] = 32'h06078180;
        font_rows[2447] = 32'h06038180;
        font_rows[2448] = 32'h06078180;
        font_rows[2449] = 32'h060fc180;
        font_rows[2450] = 32'h060ee180;
        font_rows[2451] = 32'h061c6198;
        font_rows[2452] = 32'h0e38719c;
        font_rows[2453] = 32'h0e7839dc;
        font_rows[2454] = 32'h0e7039d8;
        font_rows[2455] = 32'h0ce01dd8;
        font_rows[2456] = 32'h1c4008d8;
        font_rows[2457] = 32'h180000f8;
        font_rows[2458] = 32'h38000070;
        font_rows[2459] = 32'h10000020;
        font_rows[2460] = 32'h00000000;
        font_rows[2461] = 32'h00000000;
        font_rows[2462] = 32'h00000000;
        font_rows[2463] = 32'h00000000;
        font_rows[2464] = 32'h00000000;
        font_rows[2465] = 32'h00000000;
        font_rows[2466] = 32'h00000000;
        font_rows[2467] = 32'h03806000;
        font_rows[2468] = 32'h0380e000;
        font_rows[2469] = 32'h0380c000;
        font_rows[2470] = 32'h0381fff0;
        font_rows[2471] = 32'h0381fff0;
        font_rows[2472] = 32'h3ff38070;
        font_rows[2473] = 32'h3ff7c0e0;
        font_rows[2474] = 32'h0386e1c0;
        font_rows[2475] = 32'h038c7380;
        font_rows[2476] = 32'h03c43f00;
        font_rows[2477] = 32'h07e01e00;
        font_rows[2478] = 32'h07f07f80;
        font_rows[2479] = 32'h0fb1f3f8;
        font_rows[2480] = 32'h0f9fc0f8;
        font_rows[2481] = 32'h0f9f0018;
        font_rows[2482] = 32'h1b8c0000;
        font_rows[2483] = 32'h3b83fff0;
        font_rows[2484] = 32'h1383fff0;
        font_rows[2485] = 32'h13830070;
        font_rows[2486] = 32'h03830070;
        font_rows[2487] = 32'h03830070;
        font_rows[2488] = 32'h03830070;
        font_rows[2489] = 32'h03830070;
        font_rows[2490] = 32'h0383fff0;
        font_rows[2491] = 32'h0383fff0;
        font_rows[2492] = 32'h03830070;
        font_rows[2493] = 32'h00000000;
        font_rows[2494] = 32'h00000000;
        font_rows[2495] = 32'h00000000;
        font_rows[2496] = 32'h00000000;
        font_rows[2497] = 32'h00000000;
        font_rows[2498] = 32'h01818000;
        font_rows[2499] = 32'h03818000;
        font_rows[2500] = 32'h0301ffc0;
        font_rows[2501] = 32'h0703ffc0;
        font_rows[2502] = 32'h060300c0;
        font_rows[2503] = 32'h0e3300c0;
        font_rows[2504] = 32'h0c33ffc0;
        font_rows[2505] = 32'h1c63ffc0;
        font_rows[2506] = 32'h18e00180;
        font_rows[2507] = 32'h1fdffff8;
        font_rows[2508] = 32'h1f9ffff8;
        font_rows[2509] = 32'h01807000;
        font_rows[2510] = 32'h0301f020;
        font_rows[2511] = 32'h0607b8e0;
        font_rows[2512] = 32'h0e0f39c0;
        font_rows[2513] = 32'h1fecff80;
        font_rows[2514] = 32'h1fe1cf80;
        font_rows[2515] = 32'h1c078cc0;
        font_rows[2516] = 32'h001e1ec0;
        font_rows[2517] = 32'h00187e60;
        font_rows[2518] = 32'h0fe0e660;
        font_rows[2519] = 32'h1fe1c630;
        font_rows[2520] = 32'h1e078e38;
        font_rows[2521] = 32'h001e0e1c;
        font_rows[2522] = 32'h0018fc08;
        font_rows[2523] = 32'h0000f800;
        font_rows[2524] = 32'h00000000;
        font_rows[2525] = 32'h00000000;
        font_rows[2526] = 32'h00000000;
        font_rows[2527] = 32'h00000000;
        font_rows[2528] = 32'h00000000;
        font_rows[2529] = 32'h00000000;
        font_rows[2530] = 32'h02000000;
        font_rows[2531] = 32'h03000000;
        font_rows[2532] = 32'h038ffff0;
        font_rows[2533] = 32'h01c7fff0;
        font_rows[2534] = 32'h00e00830;
        font_rows[2535] = 32'h00406c30;
        font_rows[2536] = 32'h0c006630;
        font_rows[2537] = 32'h0c006230;
        font_rows[2538] = 32'h0dffffb0;
        font_rows[2539] = 32'h0dffffb0;
        font_rows[2540] = 32'h0c006030;
        font_rows[2541] = 32'h0c006330;
        font_rows[2542] = 32'h0cff6330;
        font_rows[2543] = 32'h0cff6630;
        font_rows[2544] = 32'h0cc37630;
        font_rows[2545] = 32'h0cc33c30;
        font_rows[2546] = 32'h0cc33c30;
        font_rows[2547] = 32'h0cff3830;
        font_rows[2548] = 32'h0cff39b0;
        font_rows[2549] = 32'h0cc079b0;
        font_rows[2550] = 32'h0c00dfb0;
        font_rows[2551] = 32'h0c3f8fb0;
        font_rows[2552] = 32'h0dff07b0;
        font_rows[2553] = 32'h0de00330;
        font_rows[2554] = 32'h0c000030;
        font_rows[2555] = 32'h0c0007f0;
        font_rows[2556] = 32'h0c0007e0;
        font_rows[2557] = 32'h00000000;
        font_rows[2558] = 32'h00000000;
        font_rows[2559] = 32'h00000000;
        font_rows[2560] = 32'h00000000;
        font_rows[2561] = 32'h00000000;
        font_rows[2562] = 32'h00c03800;
        font_rows[2563] = 32'h01c03800;
        font_rows[2564] = 32'h01c03800;
        font_rows[2565] = 32'h01bffff8;
        font_rows[2566] = 32'h01bffff8;
        font_rows[2567] = 32'h03003800;
        font_rows[2568] = 32'h03003800;
        font_rows[2569] = 32'h070fffe0;
        font_rows[2570] = 32'h070fffe0;
        font_rows[2571] = 32'h0f0c0060;
        font_rows[2572] = 32'h0f0c0060;
        font_rows[2573] = 32'h1f0c0060;
        font_rows[2574] = 32'h1f0fffe0;
        font_rows[2575] = 32'h3b0fffe0;
        font_rows[2576] = 32'h330c0060;
        font_rows[2577] = 32'h130c0060;
        font_rows[2578] = 32'h030fffe0;
        font_rows[2579] = 32'h030fffe0;
        font_rows[2580] = 32'h030c0060;
        font_rows[2581] = 32'h030c0060;
        font_rows[2582] = 32'h030fffe0;
        font_rows[2583] = 32'h030fffe0;
        font_rows[2584] = 32'h030c0060;
        font_rows[2585] = 32'h030c0060;
        font_rows[2586] = 32'h037ffffc;
        font_rows[2587] = 32'h037ffffc;
        font_rows[2588] = 32'h03000000;
        font_rows[2589] = 32'h00000000;
        font_rows[2590] = 32'h00000000;
        font_rows[2591] = 32'h00000000;
        font_rows[2592] = 32'h00000000;
        font_rows[2593] = 32'h00000000;
        font_rows[2594] = 32'h00000000;
        font_rows[2595] = 32'h00018000;
        font_rows[2596] = 32'h00018000;
        font_rows[2597] = 32'h02018040;
        font_rows[2598] = 32'h070180e0;
        font_rows[2599] = 32'h038183c0;
        font_rows[2600] = 32'h01e18780;
        font_rows[2601] = 32'h00e18f00;
        font_rows[2602] = 32'h00618400;
        font_rows[2603] = 32'h00018000;
        font_rows[2604] = 32'h00018000;
        font_rows[2605] = 32'h1ffffffc;
        font_rows[2606] = 32'h1ffffffc;
        font_rows[2607] = 32'h00183800;
        font_rows[2608] = 32'h00383800;
        font_rows[2609] = 32'h00383800;
        font_rows[2610] = 32'h00383800;
        font_rows[2611] = 32'h00383800;
        font_rows[2612] = 32'h00303800;
        font_rows[2613] = 32'h00703818;
        font_rows[2614] = 32'h0070381c;
        font_rows[2615] = 32'h00e0381c;
        font_rows[2616] = 32'h01c0381c;
        font_rows[2617] = 32'h03c03838;
        font_rows[2618] = 32'h0f003ff8;
        font_rows[2619] = 32'h3e001ff0;
        font_rows[2620] = 32'h18000000;
        font_rows[2621] = 32'h00000000;
        font_rows[2622] = 32'h00000000;
        font_rows[2623] = 32'h00000000;
        font_rows[2624] = 32'h00000000;
        font_rows[2625] = 32'h00000000;
        font_rows[2626] = 32'h00000000;
        font_rows[2627] = 32'h00000000;
        font_rows[2628] = 32'h0ff9fff0;
        font_rows[2629] = 32'h0ff9fff0;
        font_rows[2630] = 32'h0c183030;
        font_rows[2631] = 32'h0c183030;
        font_rows[2632] = 32'h0c187030;
        font_rows[2633] = 32'h0c186070;
        font_rows[2634] = 32'h0c18e7e0;
        font_rows[2635] = 32'h0ffbc7c0;
        font_rows[2636] = 32'h0ffb0000;
        font_rows[2637] = 32'h0c18fff0;
        font_rows[2638] = 32'h0c18fff0;
        font_rows[2639] = 32'h0c18c030;
        font_rows[2640] = 32'h0c18c030;
        font_rows[2641] = 32'h0c18c030;
        font_rows[2642] = 32'h0ff8fff0;
        font_rows[2643] = 32'h0ff8fff0;
        font_rows[2644] = 32'h0c18c030;
        font_rows[2645] = 32'h00003000;
        font_rows[2646] = 32'h06183040;
        font_rows[2647] = 32'h061830e0;
        font_rows[2648] = 32'h0e1c3870;
        font_rows[2649] = 32'h0c0c1830;
        font_rows[2650] = 32'h1c0e1c38;
        font_rows[2651] = 32'h180c0810;
        font_rows[2652] = 32'h00000000;
        font_rows[2653] = 32'h00000000;
        font_rows[2654] = 32'h00000000;
        font_rows[2655] = 32'h00000000;
        font_rows[2656] = 32'h00000000;
        font_rows[2657] = 32'h00010000;
        font_rows[2658] = 32'h00038000;
        font_rows[2659] = 32'h0001c000;
        font_rows[2660] = 32'h0001c000;
        font_rows[2661] = 32'h00008000;
        font_rows[2662] = 32'h1ffffff8;
        font_rows[2663] = 32'h1ffffff8;
        font_rows[2664] = 32'h00380000;
        font_rows[2665] = 32'h00380000;
        font_rows[2666] = 32'h00380000;
        font_rows[2667] = 32'h00380000;
        font_rows[2668] = 32'h00380000;
        font_rows[2669] = 32'h003fffc0;
        font_rows[2670] = 32'h003fffc0;
        font_rows[2671] = 32'h003001c0;
        font_rows[2672] = 32'h003001c0;
        font_rows[2673] = 32'h00700180;
        font_rows[2674] = 32'h00700180;
        font_rows[2675] = 32'h00700180;
        font_rows[2676] = 32'h00e00180;
        font_rows[2677] = 32'h00e00180;
        font_rows[2678] = 32'h00c00180;
        font_rows[2679] = 32'h01c00380;
        font_rows[2680] = 32'h03800380;
        font_rows[2681] = 32'h07800380;
        font_rows[2682] = 32'h0f000780;
        font_rows[2683] = 32'h1e03ff00;
        font_rows[2684] = 32'h3c03fe00;
        font_rows[2685] = 32'h18000000;
        font_rows[2686] = 32'h00000000;
        font_rows[2687] = 32'h00000000;
        font_rows[2688] = 32'h00000000;
        font_rows[2689] = 32'h00000000;
        font_rows[2690] = 32'h00000000;
        font_rows[2691] = 32'h00070000;
        font_rows[2692] = 32'h00070000;
        font_rows[2693] = 32'h00060000;
        font_rows[2694] = 32'h00060000;
        font_rows[2695] = 32'h0ffffff0;
        font_rows[2696] = 32'h0ffffff0;
        font_rows[2697] = 32'h0e000070;
        font_rows[2698] = 32'h0e000070;
        font_rows[2699] = 32'h0e000070;
        font_rows[2700] = 32'h0e000070;
        font_rows[2701] = 32'h0e3ffc70;
        font_rows[2702] = 32'h0e3ffc70;
        font_rows[2703] = 32'h0e301c70;
        font_rows[2704] = 32'h0e301c70;
        font_rows[2705] = 32'h0e301c70;
        font_rows[2706] = 32'h0e301c70;
        font_rows[2707] = 32'h0e301c70;
        font_rows[2708] = 32'h0e3ffc70;
        font_rows[2709] = 32'h0e3ffc70;
        font_rows[2710] = 32'h0e300070;
        font_rows[2711] = 32'h0e300070;
        font_rows[2712] = 32'h0e000070;
        font_rows[2713] = 32'h0e000070;
        font_rows[2714] = 32'h0e000070;
        font_rows[2715] = 32'h0e000fe0;
        font_rows[2716] = 32'h0e000fe0;
        font_rows[2717] = 32'h00000000;
        font_rows[2718] = 32'h00000000;
        font_rows[2719] = 32'h00000000;
        font_rows[2720] = 32'h00000000;
        font_rows[2721] = 32'h00000000;
        font_rows[2722] = 32'h00000000;
        font_rows[2723] = 32'h00000000;
        font_rows[2724] = 32'h07ffffe0;
        font_rows[2725] = 32'h07ffffe0;
        font_rows[2726] = 32'h06718760;
        font_rows[2727] = 32'h06399e60;
        font_rows[2728] = 32'h06119860;
        font_rows[2729] = 32'h07ffffe0;
        font_rows[2730] = 32'h07ffffe0;
        font_rows[2731] = 32'h06018060;
        font_rows[2732] = 32'h00018000;
        font_rows[2733] = 32'h0fffffe0;
        font_rows[2734] = 32'h07ffffe0;
        font_rows[2735] = 32'h00018000;
        font_rows[2736] = 32'h1ffffff8;
        font_rows[2737] = 32'h1ffffff8;
        font_rows[2738] = 32'h06183860;
        font_rows[2739] = 32'h0e018070;
        font_rows[2740] = 32'h0c018030;
        font_rows[2741] = 32'h0ffffff0;
        font_rows[2742] = 32'h07ffffe0;
        font_rows[2743] = 32'h00018000;
        font_rows[2744] = 32'h00018000;
        font_rows[2745] = 32'h3ffffffc;
        font_rows[2746] = 32'h1ffffff8;
        font_rows[2747] = 32'h00000000;
        font_rows[2748] = 32'h00000000;
        font_rows[2749] = 32'h00000000;
        font_rows[2750] = 32'h00000000;
        font_rows[2751] = 32'h00000000;
        font_rows[2752] = 32'h00000000;
        font_rows[2753] = 32'h00000000;
        font_rows[2754] = 32'h00000000;
        font_rows[2755] = 32'h007c0000;
        font_rows[2756] = 32'h0ffefff0;
        font_rows[2757] = 32'h0fe0fff0;
        font_rows[2758] = 32'h0c103830;
        font_rows[2759] = 32'h0c383830;
        font_rows[2760] = 32'h0c1c3030;
        font_rows[2761] = 32'h0c1c3030;
        font_rows[2762] = 32'h0c7e7070;
        font_rows[2763] = 32'h0ff6e070;
        font_rows[2764] = 32'h0fc7efe0;
        font_rows[2765] = 32'h0f03cfc0;
        font_rows[2766] = 32'h0c018000;
        font_rows[2767] = 32'h07ffffe0;
        font_rows[2768] = 32'h07ffffe0;
        font_rows[2769] = 32'h070180e0;
        font_rows[2770] = 32'h070180e0;
        font_rows[2771] = 32'h070180e0;
        font_rows[2772] = 32'h07ffffe0;
        font_rows[2773] = 32'h07ffffe0;
        font_rows[2774] = 32'h070180e0;
        font_rows[2775] = 32'h070180e0;
        font_rows[2776] = 32'h070180e0;
        font_rows[2777] = 32'h07ffffe0;
        font_rows[2778] = 32'h07ffffe0;
        font_rows[2779] = 32'h070000e0;
        font_rows[2780] = 32'h00000000;
        font_rows[2781] = 32'h00000000;
        font_rows[2782] = 32'h00000000;
        font_rows[2783] = 32'h00000000;
        font_rows[2784] = 32'h00000000;
        font_rows[2785] = 32'h00000000;
        font_rows[2786] = 32'h00000000;
        font_rows[2787] = 32'h00070000;
        font_rows[2788] = 32'h00070000;
        font_rows[2789] = 32'h00060000;
        font_rows[2790] = 32'h000e0000;
        font_rows[2791] = 32'h07ffffe0;
        font_rows[2792] = 32'h07ffffe0;
        font_rows[2793] = 32'h06000060;
        font_rows[2794] = 32'h06000060;
        font_rows[2795] = 32'h06000060;
        font_rows[2796] = 32'h06000060;
        font_rows[2797] = 32'h06000060;
        font_rows[2798] = 32'h06000060;
        font_rows[2799] = 32'h06000060;
        font_rows[2800] = 32'h07ffffe0;
        font_rows[2801] = 32'h07ffffe0;
        font_rows[2802] = 32'h06000060;
        font_rows[2803] = 32'h06000060;
        font_rows[2804] = 32'h06000060;
        font_rows[2805] = 32'h06000060;
        font_rows[2806] = 32'h06000060;
        font_rows[2807] = 32'h06000060;
        font_rows[2808] = 32'h06000060;
        font_rows[2809] = 32'h07ffffe0;
        font_rows[2810] = 32'h07ffffe0;
        font_rows[2811] = 32'h06000060;
        font_rows[2812] = 32'h06000060;
        font_rows[2813] = 32'h00000000;
        font_rows[2814] = 32'h00000000;
        font_rows[2815] = 32'h00000000;
        font_rows[2816] = 32'h00000000;
        font_rows[2817] = 32'h00000000;
        font_rows[2818] = 32'h00380000;
        font_rows[2819] = 32'h00780000;
        font_rows[2820] = 32'h00700000;
        font_rows[2821] = 32'h00fffc00;
        font_rows[2822] = 32'h01fffc00;
        font_rows[2823] = 32'h03801c00;
        font_rows[2824] = 32'h07003800;
        font_rows[2825] = 32'h0e007000;
        font_rows[2826] = 32'h3fffffe0;
        font_rows[2827] = 32'h1fffffe0;
        font_rows[2828] = 32'h070180e0;
        font_rows[2829] = 32'h070180e0;
        font_rows[2830] = 32'h070180e0;
        font_rows[2831] = 32'h070180e0;
        font_rows[2832] = 32'h070180e0;
        font_rows[2833] = 32'h07ffffe0;
        font_rows[2834] = 32'h07ffffe0;
        font_rows[2835] = 32'h070000e0;
        font_rows[2836] = 32'h07000000;
        font_rows[2837] = 32'h07000018;
        font_rows[2838] = 32'h07000018;
        font_rows[2839] = 32'h07000018;
        font_rows[2840] = 32'h03000038;
        font_rows[2841] = 32'h03000038;
        font_rows[2842] = 32'h03fffff0;
        font_rows[2843] = 32'h01ffffe0;
        font_rows[2844] = 32'h00000000;
        font_rows[2845] = 32'h00000000;
        font_rows[2846] = 32'h00000000;
        font_rows[2847] = 32'h00000000;
        font_rows[2848] = 32'h00000000;
        font_rows[2849] = 32'h00000000;
        font_rows[2850] = 32'h00000000;
        font_rows[2851] = 32'h007fc020;
        font_rows[2852] = 32'h1fffc070;
        font_rows[2853] = 32'h1ff800e0;
        font_rows[2854] = 32'h004081c0;
        font_rows[2855] = 32'h0861c780;
        font_rows[2856] = 32'h0c618f00;
        font_rows[2857] = 32'h0c73bc00;
        font_rows[2858] = 32'h0e337830;
        font_rows[2859] = 32'h06372070;
        font_rows[2860] = 32'h060200f0;
        font_rows[2861] = 32'h007001e0;
        font_rows[2862] = 32'h007003c0;
        font_rows[2863] = 32'h1fff8f00;
        font_rows[2864] = 32'h1fff9e00;
        font_rows[2865] = 32'h00703c00;
        font_rows[2866] = 32'h00f01018;
        font_rows[2867] = 32'h01f80038;
        font_rows[2868] = 32'h03fc0078;
        font_rows[2869] = 32'h037f00f0;
        font_rows[2870] = 32'h067781e0;
        font_rows[2871] = 32'h1c7383c0;
        font_rows[2872] = 32'h1c708f00;
        font_rows[2873] = 32'h18703e00;
        font_rows[2874] = 32'h0070f800;
        font_rows[2875] = 32'h0071f000;
        font_rows[2876] = 32'h0070c000;
        font_rows[2877] = 32'h00000000;
        font_rows[2878] = 32'h00000000;
        font_rows[2879] = 32'h00000000;
        font_rows[2880] = 32'h00000000;
        font_rows[2881] = 32'h00000000;
        font_rows[2882] = 32'h00000000;
        font_rows[2883] = 32'h000c0000;
        font_rows[2884] = 32'h000c0000;
        font_rows[2885] = 32'h001c0000;
        font_rows[2886] = 32'h001c0000;
        font_rows[2887] = 32'h1ffffff8;
        font_rows[2888] = 32'h1ffffff8;
        font_rows[2889] = 32'h00380000;
        font_rows[2890] = 32'h00380000;
        font_rows[2891] = 32'h00300000;
        font_rows[2892] = 32'h00700000;
        font_rows[2893] = 32'h00700000;
        font_rows[2894] = 32'h00600000;
        font_rows[2895] = 32'h00fffff0;
        font_rows[2896] = 32'h00fffff0;
        font_rows[2897] = 32'h01c06000;
        font_rows[2898] = 32'h03c06000;
        font_rows[2899] = 32'h03806000;
        font_rows[2900] = 32'h07006000;
        font_rows[2901] = 32'h0f006000;
        font_rows[2902] = 32'h0e006000;
        font_rows[2903] = 32'h1c006000;
        font_rows[2904] = 32'h38006000;
        font_rows[2905] = 32'h11fffff8;
        font_rows[2906] = 32'h01fffff8;
        font_rows[2907] = 32'h00000000;
        font_rows[2908] = 32'h00000000;
        font_rows[2909] = 32'h00000000;
        font_rows[2910] = 32'h00000000;
        font_rows[2911] = 32'h00000000;
        font_rows[2912] = 32'h00000000;
        font_rows[2913] = 32'h00000000;
        font_rows[2914] = 32'h00000000;
        font_rows[2915] = 32'h00070000;
        font_rows[2916] = 32'h00070000;
        font_rows[2917] = 32'h00070000;
        font_rows[2918] = 32'h00070000;
        font_rows[2919] = 32'h00070000;
        font_rows[2920] = 32'h00070000;
        font_rows[2921] = 32'h00070000;
        font_rows[2922] = 32'h0007ffe0;
        font_rows[2923] = 32'h0007ffe0;
        font_rows[2924] = 32'h00070000;
        font_rows[2925] = 32'h00070000;
        font_rows[2926] = 32'h00070000;
        font_rows[2927] = 32'h00070000;
        font_rows[2928] = 32'h00070000;
        font_rows[2929] = 32'h00070000;
        font_rows[2930] = 32'h00070000;
        font_rows[2931] = 32'h00070000;
        font_rows[2932] = 32'h00070000;
        font_rows[2933] = 32'h00070000;
        font_rows[2934] = 32'h00070000;
        font_rows[2935] = 32'h00070000;
        font_rows[2936] = 32'h00070000;
        font_rows[2937] = 32'h1ffffff8;
        font_rows[2938] = 32'h1ffffff8;
        font_rows[2939] = 32'h00000000;
        font_rows[2940] = 32'h00000000;
        font_rows[2941] = 32'h00000000;
        font_rows[2942] = 32'h00000000;
        font_rows[2943] = 32'h00000000;
        font_rows[2944] = 32'h00000000;
        font_rows[2945] = 32'h00000000;
        font_rows[2946] = 32'h00000000;
        font_rows[2947] = 32'h00030000;
        font_rows[2948] = 32'h00038000;
        font_rows[2949] = 32'h00070000;
        font_rows[2950] = 32'h00070000;
        font_rows[2951] = 32'h1ffffff8;
        font_rows[2952] = 32'h1ffffff8;
        font_rows[2953] = 32'h001c0000;
        font_rows[2954] = 32'h001c0000;
        font_rows[2955] = 32'h00380000;
        font_rows[2956] = 32'h00300000;
        font_rows[2957] = 32'h00700000;
        font_rows[2958] = 32'h00e00000;
        font_rows[2959] = 32'h01ffffe0;
        font_rows[2960] = 32'h01ffffe0;
        font_rows[2961] = 32'h03e00060;
        font_rows[2962] = 32'h07e00060;
        font_rows[2963] = 32'h0fe00060;
        font_rows[2964] = 32'h1ee00060;
        font_rows[2965] = 32'h1ce00060;
        font_rows[2966] = 32'h08e00060;
        font_rows[2967] = 32'h00e00060;
        font_rows[2968] = 32'h00e00060;
        font_rows[2969] = 32'h00ffffe0;
        font_rows[2970] = 32'h00ffffe0;
        font_rows[2971] = 32'h00e00060;
        font_rows[2972] = 32'h00e00060;
        font_rows[2973] = 32'h00000000;
        font_rows[2974] = 32'h00000000;
        font_rows[2975] = 32'h00000000;
        font_rows[2976] = 32'h00000000;
        font_rows[2977] = 32'h00000000;
        font_rows[2978] = 32'h00000000;
        font_rows[2979] = 32'h00000000;
        font_rows[2980] = 32'h1ffffff8;
        font_rows[2981] = 32'h1ffffff8;
        font_rows[2982] = 32'h00070000;
        font_rows[2983] = 32'h00070000;
        font_rows[2984] = 32'h00070000;
        font_rows[2985] = 32'h00070000;
        font_rows[2986] = 32'h00076000;
        font_rows[2987] = 32'h0007f000;
        font_rows[2988] = 32'h00077c00;
        font_rows[2989] = 32'h00073e00;
        font_rows[2990] = 32'h00070f00;
        font_rows[2991] = 32'h000707c0;
        font_rows[2992] = 32'h000701e0;
        font_rows[2993] = 32'h000700f0;
        font_rows[2994] = 32'h00070060;
        font_rows[2995] = 32'h00070000;
        font_rows[2996] = 32'h00070000;
        font_rows[2997] = 32'h00070000;
        font_rows[2998] = 32'h00070000;
        font_rows[2999] = 32'h00070000;
        font_rows[3000] = 32'h00070000;
        font_rows[3001] = 32'h00070000;
        font_rows[3002] = 32'h00070000;
        font_rows[3003] = 32'h00000000;
        font_rows[3004] = 32'h00000000;
        font_rows[3005] = 32'h00000000;
        font_rows[3006] = 32'h00000000;
        font_rows[3007] = 32'h00000000;
        font_rows[3008] = 32'h00000000;
        font_rows[3009] = 32'h00000000;
        font_rows[3010] = 32'h1ffbfffc;
        font_rows[3011] = 32'h1ffbfffc;
        font_rows[3012] = 32'h00380e00;
        font_rows[3013] = 32'h00700c00;
        font_rows[3014] = 32'h04e00c00;
        font_rows[3015] = 32'h07c00c00;
        font_rows[3016] = 32'h07c1fff0;
        font_rows[3017] = 32'h0381fff0;
        font_rows[3018] = 32'h01818030;
        font_rows[3019] = 32'h3ffd8e30;
        font_rows[3020] = 32'h3ffd8e30;
        font_rows[3021] = 32'h019d8e30;
        font_rows[3022] = 32'h01998e30;
        font_rows[3023] = 32'h01998e30;
        font_rows[3024] = 32'h01998e30;
        font_rows[3025] = 32'h01998e30;
        font_rows[3026] = 32'h01818c30;
        font_rows[3027] = 32'h01818c30;
        font_rows[3028] = 32'h01819c30;
        font_rows[3029] = 32'h01801e00;
        font_rows[3030] = 32'h01803f80;
        font_rows[3031] = 32'h018073c0;
        font_rows[3032] = 32'h0181e0e0;
        font_rows[3033] = 32'h0383c078;
        font_rows[3034] = 32'h1f8f0038;
        font_rows[3035] = 32'h1f060018;
        font_rows[3036] = 32'h00000000;
        font_rows[3037] = 32'h00000000;
        font_rows[3038] = 32'h00000000;
        font_rows[3039] = 32'h00000000;
        font_rows[3040] = 32'h00000000;
        font_rows[3041] = 32'h00000000;
        font_rows[3042] = 32'h00383000;
        font_rows[3043] = 32'h0c387000;
        font_rows[3044] = 32'h0c386000;
        font_rows[3045] = 32'h0c38fff8;
        font_rows[3046] = 32'h0c38fff8;
        font_rows[3047] = 32'h0c39c000;
        font_rows[3048] = 32'h0c399800;
        font_rows[3049] = 32'h0c3bbe00;
        font_rows[3050] = 32'h0c3f0f80;
        font_rows[3051] = 32'h0c3a01e0;
        font_rows[3052] = 32'h0c380060;
        font_rows[3053] = 32'h00000000;
        font_rows[3054] = 32'h03ffff80;
        font_rows[3055] = 32'h03ffff80;
        font_rows[3056] = 32'h03000180;
        font_rows[3057] = 32'h03038180;
        font_rows[3058] = 32'h03038180;
        font_rows[3059] = 32'h03038180;
        font_rows[3060] = 32'h0303c180;
        font_rows[3061] = 32'h0307c198;
        font_rows[3062] = 32'h0307c19c;
        font_rows[3063] = 32'h000ec018;
        font_rows[3064] = 32'h003cc018;
        font_rows[3065] = 32'h00f8e038;
        font_rows[3066] = 32'h07e0fff8;
        font_rows[3067] = 32'h1f807fe0;
        font_rows[3068] = 32'h1c000000;
        font_rows[3069] = 32'h00000000;
        font_rows[3070] = 32'h00000000;
        font_rows[3071] = 32'h00000000;
        font_rows[3072] = 32'h00000000;
        font_rows[3073] = 32'h00000000;
        font_rows[3074] = 32'h00000000;
        font_rows[3075] = 32'h00000000;
        font_rows[3076] = 32'h000fffe0;
        font_rows[3077] = 32'h1fcfffe0;
        font_rows[3078] = 32'h1fcc0060;
        font_rows[3079] = 32'h18cfffe0;
        font_rows[3080] = 32'h18cfffe0;
        font_rows[3081] = 32'h18cc0060;
        font_rows[3082] = 32'h18cfffe0;
        font_rows[3083] = 32'h18cfffe0;
        font_rows[3084] = 32'h18c0c300;
        font_rows[3085] = 32'h1fc0c300;
        font_rows[3086] = 32'h1fcffff0;
        font_rows[3087] = 32'h18cffff0;
        font_rows[3088] = 32'h18dffff8;
        font_rows[3089] = 32'h18dffff8;
        font_rows[3090] = 32'h18c18180;
        font_rows[3091] = 32'h18c718e0;
        font_rows[3092] = 32'h18df99f8;
        font_rows[3093] = 32'h18f9dbb8;
        font_rows[3094] = 32'h1fd0db08;
        font_rows[3095] = 32'h1fc0db80;
        font_rows[3096] = 32'h180fd9f0;
        font_rows[3097] = 32'h181f1878;
        font_rows[3098] = 32'h001cf818;
        font_rows[3099] = 32'h0000f000;
        font_rows[3100] = 32'h00000000;
        font_rows[3101] = 32'h00000000;
        font_rows[3102] = 32'h00000000;
        font_rows[3103] = 32'h00000000;
        font_rows[3104] = 32'h00000000;
        font_rows[3105] = 32'h00000000;
        font_rows[3106] = 32'h00000000;
        font_rows[3107] = 32'h00000000;
        font_rows[3108] = 32'h03ffffc0;
        font_rows[3109] = 32'h03ffffc0;
        font_rows[3110] = 32'h030000c0;
        font_rows[3111] = 32'h030000c0;
        font_rows[3112] = 32'h03ffffc0;
        font_rows[3113] = 32'h03ffffc0;
        font_rows[3114] = 32'h030000c0;
        font_rows[3115] = 32'h030000c0;
        font_rows[3116] = 32'h030000c0;
        font_rows[3117] = 32'h03ffffc0;
        font_rows[3118] = 32'h03ffffc0;
        font_rows[3119] = 32'h030000c0;
        font_rows[3120] = 32'h000c3000;
        font_rows[3121] = 32'h040c3060;
        font_rows[3122] = 32'h0e0c3070;
        font_rows[3123] = 32'h070c30e0;
        font_rows[3124] = 32'h038c31c0;
        font_rows[3125] = 32'h01cc3380;
        font_rows[3126] = 32'h008c3100;
        font_rows[3127] = 32'h000c3000;
        font_rows[3128] = 32'h000c3000;
        font_rows[3129] = 32'h1ffffff8;
        font_rows[3130] = 32'h1ffffff8;
        font_rows[3131] = 32'h00000000;
        font_rows[3132] = 32'h00000000;
        font_rows[3133] = 32'h00000000;
        font_rows[3134] = 32'h00000000;
        font_rows[3135] = 32'h00000000;
        font_rows[3136] = 32'h00000000;
        font_rows[3137] = 32'h00000000;
        font_rows[3138] = 32'h00000000;
        font_rows[3139] = 32'h07ffffe0;
        font_rows[3140] = 32'h07ffffe0;
        font_rows[3141] = 32'h00000000;
        font_rows[3142] = 32'h00000000;
        font_rows[3143] = 32'h00000000;
        font_rows[3144] = 32'h00000000;
        font_rows[3145] = 32'h00000000;
        font_rows[3146] = 32'h00000000;
        font_rows[3147] = 32'h3ffffffc;
        font_rows[3148] = 32'h3ffffffc;
        font_rows[3149] = 32'h0001c000;
        font_rows[3150] = 32'h0001c000;
        font_rows[3151] = 32'h00c1c300;
        font_rows[3152] = 32'h01c1c700;
        font_rows[3153] = 32'h01c1c380;
        font_rows[3154] = 32'h0381c1c0;
        font_rows[3155] = 32'h0301c0e0;
        font_rows[3156] = 32'h0701c0e0;
        font_rows[3157] = 32'h0e01c070;
        font_rows[3158] = 32'h1c01c038;
        font_rows[3159] = 32'h1c01c03c;
        font_rows[3160] = 32'h1801c018;
        font_rows[3161] = 32'h003fc000;
        font_rows[3162] = 32'h003f8000;
        font_rows[3163] = 32'h00000000;
        font_rows[3164] = 32'h00000000;
        font_rows[3165] = 32'h00000000;
        font_rows[3166] = 32'h00000000;
        font_rows[3167] = 32'h00000000;
        font_rows[3168] = 32'h00000000;
        font_rows[3169] = 32'h00000000;
        font_rows[3170] = 32'h00000000;
        font_rows[3171] = 32'h00030000;
        font_rows[3172] = 32'h00038000;
        font_rows[3173] = 32'h00018000;
        font_rows[3174] = 32'h1ffffff8;
        font_rows[3175] = 32'h1ffffff8;
        font_rows[3176] = 32'h00000000;
        font_rows[3177] = 32'h01ffff80;
        font_rows[3178] = 32'h01ffff80;
        font_rows[3179] = 32'h01800180;
        font_rows[3180] = 32'h01800180;
        font_rows[3181] = 32'h01ffff80;
        font_rows[3182] = 32'h01ffff80;
        font_rows[3183] = 32'h01800180;
        font_rows[3184] = 32'h00000000;
        font_rows[3185] = 32'h1ffffff8;
        font_rows[3186] = 32'h1ffffff8;
        font_rows[3187] = 32'h1c000038;
        font_rows[3188] = 32'h1c3ffc38;
        font_rows[3189] = 32'h003ffc00;
        font_rows[3190] = 32'h00301c18;
        font_rows[3191] = 32'h00701c1c;
        font_rows[3192] = 32'h00701c1c;
        font_rows[3193] = 32'h00e01c18;
        font_rows[3194] = 32'h07c01c38;
        font_rows[3195] = 32'h1f800ff8;
        font_rows[3196] = 32'h1c0007f0;
        font_rows[3197] = 32'h00000000;
        font_rows[3198] = 32'h00000000;
        font_rows[3199] = 32'h00000000;
        font_rows[3200] = 32'h00000000;
        font_rows[3201] = 32'h00000000;
        font_rows[3202] = 32'h00000000;
        font_rows[3203] = 32'h03010080;
        font_rows[3204] = 32'h030381c0;
        font_rows[3205] = 32'h0301c380;
        font_rows[3206] = 32'h0300c300;
        font_rows[3207] = 32'h031ffff8;
        font_rows[3208] = 32'h031ffff8;
        font_rows[3209] = 32'h031c1838;
        font_rows[3210] = 32'h3ffd99b8;
        font_rows[3211] = 32'h1ffddbb8;
        font_rows[3212] = 32'h031c9b38;
        font_rows[3213] = 32'h031c1838;
        font_rows[3214] = 32'h031ffff8;
        font_rows[3215] = 32'h031ffff8;
        font_rows[3216] = 32'h03000000;
        font_rows[3217] = 32'h03000000;
        font_rows[3218] = 32'h0307ffe0;
        font_rows[3219] = 32'h0307ffe0;
        font_rows[3220] = 32'h03060060;
        font_rows[3221] = 32'h03660060;
        font_rows[3222] = 32'h03e7ffe0;
        font_rows[3223] = 32'h1fe7ffe0;
        font_rows[3224] = 32'h3f060060;
        font_rows[3225] = 32'h18060060;
        font_rows[3226] = 32'h0007ffe0;
        font_rows[3227] = 32'h0007ffe0;
        font_rows[3228] = 32'h00060060;
        font_rows[3229] = 32'h00000000;
        font_rows[3230] = 32'h00000000;
        font_rows[3231] = 32'h00000000;
        font_rows[3232] = 32'h00000000;
        font_rows[3233] = 32'h00000000;
        font_rows[3234] = 32'h00000000;
        font_rows[3235] = 32'h00c00300;
        font_rows[3236] = 32'h00e00780;
        font_rows[3237] = 32'h00700f00;
        font_rows[3238] = 32'h00381e00;
        font_rows[3239] = 32'h00103c00;
        font_rows[3240] = 32'h1ffffff8;
        font_rows[3241] = 32'h1ffffff8;
        font_rows[3242] = 32'h00000000;
        font_rows[3243] = 32'h00181800;
        font_rows[3244] = 32'h00781e00;
        font_rows[3245] = 32'h00f80f80;
        font_rows[3246] = 32'h03e003e0;
        font_rows[3247] = 32'h0f8001f8;
        font_rows[3248] = 32'h1e000078;
        font_rows[3249] = 32'h08000010;
        font_rows[3250] = 32'h03ffffc0;
        font_rows[3251] = 32'h03ffffc0;
        font_rows[3252] = 32'h038c30c0;
        font_rows[3253] = 32'h038c30c0;
        font_rows[3254] = 32'h038c30c0;
        font_rows[3255] = 32'h038c30c0;
        font_rows[3256] = 32'h038c30c0;
        font_rows[3257] = 32'h038c30c0;
        font_rows[3258] = 32'h3ffffffc;
        font_rows[3259] = 32'h3ffffffc;
        font_rows[3260] = 32'h00000000;
        font_rows[3261] = 32'h00000000;
        font_rows[3262] = 32'h00000000;
        font_rows[3263] = 32'h00000000;
        font_rows[3264] = 32'h00000000;
        font_rows[3265] = 32'h04000c00;
        font_rows[3266] = 32'h0e000c00;
        font_rows[3267] = 32'h07060c30;
        font_rows[3268] = 32'h03060c30;
        font_rows[3269] = 32'h03860c30;
        font_rows[3270] = 32'h01060c30;
        font_rows[3271] = 32'h3ff60c30;
        font_rows[3272] = 32'h3ff7fff0;
        font_rows[3273] = 32'h0007fff0;
        font_rows[3274] = 32'h00000030;
        font_rows[3275] = 32'h30600000;
        font_rows[3276] = 32'h306ffffe;
        font_rows[3277] = 32'h18cffffe;
        font_rows[3278] = 32'h18c01c00;
        font_rows[3279] = 32'h18c01800;
        font_rows[3280] = 32'h18c01800;
        font_rows[3281] = 32'h18cffff8;
        font_rows[3282] = 32'h1ccffff8;
        font_rows[3283] = 32'h0d8c6318;
        font_rows[3284] = 32'h0d8c6318;
        font_rows[3285] = 32'h0d8c6318;
        font_rows[3286] = 32'h018c6318;
        font_rows[3287] = 32'h01ac6318;
        font_rows[3288] = 32'h0ffc6318;
        font_rows[3289] = 32'h3fcc6318;
        font_rows[3290] = 32'h380c6318;
        font_rows[3291] = 32'h000c03f8;
        font_rows[3292] = 32'h000c0078;
        font_rows[3293] = 32'h00000000;
        font_rows[3294] = 32'h00000000;
        font_rows[3295] = 32'h00000000;
        font_rows[3296] = 32'h00000000;
        font_rows[3297] = 32'h00000018;
        font_rows[3298] = 32'h00000018;
        font_rows[3299] = 32'h3fffe018;
        font_rows[3300] = 32'h3fffe618;
        font_rows[3301] = 32'h01c00618;
        font_rows[3302] = 32'h01800618;
        font_rows[3303] = 32'h03860618;
        font_rows[3304] = 32'h03060618;
        font_rows[3305] = 32'h06070618;
        font_rows[3306] = 32'h0e038618;
        font_rows[3307] = 32'h1fff8618;
        font_rows[3308] = 32'h1fffc618;
        font_rows[3309] = 32'h1f80e618;
        font_rows[3310] = 32'h00008618;
        font_rows[3311] = 32'h00700618;
        font_rows[3312] = 32'h00700618;
        font_rows[3313] = 32'h00700618;
        font_rows[3314] = 32'h3fffc618;
        font_rows[3315] = 32'h3fffc618;
        font_rows[3316] = 32'h00700618;
        font_rows[3317] = 32'h00700618;
        font_rows[3318] = 32'h00700618;
        font_rows[3319] = 32'h00700018;
        font_rows[3320] = 32'h0070e018;
        font_rows[3321] = 32'h007fe018;
        font_rows[3322] = 32'h3fff8038;
        font_rows[3323] = 32'h3fe007f8;
        font_rows[3324] = 32'h380007f0;
        font_rows[3325] = 32'h00000300;
        font_rows[3326] = 32'h00000000;
        font_rows[3327] = 32'h00000000;
        font_rows[3328] = 32'h00000000;
        font_rows[3329] = 32'h00000000;
        font_rows[3330] = 32'h000001f8;
        font_rows[3331] = 32'h3fe7fff8;
        font_rows[3332] = 32'h3fe7ffc0;
        font_rows[3333] = 32'h00e00e00;
        font_rows[3334] = 32'h01c00e00;
        font_rows[3335] = 32'h03800e00;
        font_rows[3336] = 32'h07000e00;
        font_rows[3337] = 32'h07030e00;
        font_rows[3338] = 32'h0e030e00;
        font_rows[3339] = 32'h1c030ff8;
        font_rows[3340] = 32'h1fe30ff8;
        font_rows[3341] = 32'h1fe30e00;
        font_rows[3342] = 32'h00630e00;
        font_rows[3343] = 32'h00630e00;
        font_rows[3344] = 32'h30630e00;
        font_rows[3345] = 32'h30e30e00;
        font_rows[3346] = 32'h18c30e00;
        font_rows[3347] = 32'h18c30e00;
        font_rows[3348] = 32'h1dc3fffc;
        font_rows[3349] = 32'h0f83fffc;
        font_rows[3350] = 32'h07800000;
        font_rows[3351] = 32'h07800000;
        font_rows[3352] = 32'h07e00000;
        font_rows[3353] = 32'h0efc0000;
        font_rows[3354] = 32'h1c3ffffc;
        font_rows[3355] = 32'h3807fffc;
        font_rows[3356] = 32'h10000000;
        font_rows[3357] = 32'h00000000;
        font_rows[3358] = 32'h00000000;
        font_rows[3359] = 32'h00000000;
        font_rows[3360] = 32'h00000000;
        font_rows[3361] = 32'h00000000;
        font_rows[3362] = 32'h08000000;
        font_rows[3363] = 32'h1c000000;
        font_rows[3364] = 32'h0e07fff0;
        font_rows[3365] = 32'h0707fff0;
        font_rows[3366] = 32'h07060070;
        font_rows[3367] = 32'h03860070;
        font_rows[3368] = 32'h01060070;
        font_rows[3369] = 32'h00060070;
        font_rows[3370] = 32'h00060070;
        font_rows[3371] = 32'h00060070;
        font_rows[3372] = 32'h3f060070;
        font_rows[3373] = 32'h3f07fff0;
        font_rows[3374] = 32'h0707fff0;
        font_rows[3375] = 32'h07060070;
        font_rows[3376] = 32'h07061070;
        font_rows[3377] = 32'h070e3800;
        font_rows[3378] = 32'h070e1c00;
        font_rows[3379] = 32'h070e0f00;
        font_rows[3380] = 32'h070c0780;
        font_rows[3381] = 32'h071c01c0;
        font_rows[3382] = 32'h071c00f0;
        font_rows[3383] = 32'h07380078;
        font_rows[3384] = 32'h07300038;
        font_rows[3385] = 32'h0f800010;
        font_rows[3386] = 32'h1dc00000;
        font_rows[3387] = 32'h38f80000;
        font_rows[3388] = 32'h307ffffc;
        font_rows[3389] = 32'h001ffffc;
        font_rows[3390] = 32'h00000000;
        font_rows[3391] = 32'h00000000;
        font_rows[3392] = 32'h00000000;
        font_rows[3393] = 32'h00000000;
        font_rows[3394] = 32'h00000000;
        font_rows[3395] = 32'h00000000;
        font_rows[3396] = 32'h00000000;
        font_rows[3397] = 32'h00000000;
        font_rows[3398] = 32'h00000000;
        font_rows[3399] = 32'h00000000;
        font_rows[3400] = 32'h073e0f80;
        font_rows[3401] = 32'h077f3fc0;
        font_rows[3402] = 32'h07e7f3e0;
        font_rows[3403] = 32'h0783e0e0;
        font_rows[3404] = 32'h0781c0e0;
        font_rows[3405] = 32'h0701c0e0;
        font_rows[3406] = 32'h0701c0e0;
        font_rows[3407] = 32'h0701c0e0;
        font_rows[3408] = 32'h0701c0e0;
        font_rows[3409] = 32'h0701c0e0;
        font_rows[3410] = 32'h0701c0e0;
        font_rows[3411] = 32'h0701c0e0;
        font_rows[3412] = 32'h0701c0e0;
        font_rows[3413] = 32'h0701c0e0;
        font_rows[3414] = 32'h0701c0e0;
        font_rows[3415] = 32'h00000000;
        font_rows[3416] = 32'h00000000;
        font_rows[3417] = 32'h00000000;
        font_rows[3418] = 32'h00000000;
        font_rows[3419] = 32'h00000000;
        font_rows[3420] = 32'h00000000;
        font_rows[3421] = 32'h00000000;
        font_rows[3422] = 32'h00000000;
        font_rows[3423] = 32'h00000000;
        font_rows[3424] = 32'h00000000;
        font_rows[3425] = 32'h00000000;
        font_rows[3426] = 32'h00000000;
        font_rows[3427] = 32'h00000000;
        font_rows[3428] = 32'h00000000;
        font_rows[3429] = 32'h00000000;
        font_rows[3430] = 32'h00000000;
        font_rows[3431] = 32'h00000000;
        font_rows[3432] = 32'h0007f000;
        font_rows[3433] = 32'h000ff000;
        font_rows[3434] = 32'h001e3000;
        font_rows[3435] = 32'h001c0000;
        font_rows[3436] = 32'h003c0000;
        font_rows[3437] = 32'h001c0000;
        font_rows[3438] = 32'h001f8000;
        font_rows[3439] = 32'h000fe000;
        font_rows[3440] = 32'h0003f000;
        font_rows[3441] = 32'h00007800;
        font_rows[3442] = 32'h00003800;
        font_rows[3443] = 32'h00003800;
        font_rows[3444] = 32'h00387000;
        font_rows[3445] = 32'h003ff000;
        font_rows[3446] = 32'h000fc000;
        font_rows[3447] = 32'h00000000;
        font_rows[3448] = 32'h00000000;
        font_rows[3449] = 32'h00000000;
        font_rows[3450] = 32'h00000000;
        font_rows[3451] = 32'h00000000;
        font_rows[3452] = 32'h00000000;
        font_rows[3453] = 32'h00000000;
        font_rows[3454] = 32'h00000000;
        font_rows[3455] = 32'h00000000;
        font_rows[3456] = 32'h00000000;
        font_rows[3457] = 32'h00000000;
        font_rows[3458] = 32'h00000000;
        font_rows[3459] = 32'h0380c000;
        font_rows[3460] = 32'h0380e000;
        font_rows[3461] = 32'h0380c000;
        font_rows[3462] = 32'h0381c000;
        font_rows[3463] = 32'h0381fff0;
        font_rows[3464] = 32'h0383fff0;
        font_rows[3465] = 32'h03838030;
        font_rows[3466] = 32'h1ff30030;
        font_rows[3467] = 32'h1ff70030;
        font_rows[3468] = 32'h03864030;
        font_rows[3469] = 32'h038ee030;
        font_rows[3470] = 32'h03847830;
        font_rows[3471] = 32'h03803c30;
        font_rows[3472] = 32'h03801830;
        font_rows[3473] = 32'h038001b0;
        font_rows[3474] = 32'h038007b0;
        font_rows[3475] = 32'h03803f30;
        font_rows[3476] = 32'h03b0fc30;
        font_rows[3477] = 32'h03f3f030;
        font_rows[3478] = 32'h1ff78030;
        font_rows[3479] = 32'h3f820070;
        font_rows[3480] = 32'h18000070;
        font_rows[3481] = 32'h000000f0;
        font_rows[3482] = 32'h00003fe0;
        font_rows[3483] = 32'h00003fc0;
        font_rows[3484] = 32'h00000000;
        font_rows[3485] = 32'h00000000;
        font_rows[3486] = 32'h00000000;
        font_rows[3487] = 32'h00000000;
        font_rows[3488] = 32'h00000000;
        font_rows[3489] = 32'h00000000;
        font_rows[3490] = 32'h01840000;
        font_rows[3491] = 32'h018c0000;
        font_rows[3492] = 32'h039ff3f8;
        font_rows[3493] = 32'h071ff3f8;
        font_rows[3494] = 32'h06386000;
        font_rows[3495] = 32'h0c70e000;
        font_rows[3496] = 32'h1cfff800;
        font_rows[3497] = 32'h39fff800;
        font_rows[3498] = 32'h11b19800;
        font_rows[3499] = 32'h03b19800;
        font_rows[3500] = 32'h033ffffc;
        font_rows[3501] = 32'h063ffffc;
        font_rows[3502] = 32'h06319860;
        font_rows[3503] = 32'h0e319860;
        font_rows[3504] = 32'h1e3ff860;
        font_rows[3505] = 32'h3e3ff860;
        font_rows[3506] = 32'h36339860;
        font_rows[3507] = 32'h16038060;
        font_rows[3508] = 32'h067ffc60;
        font_rows[3509] = 32'h067ff860;
        font_rows[3510] = 32'h06038060;
        font_rows[3511] = 32'h0607c060;
        font_rows[3512] = 32'h060ef060;
        font_rows[3513] = 32'h063c7860;
        font_rows[3514] = 32'h067010e0;
        font_rows[3515] = 32'h066007c0;
        font_rows[3516] = 32'h060007c0;
        font_rows[3517] = 32'h00000000;
        font_rows[3518] = 32'h00000000;
        font_rows[3519] = 32'h00000000;
        font_rows[3520] = 32'h00000000;
        font_rows[3521] = 32'h00000000;
        font_rows[3522] = 32'h0300c000;
        font_rows[3523] = 32'h0300c000;
        font_rows[3524] = 32'h0301c000;
        font_rows[3525] = 32'h03018000;
        font_rows[3526] = 32'h0303fff8;
        font_rows[3527] = 32'h0307fff8;
        font_rows[3528] = 32'h3ff70038;
        font_rows[3529] = 32'h3ffe0038;
        font_rows[3530] = 32'h030c0038;
        font_rows[3531] = 32'h030c2038;
        font_rows[3532] = 32'h07c07038;
        font_rows[3533] = 32'h07e06038;
        font_rows[3534] = 32'h0770e830;
        font_rows[3535] = 32'h0f39dc30;
        font_rows[3536] = 32'h0f118c30;
        font_rows[3537] = 32'h1b038e30;
        font_rows[3538] = 32'h3b030630;
        font_rows[3539] = 32'h13070730;
        font_rows[3540] = 32'h130fff30;
        font_rows[3541] = 32'h0307ffb0;
        font_rows[3542] = 32'h03000130;
        font_rows[3543] = 32'h03000030;
        font_rows[3544] = 32'h03000070;
        font_rows[3545] = 32'h030000f0;
        font_rows[3546] = 32'h03003fe0;
        font_rows[3547] = 32'h03003fc0;
        font_rows[3548] = 32'h03000000;
        font_rows[3549] = 32'h00000000;
        font_rows[3550] = 32'h00000000;
        font_rows[3551] = 32'h00000000;
        font_rows[3552] = 32'h00000000;
        font_rows[3553] = 32'h00000000;
        font_rows[3554] = 32'h00000000;
        font_rows[3555] = 32'h01800c80;
        font_rows[3556] = 32'h01800de0;
        font_rows[3557] = 32'h03800c70;
        font_rows[3558] = 32'h03000c20;
        font_rows[3559] = 32'h1ffbfffc;
        font_rows[3560] = 32'h1ffbfff8;
        font_rows[3561] = 32'h06000c00;
        font_rows[3562] = 32'h06c00c00;
        font_rows[3563] = 32'h06c3fff0;
        font_rows[3564] = 32'h0cc3fff0;
        font_rows[3565] = 32'h0cc30c30;
        font_rows[3566] = 32'h1cc30c30;
        font_rows[3567] = 32'h18c30c30;
        font_rows[3568] = 32'h1ffbfff0;
        font_rows[3569] = 32'h1ffbfff0;
        font_rows[3570] = 32'h00c30c30;
        font_rows[3571] = 32'h00c30c30;
        font_rows[3572] = 32'h00c30c30;
        font_rows[3573] = 32'h01fbfff0;
        font_rows[3574] = 32'h3ffbfff0;
        font_rows[3575] = 32'h3fc30c30;
        font_rows[3576] = 32'h00c30c30;
        font_rows[3577] = 32'h00c30c30;
        font_rows[3578] = 32'h00c30c30;
        font_rows[3579] = 32'h00c30df0;
        font_rows[3580] = 32'h00c30de0;
        font_rows[3581] = 32'h00000000;
        font_rows[3582] = 32'h00000000;
        font_rows[3583] = 32'h00000000;
        font_rows[3584] = 32'h00000000;
        font_rows[3585] = 32'h00000000;
        font_rows[3586] = 32'h00000000;
        font_rows[3587] = 32'h00001800;
        font_rows[3588] = 32'h00001800;
        font_rows[3589] = 32'h07fc1800;
        font_rows[3590] = 32'h07fc1800;
        font_rows[3591] = 32'h060c1800;
        font_rows[3592] = 32'h060c1800;
        font_rows[3593] = 32'h060cfff0;
        font_rows[3594] = 32'h07fcfff0;
        font_rows[3595] = 32'h07fc1830;
        font_rows[3596] = 32'h060c1830;
        font_rows[3597] = 32'h060c1830;
        font_rows[3598] = 32'h060c1830;
        font_rows[3599] = 32'h060c1830;
        font_rows[3600] = 32'h07fc1830;
        font_rows[3601] = 32'h07fc3830;
        font_rows[3602] = 32'h060c3830;
        font_rows[3603] = 32'h060c3870;
        font_rows[3604] = 32'h060c3070;
        font_rows[3605] = 32'h060c3070;
        font_rows[3606] = 32'h063f7070;
        font_rows[3607] = 32'h0fffe070;
        font_rows[3608] = 32'h3ff1e070;
        font_rows[3609] = 32'h1c01c060;
        font_rows[3610] = 32'h00038fe0;
        font_rows[3611] = 32'h00030fc0;
        font_rows[3612] = 32'h00020000;
        font_rows[3613] = 32'h00000000;
        font_rows[3614] = 32'h00000000;
        font_rows[3615] = 32'h00000000;
    end
    function [7:0] label_glyph;
        input integer label_id, column;
        begin
            label_glyph=0;
            case(label_id)
                0:case(column)
                    0:label_glyph=14;
                    1:label_glyph=15;
                    2:label_glyph=16;
                    3:label_glyph=17;
                    4:label_glyph=18;
                    5:label_glyph=19;
                    6:label_glyph=20;
                    7:label_glyph=21;
                    default:label_glyph=0; endcase
                1:case(column)
                    0:label_glyph=2;
                    1:label_glyph=10;
                    2:label_glyph=3;
                    3:label_glyph=1;
                    4:label_glyph=22;
                    5:label_glyph=2;
                    6:label_glyph=1;
                    7:label_glyph=9;
                    8:label_glyph=1;
                    default:label_glyph=0; endcase
                2:case(column)
                    0:label_glyph=23;
                    1:label_glyph=24;
                    default:label_glyph=0; endcase
                3:case(column)
                    0:label_glyph=25;
                    1:label_glyph=26;
                    2:label_glyph=27;
                    3:label_glyph=28;
                    default:label_glyph=0; endcase
                4:case(column)
                    0:label_glyph=18;
                    1:label_glyph=19;
                    2:label_glyph=29;
                    3:label_glyph=30;
                    default:label_glyph=0; endcase
                6:case(column)
                    0:label_glyph=31;
                    1:label_glyph=32;
                    2:label_glyph=33;
                    3:label_glyph=34;
                    default:label_glyph=0; endcase
                7:case(column)
                    0:label_glyph=35;
                    default:label_glyph=0; endcase
                8:case(column)
                    0:label_glyph=36;
                    default:label_glyph=0; endcase
                9:case(column)
                    0:label_glyph=37;
                    1:label_glyph=38;
                    2:label_glyph=39;
                    3:label_glyph=40;
                    default:label_glyph=0; endcase
                10:case(column)
                    0:label_glyph=31;
                    1:label_glyph=32;
                    default:label_glyph=0; endcase
                11:case(column)
                    0:label_glyph=41;
                    1:label_glyph=42;
                    default:label_glyph=0; endcase
                12:case(column)
                    0:label_glyph=43;
                    1:label_glyph=44;
                    default:label_glyph=0; endcase
                13:case(column)
                    0:label_glyph=45;
                    1:label_glyph=46;
                    default:label_glyph=0; endcase
                14:case(column)
                    0:label_glyph=47;
                    1:label_glyph=48;
                    default:label_glyph=0; endcase
                15:case(column)
                    0:label_glyph=49;
                    1:label_glyph=50;
                    2:label_glyph=20;
                    3:label_glyph=21;
                    4:label_glyph=46;
                    5:label_glyph=51;
                    default:label_glyph=0; endcase
                16:case(column)
                    0:label_glyph=52;
                    1:label_glyph=3;
                    2:label_glyph=53;
                    3:label_glyph=54;
                    4:label_glyph=18;
                    5:label_glyph=19;
                    default:label_glyph=0; endcase
                17:case(column)
                    0:label_glyph=55;
                    1:label_glyph=56;
                    2:label_glyph=57;
                    3:label_glyph=58;
                    default:label_glyph=0; endcase
                20:case(column)
                    0:label_glyph=59;
                    1:label_glyph=60;
                    2:label_glyph=61;
                    3:label_glyph=62;
                    default:label_glyph=0; endcase
                25:case(column)
                    0:label_glyph=63;
                    1:label_glyph=64;
                    2:label_glyph=59;
                    3:label_glyph=60;
                    default:label_glyph=0; endcase
                26:case(column)
                    0:label_glyph=25;
                    1:label_glyph=26;
                    default:label_glyph=0; endcase
                27:case(column)
                    0:label_glyph=65;
                    1:label_glyph=66;
                    2:label_glyph=67;
                    3:label_glyph=68;
                    default:label_glyph=0; endcase
                28:case(column)
                    0:label_glyph=69;
                    1:label_glyph=70;
                    2:label_glyph=59;
                    3:label_glyph=60;
                    default:label_glyph=0; endcase
                21:case(column)
                    0:label_glyph=71;
                    1:label_glyph=72;
                    2:label_glyph=29;
                    3:label_glyph=30;
                    default:label_glyph=0; endcase
                22:case(column)
                    0:label_glyph=43;
                    1:label_glyph=44;
                    2:label_glyph=73;
                    3:label_glyph=30;
                    default:label_glyph=0; endcase
                23:case(column)
                    0:label_glyph=74;
                    1:label_glyph=75;
                    2:label_glyph=29;
                    3:label_glyph=30;
                    default:label_glyph=0; endcase
                24:case(column)
                    0:label_glyph=76;
                    1:label_glyph=77;
                    2:label_glyph=59;
                    3:label_glyph=60;
                    default:label_glyph=0; endcase
                31:case(column)
                    0:label_glyph=75;
                    1:label_glyph=78;
                    2:label_glyph=79;
                    3:label_glyph=80;
                    default:label_glyph=0; endcase
                32:case(column)
                    0:label_glyph=81;
                    1:label_glyph=82;
                    2:label_glyph=83;
                    3:label_glyph=84;
                    default:label_glyph=0; endcase
                33:case(column)
                    0:label_glyph=85;
                    1:label_glyph=71;
                    2:label_glyph=79;
                    3:label_glyph=80;
                    default:label_glyph=0; endcase
                34:case(column)
                    0:label_glyph=86;
                    1:label_glyph=87;
                    2:label_glyph=79;
                    3:label_glyph=80;
                    default:label_glyph=0; endcase
                35:case(column)
                    0:label_glyph=88;
                    1:label_glyph=89;
                    2:label_glyph=29;
                    3:label_glyph=30;
                    default:label_glyph=0; endcase
                40:case(column)
                    0:label_glyph=90;
                    1:label_glyph=91;
                    default:label_glyph=0; endcase
                41:case(column)
                    0:label_glyph=92;
                    1:label_glyph=91;
                    default:label_glyph=0; endcase
                42:case(column)
                    0:label_glyph=90;
                    1:label_glyph=93;
                    default:label_glyph=0; endcase
                43:case(column)
                    0:label_glyph=92;
                    1:label_glyph=93;
                    default:label_glyph=0; endcase
                44:case(column)
                    0:label_glyph=14;
                    1:label_glyph=15;
                    2:label_glyph=94;
                    3:label_glyph=95;
                    default:label_glyph=0; endcase
                45:case(column)
                    0:label_glyph=96;
                    1:label_glyph=81;
                    2:label_glyph=97;
                    3:label_glyph=98;
                    default:label_glyph=0; endcase
                46:case(column)
                    0:label_glyph=99;
                    1:label_glyph=30;
                    default:label_glyph=0; endcase
                48:case(column)
                    0:label_glyph=100;
                    1:label_glyph=101;
                    default:label_glyph=0; endcase
                50:case(column)
                    0:label_glyph=102;
                    1:label_glyph=103;
                    2:label_glyph=102;
                    3:label_glyph=104;
                    4:label_glyph=105;
                    default:label_glyph=0; endcase
                51:case(column)
                    0:label_glyph=106;
                    1:label_glyph=107;
                    default:label_glyph=0; endcase
                52:case(column)
                    0:label_glyph=108;
                    1:label_glyph=109;
                    2:label_glyph=29;
                    3:label_glyph=30;
                    default:label_glyph=0; endcase
                53:case(column)
                    0:label_glyph=65;
                    1:label_glyph=66;
                    2:label_glyph=90;
                    3:label_glyph=52;
                    4:label_glyph=55;
                    5:label_glyph=56;
                    default:label_glyph=0; endcase
                54:case(column)
                    0:label_glyph=110;
                    1:label_glyph=32;
                    2:label_glyph=111;
                    3:label_glyph=112;
                    default:label_glyph=0; endcase
                default:label_glyph=0;
            endcase
        end
    endfunction
    function integer label_length;
        input integer label_id;
        begin case(label_id)
            0:label_length=8;
            1:label_length=9;
            2:label_length=2;
            3:label_length=4;
            4:label_length=4;
            6:label_length=4;
            7:label_length=1;
            8:label_length=1;
            9:label_length=4;
            10:label_length=2;
            11:label_length=2;
            12:label_length=2;
            13:label_length=2;
            14:label_length=2;
            15:label_length=6;
            16:label_length=6;
            17:label_length=4;
            20:label_length=4;
            25:label_length=4;
            26:label_length=2;
            27:label_length=4;
            28:label_length=4;
            21:label_length=4;
            22:label_length=4;
            23:label_length=4;
            24:label_length=4;
            31:label_length=4;
            32:label_length=4;
            33:label_length=4;
            34:label_length=4;
            35:label_length=4;
            40:label_length=2;
            41:label_length=2;
            42:label_length=2;
            43:label_length=2;
            44:label_length=4;
            45:label_length=4;
            46:label_length=2;
            48:label_length=2;
            50:label_length=5;
            51:label_length=2;
            52:label_length=4;
            53:label_length=6;
            54:label_length=4;
            default:label_length=0; endcase end
    endfunction
    // AE_SHADOW_BEGIN
    // ★ 2026-11-01 自动曝光读数的 2px 黑色描边掩码（由 build_ui_font.py 生成）。
    //   字模生成器会重写本块，ram_style 属性必须写在生成内容里，别手工加。
    (* ram_style = "dram" *) reg [63:0] ae_label_shadow_words [0:63];
    (* ram_style = "dram" *) reg [15:0] ae_digit_shadow_words [0:319];
    initial begin
        ae_label_shadow_words[0] = 64'h0000000000000000;
        ae_label_shadow_words[1] = 64'h0000000000000000;
        ae_label_shadow_words[2] = 64'h000000000000c000;
        ae_label_shadow_words[3] = 64'h000300000001c000;
        ae_label_shadow_words[4] = 64'h000380000000e000;
        ae_label_shadow_words[5] = 64'h0001800007fffff8;
        ae_label_shadow_words[6] = 64'h1ffffff807fffff8;
        ae_label_shadow_words[7] = 64'h1ffffff806000000;
        ae_label_shadow_words[8] = 64'h00000000060c0300;
        ae_label_shadow_words[9] = 64'h01ffff80060c0300;
        ae_label_shadow_words[10] = 64'h01ffff8007fffff8;
        ae_label_shadow_words[11] = 64'h0180018007fffff8;
        ae_label_shadow_words[12] = 64'h01800180060c0300;
        ae_label_shadow_words[13] = 64'h01ffff80060c0300;
        ae_label_shadow_words[14] = 64'h01ffff80060fff00;
        ae_label_shadow_words[15] = 64'h01800180060fff00;
        ae_label_shadow_words[16] = 64'h00000000060c0300;
        ae_label_shadow_words[17] = 64'h1ffffff806000000;
        ae_label_shadow_words[18] = 64'h1ffffff8067fffc0;
        ae_label_shadow_words[19] = 64'h1c000038067fffc0;
        ae_label_shadow_words[20] = 64'h1c3ffc38061c0380;
        ae_label_shadow_words[21] = 64'h003ffc00060e0700;
        ae_label_shadow_words[22] = 64'h00301c180e079e00;
        ae_label_shadow_words[23] = 64'h00701c1c0c03fc00;
        ae_label_shadow_words[24] = 64'h00701c1c0c01f800;
        ae_label_shadow_words[25] = 64'h00e01c181c07fe00;
        ae_label_shadow_words[26] = 64'h07c01c38187f1fc0;
        ae_label_shadow_words[27] = 64'h1f800ff839fc03f8;
        ae_label_shadow_words[28] = 64'h1c0007f011c00078;
        ae_label_shadow_words[29] = 64'h0000000000000000;
        ae_label_shadow_words[30] = 64'h0000000000000000;
        ae_label_shadow_words[31] = 64'h0000000000000000;
        ae_label_shadow_words[32] = 64'h0000000000000000;
        ae_label_shadow_words[33] = 64'h0000000000000000;
        ae_label_shadow_words[34] = 64'h0000000000000000;
        ae_label_shadow_words[35] = 64'h0301008000c00300;
        ae_label_shadow_words[36] = 64'h030381c000e00780;
        ae_label_shadow_words[37] = 64'h0301c38000700f00;
        ae_label_shadow_words[38] = 64'h0300c30000381e00;
        ae_label_shadow_words[39] = 64'h031ffff800103c00;
        ae_label_shadow_words[40] = 64'h031ffff81ffffff8;
        ae_label_shadow_words[41] = 64'h031c18381ffffff8;
        ae_label_shadow_words[42] = 64'h3ffd99b800000000;
        ae_label_shadow_words[43] = 64'h1ffddbb800181800;
        ae_label_shadow_words[44] = 64'h031c9b3800781e00;
        ae_label_shadow_words[45] = 64'h031c183800f80f80;
        ae_label_shadow_words[46] = 64'h031ffff803e003e0;
        ae_label_shadow_words[47] = 64'h031ffff80f8001f8;
        ae_label_shadow_words[48] = 64'h030000001e000078;
        ae_label_shadow_words[49] = 64'h0300000008000010;
        ae_label_shadow_words[50] = 64'h0307ffe003ffffc0;
        ae_label_shadow_words[51] = 64'h0307ffe003ffffc0;
        ae_label_shadow_words[52] = 64'h03060060038c30c0;
        ae_label_shadow_words[53] = 64'h03660060038c30c0;
        ae_label_shadow_words[54] = 64'h03e7ffe0038c30c0;
        ae_label_shadow_words[55] = 64'h1fe7ffe0038c30c0;
        ae_label_shadow_words[56] = 64'h3f060060038c30c0;
        ae_label_shadow_words[57] = 64'h18060060038c30c0;
        ae_label_shadow_words[58] = 64'h0007ffe03ffffffc;
        ae_label_shadow_words[59] = 64'h0007ffe03ffffffc;
        ae_label_shadow_words[60] = 64'h0006006000000000;
        ae_label_shadow_words[61] = 64'h0000000000000000;
        ae_label_shadow_words[62] = 64'h0000000000000000;
        ae_label_shadow_words[63] = 64'h0000000000000000;
        ae_digit_shadow_words[0] = 16'h0000;
        ae_digit_shadow_words[1] = 16'h0000;
        ae_digit_shadow_words[2] = 16'h0000;
        ae_digit_shadow_words[3] = 16'h0000;
        ae_digit_shadow_words[4] = 16'h0000;
        ae_digit_shadow_words[5] = 16'h03e0;
        ae_digit_shadow_words[6] = 16'h0ff8;
        ae_digit_shadow_words[7] = 16'h1e7c;
        ae_digit_shadow_words[8] = 16'h1c1c;
        ae_digit_shadow_words[9] = 16'h381e;
        ae_digit_shadow_words[10] = 16'h380e;
        ae_digit_shadow_words[11] = 16'h380e;
        ae_digit_shadow_words[12] = 16'h700e;
        ae_digit_shadow_words[13] = 16'h700e;
        ae_digit_shadow_words[14] = 16'h700e;
        ae_digit_shadow_words[15] = 16'h700e;
        ae_digit_shadow_words[16] = 16'h700e;
        ae_digit_shadow_words[17] = 16'h700e;
        ae_digit_shadow_words[18] = 16'h700e;
        ae_digit_shadow_words[19] = 16'h700e;
        ae_digit_shadow_words[20] = 16'h380e;
        ae_digit_shadow_words[21] = 16'h381c;
        ae_digit_shadow_words[22] = 16'h3c1c;
        ae_digit_shadow_words[23] = 16'h1e78;
        ae_digit_shadow_words[24] = 16'h0ff0;
        ae_digit_shadow_words[25] = 16'h07e0;
        ae_digit_shadow_words[26] = 16'h0000;
        ae_digit_shadow_words[27] = 16'h0000;
        ae_digit_shadow_words[28] = 16'h0000;
        ae_digit_shadow_words[29] = 16'h0000;
        ae_digit_shadow_words[30] = 16'h0000;
        ae_digit_shadow_words[31] = 16'h0000;
        ae_digit_shadow_words[32] = 16'h0000;
        ae_digit_shadow_words[33] = 16'h0000;
        ae_digit_shadow_words[34] = 16'h0000;
        ae_digit_shadow_words[35] = 16'h0000;
        ae_digit_shadow_words[36] = 16'h0000;
        ae_digit_shadow_words[37] = 16'h00e0;
        ae_digit_shadow_words[38] = 16'h01e0;
        ae_digit_shadow_words[39] = 16'h0fe0;
        ae_digit_shadow_words[40] = 16'h1fe0;
        ae_digit_shadow_words[41] = 16'h1ce0;
        ae_digit_shadow_words[42] = 16'h10e0;
        ae_digit_shadow_words[43] = 16'h00e0;
        ae_digit_shadow_words[44] = 16'h00e0;
        ae_digit_shadow_words[45] = 16'h00e0;
        ae_digit_shadow_words[46] = 16'h00e0;
        ae_digit_shadow_words[47] = 16'h00e0;
        ae_digit_shadow_words[48] = 16'h00e0;
        ae_digit_shadow_words[49] = 16'h00e0;
        ae_digit_shadow_words[50] = 16'h00e0;
        ae_digit_shadow_words[51] = 16'h00e0;
        ae_digit_shadow_words[52] = 16'h00e0;
        ae_digit_shadow_words[53] = 16'h00e0;
        ae_digit_shadow_words[54] = 16'h00e0;
        ae_digit_shadow_words[55] = 16'h00e0;
        ae_digit_shadow_words[56] = 16'h00e0;
        ae_digit_shadow_words[57] = 16'h00e0;
        ae_digit_shadow_words[58] = 16'h0000;
        ae_digit_shadow_words[59] = 16'h0000;
        ae_digit_shadow_words[60] = 16'h0000;
        ae_digit_shadow_words[61] = 16'h0000;
        ae_digit_shadow_words[62] = 16'h0000;
        ae_digit_shadow_words[63] = 16'h0000;
        ae_digit_shadow_words[64] = 16'h0000;
        ae_digit_shadow_words[65] = 16'h0000;
        ae_digit_shadow_words[66] = 16'h0000;
        ae_digit_shadow_words[67] = 16'h0000;
        ae_digit_shadow_words[68] = 16'h0000;
        ae_digit_shadow_words[69] = 16'h07e0;
        ae_digit_shadow_words[70] = 16'h1ff8;
        ae_digit_shadow_words[71] = 16'h3e7c;
        ae_digit_shadow_words[72] = 16'h301c;
        ae_digit_shadow_words[73] = 16'h001c;
        ae_digit_shadow_words[74] = 16'h000c;
        ae_digit_shadow_words[75] = 16'h001c;
        ae_digit_shadow_words[76] = 16'h001c;
        ae_digit_shadow_words[77] = 16'h001c;
        ae_digit_shadow_words[78] = 16'h0038;
        ae_digit_shadow_words[79] = 16'h0078;
        ae_digit_shadow_words[80] = 16'h01f0;
        ae_digit_shadow_words[81] = 16'h03c0;
        ae_digit_shadow_words[82] = 16'h0780;
        ae_digit_shadow_words[83] = 16'h0e00;
        ae_digit_shadow_words[84] = 16'h1c00;
        ae_digit_shadow_words[85] = 16'h3800;
        ae_digit_shadow_words[86] = 16'h3800;
        ae_digit_shadow_words[87] = 16'h3ffc;
        ae_digit_shadow_words[88] = 16'h3ffe;
        ae_digit_shadow_words[89] = 16'h7ffe;
        ae_digit_shadow_words[90] = 16'h0000;
        ae_digit_shadow_words[91] = 16'h0000;
        ae_digit_shadow_words[92] = 16'h0000;
        ae_digit_shadow_words[93] = 16'h0000;
        ae_digit_shadow_words[94] = 16'h0000;
        ae_digit_shadow_words[95] = 16'h0000;
        ae_digit_shadow_words[96] = 16'h0000;
        ae_digit_shadow_words[97] = 16'h0000;
        ae_digit_shadow_words[98] = 16'h0000;
        ae_digit_shadow_words[99] = 16'h0000;
        ae_digit_shadow_words[100] = 16'h0000;
        ae_digit_shadow_words[101] = 16'h07e0;
        ae_digit_shadow_words[102] = 16'h1ff0;
        ae_digit_shadow_words[103] = 16'h1cf8;
        ae_digit_shadow_words[104] = 16'h103c;
        ae_digit_shadow_words[105] = 16'h001c;
        ae_digit_shadow_words[106] = 16'h001c;
        ae_digit_shadow_words[107] = 16'h001c;
        ae_digit_shadow_words[108] = 16'h0038;
        ae_digit_shadow_words[109] = 16'h0070;
        ae_digit_shadow_words[110] = 16'h0fe0;
        ae_digit_shadow_words[111] = 16'h0ff0;
        ae_digit_shadow_words[112] = 16'h00f8;
        ae_digit_shadow_words[113] = 16'h003c;
        ae_digit_shadow_words[114] = 16'h001c;
        ae_digit_shadow_words[115] = 16'h001c;
        ae_digit_shadow_words[116] = 16'h001c;
        ae_digit_shadow_words[117] = 16'h001c;
        ae_digit_shadow_words[118] = 16'h203c;
        ae_digit_shadow_words[119] = 16'h38f8;
        ae_digit_shadow_words[120] = 16'h3ff0;
        ae_digit_shadow_words[121] = 16'h1fc0;
        ae_digit_shadow_words[122] = 16'h0000;
        ae_digit_shadow_words[123] = 16'h0000;
        ae_digit_shadow_words[124] = 16'h0000;
        ae_digit_shadow_words[125] = 16'h0000;
        ae_digit_shadow_words[126] = 16'h0000;
        ae_digit_shadow_words[127] = 16'h0000;
        ae_digit_shadow_words[128] = 16'h0000;
        ae_digit_shadow_words[129] = 16'h0000;
        ae_digit_shadow_words[130] = 16'h0000;
        ae_digit_shadow_words[131] = 16'h0000;
        ae_digit_shadow_words[132] = 16'h0000;
        ae_digit_shadow_words[133] = 16'h0038;
        ae_digit_shadow_words[134] = 16'h0078;
        ae_digit_shadow_words[135] = 16'h00f8;
        ae_digit_shadow_words[136] = 16'h00f8;
        ae_digit_shadow_words[137] = 16'h01f8;
        ae_digit_shadow_words[138] = 16'h01b8;
        ae_digit_shadow_words[139] = 16'h03b8;
        ae_digit_shadow_words[140] = 16'h0738;
        ae_digit_shadow_words[141] = 16'h0638;
        ae_digit_shadow_words[142] = 16'h0e38;
        ae_digit_shadow_words[143] = 16'h1c38;
        ae_digit_shadow_words[144] = 16'h3838;
        ae_digit_shadow_words[145] = 16'h7838;
        ae_digit_shadow_words[146] = 16'h7038;
        ae_digit_shadow_words[147] = 16'hffff;
        ae_digit_shadow_words[148] = 16'hffff;
        ae_digit_shadow_words[149] = 16'h0038;
        ae_digit_shadow_words[150] = 16'h0038;
        ae_digit_shadow_words[151] = 16'h0038;
        ae_digit_shadow_words[152] = 16'h0038;
        ae_digit_shadow_words[153] = 16'h0038;
        ae_digit_shadow_words[154] = 16'h0000;
        ae_digit_shadow_words[155] = 16'h0000;
        ae_digit_shadow_words[156] = 16'h0000;
        ae_digit_shadow_words[157] = 16'h0000;
        ae_digit_shadow_words[158] = 16'h0000;
        ae_digit_shadow_words[159] = 16'h0000;
        ae_digit_shadow_words[160] = 16'h0000;
        ae_digit_shadow_words[161] = 16'h0000;
        ae_digit_shadow_words[162] = 16'h0000;
        ae_digit_shadow_words[163] = 16'h0000;
        ae_digit_shadow_words[164] = 16'h0000;
        ae_digit_shadow_words[165] = 16'h1ff8;
        ae_digit_shadow_words[166] = 16'h1ff8;
        ae_digit_shadow_words[167] = 16'h1c00;
        ae_digit_shadow_words[168] = 16'h1c00;
        ae_digit_shadow_words[169] = 16'h1c00;
        ae_digit_shadow_words[170] = 16'h1c00;
        ae_digit_shadow_words[171] = 16'h1c00;
        ae_digit_shadow_words[172] = 16'h1800;
        ae_digit_shadow_words[173] = 16'h1fe0;
        ae_digit_shadow_words[174] = 16'h1ff0;
        ae_digit_shadow_words[175] = 16'h11f8;
        ae_digit_shadow_words[176] = 16'h003c;
        ae_digit_shadow_words[177] = 16'h001c;
        ae_digit_shadow_words[178] = 16'h001c;
        ae_digit_shadow_words[179] = 16'h000e;
        ae_digit_shadow_words[180] = 16'h001c;
        ae_digit_shadow_words[181] = 16'h001c;
        ae_digit_shadow_words[182] = 16'h003c;
        ae_digit_shadow_words[183] = 16'h38f8;
        ae_digit_shadow_words[184] = 16'h3ff0;
        ae_digit_shadow_words[185] = 16'h1fc0;
        ae_digit_shadow_words[186] = 16'h0000;
        ae_digit_shadow_words[187] = 16'h0000;
        ae_digit_shadow_words[188] = 16'h0000;
        ae_digit_shadow_words[189] = 16'h0000;
        ae_digit_shadow_words[190] = 16'h0000;
        ae_digit_shadow_words[191] = 16'h0000;
        ae_digit_shadow_words[192] = 16'h0000;
        ae_digit_shadow_words[193] = 16'h0000;
        ae_digit_shadow_words[194] = 16'h0000;
        ae_digit_shadow_words[195] = 16'h0000;
        ae_digit_shadow_words[196] = 16'h0000;
        ae_digit_shadow_words[197] = 16'h01f8;
        ae_digit_shadow_words[198] = 16'h03fc;
        ae_digit_shadow_words[199] = 16'h078c;
        ae_digit_shadow_words[200] = 16'h0e00;
        ae_digit_shadow_words[201] = 16'h1c00;
        ae_digit_shadow_words[202] = 16'h1c00;
        ae_digit_shadow_words[203] = 16'h3800;
        ae_digit_shadow_words[204] = 16'h3800;
        ae_digit_shadow_words[205] = 16'h39f0;
        ae_digit_shadow_words[206] = 16'h3ff8;
        ae_digit_shadow_words[207] = 16'h3e3c;
        ae_digit_shadow_words[208] = 16'h3c1e;
        ae_digit_shadow_words[209] = 16'h380e;
        ae_digit_shadow_words[210] = 16'h380e;
        ae_digit_shadow_words[211] = 16'h380e;
        ae_digit_shadow_words[212] = 16'h380e;
        ae_digit_shadow_words[213] = 16'h380e;
        ae_digit_shadow_words[214] = 16'h1c1e;
        ae_digit_shadow_words[215] = 16'h1f3c;
        ae_digit_shadow_words[216] = 16'h0ff8;
        ae_digit_shadow_words[217] = 16'h03e0;
        ae_digit_shadow_words[218] = 16'h0000;
        ae_digit_shadow_words[219] = 16'h0000;
        ae_digit_shadow_words[220] = 16'h0000;
        ae_digit_shadow_words[221] = 16'h0000;
        ae_digit_shadow_words[222] = 16'h0000;
        ae_digit_shadow_words[223] = 16'h0000;
        ae_digit_shadow_words[224] = 16'h0000;
        ae_digit_shadow_words[225] = 16'h0000;
        ae_digit_shadow_words[226] = 16'h0000;
        ae_digit_shadow_words[227] = 16'h0000;
        ae_digit_shadow_words[228] = 16'h0000;
        ae_digit_shadow_words[229] = 16'h7ffe;
        ae_digit_shadow_words[230] = 16'h7ffe;
        ae_digit_shadow_words[231] = 16'h001e;
        ae_digit_shadow_words[232] = 16'h001c;
        ae_digit_shadow_words[233] = 16'h001c;
        ae_digit_shadow_words[234] = 16'h0038;
        ae_digit_shadow_words[235] = 16'h0030;
        ae_digit_shadow_words[236] = 16'h0070;
        ae_digit_shadow_words[237] = 16'h0070;
        ae_digit_shadow_words[238] = 16'h00e0;
        ae_digit_shadow_words[239] = 16'h00e0;
        ae_digit_shadow_words[240] = 16'h01c0;
        ae_digit_shadow_words[241] = 16'h01c0;
        ae_digit_shadow_words[242] = 16'h0380;
        ae_digit_shadow_words[243] = 16'h0380;
        ae_digit_shadow_words[244] = 16'h0380;
        ae_digit_shadow_words[245] = 16'h0700;
        ae_digit_shadow_words[246] = 16'h0700;
        ae_digit_shadow_words[247] = 16'h0700;
        ae_digit_shadow_words[248] = 16'h0700;
        ae_digit_shadow_words[249] = 16'h0700;
        ae_digit_shadow_words[250] = 16'h0000;
        ae_digit_shadow_words[251] = 16'h0000;
        ae_digit_shadow_words[252] = 16'h0000;
        ae_digit_shadow_words[253] = 16'h0000;
        ae_digit_shadow_words[254] = 16'h0000;
        ae_digit_shadow_words[255] = 16'h0000;
        ae_digit_shadow_words[256] = 16'h0000;
        ae_digit_shadow_words[257] = 16'h0000;
        ae_digit_shadow_words[258] = 16'h0000;
        ae_digit_shadow_words[259] = 16'h0000;
        ae_digit_shadow_words[260] = 16'h0000;
        ae_digit_shadow_words[261] = 16'h07e0;
        ae_digit_shadow_words[262] = 16'h0ff8;
        ae_digit_shadow_words[263] = 16'h1e78;
        ae_digit_shadow_words[264] = 16'h3c1c;
        ae_digit_shadow_words[265] = 16'h381c;
        ae_digit_shadow_words[266] = 16'h380c;
        ae_digit_shadow_words[267] = 16'h381c;
        ae_digit_shadow_words[268] = 16'h181c;
        ae_digit_shadow_words[269] = 16'h0e38;
        ae_digit_shadow_words[270] = 16'h07f0;
        ae_digit_shadow_words[271] = 16'h0ff8;
        ae_digit_shadow_words[272] = 16'h1e7c;
        ae_digit_shadow_words[273] = 16'h381e;
        ae_digit_shadow_words[274] = 16'h700e;
        ae_digit_shadow_words[275] = 16'h700e;
        ae_digit_shadow_words[276] = 16'h700e;
        ae_digit_shadow_words[277] = 16'h700e;
        ae_digit_shadow_words[278] = 16'h381e;
        ae_digit_shadow_words[279] = 16'h3e7c;
        ae_digit_shadow_words[280] = 16'h1ff8;
        ae_digit_shadow_words[281] = 16'h07e0;
        ae_digit_shadow_words[282] = 16'h0000;
        ae_digit_shadow_words[283] = 16'h0000;
        ae_digit_shadow_words[284] = 16'h0000;
        ae_digit_shadow_words[285] = 16'h0000;
        ae_digit_shadow_words[286] = 16'h0000;
        ae_digit_shadow_words[287] = 16'h0000;
        ae_digit_shadow_words[288] = 16'h0000;
        ae_digit_shadow_words[289] = 16'h0000;
        ae_digit_shadow_words[290] = 16'h0000;
        ae_digit_shadow_words[291] = 16'h0000;
        ae_digit_shadow_words[292] = 16'h0000;
        ae_digit_shadow_words[293] = 16'h07e0;
        ae_digit_shadow_words[294] = 16'h0ff0;
        ae_digit_shadow_words[295] = 16'h1e78;
        ae_digit_shadow_words[296] = 16'h381c;
        ae_digit_shadow_words[297] = 16'h381c;
        ae_digit_shadow_words[298] = 16'h700e;
        ae_digit_shadow_words[299] = 16'h700e;
        ae_digit_shadow_words[300] = 16'h700e;
        ae_digit_shadow_words[301] = 16'h780e;
        ae_digit_shadow_words[302] = 16'h381e;
        ae_digit_shadow_words[303] = 16'h3c3e;
        ae_digit_shadow_words[304] = 16'h1ffe;
        ae_digit_shadow_words[305] = 16'h0fee;
        ae_digit_shadow_words[306] = 16'h000e;
        ae_digit_shadow_words[307] = 16'h000c;
        ae_digit_shadow_words[308] = 16'h001c;
        ae_digit_shadow_words[309] = 16'h001c;
        ae_digit_shadow_words[310] = 16'h0038;
        ae_digit_shadow_words[311] = 16'h38f0;
        ae_digit_shadow_words[312] = 16'h3fe0;
        ae_digit_shadow_words[313] = 16'h1fc0;
        ae_digit_shadow_words[314] = 16'h0000;
        ae_digit_shadow_words[315] = 16'h0000;
        ae_digit_shadow_words[316] = 16'h0000;
        ae_digit_shadow_words[317] = 16'h0000;
        ae_digit_shadow_words[318] = 16'h0000;
        ae_digit_shadow_words[319] = 16'h0000;
    end
    // AE_SHADOW_END
    // FONT_GENERATED_END
    // SHADOW_GENERATED_BEGIN
    // ★ ERAM 吃紧：6Kb 的提示阴影放分布式 DRAM（TD 的属性名是 dram，
    //   不是 distributed）。注意：重新生成字模会重写本块，属性要一并保留。
    (* ram_style = "dram" *) reg [15:0] hint_shadow_words [0:383];
    initial begin
        hint_shadow_words[0] = 16'h0000;
        hint_shadow_words[1] = 16'h0000;
        hint_shadow_words[2] = 16'h0000;
        hint_shadow_words[3] = 16'h0000;
        hint_shadow_words[4] = 16'h0000;
        hint_shadow_words[5] = 16'h0000;
        hint_shadow_words[6] = 16'h0000;
        hint_shadow_words[7] = 16'h0000;
        hint_shadow_words[8] = 16'h0000;
        hint_shadow_words[9] = 16'h0000;
        hint_shadow_words[10] = 16'h0000;
        hint_shadow_words[11] = 16'h0000;
        hint_shadow_words[12] = 16'h0000;
        hint_shadow_words[13] = 16'h0000;
        hint_shadow_words[14] = 16'h0000;
        hint_shadow_words[15] = 16'h0000;
        hint_shadow_words[16] = 16'h0000;
        hint_shadow_words[17] = 16'h0000;
        hint_shadow_words[18] = 16'h0000;
        hint_shadow_words[19] = 16'h0000;
        hint_shadow_words[20] = 16'h0000;
        hint_shadow_words[21] = 16'h0000;
        hint_shadow_words[22] = 16'h0000;
        hint_shadow_words[23] = 16'h0000;
        hint_shadow_words[24] = 16'h0000;
        hint_shadow_words[25] = 16'h0000;
        hint_shadow_words[26] = 16'h0000;
        hint_shadow_words[27] = 16'h0000;
        hint_shadow_words[28] = 16'h0000;
        hint_shadow_words[29] = 16'h0000;
        hint_shadow_words[30] = 16'h0000;
        hint_shadow_words[31] = 16'h0000;
        hint_shadow_words[32] = 16'h0000;
        hint_shadow_words[33] = 16'h0000;
        hint_shadow_words[34] = 16'h0000;
        hint_shadow_words[35] = 16'h0000;
        hint_shadow_words[36] = 16'h0000;
        hint_shadow_words[37] = 16'h0000;
        hint_shadow_words[38] = 16'h0000;
        hint_shadow_words[39] = 16'h0000;
        hint_shadow_words[40] = 16'h0000;
        hint_shadow_words[41] = 16'h0000;
        hint_shadow_words[42] = 16'h0000;
        hint_shadow_words[43] = 16'h0000;
        hint_shadow_words[44] = 16'h0000;
        hint_shadow_words[45] = 16'h0000;
        hint_shadow_words[46] = 16'h0000;
        hint_shadow_words[47] = 16'h0000;
        hint_shadow_words[48] = 16'h0e00;
        hint_shadow_words[49] = 16'h0000;
        hint_shadow_words[50] = 16'h0300;
        hint_shadow_words[51] = 16'h0020;
        hint_shadow_words[52] = 16'h0000;
        hint_shadow_words[53] = 16'h0000;
        hint_shadow_words[54] = 16'h0180;
        hint_shadow_words[55] = 16'h0180;
        hint_shadow_words[56] = 16'h0300;
        hint_shadow_words[57] = 16'h0007;
        hint_shadow_words[58] = 16'h0000;
        hint_shadow_words[59] = 16'h0000;
        hint_shadow_words[60] = 16'h0e00;
        hint_shadow_words[61] = 16'h0400;
        hint_shadow_words[62] = 16'h0300;
        hint_shadow_words[63] = 16'h0070;
        hint_shadow_words[64] = 16'hc000;
        hint_shadow_words[65] = 16'h0000;
        hint_shadow_words[66] = 16'h0180;
        hint_shadow_words[67] = 16'h0180;
        hint_shadow_words[68] = 16'h0300;
        hint_shadow_words[69] = 16'h0007;
        hint_shadow_words[70] = 16'h1c00;
        hint_shadow_words[71] = 16'h0000;
        hint_shadow_words[72] = 16'h0e00;
        hint_shadow_words[73] = 16'h0f00;
        hint_shadow_words[74] = 16'h0300;
        hint_shadow_words[75] = 16'h00e0;
        hint_shadow_words[76] = 16'hc000;
        hint_shadow_words[77] = 16'h0000;
        hint_shadow_words[78] = 16'h0180;
        hint_shadow_words[79] = 16'h0180;
        hint_shadow_words[80] = 16'h0300;
        hint_shadow_words[81] = 16'h0007;
        hint_shadow_words[82] = 16'h0c00;
        hint_shadow_words[83] = 16'h0000;
        hint_shadow_words[84] = 16'h0e00;
        hint_shadow_words[85] = 16'h0780;
        hint_shadow_words[86] = 16'h0300;
        hint_shadow_words[87] = 16'h00e0;
        hint_shadow_words[88] = 16'he000;
        hint_shadow_words[89] = 16'h0000;
        hint_shadow_words[90] = 16'he1c0;
        hint_shadow_words[91] = 16'h3ffb;
        hint_shadow_words[92] = 16'h0300;
        hint_shadow_words[93] = 16'h0007;
        hint_shadow_words[94] = 16'hfe00;
        hint_shadow_words[95] = 16'h7fff;
        hint_shadow_words[96] = 16'h0e00;
        hint_shadow_words[97] = 16'h01e0;
        hint_shadow_words[98] = 16'hc300;
        hint_shadow_words[99] = 16'h7fff;
        hint_shadow_words[100] = 16'he000;
        hint_shadow_words[101] = 16'h0000;
        hint_shadow_words[102] = 16'hefc0;
        hint_shadow_words[103] = 16'h3ffb;
        hint_shadow_words[104] = 16'he300;
        hint_shadow_words[105] = 16'h7fff;
        hint_shadow_words[106] = 16'hff00;
        hint_shadow_words[107] = 16'h7fff;
        hint_shadow_words[108] = 16'h0e00;
        hint_shadow_words[109] = 16'h00f8;
        hint_shadow_words[110] = 16'hdfe0;
        hint_shadow_words[111] = 16'h7fff;
        hint_shadow_words[112] = 16'hffe0;
        hint_shadow_words[113] = 16'h7fff;
        hint_shadow_words[114] = 16'h0fc0;
        hint_shadow_words[115] = 16'h3183;
        hint_shadow_words[116] = 16'he300;
        hint_shadow_words[117] = 16'h7fff;
        hint_shadow_words[118] = 16'h0380;
        hint_shadow_words[119] = 16'h0000;
        hint_shadow_words[120] = 16'h0e00;
        hint_shadow_words[121] = 16'h003e;
        hint_shadow_words[122] = 16'hdfe0;
        hint_shadow_words[123] = 16'h7000;
        hint_shadow_words[124] = 16'hffe0;
        hint_shadow_words[125] = 16'h7fff;
        hint_shadow_words[126] = 16'h8060;
        hint_shadow_words[127] = 16'h3181;
        hint_shadow_words[128] = 16'h0f00;
        hint_shadow_words[129] = 16'h0003;
        hint_shadow_words[130] = 16'hff80;
        hint_shadow_words[131] = 16'h1fff;
        hint_shadow_words[132] = 16'h8e00;
        hint_shadow_words[133] = 16'h000f;
        hint_shadow_words[134] = 16'hc300;
        hint_shadow_words[135] = 16'h7018;
        hint_shadow_words[136] = 16'h7000;
        hint_shadow_words[137] = 16'h0000;
        hint_shadow_words[138] = 16'h8060;
        hint_shadow_words[139] = 16'h7ffd;
        hint_shadow_words[140] = 16'h1f60;
        hint_shadow_words[141] = 16'h00c3;
        hint_shadow_words[142] = 16'hffe0;
        hint_shadow_words[143] = 16'h1fff;
        hint_shadow_words[144] = 16'hce00;
        hint_shadow_words[145] = 16'h0003;
        hint_shadow_words[146] = 16'h0300;
        hint_shadow_words[147] = 16'h7038;
        hint_shadow_words[148] = 16'h7000;
        hint_shadow_words[149] = 16'h0000;
        hint_shadow_words[150] = 16'hc070;
        hint_shadow_words[151] = 16'h7ffc;
        hint_shadow_words[152] = 16'hbb60;
        hint_shadow_words[153] = 16'h00c3;
        hint_shadow_words[154] = 16'h0760;
        hint_shadow_words[155] = 16'h1c00;
        hint_shadow_words[156] = 16'hce00;
        hint_shadow_words[157] = 16'h0000;
        hint_shadow_words[158] = 16'h0300;
        hint_shadow_words[159] = 16'h0018;
        hint_shadow_words[160] = 16'h3000;
        hint_shadow_words[161] = 16'h0000;
        hint_shadow_words[162] = 16'hcfe0;
        hint_shadow_words[163] = 16'h3180;
        hint_shadow_words[164] = 16'h9360;
        hint_shadow_words[165] = 16'h00c3;
        hint_shadow_words[166] = 16'hff20;
        hint_shadow_words[167] = 16'h1fff;
        hint_shadow_words[168] = 16'h0e00;
        hint_shadow_words[169] = 16'h0000;
        hint_shadow_words[170] = 16'h0300;
        hint_shadow_words[171] = 16'h001c;
        hint_shadow_words[172] = 16'h3800;
        hint_shadow_words[173] = 16'h0000;
        hint_shadow_words[174] = 16'h6fc0;
        hint_shadow_words[175] = 16'h3180;
        hint_shadow_words[176] = 16'h8360;
        hint_shadow_words[177] = 16'h00c1;
        hint_shadow_words[178] = 16'hff00;
        hint_shadow_words[179] = 16'h1fff;
        hint_shadow_words[180] = 16'hffe0;
        hint_shadow_words[181] = 16'h7fff;
        hint_shadow_words[182] = 16'h1b00;
        hint_shadow_words[183] = 16'h001c;
        hint_shadow_words[184] = 16'h3800;
        hint_shadow_words[185] = 16'h0000;
        hint_shadow_words[186] = 16'h6180;
        hint_shadow_words[187] = 16'h3ff8;
        hint_shadow_words[188] = 16'h8360;
        hint_shadow_words[189] = 16'h00c1;
        hint_shadow_words[190] = 16'h0700;
        hint_shadow_words[191] = 16'h1c00;
        hint_shadow_words[192] = 16'hffe0;
        hint_shadow_words[193] = 16'h7fff;
        hint_shadow_words[194] = 16'hff00;
        hint_shadow_words[195] = 16'hffff;
        hint_shadow_words[196] = 16'h1800;
        hint_shadow_words[197] = 16'h0000;
        hint_shadow_words[198] = 16'he180;
        hint_shadow_words[199] = 16'h3ffb;
        hint_shadow_words[200] = 16'hc370;
        hint_shadow_words[201] = 16'h20c7;
        hint_shadow_words[202] = 16'h0700;
        hint_shadow_words[203] = 16'h1c00;
        hint_shadow_words[204] = 16'h0e00;
        hint_shadow_words[205] = 16'h000e;
        hint_shadow_words[206] = 16'hef80;
        hint_shadow_words[207] = 16'hffff;
        hint_shadow_words[208] = 16'hfc00;
        hint_shadow_words[209] = 16'h3fff;
        hint_shadow_words[210] = 16'he180;
        hint_shadow_words[211] = 16'h1183;
        hint_shadow_words[212] = 16'hc320;
        hint_shadow_words[213] = 16'h70c7;
        hint_shadow_words[214] = 16'hff00;
        hint_shadow_words[215] = 16'h1fff;
        hint_shadow_words[216] = 16'h0e00;
        hint_shadow_words[217] = 16'h000e;
        hint_shadow_words[218] = 16'h03e0;
        hint_shadow_words[219] = 16'h0e06;
        hint_shadow_words[220] = 16'hfc00;
        hint_shadow_words[221] = 16'h3fff;
        hint_shadow_words[222] = 16'h1ff0;
        hint_shadow_words[223] = 16'h0183;
        hint_shadow_words[224] = 16'hc300;
        hint_shadow_words[225] = 16'h39ce;
        hint_shadow_words[226] = 16'hff00;
        hint_shadow_words[227] = 16'h1fff;
        hint_shadow_words[228] = 16'h0e00;
        hint_shadow_words[229] = 16'h000c;
        hint_shadow_words[230] = 16'h03f0;
        hint_shadow_words[231] = 16'h0607;
        hint_shadow_words[232] = 16'h0e00;
        hint_shadow_words[233] = 16'h0018;
        hint_shadow_words[234] = 16'h1fe0;
        hint_shadow_words[235] = 16'h3ffb;
        hint_shadow_words[236] = 16'he300;
        hint_shadow_words[237] = 16'h19ec;
        hint_shadow_words[238] = 16'h7700;
        hint_shadow_words[239] = 16'h1c00;
        hint_shadow_words[240] = 16'h0e00;
        hint_shadow_words[241] = 16'h001c;
        hint_shadow_words[242] = 16'h8320;
        hint_shadow_words[243] = 16'h0703;
        hint_shadow_words[244] = 16'h0f00;
        hint_shadow_words[245] = 16'h0018;
        hint_shadow_words[246] = 16'h3180;
        hint_shadow_words[247] = 16'h1ffb;
        hint_shadow_words[248] = 16'h6300;
        hint_shadow_words[249] = 16'h1dfc;
        hint_shadow_words[250] = 16'h3800;
        hint_shadow_words[251] = 16'h0000;
        hint_shadow_words[252] = 16'h0e00;
        hint_shadow_words[253] = 16'h0038;
        hint_shadow_words[254] = 16'h8300;
        hint_shadow_words[255] = 16'h0703;
        hint_shadow_words[256] = 16'h0700;
        hint_shadow_words[257] = 16'h0018;
        hint_shadow_words[258] = 16'h3180;
        hint_shadow_words[259] = 16'h0183;
        hint_shadow_words[260] = 16'h7300;
        hint_shadow_words[261] = 16'h0be8;
        hint_shadow_words[262] = 16'hfc00;
        hint_shadow_words[263] = 16'h1fff;
        hint_shadow_words[264] = 16'h0e00;
        hint_shadow_words[265] = 16'h0070;
        hint_shadow_words[266] = 16'h8300;
        hint_shadow_words[267] = 16'h038f;
        hint_shadow_words[268] = 16'h0380;
        hint_shadow_words[269] = 16'h0018;
        hint_shadow_words[270] = 16'he180;
        hint_shadow_words[271] = 16'h0181;
        hint_shadow_words[272] = 16'h3b00;
        hint_shadow_words[273] = 16'h03f0;
        hint_shadow_words[274] = 16'hfe00;
        hint_shadow_words[275] = 16'h0fff;
        hint_shadow_words[276] = 16'h0e00;
        hint_shadow_words[277] = 16'h00f0;
        hint_shadow_words[278] = 16'h0300;
        hint_shadow_words[279] = 16'h01fc;
        hint_shadow_words[280] = 16'h03c0;
        hint_shadow_words[281] = 16'h0018;
        hint_shadow_words[282] = 16'he180;
        hint_shadow_words[283] = 16'h3ffd;
        hint_shadow_words[284] = 16'h3b00;
        hint_shadow_words[285] = 16'h0370;
        hint_shadow_words[286] = 16'h3f00;
        hint_shadow_words[287] = 16'h0700;
        hint_shadow_words[288] = 16'h0e00;
        hint_shadow_words[289] = 16'h01e0;
        hint_shadow_words[290] = 16'h0300;
        hint_shadow_words[291] = 16'h01f0;
        hint_shadow_words[292] = 16'h01c0;
        hint_shadow_words[293] = 16'h0018;
        hint_shadow_words[294] = 16'hc980;
        hint_shadow_words[295] = 16'h3ffd;
        hint_shadow_words[296] = 16'h1f00;
        hint_shadow_words[297] = 16'h0738;
        hint_shadow_words[298] = 16'he3c0;
        hint_shadow_words[299] = 16'h03c0;
        hint_shadow_words[300] = 16'h0e00;
        hint_shadow_words[301] = 16'h07c6;
        hint_shadow_words[302] = 16'h0300;
        hint_shadow_words[303] = 16'h07f8;
        hint_shadow_words[304] = 16'h00e0;
        hint_shadow_words[305] = 16'h0018;
        hint_shadow_words[306] = 16'hcd80;
        hint_shadow_words[307] = 16'h0181;
        hint_shadow_words[308] = 16'h0b00;
        hint_shadow_words[309] = 16'h0e38;
        hint_shadow_words[310] = 16'hc1e0;
        hint_shadow_words[311] = 16'h00f3;
        hint_shadow_words[312] = 16'hce00;
        hint_shadow_words[313] = 16'h0f07;
        hint_shadow_words[314] = 16'h0300;
        hint_shadow_words[315] = 16'h0f3e;
        hint_shadow_words[316] = 16'h0070;
        hint_shadow_words[317] = 16'h0018;
        hint_shadow_words[318] = 16'hcf80;
        hint_shadow_words[319] = 16'h0181;
        hint_shadow_words[320] = 16'h0300;
        hint_shadow_words[321] = 16'h0e1c;
        hint_shadow_words[322] = 16'h0060;
        hint_shadow_words[323] = 16'h007f;
        hint_shadow_words[324] = 16'hfe00;
        hint_shadow_words[325] = 16'h7e03;
        hint_shadow_words[326] = 16'hc300;
        hint_shadow_words[327] = 16'h3e0f;
        hint_shadow_words[328] = 16'hfe20;
        hint_shadow_words[329] = 16'h7fff;
        hint_shadow_words[330] = 16'he780;
        hint_shadow_words[331] = 16'h0187;
        hint_shadow_words[332] = 16'h0300;
        hint_shadow_words[333] = 16'h1c0e;
        hint_shadow_words[334] = 16'hc000;
        hint_shadow_words[335] = 16'h07ff;
        hint_shadow_words[336] = 16'hfe00;
        hint_shadow_words[337] = 16'h7800;
        hint_shadow_words[338] = 16'hf3e0;
        hint_shadow_words[339] = 16'h7803;
        hint_shadow_words[340] = 16'hfe00;
        hint_shadow_words[341] = 16'h7fff;
        hint_shadow_words[342] = 16'h6380;
        hint_shadow_words[343] = 16'h001e;
        hint_shadow_words[344] = 16'h8300;
        hint_shadow_words[345] = 16'h7807;
        hint_shadow_words[346] = 16'hfc00;
        hint_shadow_words[347] = 16'h7fe1;
        hint_shadow_words[348] = 16'h1e00;
        hint_shadow_words[349] = 16'h6000;
        hint_shadow_words[350] = 16'h71e0;
        hint_shadow_words[351] = 16'h2000;
        hint_shadow_words[352] = 16'h0000;
        hint_shadow_words[353] = 16'h0000;
        hint_shadow_words[354] = 16'h3180;
        hint_shadow_words[355] = 16'hfffc;
        hint_shadow_words[356] = 16'hc300;
        hint_shadow_words[357] = 16'h7003;
        hint_shadow_words[358] = 16'h3fc0;
        hint_shadow_words[359] = 16'h7e00;
        hint_shadow_words[360] = 16'h0400;
        hint_shadow_words[361] = 16'h0000;
        hint_shadow_words[362] = 16'h0000;
        hint_shadow_words[363] = 16'h0000;
        hint_shadow_words[364] = 16'h0000;
        hint_shadow_words[365] = 16'h0000;
        hint_shadow_words[366] = 16'h3800;
        hint_shadow_words[367] = 16'h7ff0;
        hint_shadow_words[368] = 16'h8300;
        hint_shadow_words[369] = 16'h6001;
        hint_shadow_words[370] = 16'h0380;
        hint_shadow_words[371] = 16'h0000;
        hint_shadow_words[372] = 16'h0000;
        hint_shadow_words[373] = 16'h0000;
        hint_shadow_words[374] = 16'h0000;
        hint_shadow_words[375] = 16'h0000;
        hint_shadow_words[376] = 16'h0000;
        hint_shadow_words[377] = 16'h0000;
        hint_shadow_words[378] = 16'h1000;
        hint_shadow_words[379] = 16'h0000;
        hint_shadow_words[380] = 16'h0000;
        hint_shadow_words[381] = 16'h0000;
        hint_shadow_words[382] = 16'h0000;
        hint_shadow_words[383] = 16'h0000;
    end
    // SHADOW_GENERATED_END

    // Delay readout formatting lives after the generated glyph ROM so the
    // GLYPH_DOT/GLYPH_DASH indices are already declared.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            delay_s<=12'd0;
            delay_valid_s<=1'b0;
            delay_glyphs_r<=32'h0d0d0000;
        end else begin
            if(I_vs && !vs_previous) begin
                delay_s<=I_delay_ms10;
                delay_valid_s<=I_delay_valid;
            end
            if(budget_step==2'd2) begin
                if(delay_valid_s && delay_s < 12'd1000)
                    delay_glyphs_r<={digit_glyph(delay_s/12'd100),
                                     digit_glyph((delay_s/12'd10)%12'd10),
                                     GLYPH_DOT[7:0],
                                     digit_glyph(delay_s%12'd10)};
                else
                    delay_glyphs_r<={GLYPH_DASH[7:0],GLYPH_DASH[7:0],8'd0,8'd0};
            end
        end
    end

    // The active raster is 1920x1080; keep geometry comparisons at the
    // native 12-bit coordinate width instead of promoting every comparison
    // to a 32-bit integer datapath.
    reg [11:0] x,y;
    reg [23:0] background_color, text_color;
    reg glyph_active;
    reg [12:0] glyph_address;
    reg [4:0] glyph_column;
    reg [7:0] current_glyph;
    integer tile;
    function inside;
        input [11:0] px,py,left,top,width,height;
        begin inside=(px>=left && px<left+width && py>=top && py<top+height);end
    endfunction
    function rounded_inside;
        input [11:0] px,py,left,top,width,height,radius;
        reg [11:0] dxl,dxr,dyt,dyb;
        reg chamfer;
        begin
            rounded_inside=inside(px,py,left,top,width,height);
            // Fixed three-pixel chamfers keep the pixel path to comparisons
            // while giving buttons and focus frames a visibly softer shape.
            // The same cut expressed as a corner distance sum: the removed
            // pixels are exactly where (distance from side)+(distance from
            // edge) <= 2.  This shares four subtractions instead of a long
            // 12-term equality chain, which matters on the 150 MHz path.
            if(rounded_inside && radius!=0) begin
                dxl=px-left;
                dxr=left+width-1-px;
                dyt=py-top;
                dyb=top+height-1-py;
                chamfer=((dxl<=12'd2 && dyt<=12'd2 && (dxl+dyt)<=12'd2) ||
                         (dxr<=12'd2 && dyt<=12'd2 && (dxr+dyt)<=12'd2) ||
                         (dxl<=12'd2 && dyb<=12'd2 && (dxl+dyb)<=12'd2) ||
                         (dxr<=12'd2 && dyb<=12'd2 && (dxr+dyb)<=12'd2));
                if(chamfer) rounded_inside=0;
            end
        end
    endfunction
    task text;
        input integer left,top,label_id,scale;
        input [23:0] color;
        input hit;
        integer col_idx,rx,ry;
        begin
            if(hit) begin
                rx=(x-left)>>scale;ry=(y-top)>>scale;col_idx=rx>>5;
                current_glyph=label_glyph(label_id,col_idx);
                glyph_active=1;
                glyph_address=0;
                glyph_address[12:5]=current_glyph;
                glyph_address[4:0]=ry[4:0];
                glyph_column=rx&31;text_color=color;
            end
        end
    endtask
    task centered_text;
        input integer center_x,center_y,label_id,scale;
        input [23:0] color;
        input hit;
        integer text_width,text_height;
        begin
            text_width=label_length(label_id)*(32<<scale);
            text_height=32<<scale;
            text(center_x-(text_width/2),center_y-(text_height/2),label_id,scale,color,hit);
        end
    endtask
    task value_text;
        input integer left,top;
        input [23:0] glyphs;
        input [1:0] first_digit;
        input [7:0] text_width;
        input percent;
        input [23:0] color;
        input hit;
        integer rx,ry,digit_index,digit_count;
        begin
            digit_count=3-first_digit;
            if(hit) begin
                rx=x-left;ry=y-top;
                if(rx<digit_count*16) begin
                    if(rx<16) begin
                        digit_index=first_digit;
                        glyph_column=5'd8+(rx&15);
                    end else if(rx<32) begin
                        digit_index=first_digit+1;
                        glyph_column=5'd8+((rx-16)&15);
                    end else begin
                        digit_index=first_digit+2;
                        glyph_column=5'd8+((rx-32)&15);
                    end
                    case(digit_index)
                        0:current_glyph=glyphs[23:16];
                        1:current_glyph=glyphs[15:8];
                        default:current_glyph=glyphs[7:0];
                    endcase
                end else begin
                    current_glyph=GLYPH_PERCENT;
                    glyph_column=rx-(digit_count*16);
                end
                glyph_active=1;
                glyph_address=0;
                glyph_address[12:5]=current_glyph;
                glyph_address[4:0]=ry[4:0];
                text_color=color;
            end
        end
    endtask
    // Four 16-pixel glyph cells: tens, ones, dot, tenths.  Uses the same
    // centred 16-column slice of the 32x32 glyph ROM as value_text.
    task delay_text;
        input integer left,top;
        input [31:0] glyphs;
        input [7:0] text_width;
        input [23:0] color;
        input hit;
        integer rx,ry,idx;
        begin
            if(hit) begin
                rx=x-left;ry=y-top;
                idx=rx>>4;
                if(rx<text_width) begin
                    glyph_column=5'd8+(rx&15);
                    case(idx)
                        0:current_glyph=glyphs[31:24];
                        1:current_glyph=glyphs[23:16];
                        2:current_glyph=glyphs[15:8];
                        default:current_glyph=glyphs[7:0];
                    endcase
                end
                glyph_active=1;
                glyph_address=0;
                glyph_address[12:5]=current_glyph;
                glyph_address[4:0]=ry[4:0];
                text_color=color;
            end
        end
    endtask

    task centered_value;
        input integer center_x,center_y;
        input [23:0] glyphs;
        input [1:0] first_digit;
        input [7:0] text_width;
        input percent;
        input [23:0] color;
        input hit;
        begin
            value_text(center_x-(text_width/2),center_y-16,glyphs,first_digit,text_width,percent,color,hit);
        end
    endtask
    task parameter_row;
        input integer top,label_id,focus_id;
        input [23:0] glyphs;
        input [1:0] first_digit;
        input [7:0] text_width;
        input [10:0] bar_end;
        input show_bar,percent_value;
        input hit_out,hit_in,hit_lbl,hit_val,hit_bar;
        begin
            if(O_page && O_focus==focus_id && hit_out) begin
                if(!hit_in) background_color=FOCUS;
                else background_color=SELECT;
            end
            centered_text(1570,top+16,label_id,0,(O_page && O_focus==focus_id)?FOCUS:INK,hit_lbl);
            centered_value(1832,top+16,glyphs,first_digit,text_width,percent_value,
                           (O_page && O_focus==focus_id)?FOCUS:INK,hit_val);
            if(show_bar && hit_bar) begin
                background_color=(x<bar_end)?ACCENT:TRACK;
            end
        end
    endtask
    // =========================================================================
    // Geometry pre-compute stage (stage 0)
    //
    // Every inside()/rounded_inside() test in the render block below uses
    // compile-time constant geometry except for the pixel coordinate, so the
    // whole set is evaluated one cycle early here.  The render block then only
    // performs selection, cutting the 150 MHz critical path from 11-12 levels
    // (y_d8 -> inside -> chamfer -> priority mux -> bg_stage1) down to two
    // short stages.
    //
    // x_d8/y_d8/mode_d8/de_d8/vs_d8/hs_d8 all advance by exactly one cycle so
    // the OSD image is unchanged; only the total latency goes from 10 to 11
    // cycles, and O_de/O_vs/O_rgb stay aligned because they move together.
    //
    // INVARIANT: O_page/O_focus/O_compare_en/split_left/split_right and the
    // *_text_width values are sampled here one cycle before the render block
    // uses them.  Those signals may only change during vertical blanking
    // (they are updated on the I_vs rising edge, and hit_* is only consumed
    // when de_d9 is high), otherwise a one-pixel edge artifact would appear.
    // =========================================================================
    reg [11:0] x_d9, y_d9, y_content_d9;
    reg [2:0] mode_d9;
    reg hs_d9, vs_d9, de_d9;
    reg hit_content;
    reg hit_hdr_title, hit_hdr_rate, hit_hdr_fps;
    reg hit_hdr_delay_lbl, hit_hdr_delay_val, hit_hdr_delay_unit;
    reg hit_logo;
    reg hit_hint, hit_hint_shadow;
    reg [12:0] hint_shadow_index_s0;
    // ★ 2026-11-01 自动曝光读数的 2px 黑描边。阴影掩码的判据是「源像素
    //   (x-2, y-2) 有字墨」，所以 RTL 侧只需按 x-2/y-2 去查字形本身：
    //     ae_shadow_row()   把 714/754 两行折叠成 0..31
    //     ae_dig_sel()      按 value_text 的同一套映射取该位的数字字形
    function [4:0] ae_shadow_row;
        input [11:0] yv;
        reg [11:0] r;
        begin
            r=yv-12'd714;
            if(r>=12'd40) r=r-12'd40;
            ae_shadow_row=r[4:0];
        end
    endfunction
    function [7:0] ae_dig_sel;
        input [1:0] first_digit;
        input [23:0] glyphs;
        input [1:0] cell_idx;          // cell 是 Verilog 保留字，不能用作标识符
        reg [1:0] digit_index;
        begin
            digit_index=first_digit+cell_idx;
            case(digit_index)
                2'd0:ae_dig_sel=glyphs[23:16];
                2'd1:ae_dig_sel=glyphs[15:8];
                default:ae_dig_sel=glyphs[7:0];
            endcase
        end
    endfunction
    // 阴影源偏移 g = (x-2)-96 = x-98：value_text 的单元格从 x=96 起，
    // 源像素 xs=x-2 落在 [96+16k, 112+16k)，所以 g 的 [5:4] 是单元格号、
    // [3:0] 是字形内的列偏移（对应 glyph ROM 的 8..23 列）。
    // 注意用 y_content_d8（= y-78），文字是画在内容坐标系里的；
    // 用全局 y_d8 会整体错开 78 行，描边落到没有字的空白处。
    wire [11:0] ae_g = x_d8 - 12'd98;
    wire        ae_row_hi = (y_content_d8 >= 12'd754);
    wire        ae_row_ok = ((y_content_d8 >= 12'd714) && (y_content_d8 < 12'd746)) ||
                            ((y_content_d8 >= 12'd754) && (y_content_d8 < 12'd786));
    wire [7:0]  ae_dig_byte = ae_row_hi
                              ? ae_dig_sel(ae_gain_first,ae_gain_glyphs,ae_g[5:4])
                              : ae_dig_sel(ae_mean_first,ae_mean_glyphs,ae_g[5:4]);
    // 0xff 是补位空字形（digit_glyph 的空值），没有字墨，也就没有阴影。
    wire [7:0]  ae_dig_value = ae_dig_byte - GLYPH_DIGIT_ZERO;
    wire        ae_dig_ok    = (ae_dig_value <= 8'd9);
    wire [3:0]  ae_dig_num   = ae_dig_ok ? ae_dig_value[3:0] : 4'd0;
    // ★ 2026-11-01 构图辅助线几何。纯比较、没有除法：两种模式
    //   各用一组常量（缩小页面 1456×819 / 全屏 1920×1080）。
    wire        grid_vis = O_menu_visible;
    wire [11:0] grid_w   = grid_vis ? 12'd1456 : 12'd1920;
    wire [11:0] grid_h   = grid_vis ? 12'd819  : 12'd1080;
    wire [11:0] grid_y   = grid_vis ? y_content_d8 : y_d8;
    wire [11:0] g_v1     = grid_vis ? 12'd485  : 12'd640;
    wire [11:0] g_v2     = grid_vis ? 12'd970  : 12'd1280;
    wire [11:0] g_h1     = grid_vis ? 12'd273  : 12'd360;
    wire [11:0] g_h2     = grid_vis ? 12'd546  : 12'd720;
    wire [11:0] g_cx     = grid_vis ? 12'd728  : 12'd960;
    wire [11:0] g_cy     = grid_vis ? 12'd409  : 12'd540;
    wire [11:0] g_cy_lo  = g_cy - 12'd12;
    wire [11:0] g_cy_hi  = g_cy + 12'd12;
    wire grid_in_area = (x_d8 < grid_w) && (grid_y < grid_h);
    // 细线：一个像素宽（之前是 2px 粗线 + 2px 黑描边）
    wire grid_vline   = (x_d8 == g_v1) || (x_d8 == g_v2);
    wire grid_hline   = (grid_y == g_h1) || (grid_y == g_h2);
    wire grid_cross   = ((grid_y == g_cy) &&
                         (x_d8 >= (g_cx - 12'd12)) && (x_d8 <= (g_cx + 12'd12)))
                     || ((x_d8 == g_cx) &&
                         (grid_y >= g_cy_lo) && (grid_y <= g_cy_hi));
    reg  hit_grid;
    reg  hit_ae_lbl_shadow, hit_ae_dig_shadow;
    reg  [5:0] ae_lbl_idx_s0, ae_lbl_col_s0;
    reg  [8:0] ae_dig_idx_s0;
    reg  [3:0] ae_dig_col_s0;
    reg [5:0] logo_row_s1;
    reg hit_bot_line, hit_bot_title;
    reg [4:0] hit_tile_out, hit_tile_in, hit_tile_txt;
    reg [3:0] hit_rp_border;
    reg hit_rp_hdrbg, hit_rp_mode, hit_rp_div;
    reg [4:0] hit_pr_out, hit_pr_in, hit_pr_bar, hit_pr_lbl, hit_pr_val;
    reg hit_cmp_out, hit_cmp_in, hit_cmp_lbl;
    reg hit_sw_out, hit_sw_in;
    reg hit_split_out, hit_split_in, hit_split_lbl, hit_split_val, hit_split_bar;
    reg hit_page_out, hit_page_in, hit_page_btn, hit_page_inner, hit_page_lbl;
    reg hit_vid_left, hit_vid_bot, hit_vid_split;
    integer gi;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            x_d9<=0;y_d9<=0;y_content_d9<=0;mode_d9<=0;hs_d9<=0;vs_d9<=0;de_d9<=0;
            camera_rgb_d9<=0;
            hit_content<=0;
            hit_hdr_title<=0;hit_hdr_rate<=0;hit_hdr_fps<=0;
            hit_hdr_delay_lbl<=0;hit_hdr_delay_val<=0;hit_hdr_delay_unit<=0;
            hit_logo<=0;logo_row_s1<=0;hit_hint<=0;hit_hint_shadow<=0;
            hint_shadow_index_s0<=0;
            hit_ae_lbl_shadow<=0;hit_ae_dig_shadow<=0;
            hit_grid<=0;
            ae_lbl_idx_s0<=0;ae_lbl_col_s0<=0;ae_dig_idx_s0<=0;ae_dig_col_s0<=0;
            hit_bot_line<=0;
            hit_tile_out<=0;hit_tile_in<=0;hit_tile_txt<=0;
            hit_bot_title<=0;
            hit_rp_border<=0;hit_rp_hdrbg<=0;hit_rp_mode<=0;hit_rp_div<=0;
            hit_pr_out<=0;hit_pr_in<=0;hit_pr_bar<=0;hit_pr_lbl<=0;hit_pr_val<=0;
            hit_cmp_out<=0;hit_cmp_in<=0;hit_cmp_lbl<=0;
            hit_sw_out<=0;hit_sw_in<=0;
            hit_split_out<=0;hit_split_in<=0;hit_split_lbl<=0;
            hit_split_val<=0;hit_split_bar<=0;
            hit_page_out<=0;hit_page_in<=0;hit_page_btn<=0;hit_page_inner<=0;
            hit_page_lbl<=0;
            hit_vid_left<=0;hit_vid_bot<=0;hit_vid_split<=0;
        end else begin
            x_d9<=x_d8;y_d9<=y_d8;y_content_d9<=y_content_d8;mode_d9<=mode_d8;
            hs_d9<=hs_d8;vs_d9<=vs_d8;de_d9<=de_d8;
            camera_rgb_d9<=camera_rgb_d8;

            // Video content background (not gated by DE).
            hit_content<=inside(x_d8,y_content_d8,0,0,1456,819);

            // Header uses global y; every other region uses content y.
            hit_hdr_title<=inside(x_d8,y_d8,102,23,256,32);
            hit_logo<=inside(x_d8,y_d8,LOGO_X0,LOGO_Y0,12'd72,12'd50);
            hit_hint<=inside(x_d8,y_d8,12'd1704,12'd1024,12'd192,12'd32);
            hit_hint_shadow<=inside(x_d8,y_d8,12'd1706,12'd1026,12'd192,12'd32) && !O_menu_visible;
            // index = row*192 + col, packed 16 pixels per ROM word
            hint_shadow_index_s0<=((y_d8-12'd1026)<<7)
                                  + ((y_d8-12'd1026)<<6)
                                  + (x_d8-12'd1706);

            // ── 自动曝光读数的 2px 黑描边（★ 2026-11-01）──────────────
            //  标签行：字墨在 x∈[24,88) y∈{712,752}，滚动 2px 后阴影落在
            //          x∈[26,90) y∈[714,746)/[754,786)。列直接就是标签内偏移。
            //   ★ 必须同时门控 O_menu_visible：白字本身画在菜单分支里，
            //     全屏时不画；但描边掩码如果只跟 ae_disp_on 走，就会在
            //     全屏左下角留下一片黑色字形轮廓（看着像还有字）。
            hit_ae_lbl_shadow<=ae_disp_on && O_menu_visible && ae_row_ok
                              && (x_d8>=12'd26) && (x_d8<12'd90);
            ae_lbl_idx_s0<=(ae_row_hi?6'd32:6'd0)+{1'b0,ae_shadow_row(y_content_d8)};
            ae_lbl_col_s0<=x_d8-12'd26;
            //  数字行：源偏移 g 的 [5:4] 选单元格、[3:0] 选字形内列。
            hit_ae_dig_shadow<=ae_disp_on && O_menu_visible && ae_row_ok && ae_dig_ok
                              && (x_d8>=12'd98)
                              && (ae_g < (ae_row_hi?{4'd0,ae_gain_width}
                                                   :{4'd0,ae_mean_width}));
            ae_dig_idx_s0<={ae_dig_num,ae_shadow_row(y_content_d8)};
            ae_dig_col_s0<=ae_g[3:0];
            // ★ 构图辅助线：三分线（2px）+ 中心十字准星（25px 臂、2px 粗）
            hit_grid<=grid_on && grid_in_area && (grid_vline || grid_hline || grid_cross);
            logo_row_s1<=(y_d8-LOGO_Y0);
            hit_hdr_rate<=inside(x_d8,y_d8,1578,23,64,32);
            hit_hdr_fps<=inside(x_d8,y_d8,1668-(fps_width_r>>1),23,fps_width_r,32);
            hit_hdr_delay_lbl<=inside(x_d8,y_d8,1138,23,160,32);
            hit_hdr_delay_val<=inside(x_d8,y_d8,1314,23,64,32);
            hit_hdr_delay_unit<=inside(x_d8,y_d8,1386,23,64,32);

            // Bottom mode bar.
            hit_bot_line<=inside(x_d8,y_content_d8,0,819,1920,4);
            hit_bot_title<=inside(x_d8,y_content_d8,896,841,128,32);
            for(gi=0;gi<5;gi=gi+1) begin
                hit_tile_out[gi]<=rounded_inside(x_d8,y_content_d8,38+gi*377,885,336,74,6);
                hit_tile_in[gi]<=rounded_inside(x_d8,y_content_d8,42+gi*377,889,328,66,4);
                hit_tile_txt[gi]<=inside(x_d8,y_content_d8,174+gi*377,906,64,32);
            end

            // Right panel frame and mode title.
            hit_rp_border[0]<=inside(x_d8,y_content_d8,1456,0,464,4);
            hit_rp_border[1]<=inside(x_d8,y_content_d8,1456,815,464,4);
            hit_rp_border[2]<=inside(x_d8,y_content_d8,1456,0,4,819);
            hit_rp_border[3]<=inside(x_d8,y_content_d8,1916,0,4,819);
            hit_rp_hdrbg<=inside(x_d8,y_content_d8,1460,0,456,122);
            hit_rp_mode<=inside(x_d8,y_content_d8,1624,29,128,64);
            hit_rp_div<=inside(x_d8,y_content_d8,1506,520,362,2);

            // parameter_row() expanded: four call sites, five row instances
            // (rows 1/3 share top 236 and rows 2/4 share top 331).
            // Note: centered_text(cx, top+16, ..) lands on text top == top.
            hit_pr_out[0]<=rounded_inside(x_d8,y_content_d8,1492,128,392,84,9);
            hit_pr_in[0]<=rounded_inside(x_d8,y_content_d8,1496,132,384,76,7);
            hit_pr_lbl[0]<=inside(x_d8,y_content_d8,1506,140,128,32);
            hit_pr_val[0]<=inside(x_d8,y_content_d8,1832-(strength_text_width>>1),140,strength_text_width,32);
            hit_pr_bar[0]<=rounded_inside(x_d8,y_content_d8,1506,176,362,14,7);

            hit_pr_out[1]<=rounded_inside(x_d8,y_content_d8,1492,224,392,84,9);
            hit_pr_in[1]<=rounded_inside(x_d8,y_content_d8,1496,228,384,76,7);
            hit_pr_lbl[1]<=inside(x_d8,y_content_d8,1506,236,128,32);
            hit_pr_val[1]<=inside(x_d8,y_content_d8,1832-(secondary_text_width>>1),236,secondary_text_width,32);
            hit_pr_bar[1]<=rounded_inside(x_d8,y_content_d8,1506,272,362,14,7);

            hit_pr_out[2]<=rounded_inside(x_d8,y_content_d8,1492,319,392,84,9);
            hit_pr_in[2]<=rounded_inside(x_d8,y_content_d8,1496,323,384,76,7);
            hit_pr_lbl[2]<=inside(x_d8,y_content_d8,1506,331,128,32);
            // 原图页第 3 行画的是 he_strength（primary_text_width），其余模式
            // 画的是 smooth（smooth_text_width）。取两者较大值，居中对称，
            // 两种画法都覆盖得到。
            hit_pr_val[2]<=inside(x_d8,y_content_d8,1832-(val_w_row2>>1),331,val_w_row2,32);
            hit_pr_bar[2]<=rounded_inside(x_d8,y_content_d8,1506,367,362,14,7);

            // Rows 3/4 reuse the top of rows 1/2 (236/331); only their label_id
            // varies with mode, and label_length() is 4 for every id in use, so
            // the text geometry stays constant.
            hit_pr_out[3]<=rounded_inside(x_d8,y_content_d8,1492,224,392,84,9);
            hit_pr_in[3]<=rounded_inside(x_d8,y_content_d8,1496,228,384,76,7);
            hit_pr_lbl[3]<=inside(x_d8,y_content_d8,1506,236,128,32);
            hit_pr_val[3]<=inside(x_d8,y_content_d8,1832-(primary_text_width>>1),236,primary_text_width,32);
            hit_pr_bar[3]<=rounded_inside(x_d8,y_content_d8,1506,272,362,14,7);

            hit_pr_out[4]<=rounded_inside(x_d8,y_content_d8,1492,319,392,84,9);
            hit_pr_in[4]<=rounded_inside(x_d8,y_content_d8,1496,323,384,76,7);
            hit_pr_lbl[4]<=inside(x_d8,y_content_d8,1506,331,128,32);
            hit_pr_val[4]<=inside(x_d8,y_content_d8,1832-(secondary_text_width>>1),331,secondary_text_width,32);
            hit_pr_bar[4]<=rounded_inside(x_d8,y_content_d8,1506,367,362,14,7);

            // Compare row (focus 4) and the on/off switch.
            hit_cmp_out<=rounded_inside(x_d8,y_content_d8,1492,532,292,56,9);
            hit_cmp_in<=rounded_inside(x_d8,y_content_d8,1496,536,284,48,7);
            hit_cmp_lbl<=inside(x_d8,y_content_d8,1506,544,128,32);
            hit_sw_out<=rounded_inside(x_d8,y_content_d8,1796,540,72,40,20);
            hit_sw_in<=inside(x_d8,y_content_d8,O_compare_en?1832:1798,544,36,32);

            // Split row (focus 5).
            hit_split_out<=rounded_inside(x_d8,y_content_d8,1492,598,392,84,9);
            hit_split_in<=rounded_inside(x_d8,y_content_d8,1496,602,384,76,7);
            hit_split_lbl<=inside(x_d8,y_content_d8,1506,616,128,32);
            hit_split_val<=inside(x_d8,y_content_d8,1832-(split_text_width>>1),616,split_text_width,32);
            hit_split_bar<=rounded_inside(x_d8,y_content_d8,1506,655,362,14,7);

            // Page row (focus 6).
            hit_page_out<=rounded_inside(x_d8,y_content_d8,1492,709,392,76,9);
            hit_page_in<=rounded_inside(x_d8,y_content_d8,1496,713,384,68,7);
            hit_page_btn<=rounded_inside(x_d8,y_content_d8,1504,717,368,60,8);
            hit_page_inner<=rounded_inside(x_d8,y_content_d8,1508,721,360,52,6);
            hit_page_lbl<=inside(x_d8,y_content_d8,1624,731,128,32);

            // Video area borders and the compare split line.
            hit_vid_left<=inside(x_d8,y_content_d8,1452,0,4,819);
            hit_vid_bot<=inside(x_d8,y_content_d8,0,815,1456,4);
            hit_vid_split<=inside(x_d8,y_content_d8,split_left,0,split_right-split_left,819);
        end
    end

    reg logo_active_s1;
    reg [6:0] logo_column_s1;

    always @* begin
        x=x_d9;
        // Render the reference-style 78-pixel header in global coordinates;
        // all remaining geometry uses content-local y, so the whole UI moves
        // down together without touching HDMI timing.
        y=y_content_d9;
        if(y_d9<12'd78)
            background_color=PANEL;
        else if(hit_content) begin
            // ★ 2026-10-21 滤镜搬到 ISP 写侧（ui_filter_domain_new）：
            //   DDR 里存的就是滤镜后的图，这里只负责显示；
            //   原图对比也在写侧由延迟链完成，显示域不再做像素选择。
            background_color=camera_rgb_d9;
        end
        else
            background_color=CANVAS;
        glyph_active=0;glyph_address=0;glyph_column=0;text_color=INK;current_glyph=0;
        logo_active_s1=hit_logo && O_menu_visible;
        logo_column_s1=x_d9-LOGO_X0;
        tile=0;
        if(de_d9) begin
            if(!O_menu_visible) begin
                // Hidden UI: camera fills the whole active raster, only the
                // restore hint is drawn (white glyphs with a 2-pixel shadow).
                background_color=camera_rgb_d9;
                y=y_d9;
                text(1706,1026,53,0,24'h000000,hit_hint_shadow);
                text(1704,1024,53,0,24'hffffff,hit_hint);
            end else if(y_d9<12'd78) begin
                // Header: same hierarchy as the generated reference image.
                background_color=PANEL;
                y=y_d9;
                centered_text(230,39,0,0,WHITE,hit_hdr_title);
                // ★ 2026-10-23：帧率从固定文本"--"改成实测值。
                //   "帧率"两字占 64 像素，数字半宽 16/位，整体居中在 1630。
                centered_text(1218,39,50,0,MUTED,hit_hdr_delay_lbl);
                delay_text(1314,23,delay_glyphs_r,8'd64,MUTED,hit_hdr_delay_val);
                centered_text(1418,39,51,0,MUTED,hit_hdr_delay_unit);
                centered_text(1610,39,2,0,MUTED,hit_hdr_rate);
                centered_value(1668,39,fps_glyphs_r,fps_first_r,fps_width_r,1'b0,MUTED,hit_hdr_fps);
                y=y_content_d9;
            end else if(y_d9>=12'd897) begin
                // Bottom mode bar (content y 819..1001) and the right panel are
                // spatially disjoint; keep the two override chains parallel.
                background_color=CANVAS;
                if(hit_bot_line) background_color=BORDER;
                centered_text(960,857,20,0,MUTED,hit_bot_title);
                for(tile=0;tile<5;tile=tile+1) begin
                    if(hit_tile_out[tile]) begin
                        if(mode_d9==tile) background_color=ACCENT;
                        else if(!hit_tile_in[tile]) background_color=TRACK;
                        // 底栏 5 个格子的位置是 UI 顺序，标签要按"该位置对应哪个滤镜"取
                        centered_text(206+tile*377,922,10+mode_to_oldpos(tile),0,mode_d9==tile?WHITE:INK,hit_tile_txt[tile]);
                    end
                end
            end else if(x_d9>=12'd1456) begin
                background_color=PANEL;
                if(|hit_rp_border) background_color=BORDER;
                if(hit_rp_hdrbg) background_color=CANVAS;
                centered_text(1688,61,10+mode_to_oldpos(mode_d9),1,WHITE,hit_rp_mode);
                // ★ 2026-11-01 原图页三行，顺序为：构图辅助 / 曝光显示 / 均衡强度
                //   前两行是胶囊开关（样式与「原图对比」一致），第三行是滑块。
                //   focus 0/1/2 分别对应这三行。
                if(mode_d9==3'd0) begin
                    // 第一行 y=140：构图辅助
                    if(O_page && O_focus==0 && hit_pr_out[0]) begin
                        if(!hit_pr_in[0]) background_color=FOCUS;
                        else background_color=SELECT;
                    end
                    centered_text(1570,156,54,0,(O_page && O_focus==0)?FOCUS:INK,hit_pr_lbl[0]);
                    if(rounded_inside(x,y,1796,140,72,40,20))
                        background_color = grid_on ? ACCENT : TRACK;
                    if(inside(x,y, grid_on?1832:1798, 144, 36, 32))
                        background_color = WHITE;

                    // 第二行 y=236：曝光显示
                    if(O_page && O_focus==1 && hit_pr_out[1]) begin
                        if(!hit_pr_in[1]) background_color=FOCUS;
                        else background_color=SELECT;
                    end
                    centered_text(1570,252,45,0,(O_page && O_focus==1)?FOCUS:INK,hit_pr_lbl[1]);
                    if(rounded_inside(x,y,1796,236,72,40,20))
                        background_color = ae_disp_on ? ACCENT : TRACK;
                    if(inside(x,y, ae_disp_on?1832:1798, 240, 36, 32))
                        background_color = WHITE;

                    // 第三行 y=331：均衡强度（滑块）
                    parameter_row(331,52,2,primary_glyphs,primary_first_digit,primary_text_width,primary_end,1,1,
                                  hit_pr_out[2],hit_pr_in[2],hit_pr_lbl[2],hit_pr_val[2],hit_pr_bar[2]);
                end else begin
                    parameter_row(140,4,0,strength_glyphs,strength_first_digit,strength_text_width,strength_end,1,1,
                                  hit_pr_out[0],hit_pr_in[0],hit_pr_lbl[0],hit_pr_val[0],hit_pr_bar[0]);
                end
                // ★ 2026-10-29：分支条件按 **滤镜号**（mode_to_filt）选；
                //   标签 id 按 **旧顺序位置**（mode_to_oldpos）取（字模表就是这样编号的）；
                //   参数数组按 **UI 位置**（mode_d9）索引。三者别混。
                if(mode_to_filt(mode_d9)==3'd3) begin
                    // 铅笔：线条强度已删除，只保留边缘阈值 / 色彩强度
                    parameter_row(236,31,1,secondary_glyphs,secondary_first_digit,secondary_text_width,secondary_end,1,0,
                                  hit_pr_out[1],hit_pr_in[1],hit_pr_lbl[1],hit_pr_val[1],hit_pr_bar[1]);
                    parameter_row(331,35,2,smooth_glyphs,smooth_first_digit,smooth_text_width,smooth_end,1,1,
                                  hit_pr_out[2],hit_pr_in[2],hit_pr_lbl[2],hit_pr_val[2],hit_pr_bar[2]);
                end else if(mode_to_filt(mode_d9)==3'd4) begin
                    // 像素：只有一行“风格模式” 0=D / 1=E / 2=F
                    parameter_row(236,24,1,primary_glyphs,primary_first_digit,primary_text_width,primary_end,1,0,
                                  hit_pr_out[3],hit_pr_in[3],hit_pr_lbl[3],hit_pr_val[3],hit_pr_bar[3]);
                end else if(mode_to_filt(mode_d9)!=3'd0) begin
                    parameter_row(236,20+mode_to_oldpos(mode_d9),1,primary_glyphs,primary_first_digit,primary_text_width,primary_end,1,1,
                                  hit_pr_out[3],hit_pr_in[3],hit_pr_lbl[3],hit_pr_val[3],hit_pr_bar[3]);
                    parameter_row(331,30+mode_to_oldpos(mode_d9),2,secondary_glyphs,secondary_first_digit,secondary_text_width,secondary_end,1,0,
                                  hit_pr_out[4],hit_pr_in[4],hit_pr_lbl[4],hit_pr_val[4],hit_pr_bar[4]);
                end
                if(hit_rp_div) background_color=BORDER;
                if(O_page && O_focus==4 && hit_cmp_out) begin
                    if(!hit_cmp_in) background_color=FOCUS;
                    else background_color=SELECT;
                end
                centered_text(1570,560,6,0,INK,hit_cmp_lbl);
                if(hit_sw_out) begin
                    background_color=O_compare_en?ACCENT:TRACK;
                    if(hit_sw_in) background_color=WHITE;
                end
                if(O_page && O_focus==5 && hit_split_out) begin
                    if(!hit_split_in) background_color=FOCUS;
                    else background_color=SELECT;
                end
                centered_text(1570,632,9,0,INK,hit_split_lbl);
                centered_value(1832,632,split_glyphs,split_first_digit,split_text_width,1,INK,hit_split_val);
                if(hit_split_bar) background_color=(x<split_end)?ACCENT:TRACK;
                if(O_page && O_focus==6 && hit_page_out) begin
                    if(!hit_page_in) background_color=FOCUS;
                    else background_color=SELECT;
                end
                if(hit_page_btn) begin
                    if(O_page && O_focus==6)
                        background_color=hit_page_inner?SELECT:FOCUS;
                    else
                        background_color=hit_page_inner?SELECT:BORDER;
                    centered_text(1688,747,17,0,INK,hit_page_lbl);
                end
            end else begin
                // Video content area: only its 4-pixel borders and compare split.
                if(hit_vid_left || hit_vid_bot) background_color=BORDER;
                if(O_compare_en && hit_vid_split)
                    background_color=WHITE;

                // ★ 2026-10-31 自动曝光读数：画面左下角两行竖排（亮度 / 增益）。
                //   预览区是 x 0..1452 / y 0..815，两行放在 712 / 752。
                //   ★ hit 参数必须是命中判定，不能传 1'b1，否则整屏平铺。
                //   ★ 2026-11-01 改成白字 + 2px 黑阴影。阴影不走这条通路（一次
                //     像素只读一次字模 ROM），由 ae_*_shadow_* 掩码 ROM 在
                //     输出级做第三层合成，见下方 O_rgb 的优先级。
                if(ae_disp_on) begin
                    text(24,712,46,0,24'hffffff,
                         inside(x,y,24,712,64,32));                  // 亮度
                    value_text(96,712,ae_mean_glyphs,ae_mean_first,
                               ae_mean_width,1'b0,24'hffffff,
                               inside(x,y,96,712,ae_mean_width,32));
                    text(24,752,48,0,24'hffffff,
                         inside(x,y,24,752,64,32));                  // 增益
                    value_text(96,752,ae_gain_glyphs,ae_gain_first,
                               ae_gain_width,1'b0,
                               (|I_ae_nack) ? 24'hFF0000 : 24'hffffff,
                               inside(x,y,96,752,ae_gain_width,32));
                end
            end
        end
    end

    // Split the OSD/font path into two registered stages.  The first stage
    // captures the geometry result and glyph address; the second stage reads
    // the synchronous font ROM and composites it.  This removes the long
    // y_count -> glyph_address -> font RAM path from the 150 MHz pixel cycle.
    reg [23:0] bg_stage1, fg_stage1;
    reg [12:0] glyph_address_stage1;
    reg glyph_active_stage1;
    reg [4:0] column_stage1;
    reg [5:0] logo_row_stage1;
    reg [6:0] logo_column_stage1;
    reg logo_active_stage1;
    reg vs_stage1,hs_stage1,de_stage1;
    reg [23:0] bg_stage2, fg_stage2;
    reg [31:0] font_stage2;
    reg [71:0] logo_bits_s2;
    reg logo_active_s2;
    reg [6:0] logo_column_s2;
    wire logo_pixel = logo_active_s2 & logo_bits_s2[7'd71-logo_column_s2];
    reg glyph_stage2;
    reg [4:0] column_stage2;
    reg vs_stage2,hs_stage2,de_stage2;
    reg [12:0] hint_shadow_index_s1;
    reg hint_shadow_active_s1;
    reg [15:0] hint_shadow_word_s2;
    reg hint_shadow_active_s2;
    reg [3:0] hint_shadow_bit_s2;
    wire hint_shadow_pixel = hint_shadow_active_s2 & hint_shadow_word_s2[hint_shadow_bit_s2];
    // 自动曝光读数的描边掩码，和提示阴影走同一条两级流水。
    reg  [5:0] ae_lbl_idx_s1, ae_lbl_col_s1;
    reg        ae_lbl_act_s1;
    reg  [8:0] ae_dig_idx_s1;
    reg  [3:0] ae_dig_col_s1;
    reg        ae_dig_act_s1;
    reg [63:0] ae_lbl_word_s2;
    reg [15:0] ae_dig_word_s2;
    reg  [5:0] ae_lbl_col_s2;
    reg  [3:0] ae_dig_col_s2;
    reg        ae_lbl_act_s2, ae_dig_act_s2;
    reg        grid_s1, grid_s2;
    wire ae_shadow_pixel = (ae_lbl_act_s2 & ae_lbl_word_s2[6'd63-ae_lbl_col_s2])
                         | (ae_dig_act_s2 & ae_dig_word_s2[4'd15-ae_dig_col_s2]);
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            bg_stage1<=0;fg_stage1<=0;glyph_address_stage1<=0;glyph_active_stage1<=0;
            column_stage1<=0;vs_stage1<=0;hs_stage1<=0;de_stage1<=0;
            logo_row_stage1<=0;logo_column_stage1<=0;logo_active_stage1<=0;
            bg_stage2<=0;fg_stage2<=0;font_stage2<=0;glyph_stage2<=0;column_stage2<=0;
            logo_bits_s2<=0;logo_active_s2<=0;logo_column_s2<=0;
            hint_shadow_index_s1<=0;hint_shadow_active_s1<=0;
            hint_shadow_word_s2<=0;hint_shadow_active_s2<=0;hint_shadow_bit_s2<=0;
            ae_lbl_idx_s1<=0;ae_lbl_col_s1<=0;ae_lbl_act_s1<=0;
            ae_dig_idx_s1<=0;ae_dig_col_s1<=0;ae_dig_act_s1<=0;
            ae_lbl_word_s2<=0;ae_dig_word_s2<=0;
            ae_lbl_col_s2<=0;ae_dig_col_s2<=0;ae_lbl_act_s2<=0;ae_dig_act_s2<=0;
            grid_s1<=0;grid_s2<=0;
            vs_stage2<=0;hs_stage2<=0;de_stage2<=0;O_rgb<=0;O_vs<=0;O_hs<=0;O_de<=0;
        end else begin
            // Stage 1: geometry, selected glyph address and filter background.
            glyph_address_stage1<=glyph_address;column_stage1<=glyph_column;
            glyph_active_stage1<=glyph_active;
            logo_row_stage1<=logo_row_s1;logo_column_stage1<=logo_column_s1;
            logo_active_stage1<=logo_active_s1;
            hint_shadow_index_s1<=hint_shadow_index_s0;
            hint_shadow_active_s1<=hit_hint_shadow;
            ae_lbl_idx_s1<=ae_lbl_idx_s0;ae_lbl_col_s1<=ae_lbl_col_s0;
            ae_lbl_act_s1<=hit_ae_lbl_shadow;
            ae_dig_idx_s1<=ae_dig_idx_s0;ae_dig_col_s1<=ae_dig_col_s0;
            ae_dig_act_s1<=hit_ae_dig_shadow;
            grid_s1<=hit_grid;
            bg_stage1<=background_color;fg_stage1<=text_color;
            vs_stage1<=vs_d9;hs_stage1<=hs_d9;de_stage1<=de_d9;
            // Stage 2: synchronous ROM read plus delayed OSD metadata.
            font_stage2<=font_rows[glyph_address_stage1];
            logo_bits_s2<=logo_rows[logo_row_stage1];
            logo_active_s2<=logo_active_stage1;
            logo_column_s2<=logo_column_stage1;
            bg_stage2<=bg_stage1;fg_stage2<=fg_stage1;
            glyph_stage2<=glyph_active_stage1;column_stage2<=column_stage1;
            hint_shadow_word_s2<=hint_shadow_words[hint_shadow_index_s1[12:4]];
            hint_shadow_active_s2<=hint_shadow_active_s1;
            hint_shadow_bit_s2<=hint_shadow_index_s1[3:0];
            ae_lbl_word_s2<=ae_label_shadow_words[ae_lbl_idx_s1];
            ae_dig_word_s2<=ae_digit_shadow_words[ae_dig_idx_s1];
            ae_lbl_col_s2<=ae_lbl_col_s1;ae_lbl_act_s2<=ae_lbl_act_s1;
            ae_dig_col_s2<=ae_dig_col_s1;ae_dig_act_s2<=ae_dig_act_s1;
            grid_s2<=grid_s1;
            vs_stage2<=vs_stage1;hs_stage2<=hs_stage1;de_stage2<=de_stage1;
            // 三层合成，优先级：logo > 白字 > 黑阴影 > 背景。
            // 阴影只有在字模没点亮的像素上才生效，所以白字始终压在阴影上面。
            O_rgb<=de_stage2?((logo_pixel)?LOGO_INK
                   :((glyph_stage2 && font_stage2[31-column_stage2])?fg_stage2
                   :((ae_shadow_pixel||hint_shadow_pixel)?24'h000000
                   :(grid_s2?24'hffffff:bg_stage2)))):24'h000000;
            O_vs<=vs_stage2;O_hs<=hs_stage2;O_de<=de_stage2;
        end
    end
endmodule



