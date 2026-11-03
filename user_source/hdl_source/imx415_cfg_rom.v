//=============================================================================
//  imx415_cfg_rom.v  ---  IMX415 上电配置表（只读 ROM，201 条）
//
//  !!!  本文件由脚本生成，请勿手工编辑  !!!
//       生成脚本 : sim/tools/gen_cfg_rom.py
//       数据来源 : sim/imx415_cfg_rom.expected.hex
//       原始出处 : IMX415摄像头工程总结与IIC初始化指南.md 第 5.4 节
//                  (= 米联客 MLK-CAM002-IMX415 工程 uics500reg.v, REG_SIZE=201)
//
//  数据格式 : O_data[23:0] = { addr[15:0], value[7:0] }
//  I2C 发送顺序 : 从机地址+W -> addr[15:8] -> addr[7:0] -> value
//
//  ★ 本表为修正版 594 Mbps/lane profile：SYS_MODE(0x3033) = 0x07，
//    PHY rate 与 timing.sdc 中 74.25MHz byte clock 一致。
//
//  ★ 本表已含 0x3000 = 0x01（进入待机态）。待机取消的 0x3000=0x00 /
//    0x3002=0x00 两条 **不在本表内**，由 imx415_cfg_fsm.v 在第一段写完后追加。
//
//  ★ 本表采用 2/2-line binning（3020=01, 3021=01, 3022=01, 30D9=02, 30DA=01），
//    输出行宽 1944 像素（RAW10 WC=2430），交给 ISP 链裁到 1920。
//=============================================================================
`timescale 1ns / 1ps

module imx415_cfg_rom (
    input  wire [7:0]  I_index,     // 0 .. ROM_SIZE-1
    output reg  [23:0] O_data       // { addr[15:0], value[7:0] }
);

    localparam ROM_SIZE = 201;

    always @(*) begin
        case (I_index)
            8'd0   : O_data = 24'h300001;
            8'd1   : O_data = 24'h300100;
            8'd2   : O_data = 24'h300201;
            8'd3   : O_data = 24'h300300;
            8'd4   : O_data = 24'h30085D;
            8'd5   : O_data = 24'h300900;
            8'd6   : O_data = 24'h300A42;
            8'd7   : O_data = 24'h300BA0;
            8'd8   : O_data = 24'h301C00;
            8'd9   : O_data = 24'h301D08;
            8'd10  : O_data = 24'h302001;
            8'd11  : O_data = 24'h302101;
            8'd12  : O_data = 24'h302201;
            8'd13  : O_data = 24'h302301;
            8'd14  : O_data = 24'h3024EB;
            8'd15  : O_data = 24'h302508;
            8'd16  : O_data = 24'h302600;
            8'd17  : O_data = 24'h30281E;
            8'd18  : O_data = 24'h302902;
            8'd19  : O_data = 24'h302C00;
            8'd20  : O_data = 24'h302D00;
            8'd21  : O_data = 24'h303000;
            8'd22  : O_data = 24'h303100;
            8'd23  : O_data = 24'h303200;
            8'd24  : O_data = 24'h303307;
            8'd25  : O_data = 24'h304000;
            8'd26  : O_data = 24'h304100;
            8'd27  : O_data = 24'h304218;
            8'd28  : O_data = 24'h30430F;
            8'd29  : O_data = 24'h304400;
            8'd30  : O_data = 24'h304500;
            8'd31  : O_data = 24'h304620;
            8'd32  : O_data = 24'h304711;
            8'd33  : O_data = 24'h305008;
            8'd34  : O_data = 24'h305100;
            8'd35  : O_data = 24'h305200;
            8'd36  : O_data = 24'h305419;
            8'd37  : O_data = 24'h305500;
            8'd38  : O_data = 24'h305600;
            8'd39  : O_data = 24'h30583E;
            8'd40  : O_data = 24'h305900;
            8'd41  : O_data = 24'h305A00;
            8'd42  : O_data = 24'h305C66;
            8'd43  : O_data = 24'h305D00;
            8'd44  : O_data = 24'h305E00;
            8'd45  : O_data = 24'h306025;
            8'd46  : O_data = 24'h306100;
            8'd47  : O_data = 24'h306200;
            8'd48  : O_data = 24'h30644A;
            8'd49  : O_data = 24'h306500;
            8'd50  : O_data = 24'h306600;
            8'd51  : O_data = 24'h30902A;
            8'd52  : O_data = 24'h309100;
            8'd53  : O_data = 24'h30922A;
            8'd54  : O_data = 24'h309300;
            8'd55  : O_data = 24'h30942A;
            8'd56  : O_data = 24'h309500;
            8'd57  : O_data = 24'h30962A;
            8'd58  : O_data = 24'h309700;
            8'd59  : O_data = 24'h30C02A;
            8'd60  : O_data = 24'h30C100;
            8'd61  : O_data = 24'h30CC00;
            8'd62  : O_data = 24'h30CD00;
            8'd63  : O_data = 24'h30CF00;
            8'd64  : O_data = 24'h30D902;
            8'd65  : O_data = 24'h30DA01;
            8'd66  : O_data = 24'h30E232;
            8'd67  : O_data = 24'h30E300;
            8'd68  : O_data = 24'h311500;
            8'd69  : O_data = 24'h311623;
            8'd70  : O_data = 24'h311884;
            8'd71  : O_data = 24'h311900;
            8'd72  : O_data = 24'h311AE7;
            8'd73  : O_data = 24'h311B00;
            8'd74  : O_data = 24'h311E23;
            8'd75  : O_data = 24'h326001;
            8'd76  : O_data = 24'h32C801;
            8'd77  : O_data = 24'h32D421;
            8'd78  : O_data = 24'h32ECA1;
            8'd79  : O_data = 24'h344C2B;
            8'd80  : O_data = 24'h344D01;
            8'd81  : O_data = 24'h344EED;
            8'd82  : O_data = 24'h344F01;
            8'd83  : O_data = 24'h3450F6;
            8'd84  : O_data = 24'h345102;
            8'd85  : O_data = 24'h34527F;
            8'd86  : O_data = 24'h345303;
            8'd87  : O_data = 24'h358A04;
            8'd88  : O_data = 24'h35A102;
            8'd89  : O_data = 24'h35EC27;
            8'd90  : O_data = 24'h35EE8D;
            8'd91  : O_data = 24'h35F08D;
            8'd92  : O_data = 24'h35F229;
            8'd93  : O_data = 24'h36BC0C;
            8'd94  : O_data = 24'h36CC53;
            8'd95  : O_data = 24'h36CD00;
            8'd96  : O_data = 24'h36CE3C;
            8'd97  : O_data = 24'h36D08C;
            8'd98  : O_data = 24'h36D100;
            8'd99  : O_data = 24'h36D271;
            8'd100 : O_data = 24'h36D43C;
            8'd101 : O_data = 24'h36D653;
            8'd102 : O_data = 24'h36D700;
            8'd103 : O_data = 24'h36D871;
            8'd104 : O_data = 24'h36DA8C;
            8'd105 : O_data = 24'h36DB00;
            8'd106 : O_data = 24'h370100;
            8'd107 : O_data = 24'h372000;
            8'd108 : O_data = 24'h372402;
            8'd109 : O_data = 24'h372602;
            8'd110 : O_data = 24'h373202;
            8'd111 : O_data = 24'h373403;
            8'd112 : O_data = 24'h373603;
            8'd113 : O_data = 24'h374203;
            8'd114 : O_data = 24'h3862E0;
            8'd115 : O_data = 24'h38CC30;
            8'd116 : O_data = 24'h38CD2F;
            8'd117 : O_data = 24'h395C0C;
            8'd118 : O_data = 24'h39A407;
            8'd119 : O_data = 24'h39A832;
            8'd120 : O_data = 24'h39AA32;
            8'd121 : O_data = 24'h39AC32;
            8'd122 : O_data = 24'h39AE32;
            8'd123 : O_data = 24'h39B032;
            8'd124 : O_data = 24'h39B22F;
            8'd125 : O_data = 24'h39B42D;
            8'd126 : O_data = 24'h39B628;
            8'd127 : O_data = 24'h39B830;
            8'd128 : O_data = 24'h39BA30;
            8'd129 : O_data = 24'h39BC30;
            8'd130 : O_data = 24'h39BE30;
            8'd131 : O_data = 24'h39C030;
            8'd132 : O_data = 24'h39C22E;
            8'd133 : O_data = 24'h39C42B;
            8'd134 : O_data = 24'h39C625;
            8'd135 : O_data = 24'h3A42D1;
            8'd136 : O_data = 24'h3A4C77;
            8'd137 : O_data = 24'h3AE002;
            8'd138 : O_data = 24'h3AEC0C;
            8'd139 : O_data = 24'h3B002E;
            8'd140 : O_data = 24'h3B0629;
            8'd141 : O_data = 24'h3B9825;
            8'd142 : O_data = 24'h3B9921;
            8'd143 : O_data = 24'h3B9B13;
            8'd144 : O_data = 24'h3B9C13;
            8'd145 : O_data = 24'h3B9D13;
            8'd146 : O_data = 24'h3B9E13;
            8'd147 : O_data = 24'h3BA100;
            8'd148 : O_data = 24'h3BA206;
            8'd149 : O_data = 24'h3BA30B;
            8'd150 : O_data = 24'h3BA410;
            8'd151 : O_data = 24'h3BA514;
            8'd152 : O_data = 24'h3BA618;
            8'd153 : O_data = 24'h3BA71A;
            8'd154 : O_data = 24'h3BA81A;
            8'd155 : O_data = 24'h3BA91A;
            8'd156 : O_data = 24'h3BACED;
            8'd157 : O_data = 24'h3BAD01;
            8'd158 : O_data = 24'h3BAEF6;
            8'd159 : O_data = 24'h3BAF02;
            8'd160 : O_data = 24'h3BB0A2;
            8'd161 : O_data = 24'h3BB103;
            8'd162 : O_data = 24'h3BB2E0;
            8'd163 : O_data = 24'h3BB303;
            8'd164 : O_data = 24'h3BB4E0;
            8'd165 : O_data = 24'h3BB503;
            8'd166 : O_data = 24'h3BB6E0;
            8'd167 : O_data = 24'h3BB703;
            8'd168 : O_data = 24'h3BB8E0;
            8'd169 : O_data = 24'h3BBAE0;
            8'd170 : O_data = 24'h3BBCDA;
            8'd171 : O_data = 24'h3BBE88;
            8'd172 : O_data = 24'h3BC044;
            8'd173 : O_data = 24'h3BC27B;
            8'd174 : O_data = 24'h3BC4A2;
            8'd175 : O_data = 24'h3BC8BD;
            8'd176 : O_data = 24'h3BCABD;
            8'd177 : O_data = 24'h400010;
            8'd178 : O_data = 24'h400103;
            8'd179 : O_data = 24'h4004C0;
            8'd180 : O_data = 24'h400506;
            8'd181 : O_data = 24'h400C00;
            8'd182 : O_data = 24'h4018B7;
            8'd183 : O_data = 24'h401900;
            8'd184 : O_data = 24'h401A67;
            8'd185 : O_data = 24'h401B00;
            8'd186 : O_data = 24'h401C6F;
            8'd187 : O_data = 24'h401D00;
            8'd188 : O_data = 24'h401EDF;
            8'd189 : O_data = 24'h401F01;
            8'd190 : O_data = 24'h40206F;
            8'd191 : O_data = 24'h402100;
            8'd192 : O_data = 24'h4022CF;
            8'd193 : O_data = 24'h402300;
            8'd194 : O_data = 24'h40246F;
            8'd195 : O_data = 24'h402500;
            8'd196 : O_data = 24'h4026B7;
            8'd197 : O_data = 24'h402700;
            8'd198 : O_data = 24'h40285F;
            8'd199 : O_data = 24'h402900;
            8'd200 : O_data = 24'h407401;
            default: O_data = 24'h000000;
        endcase
    end

endmodule
