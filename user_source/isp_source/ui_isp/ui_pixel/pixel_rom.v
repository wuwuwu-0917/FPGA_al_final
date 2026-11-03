//=============================================================================
//  pixel_rom.v --- 像素滤镜的 map/palette 只读表（PH1_LOGIC_ERAM + INIT_FILE）
//
//  文件格式：.dath（每行一个十六进制数），由 xiangsu/luts/*.mem 直接拷贝。
//  PH1_LOGIC_ERAM 的 INIT_FILE 检查最后 4 个字符 == "dath"。
//
//  读延迟：1 拍（REGMODE_A="NOREG"，同步读）。
//=============================================================================
`timescale 1ns/1ps

module pixel_rom #(
    parameter integer DATA_WIDTH = 8,
    parameter integer ADDR_WIDTH = 12,
    parameter integer DEPTH      = 4096,
    parameter         INIT_FILE  = "pixel_map.dath"
)(
    input  wire                  I_clk,
    input  wire [ADDR_WIDTH-1:0] I_addr,
    output wire [DATA_WIDTH-1:0] O_data
);

    PH1_LOGIC_ERAM #(
        .DATA_WIDTH_A        (DATA_WIDTH        ),
        .DATA_WIDTH_B        (DATA_WIDTH        ),
        .ADDR_WIDTH_A        (ADDR_WIDTH        ),
        .ADDR_WIDTH_B        (ADDR_WIDTH        ),
        .DATA_DEPTH_A        (DEPTH             ),
        .DATA_DEPTH_B        (DEPTH             ),
        .MODE                ("SP"              ),
        .REGMODE_A           ("NOREG"           ),
        .REGMODE_B           ("NOREG"           ),
        .WRITEMODE_A         ("NORMAL"          ),
        .WRITEMODE_B         ("NORMAL"          ),
        .RESETMODE_A         ("ASYNC"           ),
        .RESETMODE_B         ("ASYNC"           ),
        .IMPLEMENT           ("20K(FAST)"       ),
        .ECC_ENCODE          ("DISABLE"         ),
        .ECC_DECODE          ("DISABLE"         ),
        // SP 模式只允许 ASYNC（厂商模型会检查）；读仍是同步时钟沿
        .CLKMODE             ("ASYNC"           ),
        .SSROVERCE           ("DISABLE"         ),
        .OREGSET_A           ("RESET"           ),
        .OREGSET_B           ("RESET"           ),
        .ASYNC_RESET_RELEASE_A("ASYNC"          ),
        .ASYNC_RESET_RELEASE_B("ASYNC"          ),
        .INIT_FILE           (INIT_FILE         ),
        .FILL_ALL            ("NONE"            )
    ) inst (
        .dia                ({DATA_WIDTH{1'b0}} ),
        .dib                ({DATA_WIDTH{1'b0}} ),
        .addra              (I_addr             ),
        .addrb              ({ADDR_WIDTH{1'b0}} ),
        .cea                (1'b1               ),
        .ceb                (1'b0               ),
        .ocea               (1'b0               ),
        .oceb               (1'b0               ),
        .clka               (I_clk              ),
        .clkb               (I_clk              ),
        .wea                (1'b0               ),
        .web                (1'b0               ),
        .bea                (1'b0               ),
        .beb                (1'b0               ),
        .rsta               (1'b0               ),
        .rstb               (1'b0               ),
        .doa                (O_data             ),
        .dob                (                   ),
        .ecc_sbiterr        (                   ),
        .ecc_dbiterr        (                   ),
        .ecc_sbiterrinj     (1'b0               ),
        .ecc_dbiterrinj     (1'b0               )
    );

endmodule
