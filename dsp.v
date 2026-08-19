`timescale 1ns/1ps
module dspfp32_pe #(
    parameter IS_FIRST = 0
) (
    input  wire        clk,
    input  wire         rst,

    input  wire         ce_a,
    input  wire         ce_b,
    input  wire         ce_pipe,

    input  wire [31:0]  a_row,
    input  wire [31:0]  b_weight,
    input  wire [31:0]  pcin,
    output wire [31:0]  pcout
);

    localparam [6:0] FPOPMODE_FIRST = 7'h01; // P = 0 + M
    localparam [6:0] FPOPMODE_CHAIN = 7'h1D; // P = PCIN + M
    wire [6:0] fpopmode_c = IS_FIRST ? FPOPMODE_FIRST : FPOPMODE_CHAIN;

    wire         a_sign = a_row[31];
    wire [7:0]   a_exp  = a_row[30:23];
    wire [22:0]  a_man  = a_row[22:0];

    wire         b_sign = b_weight[31];
    wire [7:0]   b_exp  = b_weight[30:23];
    wire [22:0]  b_man  = b_weight[22:0];

    DSPFP32 #(
        .A_FPTYPE("B32"),
        .A_INPUT("DIRECT"),
        .BCASCSEL("B"),
        .B_D_FPTYPE("B32"),
        .B_INPUT("DIRECT"),
        .PCOUTSEL("FPA"),
        .USE_MULT("MULTIPLY"),
        .IS_CLK_INVERTED(1'b0),
        .IS_FPINMODE_INVERTED(1'b0),
        .IS_FPOPMODE_INVERTED(7'b0000000),
        .IS_RSTA_INVERTED(1'b0),
        .IS_RSTB_INVERTED(1'b0),
        .IS_RSTC_INVERTED(1'b0),
        .IS_RSTD_INVERTED(1'b0),
        .IS_RSTFPA_INVERTED(1'b0),
        .IS_RSTFPINMODE_INVERTED(1'b0),
        .IS_RSTFPMPIPE_INVERTED(1'b0),
        .IS_RSTFPM_INVERTED(1'b0),
        .IS_RSTFPOPMODE_INVERTED(1'b0),
        .ACASCREG(1),   
        .AREG(1),       
        .FPA_PREG(1),
        .FPBREG(1),
        .FPCREG(3),
        .FPDREG(1),
        .FPMPIPEREG(1),
        .FPM_PREG(1),
        .FPOPMREG(3),
        .INMODEREG(1),
        .RESET_MODE("SYNC")
    ) u_dspfp32 (
        .ACOUT_EXP(), .ACOUT_MAN(), .ACOUT_SIGN(),
        .BCOUT_EXP(), .BCOUT_MAN(), .BCOUT_SIGN(),
        .PCOUT(pcout),

        .FPA_INVALID(),
        .FPA_OUT(),
        .FPA_OVERFLOW(),
        .FPA_UNDERFLOW(),
        .FPM_INVALID(),
        .FPM_OUT(),
        .FPM_OVERFLOW(),
        .FPM_UNDERFLOW(),

        .ACIN_EXP(8'b0), .ACIN_MAN(23'b0), .ACIN_SIGN(1'b0),
        .BCIN_EXP(8'b0), .BCIN_MAN(23'b0), .BCIN_SIGN(1'b0),
        .PCIN(pcin),

        .CLK(clk),
        .FPINMODE(1'b1),            
        .FPOPMODE(fpopmode_c),

        .A_EXP(a_exp), .A_MAN(a_man), .A_SIGN(a_sign),
        .B_EXP(b_exp), .B_MAN(b_man), .B_SIGN(b_sign),
        .C(32'b0),
        .D_EXP(8'b0), .D_MAN(23'b0), .D_SIGN(1'b0),

        .ASYNC_RST(1'b0),
        .CEA1(ce_a), .CEA2(ce_a),
        .CEB(ce_b),
        .CEC(1'b0),
        .CED(1'b0),
        .CEFPA(ce_pipe),
        .CEFPINMODE(ce_pipe),
        .CEFPM(ce_pipe),
        .CEFPMPIPE(ce_pipe),
        .CEFPOPMODE(ce_pipe),
        .RSTA(rst), .RSTB(rst), .RSTC(rst), .RSTD(rst),
        .RSTFPA(rst), .RSTFPINMODE(rst), .RSTFPM(rst),
        .RSTFPMPIPE(rst), .RSTFPOPMODE(rst)
    );

endmodule