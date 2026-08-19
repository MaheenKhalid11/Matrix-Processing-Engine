`timescale 1ns / 1ps
//======================================================================
// accum_adder16  --  DATAPATH ONLY
//
// A 16-lane FP32 adder bank built from the SAME verified dspfp32_pe
// primitive the multiply cascade uses (dsp.v) -- zero new floating
// point arithmetic. Each lane computes P = PCIN + M with A tied to
// the constant 1.0f, so M = 1.0f * B = B, giving P = PCIN + B: a pure
// elementwise add, reusing hardware already proven correct in
// dsp_cascade.sv.
//
// Unlike dsp_cascade's 16 chained DSPs (PCOUT of one feeds PCIN of
// the next), the 16 lanes here are fully INDEPENDENT -- lane i adds
// a_vec's i-th column to b_vec's i-th column, nothing crosses lanes.
// That independence is what lets the controller (vecadd_fsm) pipeline
// rows back-to-back instead of serializing like a reduction chain.
//======================================================================
module accum_adder16 (
    input  logic         clk,
    input  logic         rst,

    input  logic         ce_a,
    input  logic         ce_b,
    input  logic         ce_pipe,

    input  logic [511:0] a_vec,    // pcin operand, 16 x FP32 (running value)
    input  logic [511:0] b_vec,    // operand to add in, 16 x FP32
    output logic [511:0] sum_vec   // a_vec + b_vec, elementwise FP32
);

    localparam [31:0] ONE_F32 = 32'h3F80_0000;

    genvar i;
    generate
        for (i = 0; i < 16; i = i + 1) begin : ADD_LANE
            dspfp32_pe #(
                .IS_FIRST(0)   // chain mode: P = PCIN + M, same as dsp_cascade
            ) PE (
                .clk      (clk),
                .rst      (rst),
                .ce_a     (ce_a),
                .ce_b     (ce_b),
                .ce_pipe  (ce_pipe),
                .a_row    (ONE_F32),
                .b_weight (b_vec[(i+1)*32-1 -: 32]),
                .pcin     (a_vec[(i+1)*32-1 -: 32]),
                .pcout    (sum_vec[(i+1)*32-1 -: 32])
            );
        end
    endgenerate

endmodule
