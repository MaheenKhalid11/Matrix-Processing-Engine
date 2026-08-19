`timescale 1ns/1ps
//======================================================================
// dsp_cascade  --  DATAPATH ONLY
//
// A K-wide DSPFP32 cascade that reduces one K-wide slice of the
// activation/weight vectors per LOAD->RUN->CAPTURE pass. It has no
// knowledge of control state: it muxes pass_cnt into the bus-slice
// index, muxes acc_reg into the head of the chain, and forwards the
// tail (chain_out) back out. All sequencing lives in vpu_fsm.
//======================================================================
module dsp_cascade #(
    parameter K     = 16,   // physical cascade width (# DSPs, fixed hardware)
    parameter DEPTH = 256   // total reduction depth for one dot product
)(
    input  logic                        clk,
    input  logic                        rst,

    // control inputs, driven by vpu_fsm
    input  logic                        ce_a,
    input  logic                        ce_b,
    input  logic                        ce_pipe,
    input  logic [$clog2(DEPTH/K)-1:0]  pass_cnt,
    input  logic [31:0]                 acc_reg,   // running total, fed back from FSM

    // data inputs: full-depth buses from outside the VPU
    input  logic [DEPTH*32-1:0]         activation_full,
    input  logic [DEPTH*32-1:0]         weight_full,

    // datapath result, consumed by vpu_fsm
    output logic [31:0]                 chain_out
);

    // Current pass's K-wide slice, sliced out of the full DEPTH-wide buses.
    wire [K*32-1:0] act_slice = activation_full[pass_cnt*K*32 +: K*32];
    wire [K*32-1:0] wt_slice  = weight_full[pass_cnt*K*32 +: K*32];

    // pc[0] is the running partial sum fed into the head of the chain:
    // 0 on the first real pass of a dot product, held acc_reg otherwise.
    // (During the throwaway priming pass pass_cnt is still 0, so acc_in
    // is 0 there too -- harmless, since that result gets discarded by
    // the FSM regardless.)
    wire [31:0] acc_in = (pass_cnt == 0) ? 32'b0 : acc_reg;

    wire [31:0] pc [0:K];
    assign pc[0] = acc_in;

    genvar i;
    generate
        for (i = 0; i < K; i = i + 1) begin : DSP_CHAIN
            dspfp32_pe #(
                .IS_FIRST(0)   // always chain mode -- pc[0] mux above replaces IS_FIRST's job
            ) PE (
                .clk      (clk),
                .rst      (rst),
                .ce_a     (ce_a),
                .ce_b     (ce_b),
                .ce_pipe  (ce_pipe),
                .a_row    (act_slice[(i+1)*32-1 -: 32]),
                .b_weight (wt_slice[(i+1)*32-1 -: 32]),
                .pcin     (pc[i]),
                .pcout    (pc[i+1])
            );
        end
    endgenerate

    assign chain_out = pc[K];

endmodule