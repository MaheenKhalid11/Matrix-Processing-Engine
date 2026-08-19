`timescale 1ns/1ps
//======================================================================
// vpu  --  TOP LEVEL
//
// Just wires vpu_fsm (control) to dsp_cascade (datapath). No behavior
// of its own.
//======================================================================
module vpu #(
    parameter K            = 16,   // physical cascade width (# DSPs, fixed hardware)
    parameter DEPTH        = 256,  // total reduction depth for one dot product
    // Cycles to wait per pass for the K-wide DSPFP32 cascade to settle.
    // Derived, not simulator-measured: a single isolated DSPFP32 MAC
    // (PCIN already valid) takes 4 cycles -- AREG/FPBREG load (1) +
    // FPMPIPEREG/FPM_PREG multiply (2) + FPA_PREG add (1). In the
    // cascade all K multiplies run in parallel (PCIN is wired
    // combinationally between stages in dsp_cascade.sv, no extra
    // register per hop), so only the final 1-cycle add stage ripples
    // serially across the remaining K-1 DSPs: latency = 4 + (K-1).
    // For K=16 that is 19; this leaves +5 cycles of margin since the
    // derivation isn't simulator-verified. Tighten/verify with tb_vpu
    // (see mpu_fsm.sv header) before trusting this on real hardware.
    parameter PIPE_LATENCY = 24
)(
    input  logic                     clk,
    input  logic                     rst,
    input  logic                     start,            // pulse: begin one full-depth dot product
    input  logic [DEPTH*32-1:0]      activation_full,  // 1 x DEPTH, broadcast
    input  logic [DEPTH*32-1:0]      weight_full,       // DEPTH x 1, this VPU's column
    output logic                     busy,
    output logic                     valid,             // pulses high for 1 cycle when dot_product is final
    output logic [31:0]              dot_product
);

    // control <-> datapath wiring
    logic                              ce_a, ce_b, ce_pipe;
    logic [31:0]                       acc_reg;
    logic [$clog2(DEPTH/K)-1:0]        pass_cnt;
    logic [31:0]                       chain_out;

    vpu_fsm #(
        .DEPTH        (DEPTH),
        .K            (K),
        .PIPE_LATENCY (PIPE_LATENCY)
    ) u_fsm (
        .clk         (clk),
        .rst         (rst),
        .start       (start),
        .chain_out   (chain_out),
        .ce_a        (ce_a),
        .ce_b        (ce_b),
        .ce_pipe     (ce_pipe),
        .busy        (busy),
        .valid       (valid),
        .acc_reg     (acc_reg),
        .pass_cnt    (pass_cnt),
        .dot_product (dot_product)
    );

    dsp_cascade #(
        .K     (K),
        .DEPTH (DEPTH)
    ) u_datapath (
        .clk             (clk),
        .rst             (rst),
        .ce_a            (ce_a),
        .ce_b            (ce_b),
        .ce_pipe         (ce_pipe),
        .pass_cnt        (pass_cnt),
        .acc_reg         (acc_reg),
        .activation_full (activation_full),
        .weight_full     (weight_full),
        .chain_out       (chain_out)
    );

endmodule