`timescale 1ns / 1ps
//======================================================================
// output_bram.sv -- True Dual-Port Block RAM for Output Vector
//
// Wrapper around bram_tdp_init storing completed Column Group results.
//
// Dimensions:
//   - Depth: NUM_CGS (16)
//   - Width: CG_WIDTH = NUM_VPUS * DATA_WIDTH (512 bits)
//
// Ports:
//   Port A (MPU Core): Write-only port. Latches all 16 VPU FP32 outputs
//                      in a single cycle when write_output pulses.
//   Port B (Host/DMA): Read-only port used by the system host or 
//                      testbench to harvest completed inference data.
//======================================================================

module output_bram #(
    parameter DATA_WIDTH  = 32,
    parameter NUM_VPUS    = 16,
    parameter DEPTH       = 256,
    parameter INIT_FILE   = "",
    localparam NUM_CGS    = DEPTH / NUM_VPUS,             // 16
    localparam CG_WIDTH   = NUM_VPUS * DATA_WIDTH,        // 512 bits
    localparam ADDR_WIDTH = $clog2(NUM_CGS)               // 4 bits
)(
    input  logic                  clk,

    // Port A: MPU Core Write Interface (512-bit wide)
    input  logic                  a_en,
    input  logic                  a_we,
    input  logic [ADDR_WIDTH-1:0] a_addr,
    input  logic [CG_WIDTH-1:0]   a_din,
    output logic [CG_WIDTH-1:0]   a_dout,

    // Port B: External Host / DMA Read Interface
    input  logic                  b_en,
    input  logic [ADDR_WIDTH-1:0] b_addr,
    output logic [CG_WIDTH-1:0]   b_dout
);

    // Instantiate core TDP BRAM configured for 512-bit wide transactions
    bram_tdp_init #(
        .DATA_WIDTH ( CG_WIDTH   ),
        .ADDR_WIDTH ( ADDR_WIDTH ),
        .INIT_FILE  ( INIT_FILE  )
    ) u_bram_tdp (
        .clk    ( clk    ),

        // Port A (MPU Write Path)
        .en_a   ( a_en   ),
        .we_a   ( a_we   ),
        .addr_a ( a_addr ),
        .din_a  ( a_din  ),
        .dout_a ( a_dout ),

        // Port B (Host Read Path - write path tied off)
        .en_b   ( b_en   ),
        .we_b   ( 1'b0   ),
        .addr_b ( b_addr ),
        .din_b  ( '0     ),
        .dout_b ( b_dout )
    );

endmodule