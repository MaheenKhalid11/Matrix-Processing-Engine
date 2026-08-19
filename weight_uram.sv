`timescale 1ns / 1ps
//======================================================================
// weight_uram.sv -- UltraRAM Wrapper for Matrix Weights
//
// Wraps uram_tdp_init to store 256x256 FP32 weight matrix tiles.
//
// Dimensions:
//   - Depth: NUM_CGS * DEPTH (16 * 256 = 4096 words)
//   - Width: CG_WIDTH = NUM_VPUS * DATA_WIDTH (512 bits per row)
//
// Ports:
//   Port A (Host/DMA): Used by DMA/TB to write weight matrix tiles.
//   Port B (MPU Core): Read-only port used by the Memory Controller
//                      to stream 512-bit CG rows to VPUs.
//======================================================================

module weight_uram #(
    parameter DATA_WIDTH  = 32,
    parameter NUM_VPUS    = 16,
    parameter DEPTH       = 256,
    parameter INIT_FILE   = "weight_data.hex",
    localparam NUM_CGS    = DEPTH / NUM_VPUS,             // 16
    localparam CG_WIDTH   = NUM_VPUS * DATA_WIDTH,        // 512 bits
    localparam ADDR_WIDTH = $clog2(NUM_CGS * DEPTH)       // 12 bits (4096 entries)
)(
    input  logic                  clk,

    // Port A: External Host / DMA Write & Read Interface
    input  logic                  a_en,
    input  logic                  a_we,
    input  logic [ADDR_WIDTH-1:0] a_addr,
    input  logic [CG_WIDTH-1:0]   a_din,
    output logic [CG_WIDTH-1:0]   a_dout,

    // Port B: MPU Memory Controller Read Interface
    input  logic                  b_en,
    input  logic [ADDR_WIDTH-1:0] b_addr,
    output logic [CG_WIDTH-1:0]   b_dout
);

    // Instantiate core TDP URAM configured for 512-bit wide, 4096-depth matrix storage
    uram_tdp_init #(
        .DATA_WIDTH ( CG_WIDTH   ),
        .ADDR_WIDTH ( ADDR_WIDTH ),
        .INIT_FILE  ( INIT_FILE  )
    ) u_uram_tdp (
        .clk    ( clk    ),

        // Port A (Host/DMA Write & Read)
        .en_a   ( a_en   ),
        .we_a   ( a_we   ),
        .addr_a ( a_addr ),
        .din_a  ( a_din  ),
        .dout_a ( a_dout ),

        // Port B (MPU Read Path - write path tied off)
        .en_b   ( b_en   ),
        .we_b   ( 1'b0   ),     // Tied off: Read-only for MPU
        .addr_b ( b_addr ),
        .din_b  ( '0     ),     // Unused
        .dout_b ( b_dout )
    );

endmodule