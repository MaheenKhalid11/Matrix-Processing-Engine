`timescale 1ns / 1ps
//======================================================================
// activation_bram.sv -- True Dual-Port Block RAM for Activations
//
// Wrapper around bram_tdp_init storing 1x256 FP32 activation vectors.
//
// Ports:
//   Port A (Host/DMA): Used to write/read activation data from top level.
//   Port B (MPU Core): Read-only port used by the Memory Controller
//                      to stream scalar activations to VPUs.
//======================================================================

module activation_bram #(
    parameter DATA_WIDTH = 32,
    parameter DEPTH      = 256,
    parameter INIT_FILE  = "activation_data.hex",
    localparam ADDR_WIDTH = $clog2(DEPTH)
)(
    input  logic                  clk,

    // Port A: External Host / DMA Write & Read Interface
    input  logic                  a_en,
    input  logic                  a_we,
    input  logic [ADDR_WIDTH-1:0] a_addr,
    input  logic [DATA_WIDTH-1:0] a_din,
    output logic [DATA_WIDTH-1:0] a_dout,

    // Port B: MPU Memory Controller Read Interface
    input  logic                  b_en,
    input  logic [ADDR_WIDTH-1:0] b_addr,
    output logic [DATA_WIDTH-1:0] b_dout
);

    // Port B write is permanently disabled (MPU core never overwrites activation BRAM)
    bram_tdp_init #(
        .DATA_WIDTH ( DATA_WIDTH ),
        .ADDR_WIDTH ( ADDR_WIDTH ),
        .INIT_FILE  ( INIT_FILE  )
    ) u_bram_tdp (
        .clk    ( clk    ),

        // Port A
        .en_a   ( a_en   ),
        .we_a   ( a_we   ),
        .addr_a ( a_addr ),
        .din_a  ( a_din  ),
        .dout_a ( a_dout ),

        // Port B
        .en_b   ( b_en   ),
        .we_b   ( 1'b0   ),     // Tied off: Read-only for MPU
        .addr_b ( b_addr ),
        .din_b  ( '0     ),     // Unused
        .dout_b ( b_dout )
    );

endmodule