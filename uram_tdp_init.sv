`timescale 1ns / 1ps
//======================================================================
// uram_tdp_init.sv -- Generic True Dual-Port UltraRAM Primitive
//
// Vendor-agnostic TDP UltraRAM module using Xilinx "ultra" style inference.
// Supports $readmemh initialization for simulation environments.
//======================================================================

module uram_tdp_init #(
    parameter DATA_WIDTH = 512,
    parameter ADDR_WIDTH = 12,
    parameter INIT_FILE  = ""
)(
    input  logic                  clk,

    // Port A Interface
    input  logic                  en_a,
    input  logic                  we_a,
    input  logic [ADDR_WIDTH-1:0] addr_a,
    input  logic [DATA_WIDTH-1:0] din_a,
    output logic [DATA_WIDTH-1:0] dout_a,

    // Port B Interface
    input  logic                  en_b,
    input  logic                  we_b,
    input  logic [ADDR_WIDTH-1:0] addr_b,
    input  logic [DATA_WIDTH-1:0] din_b,
    output logic [DATA_WIDTH-1:0] dout_b
);

    // Xilinx synthesis attribute to force UltraRAM implementation
    (* ram_style = "ultra" *) logic [DATA_WIDTH-1:0] uram [0:(1<<ADDR_WIDTH)-1];

    // Readmemh initialization for simulation only
`ifndef SYNTHESIS
    initial begin
        if (INIT_FILE != "") begin
            $readmemh(INIT_FILE, uram);
        end
    end
`endif

    // Port A - Read-first behavior
    always_ff @(posedge clk) begin
        if (en_a) begin
            dout_a <= uram[addr_a];
            if (we_a) begin
                uram[addr_a] <= din_a;
            end
        end
    end

    // Port B - Read-first behavior
    always_ff @(posedge clk) begin
        if (en_b) begin
            dout_b <= uram[addr_b];
            if (we_b) begin
                uram[addr_b] <= din_b;
            end
        end
    end

endmodule