`timescale 1ns / 1ps

module bram_tdp_init #(
    parameter DATA_WIDTH = 32,
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

    // RAM declaration with synthesis attribute on the same line
    (* ram_style = "block" *) logic [DATA_WIDTH-1:0] ram [0:(1<<ADDR_WIDTH)-1];

    // File Initialization
    initial begin
        if (INIT_FILE != "") begin
            $readmemh(INIT_FILE, ram);
        end
    end

    // Port A - Read-first behavior
    always_ff @(posedge clk) begin
        if (en_a) begin
            dout_a <= ram[addr_a];
            if (we_a) begin
                ram[addr_a] <= din_a;
            end
        end
    end

    // Port B - Read-first behavior
    always_ff @(posedge clk) begin
        if (en_b) begin
            dout_b <= ram[addr_b];
            if (we_b) begin
                ram[addr_b] <= din_b;
            end
        end
    end

endmodule