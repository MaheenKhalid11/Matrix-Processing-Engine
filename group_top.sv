`timescale 1ns / 1ps
//======================================================================
// group_top.sv -- structural top-level wrapper for a Group.
//
// Mirrors mpu_top.sv's own role one level up:
//   - group_fsm      : sole command authority (control only)
//   - group_datapath : subordinate data mover -- owns 4x unmodified
//                       mpu_top instances, their per-MPU chunk stores,
//                       the shared self-accumulate/final-combine adder,
//                       and the Group's own output_bram.
//
// A Group covers a fixed 256-column slice of the final output across
// the full 4096-deep reduction: NUM_MPUS=4 MPUs, each responsible for
// NUM_CHUNKS=4 of the 16 total 256-deep chunks (4x256=1024 depth per
// MPU), self-accumulated per MPU then combined 4-way at the end.
//======================================================================

module group_top #(
    parameter DATA_WIDTH   = 32,
    parameter NUM_VPUS     = 16,
    parameter MPU_DEPTH    = 256,
    parameter K            = 16,
    parameter PIPE_LATENCY = 24,
    parameter NUM_MPUS     = 4,
    parameter NUM_CHUNKS   = 4,
    parameter ADD_LATENCY  = 12,

    localparam NUM_CGS          = MPU_DEPTH / NUM_VPUS,                       // 16
    localparam CG_WIDTH         = NUM_VPUS * DATA_WIDTH,                      // 512 bits
    localparam OUT_ADDR_W       = $clog2(NUM_CGS),                           // 4 bits
    localparam CHUNK_BITS       = $clog2(NUM_CHUNKS),                        // 2 bits
    localparam ACT_CHUNK_ADDR_W = $clog2(NUM_CHUNKS * MPU_DEPTH),            // 10 bits
    localparam WGT_CHUNK_ADDR_W = $clog2(NUM_CHUNKS * NUM_CGS * MPU_DEPTH)   // 14 bits
)(
    input  logic clk,
    input  logic rst,

    // Host system control interface
    input  logic start,
    output logic busy,
    output logic valid,

    // Host DMA ports -- per-MPU chunk-store preload (Port A). Each
    // MPU's full NUM_CHUNKS-chunk activation + weight allocation is
    // loaded once, before start, from HBM (or a testbench, in sim).
    input  logic                        grp_act_en   [0:NUM_MPUS-1],
    input  logic                        grp_act_we   [0:NUM_MPUS-1],
    input  logic [ACT_CHUNK_ADDR_W-1:0] grp_act_addr [0:NUM_MPUS-1],
    input  logic [DATA_WIDTH-1:0]       grp_act_din  [0:NUM_MPUS-1],

    input  logic                        grp_wgt_en   [0:NUM_MPUS-1],
    input  logic                        grp_wgt_we   [0:NUM_MPUS-1],
    input  logic [WGT_CHUNK_ADDR_W-1:0] grp_wgt_addr [0:NUM_MPUS-1],
    input  logic [CG_WIDTH-1:0]         grp_wgt_din  [0:NUM_MPUS-1],

    // Host DMA port -- Group's final combined 1x256 output (Port B)
    input  logic                  grp_out_en,
    input  logic [OUT_ADDR_W-1:0] grp_out_addr,
    output logic [CG_WIDTH-1:0]   grp_out_dout
);

    //------------------------------------------------------------
    // FSM <-> datapath handshake wires
    //------------------------------------------------------------
    logic                  fsm_load_chunk;
    logic [CHUNK_BITS-1:0] fsm_chunk_idx;
    logic                  dp_all_chunks_loaded;

    logic fsm_mpu_start;
    logic dp_all_mpu_valid;

    logic fsm_do_readout;
    logic dp_all_readout_done;

    logic       fsm_vecadd_start;
    logic [2:0] fsm_operand_sel;
    logic       dp_vecadd_done;

    logic       fsm_accum_clear;

    //------------------------------------------------------------
    // 1. group_fsm -- sole command authority
    //------------------------------------------------------------
    group_fsm #(
        .NUM_MPUS   ( NUM_MPUS   ),
        .NUM_CHUNKS ( NUM_CHUNKS )
    ) u_group_fsm (
        .clk               ( clk                   ),
        .rst               ( rst                   ),
        .start             ( start                 ),
        .busy              ( busy                  ),
        .valid             ( valid                 ),
        .load_chunk        ( fsm_load_chunk        ),
        .chunk_idx         ( fsm_chunk_idx         ),
        .all_chunks_loaded ( dp_all_chunks_loaded  ),
        .mpu_start         ( fsm_mpu_start         ),
        .all_mpu_valid     ( dp_all_mpu_valid      ),
        .do_readout        ( fsm_do_readout        ),
        .all_readout_done  ( dp_all_readout_done   ),
        .vecadd_start      ( fsm_vecadd_start      ),
        .operand_sel       ( fsm_operand_sel       ),
        .vecadd_done       ( dp_vecadd_done        ),
        .accum_clear       ( fsm_accum_clear       )
    );

    //------------------------------------------------------------
    // 2. group_datapath -- subordinate data mover
    //------------------------------------------------------------
    group_datapath #(
        .DATA_WIDTH   ( DATA_WIDTH   ),
        .NUM_VPUS     ( NUM_VPUS     ),
        .MPU_DEPTH    ( MPU_DEPTH    ),
        .K            ( K            ),
        .PIPE_LATENCY ( PIPE_LATENCY ),
        .NUM_MPUS     ( NUM_MPUS     ),
        .NUM_CHUNKS   ( NUM_CHUNKS   ),
        .ADD_LATENCY  ( ADD_LATENCY  )
    ) u_group_datapath (
        .clk ( clk ),
        .rst ( rst ),

        .grp_act_en   ( grp_act_en   ),
        .grp_act_we   ( grp_act_we   ),
        .grp_act_addr ( grp_act_addr ),
        .grp_act_din  ( grp_act_din  ),

        .grp_wgt_en   ( grp_wgt_en   ),
        .grp_wgt_we   ( grp_wgt_we   ),
        .grp_wgt_addr ( grp_wgt_addr ),
        .grp_wgt_din  ( grp_wgt_din  ),

        .grp_out_en   ( grp_out_en   ),
        .grp_out_addr ( grp_out_addr ),
        .grp_out_dout ( grp_out_dout ),

        .load_chunk        ( fsm_load_chunk        ),
        .chunk_idx         ( fsm_chunk_idx         ),
        .all_chunks_loaded ( dp_all_chunks_loaded  ),

        .mpu_start     ( fsm_mpu_start     ),
        .all_mpu_valid ( dp_all_mpu_valid  ),

        .do_readout       ( fsm_do_readout       ),
        .all_readout_done ( dp_all_readout_done  ),

        .vecadd_start ( fsm_vecadd_start ),
        .operand_sel  ( fsm_operand_sel  ),
        .vecadd_done  ( dp_vecadd_done   ),
        .accum_clear  ( fsm_accum_clear  )
    );

endmodule
