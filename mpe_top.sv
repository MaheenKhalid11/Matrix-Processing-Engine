`timescale 1ns / 1ps
//======================================================================
// mpe_top.sv -- structural top-level wrapper for an MPE.
//
// Mirrors group_top.sv's own role one level up:
//   - mpe_fsm      : sole command authority (control only)
//   - mpe_datapath : subordinate data mover -- owns NUM_GROUPS
//                     unmodified group_top instances, their per-group
//                     bulk weight-store lanes, the shared serialized
//                     readout engine, and the MPE's own single global
//                     output_bram.
//
// An MPE covers the full column width of one GEMV: NUM_GROUPS groups,
// each responsible for a fixed 2*MPU_DEPTH*NUM_MPUS*NUM_CHUNKS-wide
// column slice (1024 in the default config: 4 MPUs x 4 chunks x
// 256 x... no -- concretely, with the defaults below, 8 groups x
// 2 passes x 256 columns/pass = 4096 total columns, matching a
// 1x4096 x 4096x4096 GEMV. Activation is depth-only, so it's loaded
// ONCE and broadcast to every group; only weight data differs per
// (group, pass, MPU-lane) and gets its own dedicated bulk storage.
//======================================================================

module mpe_top #(
    parameter DATA_WIDTH   = 32,
    parameter NUM_VPUS     = 16,
    parameter MPU_DEPTH    = 256,
    parameter K            = 16,
    parameter PIPE_LATENCY = 24,
    parameter NUM_MPUS     = 4,
    parameter NUM_CHUNKS   = 4,
    parameter ADD_LATENCY  = 12,
    parameter NUM_GROUPS   = 8,
    parameter NUM_PASSES   = 2,

    localparam NUM_CGS          = MPU_DEPTH / NUM_VPUS,                       // 16
    localparam CG_WIDTH         = NUM_VPUS * DATA_WIDTH,                      // 512 bits
    localparam GRP_ACT_CHUNK_ADDR_W = $clog2(NUM_CHUNKS * MPU_DEPTH),        // 10 bits
    localparam WGT_CHUNK_DEPTH  = NUM_CHUNKS * NUM_CGS * MPU_DEPTH,          // 16384
    localparam GRP_WGT_CHUNK_ADDR_W = $clog2(WGT_CHUNK_DEPTH),               // 14 bits
    localparam GROUP_BITS       = $clog2(NUM_GROUPS),                        // 3 bits
    localparam PASS_BITS        = $clog2(NUM_PASSES),                       // 1 bit
    localparam BULK_ADDR_W      = PASS_BITS + GRP_WGT_CHUNK_ADDR_W,         // 15 bits
    localparam MPE_OUT_DEPTH    = NUM_GROUPS * NUM_PASSES * MPU_DEPTH,      // 4096
    localparam MPE_OUT_ADDR_W   = GROUP_BITS + PASS_BITS + $clog2(NUM_CGS)  // 8 bits
)(
    input  logic clk,
    input  logic rst,

    // Host system control interface
    input  logic start,
    output logic busy,
    output logic valid,

    // Host DMA ports -- activation preload, broadcast fan-out (one set
    // of ports per MPU-slot; identical data reaches every group's
    // matching MPU chunk store simultaneously since activation is
    // depth-only, never column-dependent).
    input  logic                            mpe_act_en   [0:NUM_MPUS-1],
    input  logic                            mpe_act_we   [0:NUM_MPUS-1],
    input  logic [GRP_ACT_CHUNK_ADDR_W-1:0] mpe_act_addr [0:NUM_MPUS-1],
    input  logic [DATA_WIDTH-1:0]           mpe_act_din  [0:NUM_MPUS-1],

    // Host DMA ports -- weight bulk preload, genuinely independent per
    // (group, MPU-lane); both passes' data for that lane, loaded once.
    input  logic                   mpe_wgt_en   [0:NUM_GROUPS-1][0:NUM_MPUS-1],
    input  logic                   mpe_wgt_we   [0:NUM_GROUPS-1][0:NUM_MPUS-1],
    input  logic [BULK_ADDR_W-1:0] mpe_wgt_addr [0:NUM_GROUPS-1][0:NUM_MPUS-1],
    input  logic [CG_WIDTH-1:0]    mpe_wgt_din  [0:NUM_GROUPS-1][0:NUM_MPUS-1],

    // Host DMA port -- MPE's final combined 1x(NUM_GROUPS*NUM_PASSES*
    // MPU_DEPTH) output (Port B), the single global 1x4096 buffer.
    input  logic                      mpe_out_en,
    input  logic [MPE_OUT_ADDR_W-1:0] mpe_out_addr,
    output logic [CG_WIDTH-1:0]       mpe_out_dout
);

    //------------------------------------------------------------
    // FSM <-> datapath handshake wires
    //------------------------------------------------------------
    logic                 fsm_load_pass;
    logic [PASS_BITS-1:0] fsm_pass_sel;
    logic                 dp_all_groups_loaded;

    logic fsm_grp_start;
    logic dp_all_groups_valid;

    logic                  fsm_do_readout;
    logic [GROUP_BITS-1:0] fsm_group_sel;
    logic                  dp_readout_done;

    //------------------------------------------------------------
    // 1. mpe_fsm -- sole command authority
    //------------------------------------------------------------
    mpe_fsm #(
        .NUM_GROUPS ( NUM_GROUPS ),
        .NUM_PASSES ( NUM_PASSES )
    ) u_mpe_fsm (
        .clk               ( clk                   ),
        .rst               ( rst                   ),
        .start             ( start                 ),
        .busy              ( busy                  ),
        .valid             ( valid                 ),
        .load_pass         ( fsm_load_pass         ),
        .pass_sel          ( fsm_pass_sel          ),
        .all_groups_loaded ( dp_all_groups_loaded  ),
        .grp_start         ( fsm_grp_start         ),
        .all_groups_valid  ( dp_all_groups_valid   ),
        .do_readout        ( fsm_do_readout        ),
        .group_sel         ( fsm_group_sel         ),
        .readout_done      ( dp_readout_done       )
    );

    //------------------------------------------------------------
    // 2. mpe_datapath -- subordinate data mover
    //------------------------------------------------------------
    mpe_datapath #(
        .DATA_WIDTH   ( DATA_WIDTH   ),
        .NUM_VPUS     ( NUM_VPUS     ),
        .MPU_DEPTH    ( MPU_DEPTH    ),
        .K            ( K            ),
        .PIPE_LATENCY ( PIPE_LATENCY ),
        .NUM_MPUS     ( NUM_MPUS     ),
        .NUM_CHUNKS   ( NUM_CHUNKS   ),
        .ADD_LATENCY  ( ADD_LATENCY  ),
        .NUM_GROUPS   ( NUM_GROUPS   ),
        .NUM_PASSES   ( NUM_PASSES   )
    ) u_mpe_datapath (
        .clk ( clk ),
        .rst ( rst ),

        .mpe_act_en   ( mpe_act_en   ),
        .mpe_act_we   ( mpe_act_we   ),
        .mpe_act_addr ( mpe_act_addr ),
        .mpe_act_din  ( mpe_act_din  ),

        .mpe_wgt_en   ( mpe_wgt_en   ),
        .mpe_wgt_we   ( mpe_wgt_we   ),
        .mpe_wgt_addr ( mpe_wgt_addr ),
        .mpe_wgt_din  ( mpe_wgt_din  ),

        .mpe_out_en   ( mpe_out_en   ),
        .mpe_out_addr ( mpe_out_addr ),
        .mpe_out_dout ( mpe_out_dout ),

        .load_pass         ( fsm_load_pass         ),
        .pass_sel          ( fsm_pass_sel          ),
        .all_groups_loaded ( dp_all_groups_loaded  ),

        .grp_start        ( fsm_grp_start        ),
        .all_groups_valid ( dp_all_groups_valid  ),

        .do_readout   ( fsm_do_readout   ),
        .group_sel    ( fsm_group_sel    ),
        .readout_done ( dp_readout_done  )
    );

endmodule
