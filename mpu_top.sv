`timescale 1ns / 1ps
//======================================================================
// mpu_top.sv -- REBUILT FROM SCRATCH
//
// Structural top-level wrapper:
//   - mpu_fsm       : sole command authority (control only)
//   - memory_controller : subordinate data mover (control + datapath
//                         glue between BRAM/URAM and the VPU array)
//   - 16x vpu       : compute array, fully unmodified
//   - activation_bram / weight_uram / output_bram : physical storage
//
// mpu_fsm never touches activation_full/weight_full/vpu_result
// directly -- those wide data buses flow only between
// memory_controller and the VPU array. mpu_fsm only issues control
// pulses and reads status pulses/flags.
//======================================================================

module mpu_top #(
    parameter DATA_WIDTH  = 32,
    parameter NUM_VPUS    = 16,
    parameter DEPTH       = 256,
    parameter K           = 16,
    parameter PIPE_LATENCY = 24,   // see vpu.sv -- derived from K=16 cascade add-ripple, needs tb_vpu verification
    parameter ACT_INIT    = "",
    parameter WGT_INIT    = "",

    localparam NUM_CGS    = DEPTH / NUM_VPUS,                   // 16
    localparam CG_WIDTH   = NUM_VPUS * DATA_WIDTH,              // 512 bits
    localparam ACT_ADDR_W = $clog2(DEPTH),                      // 8 bits
    localparam WGT_ADDR_W = $clog2(NUM_CGS * DEPTH),            // 12 bits
    localparam OUT_ADDR_W = $clog2(NUM_CGS)                     // 4 bits
)(
    input  logic                  clk,
    input  logic                  rst,

    // Host system control interface
    input  logic                  start,
    output logic                  busy,
    output logic                  valid,

    // Host DMA ports -- activation BRAM write (Port A)
    input  logic                  host_act_en,
    input  logic                  host_act_we,
    input  logic [ACT_ADDR_W-1:0] host_act_addr,
    input  logic [DATA_WIDTH-1:0] host_act_din,

    // Host DMA ports -- weight URAM write (Port A)
    input  logic                  host_wgt_en,
    input  logic                  host_wgt_we,
    input  logic [WGT_ADDR_W-1:0] host_wgt_addr,
    input  logic [CG_WIDTH-1:0]   host_wgt_din,

    // Host DMA ports -- output BRAM read (Port B)
    input  logic                  host_out_en,
    input  logic [OUT_ADDR_W-1:0] host_out_addr,
    output logic [CG_WIDTH-1:0]   host_out_dout
);

    //------------------------------------------------------------
    // FSM <-> memory_controller handshake wires
    //------------------------------------------------------------
    logic                fsm_load_act;
    logic                mem_act_done;
    logic                fsm_load_weights;
    logic                mem_weights_done;
    logic [OUT_ADDR_W-1:0] fsm_fetch_cg_index;
    logic [OUT_ADDR_W-1:0] fsm_compute_cg_index;
    logic                fsm_commit_output;
    logic                mem_output_done;
    logic                mem_busy;

    // FSM <-> VPU array handshake wires
    logic                fsm_vpu_start;
    logic [NUM_VPUS-1:0] vpu_valid;
    logic [DATA_WIDTH-1:0] vpu_result [0:NUM_VPUS-1];

    // memory_controller <-> VPU array data buses
    logic [DEPTH*DATA_WIDTH-1:0] activation_full;
    logic [DEPTH*DATA_WIDTH-1:0] weight_full [0:NUM_VPUS-1];

    // memory_controller <-> physical memory ports
    logic                  act_b_en;
    logic [ACT_ADDR_W-1:0] act_b_addr;
    logic [DATA_WIDTH-1:0] act_b_dout;

    logic                  wgt_b_en;
    logic [WGT_ADDR_W-1:0] wgt_b_addr;
    logic [CG_WIDTH-1:0]   wgt_b_dout;

    logic                  out_a_en;
    logic                  out_a_we;
    logic [OUT_ADDR_W-1:0] out_a_addr;
    logic [CG_WIDTH-1:0]   out_a_din;

    //------------------------------------------------------------
    // 1. mpu_fsm -- sole command authority
    //------------------------------------------------------------
    mpu_fsm #(
        .NUM_VPUS ( NUM_VPUS ),
        .DEPTH    ( DEPTH    )
    ) u_mpu_fsm (
        .clk           ( clk               ),
        .rst           ( rst               ),
        .start         ( start             ),
        .busy          ( busy              ),
        .valid         ( valid             ),
        .load_act         ( fsm_load_act         ),
        .act_done         ( mem_act_done         ),
        .load_weights     ( fsm_load_weights     ),
        .fetch_cg_index   ( fsm_fetch_cg_index   ),
        .weights_done     ( mem_weights_done     ),
        .commit_output    ( fsm_commit_output    ),
        .compute_cg_index ( fsm_compute_cg_index ),
        .output_done      ( mem_output_done      ),
        .vpu_start        ( fsm_vpu_start        ),
        .vpu_valid        ( vpu_valid            )
    );

    //------------------------------------------------------------
    // 2. memory_controller -- subordinate data mover
    //------------------------------------------------------------
    memory_controller #(
        .DATA_WIDTH ( DATA_WIDTH ),
        .NUM_VPUS   ( NUM_VPUS   ),
        .DEPTH      ( DEPTH      )
    ) u_memory_controller (
        .clk             ( clk               ),
        .rst             ( rst               ),

        .load_act        ( fsm_load_act         ),
        .act_done        ( mem_act_done         ),
        .load_weights    ( fsm_load_weights     ),
        .fetch_cg_index  ( fsm_fetch_cg_index   ),
        .weights_done    ( mem_weights_done     ),
        .commit_output   ( fsm_commit_output    ),
        .compute_cg_index( fsm_compute_cg_index ),
        .output_done     ( mem_output_done      ),
        .mem_busy        ( mem_busy             ),

        .activation_full ( activation_full   ),
        .weight_full     ( weight_full       ),
        .vpu_result      ( vpu_result        ),

        .act_b_en        ( act_b_en          ),
        .act_b_addr      ( act_b_addr        ),
        .act_b_dout      ( act_b_dout        ),

        .wgt_b_en        ( wgt_b_en          ),
        .wgt_b_addr      ( wgt_b_addr        ),
        .wgt_b_dout      ( wgt_b_dout        ),

        .out_a_en        ( out_a_en          ),
        .out_a_we        ( out_a_we          ),
        .out_a_addr      ( out_a_addr        ),
        .out_a_din       ( out_a_din         )
    );

    //------------------------------------------------------------
    // 3. 16x VPU array -- fully unmodified
    //------------------------------------------------------------
    generate
        for (genvar i = 0; i < NUM_VPUS; i++) begin : gen_vpu_array
            vpu #(
                .K            ( K            ),
                .DEPTH        ( DEPTH        ),
                .PIPE_LATENCY ( PIPE_LATENCY )
            ) u_vpu (
                .clk             ( clk               ),
                .rst             ( rst               ),
                .start           ( fsm_vpu_start     ),
                .activation_full ( activation_full   ),
                .weight_full     ( weight_full[i]    ),
                .busy            (                   ), // unused at top level
                .valid           ( vpu_valid[i]      ),
                .dot_product     ( vpu_result[i]     )
            );
        end
    endgenerate

    //------------------------------------------------------------
    // 4. Physical storage wrappers
    //------------------------------------------------------------
    activation_bram #(
        .DATA_WIDTH ( DATA_WIDTH ),
        .DEPTH      ( DEPTH      ),
        .INIT_FILE  ( ACT_INIT   )
    ) u_activation_bram (
        .clk    ( clk           ),
        .a_en   ( host_act_en   ),
        .a_we   ( host_act_we   ),
        .a_addr ( host_act_addr ),
        .a_din  ( host_act_din  ),
        .a_dout (               ),
        .b_en   ( act_b_en      ),
        .b_addr ( act_b_addr    ),
        .b_dout ( act_b_dout    )
    );

    weight_uram #(
        .DATA_WIDTH ( DATA_WIDTH ),
        .NUM_VPUS   ( NUM_VPUS   ),
        .DEPTH      ( DEPTH      ),
        .INIT_FILE  ( WGT_INIT   )
    ) u_weight_uram (
        .clk    ( clk           ),
        .a_en   ( host_wgt_en   ),
        .a_we   ( host_wgt_we   ),
        .a_addr ( host_wgt_addr ),
        .a_din  ( host_wgt_din  ),
        .a_dout (               ),
        .b_en   ( wgt_b_en      ),
        .b_addr ( wgt_b_addr    ),
        .b_dout ( wgt_b_dout    )
    );

    output_bram #(
        .DATA_WIDTH ( DATA_WIDTH ),
        .NUM_VPUS   ( NUM_VPUS   ),
        .DEPTH      ( DEPTH      )
    ) u_output_bram (
        .clk    ( clk           ),
        .a_en   ( out_a_en      ),
        .a_we   ( out_a_we      ),
        .a_addr ( out_a_addr    ),
        .a_din  ( out_a_din     ),
        .a_dout (               ),
        .b_en   ( host_out_en   ),
        .b_addr ( host_out_addr ),
        .b_dout ( host_out_dout )
    );

endmodule