`timescale 1ns / 1ps
//======================================================================
// mpe_datapath.sv -- subordinate data mover for mpe_top, same role one
// level up that group_datapath plays inside group_top: owns every wide
// data bus, does all the streaming/sequencing work, and only exchanges
// simple pulses + small index/select codes with mpe_fsm.
//
// An MPE covers the full column width (NUM_GROUPS x 2 x MPU_DEPTH x
// NUM_MPUS x NUM_CHUNKS columns -- 8 x 512 = 4096 in the default
// config) by running NUM_GROUPS unmodified group_top instances, each
// responsible for a fixed 512-column slice, split into NUM_PASSES=2
// column-chunks of 256 columns each (matching one group_top run).
//
// Per group (all NUM_GROUPS run in parallel, identical hardware,
// broadcast-triggered so they finish in lockstep -- same reasoning
// mpu_fsm/group_fsm already rely on):
//   - 4 bulk weight-store lanes (one per MPU), each sized for BOTH
//     passes' worth of that MPU's weight data, host-preloaded once.
//     Own dedicated ports per group per lane -- no sharing, so the
//     per-group "pass reloader" (below) can run in full parallel
//     across all NUM_GROUPS.
//   - an unmodified group_top instance. Its grp_act_* ports are pure
//     broadcast fan-out (activation only depends on depth, not
//     column, so it's identical for every group and every pass --
//     no MPE-level activation storage needed at all, just wiring).
//   - a "pass reloader": on load_pass (with pass_sel), streams that
//     pass's weight data from the 4 bulk-store lanes into this
//     group's own grp_wgt_* ports, all 4 lanes in parallel. Single
//     phase (weight only, no ACT->WGT chaining), so it does NOT need
//     group_datapath's L_WGT_START priming-state fix -- that fix was
//     specifically for a two-phase chain within one continuous run,
//     which doesn't exist here.
//
// Readout is the one thing that CANNOT run in parallel across groups:
// the whole point of this level is a single shared 1x4096 output
// buffer (one output_bram), which only has one write port. So a
// SINGLE shared readout engine (mirrors group_datapath's own shared
// vecadd_fsm pattern) is muxed across whichever group mpe_fsm
// currently selects via group_sel, one group at a time. Compute and
// weight-reload have no such shared resource and stay fully parallel.
//======================================================================

module mpe_datapath #(
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

    localparam NUM_CGS          = MPU_DEPTH / NUM_VPUS,                        // 16 (per group, per pass)
    localparam CG_WIDTH         = NUM_VPUS * DATA_WIDTH,                       // 512
    localparam GRP_OUT_ADDR_W   = $clog2(NUM_CGS),                            // 4
    localparam GRP_ACT_CHUNK_ADDR_W = $clog2(NUM_CHUNKS * MPU_DEPTH),         // 10
    localparam WGT_CHUNK_DEPTH  = NUM_CHUNKS * NUM_CGS * MPU_DEPTH,           // 16384
    localparam GRP_WGT_CHUNK_ADDR_W = $clog2(WGT_CHUNK_DEPTH),                // 14
    localparam GROUP_BITS       = $clog2(NUM_GROUPS),                         // 3
    localparam PASS_BITS        = $clog2(NUM_PASSES),                        // 1
    localparam BULK_ADDR_W      = PASS_BITS + GRP_WGT_CHUNK_ADDR_W,          // 15
    localparam MPE_OUT_DEPTH    = NUM_GROUPS * NUM_PASSES * MPU_DEPTH,       // 4096
    localparam MPE_OUT_ADDR_W   = GROUP_BITS + PASS_BITS + GRP_OUT_ADDR_W    // 8
)(
    input  logic clk,
    input  logic rst,

    //--------------------------------------------------------------
    // Activation preload -- broadcast fan-out, one set of ports per
    // MPU-slot (0..NUM_MPUS-1), identical data driven into every
    // group's matching per-MPU activation chunk store simultaneously.
    //--------------------------------------------------------------
    input  logic                            mpe_act_en   [0:NUM_MPUS-1],
    input  logic                            mpe_act_we   [0:NUM_MPUS-1],
    input  logic [GRP_ACT_CHUNK_ADDR_W-1:0] mpe_act_addr [0:NUM_MPUS-1],
    input  logic [DATA_WIDTH-1:0]           mpe_act_din  [0:NUM_MPUS-1],

    //--------------------------------------------------------------
    // Weight bulk preload -- genuinely independent per (group, MPU
    // lane); both passes' data for that lane, loaded once up front.
    //--------------------------------------------------------------
    input  logic                    mpe_wgt_en   [0:NUM_GROUPS-1][0:NUM_MPUS-1],
    input  logic                    mpe_wgt_we   [0:NUM_GROUPS-1][0:NUM_MPUS-1],
    input  logic [BULK_ADDR_W-1:0]  mpe_wgt_addr [0:NUM_GROUPS-1][0:NUM_MPUS-1],
    input  logic [CG_WIDTH-1:0]     mpe_wgt_din  [0:NUM_GROUPS-1][0:NUM_MPUS-1],

    // Final MPE output (Port B, host read)
    input  logic                      mpe_out_en,
    input  logic [MPE_OUT_ADDR_W-1:0] mpe_out_addr,
    output logic [CG_WIDTH-1:0]       mpe_out_dout,

    //--------------------------------------------------------------
    // Command interface -- driven by mpe_fsm
    //--------------------------------------------------------------
    input  logic                  load_pass,
    input  logic [PASS_BITS-1:0]  pass_sel,
    output logic                  all_groups_loaded,

    input  logic                  grp_start,
    output logic                  all_groups_valid,

    input  logic                  do_readout,
    input  logic [GROUP_BITS-1:0] group_sel,
    output logic                  readout_done
);

    // Per-group status/data, collected across the generate loop so the
    // shared readout engine and the AND-reductions below can index
    // them at runtime.
    logic [NUM_GROUPS-1:0] grp_valid_arr;
    logic [NUM_GROUPS-1:0] group_loaded_arr;
    logic [CG_WIDTH-1:0]   grp_out_dout_arr [0:NUM_GROUPS-1];

    assign all_groups_valid  = &grp_valid_arr;
    assign all_groups_loaded = &group_loaded_arr;

    // Shared readout engine's request, fanned out to whichever group
    // group_sel currently names (see gen_group below for the per-group
    // gating), and that group's response muxed back in above.
    logic                      shared_out_en;
    logic [GRP_OUT_ADDR_W-1:0] shared_out_addr;

    //================================================================
    // Per-group: bulk weight lanes, group_top instance, pass reloader
    //================================================================
    genvar g;
    generate
    for (g = 0; g < NUM_GROUPS; g = g + 1) begin : gen_group

        logic [CG_WIDTH-1:0]    bulk_b_dout [0:NUM_MPUS-1];
        logic                   bulk_b_en   [0:NUM_MPUS-1];
        logic [BULK_ADDR_W-1:0] bulk_b_addr [0:NUM_MPUS-1];

        genvar m;
        for (m = 0; m < NUM_MPUS; m = m + 1) begin : gen_bulk
            uram_tdp_init #(
                .DATA_WIDTH ( CG_WIDTH    ),
                .ADDR_WIDTH ( BULK_ADDR_W ),
                .INIT_FILE  ( ""          )
            ) u_bulk (
                .clk    ( clk                     ),
                .en_a   ( mpe_wgt_en[g][m]         ),
                .we_a   ( mpe_wgt_we[g][m]         ),
                .addr_a ( mpe_wgt_addr[g][m]       ),
                .din_a  ( mpe_wgt_din[g][m]        ),
                .dout_a (                          ),
                .en_b   ( bulk_b_en[m]             ),
                .we_b   ( 1'b0                     ),
                .addr_b ( bulk_b_addr[m]           ),
                .din_b  ( '0                       ),
                .dout_b ( bulk_b_dout[m]           )
            );
        end

        //------------------------------------------------------------
        // group_top instance -- unmodified. Activation ports are pure
        // fan-out from the MPE-level broadcast; weight ports are
        // driven by this group's own pass reloader below.
        //------------------------------------------------------------
        logic                            grp_act_en_i   [0:NUM_MPUS-1];
        logic                            grp_act_we_i   [0:NUM_MPUS-1];
        logic [GRP_ACT_CHUNK_ADDR_W-1:0] grp_act_addr_i [0:NUM_MPUS-1];
        logic [DATA_WIDTH-1:0]           grp_act_din_i  [0:NUM_MPUS-1];

        logic                            grp_wgt_en_i   [0:NUM_MPUS-1];
        logic                            grp_wgt_we_i   [0:NUM_MPUS-1];
        logic [GRP_WGT_CHUNK_ADDR_W-1:0] grp_wgt_addr_i [0:NUM_MPUS-1];
        logic [CG_WIDTH-1:0]             grp_wgt_din_i  [0:NUM_MPUS-1];

        logic                      grp_out_en_i;
        logic [GRP_OUT_ADDR_W-1:0] grp_out_addr_i;
        logic [CG_WIDTH-1:0]       grp_out_dout_i;

        logic grp_valid_i;

        for (m = 0; m < NUM_MPUS; m = m + 1) begin : gen_act_fanout
            assign grp_act_en_i[m]   = mpe_act_en[m];
            assign grp_act_we_i[m]   = mpe_act_we[m];
            assign grp_act_addr_i[m] = mpe_act_addr[m];
            assign grp_act_din_i[m]  = mpe_act_din[m];
        end

        group_top #(
            .DATA_WIDTH   ( DATA_WIDTH   ),
            .NUM_VPUS     ( NUM_VPUS     ),
            .MPU_DEPTH    ( MPU_DEPTH    ),
            .K            ( K            ),
            .PIPE_LATENCY ( PIPE_LATENCY ),
            .NUM_MPUS     ( NUM_MPUS     ),
            .NUM_CHUNKS   ( NUM_CHUNKS   ),
            .ADD_LATENCY  ( ADD_LATENCY  )
        ) u_group (
            .clk ( clk ),
            .rst ( rst ),

            .start ( grp_start   ),
            .busy  (             ),
            .valid ( grp_valid_i ),

            .grp_act_en   ( grp_act_en_i   ),
            .grp_act_we   ( grp_act_we_i   ),
            .grp_act_addr ( grp_act_addr_i ),
            .grp_act_din  ( grp_act_din_i  ),

            .grp_wgt_en   ( grp_wgt_en_i   ),
            .grp_wgt_we   ( grp_wgt_we_i   ),
            .grp_wgt_addr ( grp_wgt_addr_i ),
            .grp_wgt_din  ( grp_wgt_din_i  ),

            .grp_out_en   ( grp_out_en_i   ),
            .grp_out_addr ( grp_out_addr_i ),
            .grp_out_dout ( grp_out_dout_i )
        );

        assign grp_valid_arr[g]    = grp_valid_i;
        assign grp_out_dout_arr[g] = grp_out_dout_i;

        // This group's own output port is only driven by the shared
        // readout engine when group_sel names this group; otherwise
        // it sits idle. Safe as a plain mux since mpe_fsm only ever
        // asserts do_readout for one group_sel value at a time.
        assign grp_out_en_i   = (group_sel == g) ? shared_out_en : 1'b0;
        assign grp_out_addr_i = shared_out_addr;

        //------------------------------------------------------------
        // Pass reloader: on load_pass, stream pass_sel's weight data
        // from this group's 4 bulk-store lanes into its own grp_wgt_*
        // ports, all 4 lanes in parallel (independent physical ports,
        // no conflict). Single phase -- no ACT/WGT chaining, so none
        // of group_datapath's L_WGT_START subtlety applies here.
        //------------------------------------------------------------
        typedef enum logic { RL_IDLE, RL_RUN } rlstate_t;
        rlstate_t rlstate, rlstate_next;

        logic [GRP_WGT_CHUNK_ADDR_W-1:0] r_liss, r_liss_next, r_lcap;
        logic                             r_lread_valid;

        assign r_liss_next = r_liss + 1'b1;

        always_ff @(posedge clk) begin
            if (rst) rlstate <= RL_IDLE;
            else     rlstate <= rlstate_next;
        end

        always_comb begin
            rlstate_next = rlstate;
            case (rlstate)
                RL_IDLE: if (load_pass) rlstate_next = RL_RUN;
                RL_RUN:  if (r_lread_valid && r_lcap == WGT_CHUNK_DEPTH-1) rlstate_next = RL_IDLE;
                default: rlstate_next = RL_IDLE;
            endcase
        end

        // Issue side -- all 4 lanes advance in lockstep (lane 0 used
        // as the representative "are we still issuing" flag; all 4
        // are always set/cleared together, so this is exact).
        always_ff @(posedge clk) begin
            if (rst) begin
                for (int mm = 0; mm < NUM_MPUS; mm++) begin
                    bulk_b_en[mm]   <= 1'b0;
                    bulk_b_addr[mm] <= '0;
                end
                r_liss <= '0;
            end else begin
                case (rlstate)
                    RL_IDLE: begin
                        r_liss <= '0;
                        if (load_pass) begin
                            for (int mm = 0; mm < NUM_MPUS; mm++) begin
                                bulk_b_en[mm]   <= 1'b1;
                                bulk_b_addr[mm] <= {pass_sel, {GRP_WGT_CHUNK_ADDR_W{1'b0}}};
                            end
                        end else begin
                            for (int mm = 0; mm < NUM_MPUS; mm++) bulk_b_en[mm] <= 1'b0;
                        end
                    end

                    RL_RUN: begin
                        if (bulk_b_en[0]) begin
                            if (r_liss == WGT_CHUNK_DEPTH-1) begin
                                for (int mm = 0; mm < NUM_MPUS; mm++) bulk_b_en[mm] <= 1'b0;
                            end else begin
                                r_liss <= r_liss_next;
                                for (int mm = 0; mm < NUM_MPUS; mm++)
                                    bulk_b_addr[mm] <= {pass_sel, r_liss_next};
                            end
                        end
                    end

                    default: for (int mm = 0; mm < NUM_MPUS; mm++) bulk_b_en[mm] <= 1'b0;
                endcase
            end
        end

        always_ff @(posedge clk) begin
            if (rst) r_lread_valid <= 1'b0;
            else     r_lread_valid <= bulk_b_en[0];
        end

        always_ff @(posedge clk) begin
            if (rst || rlstate == RL_IDLE) r_lcap <= '0;
            else if (r_lread_valid)        r_lcap <= r_lcap + 1'b1;
        end

        // Capture side -- write the just-arrived word into group_top's
        // host ports, all 4 lanes in parallel.
        always_ff @(posedge clk) begin
            if (rst) begin
                for (int mm = 0; mm < NUM_MPUS; mm++) begin
                    grp_wgt_en_i[mm]   <= 1'b0;
                    grp_wgt_we_i[mm]   <= 1'b0;
                    grp_wgt_addr_i[mm] <= '0;
                    grp_wgt_din_i[mm]  <= '0;
                end
            end else begin
                for (int mm = 0; mm < NUM_MPUS; mm++) begin
                    grp_wgt_en_i[mm] <= 1'b0;
                    grp_wgt_we_i[mm] <= 1'b0;
                end
                if (r_lread_valid) begin
                    for (int mm = 0; mm < NUM_MPUS; mm++) begin
                        grp_wgt_en_i[mm]   <= 1'b1;
                        grp_wgt_we_i[mm]   <= 1'b1;
                        grp_wgt_addr_i[mm] <= r_lcap;
                        grp_wgt_din_i[mm]  <= bulk_b_dout[mm];
                    end
                end
            end
        end

        assign group_loaded_arr[g] = r_lread_valid && rlstate == RL_RUN && (r_lcap == WGT_CHUNK_DEPTH-1);

    end
    endgenerate

    //================================================================
    // Shared readout engine -- the ONE resource that cannot be
    // parallelized across groups, since it's the only thing touching
    // the single shared global output_bram. mpe_fsm serializes access
    // by holding group_sel stable for one full 16-row readout, then
    // moving to the next group.
    //================================================================
    typedef enum logic { RO_IDLE, RO_RUN } rostate_t;
    rostate_t rostate, rostate_next;

    logic [GRP_OUT_ADDR_W-1:0] ro_riss, ro_rcap;
    logic                      ro_rread_valid;

    always_ff @(posedge clk) begin
        if (rst) rostate <= RO_IDLE;
        else     rostate <= rostate_next;
    end

    always_comb begin
        rostate_next = rostate;
        case (rostate)
            RO_IDLE: if (do_readout) rostate_next = RO_RUN;
            RO_RUN:  if (ro_rread_valid && ro_rcap == NUM_CGS-1) rostate_next = RO_IDLE;
            default: rostate_next = RO_IDLE;
        endcase
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            shared_out_en   <= 1'b0;
            shared_out_addr <= '0;
            ro_riss         <= '0;
        end else begin
            case (rostate)
                RO_IDLE: begin
                    ro_riss <= '0;
                    if (do_readout) begin
                        shared_out_en   <= 1'b1;
                        shared_out_addr <= '0;
                    end else begin
                        shared_out_en <= 1'b0;
                    end
                end
                RO_RUN: begin
                    if (shared_out_en) begin
                        if (ro_riss == NUM_CGS-1) begin
                            shared_out_en <= 1'b0;
                        end else begin
                            ro_riss         <= ro_riss + 1'b1;
                            shared_out_addr <= ro_riss + 1'b1;
                        end
                    end
                end
                default: shared_out_en <= 1'b0;
            endcase
        end
    end

    always_ff @(posedge clk) begin
        if (rst) ro_rread_valid <= 1'b0;
        else     ro_rread_valid <= shared_out_en;
    end

    always_ff @(posedge clk) begin
        if (rst || rostate == RO_IDLE) ro_rcap <= '0;
        else if (ro_rread_valid)       ro_rcap <= ro_rcap + 1'b1;
    end

    assign readout_done = ro_rread_valid && rostate == RO_RUN && (ro_rcap == NUM_CGS-1);

    //================================================================
    // Global output_bram -- reused unmodified, DEPTH sized to cover
    // every group's every pass. Write address = {group_sel, pass_sel,
    // local CG-row}, a plain concatenation since NUM_GROUPS, NUM_PASSES
    // and NUM_CGS are all powers of two -- same adderless addressing
    // principle used at every level below this one.
    //================================================================
    logic out_a_en;
    assign out_a_en = ro_rread_valid;

    output_bram #(
        .DATA_WIDTH ( DATA_WIDTH    ),
        .NUM_VPUS   ( NUM_VPUS      ),
        .DEPTH      ( MPE_OUT_DEPTH )
    ) u_global_out (
        .clk    ( clk                                    ),
        .a_en   ( out_a_en                                ),
        .a_we   ( out_a_en                                ),
        .a_addr ( {group_sel, pass_sel, ro_rcap}           ),
        .a_din  ( grp_out_dout_arr[group_sel]              ),
        .a_dout (                                          ),
        .b_en   ( mpe_out_en                                ),
        .b_addr ( mpe_out_addr                              ),
        .b_dout ( mpe_out_dout                              )
    );

endmodule
