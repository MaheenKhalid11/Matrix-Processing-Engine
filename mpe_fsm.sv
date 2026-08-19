`timescale 1ns / 1ps
//======================================================================
// mpe_fsm.sv -- sole command authority over mpe_datapath. Same role
// one level up that group_fsm plays inside group_top: every handshake
// step gets its own request state and its own dedicated done signal;
// this FSM never touches the wide data buses itself, only pulses +
// small index/select codes.
//
// Sequence (NUM_PASSES column-chunk passes, each covering all
// NUM_GROUPS groups):
//   IDLE
//    -> [ REQ_LOAD    (pulse load_pass, pass_sel=pass_cnt, broadcast
//                       to all groups' pass reloaders -- runs in full
//                       parallel, no shared resource)
//         WAIT_LOAD    (poll all_groups_loaded)
//         REQ_COMPUTE  (pulse grp_start, broadcast to all group_tops)
//         WAIT_COMPUTE (poll all_groups_valid)
//         REQ_READOUT x NUM_GROUPS (pulse do_readout, group_sel=group_cnt
//                       -- the ONE thing that must serialize, since
//                       there's only one physical output_bram write port)
//         WAIT_READOUT
//       ] repeated for pass_cnt = 0 .. NUM_PASSES-1
//    -> DONE (pulse valid) -> IDLE
//======================================================================

module mpe_fsm #(
    parameter NUM_GROUPS = 8,
    parameter NUM_PASSES = 2,
    localparam GROUP_BITS = $clog2(NUM_GROUPS),
    localparam PASS_BITS  = $clog2(NUM_PASSES)
)(
    input  logic clk,
    input  logic rst,

    // Host handshake
    input  logic start,
    output logic busy,
    output logic valid,

    // Pass-reload handshake -- broadcast to all groups in parallel
    output logic                 load_pass,
    output logic [PASS_BITS-1:0] pass_sel,
    input  logic                 all_groups_loaded,

    // Compute handshake -- broadcast start to all group_tops
    output logic grp_start,
    input  logic all_groups_valid,

    // Readout handshake -- serialized across groups (shared output_bram)
    output logic                  do_readout,
    output logic [GROUP_BITS-1:0] group_sel,
    input  logic                  readout_done
);

    typedef enum logic [2:0] {
        S_IDLE,
        S_REQ_LOAD,
        S_WAIT_LOAD,
        S_REQ_COMPUTE,
        S_WAIT_COMPUTE,
        S_REQ_READOUT,
        S_WAIT_READOUT,
        S_DONE
    } state_t;

    state_t state, next_state;

    logic [PASS_BITS-1:0]  pass_cnt;   // 0 .. NUM_PASSES-1
    logic [GROUP_BITS-1:0] group_cnt;  // 0 .. NUM_GROUPS-1, readout inner loop

    //------------------------------------------------------------
    // Block 1: state register
    //------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) state <= S_IDLE;
        else     state <= next_state;
    end

    //------------------------------------------------------------
    // Block 2: next-state logic (no output assignments here)
    //------------------------------------------------------------
    always_comb begin
        next_state = state;
        case (state)
            S_IDLE:         if (start)             next_state = S_REQ_LOAD;

            S_REQ_LOAD:                             next_state = S_WAIT_LOAD;
            S_WAIT_LOAD:    if (all_groups_loaded)  next_state = S_REQ_COMPUTE;

            S_REQ_COMPUTE:                          next_state = S_WAIT_COMPUTE;
            S_WAIT_COMPUTE: if (all_groups_valid)   next_state = S_REQ_READOUT;

            S_REQ_READOUT:                          next_state = S_WAIT_READOUT;
            S_WAIT_READOUT: if (readout_done) begin
                                 if (group_cnt != NUM_GROUPS-1)
                                     next_state = S_REQ_READOUT;
                                 else if (pass_cnt != NUM_PASSES-1)
                                     next_state = S_REQ_LOAD;
                                 else
                                     next_state = S_DONE;
                             end

            S_DONE:                                 next_state = S_IDLE;

            default:                                next_state = S_IDLE;
        endcase
    end

    //------------------------------------------------------------
    // Block 3: registered outputs
    //------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            busy       <= 1'b0;
            valid      <= 1'b0;
            load_pass  <= 1'b0;
            pass_sel   <= '0;
            grp_start  <= 1'b0;
            do_readout <= 1'b0;
            group_sel  <= '0;
            pass_cnt   <= '0;
            group_cnt  <= '0;
        end else begin
            // Pulse defaults -- every request pulse is exactly one
            // cycle wide unless explicitly re-asserted below.
            load_pass  <= 1'b0;
            grp_start  <= 1'b0;
            do_readout <= 1'b0;
            valid      <= 1'b0;

            case (state)
                S_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        busy      <= 1'b1;
                        pass_cnt  <= '0;
                        group_cnt <= '0;
                    end
                end

                S_REQ_LOAD: begin
                    load_pass <= 1'b1;
                    pass_sel  <= pass_cnt;
                end
                S_WAIT_LOAD: ; // just waiting on all_groups_loaded

                S_REQ_COMPUTE:  grp_start <= 1'b1;
                S_WAIT_COMPUTE: ; // just waiting on all_groups_valid

                S_REQ_READOUT: begin
                    do_readout <= 1'b1;
                    group_sel  <= group_cnt;
                end
                S_WAIT_READOUT: begin
                    if (readout_done) begin
                        if (group_cnt != NUM_GROUPS-1) begin
                            group_cnt <= group_cnt + 1'b1;
                        end else begin
                            group_cnt <= '0;
                            if (pass_cnt != NUM_PASSES-1)
                                pass_cnt <= pass_cnt + 1'b1;
                        end
                    end
                end

                S_DONE: begin
                    busy  <= 1'b0;
                    valid <= 1'b1;
                end

                default: ;
            endcase
        end
    end

endmodule
