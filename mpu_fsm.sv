`timescale 1ns / 1ps
//======================================================================
// mpu_fsm.sv -- Sole command authority over memory_controller and the
// 16 VPUs, with weight-prefetch overlap.
//
// Pipelined-by-one-group sequence: while column group `c` is running
// on the VPU array, the weights for column group `c+1` are fetched
// into the OTHER weight buffer in memory_controller (double-buffered
// there). So each group's ~256-cycle weight fetch is hidden under the
// much longer VPU compute time instead of being paid serially.
//
//   IDLE
//    -> REQ_ACT    (pulse load_act, once per run)
//    -> WAIT_ACT   (poll act_done)
//    -> REQ_WGT0   (pulse load_weights for group 0 -- priming fetch,
//                   nothing to overlap it with yet)
//    -> WAIT_WGT0  (poll weights_done)
//    -> REQ_VPU    (pulse vpu_start for compute_idx; if there is a
//                   next group, ALSO pulse load_weights for
//                   compute_idx+1 in the same cycle -- memory_controller
//                   writes that fetch into the buffer NOT being read
//                   by the VPUs this pass)
//    -> WAIT_VPU   (poll vpu_valid and, if a prefetch was launched,
//                   weights_done for it too -- sticky-latched so
//                   either pulse can land in any order/cycle)
//    -> REQ_COMMIT (pulse commit_output for compute_idx)
//    -> WAIT_COMMIT(poll output_done)
//    -> [compute_idx == NUM_CGS-1 ? DONE : REQ_VPU]  (next group's
//        weights are already sitting in the other buffer)
//    -> DONE (pulse valid for 1 cycle) -> IDLE
//======================================================================

module mpu_fsm #(
    parameter NUM_VPUS = 16,
    parameter DEPTH    = 256,
    localparam NUM_CGS = DEPTH / NUM_VPUS,          // 16
    localparam CG_BITS = $clog2(NUM_CGS)            // 4
)(
    input  logic                 clk,
    input  logic                 rst,

    // Host handshake
    input  logic                 start,
    output logic                 busy,
    output logic                 valid,

    // Memory controller handshake -- each operation has its own
    // dedicated request pulse and its own dedicated done pulse.
    output logic                 load_act,
    input  logic                 act_done,

    output logic                 load_weights,
    output logic [CG_BITS-1:0]   fetch_cg_index,    // which group's weights this fetch targets
    input  logic                 weights_done,

    output logic                 commit_output,
    output logic [CG_BITS-1:0]   compute_cg_index,  // group currently on the VPU array / being committed
    input  logic                 output_done,

    // VPU array handshake -- one broadcast start, one valid per VPU
    output logic                 vpu_start,
    input  logic [NUM_VPUS-1:0]  vpu_valid
);

    typedef enum logic [3:0] {
        S_IDLE,
        S_REQ_ACT,
        S_WAIT_ACT,
        S_REQ_WGT0,
        S_WAIT_WGT0,
        S_REQ_VPU,
        S_WAIT_VPU,
        S_REQ_COMMIT,
        S_WAIT_COMMIT,
        S_DONE
    } state_t;

    state_t state, next_state;

    logic [CG_BITS-1:0] compute_idx;   // group index on the VPU array this pass
    logic                has_next;      // there is a compute_idx+1 group to prefetch
    logic                vpu_seen_done; // sticky: all_vpus_done has fired this pass
    logic                wgt_seen_done; // sticky: the overlapped weights_done has fired this pass
    logic                all_vpus_done;
    logic                ready_to_commit;

    assign has_next        = (compute_idx != NUM_CGS-1);
    assign all_vpus_done    = &vpu_valid;
    assign ready_to_commit  = (all_vpus_done || vpu_seen_done) &&
                               (!has_next || weights_done || wgt_seen_done);

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
            S_IDLE:        if (start)          next_state = S_REQ_ACT;

            S_REQ_ACT:                          next_state = S_WAIT_ACT;
            S_WAIT_ACT:    if (act_done)        next_state = S_REQ_WGT0;

            S_REQ_WGT0:                         next_state = S_WAIT_WGT0;
            S_WAIT_WGT0:   if (weights_done)    next_state = S_REQ_VPU;

            S_REQ_VPU:                          next_state = S_WAIT_VPU;
            S_WAIT_VPU:    if (ready_to_commit) next_state = S_REQ_COMMIT;

            S_REQ_COMMIT:                       next_state = S_WAIT_COMMIT;
            S_WAIT_COMMIT: if (output_done) begin
                               if (compute_idx == NUM_CGS-1)
                                   next_state = S_DONE;
                               else
                                   next_state = S_REQ_VPU;
                           end

            S_DONE:                             next_state = S_IDLE;

            default:                             next_state = S_IDLE;
        endcase
    end

    //------------------------------------------------------------
    // Block 3: registered outputs
    //------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            busy             <= 1'b0;
            valid            <= 1'b0;
            load_act         <= 1'b0;
            load_weights     <= 1'b0;
            commit_output    <= 1'b0;
            vpu_start        <= 1'b0;
            compute_idx      <= '0;
            fetch_cg_index   <= '0;
            compute_cg_index <= '0;
            vpu_seen_done    <= 1'b0;
            wgt_seen_done    <= 1'b0;
        end else begin
            // Pulse defaults -- every request pulse is exactly one
            // cycle wide unless explicitly re-asserted below.
            valid         <= 1'b0;
            load_act      <= 1'b0;
            load_weights  <= 1'b0;
            commit_output <= 1'b0;
            vpu_start     <= 1'b0;

            case (state)
                S_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        busy        <= 1'b1;
                        compute_idx <= '0;
                    end
                end

                S_REQ_ACT:    load_act <= 1'b1;
                S_WAIT_ACT:   ; // just waiting on act_done

                S_REQ_WGT0: begin
                    load_weights   <= 1'b1;
                    fetch_cg_index <= '0;      // priming fetch, group 0
                end
                S_WAIT_WGT0:  ; // just waiting on weights_done

                S_REQ_VPU: begin
                    vpu_start        <= 1'b1;
                    compute_cg_index <= compute_idx;
                    vpu_seen_done    <= 1'b0;
                    wgt_seen_done    <= 1'b0;

                    // Overlap: launch the fetch for the NEXT group
                    // the same cycle compute starts on this one.
                    if (compute_idx != NUM_CGS-1) begin
                        load_weights   <= 1'b1;
                        fetch_cg_index <= compute_idx + 1'b1;
                    end
                end

                S_WAIT_VPU: begin
                    if (all_vpus_done) vpu_seen_done <= 1'b1;
                    if (weights_done)  wgt_seen_done <= 1'b1;
                end

                S_REQ_COMMIT: commit_output <= 1'b1;

                S_WAIT_COMMIT: begin
                    if (output_done && compute_idx != NUM_CGS-1)
                        compute_idx <= compute_idx + 1'b1;
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
