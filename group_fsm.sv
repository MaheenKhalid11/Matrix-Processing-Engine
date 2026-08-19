`timescale 1ns / 1ps
//======================================================================
// group_fsm.sv -- sole command authority over group_datapath.
// Same role one level up that mpu_fsm plays inside mpu_top: every
// handshake step gets its own request state and its own dedicated
// done signal; this FSM never touches the wide 512-bit buses itself,
// only pulses + small index/select codes.
//
// Sequence (NUM_CHUNKS chunks per MPU, all NUM_MPUS MPUs run each
// chunk in lockstep since they're identical hardware started
// together):
//   IDLE
//    -> [ REQ_LOAD  (pulse load_chunk, broadcast to all MPUs' loaders)
//         WAIT_LOAD (poll all_chunks_loaded)
//         REQ_MPU   (pulse mpu_start, broadcast to all mpu_tops)
//         WAIT_MPU  (poll all_mpu_valid)
//         REQ_READOUT  (pulse do_readout, broadcast, parallel per-MPU)
//         WAIT_READOUT (poll all_readout_done)
//         REQ_ACCUM x NUM_MPUS (pulse vecadd_start, operand_sel=mpu,
//                      looping the shared adder bank across all 4
//                      MPUs' self-accumulate one at a time)
//         WAIT_ACCUM
//       ] repeated for chunk_idx = 0 .. NUM_CHUNKS-1
//    -> REQ_COMBINE x 3 (pulse vecadd_start, operand_sel=4,5,6 --
//         the 3 sequential stages combining all 4 MPUs' accumulators;
//         stage 3 writes straight into group_datapath's output_bram)
//    -> WAIT_COMBINE
//    -> DONE (pulse valid) -> IDLE
//
// v1 is fully serial (no chunk-load/compute prefetch overlap at the
// group level, unlike mpu_fsm's weight prefetch) -- same reasoning
// as mpu_fsm's own first version: get it correct first, then layer
// the same prefetch trick on top later if the load time turns out to
// matter next to 4x mpu_top's compute time.
//======================================================================

module group_fsm #(
    parameter NUM_MPUS   = 4,
    parameter NUM_CHUNKS = 4,
    localparam MPU_BITS   = $clog2(NUM_MPUS),
    localparam CHUNK_BITS = $clog2(NUM_CHUNKS)
)(
    input  logic clk,
    input  logic rst,

    // Host handshake
    input  logic start,
    output logic busy,
    output logic valid,

    // Chunk loader handshake -- broadcast to all NUM_MPUS loaders in parallel
    output logic                  load_chunk,
    output logic [CHUNK_BITS-1:0] chunk_idx,
    input  logic                  all_chunks_loaded,

    // MPU compute handshake -- broadcast start to all NUM_MPUS mpu_tops
    output logic mpu_start,
    input  logic all_mpu_valid,

    // Per-MPU output readout handshake -- broadcast, parallel per-MPU
    output logic do_readout,
    input  logic all_readout_done,

    // Shared vector-adder handshake -- serialized: only one physical
    // adder bank exists, reused across 4 self-accumulate ops per
    // chunk plus 3 final-combine stages.
    output logic       vecadd_start,
    output logic [2:0] operand_sel,
    input  logic       vecadd_done,

    // Pulsed once at the start of every run (S_IDLE -> S_REQ_LOAD) so
    // group_datapath can zero its persistent per-MPU accumulator array
    // before the first self-accumulate op. Without this, a group_top
    // instance re-triggered by a second `start` (e.g. MPE's second
    // column-chunk pass reusing the same group_top) would silently add
    // its new self-accumulate results onto the previous run's leftover
    // totals -- accumulator[] is otherwise only cleared by global rst.
    output logic accum_clear
);

    typedef enum logic [3:0] {
        S_IDLE,
        S_REQ_LOAD,
        S_WAIT_LOAD,
        S_REQ_MPU,
        S_WAIT_MPU,
        S_REQ_READOUT,
        S_WAIT_READOUT,
        S_REQ_ACCUM,
        S_WAIT_ACCUM,
        S_REQ_COMBINE,
        S_WAIT_COMBINE,
        S_DONE
    } state_t;

    state_t state, next_state;

    logic [CHUNK_BITS-1:0] chunk_cnt;    // 0 .. NUM_CHUNKS-1
    logic [MPU_BITS-1:0]   mpu_cnt;      // 0 .. NUM_MPUS-1, self-accumulate inner loop
    logic [1:0]            combine_cnt;  // 0 .. 2, the 3 final-combine stages

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
            S_WAIT_LOAD:    if (all_chunks_loaded)  next_state = S_REQ_MPU;

            S_REQ_MPU:                              next_state = S_WAIT_MPU;
            S_WAIT_MPU:     if (all_mpu_valid)      next_state = S_REQ_READOUT;

            S_REQ_READOUT:                          next_state = S_WAIT_READOUT;
            S_WAIT_READOUT: if (all_readout_done)   next_state = S_REQ_ACCUM;

            S_REQ_ACCUM:                            next_state = S_WAIT_ACCUM;
            S_WAIT_ACCUM:   if (vecadd_done) begin
                                 if (mpu_cnt != NUM_MPUS-1)
                                     next_state = S_REQ_ACCUM;
                                 else if (chunk_cnt != NUM_CHUNKS-1)
                                     next_state = S_REQ_LOAD;
                                 else
                                     next_state = S_REQ_COMBINE;
                             end

            S_REQ_COMBINE:                          next_state = S_WAIT_COMBINE;
            S_WAIT_COMBINE: if (vecadd_done) begin
                                 if (combine_cnt != 2'd2)
                                     next_state = S_REQ_COMBINE;
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
            busy         <= 1'b0;
            valid        <= 1'b0;
            load_chunk   <= 1'b0;
            chunk_idx    <= '0;
            mpu_start    <= 1'b0;
            do_readout   <= 1'b0;
            vecadd_start <= 1'b0;
            operand_sel  <= '0;
            chunk_cnt    <= '0;
            mpu_cnt      <= '0;
            combine_cnt  <= '0;
            accum_clear  <= 1'b0;
        end else begin
            // Pulse defaults -- every request pulse is exactly one
            // cycle wide unless explicitly re-asserted below.
            load_chunk   <= 1'b0;
            mpu_start    <= 1'b0;
            do_readout   <= 1'b0;
            vecadd_start <= 1'b0;
            valid        <= 1'b0;
            accum_clear  <= 1'b0;

            case (state)
                S_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        busy        <= 1'b1;
                        chunk_cnt   <= '0;
                        mpu_cnt     <= '0;
                        combine_cnt <= '0;
                        accum_clear <= 1'b1;
                    end
                end

                S_REQ_LOAD: begin
                    load_chunk <= 1'b1;
                    chunk_idx  <= chunk_cnt;
                end
                S_WAIT_LOAD: ; // just waiting on all_chunks_loaded

                S_REQ_MPU:  mpu_start <= 1'b1;
                S_WAIT_MPU: ; // just waiting on all_mpu_valid

                S_REQ_READOUT:  do_readout <= 1'b1;
                S_WAIT_READOUT: ; // just waiting on all_readout_done

                S_REQ_ACCUM: begin
                    vecadd_start <= 1'b1;
                    operand_sel  <= {1'b0, mpu_cnt};   // 0..3 -> ACC_SELF(mpu_cnt)
                end
                S_WAIT_ACCUM: begin
                    if (vecadd_done) begin
                        if (mpu_cnt != NUM_MPUS-1) begin
                            mpu_cnt <= mpu_cnt + 1'b1;
                        end else begin
                            mpu_cnt <= '0;
                            if (chunk_cnt != NUM_CHUNKS-1)
                                chunk_cnt <= chunk_cnt + 1'b1;
                        end
                    end
                end

                S_REQ_COMBINE: begin
                    vecadd_start <= 1'b1;
                    operand_sel  <= 3'd4 + {1'b0, combine_cnt};  // 4, 5, 6
                end
                S_WAIT_COMBINE: begin
                    if (vecadd_done && combine_cnt != 2'd2)
                        combine_cnt <= combine_cnt + 1'b1;
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
