`timescale 1ns / 1ps
//======================================================================
// vecadd_fsm  --  CONTROL PATH ONLY
//
// Drives accum_adder16 to add two ROWS-row (ROWS x 512-bit) vectors
// together, one 512-bit/16-column row per cycle.
//
// SERIALIZED, not pipelined -- and that's deliberate, learned the hard
// way. An earlier version streamed a new row_sel every cycle (rows
// are independent, so nothing seemed to forbid it), but that's wrong:
// pcin (a_vec) fed the DSP's adder stage directly from this FSM's mux,
// while the multiply (b_vec * 1.0) takes several cycles to settle
// internally. By the time the add actually happened, pcin had already
// raced ahead to a LATER row's value -- silently computing
// b[row N] + a[row N+shift] instead of a[N]+b[N]. dsp_cascade.sv never
// hits this because its pcin comes from a NEIGHBOR DSP's own
// registered pcout, which stays stable for an entire pass simply
// because nothing upstream changes it until the next pass. This FSM
// reproduces that same stability directly: row_sel (and therefore
// a_vec/b_vec) is held constant for the full LOAD/RUN/CAPTURE window
// of one row before the next row's data is ever presented.
//
// Priming: accum_adder16 wraps the SAME dspfp32_pe primitive vpu_fsm
// drives, and vpu_fsm's own header documents that a "cold" (recently
// idle, ce_pipe held low) DSPFP32 chain needs one throwaway pass
// before its output is trustworthy. Reproduced here exactly the same
// way vpu_fsm does it: row 0 runs once as a throwaway, discarded, then
// every row 0..ROWS-1 runs for real.
//
// ADD_LATENCY: cycles to hold RUN before capturing -- see vpu.sv for
// how this is derived/verified for the general case; use vecadd_tb.sv
// to bisect the value for this FSM specifically now that it's
// serialized (a single DSP settling from a stable input, no cascade
// ripple, so it should track PIPE_LATENCY's own single-MAC derivation
// again).
//
// Row data itself (a_vec/b_vec muxing, sum_vec capture into the right
// destination register) is NOT handled here -- this FSM only drives
// row_sel (an index) and ce_a/ce_b/ce_pipe/capture_valid/
// capture_row_sel; the wide 512-bit buses are muxed entirely in
// group_datapath, same separation principle as mpu_fsm never touching
// activation_full/weight_full/vpu_result directly.
//======================================================================
module vecadd_fsm #(
    parameter ROWS        = 16,
    parameter ADD_LATENCY = 8,
    localparam ROW_BITS   = $clog2(ROWS)
)(
    input  logic                  clk,
    input  logic                  rst,

    input  logic                  start,
    output logic                  busy,
    output logic                  valid,           // pulses 1 cycle when all ROWS rows are captured

    output logic                  ce_a,
    output logic                  ce_b,
    output logic                  ce_pipe,
    output logic [ROW_BITS-1:0]   row_sel,         // which row is on a_vec/b_vec -- held stable through LOAD/RUN/CAPTURE

    output logic                  capture_valid,   // pulses when a row's sum_vec is valid this cycle
    output logic [ROW_BITS-1:0]   capture_row_sel  // which row that valid sum belongs to
);

    typedef enum logic [1:0] {
        IDLE,
        LOAD,
        RUN,
        CAPTURE
    } state_t;

    state_t state, next_state;

    logic [ROW_BITS-1:0]            row_cnt;
    logic [$clog2(ADD_LATENCY+1)-1:0] wait_cnt;
    logic                            priming;

    //------------------------------------------------------------
    // 1) state register
    //------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) state <= IDLE;
        else     state <= next_state;
    end

    //------------------------------------------------------------
    // 2) next-state logic
    //------------------------------------------------------------
    always_comb begin
        next_state = state;
        case (state)
            IDLE:    if (start) next_state = LOAD;

            LOAD:    next_state = RUN;

            RUN:     if (wait_cnt == ADD_LATENCY) next_state = CAPTURE;

            CAPTURE: if (priming)                  next_state = LOAD;
                     else if (row_cnt == ROWS-1)    next_state = IDLE;
                     else                           next_state = LOAD;

            default: next_state = IDLE;
        endcase
    end

    //------------------------------------------------------------
    // 3) registered outputs & datapath control registers
    //------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            busy            <= 1'b0;
            valid           <= 1'b0;
            ce_a            <= 1'b0;
            ce_b            <= 1'b0;
            ce_pipe         <= 1'b0;
            row_sel         <= '0;
            row_cnt         <= '0;
            wait_cnt        <= '0;
            priming         <= 1'b0;
            capture_valid   <= 1'b0;
            capture_row_sel <= '0;
        end else begin
            valid         <= 1'b0;   // default; only pulses in CAPTURE on the final real row
            capture_valid <= 1'b0;   // default; only pulses in CAPTURE on a real (non-priming) row

            case (state)
                IDLE: begin
                    busy    <= 1'b0;
                    ce_a    <= 1'b0;
                    ce_b    <= 1'b0;
                    ce_pipe <= 1'b0;

                    if (start) begin
                        busy    <= 1'b1;
                        row_cnt <= '0;
                        row_sel <= '0;
                        priming <= 1'b1;
                    end
                end

                LOAD: begin
                    ce_a     <= 1'b1;
                    ce_b     <= 1'b1;
                    ce_pipe  <= 1'b1;
                    wait_cnt <= '0;
                    row_sel  <= row_cnt;   // held stable through RUN/CAPTURE
                end

                RUN: begin
                    ce_a    <= 1'b0;
                    ce_b    <= 1'b0;
                    ce_pipe <= 1'b1;
                    if (wait_cnt != ADD_LATENCY)
                        wait_cnt <= wait_cnt + 1'b1;
                end

                CAPTURE: begin
                    ce_a    <= 1'b0;
                    ce_b    <= 1'b0;
                    ce_pipe <= 1'b1;

                    if (priming) begin
                        // throwaway row: discard the result entirely,
                        // leave row_cnt untouched, redo row 0 for real
                        priming <= 1'b0;
                    end else begin
                        capture_valid   <= 1'b1;
                        capture_row_sel <= row_cnt;

                        if (row_cnt == ROWS-1) begin
                            busy  <= 1'b0;
                            valid <= 1'b1;
                        end else begin
                            row_cnt <= row_cnt + 1'b1;
                        end
                    end
                end

                default: ;
            endcase
        end
    end

endmodule
