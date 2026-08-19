`timescale 1ns/1ps
//======================================================================
// vpu_fsm  --  CONTROL PATH ONLY
//
// Owns all state: the 4-state IDLE/LOAD/RUN/CAPTURE sequencing, the
// priming workaround for a cold DSP chain, and every registered output
// (busy/valid/dot_product/pass_cnt/acc_reg/ce_*). It has no knowledge
// of DSPs -- it watches chain_out come back from dsp_cascade and drives
// ce_a/ce_b/ce_pipe/pass_cnt/acc_reg out to it.
//
// Structure follows Cliff Cummings' 3-always-block FSM style:
//   1) state register             (always_ff)
//   2) next-state comb. logic     (always_comb)
//   3) registered outputs/data    (always_ff, keyed on state)
//
// Block 3 is keyed on the CURRENT `state`, not `next_state`. An
// earlier version of this FSM keyed it on `next_state` to shave a
// cycle of output latency, but that breaks here: pass_cnt and priming
// are both read AND mutated inside this same block to decide "was
// that the last pass" / "was that the priming pass". Keying on
// next_state mutates them one cycle before the state register catches
// up, so by the time next_state==CAPTURE is evaluated for the
// following transition, it's reading a pass_cnt/priming that block 3
// already advanced -- the last-pass and priming checks fire a cycle
// early and the FSM exits to IDLE before the real final pass ever
// runs. Keying on `state` avoids this: it reproduces the exact timing
// of the original single-always-block design, just split into three
// blocks for readability.
//======================================================================
module vpu_fsm #(
    parameter DEPTH        = 256,
    parameter K            = 16,
    parameter PIPE_LATENCY = 20
)(
    input  logic         clk,
    input  logic         rst,

    input  logic         start,
    input  logic [31:0]  chain_out,       // from dsp_cascade

    output logic         ce_a,
    output logic         ce_b,
    output logic         ce_pipe,

    output logic         busy,
    output logic         valid,

    output logic [31:0]                 acc_reg,    // -> dsp_cascade (pc[0] source)
    output logic [$clog2(DEPTH/K)-1:0]  pass_cnt,    // -> dsp_cascade (slice index)

    output logic [31:0]  dot_product
);

    localparam NUM_PASSES = DEPTH / K;

    typedef enum logic [1:0] {
        IDLE,
        LOAD,
        RUN,
        CAPTURE
    } state_t;

    state_t state, next_state;

    logic [$clog2(PIPE_LATENCY+1)-1:0] wait_cnt;
    logic                               priming;

    // ------------------------------------------------------------
    // 1) state register
    // ------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst)
            state <= IDLE;
        else
            state <= next_state;
    end

    // ------------------------------------------------------------
    // 2) next-state logic (no output assignments here)
    // ------------------------------------------------------------
    always_comb begin
        next_state = state;

        case (state)
            IDLE:    if (start) next_state = LOAD;
                    
            LOAD:    next_state = RUN;
                    
            RUN:     if (wait_cnt == PIPE_LATENCY) next_state = CAPTURE;

            CAPTURE: if (priming)                    next_state = LOAD;
                     else if (pass_cnt == NUM_PASSES-1) next_state = IDLE;
                     else                              next_state = LOAD;
        endcase
    end

    // ------------------------------------------------------------
    // 3) registered outputs & datapath control registers
    // ------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            busy        <= 1'b0;
            valid       <= 1'b0;

            ce_a        <= 1'b0;
            ce_b        <= 1'b0;
            ce_pipe     <= 1'b0;

            pass_cnt    <= '0;
            wait_cnt    <= '0;

            acc_reg     <= 32'b0;

            priming     <= 1'b0;

            dot_product <= 32'b0;
        end
        else begin
            valid <= 1'b0;   // default; only pulses in CAPTURE on the final real pass

            case (state)

                IDLE: begin
                    busy    <= 1'b0;
                    ce_a    <= 1'b0;
                    ce_b    <= 1'b0;
                    ce_pipe <= 1'b0;

                    if (start) begin
                        busy     <= 1'b1;
                        pass_cnt <= '0;
                        wait_cnt <= '0;
                        acc_reg  <= 32'b0;
                        priming  <= 1'b1;
                    end
                end

                LOAD: begin
                    ce_a     <= 1'b1;
                    ce_b     <= 1'b1;
                    ce_pipe  <= 1'b1;
                    wait_cnt <= '0;
                end

                RUN: begin
                    ce_a     <= 1'b0;
                    ce_b     <= 1'b0;
                    ce_pipe  <= 1'b1;
                    if (wait_cnt != PIPE_LATENCY)
                        wait_cnt <= wait_cnt + 1'b1;
                end

                CAPTURE: begin
                    ce_a    <= 1'b0;
                    ce_b    <= 1'b0;
                    ce_pipe <= 1'b1;

                    if (priming) begin
                        // throwaway pass: discard chain_out entirely,
                        // leave acc_reg/pass_cnt untouched
                        priming <= 1'b0;
                    end
                    else begin
                        acc_reg <= chain_out;

                        if (pass_cnt == NUM_PASSES-1) begin
                            dot_product <= chain_out;
                            valid       <= 1'b1;
                            busy        <= 1'b0;
                        end
                        else begin
                            pass_cnt <= pass_cnt + 1'b1;
                        end
                    end
                end

            endcase
        end
    end

endmodule