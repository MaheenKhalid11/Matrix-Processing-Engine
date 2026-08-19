`timescale 1ns / 1ps
//======================================================================
// memory_controller.sv -- with double-buffered weight prefetch
//
// Design principle (fixes the earlier race): every operation gets its
// OWN dedicated one-cycle completion pulse (act_done / weights_done /
// output_done). There is no shared "data_ready" signal that different
// phases could be confused for -- each pulse is only ever asserted by
// the state that actually performed that specific operation, so the
// caller (mpu_fsm) can never read a stale pulse left over from a
// different phase.
//
// Assumes activation_bram and weight_uram are ALREADY loaded by the
// host via their Port A (this controller only ever touches Port B,
// read-only). Handles:
//   1. Streaming the full 1x256 activation vector out of BRAM once
//      per MPU run.
//   2. Streaming one column-group's 256-deep x 16-column weight slab
//      out of URAM, unpacked into 16 separate per-VPU column buffers.
//   3. Committing all 16 VPUs' results as one 512-bit write into
//      output_bram, at the correct column-group address.
//
// Weight prefetch: weight storage is DOUBLE-BUFFERED (weight_buf[0]
// and weight_buf[1]). mpu_fsm fetches column group `c+1`'s weights
// (fetch_cg_index) while the VPU array is still computing on column
// group `c` (compute_cg_index). Because fetch_cg_index and
// compute_cg_index always differ by exactly 1 while overlapped, their
// LSBs always differ, so the fetch never writes into the buffer the
// VPUs are currently reading:
//   - weight_full[] (driven to the VPUs) is a continuous mux of
//     weight_buf[compute_cg_index[0]].
//   - a fetch in flight writes into weight_buf[fetch_buf_sel], where
//     fetch_buf_sel is captured from fetch_cg_index[0] at the moment
//     load_weights pulses (held stable for that fetch's duration,
//     immune to fetch_cg_index moving on to describe the *next*
//     prefetch before this one's writes have all landed).
//======================================================================

module memory_controller #(
    parameter DATA_WIDTH  = 32,
    parameter NUM_VPUS    = 16,
    parameter DEPTH       = 256,
    localparam NUM_CGS    = DEPTH / NUM_VPUS,                // 16
    localparam CG_WIDTH   = NUM_VPUS * DATA_WIDTH,           // 512 bits
    localparam VEC_WIDTH  = DEPTH * DATA_WIDTH,               // 8192 bits
    localparam ACT_ADDR_W = $clog2(DEPTH),                    // 8 bits
    localparam WGT_ADDR_W = $clog2(NUM_CGS * DEPTH),          // 12 bits
    localparam OUT_ADDR_W = $clog2(NUM_CGS)                   // 4 bits
)(
    input  logic                      clk,
    input  logic                      rst,

    //--------------------------------------------------------------
    // Command interface -- driven by mpu_fsm. Each request has its
    // OWN dedicated completion pulse; none are shared.
    //--------------------------------------------------------------
    input  logic                      load_act,      // pulse: fetch the 1x256 activation vector
    output logic                      act_done,       // pulse: activation_full is now valid

    input  logic                      load_weights,   // pulse: fetch column-group fetch_cg_index's weights
    input  logic [OUT_ADDR_W-1:0]     fetch_cg_index,
    output logic                      weights_done,   // pulse: that fetch's buffer is now valid

    input  logic                      commit_output,  // pulse: latch vpu_result[] and write it out
    input  logic [OUT_ADDR_W-1:0]     compute_cg_index, // group currently on the VPU array / being committed
    output logic                      output_done,    // pulse: the write has landed in output_bram

    output logic                      mem_busy,       // combinational: any fetch in flight

    //--------------------------------------------------------------
    // Compute-side vectors, driven to the 16 VPUs
    //--------------------------------------------------------------
    output logic [VEC_WIDTH-1:0]      activation_full,
    output logic [VEC_WIDTH-1:0]      weight_full [0:NUM_VPUS-1],
    input  logic [DATA_WIDTH-1:0]     vpu_result  [0:NUM_VPUS-1],

    //--------------------------------------------------------------
    // Physical memory ports
    //--------------------------------------------------------------
    // activation_bram, Port B (read-only)
    output logic                      act_b_en,
    output logic [ACT_ADDR_W-1:0]     act_b_addr,
    input  logic [DATA_WIDTH-1:0]     act_b_dout,

    // weight_uram, Port B (read-only)
    output logic                      wgt_b_en,
    output logic [WGT_ADDR_W-1:0]     wgt_b_addr,
    input  logic [CG_WIDTH-1:0]       wgt_b_dout,

    // output_bram, Port A (write-only from this controller's side)
    output logic                      out_a_en,
    output logic                      out_a_we,
    output logic [OUT_ADDR_W-1:0]     out_a_addr,
    output logic [CG_WIDTH-1:0]       out_a_din
);

    //--------------------------------------------------------------
    // Protocol guard: catch mpu_fsm asserting two mutually-exclusive
    // requests at once, in simulation, before it produces silently
    // ambiguous behavior. load_weights and commit_output legitimately
    // overlap with vpu_start (that's the whole point of the prefetch),
    // but never with each other or with load_act.
    //--------------------------------------------------------------
`ifndef SYNTHESIS
    property p_mutex_commands;
        @(posedge clk) disable iff (rst)
        $onehot0({load_act, load_weights, commit_output});
    endproperty
    assert property (p_mutex_commands)
    else $error("[%0t ns] Protocol Failure: more than one memory request asserted simultaneously!", $time);
`endif

    //--------------------------------------------------------------
    // Fetch state machine -- handles load_act / load_weights only.
    // commit_output is handled entirely separately, below, since a
    // single-cycle write needs no multi-cycle sequencing.
    //--------------------------------------------------------------
    typedef enum logic [1:0] {
        IDLE,
        FETCH_ACT,
        FETCH_WGT
    } state_t;

    state_t state, next_state;

    logic [$clog2(DEPTH)-1:0]   row_cnt;   // which row is currently being ISSUED
    logic [$clog2(DEPTH)-1:0]   row_next;  // row_cnt+1, kept at full row width
    logic [$clog2(DEPTH+1)-1:0] cnt;       // which row has just been CAPTURED
    logic                        read_valid;
    logic                        fetch_buf_sel; // which weight_buf[] this fetch writes into

    // Double-buffered weight storage: weight_buf[0] and weight_buf[1].
    // See module header for the buffer-selection invariant.
    logic [VEC_WIDTH-1:0] weight_buf [0:1][0:NUM_VPUS-1];

    // Explicitly-sized successor. Do NOT inline this as logic'(row_cnt+1)
    // inside the {fetch_cg_index, row} concatenation: `logic` is a 1-bit
    // type, so that cast silently truncates the row index to its LSB.
    assign row_next = row_cnt + 1'b1;

    logic fetch_done;
    assign fetch_done = read_valid && (cnt == DEPTH - 1);

    assign mem_busy = (state != IDLE);

    //------------------------------------------------------------
    // Block 1: state register + next-state logic
    //------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) state <= IDLE;
        else     state <= next_state;
    end

    always_comb begin
        next_state = state;
        case (state)
            IDLE: begin
                if (load_act)          next_state = FETCH_ACT;
                else if (load_weights) next_state = FETCH_WGT;
            end
            FETCH_ACT: if (fetch_done) next_state = IDLE;
            FETCH_WGT: if (fetch_done) next_state = IDLE;
            default:                   next_state = IDLE;
        endcase
    end

    //------------------------------------------------------------
    // Block 2: the two completion pulses -- each is gated on ITS
    // OWN state, so there is no way to read one operation's pulse
    // as though it belonged to the other.
    //------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            act_done     <= 1'b0;
            weights_done <= 1'b0;
        end else begin
            act_done     <= fetch_done && (state == FETCH_ACT);
            weights_done <= fetch_done && (state == FETCH_WGT);
        end
    end

    //------------------------------------------------------------
    // Block 3: address generator (adderless -- valid because DEPTH
    // and NUM_CGS are both powers of two, so {fetch_cg_index,row} ==
    // fetch_cg_index*DEPTH + row exactly).
    //------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            act_b_en      <= 1'b0;
            act_b_addr    <= '0;
            wgt_b_en      <= 1'b0;
            wgt_b_addr    <= '0;
            row_cnt       <= '0;
            fetch_buf_sel <= 1'b0;
        end else begin
            case (state)
                IDLE: begin
                    row_cnt <= '0;
                    if (load_act) begin
                        act_b_en   <= 1'b1;
                        act_b_addr <= '0;
                        wgt_b_en   <= 1'b0;
                    end else if (load_weights) begin
                        wgt_b_en      <= 1'b1;
                        wgt_b_addr    <= {fetch_cg_index, {$clog2(DEPTH){1'b0}}};
                        fetch_buf_sel <= fetch_cg_index[0]; // captured for the whole fetch
                        act_b_en      <= 1'b0;
                    end else begin
                        act_b_en <= 1'b0;
                        wgt_b_en <= 1'b0;
                    end
                end

                FETCH_ACT: begin
                    if (act_b_en) begin
                        if (row_cnt == DEPTH - 1) begin
                            act_b_en <= 1'b0;
                        end else begin
                            row_cnt    <= row_next;
                            act_b_addr <= row_next;
                        end
                    end
                end

                FETCH_WGT: begin
                    if (wgt_b_en) begin
                        if (row_cnt == DEPTH - 1) begin
                            wgt_b_en <= 1'b0;
                        end else begin
                            row_cnt    <= row_next;
                            wgt_b_addr <= {fetch_cg_index, row_next};
                        end
                    end
                end

                default: begin
                    act_b_en <= 1'b0;
                    wgt_b_en <= 1'b0;
                end
            endcase
        end
    end

    // 1-cycle read pipeline (matches BRAM/URAM's own registered read latency)
    always_ff @(posedge clk) begin
        if (rst) read_valid <= 1'b0;
        else     read_valid <= act_b_en | wgt_b_en;
    end

    // Captured-word counter -- always matches the index whose data
    // has JUST arrived on act_b_dout/wgt_b_dout this cycle.
    always_ff @(posedge clk) begin
        if (rst || state == IDLE) cnt <= '0;
        else if (read_valid)      cnt <= cnt + 1'b1;
    end

    //------------------------------------------------------------
    // Block 4: vector assemblers
    //------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (read_valid && (state == FETCH_ACT)) begin
            activation_full[cnt*DATA_WIDTH +: DATA_WIDTH] <= act_b_dout;
        end
    end

    always_ff @(posedge clk) begin
        if (read_valid && (state == FETCH_WGT)) begin
            for (int v = 0; v < NUM_VPUS; v++) begin
                weight_buf[fetch_buf_sel][v][cnt*DATA_WIDTH +: DATA_WIDTH] <= wgt_b_dout[v*DATA_WIDTH +: DATA_WIDTH];
            end
        end
    end

    // weight_full[] always presents the buffer holding compute_cg_index's
    // weights -- the buffer NOT being written by any fetch in flight,
    // since fetch_cg_index (while overlapped) is always compute_cg_index+1
    // and their LSBs therefore always differ.
    generate
        for (genvar v = 0; v < NUM_VPUS; v++) begin : gen_weight_mux
            assign weight_full[v] = weight_buf[compute_cg_index[0]][v];
        end
    endgenerate

    //------------------------------------------------------------
    // Block 5: output commit -- entirely independent of the fetch
    // state machine above (a write needs no multi-cycle sequencing).
    // output_done is a clean 1-cycle-delayed pulse confirming the
    // write has landed, so the FSM has a definitive "safe to
    // proceed" signal rather than assuming write timing.
    //------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            out_a_en    <= 1'b0;
            out_a_we    <= 1'b0;
            out_a_addr  <= '0;
            out_a_din   <= '0;
            output_done <= 1'b0;
        end else begin
            output_done <= 1'b0;

            if (commit_output) begin
                out_a_en   <= 1'b1;
                out_a_we   <= 1'b1;
                out_a_addr <= compute_cg_index;
                for (int i = 0; i < NUM_VPUS; i++) begin
                    out_a_din[i*DATA_WIDTH +: DATA_WIDTH] <= vpu_result[i];
                end
                output_done <= 1'b1; // write completes this same registered edge
            end else begin
                out_a_en <= 1'b0;
                out_a_we <= 1'b0;
            end
        end
    end

endmodule
