`timescale 1ns / 1ps
//======================================================================
// group_datapath.sv -- subordinate data mover for group_top, same role
// one level up that memory_controller plays inside mpu_top: owns every
// wide data bus, does all the streaming/sequencing work, and only
// exchanges simple pulses + small index/select codes with group_fsm.
//
// Owns, per MPU (NUM_MPUS = 4, unmodified mpu_top instances):
//   - a weight chunk store (raw uram_tdp_init, holds all NUM_CHUNKS
//     chunks' 256x256 tiles for that MPU, ~1 MiB) and an activation
//     chunk store (reused activation_bram.sv, DEPTH=NUM_CHUNKS*MPU_DEPTH)
//     -- both preloaded once from the host via their own Port A.
//   - a "loader": streams the chunk selected by chunk_idx out of those
//     stores and into that mpu_top's own host_act/host_wgt ports
//     (mirrors memory_controller's own FETCH_ACT/FETCH_WGT streaming,
//     just writing outward into a downstream mpu_top instead of
//     assembling a register).
//   - a "readout": after that mpu_top's `valid` pulses, streams its
//     16 CG-rows out of its own output_bram into a plain register
//     array (mpu_readout[mpu]) -- mirrors the same fetch-into-register
//     pattern used for activation_full in memory_controller.
//   - an accumulator register array (16 x 512-bit), self-added-into
//     once per chunk.
//
// All 4 MPUs' loaders and readouts run in full parallel (each owns
// its own physical stores/ports, no sharing -- true parallelism, not
// time-multiplexed). The actual FP add hardware (accum_adder16 +
// vecadd_fsm) is a SINGLE shared bank, reused serially across the 4
// self-accumulate ops per chunk and the 3 final-combine stages --
// add time is negligible next to compute time, so sharing costs
// latency, not throughput, in exchange for 4x less adder hardware.
//
// operand_sel encoding (driven by group_fsm, consumed here to mux the
// shared vecadd_fsm's a_vec/b_vec sources and sum_vec destination):
//   0..3 : ACC_SELF(mpu)   a=accumulator[mpu],  b=mpu_readout[mpu],  dest=accumulator[mpu]
//   4    : COMBINE1        a=accumulator[0],    b=accumulator[1],   dest=combine_temp
//   5    : COMBINE2        a=combine_temp,      b=accumulator[2],   dest=combine_temp
//   6    : COMBINE3        a=combine_temp,      b=accumulator[3],   dest=output_bram (direct write)
//======================================================================

module group_datapath #(
    parameter DATA_WIDTH   = 32,
    parameter NUM_VPUS     = 16,
    parameter MPU_DEPTH    = 256,
    parameter K            = 16,
    parameter PIPE_LATENCY = 24,
    parameter NUM_MPUS     = 4,
    parameter NUM_CHUNKS   = 4,
    parameter ADD_LATENCY  = 12,

    localparam NUM_CGS          = MPU_DEPTH / NUM_VPUS,                        // 16
    localparam CG_WIDTH         = NUM_VPUS * DATA_WIDTH,                       // 512
    localparam ACT_ADDR_W       = $clog2(MPU_DEPTH),                          // 8
    localparam WGT_ADDR_W       = $clog2(NUM_CGS * MPU_DEPTH),                // 12
    localparam OUT_ADDR_W       = $clog2(NUM_CGS),                            // 4
    localparam CHUNK_BITS       = $clog2(NUM_CHUNKS),                         // 2
    localparam ACT_CHUNK_ADDR_W = $clog2(NUM_CHUNKS * MPU_DEPTH),             // 10
    localparam WGT_CHUNK_ADDR_W = $clog2(NUM_CHUNKS * NUM_CGS * MPU_DEPTH)    // 14
)(
    input  logic clk,
    input  logic rst,

    //--------------------------------------------------------------
    // Host preload, per MPU (Port A of each chunk store)
    //--------------------------------------------------------------
    input  logic                        grp_act_en   [0:NUM_MPUS-1],
    input  logic                        grp_act_we   [0:NUM_MPUS-1],
    input  logic [ACT_CHUNK_ADDR_W-1:0] grp_act_addr [0:NUM_MPUS-1],
    input  logic [DATA_WIDTH-1:0]       grp_act_din  [0:NUM_MPUS-1],

    input  logic                        grp_wgt_en   [0:NUM_MPUS-1],
    input  logic                        grp_wgt_we   [0:NUM_MPUS-1],
    input  logic [WGT_CHUNK_ADDR_W-1:0] grp_wgt_addr [0:NUM_MPUS-1],
    input  logic [CG_WIDTH-1:0]         grp_wgt_din  [0:NUM_MPUS-1],

    // Group's final combined output (Port B, host read)
    input  logic                  grp_out_en,
    input  logic [OUT_ADDR_W-1:0] grp_out_addr,
    output logic [CG_WIDTH-1:0]   grp_out_dout,

    //--------------------------------------------------------------
    // Command interface -- driven by group_fsm
    //--------------------------------------------------------------
    input  logic                  load_chunk,
    input  logic [CHUNK_BITS-1:0] chunk_idx,
    output logic                  all_chunks_loaded,

    input  logic                  mpu_start,
    output logic                  all_mpu_valid,

    input  logic                  do_readout,
    output logic                  all_readout_done,

    input  logic                  vecadd_start,
    input  logic [2:0]            operand_sel,
    output logic                  vecadd_done,

    // Pulsed by group_fsm at the start of every run -- zeroes the
    // persistent per-MPU accumulator array so a re-triggered group_top
    // (e.g. MPE's second column-chunk pass) doesn't add its results
    // onto the previous run's leftover totals.
    input  logic                  accum_clear
);

    // Per-MPU status, collected across the generate loop below.
    logic [NUM_MPUS-1:0] mpu_valid_arr;
    logic [NUM_MPUS-1:0] loader_done_arr;
    logic [NUM_MPUS-1:0] readout_done_arr;

    assign all_mpu_valid     = &mpu_valid_arr;
    assign all_chunks_loaded = &loader_done_arr;
    assign all_readout_done  = &readout_done_arr;

    // Shared state read by the operand mux below (module-scope, not
    // nested in the generate loop, so the mux can index into it with
    // operand_sel at runtime).
    logic [CG_WIDTH-1:0] accumulator [0:NUM_MPUS-1][0:NUM_CGS-1];
    logic [CG_WIDTH-1:0] mpu_readout [0:NUM_MPUS-1][0:NUM_CGS-1];
    logic [CG_WIDTH-1:0] combine_temp [0:NUM_CGS-1];

    //================================================================
    // Per-MPU: chunk stores, loader, mpu_top, readout
    //================================================================
    genvar mpu;
    generate
    for (mpu = 0; mpu < NUM_MPUS; mpu = mpu + 1) begin : gen_mpu

        //------------------------------------------------------------
        // Chunk stores -- Port A host preload, Port B internal stream
        //------------------------------------------------------------
        logic                        wgt_chunk_b_en;
        logic [WGT_CHUNK_ADDR_W-1:0] wgt_chunk_b_addr;
        logic [CG_WIDTH-1:0]         wgt_chunk_b_dout;

        uram_tdp_init #(
            .DATA_WIDTH ( CG_WIDTH         ),
            .ADDR_WIDTH ( WGT_CHUNK_ADDR_W ),
            .INIT_FILE  ( ""               )
        ) u_wgt_chunk_store (
            .clk    ( clk                    ),
            .en_a   ( grp_wgt_en[mpu]        ),
            .we_a   ( grp_wgt_we[mpu]        ),
            .addr_a ( grp_wgt_addr[mpu]      ),
            .din_a  ( grp_wgt_din[mpu]       ),
            .dout_a (                        ),
            .en_b   ( wgt_chunk_b_en         ),
            .we_b   ( 1'b0                   ),
            .addr_b ( wgt_chunk_b_addr       ),
            .din_b  ( '0                     ),
            .dout_b ( wgt_chunk_b_dout       )
        );

        logic                        act_chunk_b_en;
        logic [ACT_CHUNK_ADDR_W-1:0] act_chunk_b_addr;
        logic [DATA_WIDTH-1:0]       act_chunk_b_dout;

        activation_bram #(
            .DATA_WIDTH ( DATA_WIDTH               ),
            .DEPTH      ( NUM_CHUNKS * MPU_DEPTH    ),
            .INIT_FILE  ( ""                        )
        ) u_act_chunk_store (
            .clk    ( clk                    ),
            .a_en   ( grp_act_en[mpu]        ),
            .a_we   ( grp_act_we[mpu]        ),
            .a_addr ( grp_act_addr[mpu]      ),
            .a_din  ( grp_act_din[mpu]       ),
            .a_dout (                        ),
            .b_en   ( act_chunk_b_en         ),
            .b_addr ( act_chunk_b_addr       ),
            .b_dout ( act_chunk_b_dout       )
        );

        //------------------------------------------------------------
        // mpu_top -- unmodified, its host ports driven internally by
        // this MPU's own loader/readout, never exposed externally.
        //------------------------------------------------------------
        logic                    mt_host_act_en, mt_host_act_we;
        logic [ACT_ADDR_W-1:0]   mt_host_act_addr;
        logic [DATA_WIDTH-1:0]   mt_host_act_din;

        logic                    mt_host_wgt_en, mt_host_wgt_we;
        logic [WGT_ADDR_W-1:0]   mt_host_wgt_addr;
        logic [CG_WIDTH-1:0]     mt_host_wgt_din;

        logic                    mt_host_out_en;
        logic [OUT_ADDR_W-1:0]   mt_host_out_addr;
        logic [CG_WIDTH-1:0]     mt_host_out_dout;

        logic mt_busy, mt_valid;

        mpu_top #(
            .DATA_WIDTH   ( DATA_WIDTH   ),
            .NUM_VPUS     ( NUM_VPUS     ),
            .DEPTH        ( MPU_DEPTH    ),
            .K            ( K            ),
            .PIPE_LATENCY ( PIPE_LATENCY ),
            .ACT_INIT     ( ""           ),
            .WGT_INIT     ( ""           )
        ) u_mpu (
            .clk           ( clk               ),
            .rst           ( rst               ),
            .start         ( mpu_start         ),
            .busy          ( mt_busy           ),
            .valid         ( mt_valid          ),
            .host_act_en   ( mt_host_act_en    ),
            .host_act_we   ( mt_host_act_we    ),
            .host_act_addr ( mt_host_act_addr  ),
            .host_act_din  ( mt_host_act_din   ),
            .host_wgt_en   ( mt_host_wgt_en    ),
            .host_wgt_we   ( mt_host_wgt_we    ),
            .host_wgt_addr ( mt_host_wgt_addr  ),
            .host_wgt_din  ( mt_host_wgt_din   ),
            .host_out_en   ( mt_host_out_en    ),
            .host_out_addr ( mt_host_out_addr  ),
            .host_out_dout ( mt_host_out_dout  )
        );

        assign mpu_valid_arr[mpu] = mt_valid;

        //------------------------------------------------------------
        // Loader: on load_chunk, stream chunk_idx's activation (256
        // words) then weights (NUM_CGS*MPU_DEPTH rows) from the chunk
        // stores into this mpu_top's host ports. Mirrors
        // memory_controller's row_cnt/cnt/read_valid pattern exactly,
        // just writing outward instead of assembling a register.
        //------------------------------------------------------------
        // L_WGT_START is a dedicated one-cycle priming state for the
        // weight fetch, mirroring the role L_IDLE plays for the
        // activation fetch. Priming wgt_chunk_b_addr=0 directly inside
        // L_ACT's last-iteration branch (the original approach) primes
        // the address one cycle before lstate actually reaches L_WGT
        // (the transition itself waits on the CAPTURE side catching up,
        // not the issue side), so the address sits stagnant at 0 for
        // an extra cycle before L_WGT's own advance logic starts --
        // sampling row 0 twice and losing the real last row. Splitting
        // priming into its own state gives weight fetch the exact same
        // "prime once, advance from the next cycle" timing the
        // activation fetch already gets for free from L_IDLE.
        typedef enum logic [1:0] { L_IDLE, L_ACT, L_WGT_START, L_WGT } lstate_t;
        lstate_t lstate, lstate_next;

        logic [WGT_ADDR_W-1:0] liss;      // issue index within the current sub-phase
        logic [WGT_ADDR_W-1:0] liss_next; // explicitly-sized liss+1 -- do NOT inline
                                           // this as (liss+1'b1)[N-1:0]; Vivado's
                                           // parser rejects a part-select applied
                                           // directly to an arithmetic expression
        logic [WGT_ADDR_W-1:0] lcap;      // capture index (issue index delayed by 1)
        logic                  lread_valid;

        assign liss_next = liss + 1'b1;

        always_ff @(posedge clk) begin
            if (rst) lstate <= L_IDLE;
            else     lstate <= lstate_next;
        end

        always_comb begin
            lstate_next = lstate;
            case (lstate)
                L_IDLE:      if (load_chunk) lstate_next = L_ACT;
                L_ACT:       if (lread_valid && lcap == MPU_DEPTH-1) lstate_next = L_WGT_START;
                L_WGT_START:                                        lstate_next = L_WGT;
                L_WGT:       if (lread_valid && lcap == (NUM_CGS*MPU_DEPTH)-1) lstate_next = L_IDLE;
                default:     lstate_next = L_IDLE;
            endcase
        end

        // Issue-side address generator
        always_ff @(posedge clk) begin
            if (rst) begin
                act_chunk_b_en   <= 1'b0;
                act_chunk_b_addr <= '0;
                wgt_chunk_b_en   <= 1'b0;
                wgt_chunk_b_addr <= '0;
                liss             <= '0;
            end else begin
                case (lstate)
                    L_IDLE: begin
                        liss <= '0;
                        if (load_chunk) begin
                            act_chunk_b_en   <= 1'b1;
                            act_chunk_b_addr <= {chunk_idx, {ACT_ADDR_W{1'b0}}};
                            wgt_chunk_b_en   <= 1'b0;
                        end else begin
                            act_chunk_b_en <= 1'b0;
                            wgt_chunk_b_en <= 1'b0;
                        end
                    end

                    L_ACT: begin
                        if (act_chunk_b_en) begin
                            if (liss == MPU_DEPTH-1) begin
                                act_chunk_b_en <= 1'b0;
                                // Weight fetch's first address is now
                                // primed in L_WGT_START, not here --
                                // see that state's own comment.
                            end else begin
                                liss             <= liss_next;
                                act_chunk_b_addr <= {chunk_idx, liss_next[ACT_ADDR_W-1:0]};
                            end
                        end
                    end

                    L_WGT_START: begin
                        wgt_chunk_b_en   <= 1'b1;
                        wgt_chunk_b_addr <= {chunk_idx, {WGT_ADDR_W{1'b0}}};
                        liss             <= '0;
                    end

                    L_WGT: begin
                        if (wgt_chunk_b_en) begin
                            if (liss == (NUM_CGS*MPU_DEPTH)-1) begin
                                wgt_chunk_b_en <= 1'b0;
                            end else begin
                                liss             <= liss_next;
                                wgt_chunk_b_addr <= {chunk_idx, liss_next[WGT_ADDR_W-1:0]};
                            end
                        end
                    end

                    default: begin
                        act_chunk_b_en <= 1'b0;
                        wgt_chunk_b_en <= 1'b0;
                    end
                endcase
            end
        end

        always_ff @(posedge clk) begin
            if (rst) lread_valid <= 1'b0;
            else     lread_valid <= act_chunk_b_en | wgt_chunk_b_en;
        end

        // Capture-index resets both at the very start AND at the
        // ACT->WGT boundary, since WGT re-issues its own 0..N-1
        // address range from scratch.
        always_ff @(posedge clk) begin
            if (rst || lstate == L_IDLE)
                lcap <= '0;
            else if (lstate == L_ACT && lread_valid && lcap == MPU_DEPTH-1)
                lcap <= '0;
            else if (lread_valid)
                lcap <= lcap + 1'b1;
        end

        // Capture-side: write the just-arrived word into mpu_top's
        // host port. `lstate` is still L_ACT during ACT's own final
        // capture cycle (the transition condition itself requires
        // that capture to already be happening), so this gating is
        // exact -- same property memory_controller relies on.
        always_ff @(posedge clk) begin
            if (rst) begin
                mt_host_act_en <= 1'b0; mt_host_act_we <= 1'b0;
                mt_host_act_addr <= '0; mt_host_act_din <= '0;
                mt_host_wgt_en <= 1'b0; mt_host_wgt_we <= 1'b0;
                mt_host_wgt_addr <= '0; mt_host_wgt_din <= '0;
            end else begin
                mt_host_act_en <= 1'b0; mt_host_act_we <= 1'b0;
                mt_host_wgt_en <= 1'b0; mt_host_wgt_we <= 1'b0;

                if (lread_valid && lstate == L_ACT) begin
                    mt_host_act_en   <= 1'b1;
                    mt_host_act_we   <= 1'b1;
                    mt_host_act_addr <= lcap[ACT_ADDR_W-1:0];
                    mt_host_act_din  <= act_chunk_b_dout;
                end else if (lread_valid && lstate == L_WGT) begin
                    mt_host_wgt_en   <= 1'b1;
                    mt_host_wgt_we   <= 1'b1;
                    mt_host_wgt_addr <= lcap[WGT_ADDR_W-1:0];
                    mt_host_wgt_din  <= wgt_chunk_b_dout;
                end
            end
        end

        assign loader_done_arr[mpu] = lread_valid && lstate == L_WGT && (lcap == (NUM_CGS*MPU_DEPTH)-1);

        //------------------------------------------------------------
        // Readout: after mt_valid, stream this MPU's own output_bram
        // (16 CG-rows) into mpu_readout[mpu]. Mirrors
        // memory_controller's single-phase fetch-into-register.
        //------------------------------------------------------------
        typedef enum logic { R_IDLE, R_RUN } rstate_t;
        rstate_t rstate, rstate_next;

        logic [OUT_ADDR_W-1:0] riss;
        logic [OUT_ADDR_W-1:0] rcap;
        logic                  rread_valid;

        always_ff @(posedge clk) begin
            if (rst) rstate <= R_IDLE;
            else     rstate <= rstate_next;
        end

        always_comb begin
            rstate_next = rstate;
            case (rstate)
                R_IDLE: if (do_readout) rstate_next = R_RUN;
                R_RUN:  if (rread_valid && rcap == NUM_CGS-1) rstate_next = R_IDLE;
                default: rstate_next = R_IDLE;
            endcase
        end

        always_ff @(posedge clk) begin
            if (rst) begin
                mt_host_out_en   <= 1'b0;
                mt_host_out_addr <= '0;
                riss             <= '0;
            end else begin
                case (rstate)
                    R_IDLE: begin
                        riss <= '0;
                        if (do_readout) begin
                            mt_host_out_en   <= 1'b1;
                            mt_host_out_addr <= '0;
                        end else begin
                            mt_host_out_en <= 1'b0;
                        end
                    end
                    R_RUN: begin
                        if (mt_host_out_en) begin
                            if (riss == NUM_CGS-1) begin
                                mt_host_out_en <= 1'b0;
                            end else begin
                                riss             <= riss + 1'b1;
                                mt_host_out_addr <= riss + 1'b1;
                            end
                        end
                    end
                    default: mt_host_out_en <= 1'b0;
                endcase
            end
        end

        always_ff @(posedge clk) begin
            if (rst) rread_valid <= 1'b0;
            else     rread_valid <= mt_host_out_en;
        end

        always_ff @(posedge clk) begin
            if (rst || rstate == R_IDLE) rcap <= '0;
            else if (rread_valid)        rcap <= rcap + 1'b1;
        end

        always_ff @(posedge clk) begin
            if (rread_valid) mpu_readout[mpu][rcap] <= mt_host_out_dout;
        end

        assign readout_done_arr[mpu] = rread_valid && rstate == R_RUN && rcap == NUM_CGS-1;

    end
    endgenerate

    //================================================================
    // Shared vector-adder: one accum_adder16 + one vecadd_fsm, reused
    // serially across the 4 self-accumulate ops (per chunk) and the 3
    // final-combine stages. group_fsm selects the operation via
    // operand_sel; all wide-bus muxing lives here.
    //================================================================
    logic [CG_WIDTH-1:0]   va_a_vec, va_b_vec, va_sum_vec;
    logic                  va_ce_a, va_ce_b, va_ce_pipe;
    logic [OUT_ADDR_W-1:0] va_row_sel, va_capture_row_sel;
    logic                  va_capture_valid;

    accum_adder16 u_accum_adder (
        .clk     ( clk        ),
        .rst     ( rst        ),
        .ce_a    ( va_ce_a    ),
        .ce_b    ( va_ce_b    ),
        .ce_pipe ( va_ce_pipe ),
        .a_vec   ( va_a_vec   ),
        .b_vec   ( va_b_vec   ),
        .sum_vec ( va_sum_vec )
    );

    vecadd_fsm #(
        .ROWS        ( NUM_CGS     ),
        .ADD_LATENCY ( ADD_LATENCY )
    ) u_vecadd_fsm (
        .clk             ( clk                 ),
        .rst             ( rst                 ),
        .start           ( vecadd_start        ),
        .busy            (                     ),
        .valid           ( vecadd_done         ),
        .ce_a            ( va_ce_a             ),
        .ce_b            ( va_ce_b             ),
        .ce_pipe         ( va_ce_pipe          ),
        .row_sel         ( va_row_sel          ),
        .capture_valid   ( va_capture_valid    ),
        .capture_row_sel ( va_capture_row_sel  )
    );

    // Issue-side operand mux (see operand_sel encoding in header comment).
    always_comb begin
        case (operand_sel)
            3'd0:    begin va_a_vec = accumulator[0][va_row_sel]; va_b_vec = mpu_readout[0][va_row_sel]; end
            3'd1:    begin va_a_vec = accumulator[1][va_row_sel]; va_b_vec = mpu_readout[1][va_row_sel]; end
            3'd2:    begin va_a_vec = accumulator[2][va_row_sel]; va_b_vec = mpu_readout[2][va_row_sel]; end
            3'd3:    begin va_a_vec = accumulator[3][va_row_sel]; va_b_vec = mpu_readout[3][va_row_sel]; end
            3'd4:    begin va_a_vec = accumulator[0][va_row_sel]; va_b_vec = accumulator[1][va_row_sel]; end
            3'd5:    begin va_a_vec = combine_temp[va_row_sel];   va_b_vec = accumulator[2][va_row_sel]; end
            3'd6:    begin va_a_vec = combine_temp[va_row_sel];   va_b_vec = accumulator[3][va_row_sel]; end
            default: begin va_a_vec = '0; va_b_vec = '0; end
        endcase
    end

    // Capture-side: accumulator[0..3] (operand_sel 0..3, mpu = operand_sel[1:0])
    always_ff @(posedge clk) begin
        if (rst || accum_clear) begin
            for (int m = 0; m < NUM_MPUS; m++)
                for (int r = 0; r < NUM_CGS; r++)
                    accumulator[m][r] <= '0;
        end else if (va_capture_valid && operand_sel < 3'd4) begin
            accumulator[operand_sel[1:0]][va_capture_row_sel] <= va_sum_vec;
        end
    end

    // Capture-side: combine_temp (operand_sel 4 or 5)
    always_ff @(posedge clk) begin
        if (rst) begin
            for (int r = 0; r < NUM_CGS; r++)
                combine_temp[r] <= '0;
        end else if (va_capture_valid && (operand_sel == 3'd4 || operand_sel == 3'd5)) begin
            combine_temp[va_capture_row_sel] <= va_sum_vec;
        end
    end

    //================================================================
    // Group's final output_bram -- reused unmodified. COMBINE3
    // (operand_sel==6) writes each row directly as vecadd_fsm
    // produces it; no separate commit stage needed.
    //================================================================
    logic out_a_en, out_a_we;

    assign out_a_en = va_capture_valid && (operand_sel == 3'd6);
    assign out_a_we = out_a_en;

    output_bram #(
        .DATA_WIDTH ( DATA_WIDTH ),
        .NUM_VPUS   ( NUM_VPUS   ),
        .DEPTH      ( MPU_DEPTH  )
    ) u_output_bram (
        .clk    ( clk                 ),
        .a_en   ( out_a_en            ),
        .a_we   ( out_a_we            ),
        .a_addr ( va_capture_row_sel  ),
        .a_din  ( va_sum_vec          ),
        .a_dout (                     ),
        .b_en   ( grp_out_en          ),
        .b_addr ( grp_out_addr        ),
        .b_dout ( grp_out_dout        )
    );

endmodule
