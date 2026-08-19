`timescale 1ns / 1ps
//======================================================================
// mpe_tb.sv -- Testbench for mpe_top
//
// Verification flow:
//   1. Generates 100 MHz clock and synchronous reset pulse.
//   2. Pre-loads the 4 shared activation ports (broadcast to all
//      8 groups by mpe_top's own wiring) and all 32 (group x MPU-lane)
//      weight bulk-store ports via DMA (Port A). Each port gets its
//      own dedicated $readmemh target array, generated via `generate`
//      -- NOT a slice of a 2D/3D array, since Vivado's xsim silently
//      corrupts $readmemh into a sliced multi-dimensional array (the
//      exact bug that produced the 255/256-mismatched weight rows
//      diagnosed during Group bring-up).
//   3. Asserts `start` pulse to uut (mpe_top).
//   4. Monitors `busy` and waits for `valid`.
//   5. Reads the MPE's combined 1x4096 output from the global
//      output_bram (Port B) via `mpe_out_*`.
//   6. Performs FP32 self-checking against mpe_golden_output.mem
//      (see gen_mpe_mem.py -- computed with the exact same
//      summation grouping the hardware uses).
//
// Data files (see gen_mpe_mem.py): mpe_mpu{0..3}_act.mem,
// mpe_g{0..7}_mpu{0..3}_wgt.mem, mpe_golden_output.mem.
//
// This exercises the full default config (NUM_GROUPS=8, NUM_PASSES=2,
// 64 MiB of weight data) -- expect a long run. Each pass alone costs
// ~16k cycles of internal weight reload (parallel across groups) plus
// each group's own ~6800-cycle compute, times 2 passes, plus the
// initial ~34k-cycle host DMA preload (1024 activation words x 4
// parallel ports negligible; 32768 weight words x 32 parallel ports
// dominates). Budget real wall-clock time accordingly.
//======================================================================

module mpe_tb;

    // Accelerator parameters (mirror mpe_top's defaults)
    parameter DATA_WIDTH   = 32;
    parameter NUM_VPUS     = 16;
    parameter MPU_DEPTH    = 256;
    parameter K            = 16;
    parameter PIPE_LATENCY = 24;
    parameter NUM_MPUS     = 4;
    parameter NUM_CHUNKS   = 4;
    parameter ADD_LATENCY  = 12;
    parameter NUM_GROUPS   = 8;
    parameter NUM_PASSES   = 2;

    localparam NUM_CGS              = MPU_DEPTH / NUM_VPUS;                       // 16
    localparam CG_WIDTH             = NUM_VPUS * DATA_WIDTH;                      // 512
    localparam GRP_ACT_CHUNK_ADDR_W = $clog2(NUM_CHUNKS * MPU_DEPTH);            // 10
    localparam GRP_ACT_CHUNK_DEPTH  = NUM_CHUNKS * MPU_DEPTH;                     // 1024
    localparam WGT_CHUNK_DEPTH      = NUM_CHUNKS * NUM_CGS * MPU_DEPTH;          // 16384
    localparam GRP_WGT_CHUNK_ADDR_W = $clog2(WGT_CHUNK_DEPTH);                    // 14
    localparam GROUP_BITS           = $clog2(NUM_GROUPS);                        // 3
    localparam PASS_BITS            = $clog2(NUM_PASSES);                        // 1
    localparam BULK_ADDR_W          = PASS_BITS + GRP_WGT_CHUNK_ADDR_W;          // 15
    localparam BULK_DEPTH           = NUM_PASSES * WGT_CHUNK_DEPTH;              // 32768
    localparam MPE_OUT_DEPTH        = NUM_GROUPS * NUM_PASSES * MPU_DEPTH;       // 4096
    localparam MPE_OUT_ADDR_W       = GROUP_BITS + PASS_BITS + $clog2(NUM_CGS);  // 8
    localparam MPE_OUT_CGS          = MPE_OUT_DEPTH / NUM_VPUS;                  // 256

    // Clock and reset
    logic clk;
    logic rst;

    // Host control interface
    logic start;
    logic busy;
    logic valid;

    // Host DMA ports -- activation broadcast preload
    logic                            mpe_act_en   [0:NUM_MPUS-1];
    logic                            mpe_act_we   [0:NUM_MPUS-1];
    logic [GRP_ACT_CHUNK_ADDR_W-1:0] mpe_act_addr [0:NUM_MPUS-1];
    logic [DATA_WIDTH-1:0]           mpe_act_din  [0:NUM_MPUS-1];

    // Host DMA ports -- weight bulk preload
    logic                   mpe_wgt_en   [0:NUM_GROUPS-1][0:NUM_MPUS-1];
    logic                   mpe_wgt_we   [0:NUM_GROUPS-1][0:NUM_MPUS-1];
    logic [BULK_ADDR_W-1:0] mpe_wgt_addr [0:NUM_GROUPS-1][0:NUM_MPUS-1];
    logic [CG_WIDTH-1:0]    mpe_wgt_din  [0:NUM_GROUPS-1][0:NUM_MPUS-1];

    // Host DMA port -- MPE's final output (Port B)
    logic                      mpe_out_en;
    logic [MPE_OUT_ADDR_W-1:0] mpe_out_addr;
    logic [CG_WIDTH-1:0]       mpe_out_dout;

    // Golden reference
    logic [CG_WIDTH-1:0] gold_file_mem [0:MPE_OUT_CGS-1];

    int error_count = 0;

    function automatic shortreal abs_diff(shortreal a, shortreal b);
        return (a > b) ? (a - b) : (b - a);
    endfunction

    //------------------------------------------------------------
    // Unit Under Test
    //------------------------------------------------------------
    mpe_top #(
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
    ) uut (
        .clk ( clk ),
        .rst ( rst ),

        .start ( start ),
        .busy  ( busy  ),
        .valid ( valid ),

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
        .mpe_out_dout ( mpe_out_dout )
    );

    //------------------------------------------------------------
    // 100 MHz clock
    //------------------------------------------------------------
    always #5 clk = ~clk;

    //------------------------------------------------------------
    // Preload synchronization barriers -- each generated loader waits
    // on these before starting its own DMA loop, so all parallel
    // ports begin (and therefore finish) together.
    //------------------------------------------------------------
    logic act_load_start;
    logic wgt_load_start;

    //------------------------------------------------------------
    // Activation preload -- 4 independent ports (one per MPU-slot),
    // broadcast by mpe_top to every group. Each gets its own flat 1D
    // $readmemh target, generated so no array-of-array slice is ever
    // passed to $readmemh.
    //------------------------------------------------------------
    genvar m;
    generate
    for (m = 0; m < NUM_MPUS; m = m + 1) begin : gen_act_load
        logic [DATA_WIDTH-1:0] act_mem [0:GRP_ACT_CHUNK_DEPTH-1];

        initial begin
            $readmemh($sformatf("mpe_mpu%0d_act.mem", m), act_mem);
        end

        initial begin
            mpe_act_en[m]   = 1'b0;
            mpe_act_we[m]   = 1'b0;
            mpe_act_addr[m] = '0;
            mpe_act_din[m]  = '0;
            wait (act_load_start == 1'b1);
            for (int i = 0; i < GRP_ACT_CHUNK_DEPTH; i = i + 1) begin
                @(posedge clk);
                mpe_act_en[m]   <= 1'b1;
                mpe_act_we[m]   <= 1'b1;
                mpe_act_addr[m] <= i[GRP_ACT_CHUNK_ADDR_W-1:0];
                mpe_act_din[m]  <= act_mem[i];
            end
            @(posedge clk);
            mpe_act_en[m] <= 1'b0;
            mpe_act_we[m] <= 1'b0;
        end
    end
    endgenerate

    //------------------------------------------------------------
    // Weight preload -- 32 independent ports (group x MPU-lane), each
    // its own flat 1D $readmemh target, same reasoning as above.
    //------------------------------------------------------------
    genvar g;
    generate
    for (g = 0; g < NUM_GROUPS; g = g + 1) begin : gen_wgt_load_g
        for (m = 0; m < NUM_MPUS; m = m + 1) begin : gen_wgt_load_m
            logic [CG_WIDTH-1:0] wgt_mem [0:BULK_DEPTH-1];

            initial begin
                $readmemh($sformatf("mpe_g%0d_mpu%0d_wgt.mem", g, m), wgt_mem);
            end

            initial begin
                mpe_wgt_en[g][m]   = 1'b0;
                mpe_wgt_we[g][m]   = 1'b0;
                mpe_wgt_addr[g][m] = '0;
                mpe_wgt_din[g][m]  = '0;
                wait (wgt_load_start == 1'b1);
                for (int i = 0; i < BULK_DEPTH; i = i + 1) begin
                    @(posedge clk);
                    mpe_wgt_en[g][m]   <= 1'b1;
                    mpe_wgt_we[g][m]   <= 1'b1;
                    mpe_wgt_addr[g][m] <= i[BULK_ADDR_W-1:0];
                    mpe_wgt_din[g][m]  <= wgt_mem[i];
                end
                @(posedge clk);
                mpe_wgt_en[g][m] <= 1'b0;
                mpe_wgt_we[g][m] <= 1'b0;
            end
        end
    end
    endgenerate

    //------------------------------------------------------------
    // Task: Readout & golden verification
    //------------------------------------------------------------
    task automatic verify_results();
        shortreal hw_val, golden_val;

        $display("[%0t ns] Verification: Reading mpe_golden_output.mem...", $time);
        $readmemh("mpe_golden_output.mem", gold_file_mem);

        $display("\n=================================================");
        $display("       STARTING MPE HARDWARE RESULT VERIFICATION   ");
        $display("=================================================");

        for (int cg = 0; cg < MPE_OUT_CGS; cg++) begin
            @(posedge clk);
            mpe_out_en   <= 1'b1;
            mpe_out_addr <= cg[MPE_OUT_ADDR_W-1:0];
            @(posedge clk); // output_bram samples en/addr on this edge
            @(posedge clk); // dout_b (NBA-driven) is settled by this edge

            for (int v = 0; v < NUM_VPUS; v++) begin
                int col_idx = cg * NUM_VPUS + v;

                hw_val     = $bitstoshortreal(mpe_out_dout[v*32 +: 32]);
                golden_val = $bitstoshortreal(gold_file_mem[cg][v*32 +: 32]);

                if (abs_diff(hw_val, golden_val) > 0.001) begin
                    $display("[FAIL] Y[%0d] -- HW Output: %f (0x%08h) | Expected Golden: %f (0x%08h)",
                             col_idx, hw_val, mpe_out_dout[v*32 +: 32],
                             golden_val, gold_file_mem[cg][v*32 +: 32]);
                    error_count++;
                end else begin
                    $display("[PASS] Y[%0d] = %f", col_idx, hw_val);
                end
            end
        end

        mpe_out_en <= 1'b0;

        $display("=================================================");
        if (error_count == 0) begin
            $display(" [PASS] SUCCESS: All %0d MPE FP32 Outputs Match!", MPE_OUT_CGS * NUM_VPUS);
        end else begin
            $display(" [FAIL] FAILURE: %0d Mismatches Detected!", error_count);
        end
        $display("=================================================\n");
    endtask

    //------------------------------------------------------------
    // Main stimulus
    //------------------------------------------------------------
    initial begin
        clk = 0;
        rst = 1;
        start = 0;
        act_load_start = 1'b0;
        wgt_load_start = 1'b0;
        mpe_out_en   = 0;
        mpe_out_addr = 0;

        #50;
        rst = 0;
        $display("[%0t ns] System Reset Released.", $time);
        #20;

        $display("[%0t ns] DMA: Preloading 4 activation ports (broadcast to all %0d groups)...", $time, NUM_GROUPS);
        act_load_start <= 1'b1;
        repeat (GRP_ACT_CHUNK_DEPTH + 5) @(posedge clk);
        act_load_start <= 1'b0;

        $display("[%0t ns] DMA: Preloading %0d weight bulk ports (%0d groups x %0d MPU lanes)...",
                  $time, NUM_GROUPS*NUM_MPUS, NUM_GROUPS, NUM_MPUS);
        wgt_load_start <= 1'b1;
        repeat (BULK_DEPTH + 5) @(posedge clk);
        wgt_load_start <= 1'b0;

        #50;

        @(posedge clk);
        start <= 1'b1;
        $display("[%0t ns] Host: Triggered start pulse.", $time);
        @(posedge clk);
        start <= 1'b0;

        $display("[%0t ns] Host: Waiting for valid signal pulse...", $time);
        wait (valid == 1'b1);
        $display("[%0t ns] Host: Computation Done (valid pulsed high)!", $time);

        #20;
        verify_results();

        $finish;
    end

endmodule
