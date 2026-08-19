# Group Architecture

**Top module:** [`group_top.sv`](E:/vivado_projs/memory_and_vpu/memory_and_vpu.srcs/sources_1/new/group_top.sv)
**Control:** [`group_fsm.sv`](E:/vivado_projs/memory_and_vpu/memory_and_vpu.srcs/sources_1/new/group_fsm.sv)
**Datapath:** [`group_datapath.sv`](E:/vivado_projs/memory_and_vpu/memory_and_vpu.srcs/sources_1/new/group_datapath.sv)
**Built from:** 4x [`mpu_top`](04_MPU_Architecture.md) + a shared FP32 adder
tree ([`accum_adder16`](01_DSPFP32_Building_Block.md) +
[`vecadd_fsm`](#the-shared-vector-adder))

## Diagram

[`diagrams_mpe/GROUP.drawio`](diagrams_mpe/GROUP.drawio) — the Group
block diagram: `group_fsm` and `group_datapath`, the 4 `mpu_top`
instances, the shared `accum_adder16`/`vecadd_fsm` pair, and the Group's
own `output_bram`. Open in [diagrams.net](https://app.diagrams.net) or
VS Code's **Draw.io Integration** extension (GitHub doesn't render
`.drawio` files inline).

An MPU covers a full `256 x 256` weight tile — but only `256` of the
reduction dimension. The real reduction in this design is `4096` deep. A
Group is the level that closes that gap: it covers the **full 4096-deep
reduction for one fixed 256-column slice** of the final output, by running
4 MPUs, each responsible for a different quarter of the depth, and then
adding their partial results together.

See [00_System_Overview.md](00_System_Overview.md) for where the Group sits
in the overall hierarchy, and
[07_Group_Linkage.md](07_Group_Linkage.md) for the exact wiring and state
sequence.

## What a Group computes

```
Given: activation[0..4095]      (1x4096, this group's own 4096-deep slice)
       weight[0..4095][0..255]  (4096x256, this group's own column slice)
Produces: output[0..255], where
          output[c] = sum(activation[i] * weight[i][c] for i in 0..4095)
```

256 output values (matching one MPU's column width), each now summed over
the *full* 4096-deep reduction rather than just 256.

## Splitting 4096 depth across 4 MPUs x 4 chunks

An MPU can only do a 256-deep reduction per run. To cover 4096, the Group
uses **`NUM_MPUS=4`** MPUs, each handling **`NUM_CHUNKS=4`** separate
256-deep "chunks" *of the same 256 output columns*:

```
NUM_MPUS(4) x NUM_CHUNKS(4) x MPU_DEPTH(256) = 4096   (total depth covered)
```

Concretely: MPU 0 might cover depth-chunks 0,1,2,3 (0..1023); MPU 1 covers
chunks 4,5,6,7 (1024..2047); and so on — the exact chunk-to-MPU assignment
is a host/data-layout decision (which chunk store holds which slice of
data), not something the RTL itself hardcodes. What the RTL *does* enforce
is: each MPU runs its own 4 chunks **sequentially**, self-accumulating
(adding) its own chunk results together into a running per-MPU total
(`4 x 256 = 1024` depth covered per MPU). All 4 MPUs do this **in
parallel** with each other — they're identical, independent hardware,
started together and running in lockstep.

Once every MPU has finished self-accumulating its own 1024-deep partial
total, those 4 partial totals (one per MPU) are combined with a small
3-stage adder tree to produce the final, true 4096-deep result for this
Group's 256 columns:

```
combine_temp = accumulator[0] + accumulator[1]      (COMBINE1)
combine_temp = combine_temp   + accumulator[2]       (COMBINE2)
output_bram  = combine_temp   + accumulator[3]       (COMBINE3, written directly)
```

## Building blocks inside `group_top`

```
group_top
 ├── group_fsm       -- control only
 ├── group_datapath  -- datapath: chunk stores, loaders, readouts, shared adder
 │    ├── 4x uram_tdp_init      -- per-MPU weight chunk store (all 4 chunks)
 │    ├── 4x activation_bram    -- per-MPU activation chunk store (all 4 chunks)
 │    ├── 4x mpu_top            -- unmodified, one per MPU
 │    ├── accum_adder16         -- ONE shared 16-lane FP32 adder bank
 │    ├── vecadd_fsm            -- ONE shared adder's control (rows/latency)
 │    └── output_bram           -- Group's own final 1x256 output
```

## Two very different parallelism strategies, side by side

This is the most important structural idea in the Group, and it's worth
calling out explicitly because it reappears (in a slightly different shape)
at the MPE level too:

- **Per-MPU loaders and readouts run in FULL PARALLEL.** Each MPU owns its
  own physical chunk-store ports and its own `mpu_top` instance — no
  sharing, no time-multiplexing. Loading chunk data into MPU 0 doesn't
  block loading chunk data into MPU 1.
- **The FP32 adder is a SINGLE SHARED bank, reused serially.** There's only
  *one* `accum_adder16` + `vecadd_fsm` pair in the whole Group, even though
  there are 4 self-accumulate operations per chunk (one per MPU) plus 3
  final-combine stages. `group_fsm` loops the shared adder across all of
  these, one at a time.

The reasoning (from the source comments): add time is negligible compared
to compute time, so sharing the adder costs a little latency but buys 4x
less adder hardware — a good trade when the adder isn't the bottleneck.
Compute (`mpu_top` runs) and weight/activation reload have no such shared
resource and get true parallelism, because *those* operations dominate
runtime and are worth the extra hardware.

## The `operand_sel` encoding — how the shared adder is time-shared

`group_fsm` selects what the shared adder is doing on any given cycle via a
3-bit `operand_sel` code, consumed inside `group_datapath` to mux the
adder's `a_vec`/`b_vec` sources and `sum_vec` destination:

| `operand_sel` | Operation | a | b | destination |
|---|---|---|---|---|
| 0..3 | Self-accumulate MPU `m` | `accumulator[m]` | `mpu_readout[m]` | `accumulator[m]` |
| 4 | Combine stage 1 | `accumulator[0]` | `accumulator[1]` | `combine_temp` |
| 5 | Combine stage 2 | `combine_temp` | `accumulator[2]` | `combine_temp` |
| 6 | Combine stage 3 | `combine_temp` | `accumulator[3]` | `output_bram` (direct write) |

`operand_sel[1:0]` doubles as the MPU index for codes 0..3 — a small detail
that avoids a separate index signal for the self-accumulate case.

## Per-MPU accumulator array and the `accum_clear` reset

`accumulator[0..3][0..15]` (4 MPUs x 16 column-group rows x 512 bits) is a
*persistent* register array — it survives across chunk boundaries within one
Group run (that's the whole point: it accumulates). But if the same
`group_top` instance gets triggered a second time (e.g. the MPE re-uses the
same Group hardware for its second column-chunk pass — see
[08_MPE_Architecture.md](08_MPE_Architecture.md)), those old totals would
silently corrupt the new run's results unless cleared first.

`group_fsm` pulses `accum_clear` for exactly one cycle at the very start of
every run (`S_IDLE -> S_REQ_LOAD`), zeroing the whole accumulator array
before the first self-accumulate op. Without this pulse, `accumulator[]`
would only ever be cleared by a global `rst` — not enough for a
multi-run/multi-pass system.

## The shared vector-adder (`accum_adder16` + `vecadd_fsm`)

`accum_adder16` (see
[01_DSPFP32_Building_Block.md](01_DSPFP32_Building_Block.md)) is 16
independent FP32 adder lanes, each a `dspfp32_pe` with its multiplicand
tied to `1.0`. `vecadd_fsm` drives it through the same
`IDLE → LOAD → RUN → CAPTURE` shape as `vpu_fsm` (including its own
priming-pass throwaway, for the same "cold DSP chain" reason — see
[02_VPU_Architecture.md](02_VPU_Architecture.md)), but for **16 independent
512-bit rows** rather than a 256-deep reduction chain.

One subtlety documented directly in `vecadd_fsm.sv`'s header, worth
repeating here because it's a real, previously-hit bug: the adder is
**serialized, not pipelined**, on purpose. An earlier version tried
streaming a new `row_sel` every cycle (reasoning: the 16 rows are
independent, so nothing seemed to forbid it) — but that's wrong, because
`pcin` feeds the DSP's adder stage directly while the internal multiply
(`b_vec * 1.0`) takes several cycles to settle. By the time the add
actually happened, `pcin` had already raced ahead to a *later* row's value,
silently computing `b[row N] + a[row N+shift]` instead of `a[N]+b[N]`.
`dsp_cascade` never hits this because its `pcin` comes from a neighbor
DSP's own *registered* `pcout`, which is naturally stable for a whole pass.
`vecadd_fsm` reproduces that same stability directly instead, by holding
`row_sel` constant for the *entire* LOAD/RUN/CAPTURE window of one row
before presenting the next row's data.

## Port summary

```systemverilog
module group_top #(
    parameter NUM_MPUS = 4, NUM_CHUNKS = 4, MPU_DEPTH = 256, NUM_VPUS = 16
)(
    input  logic clk, rst,
    input  logic start,  output logic busy, valid,

    // Host preload, per MPU (4 sets of ports)
    input logic grp_act_en[0:3], grp_act_we[0:3],
    input logic [9:0] grp_act_addr[0:3],   // $clog2(4*256)
    input logic [31:0] grp_act_din[0:3],

    input logic grp_wgt_en[0:3], grp_wgt_we[0:3],
    input logic [13:0] grp_wgt_addr[0:3],  // $clog2(4*16*256)
    input logic [511:0] grp_wgt_din[0:3],

    // Group's final combined 1x256 output
    input  logic grp_out_en,
    input  logic [3:0] grp_out_addr,        // $clog2(16)
    output logic [511:0] grp_out_dout
);
```

Each of the 4 MPU-slot port sets holds **all `NUM_CHUNKS=4` chunks'** worth
of activation/weight data for that MPU, host-loaded once up front —
`group_datapath`'s "loader" then streams out just the chunk currently
needed on demand.

Next: [07_Group_Linkage.md](07_Group_Linkage.md) for `group_fsm`'s full
state sequence and the loader/readout mechanics inside `group_datapath`.
