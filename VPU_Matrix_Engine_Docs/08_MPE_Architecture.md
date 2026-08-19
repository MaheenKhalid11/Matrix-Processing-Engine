# MPE Architecture — The Top-Level Engine

**Top module:** [`mpe_top.sv`](E:/vivado_projs/memory_and_vpu/memory_and_vpu.srcs/sources_1/new/mpe_top.sv)
**Control:** [`mpe_fsm.sv`](E:/vivado_projs/memory_and_vpu/memory_and_vpu.srcs/sources_1/new/mpe_fsm.sv)
**Datapath:** [`mpe_datapath.sv`](E:/vivado_projs/memory_and_vpu/memory_and_vpu.srcs/sources_1/new/mpe_datapath.sv)
**Built from:** 8x [`group_top`](06_Group_Architecture.md) (bulk weight
storage + a single shared output readout)

## Diagram

[`diagrams_mpe/MPE.drawio`](diagrams_mpe/MPE.drawio) — the full MPE block
diagram: `mpe_fsm` and `mpe_datapath`, the 8 `group_top` instances, the
bulk weight lanes, the per-group pass reloaders, the shared readout
engine, and the global `output_bram`. Open in
[diagrams.net](https://app.diagrams.net) or VS Code's **Draw.io
Integration** extension (GitHub doesn't render `.drawio` files inline).

The MPE (top level of this design) is where the full-width GEMV finally
comes together. A Group covers the full 4096-deep reduction, but only for a
256-column slice. The MPE covers the **entire 4096-wide output** by running
8 Groups in parallel, over 2 sequential passes, using bulk weight storage
that is unique to this level.

See [00_System_Overview.md](00_System_Overview.md) for the full-hierarchy
picture, and [09_MPE_Linkage.md](09_MPE_Linkage.md) for the exact wiring
and state sequence.

## What the MPE computes

```
Given: activation[0..4095]         (1x4096, loaded ONCE, shared everywhere)
       weight[0..4095][0..4095]    (4096x4096, the full matrix)
Produces: output[0..4095], where
          output[c] = sum(activation[i] * weight[i][c] for i in 0..4095)
```

This is the complete GEMV described in
[00_System_Overview.md](00_System_Overview.md) — the whole reason this
engine exists.

## Covering 4096 columns: 8 groups x 2 passes

A Group produces 256 columns' worth of output per run. To cover 4096
columns, the MPE uses **`NUM_GROUPS=8`** groups running in parallel
(`8 x 256 = 2048` columns per pass) across **`NUM_PASSES=2`** sequential
passes (`2 x 2048 = 4096` columns total):

```
NUM_GROUPS(8) x NUM_PASSES(2) x MPU_DEPTH(256) = 4096   (total output width)
```

Each pass reuses the *same* 8 `group_top` instances with *different*
weight data loaded in — the group hardware itself is never duplicated
across passes, only the weight data it operates on changes. This is exactly
why [06_Group_Architecture.md](06_Group_Architecture.md)'s `accum_clear`
mechanism matters: the same physical group gets re-triggered for pass 2
with fresh weight data, and its leftover accumulator state from pass 1 must
be zeroed first.

## Activation is loaded once — a key simplification

Activation values depend only on **depth** (the reduction dimension, i2 in
`weight[i][c]`), never on which output column `c` they're being multiplied
against. Since every group and every pass covers the *same* full 4096-deep
activation vector, there is **no per-group or per-pass activation
storage at the MPE level at all** — the MPE-level activation ports are pure
broadcast fan-out wiring, identical data reaching every group's matching
MPU chunk store simultaneously. Only weight data genuinely differs per
(group, pass, MPU-lane), and only weight data needs its own dedicated bulk
storage at this level.

## Weight is genuinely per-(group, MPU-lane), loaded once, covering both passes

Unlike activation, weight data *does* differ across groups (different
column slices) and passes (different halves of the column range). Rather
than re-streaming weight data from the host between passes, the MPE
preloads **both passes' worth** of weight data for every (group, MPU-lane)
combination up front, into per-lane bulk storage (`uram_tdp_init`,
`NUM_GROUPS x NUM_MPUS = 8 x 4 = 32` independent lanes total). A per-group
"pass reloader" then streams the currently-needed pass's slice out of bulk
storage into that group's own `group_top` weight ports on demand — see
[09_MPE_Linkage.md](09_MPE_Linkage.md) for the mechanism.

## Readout is the one thing that can't be parallel

Every group can load its weights and compute fully independently and in
parallel with every other group — no shared resource there. But there is
only **one physical write port** into the MPE's single global `output_bram`
(the `1x4096` final result buffer). So readout — pulling each group's
finished 256-wide result out and writing it into the right slice of the
global output — must be **serialized**, one group at a time, exactly
mirroring the same "one shared adder, many parallel compute units" pattern
already seen at the Group level (see
[06_Group_Architecture.md](06_Group_Architecture.md)'s "two parallelism
strategies" section) — just applied to a readout port instead of an adder.

## Building blocks inside `mpe_top`

```
mpe_top
 ├── mpe_fsm       -- control only
 ├── mpe_datapath  -- datapath: bulk weight lanes, 8x group_top, pass
 │    │              reloaders, shared readout engine, global output_bram
 │    ├── 8 x 4 = 32x uram_tdp_init  -- bulk weight lanes (group x MPU-lane)
 │    ├── 8x group_top               -- unmodified, one per group
 │    ├── 8x "pass reloader"         -- per-group weight stream-in state machine
 │    ├── ONE shared readout engine  -- serializes group -> global output_bram
 │    └── output_bram (MPE_OUT_DEPTH=4096) -- the final 1x4096 result
```

## Memory sizing at the MPE level

| Store | Instances | Depth (per instance) | Width | Total content |
|---|---|---|---|---|
| Bulk weight lane | `NUM_GROUPS x NUM_MPUS = 32` | `PASS_BITS + GRP_WGT_CHUNK_ADDR_W` (both passes) | 512 bits | All weight data, both passes, every group/MPU-lane |
| Global output | 1 | `NUM_GROUPS x NUM_PASSES x MPU_DEPTH = 4096` | 512 bits (16 x FP32) | The complete `1x4096` output vector |

The global output address is a plain concatenation
`{group_sel, pass_sel, row}` — adderless, for the same reason weight/output
addressing is adderless at every lower level: `NUM_GROUPS`, `NUM_PASSES`,
and `NUM_CGS` are all powers of two.

## Port summary

```systemverilog
module mpe_top #(
    parameter NUM_GROUPS = 8, NUM_PASSES = 2, NUM_MPUS = 4,
    parameter MPU_DEPTH = 256, NUM_VPUS = 16, NUM_CHUNKS = 4
)(
    input  logic clk, rst,
    input  logic start,  output logic busy, valid,

    // Activation preload -- ONE set of ports per MPU-slot, broadcast to
    // every group's matching MPU chunk store simultaneously
    input logic mpe_act_en[0:3], mpe_act_we[0:3],
    input logic [9:0] mpe_act_addr[0:3],
    input logic [31:0] mpe_act_din[0:3],

    // Weight bulk preload -- independent per (group, MPU-lane): both
    // passes' data for that lane, loaded once
    input logic mpe_wgt_en[0:7][0:3], mpe_wgt_we[0:7][0:3],
    input logic [14:0] mpe_wgt_addr[0:7][0:3],
    input logic [511:0] mpe_wgt_din[0:7][0:3],

    // Final combined 1x4096 output -- the whole engine's answer
    input  logic mpe_out_en,
    input  logic [7:0] mpe_out_addr,       // $clog2(4096/16) = 8 bits, 16 words/beat
    output logic [511:0] mpe_out_dout
);
```

Next: [09_MPE_Linkage.md](09_MPE_Linkage.md) for `mpe_fsm`'s full state
sequence, the pass-reloader mechanism, and the shared readout engine.
