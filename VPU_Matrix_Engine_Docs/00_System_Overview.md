# System Overview

## Diagram

[`diagrams_mpe/SYSTEM.drawio`](diagrams_mpe/SYSTEM.drawio) — the full
top-to-bottom hierarchy in one picture: MPE → Group → MPU → VPU, with the
`*_fsm`/`*_datapath` split and the DSP/instance counts labeled at each
level. Open it in [diagrams.net](https://app.diagrams.net) (File → Open
From → Device) or VS Code's **Draw.io Integration** extension — GitHub
does not render `.drawio` files inline, so this is a link, not an
embedded image.

## What this engine does

At the top, this design computes one operation: a large **GEMV**
(matrix-vector multiply) in FP32.

```
[1 x 4096 activation]  .  [4096 x 4096 weight matrix]  =  [1 x 4096 output]
```

That's a single "layer" of matrix-vector math — the kind of operation that
dominates inference in a fully-connected neural-network layer. The reduction
dimension (the "4096" that gets summed over per output element) and the
output width (also 4096, since the matrix is square in the default
configuration) are both split up and spread across a five-level hierarchy of
identical, repeated hardware blocks.

The whole point of the hierarchy is: **do the same 256-deep FP32 dot product
over and over, in parallel, with the results combined by simple adders**.
Nothing above the lowest level does any new arithmetic — every level from
the Group upward is just sequencing and adding together results that were
already computed underneath it.

## The five levels, top to bottom

| Level | Module | Job | Default size |
|---|---|---|---|
| 1. MPE | `mpe_top` | Whole engine: full 1x4096 . 4096x4096 GEMV | 8 groups x 2 passes |
| 2. Group | `group_top` | One 256-column slice, full 4096-deep reduction | 4 MPUs x 4 chunks |
| 3. MPU | `mpu_top` | One 256x256 weight tile: 256-deep partial reduction, 256 columns | 16 VPUs x 16 column-groups |
| 4. VPU | `vpu` | One 256-deep FP32 dot product (one output column) | 16-wide DSP cascade x 16 passes |
| 5. DSP cascade | `dsp_cascade` | 16 chained multiply-accumulates per pass | 16 x `dspfp32_pe` |

Read [01_DSPFP32_Building_Block.md](01_DSPFP32_Building_Block.md) for the
bottom of this table, then [02_VPU_Architecture.md](02_VPU_Architecture.md)
through [09_MPE_Linkage.md](09_MPE_Linkage.md) for everything above it, in
order.

## How the numbers connect

This is the part that's easy to get lost in, so it's worth working through
once, bottom-up, with the actual default parameter values used in the RTL.

**Level 5 — DSP cascade (`dsp_cascade`, `K=16`).** A dot product needs one
multiply-accumulate per element. `K=16` `dspfp32_pe` primitives are chained
so that DSP `i`'s multiply result feeds into DSP `i+1`'s adder
(`PCOUT -> PCIN`). One pass through the chain reduces 16 elements down to a
single running sum.

**Level 4 — VPU (`vpu`, `DEPTH=256`).** A VPU needs to reduce 256 elements,
not just 16. `vpu_fsm` runs the `dsp_cascade` `DEPTH/K = 256/16 = 16` times
("passes"), feeding the previous pass's running sum back in as the new
starting point (`acc_reg`) for the next. After 16 passes, the VPU has one
complete 256-deep FP32 dot product: **one output value**, i.e. one column of
the weight matrix multiplied against the full activation vector.

**Level 3 — MPU (`mpu_top`, `NUM_VPUS=16`).** An MPU is responsible for a
full 256x256 weight tile — 256 columns, each needing its own 256-deep dot
product. Since a VPU produces exactly one column's result, an MPU
instantiates `NUM_VPUS=16` VPUs side by side (all fed the *same* activation
vector, each with its *own* 256-deep weight column). 16 VPUs cover 16
columns per "compute pass"; the MPU loops `NUM_CGS = 256/16 = 16`
compute passes (`mpu_fsm`'s `compute_idx`) — one per "column group" — to
cover all 256 columns of its tile.

**Level 2 — Group (`group_top`, `NUM_MPUS=4`, `NUM_CHUNKS=4`).** One MPU's
256x256 tile only covers 256 of the reduction dimension, but the real
reduction is 4096 deep. A Group covers the *full* 4096-deep reduction for a
*fixed* 256-column slice by running `NUM_MPUS=4` MPUs, each processing
`NUM_CHUNKS=4` different 256-deep chunks *of the same 256 columns*,
one chunk at a time (self-accumulating its own 4 chunks into a running
per-MPU total: `4 x 256 = 1024` depth per MPU). The 4 MPUs' partial totals
are then combined with a small 3-stage adder tree
(`(mpu0+mpu1) + mpu2 + mpu3`) to produce the true 4096-deep result
(`4 MPUs x 1024 depth = 4096`) for that one 256-column slice.

**Level 1 — MPE (`mpe_top`, `NUM_GROUPS=8`, `NUM_PASSES=2`).** A Group
covers 256 columns; the real output is 4096 columns wide. The MPE covers the
full width by running `NUM_GROUPS=8` groups in parallel (`8 x 256 = 2048`
columns), across `NUM_PASSES=2` sequential weight passes
(`2 x 2048 = 4096` columns total). Because activation values only depend on
depth, never on column, the same 4096-deep activation vector is loaded
*once* and broadcast to every group and every pass — only the weight data
differs.

Put together: `NUM_GROUPS(8) x NUM_PASSES(2) x MPU_DEPTH(256) = 4096`
output columns, and `NUM_MPUS(4) x NUM_CHUNKS(4) x MPU_DEPTH(256) = 4096`
reduction depth. Both come out to 4096, matching the `1x4096 . 4096x4096`
GEMV this whole engine is built to compute.

## The control/datapath split (the pattern repeated at every level)

Every level above the DSP primitive follows the **same two-module split**:

- **`*_fsm`** — the *sole command authority*. Pure control: a state
  machine that issues request pulses (`load_*`, `*_start`, `do_readout`,
  ...) and small index/select codes, and waits on matching "done" pulses.
  It never touches a wide data bus directly.
- **`*_datapath`** (or, at the two lowest levels, the top module's own
  wiring) — the *subordinate data mover*. Owns every wide bus (activation
  vectors, weight tiles, accumulators), and does the actual streaming,
  muxing, and storage. It has no opinion about sequencing — it just reacts
  to the pulses the FSM sends it.

This shows up literally in the file names: `vpu_fsm` + `dsp_cascade` inside
`vpu`; `mpu_fsm` + `memory_controller` inside `mpu_top`; `group_fsm` +
`group_datapath` inside `group_top`; `mpe_fsm` + `mpe_datapath` inside
`mpe_top`. The reason for the split, consistently, is so that wide buses
(up to 512 bits, or much wider internally) never have to route through
control logic, and so each level's timing/pipeline-latency concerns stay
local to its own FSM instead of leaking into the level above or below it.

See the "linkage" document for each level (
[03_VPU_Linkage.md](03_VPU_Linkage.md),
[05_MPU_Linkage.md](05_MPU_Linkage.md),
[07_Group_Linkage.md](07_Group_Linkage.md),
[09_MPE_Linkage.md](09_MPE_Linkage.md)) for the exact handshake pulses and
timing at each level.

## Data flow summary

1. **Host preload** (before `start`): activation data and weight data are
   DMA'd into BRAM/URAM storage at the MPE's outer ports — activation is a
   single broadcast fan-out shared by every group and pass; weight data is
   loaded per (group, MPU-lane), covering both passes' data.
2. **`start` pulses at the top** (`mpe_fsm`), and the command cascades
   downward: MPE tells its groups to reload weights for the current pass and
   compute; each group tells its MPUs to load chunks and compute; each MPU
   tells its memory controller to fetch activation/weights and its VPUs to
   run.
3. **Results flow back upward** the same way, but through dedicated
   `output_bram` instances at each level (never through the FSMs): VPU
   results are committed into the MPU's `output_bram`; MPU results are read
   out and summed together into the Group's `output_bram`; Group results
   are read out into the MPE's single global `output_bram`, which the host
   finally reads.
4. **`valid` pulses at the top** once the whole `1x4096` output vector is
   ready in the MPE's `output_bram`.

## Where to go next

- New to the arithmetic hardware? Start at
  [01_DSPFP32_Building_Block.md](01_DSPFP32_Building_Block.md).
- Want the VPU (the "compute engine" every larger block is made of)?
  Go to [02_VPU_Architecture.md](02_VPU_Architecture.md).
- Already familiar with the VPU and want to understand how MPUs, Groups
  and the MPE wire it together? Jump straight to
  [04_MPU_Architecture.md](04_MPU_Architecture.md),
  [06_Group_Architecture.md](06_Group_Architecture.md), or
  [08_MPE_Architecture.md](08_MPE_Architecture.md).
- Need to know the shared memory wrappers referenced throughout? See
  [10_Memory_Primitives.md](10_Memory_Primitives.md).
