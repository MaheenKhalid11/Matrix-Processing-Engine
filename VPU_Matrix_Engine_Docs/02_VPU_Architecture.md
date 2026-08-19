# VPU Architecture — The Vector Processing Element

**Top module:** [`vpu.sv`](E:/vivado_projs/memory_and_vpu/memory_and_vpu.srcs/sources_1/new/vpu.sv)
**Control:** [`vpu_fsm.sv`](E:/vivado_projs/memory_and_vpu/memory_and_vpu.srcs/sources_1/new/vpu_fsm.sv)
**Datapath:** [`dsp_cascade.sv`](E:/vivado_projs/memory_and_vpu/memory_and_vpu.srcs/sources_1/new/dsp_cascade.sv)
**Built from:** [`dspfp32_pe`](01_DSPFP32_Building_Block.md) (`dsp.v`)

## Diagram

- [`diagrams_mpe/VPU.drawio`](diagrams_mpe/VPU.drawio) — the VPU block
  diagram: `vpu_fsm` and `dsp_cascade` side by side with their handshake
  wires.
- [`diagrams_mpe/FSM.drawio`](diagrams_mpe/FSM.drawio) — `vpu_fsm`'s own
  `IDLE → LOAD → RUN → CAPTURE` state diagram, with the state-encoding
  bits and transition conditions labeled on each arrow.

Open either in [diagrams.net](https://app.diagrams.net) or VS Code's
**Draw.io Integration** extension — GitHub doesn't render `.drawio` files
inline.

The VPU (Vector Processing Unit) is the smallest *complete* compute unit in
this engine — the first level where "start a computation, wait, get one
finished answer" makes sense as a self-contained operation. Every larger
block (MPU, Group, MPE) is ultimately just many VPUs running with different
data, plus the sequencing and adders needed to combine their answers. See
[00_System_Overview.md](00_System_Overview.md) for how the VPU fits into the
bigger hierarchy, and [03_VPU_Linkage.md](03_VPU_Linkage.md) for the
signal-level detail of how its two internal modules talk to each other.

## What a VPU computes

One VPU computes **one 256-deep FP32 dot product** — one element of a
matrix-vector multiply:

```
dot_product = sum( activation[i] * weight[i]  for i = 0 .. DEPTH-1 )
```

with `DEPTH = 256` by default. In the context of the larger engine, the
`activation` vector is shared by every VPU in an MPU (it's the same 256
values everywhere), and `weight` is one VPU's *own* private 256-deep column
of the weight matrix — so each VPU, run in parallel with 15 others,
produces one distinct output column.

## Why it's split into `vpu_fsm` + `dsp_cascade`

The VPU follows the control/datapath split described in
[00_System_Overview.md](00_System_Overview.md):

- **`vpu_fsm`** owns *all* state: the IDLE/LOAD/RUN/CAPTURE sequencing,
  the priming workaround for a cold DSP chain, and every registered output
  (`busy`, `valid`, `dot_product`, `pass_cnt`, `acc_reg`, the clock-enables).
  It has no idea DSPs exist — it just watches a `chain_out` value come back
  and drives control signals out.
- **`dsp_cascade`** is pure datapath: it holds no control state of its own.
  It slices the right 16-wide chunk out of the full 256-wide activation and
  weight buses (using an index the FSM hands it), muxes in the running
  accumulator, and forwards the chain's tail back out.

This separation means the wide 256x32-bit (8192-bit) activation/weight
buses never have to pass through — or be understood by — the state machine;
they only ever touch `dsp_cascade`.

## Why one pass isn't enough: `K` vs `DEPTH`

The physical hardware only has `K=16` chained `dspfp32_pe` cells (a fixed
resource — this is how wide the silicon cascade actually is). But a dot
product needs to reduce `DEPTH=256` elements. So the VPU can't do the whole
256-deep reduction in one shot; it does it in
`NUM_PASSES = DEPTH / K = 256 / 16 = 16` separate passes, each covering 16
of the 256 elements, with the running total (`acc_reg`) carried from one
pass into the next as the starting point (`pcin`) for the following pass's
chain.

This is the central mechanism that makes the VPU work: **the K=16 physical
DSP chain gets reused 16 times to cover a 256-deep reduction**, the same way
a small ALU gets reused across loop iterations in software.

## The four-state pipeline: IDLE → LOAD → RUN → CAPTURE

Each of the 16 passes goes through the same four states, driven by
`vpu_fsm`:

1. **IDLE** — waiting for `start`. On `start`, resets `pass_cnt`, `acc_reg`,
   and sets `busy`. Also sets a one-time `priming` flag (see below).
2. **LOAD** — pulses `ce_a`/`ce_b`/`ce_pipe` for one cycle to latch this
   pass's 16-wide activation/weight slice into the DSP chain's input
   registers.
3. **RUN** — holds `ce_pipe` high and waits `PIPE_LATENCY` cycles for the
   16-deep chain to settle (the multiply + the rippling chain of adds takes
   several cycles to reach the far end — see "Pipeline latency" below).
4. **CAPTURE** — the chain's output (`chain_out`) is now valid. It's latched
   into `acc_reg` as the new running total. If this was the *last* pass
   (`pass_cnt == NUM_PASSES-1`), it's also latched into `dot_product` and
   `valid` pulses for one cycle. Otherwise `pass_cnt` increments and the FSM
   loops back to LOAD for the next pass.

This exactly mirrors "Cliff Cummings' 3-always-block FSM style" — one
`always_ff` for the state register, one `always_comb` for next-state logic,
one `always_ff` (keyed on the *current* state, not next-state — see the
in-file comment in `vpu_fsm.sv` for why that specific choice matters for
correctness) for every registered output.

## The priming pass — a deliberate throwaway first iteration

The very first pass after `start` is marked `priming`. Its `CAPTURE` result
is **discarded entirely** — `acc_reg`/`pass_cnt` are left untouched, and
the FSM loops back to `LOAD` to redo "pass 0" for real. This exists because
a `DSPFP32` chain that has been idle (clock-enables held low) needs one
throwaway pass to reach steady internal pipeline state before its output can
be trusted — a hardware quirk of the DSP macro's own internal pipelining,
not a bug being worked around. The same priming pattern reappears wherever
`dspfp32_pe` chains are driven from a cold/idle state — see
`vecadd_fsm` in [06_Group_Architecture.md](06_Group_Architecture.md).

## Pipeline latency (`PIPE_LATENCY`)

```
PIPE_LATENCY = 24   (default, for K=16)
```

This is *derived*, not simulator-measured, from the internal pipeline depth
of a single `DSPFP32`: `AREG`/`FPBREG` load (1 cycle) + `FPMPIPEREG`/
`FPM_PREG` multiply (2 cycles) + `FPA_PREG` add (1 cycle) = 4 cycles for one
isolated MAC. Because all 16 multiplies in a cascade pass run in
parallel (`PCIN` is wired *combinationally* between DSP stages inside
`dsp_cascade`, with no extra register per hop), only the final 1-cycle add
stage has to ripple serially across the remaining `K-1 = 15` DSPs, giving
`latency = 4 + (K-1) = 19` for `K=16`. The default value of `24` bakes in
`+5` cycles of margin, since — per the file's own header comment — this
derivation hasn't been verified against real hardware or the `tb_vpu.sv`
testbench yet. If you're tuning timing, that's the number to
bisect/re-verify with `tb_vpu.sv`.

## Port summary

```systemverilog
module vpu #(
    parameter K            = 16,   // physical cascade width
    parameter DEPTH        = 256,  // total reduction depth
    parameter PIPE_LATENCY = 24
)(
    input  logic                     clk, rst,
    input  logic                     start,
    input  logic [DEPTH*32-1:0]      activation_full, // broadcast, shared by every VPU
    input  logic [DEPTH*32-1:0]      weight_full,      // this VPU's own private column
    output logic                     busy,
    output logic                     valid,
    output logic [31:0]              dot_product
);
```

`activation_full` is the *same* wire fanned out to all 16 VPUs in an MPU;
`weight_full` is unique per VPU. See
[04_MPU_Architecture.md](04_MPU_Architecture.md) for how `mpu_top`
instantiates 16 of these and wires them up this way.

Next: [03_VPU_Linkage.md](03_VPU_Linkage.md) for the exact wiring and
cycle-by-cycle handshake between `vpu.sv`, `vpu_fsm.sv`, and
`dsp_cascade.sv`.
