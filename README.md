FP32 Matrix-Vector Engine (Versal DSPFP32)

An FPGA compute engine that performs a full single-precision (FP32)
GEMV — `[1 x 4096] . [4096 x 4096] -> [1 x 4096]` — entirely in hardware,
built on AMD/Xilinx Versal's dedicated `DSPFP32` hard macro. No soft-logic
floating point is used anywhere in the design: every multiply and every add
runs on native FP32 DSP silicon.

Built and simulated in **Vivado 2025.2**, targeting part
`xcv80-lsva4737-2MHP-e-S` (AMD Versal, V80 series).

## What it does

Computes one dense layer's worth of matrix-vector math — the operation that
dominates inference in a fully-connected neural network layer:

```
activation (1x4096, FP32)  .  weight (4096x4096, FP32)  =  output (1x4096, FP32)
```

The reduction dimension and the output width are both spread across a
five-level hierarchy of identical, repeated hardware blocks, all reducing
down to one proven primitive:

```
mpe_top        the whole engine  (1x4096 . 4096x4096 -> 1x4096)
 └── group_top x8   256-column slice, full 4096-deep reduction
      └── mpu_top x4   256x256 tile, 256-deep partial reduction
           └── vpu x16     one 256-deep FP32 dot product
                └── dsp_cascade  16x chained DSPFP32 MACs
                     └── dspfp32_pe   one DSPFP32 hard-macro instance
```

Every level above the DSP primitive follows the same pattern: an `*_fsm`
module that is the sole command authority (pure control, no wide data
buses) paired with a `*_datapath` module that owns every wide bus and does
the actual streaming. See **Architecture documentation** below for the full
write-up of why this split exists and how the numbers at each level fit
together.

## Key design points

- **Zero soft-logic floating point.** Every multiply-accumulate and every
  add is a `DSPFP32` hard macro (`dspfp32_pe`, wrapping the Versal
  `DSPFP32` primitive) — ~8,320 DSP instances in the default configuration.
- **Reused, not duplicated, hardware.** A physical 16-wide DSP cascade
  covers a 256-deep reduction over 16 passes; 16 physical VPUs cover 256
  output columns over 16 compute passes — the same reuse pattern repeats at
  every level instead of instantiating dedicated hardware per data point.
- **Double-buffered weight prefetch** inside each MPU hides weight-fetch
  latency completely behind VPU compute time.
- **Deliberate serial vs. parallel tradeoffs.** Compute and data reload
  run fully parallel across MPUs/groups; the FP32 accumulate/combine
  adders and the final output readout are each a *single shared* hardware
  block, time-multiplexed across many parallel compute units — a
  documented tradeoff (add/readout time is negligible next to compute
  time, so sharing there costs latency, not throughput, in exchange for
  far less hardware).
- **Adderless address generation** throughout — every level's address
  concatenation (`{group_sel, pass_sel, row}` etc.) works out to be
  numerically identical to a multiply-add because every dimension is a
  power of two.

## Repository layout

```
memory_and_vpu.srcs/
 ├── sources_1/new/     RTL sources (SystemVerilog + one Verilog primitive)
 │    ├── dsp.v                  DSPFP32 hard-macro wrapper (dspfp32_pe)
 │    ├── dsp_cascade.sv         16-wide chained MAC datapath
 │    ├── vpu.sv / vpu_fsm.sv    Vector Processing Unit (256-deep dot product)
 │    ├── memory_controller.sv   MPU-level fetch/commit datapath
 │    ├── mpu_top.sv / mpu_fsm.sv           Matrix Processing Unit (256x256 tile)
 │    ├── group_datapath.sv
 │    ├── group_top.sv / group_fsm.sv       Group (4 MPUs, 4096-deep reduction)
 │    ├── mpe_datapath.sv
 │    ├── mpe_top.sv / mpe_fsm.sv           top-level engine (8 groups, 2 passes)
 │    ├── accum_adder16.sv / vecadd_fsm.sv  shared FP32 vector-add bank
 │    ├── activations_bram.sv / weight_uram.sv / output_bram.sv   storage wrappers
 │    └── bram_tdp_init.sv / uram_tdp_init.sv                     generic dual-port memory
 └── sim_1/new/          Testbenches (see below)
```

## Simulating

Open `memory_and_vpu.xpr` in Vivado, or run the simulator directly on the
sources under `memory_and_vpu.srcs/sim_1/new/`. Testbenches are organized
bottom-up, matching the hierarchy:

| Testbench | Exercises |
|---|---|
| `tb_vpu.sv` | `vpu` — single 256-deep dot product, used to verify/tune `PIPE_LATENCY` |
| `vecadd_tb.sv` | `vecadd_fsm` + `accum_adder16` — the shared FP32 adder bank |
| `mpu_tb.sv` | `mpu_top` — one 256x256 tile, including the weight prefetch |
| `group_tb.sv`, `reload_group_tb.sv`, `group2pass_tb.sv` | `group_top` — chunked reduction and multi-pass reload |
| `mpe_tb.sv`, `mpe2g_tb.sv`, `mpe8small_tb.sv`, `mpe8mid_tb.sv`, `mpe8fulld_tb.sv` | `mpe_top` at increasing scale (2 groups up to the full 8-group / full-depth configuration) |

## Status / known caveats

- `PIPE_LATENCY` (default `24`, for `K=16`) is a *derived* estimate for the
  DSPFP32 cascade's settle time, not yet confirmed against real hardware —
  see the header comment in `vpu.sv` and bisect it with `tb_vpu.sv` before
  trusting it on silicon.
- Group-level chunk loading and MPU compute currently run fully serially
  (`group_fsm` is "v1" per its own header comment) — a prefetch overlap
  similar to `mpu_fsm`'s weight prefetch is a possible future optimization
  if load time turns out to matter next to compute time.
- No constraints (`.xdc`) file is present yet in this project.

## Architecture documentation

A detailed, cross-linked documentation set — system overview, the DSPFP32
primitive, and a dedicated architecture + linkage document for every level
(VPU, MPU, Group, MPE) plus the shared memory primitives — is maintained
alongside this project. If you're setting this repo up on GitHub, add
those docs under a `docs/` folder at the repo root and they'll cross-link
correctly using relative paths back into `memory_and_vpu.srcs/sources_1/new/`.

## License

No license file is currently included in this repository. 
