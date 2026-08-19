# The DSPFP32 Building Block

**File:** [`dsp.v`](E:/vivado_projs/memory_and_vpu/memory_and_vpu.srcs/sources_1/new/dsp.v)
**Module:** `dspfp32_pe`

## Diagram

[`diagrams_mpe/DSP.drawio`](diagrams_mpe/DSP.drawio) — the `dspfp32_pe`
wrapper: its ports, the `FPOPMODE` mux, and how it maps onto the `DSPFP32`
hard macro. Open in [diagrams.net](https://app.diagrams.net) or VS Code's
**Draw.io Integration** extension (GitHub doesn't render `.drawio` files
inline).

Every single multiply and every single add in this entire engine — from the
lowest VPU dot product up to the Group-level accumulate tree — is built out
of one primitive: `dspfp32_pe`. It's worth understanding this module well
before looking at anything built on top of it, because nothing above it
introduces any new floating-point arithmetic; everything else is wiring,
sequencing, and reuse of this one block.

Related: [02_VPU_Architecture.md](02_VPU_Architecture.md) (the first thing
built from it) and [06_Group_Architecture.md](06_Group_Architecture.md)
(where the *same* primitive is reused as a plain FP adder, not a
multiply-accumulator).

## What it wraps

`dspfp32_pe` is a thin Verilog wrapper around the Xilinx **`DSPFP32`** hard
macro — a dedicated hardware primitive available on newer Xilinx/AMD FPGAs
(e.g. Versal) that natively performs **single-precision (FP32) IEEE-754
floating-point multiply and add**, entirely in dedicated silicon, no soft
logic. Using the hard macro instead of synthesizing FP32 arithmetic from
LUTs/soft logic is what makes the huge instance counts elsewhere in this
design (thousands of DSPs across the full MPE) practical to fit and run at
speed.

## Interface

```systemverilog
module dspfp32_pe #(
    parameter IS_FIRST = 0
) (
    input  wire        clk,
    input  wire        rst,
    input  wire         ce_a,
    input  wire         ce_b,
    input  wire         ce_pipe,
    input  wire [31:0]  a_row,      // FP32 operand A (the multiplicand)
    input  wire [31:0]  b_weight,   // FP32 operand B (the multiplier)
    input  wire [31:0]  pcin,       // running partial sum coming IN
    output wire [31:0]  pcout       // running partial sum going OUT
);
```

- **`a_row` / `b_weight`** — two FP32 operands, in standard IEEE-754 32-bit
  format (sign / 8-bit exponent / 23-bit mantissa). The naming reflects the
  original use — one activation row value, one weight value — but the
  primitive itself is a general FP32 multiply-accumulate cell and gets
  reused elsewhere (see below) with different operand meanings.
- **`pcin` / `pcout`** — the cascade ports. `pcout` of one instance is wired
  directly to `pcin` of the next, letting many `dspfp32_pe`s be chained into
  a reduction tree/chain without routing an explicit adder between them —
  the accumulation happens *inside* each DSP.
- **`ce_a` / `ce_b` / `ce_pipe`** — clock-enables for the A/B input
  registers and the internal pipeline registers respectively. Control
  logic outside this module (`vpu_fsm`, `vecadd_fsm`) drives these to
  step data through the pipe one stage at a time.
- **`IS_FIRST`** — a compile-time parameter selecting the DSP's operating
  mode (see below). In this design it's always instantiated with
  `IS_FIRST=0` everywhere it's used — see "Why IS_FIRST is unused" below.

## What it computes

Internally the wrapper picks one of two `FPOPMODE` settings for the
`DSPFP32` macro:

```systemverilog
localparam [6:0] FPOPMODE_FIRST = 7'h01; // P = 0 + M
localparam [6:0] FPOPMODE_CHAIN = 7'h1D; // P = PCIN + M
wire [6:0] fpopmode_c = IS_FIRST ? FPOPMODE_FIRST : FPOPMODE_CHAIN;
```

Where `M = a_row * b_weight` (the internal FP32 multiply) and `P` is the
DSP's output (`pcout`). So functionally, every instance computes:

```
pcout = pcin + (a_row * b_weight)
```

This is a classic **multiply-accumulate (MAC)**: multiply two FP32 numbers,
add the result to an incoming running sum, and pass the new running sum
downstream. `FPOPMODE_FIRST` (`P = 0 + M`, i.e. ignore `pcin`) exists as an
option in the macro for starting a fresh chain with no prior partial sum,
but this design never uses it — every consumer instead injects the "start
value" onto `pcin` itself via an external mux (see below), always leaving
`IS_FIRST=0` / `FPOPMODE_CHAIN` selected.

### Why `IS_FIRST` is effectively unused

Both places that instantiate `dspfp32_pe` —
[`dsp_cascade.sv`](E:/vivado_projs/memory_and_vpu/memory_and_vpu.srcs/sources_1/new/dsp_cascade.sv)
and
[`accum_adder16.sv`](E:/vivado_projs/memory_and_vpu/memory_and_vpu.srcs/sources_1/new/accum_adder16.sv)
— always pass `IS_FIRST(0)`. Instead of using the macro's own "no `pcin`"
mode for the first element of a reduction, they mux the correct starting
value (usually `32'b0`) directly onto the `pcin` wire from outside. This
keeps every DSP in a chain electrically identical and lets control logic
(the FSMs) decide what "start of a reduction" means without needing a
different `FPOPMODE` per DSP position. See
[03_VPU_Linkage.md](03_VPU_Linkage.md) for exactly how `dsp_cascade` does
this muxing.

## Reused as a plain adder, not just a MAC

[`accum_adder16.sv`](E:/vivado_projs/memory_and_vpu/memory_and_vpu.srcs/sources_1/new/accum_adder16.sv)
(used inside a Group's accumulate tree — see
[06_Group_Architecture.md](06_Group_Architecture.md)) instantiates 16 of
these same `dspfp32_pe` cells, but ties `a_row` to the FP32 constant `1.0`
(`32'h3F80_0000`):

```
pcout = pcin + (1.0 * b_weight) = pcin + b_weight
```

Multiplying by exactly `1.0` in IEEE-754 is a no-op on the mantissa/exponent,
so this collapses the MAC into a pure elementwise FP32 **add**, using
exactly the same verified hardware and timing behavior as the multiply path
— no new arithmetic logic needs to exist anywhere else in the design for
addition.

## Pipeline configuration

The `DSPFP32` instantiation configures several internal pipeline registers
(`AREG=1`, `FPBREG=1`, `FPCREG=3`, `FPMPIPEREG=1`, `FPA_PREG=1`,
`FPM_PREG=1`, `FPOPMREG=3`, `INMODEREG=1`, ...). These add up to a fixed,
multi-cycle latency from valid inputs to a valid `pcout`, which is why
every FSM that drives a `dspfp32_pe` (directly or via a cascade) has to wait
a known number of cycles after asserting its clock-enables before treating
the DSP's output as trustworthy — see the "pipeline latency" discussion in
[02_VPU_Architecture.md](02_VPU_Architecture.md).

Reset (`RSTA`/`RSTB`/`RSTC`/`RSTD`/`RSTFPA`/etc.) is synchronous
(`RESET_MODE("SYNC")`), and the `C` and `D` operand paths of the macro are
tied off (`C = 32'b0`, `D = 8'b0/23'b0/1'b0`) since this design only ever
uses the `A*B (+PCIN)` multiply-accumulate path, never the macro's separate
`C`/`D` adder inputs.

## Where it's instantiated

| Consumer | Count | Role |
|---|---|---|
| [`dsp_cascade.sv`](E:/vivado_projs/memory_and_vpu/memory_and_vpu.srcs/sources_1/new/dsp_cascade.sv) | `K=16` per VPU | Chained MAC — one 16-wide reduction pass of a VPU's dot product |
| [`accum_adder16.sv`](E:/vivado_projs/memory_and_vpu/memory_and_vpu.srcs/sources_1/new/accum_adder16.sv) | 16 per Group | Independent, `a_row` tied to `1.0` — plain FP32 elementwise add lanes |

Next: [02_VPU_Architecture.md](02_VPU_Architecture.md) for how 16 of these
cells get chained into a full 256-deep dot-product engine.
