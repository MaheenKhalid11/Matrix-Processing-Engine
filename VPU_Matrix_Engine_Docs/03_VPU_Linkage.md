# VPU Linkage — How `vpu`, `vpu_fsm`, and `dsp_cascade` Connect

This document is the wiring-level companion to
[02_VPU_Architecture.md](02_VPU_Architecture.md) — read that first for what
each module *does*; this covers exactly how they're connected and what
happens on the wires, cycle by cycle.

**Files:**
[`vpu.sv`](E:/vivado_projs/memory_and_vpu/memory_and_vpu.srcs/sources_1/new/vpu.sv) (top),
[`vpu_fsm.sv`](E:/vivado_projs/memory_and_vpu/memory_and_vpu.srcs/sources_1/new/vpu_fsm.sv) (control),
[`dsp_cascade.sv`](E:/vivado_projs/memory_and_vpu/memory_and_vpu.srcs/sources_1/new/dsp_cascade.sv)
(datapath).

> Note: there is also a `vpu_fsm.v` file in the sources tree. It is an
> empty auto-generated stub (`module vpu_fsm(); endmodule`) left over from
> project scaffolding — the real, active FSM is the SystemVerilog
> `vpu_fsm.sv` documented here. Don't confuse the two when browsing the
> `sources_1/new` folder.

## Diagram — the DSP chain

[`diagrams_mpe/DSP_CASCADE.drawio`](diagrams_mpe/DSP_CASCADE.drawio) — the
`pcin`/`pcout` chain inside `dsp_cascade.sv`: how `pass_cnt` slices the
16-wide activation/weight buses, how `acc_reg` muxes into the head of the
chain (`pc[0]`), and how each `dspfp32_pe`'s `pcout` feeds the next one's
`pcin` down to `chain_out`. Open in
[diagrams.net](https://app.diagrams.net) or VS Code's **Draw.io
Integration** extension (GitHub doesn't render `.drawio` files inline).

## The three modules and their one job each

```
vpu.sv               -- pure wiring, no logic of its own
 ├── vpu_fsm.sv       -- control: state machine, drives ce_*/pass_cnt/acc_reg
 └── dsp_cascade.sv   -- datapath: 16x dspfp32_pe chain + bus slicing/muxing
```

`vpu.sv` itself contains no behavior — it just instantiates `vpu_fsm` and
`dsp_cascade` and connects their handshake wires together. That handshake
is the entire content of this document.

## The handshake wires between `vpu_fsm` and `dsp_cascade`

| Signal | Direction | Meaning |
|---|---|---|
| `ce_a`, `ce_b`, `ce_pipe` | fsm → cascade | Clock-enables for the DSP chain's A/B input regs and pipeline regs |
| `pass_cnt` | fsm → cascade | Which of the 16 passes is active — selects the 16-wide bus slice |
| `acc_reg` | fsm → cascade | Running total, fed into the head of the chain (`pc[0]`) |
| `chain_out` | cascade → fsm | The chain's tail output (`pc[K]`) — this pass's partial result |

Everything else (`activation_full`, `weight_full`, `dot_product`) is wired
directly at the `vpu.sv` top level, not through the FSM.

## How `dsp_cascade` builds one pass

```systemverilog
wire [K*32-1:0] act_slice = activation_full[pass_cnt*K*32 +: K*32];
wire [K*32-1:0] wt_slice  = weight_full[pass_cnt*K*32 +: K*32];
wire [31:0] acc_in = (pass_cnt == 0) ? 32'b0 : acc_reg;

wire [31:0] pc [0:K];
assign pc[0] = acc_in;
// pc[i+1] = dspfp32_pe(a=act_slice[i], b=wt_slice[i], pcin=pc[i])
assign chain_out = pc[K];
```

Three things worth noticing:

1. **Bus slicing is address-driven, not sequential shifting.** The full
   256-wide activation/weight buses live entirely outside the VPU (in
   `memory_controller`, one level up — see
   [05_MPU_Linkage.md](05_MPU_Linkage.md)); `dsp_cascade` just indexes into
   them combinationally using `pass_cnt`. No data is ever shifted through
   registers to get to the right slice.
2. **`acc_reg` only feeds in on passes 1..15, not pass 0.** On `pass_cnt==0`
   the head of the chain (`pc[0]`) is forced to `32'b0` instead of
   `acc_reg`, so the first real pass starts a fresh sum. (During the
   priming pass, `pass_cnt` is *also* 0, so this is harmless there too —
   its result gets thrown away by the FSM regardless.)
3. **Every `dspfp32_pe` in the chain is instantiated with `IS_FIRST(0)`.**
   The `pc[0]` mux above does the job that the DSP macro's own
   `IS_FIRST`/`FPOPMODE_FIRST` mode would otherwise do — see
   [01_DSPFP32_Building_Block.md](01_DSPFP32_Building_Block.md) for why this
   choice was made (uniform DSPs, decision lives in control logic instead).

## Cycle-by-cycle walkthrough of one pass

Assume `vpu_fsm` is in `LOAD` for pass `p` (not the priming pass):

1. **LOAD (1 cycle):** `ce_a=1, ce_b=1, ce_pipe=1`. `dsp_cascade` combinationally
   presents `act_slice`/`wt_slice` for pass `p` and `acc_in` (0 or `acc_reg`)
   at `pc[0]`; the clock edge latches these into the 16 DSPs' A/B input
   registers.
2. **RUN (`PIPE_LATENCY` cycles, default 24):** `ce_a=0, ce_b=0, ce_pipe=1`.
   The 16 multiplies happen in parallel inside the DSPs; the resulting
   partial sums ripple through the `pc[0]→pc[16]` chain one add-stage at a
   time. `vpu_fsm`'s `wait_cnt` counts up to `PIPE_LATENCY`.
3. **CAPTURE (1 cycle):** `chain_out` (`= pc[16]`) is now valid.
   `vpu_fsm` latches it into `acc_reg`. If `pass_cnt == NUM_PASSES-1` (the
   16th pass), it *also* latches into `dot_product` and pulses `valid`;
   otherwise `pass_cnt` increments and the FSM returns to LOAD for pass
   `p+1`, which will read the just-updated `acc_reg` back out through
   `acc_in`.

This repeats 16 times (plus the one discarded priming pass at the very
start), for a total of `1 (priming) + 16 (real passes) = 17` LOAD→RUN→CAPTURE
cycles per `start` pulse, each costing `1 + PIPE_LATENCY + 1` clock cycles.

## Top-level (`vpu.sv`) port wiring

```systemverilog
vpu_fsm #( .DEPTH(DEPTH), .K(K), .PIPE_LATENCY(PIPE_LATENCY) ) u_fsm (
    .clk(clk), .rst(rst), .start(start),
    .chain_out(chain_out),                 // <- from dsp_cascade
    .ce_a(ce_a), .ce_b(ce_b), .ce_pipe(ce_pipe),  // -> to dsp_cascade
    .busy(busy), .valid(valid),
    .acc_reg(acc_reg), .pass_cnt(pass_cnt),       // -> to dsp_cascade
    .dot_product(dot_product)
);

dsp_cascade #( .K(K), .DEPTH(DEPTH) ) u_datapath (
    .clk(clk), .rst(rst),
    .ce_a(ce_a), .ce_b(ce_b), .ce_pipe(ce_pipe),  // <- from vpu_fsm
    .pass_cnt(pass_cnt), .acc_reg(acc_reg),       // <- from vpu_fsm
    .activation_full(activation_full),            // <- from vpu's own top-level port
    .weight_full(weight_full),                    // <- from vpu's own top-level port
    .chain_out(chain_out)                         // -> to vpu_fsm
);
```

`busy`, `valid`, and `dot_product` are `vpu_fsm`'s registered outputs,
passed straight through as `vpu.sv`'s own outputs — the top level adds no
extra registering or logic to them.

## Where this connects upward

`vpu.sv`'s three top-level ports (`start`/`activation_full`/`weight_full`
in, `busy`/`valid`/`dot_product` out) are exactly what `mpu_top` drives when
it instantiates 16 VPUs. See
[05_MPU_Linkage.md](05_MPU_Linkage.md) for how `memory_controller` feeds
`activation_full` (shared) and `weight_full[i]` (per-VPU) into this
interface, and how `mpu_fsm` uses the aggregated `vpu_valid` bus to know
when all 16 VPUs are done.
