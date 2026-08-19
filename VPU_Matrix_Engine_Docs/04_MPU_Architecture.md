# MPU Architecture — The Matrix Processing Unit

**Top module:** [`mpu_top.sv`](E:/vivado_projs/memory_and_vpu/memory_and_vpu.srcs/sources_1/new/mpu_top.sv)
**Control:** [`mpu_fsm.sv`](E:/vivado_projs/memory_and_vpu/memory_and_vpu.srcs/sources_1/new/mpu_fsm.sv)
**Datapath:** [`memory_controller.sv`](E:/vivado_projs/memory_and_vpu/memory_and_vpu.srcs/sources_1/new/memory_controller.sv)
**Built from:** 16x [`vpu`](02_VPU_Architecture.md) + BRAM/URAM storage (see
[10_Memory_Primitives.md](10_Memory_Primitives.md))

## Diagram

[`diagrams_mpe/MPU.drawio`](diagrams_mpe/MPU.drawio) — the MPU block
diagram: `mpu_fsm` and `memory_controller` alongside the 16-VPU array and
the three memory wrappers. Open in
[diagrams.net](https://app.diagrams.net) or VS Code's **Draw.io
Integration** extension (GitHub doesn't render `.drawio` files inline).

The MPU (Matrix Processing Unit) is the first level above the VPU. Where a
VPU produces *one* dot-product result, an MPU produces a **whole 256-wide
row of results** — it computes one complete `256 x 256` weight tile against
the shared activation vector, i.e. 256 separate 256-deep dot products,
reusing the same 16 physical VPUs across 16 sequential "column groups."

See [00_System_Overview.md](00_System_Overview.md) for where the MPU sits
in the full hierarchy, and
[05_MPU_Linkage.md](05_MPU_Linkage.md) for the exact handshake between
`mpu_fsm` and `memory_controller`.

## What an MPU computes

```
Given: activation[0..255]   (1x256, shared)
       weight[0..255][0..255] (256x256 tile, this MPU's own)
Produces: output[0..255], where
          output[c] = sum(activation[i] * weight[i][c] for i in 0..255)
```

256 output values, each a full 256-deep dot product — but the MPU only has
16 physical VPUs, not 256. So, just like the VPU reuses its `K=16` DSP
chain across 16 passes to cover a 256-deep reduction, the MPU reuses its 16
VPUs across `NUM_CGS = DEPTH / NUM_VPUS = 256 / 16 = 16` **column groups**
to cover all 256 output columns — 16 VPUs computing 16 columns at a time,
16 times over.

## Building blocks inside `mpu_top`

```
mpu_top
 ├── mpu_fsm            -- control only
 ├── memory_controller  -- datapath: fetch + assemble + commit
 ├── 16x vpu            -- compute array, unmodified
 ├── activation_bram    -- 1x256 FP32 activation storage (activations_bram.sv)
 ├── weight_uram        -- 256x256 FP32 weight storage (4096 x 512-bit words)
 └── output_bram        -- 16-deep x 512-bit output storage (16 column groups)
```

`mpu_fsm` never touches `activation_full`, `weight_full`, or `vpu_result`
directly — those wide buses flow only between `memory_controller` and the
VPU array. The FSM only issues pulses and small index values.

## The double-buffered weight prefetch — the MPU's key optimization

A naive MPU would: fetch weights for column group 0 (256 cycles), *then*
run the VPUs (17 x ~26 cycles ≈ 442 cycles), *then* commit, *then* fetch
group 1's weights, and so on — paying the weight-fetch cost serially every
time.

Instead, `memory_controller` keeps **two** weight buffers
(`weight_buf[0]` and `weight_buf[1]`). While the VPU array computes on
column group `c` (reading from one buffer), `mpu_fsm` simultaneously issues
a fetch for column group `c+1`'s weights into the *other* buffer. Since
compute for one group takes far longer (~450+ cycles) than a 256-row weight
fetch, the fetch for the *next* group is essentially free — hidden entirely
under the current group's compute time. Only the very first group's weight
fetch (`REQ_WGT0` in `mpu_fsm`) has to be paid serially, since there's
nothing yet to overlap it with.

This buffer selection is entirely address-driven, with no extra bookkeeping
register needed: because the fetch index and the compute index always
differ by exactly 1 while overlapped, their LSBs always differ, so a fetch
in flight can never write into the buffer the VPUs are currently reading.
See [05_MPU_Linkage.md](05_MPU_Linkage.md) for the exact mechanism.

## Memory layout

| Store | Module | Depth | Width | Content |
|---|---|---|---|---|
| Activation | `activation_bram` | 256 | 32 bits | 1x256 FP32 activation vector, shared by all 16 VPUs |
| Weight | `weight_uram` | 4096 (`16 groups x 256`) | 512 bits (16 x FP32) | Full 256x256 tile, addressed `{column_group, row}` |
| Output | `output_bram` | 16 | 512 bits (16 x FP32) | One 512-bit word per column group (16 VPU results packed together) |

All three are true dual-port: Port A is the host/DMA preload interface
(written before `start`, or read by the level above), Port B is the
internal read/write path used by `memory_controller` during a run. See
[10_Memory_Primitives.md](10_Memory_Primitives.md) for the generic
`bram_tdp_init`/`uram_tdp_init` primitives these wrap.

## Port summary

```systemverilog
module mpu_top #(
    parameter DATA_WIDTH = 32, NUM_VPUS = 16, DEPTH = 256,
    parameter K = 16, PIPE_LATENCY = 24
)(
    input  logic clk, rst,
    input  logic start,  output logic busy, valid,

    // Host preload -- activation BRAM (Port A)
    input logic host_act_en, host_act_we,
    input logic [7:0]  host_act_addr,   // $clog2(256)
    input logic [31:0] host_act_din,

    // Host preload -- weight URAM (Port A)
    input logic host_wgt_en, host_wgt_we,
    input logic [11:0] host_wgt_addr,   // $clog2(16*256)
    input logic [511:0] host_wgt_din,

    // Host readout -- output BRAM (Port B)
    input  logic host_out_en,
    input  logic [3:0] host_out_addr,    // $clog2(16)
    output logic [511:0] host_out_dout
);
```

One MPU consumes `256 x 32 = 8192 bits` of activation storage, `4096 x 512
= ~2 Mbit` of weight storage, and produces `16 x 512 = 8192 bits` of output
— matching the 256x256 tile it's responsible for.

Next: [05_MPU_Linkage.md](05_MPU_Linkage.md) for the full `mpu_fsm` state
sequence and how `memory_controller` streams data to/from the VPU array.
