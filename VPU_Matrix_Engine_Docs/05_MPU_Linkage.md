# MPU Linkage — How `mpu_top`, `mpu_fsm`, and `memory_controller` Connect

Companion to [04_MPU_Architecture.md](04_MPU_Architecture.md) — read that
first for the "why", this covers the exact handshake and cycle sequencing.

**Files:**
[`mpu_top.sv`](E:/vivado_projs/memory_and_vpu/memory_and_vpu.srcs/sources_1/new/mpu_top.sv) (top),
[`mpu_fsm.sv`](E:/vivado_projs/memory_and_vpu/memory_and_vpu.srcs/sources_1/new/mpu_fsm.sv) (control),
[`memory_controller.sv`](E:/vivado_projs/memory_and_vpu/memory_and_vpu.srcs/sources_1/new/memory_controller.sv)
(datapath).

## The handshake wires between `mpu_fsm` and `memory_controller`

| Signal | Direction | Meaning |
|---|---|---|
| `load_act` | fsm → mc | Pulse: fetch the 1x256 activation vector |
| `act_done` | mc → fsm | Pulse: `activation_full` is now valid |
| `load_weights` | fsm → mc | Pulse: fetch one column group's weights |
| `fetch_cg_index` | fsm → mc | Which column group to fetch weights for |
| `weights_done` | mc → fsm | Pulse: that fetch's buffer is now valid |
| `commit_output` | fsm → mc | Pulse: latch `vpu_result[]` and write it out |
| `compute_cg_index` | fsm → mc | Which group is on the VPU array / being committed |
| `output_done` | mc → fsm | Pulse: the output write has landed |
| `mem_busy` | mc → fsm | Combinational: any fetch currently in flight |

Every operation gets its **own** dedicated one-cycle completion pulse —
`act_done`/`weights_done`/`output_done` are never shared or aliased. This
is a deliberate design decision (documented in `memory_controller.sv`'s
header) fixing an earlier race where a single shared "data ready" signal
could be misread as belonging to the wrong phase. Each pulse is asserted
only by the specific state that performed that specific operation.

There's also a `SYSTEMVERILOG` assertion in `memory_controller.sv`
(simulation-only, `` `ifndef SYNTHESIS``) that catches `mpu_fsm` ever
asserting more than one of `load_act`/`load_weights`/`commit_output`
simultaneously — a protocol guard, not functional logic.

## `mpu_fsm`'s state sequence

```
IDLE
 -> REQ_ACT     (pulse load_act, once per run)
 -> WAIT_ACT    (poll act_done)
 -> REQ_WGT0    (pulse load_weights for group 0 -- priming fetch)
 -> WAIT_WGT0   (poll weights_done)
 -> REQ_VPU     (pulse vpu_start for compute_idx;
                 if there IS a next group, ALSO pulse load_weights
                 for compute_idx+1 in the same cycle -- the prefetch)
 -> WAIT_VPU    (poll vpu_valid AND, if a prefetch was launched,
                 weights_done for it too)
 -> REQ_COMMIT  (pulse commit_output for compute_idx)
 -> WAIT_COMMIT (poll output_done)
 -> [compute_idx == NUM_CGS-1 ? DONE : REQ_VPU]
 -> DONE (pulse valid) -> IDLE
```

This is "pipelined by one group": while group `c` computes, group `c+1`'s
weights are already being fetched. See
[04_MPU_Architecture.md](04_MPU_Architecture.md)'s "double-buffered weight
prefetch" section for why this matters.

### The sticky-done trick in `WAIT_VPU`

Two independent events have to both complete before moving on: all 16 VPUs
finishing (`all_vpus_done = &vpu_valid`) and the overlapped weight prefetch
finishing (`weights_done`). These can arrive in either order, and each is
only a *one-cycle pulse* — if VPU compute finishes first and the FSM simply
polled `all_vpus_done && weights_done` on the same cycle, it could miss a
pulse that already happened.

`mpu_fsm` solves this with two sticky latch flags, cleared each time a new
`REQ_VPU` starts:

```systemverilog
assign ready_to_commit = (all_vpus_done || vpu_seen_done) &&
                          (!has_next || weights_done || wgt_seen_done);
```

`vpu_seen_done`/`wgt_seen_done` latch high the moment their respective pulse
is observed, and stay high until the next `REQ_VPU`, so it doesn't matter
which pulse arrives first or whether they land on different cycles.

## How `memory_controller` fetches and assembles data

`memory_controller` has its own small state machine (separate from
`mpu_fsm`) handling `load_act`/`load_weights`:

```
IDLE -> FETCH_ACT -> IDLE     (on load_act)
IDLE -> FETCH_WGT -> IDLE     (on load_weights)
```

Both fetch states stream 256 rows out of BRAM/URAM Port B, one row per
cycle, using an address generator that is deliberately **adderless** for
the weight case — since `DEPTH` and `NUM_CGS` are both powers of two,
`{fetch_cg_index, row}` (a plain bit concatenation) is numerically
identical to `fetch_cg_index * DEPTH + row`, so no multiply/add hardware is
needed to compute the address.

- **`FETCH_ACT`**: reads `activation_bram` Port B, row 0..255, and
  assembles each 32-bit word into the right slice of the 8192-bit
  `activation_full` register (`activation_full[cnt*32 +: 32] <= act_b_dout`).
- **`FETCH_WGT`**: reads `weight_uram` Port B, row 0..255 of column group
  `fetch_cg_index`. Each 512-bit row is unpacked into 16 separate per-VPU
  256-deep buffers (`weight_buf[fetch_buf_sel][v][cnt*32 +: 32] <= ...`),
  one 32-bit column per VPU.

`fetch_buf_sel` — which of the two weight buffers this fetch writes into —
is captured *once*, from `fetch_cg_index[0]`, at the moment `load_weights`
pulses, and held stable for the whole fetch. This matters because
`fetch_cg_index` itself may move on to describe the *next* prefetch before
this fetch's writes have all finished landing; capturing the buffer
selection up front prevents a fetch from drifting into the wrong buffer
partway through.

`weight_full[v]` (what's actually driven to VPU `v`) is a continuous,
purely combinational mux:

```systemverilog
assign weight_full[v] = weight_buf[compute_cg_index[0]][v];
```

— always presenting whichever buffer holds `compute_cg_index`'s data, i.e.
never the buffer a fetch might currently be writing into.

## Committing output — independent of the fetch state machine

`commit_output` is handled entirely separately from `FETCH_ACT`/`FETCH_WGT`,
since a register write needs no multi-cycle sequencing:

```systemverilog
if (commit_output) begin
    out_a_addr <= compute_cg_index;
    for (i = 0; i < 16; i++)
        out_a_din[i*32 +: 32] <= vpu_result[i];   // pack all 16 VPU results
    output_done <= 1'b1;  // completes the same registered edge
end
```

All 16 VPUs' single-cycle FP32 results are packed side-by-side into one
512-bit word and written to `output_bram` in a single cycle, at the address
matching the current column group.

## Read-pipeline detail worth knowing

Both `activation_bram`/`weight_uram` (via `bram_tdp_init`/`uram_tdp_init`)
are **registered-read** memories: asserting `en` on cycle `N` returns data
on cycle `N+1`. `memory_controller` accounts for this with a 1-cycle
`read_valid` shadow register (`read_valid <= act_b_en | wgt_b_en`) so that
the "just arrived" word is captured on the correct cycle, and a `cnt`
counter that only advances when `read_valid` is high — i.e. `cnt` always
matches the index of the data that has *just* landed on the read-data bus,
not the index that was just issued.

## Top-level (`mpu_top.sv`) wiring

`mpu_top` instantiates `mpu_fsm`, `memory_controller`, 16x `vpu`, and the
three memory wrappers, connecting:

- `mpu_fsm` ↔ `memory_controller`: the handshake table above.
- `mpu_fsm.vpu_start` → broadcast `start` to all 16 `vpu` instances.
- `vpu_valid[i]` (one bit per VPU) → `mpu_fsm`, ANDed together as
  `all_vpus_done`.
- `memory_controller.activation_full` → broadcast to all 16 `vpu`
  instances' `activation_full` port (identical wire, fanned out).
- `memory_controller.weight_full[i]` → `vpu[i]`'s own private
  `weight_full` port.
- `vpu[i].dot_product` → `memory_controller.vpu_result[i]`, used only
  during `commit_output`.
- Host DMA ports (`host_act_*`, `host_wgt_*`, `host_out_*`) → straight
  through to the three memory wrappers' Port A/B, bypassing
  `memory_controller` entirely (it only ever touches Port B, read-only, of
  activation/weight, and Port A write-only of output).

## Where this connects upward

`mpu_top`'s host DMA ports and `start`/`busy`/`valid` handshake are exactly
what `group_datapath` drives when it instantiates 4 unmodified `mpu_top`
instances. See [07_Group_Linkage.md](07_Group_Linkage.md) for how a
Group's "loader" streams chunk data into these ports and its "readout"
streams `mpu_top`'s output back out.
