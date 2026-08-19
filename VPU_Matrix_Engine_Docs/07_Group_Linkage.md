# Group Linkage — How `group_top`, `group_datapath`, and `group_fsm` Connect

Companion to [06_Group_Architecture.md](06_Group_Architecture.md) — read
that first for the "why", this covers the exact handshake and cycle
sequencing.

**Files:**
[`group_top.sv`](E:/vivado_projs/memory_and_vpu/memory_and_vpu.srcs/sources_1/new/group_top.sv) (top),
[`group_fsm.sv`](E:/vivado_projs/memory_and_vpu/memory_and_vpu.srcs/sources_1/new/group_fsm.sv)
(control),
[`group_datapath.sv`](E:/vivado_projs/memory_and_vpu/memory_and_vpu.srcs/sources_1/new/group_datapath.sv)
(datapath).

## The handshake wires between `group_fsm` and `group_datapath`

| Signal | Direction | Meaning |
|---|---|---|
| `load_chunk` | fsm → dp | Pulse: broadcast to all 4 MPUs' loaders |
| `chunk_idx` | fsm → dp | Which chunk (0..3) to load |
| `all_chunks_loaded` | dp → fsm | All 4 loaders finished (AND of 4 per-MPU flags) |
| `mpu_start` | fsm → dp | Pulse: broadcast `start` to all 4 `mpu_top`s |
| `all_mpu_valid` | dp → fsm | All 4 MPUs finished compute (AND of 4 `valid`s) |
| `do_readout` | fsm → dp | Pulse: broadcast, streams each MPU's own `output_bram` into a register |
| `all_readout_done` | dp → fsm | All 4 readouts finished |
| `vecadd_start` | fsm → dp | Pulse: kick the *shared* adder for one operation |
| `operand_sel` | fsm → dp | Which operation (see the encoding table in the architecture doc) |
| `vecadd_done` | dp → fsm | The shared adder finished that one operation |
| `accum_clear` | fsm → dp | Pulse, once per run: zero the persistent accumulator array |

## `group_fsm`'s full state sequence

```
IDLE
 -> [ REQ_LOAD    (pulse load_chunk, broadcast to all 4 MPUs -- parallel)
      WAIT_LOAD   (poll all_chunks_loaded)
      REQ_MPU     (pulse mpu_start, broadcast to all 4 mpu_tops)
      WAIT_MPU    (poll all_mpu_valid)
      REQ_READOUT (pulse do_readout, broadcast -- parallel per-MPU)
      WAIT_READOUT(poll all_readout_done)
      REQ_ACCUM x4 (pulse vecadd_start, operand_sel = 0,1,2,3 --
                    looping the ONE shared adder across all 4 MPUs'
                    self-accumulate, one at a time)
      WAIT_ACCUM
    ] repeated for chunk_idx = 0 .. NUM_CHUNKS-1 (4 times)
 -> REQ_COMBINE x3 (pulse vecadd_start, operand_sel = 4, 5, 6 --
      the 3 sequential combine stages; stage 3 writes straight into
      group_datapath's output_bram)
 -> WAIT_COMBINE
 -> DONE (pulse valid) -> IDLE
```

`accum_clear` is pulsed exactly once, in the `S_IDLE` state on the cycle
`start` is seen — before the very first `REQ_LOAD`.

This design is explicitly documented as **v1, fully serial** — chunk-load
and MPU-compute are *not* overlapped/prefetched the way `mpu_fsm` overlaps
weight-fetch with VPU compute (see
[05_MPU_Linkage.md](05_MPU_Linkage.md)). The source comment notes this is
the same "get it correct first" reasoning `mpu_fsm` itself originally used,
with the same prefetch trick available to layer on top later if
chunk-load time turns out to matter next to `4x mpu_top`'s compute time.

## Inside `group_datapath`: per-MPU chunk stores, loader, `mpu_top`, readout

For each of the 4 MPUs, `group_datapath` (in a `generate for` loop) owns:

- A **weight chunk store** (`uram_tdp_init`, ~1 MiB) holding *all*
  `NUM_CHUNKS=4` chunks' `256x256` tiles for that MPU, and an
  **activation chunk store** (`activation_bram`, depth
  `NUM_CHUNKS*MPU_DEPTH`) — both preloaded once from the host, via their
  own Port A.
- A **loader**: on `load_chunk`, streams the chunk selected by `chunk_idx`
  out of those two chunk stores and into that MPU's own `mpu_top` host
  ports (`mt_host_act_*`/`mt_host_wgt_*`). This directly mirrors
  `memory_controller`'s own `FETCH_ACT`/`FETCH_WGT` streaming pattern (see
  [05_MPU_Linkage.md](05_MPU_Linkage.md)) — just writing *outward* into a
  downstream `mpu_top` instance instead of assembling an internal register.
- An unmodified `mpu_top` instance, whose host ports are driven entirely
  internally by this loader/readout — never exposed to the Group's own
  external ports.
- A **readout**: after `mpu_top`'s `valid` pulses, streams its 16
  column-group rows out of its own `output_bram` into a plain register
  array, `mpu_readout[mpu][0..15]` — mirrors the same "fetch into a
  register" pattern `memory_controller` uses for `activation_full`.

All 4 MPUs' loaders and readouts run in **full parallel** — each owns its
own physical stores/ports, so there's no time-multiplexing here, unlike the
shared adder (see the architecture doc's "two parallelism strategies"
section).

### The `L_WGT_START` priming state — a subtle timing fix

The loader's state machine is `L_IDLE → L_ACT → L_WGT_START → L_WGT →
L_IDLE`. The extra `L_WGT_START` state (instead of just going straight from
`L_ACT` to `L_WGT`) exists to fix a real timing bug that was found during
development, documented directly in the source:

Priming `wgt_chunk_b_addr = 0` inside `L_ACT`'s own last-iteration branch
(the original, simpler approach) primes the address **one cycle before**
the state register actually transitions into `L_WGT` — because the
transition itself waits on the *capture* side catching up, not the *issue*
side. That leaves the weight address sitting stagnant at 0 for one extra
cycle before `L_WGT`'s own advance logic starts, which means row 0 gets
sampled *twice* and the real last row is lost. Splitting the priming step
into its own dedicated one-cycle state (`L_WGT_START`) gives the weight
fetch the exact same "prime once, then advance from the next cycle" timing
that the activation fetch already gets for free by using `L_IDLE` as its
own priming state.

This is a good example of a class of off-by-one timing bug that shows up
repeatedly in this codebase wherever a state transition depends on when a
*capture* (not just an *issue*) actually completes — worth remembering if
you're modifying any of the loader/readout state machines at any level.

## The shared adder: issue-side mux and capture-side writeback

`group_datapath` module-scope declares three shared register arrays that
the adder mux reads/writes at runtime, indexed by `operand_sel`:

```systemverilog
logic [511:0] accumulator [0:3][0:15];    // per-MPU running totals
logic [511:0] mpu_readout [0:3][0:15];    // just-read MPU results
logic [511:0] combine_temp [0:15];        // scratch for the 3-stage combine
```

**Issue side** (feeds `accum_adder16`'s `a_vec`/`b_vec`):

```systemverilog
case (operand_sel)
    0: a_vec = accumulator[0][row]; b_vec = mpu_readout[0][row];
    1: a_vec = accumulator[1][row]; b_vec = mpu_readout[1][row];
    2: a_vec = accumulator[2][row]; b_vec = mpu_readout[2][row];
    3: a_vec = accumulator[3][row]; b_vec = mpu_readout[3][row];
    4: a_vec = accumulator[0][row]; b_vec = accumulator[1][row];
    5: a_vec = combine_temp[row];   b_vec = accumulator[2][row];
    6: a_vec = combine_temp[row];   b_vec = accumulator[3][row];
endcase
```

**Capture side** (writes `accum_adder16`'s `sum_vec` back to the right
destination, gated on `va_capture_valid` from `vecadd_fsm`):

- `operand_sel < 4`: `accumulator[operand_sel[1:0]][row] <= sum_vec`
- `operand_sel == 4 || 5`: `combine_temp[row] <= sum_vec`
- `operand_sel == 6`: written directly into `output_bram` (see below),
  not into any register array — this is the Group's final result.

## The Group's `output_bram` write

```systemverilog
assign out_a_en = va_capture_valid && (operand_sel == 6);
assign out_a_we = out_a_en;
// a_addr = va_capture_row_sel, a_din = va_sum_vec
```

Unlike the MPU level (where output is committed as one batched 512-bit
write covering all 16 VPUs at once — see
[05_MPU_Linkage.md](05_MPU_Linkage.md)), the Group's `output_bram` is
written **row by row**, directly as `vecadd_fsm` produces each of the 16
combine-stage-3 results — no separate commit/batching stage needed, since
the adder already only produces one row at a time.

## Where this connects upward

`group_top`'s host preload ports (4 sets, one per MPU-slot) and its final
`grp_out_*` readout port are exactly what `mpe_datapath` drives when it
instantiates 8 unmodified `group_top` instances. See
[09_MPE_Linkage.md](09_MPE_Linkage.md) for how the MPE broadcasts a shared
activation fan-out to every group and streams per-pass weight data from its
own bulk storage into each group's ports.
