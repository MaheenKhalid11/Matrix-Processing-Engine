# MPE Linkage — How `mpe_top`, `mpe_datapath`, and `mpe_fsm` Connect

Companion to [08_MPE_Architecture.md](08_MPE_Architecture.md) — read that
first for the "why", this covers the exact handshake and cycle sequencing
at the top of the hierarchy.

**Files:**
[`mpe_top.sv`](E:/vivado_projs/memory_and_vpu/memory_and_vpu.srcs/sources_1/new/mpe_top.sv) (top),
[`mpe_fsm.sv`](E:/vivado_projs/memory_and_vpu/memory_and_vpu.srcs/sources_1/new/mpe_fsm.sv) (control),
[`mpe_datapath.sv`](E:/vivado_projs/memory_and_vpu/memory_and_vpu.srcs/sources_1/new/mpe_datapath.sv)
(datapath).

## The handshake wires between `mpe_fsm` and `mpe_datapath`

| Signal | Direction | Meaning |
|---|---|---|
| `load_pass` | fsm → dp | Pulse: broadcast to all 8 groups' pass reloaders |
| `pass_sel` | fsm → dp | Which pass (0 or 1) to reload weight data for |
| `all_groups_loaded` | dp → fsm | All 8 groups' pass reloaders finished |
| `grp_start` | fsm → dp | Pulse: broadcast `start` to all 8 `group_top`s |
| `all_groups_valid` | dp → fsm | All 8 groups finished compute (AND of 8 `valid`s) |
| `do_readout` | fsm → dp | Pulse: read out ONE group's result (serialized) |
| `group_sel` | fsm → dp | Which group's result to read out this time |
| `readout_done` | dp → fsm | That one group's readout finished |

## `mpe_fsm`'s full state sequence

```
IDLE
 -> [ REQ_LOAD    (pulse load_pass, pass_sel=pass_cnt, broadcast to all
                    8 groups' pass reloaders -- runs in full parallel,
                    no shared resource)
      WAIT_LOAD    (poll all_groups_loaded)
      REQ_COMPUTE  (pulse grp_start, broadcast to all 8 group_tops)
      WAIT_COMPUTE (poll all_groups_valid)
      REQ_READOUT x8 (pulse do_readout, group_sel=group_cnt --
                    the ONE thing that must serialize, since there's
                    only one physical output_bram write port)
      WAIT_READOUT
    ] repeated for pass_cnt = 0 .. NUM_PASSES-1 (2 times)
 -> DONE (pulse valid) -> IDLE
```

This is structurally identical in shape to `group_fsm`'s own sequence (see
[07_Group_Linkage.md](07_Group_Linkage.md)) — load → compute → readout,
repeated per outer iteration (chunks there, passes here) — which is exactly
the "same pattern repeated at every level" design principle called out in
[00_System_Overview.md](00_System_Overview.md).

## The per-group "pass reloader" — streaming bulk storage into `group_top`

Each of the 8 groups gets its own dedicated pass-reloader state machine
inside `mpe_datapath` (in a `generate for` loop):

```
RL_IDLE -> RL_RUN -> RL_IDLE
```

On `load_pass`, all 4 of that group's bulk weight lanes (one per MPU-slot)
begin streaming in parallel, all issuing/capturing in lockstep — lane 0 is
used as the representative "are we still issuing" flag, since all 4 lanes
are always set/cleared together by construction. Each lane's bulk-store
Port B is read at address `{pass_sel, row}` — again an adderless
concatenation, since the bulk store holds both passes back-to-back and
`PASS_BITS + GRP_WGT_CHUNK_ADDR_W` are sized to make this exact — and the
data captured one cycle later is written straight into that group's
`group_top` weight preload ports (`grp_wgt_en/we/addr/din[m]`).

This is a **single-phase** reload (weight data only — activation needs no
per-pass reload at all, since it's the same for every pass). Because of
that, the reloader does *not* need the `L_WGT_START` priming-state fix that
`group_datapath`'s own loader needed (see
[07_Group_Linkage.md](07_Group_Linkage.md)) — that fix was specifically for
a two-phase ACT→WGT chain within one continuous run, which doesn't exist
here; there's only ever the one phase.

All 8 groups' pass reloaders run in **full parallel** with each other too —
each group owns its own 4 bulk-store lanes and its own reloader state
machine, no sharing.

## Activation fan-out — pure wiring, no state machine at all

```systemverilog
for (m = 0; m < NUM_MPUS; m = m + 1) begin : gen_act_fanout
    assign grp_act_en_i[m]   = mpe_act_en[m];
    assign grp_act_we_i[m]   = mpe_act_we[m];
    assign grp_act_addr_i[m] = mpe_act_addr[m];
    assign grp_act_din_i[m]  = mpe_act_din[m];
end
```

Every group's activation preload ports are wired directly (combinationally)
to the MPE's own activation preload ports — the *exact same* signals reach
every group simultaneously. No streaming, no state machine, because there's
no per-group difference to manage: activation only depends on depth, which
is identical across every group and pass (see the architecture doc's
"Activation is loaded once" section for why).

## The shared readout engine — serializing across 8 groups

Structurally almost identical to `group_datapath`'s own shared adder
pattern (see [07_Group_Linkage.md](07_Group_Linkage.md)), just applied to a
BRAM write port instead of an FP32 adder:

```
RO_IDLE -> RO_RUN -> RO_IDLE
```

On `do_readout`, the engine streams `NUM_CGS=16` rows out of *one* group's
own `output_bram` (whichever `group_sel` currently names) into the MPE's
global `output_bram`. Each group's own `grp_out_en_i`/`grp_out_addr_i`
ports are muxed:

```systemverilog
assign grp_out_en_i   = (group_sel == g) ? shared_out_en : 1'b0;
assign grp_out_addr_i = shared_out_addr;
```

— i.e. only the currently-selected group actually receives the readout
engine's `en` pulse; every other group's output port sits idle. This is
safe as a plain combinational mux specifically because `mpe_fsm` only ever
asserts `do_readout` for one `group_sel` value at a time — there's no risk
of two groups being selected simultaneously.

The captured data (`grp_out_dout_arr[group_sel]`, itself a runtime-indexed
mux over all 8 groups' output registers) is written into the MPE's global
`output_bram` at address `{group_sel, pass_sel, row}` — a three-way
concatenation, still adderless since `NUM_GROUPS`, `NUM_PASSES`, and
`NUM_CGS` are all powers of two.

## Top-level (`mpe_top.sv`) wiring

`mpe_top` simply instantiates `mpe_fsm` and `mpe_datapath`, connecting the
handshake table above — no additional logic. `mpe_datapath` in turn
instantiates the 32 bulk weight lanes, 8x `group_top`, 8x pass reloader, the
shared readout engine, and the single global `output_bram`, all as
described above.

## Full top-to-bottom instance count

Putting every level's multiplicity together, here is what one `mpe_top`
actually contains, in terms of the lowest-level compute primitive:

```
8 groups x 4 MPUs x 16 VPUs x 16 dspfp32_pe (cascade)  = 8192 dspfp32_pe (multiply-accumulate)
                                    + 1 shared adder/group x 8 groups x 16 lanes = 128 dspfp32_pe (add)
```

That's roughly **8320 `DSPFP32` hard-macro instances** for one full MPE,
computing a `1x4096 . 4096x4096` FP32 GEMV. This is the number to keep in
mind when checking device DSP-slice budget against a target part.

## Where this connects — the top of the hierarchy

`mpe_top` is the top of this design's scope — its `start`/`busy`/`valid`
handshake and DMA ports are what a host (a CPU, a DMA engine, or a
testbench like `mpu_tb.sv`/`tb_vpu.sv`'s siblings) drives directly. There
is no level above it in this codebase. For the complete top-to-bottom
picture in one place, see [00_System_Overview.md](00_System_Overview.md).
