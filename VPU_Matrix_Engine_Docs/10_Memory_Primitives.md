# Memory Primitives — Shared Storage Wrappers

**Files:**
[`bram_tdp_init.sv`](E:/vivado_projs/memory_and_vpu/memory_and_vpu.srcs/sources_1/new/bram_tdp_init.sv),
[`uram_tdp_init.sv`](E:/vivado_projs/memory_and_vpu/memory_and_vpu.srcs/sources_1/new/uram_tdp_init.sv),
[`activations_bram.sv`](E:/vivado_projs/memory_and_vpu/memory_and_vpu.srcs/sources_1/new/activations_bram.sv),
[`weight_uram.sv`](E:/vivado_projs/memory_and_vpu/memory_and_vpu.srcs/sources_1/new/weight_uram.sv),
[`output_bram.sv`](E:/vivado_projs/memory_and_vpu/memory_and_vpu.srcs/sources_1/new/output_bram.sv)

Every storage element used anywhere in this design — from the smallest
per-MPU activation buffer up to the MPE's global `1x4096` output — is built
from just two generic, vendor-style dual-port memory primitives. This
document is a shared reference: it's linked from every architecture
document ([04_MPU_Architecture.md](04_MPU_Architecture.md),
[06_Group_Architecture.md](06_Group_Architecture.md),
[08_MPE_Architecture.md](08_MPE_Architecture.md)) rather than re-explained
in each one.

## The two generic primitives

### `bram_tdp_init` — Block RAM

```systemverilog
module bram_tdp_init #(
    parameter DATA_WIDTH = 32, ADDR_WIDTH = 12, INIT_FILE = ""
)( ... true dual-port, Port A and Port B ... );

(* ram_style = "block" *) logic [DATA_WIDTH-1:0] ram [0:(1<<ADDR_WIDTH)-1];
```

### `uram_tdp_init` — UltraRAM

```systemverilog
module uram_tdp_init #(
    parameter DATA_WIDTH = 512, ADDR_WIDTH = 12, INIT_FILE = ""
)( ... true dual-port, Port A and Port B ... );

(* ram_style = "ultra" *) logic [DATA_WIDTH-1:0] uram [0:(1<<ADDR_WIDTH)-1];
```

Both are **behaviorally identical** — true dual-port, synchronous,
**read-first** on each port (asserting `en` returns the *old* data one
cycle later, even on the same cycle a write happens to that address), with
optional `$readmemh`-based simulation initialization from `INIT_FILE`.

The only difference is the Xilinx synthesis attribute
(`ram_style = "block"` vs `"ultra"`), which tells the synthesis tool which
physical FPGA resource to map the memory onto:

| | Block RAM (`bram_tdp_init`) | UltraRAM (`uram_tdp_init`) |
|---|---|---|
| Typical size per instance | Smaller (18-36 Kb range) | Much larger (288 Kb per instance) |
| Used for | Activation vectors, all output buffers | Weight matrix tiles (the bulk of total storage) |
| Why | Small, frequently-read vectors | Large, dense tile storage — weight data dominates total memory footprint |

`uram_tdp_init`'s `$readmemh` initialization is wrapped in
`` `ifndef SYNTHESIS`` — simulation-only, since real UltraRAM hardware
can't be pre-loaded with file contents outside of the host-driven DMA
path the design uses at runtime.

### Read-first timing — why every consumer has a `read_valid` shadow register

Because both primitives are read-first with a **one-cycle registered
output**, every consumer that streams multiple words out of one of these
memories (e.g. `memory_controller`'s `FETCH_ACT`/`FETCH_WGT`, or any of the
per-MPU/per-group "loader"/"readout" state machines) has to account for
that one-cycle delay between asserting `en`/`addr` and the corresponding
`dout` actually being valid. The consistent pattern used throughout this
codebase (see [05_MPU_Linkage.md](05_MPU_Linkage.md),
[07_Group_Linkage.md](07_Group_Linkage.md)) is a one-cycle `read_valid`
shadow register (`read_valid <= en`), so downstream logic can gate on
"the data that just arrived is real" rather than assuming a fixed relation
between the issue index and the capture index.

## The three role-specific wrappers

Above the two generic primitives sit three thin, purpose-named wrappers —
each is just a `bram_tdp_init`/`uram_tdp_init` instance with fixed sizing
and one port's write path permanently tied off, matching how that storage
is actually used in this design.

### `activation_bram` (file: `activations_bram.sv`)

```systemverilog
module activation_bram #(
    parameter DATA_WIDTH = 32, DEPTH = 256, INIT_FILE = "activation_data.hex"
)( ... Port A: host read/write, Port B: MPU read-only ... );
```

Wraps `bram_tdp_init`. Stores a `1 x DEPTH` FP32 activation vector.
**Port B's write path is permanently tied off** (`we_b = 1'b0`) — the
compute core (`memory_controller`, or a loader/reloader one level up) only
ever *reads* activation data; it's the host's job (Port A) to write it.

> Note the file/module name mismatch: the file is named
> `activations_bram.sv` (plural) but the module inside it is
> `activation_bram` (singular) — this is just how the file happens to be
> named in the source tree; instantiate the module by its actual name,
> `activation_bram`.

### `weight_uram` (file: `weight_uram.sv`)

```systemverilog
module weight_uram #(
    parameter DATA_WIDTH = 32, NUM_VPUS = 16, DEPTH = 256, INIT_FILE = "weight_data.hex"
)( ... Port A: host read/write, Port B: MPU read-only ... );
```

Wraps `uram_tdp_init`. Stores a `(DEPTH/NUM_VPUS) x DEPTH` weight matrix
tile packed 16-wide (`CG_WIDTH = NUM_VPUS * DATA_WIDTH = 512` bits per
row — 16 FP32 weight values, one per VPU, packed into a single addressable
word). Same read-only-Port-B convention as `activation_bram`.

### `output_bram` (file: `output_bram.sv`)

```systemverilog
module output_bram #(
    parameter DATA_WIDTH = 32, NUM_VPUS = 16, DEPTH = 256, INIT_FILE = ""
)( ... Port A: compute-core write-only, Port B: host read-only ... );
```

Wraps `bram_tdp_init`. Stores `NUM_CGS = DEPTH/NUM_VPUS` rows of 512-bit
(16 x FP32) results. **Direction is flipped relative to the other two**:
Port A here is the *compute core's write* path (results flow in from
whatever level produced them), and Port B is the *host's read-only* path
(or, one level up, the next level's own readout streams data out through
Port B). This same `output_bram` module is reused, unmodified, at every
level of the hierarchy — MPU, Group, and MPE each instantiate their own,
just with different `DEPTH` parameters:

| Level | `output_bram` depth | Total bits stored |
|---|---|---|
| MPU (`mpu_top`) | 16 (`NUM_CGS`) | `16 x 512 = 8192` |
| Group (`group_top`) | 256 (`MPU_DEPTH`) | `256 x 512 = 128K` |
| MPE (`mpe_top`) | 4096 (`NUM_GROUPS x NUM_PASSES x MPU_DEPTH`) | `4096 x 512 = ~2M` |

## Where these are instantiated

| Wrapper | Instantiated in | Count (default config) |
|---|---|---|
| `activation_bram` | `mpu_top` (1), `group_datapath` (4, per-MPU chunk store) | 1 + 4 = 5 per group; `x8` groups = 40 total in one MPE (MPE itself needs none — pure fan-out) |
| `weight_uram` | `mpu_top` (1) | 4 per group (via `group_datapath`'s 4 MPUs) x 8 groups = 32, **plus** 32 raw `uram_tdp_init` bulk lanes at the MPE level (not wrapped in `weight_uram`, used directly — see `mpe_datapath`) |
| `output_bram` | `mpu_top` (1), `group_datapath` (1, Group's own), `mpe_datapath` (1, MPE's global one) | 4 per group + 1 per group + 1 total = `4x8 + 8 + 1 = 41` total in one MPE |

For the full picture of how these fit into each level's architecture, see
[04_MPU_Architecture.md](04_MPU_Architecture.md)'s "Memory layout" section,
[06_Group_Architecture.md](06_Group_Architecture.md)'s "Building blocks"
section, and [08_MPE_Architecture.md](08_MPE_Architecture.md)'s "Memory
sizing" section.
