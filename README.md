# punto

A GPU-driven point cloud viewer for large clouds. Opens a 32-million-point
HD map in about a second and draws it in under half a millisecond a frame.

Reads PCD, PLY, LAS, LAZ and raw KITTI `.bin`. The name is deliberately not
a format and not a verb: viewing is what exists today, and processing is
where this is going.

![The viewer with a 32M-point HD map open](docs/screenshot.png)

## Why

`pcl_viewer` is a thin shell over VTK, and that explains most of what is
frustrating about it. One VTK actor holds the whole cloud, so every frame
redraws every point — there is no spatial subdivision, no culling and no
level of detail. Vertices go to the GPU as unquantised float32. Parsing runs
on one core through PCL's templated point-type machinery. And the GUI is a
generic wrapper that nobody designed for looking at point clouds.

This is built the other way round:

|                     | `pcl_viewer`                     | this                                        |
| ------------------- | -------------------------------- | ------------------------------------------- |
| Per-frame CPU work  | grows with cloud size            | constant — 3 GL calls                       |
| Culling             | none                             | per-chunk, on the GPU                       |
| Level of detail     | none                             | continuous, zero extra memory               |
| GPU vertex size     | 16–32 B/point                    | 8 B/point                                   |
| Loading             | single-threaded                  | all cores, straight out of an `mmap`        |
| Fields              | fixed point structs              | whatever the file declares, natively typed  |
| Inspection          | —                                | per-field statistics, histograms, picking   |

## Performance

Measured on an RTX 5060 Laptop GPU with a 32,300,421-point, 493 MB binary
PCD (`x y z rgb`), 28 worker threads, page cache warm. Loading and chunking
are CPU work and do not depend on the GPU.

| | |
| --- | --- |
| Open to first frame | **1.0 s** (0.30 s parse · 0.63 s chunk · upload) |
| Frame time @ 1920×1080 | **0.47 ms median, 2109 fps** (p95 1.26 ms) |
| GPU breakdown | cull 0.01 · draw 0.93 · EDL 0.04 ms |
| Points drawn at overview | ~350 k of 32.3 M (1–10%) |
| VRAM | 431 MB of 8151 |
| Host RAM | 1.1 GB peak |

The frame cost tracks screen area, not cloud size: a chunk 500 m away whose
100,000 points land on 40 pixels is drawn with 40 points, because the other
99,960 cannot be seen.

The same scene on the integrated GPU runs at 7.4 ms, 135 fps — still
interactive, because LOD has already capped the work at roughly what the
screen can show. See *Hybrid graphics* below for when that path gets used.

LAZ decompression scales across cores too — 3 M points in 179 ms on 28
threads against 1440 ms on one.

## Building

Needs **GCC 13 or newer**, CMake 3.20+ and OpenGL 4.6. `std::format` is used
throughout, and libstdc++ only shipped it in GCC 13 — Ubuntu 24.04 has it,
22.04 ships GCC 11 and needs a newer one:

```sh
# Ubuntu 22.04 only
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt install g++-13      # then add -DCMAKE_CXX_COMPILER=g++-13 below
```

```sh
sudo apt install build-essential cmake libglfw3-dev libglew-dev libgl1-mesa-dev
sudo apt install liblaszip-dev libxrandr-dev   # optional: LAZ, PRIME detection
```

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

Dear ImGui is vendored, so there is nothing else to fetch. Both optional
dependencies are detected at configure time and the build reports what it
found. Shaders are embedded into the binary, which is a single ~2 MB file.

| CMake option | Default | |
| --- | --- | --- |
| `PUNTO_NATIVE_ARCH` | `ON` | `-march=native`. **Turn off for binaries you distribute** — otherwise they crash on older CPUs. |
| `PUNTO_WERROR` | `OFF` | warnings as errors |
| `PUNTO_STATIC_CXX_RUNTIME` | `OFF` | links libstdc++ and libgcc statically. For binaries built on an older distribution to run on newer ones: the glibc requirement comes from the build host, but libstdc++ comes from the compiler, so a GCC 13 binary built on 22.04 otherwise demands a libstdc++ that 22.04 does not have. |

Two environment variables are read at runtime: `PUNTO_THREADS` caps worker
threads, and `PUNTO_GL_SYNC_DEBUG=1` forces synchronous GL debug output, which
reports driver errors at the call that caused them but serialises the
pipeline. `PUNTO_SHADER_DIR` points the shader loader at a directory so `R`
reloads from disk.

## Using it

```sh
./build/punto-viewer cloud.pcd          # or drag a file onto the window
./build/punto-viewer --color scalar --field intensity --ramp turbo scan.las
./build/punto-viewer --nav top map.laz  # open straight into the map view
```

![Top-down map view coloured by height](docs/screenshot-topdown.png)

Top-down mode with height mapped through Turbo. The histogram under the
ramp is the field's real distribution; dragging its edges sets the colour
range, which is how you pull detail out of a channel whose nominal range is
set by a handful of outliers.

### Controls

| | |
| --- | --- |
| left drag | orbit |
| middle / right drag | pan |
| scroll | zoom |
| **shift + click** | select a point |
| **double click** | select and focus |
| `1` `2` `3` | orbit / fly / top-down |
| `W A S D` `Q E` | fly; `shift` faster, `alt` slower |
| `F` | frame the whole cloud |
| `Tab` | show or hide all panels |
| `C` `E` `L` | cycle colour · toggle EDL · toggle LOD |
| `R` | reload shaders from disk |

`--help` lists the full command line, including `--screenshot`,
`--benchmark` and `--pick`, which drive the viewer without an interactive
session.

## Formats

| | |
| --- | --- |
| **PCD** | `ascii`, `binary`, `binary_compressed` (LZF decoded in-tree) |
| **PLY** | `ascii`, `binary_little_endian`, `binary_big_endian` |
| **LAS** | point data record formats 0–10, LAS 1.0–1.4 |
| **LAZ** | via `laszip`, decompressed in parallel |
| **`.bin`** | headerless KITTI-style float32 tuples |

Files are identified by content, not extension, so a mislabelled one still
opens. Arbitrary fields come through with the exact type the file declared —
a `uint8` ring stays one byte, a `float64` GPS time keeps every bit it can —
and any of them can be used as a colour source.

## Inspecting a cloud

The viewer answers "what is actually in this file" in the Cloud statistics
panel: exact bounds, every field with its type, range, mean, standard
deviation and distribution, per-channel colour histograms, invalid point
counts, and where the load time went.

`shift`-clicking a point shows every field it carries at full precision —
and its **row in the source file**, which is not the same as its index in
memory, because chunking reorders points. Measure mode chains picks and
reports 3D distance along with horizontal and vertical components
separately.

The same report is available from the terminal:

```
$ punto-probe map.pcd

  format        PCD (binary)
  file size     492.9 MB
  points        32,300,421
  colors        yes

  bounds (world)
    min            -4.3965        -226.0963          -7.4408
    max           396.8555          68.3249           9.4218
    size          401.2520         294.4211          16.8626
    diag  497.9672 m
  origin          197.0000         -81.0000          -1.0000

  fields
    name           type      role              min          max         mean    stddev  distribution
    x              float32   position      -4.3965     396.8555     225.2695  104.6336  ▇▇▇▇▇▇▇▇▇▇▇▇
    y              float32   position    -226.0963      68.3249     -87.0321   74.8285  ▆▇▇▇▇▇▇▇▇▇▇▆
    z              float32   position      -7.4408       9.4218       0.1419    1.4754  ▅▅▅▇▇▆▆▆▅▄▃▂
    rgb            float32   color      packed -> colour channels below

  colour channels
    red            uint8     color               0          255       158.29     58.93  ▆▆▆▇▇▆▇▇▇▇▇▆
    green          uint8     color               0          255       155.75     56.22  ▆▆▆▆▆▇▇▇▇▇▇▆
    blue           uint8     color               0          255       145.50     57.93  ▆▆▇▇▆▆▇▇▇▇▆▆

  load  316 ms total   (1634 MB/s)
    map 0.0  header 0.0  decode 233.6  stats 72.4  (ms)
    28 worker threads
```

Add `--chunks` to also build the chunk tree and report its shape, GPU memory
and quantization error.

## How it works

```
 LOAD        mmap → sniff format → parse header → decode on every core
                                                → columnar arrays + statistics
 SPATIALIZE  Morton sort → linear octree → shuffle points within each chunk
 UPLOAD      quantise to 8 B/point → VRAM
 RENDER      compute shader culls chunks and picks each one's detail level
             → one MultiDrawArraysIndirectCount → EDL → ImGui
```

A few decisions that matter more than they look:

**World space is double precision.** HD maps sit at UTM-like coordinates
where float32 resolves to about half a metre. Positions are rebased onto a
double origin at parse time, so float32 then gives sub-millimetre precision
across a multi-kilometre map at 12 bytes a point.

**Level of detail is a prefix.** Points are shuffled within each chunk at
load, so the first N of them are a uniform sample of that chunk for any N.
Drawing less detail means drawing fewer points of the same array — no
duplicated data, no popping, and the prefix is contiguous, so it will stream
from disk just as well when out-of-core lands.

**Depth is reversed.** Near maps to 1 and infinity to 0, against a float
depth buffer. A map spanning 0.1 m to 2 km does not z-fight.

**Splats are sized from measured point spacing**, widened by the square root
of however much LOD thinned the chunk, so a chunk drawn at 1/16 detail gets
4× wider points and stays a surface instead of turning to confetti.

Chunk supply sits behind a `ChunkSource` interface. `RamChunkSource` holds
everything in memory today; a disk-backed streaming implementation can
replace it without the renderer, the LOD selector, the culling pass or the
UI changing.

## Hybrid graphics

On a laptop with both an integrated and a discrete GPU, a program that does
nothing special renders on the integrated one — `pcl_viewer` included. This
viewer selects the discrete GPU itself at startup, with no wrapper script.

The exception is when an external monitor is wired to the discrete GPU while
the X screen renders on the integrated one (reverse PRIME). Frames are
already being copied between GPUs for scanout, and forcing render offload on
top of that sends every frame across the bus twice — enough to wedge a
2560×1440 display, not merely slow it. `--gpu auto` detects this, declines,
and says so. `--gpu nvidia` forces it anyway.

## Tests

```sh
python3 tests/make_fixtures.py     # writes tests/fixtures/
ctest --test-dir build --output-on-failure
```

Around 20,500 assertions. The loader suite writes the same helix in every
encoding and demands they decode identically — with `binary_compressed`
produced by PCL and LAZ by `laszip`, so those paths are checked against the
reference implementations rather than against our own writers. The chunk
suite covers coverage and contiguity, that attributes stay attached to their
points through the reorder, the LOD prefix property, determinism, and
quantization round-trip. Both suites are CPU-only and need no GPU.

Fixtures requiring PCL or laszip are skipped when those are absent.

## Releases

`.github/workflows/build.yml` builds, tests and packages on every push.
Pushing to the default branch replaces a rolling `latest` prerelease;
pushing a `v*` tag cuts a permanent one. Both attach a tarball of the two
binaries with a SHA-256 sum.

Release builds set `-DPUNTO_NATIVE_ARCH=OFF` and run on Ubuntu 22.04. Both
matter: `-march=native` would bake in whatever instructions the build
machine happens to have, and a binary runs on the glibc it was built
against or newer, so building on 22.04 covers 22.04 and 24.04 while
building on 24.04 would not run on 22.04.

A second job builds without `laszip` or `Xrandr` and runs the suite again,
because those sit behind compile-time checks and are otherwise never
exercised — the tests detect the missing library and assert that a `.laz`
file is refused with instructions rather than crashing.

## Status

Viewing is complete and in daily use. In-core, comfortable to about 150 M
points.

Planned, in rough order:

- **Out-of-core streaming**, for clouds past 150 M points. The `ChunkSource`
  interface the renderer draws through already exists for this; a
  disk-backed implementation slots in behind it without the renderer, LOD
  selector, culling pass or UI changing.
- **Post-processing** — filtering, downsampling, normal estimation.
- **Segmentation and object removal** — isolating and deleting things like
  parked cars from a map.

The last two need something the current design does not have: the cloud is
loaded once and treated as immutable all the way down, so editing it means
reworking how chunks, the GPU buffers and the source-row mapping stay in
step when points disappear. Worth knowing before building on this.

## Licence

MIT — see [LICENSE](LICENSE).

Vendored dependencies keep their own: [Dear ImGui](third_party/imgui/LICENSE.txt)
(MIT) and `stb_image_write` (public domain / MIT, stated in the header).
`laszip` is linked optionally at build time and is licensed LGPL-2.1, so it is
a dependency rather than a bundled component.

## Attribution

Developed with assistance from Claude (Anthropic).
