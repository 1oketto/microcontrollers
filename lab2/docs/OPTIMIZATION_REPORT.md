# Galton-board optimization report

The final program supports a configured pool of 20,000 balls. That is a storage limit, **not a measured claim that 20,000 balls run at 60 frames/s**. No Pico 2 was connected for on-board timing during this work, so the runtime ball-count results remain pending. Memory savings and transfer counts below are calculated directly from the code and can be reported now.

The [runtime ball-capacity chart](../bench/ball_capacity.svg) plots optimization variants on the x-axis and measured ball counts on the y-axis when measurement logs are supplied. The separate [memory-capacity chart](../bench/memory_capacity.svg) shows storage equivalents; its values must not be presented as measured rendering throughput.

## Baselines and attribution

Three starting points matter for interpreting these changes:

| Starting point | What it contains | Appropriate comparison |
| --- | --- | --- |
| `c14c01847744668fc6eef7a8ffd2d05d4b7c3631`, earliest available repository version | A working 16-row, 136-peg Galton-board application with float physics, generic filled-circle rendering, one active physics core, dynamic allocation, 150 MHz CPU clock, and two 4-bit VGA buffers | Earliest reproducible project baseline. It is already adapted from the course template; the unmodified upstream template is not preserved by this commit. |
| `0c219b0ff8fb846c8bd7dc45850cbc629070b628`, local `joanne` HEAD before this task | Q15 physics, 300 MHz clock, outline circles, a histogram, and 24-byte ball records | Committed baseline for the current project |
| Initial uncommitted workspace | Already included compact ball records, multicore physics and rendering, spatial collision filtering, monochrome VGA, cached drawing/text, SRAM background DMA, and startup calibration | These optimizations were retained or extended during this task; they must not all be attributed to this session. The initial diff was backed up to `/private/tmp/lab2-before-partner-pull.patch`. |

`git pull --rebase --autostash` found `joanne` already up to date. Fetching exposed partner commit `bbae48362e28cf94edfb0517b2ef9f04746243d9` on `origin/alyssa`. Its ideas were reviewed and integrated into the existing implementation where applicable, rather than merging its source unchanged. The final source and benchmark results should be saved with the lab submission so that the measurements refer to an identifiable revision.

## Optimization inventory

“Pending” means no on-board before/after measurement was collected. It does not mean zero improvement. Some changes improve correctness, reproducibility, or memory capacity without increasing the CPU-limited ball count.

| # | Change from the earliest available project | Implementation and provenance | Quantified effect now | Additional balls at 60 frames/s |
| --- | --- | --- | --- | --- |
| 1 | Increase CPU clock from 150 to 300 MHz | Already in committed `0c219b0`; retained with explicit regulator setup | Twice the clock frequency; 2× is an ideal compute-bound scaling ceiling, not a demonstrated whole-program speedup | Pending |
| 2 | Replace float state/physics with fixed-point representation | Q15 arithmetic was already in the committed code; final arithmetic preserves intentional signed rounding | Representation and reproducibility change. RP2350 has a hardware FPU, so fixed-point arithmetic is not inherently faster for every operation | Pending |
| 3 | Reduce each ball record | Pre-existing workspace stores position as Q5 `int16_t`, velocity as Q10 `int16_t`, one-byte last-peg index, and one-byte histogram flag; physics uses Q15 intermediates | 20 bytes in earliest baseline, 24 bytes in committed Q15 baseline, 10 bytes now. Savings: 50% or 58.3%, respectively | Memory capacity only; runtime gain pending |
| 4 | Replace repeated allocation with a bounded static pool | Pre-existing workspace replaced growing `realloc` storage with `boids[MAX_BALLS]` | 20,000 records occupy 200,000 bytes; no heap growth or record relocation when adjusting count | Pending |
| 5 | Parallelize physics | Pre-existing workspace and partner both pursue this; final code partitions ball indices between cores, accumulates per-core statistics, and synchronizes before merging/drawing | Two independent physics workers; speedup depends on balance, memory traffic, and serial frame work | Pending |
| 6 | Parallelize drawing without overlapping writes | Pre-existing workspace assigns even screen rows to core 0 and odd rows to core 1; each core draws its rows for all balls | Each framebuffer byte has one rendering owner, including overlapping balls | Pending |
| 7 | Restrict collision candidates spatially | Pre-existing workspace uses peg-row and peg-column bounds instead of always scanning 136 pegs; retained instead of partner's one-child prediction | Fewer candidates per ball. All 136 pegs remain part of the simulation | Pending |
| 8 | Reject noncollisions with squared distance first | Pre-existing workspace applies bounding-box rejection followed by squared-distance rejection | Avoids square root and normalization for candidates outside the circular collision boundary | Pending |
| 9 | Accelerate distance calculation | Pre-existing workspace replaced iterative integer square root with hardware `sqrtf` estimate and integer correction | Corrected root matches the integer squared-distance calculation; no assumed measured cycle saving | Pending |
| 10 | Accelerate collision normal calculation | Pre-existing workspace uses a floating-point reciprocal estimate with integer-product correction | Avoids two general signed 64-bit divisions per accepted collision while retaining the intended integer quotient | Pending |
| 11 | Make constant collision scaling use powers of two | Added in this task; radii/geometry and safe shift details are described below | Constant scaling is cheaper; the variable center distance still requires normalization | Pending |
| 12 | Express reflection doubling as a shift | Added in this task with a nonnegative magnitude to avoid shifting a negative signed value | Optimizing compilers normally already simplify multiplication by 2. Expected independent gain is approximately **0 balls**, not a measured result | No separately established gain |
| 13 | Use power-of-two damping values | Added in this task; signed shifts preserve rounding toward zero | Removes general Q15 multiplication from the selected damping path | Pending |
| 14 | Stamp precomputed filled circles | Pre-existing workspace had specialized outline masks; this task supplies the requested filled `newCircle` path | Precomputed row masks replace repeated circle geometry; row ownership and edge clipping are preserved | Pending |
| 15 | Skip redundant coincident circle stamps | Pre-existing workspace keeps an exact-center cache per rendering core | Identical integer centers can skip drawing. Hash collisions merely cause another draw and do not hide balls | Pending |
| 16 | Reduce VGA color depth from 4 bits to 1 bit | Already present in initial workspace; retained and completed in the driver | Two buffers shrink from 307,200 to 76,800 bytes: **230,400 bytes saved (75%)** | Equivalent to 23,040 ten-byte records; throughput pending |
| 17 | Transfer packed pixels as 32-bit DMA words | Already present in initial monochrome driver; scanout uses 32 pixels per word | 153,600 byte transfers/frame become 9,600 word transfers/frame: 16× fewer transfers and 4× fewer payload bytes | Pending |
| 18 | Precompute the static peg/text scene | Initial workspace pre-rendered pegs to SRAM; this task generates the immutable scene in flash | Removes repeated peg geometry and fixed-label drawing from each frame | Pending |
| 19 | Move static background storage to flash | Added in this task; the third image is an immutable source, not another mutable scanout buffer | **38,400 bytes of SRAM saved**, equivalent to 3,840 ten-byte records | Memory capacity only; runtime difference pending |
| 20 | Restore the background with DMA and gate frame readiness | Initial workspace copied SRAM background alongside physics; this task changes the handoff so restoration completes before the renderer receives the buffer | Four VGA/background DMA channels instead of five; six total including sound. Flash traffic and loss of overlap can offset CPU savings | Pending |
| 21 | Cache formatted statistics | Pre-existing workspace formats strings only when their displayed values change; final static labels live in the background | Avoids repeated `snprintf` for unchanged values; dynamic values still render each frame | Pending |
| 22 | Reduce decorative drawing work | Pre-existing workspace changed size-2 text to size-1 text and filled histogram bars to outlines | Fewer pixel writes with a visible presentation change; not an equivalent-output microbenchmark | Pending |
| 23 | Use per-core random-number state | Pre-existing workspace uses local xorshift generators instead of shared `rand()` | Avoids shared RNG state in physics workers; the random sequence changes | Pending |
| 24 | Keep drawing and scanout ownership consistent | Pre-existing workspace latched a stable drawing pointer; this task additionally submits completed frames explicitly and repeats the old front buffer if a new frame is unfinished | Prevents scanout from displaying or background DMA from clearing an unfinished draw buffer | Correctness improvement; no standalone ball gain |
| 25 | Add repeatable capacity calibration and benchmark variants | Initial workspace had a short startup calibration; this task adds warmup, longer timed trials, named variants, serial records, and chart tooling | Measurement infrastructure, not a claimed simulation speedup | No standalone ball gain |
| 26 | Make compiler optimization explicit | This task uses target-specific `-O3` in place of unscoped `-Ofast` configuration | Makes application optimization explicit and avoids relying on fast-math reassociation for corrected floating-point estimates | Pending; not claimed faster than a verified `-Ofast` build |

The original project already used PIO/DMA VGA scanout, double buffering, a precomputed DAC waveform, and DMA sound playback. Those are inherited design features, not new optimizations in this task. Removing unused demo state or stale driver code is cleanup; no independent ball-count gain is assigned to it. Histogram recording, encoder debounce, statistics resets, and sound-timing fixes are functionality/correctness changes rather than independently measured speed optimizations.

## Collision arithmetic details

The ball radius remains **4 pixels** and the peg radius changes from **6 to 4 pixels**, giving an **8-pixel contact radius**. Geometry is shared through [`board_config.h`](../board_config.h), including the 16 rows, 38-pixel horizontal spacing, and 19-pixel vertical spacing. A compile-time assertion checks that the sum of the radii remains the configured power of two. Smaller pegs alter the board geometry and resulting trajectories; this is an intentional design change, not a mathematically identical rewrite of the previous 6-pixel pegs.

Collision detection still uses the actual `dx`, `dy`, and distance. The corrected normal is retained: dividing by the constant contact radius when a ball has penetrated the collision boundary would produce a nonunit normal. The valid shift optimization applies to the constant separation correction. The ball center is moved to `peg + normal * (8 + 1)`; the integer-normal component scales as `(magnitude << 3) + magnitude`, followed by restoring its sign. Reflection similarly doubles the nonnegative magnitude of `-dot` using a one-bit shift. This avoids undefined signed left shifts of negative components.

The encoder now selects one of six damping factors: **0, 1/16, 1/8, 1/4, 1/2, or 1**. The default changes from the initial workspace's approximately 20% to **25%**. The factor is decoded once per core per frame, and velocity damping uses shifts with truncation toward zero. A general Q15 multiplication variant uses the same factor for a fair arithmetic comparison. The main Q15 multiplication helper also shifts an unsigned product magnitude by 15 before restoring its sign, preserving the previous `/ 32768` rounding behavior. Signed arithmetic right shift alone would round negative values differently.

The position/velocity compaction predates this session. Position precision is 1/32 pixel, velocity precision is 1/1024 pixel per frame, and packing saturates rather than wrapping. This reduces storage and traffic but is another numerical change from unrestricted 32-bit Q15 state. Preserve these precisions between benchmark variants.

## Drawing, flash storage, and DMA details

[`VGA/mono_circle.h`](../VGA/mono_circle.h) contains a radius-four filled midpoint circle in a 9 × 9 bounding box. Its nine `uint16_t` row masks occupy **18 bytes**. `newCircle(x, y)` stamps all rows; `newCircleRows(x, y, parity)` stamps only the even or odd screen rows owned by that core. A shifted row mask is ORed into at most two framebuffer bytes. The edge path clips before accessing memory, including preventing the second byte from spilling into the next row. `referenceCircleRows` draws the same midpoint-filled shape with generic per-pixel writes for the `pixel_stamps` comparison. This fair comparison isolates stamping cost; it does not compare filled balls against the earlier outline appearance.

[`tools/generate_vga_background.py`](../tools/generate_vga_background.py) creates a **38,400-byte, aligned, constant bitmap** from the shared geometry and GLCD font. It contains the peg image and four fixed statistics labels. The default firmware reads this image from memory-mapped flash; there are no per-frame flash writes. The `ram_background` benchmark copies the same image into an optional SRAM source so that the visual workload is matched. Dynamic statistics values are drawn over the freshly restored static labels.

The monochrome PIO program consumes one framebuffer bit per pixel and drives all four existing color output pins together. Black and white each take twelve system clocks per pixel at the configured 300 MHz clock. The driver retains the existing 1-bit scanout while removing stale nibble masks and automatic buffer-pointer tables. Four VGA/background DMA channels replace the initial workspace's five; the two audio DMA channels bring the application total to six.

The background handoff follows **draw-target DMA → background-copy DMA → start DMA**, independently of the active scanout stream. The renderer receives a buffer only after the copy completes, and frame timing includes restoration. At a frame boundary, scanout adopts the back buffer only after `vga_frame_complete()` submits it; otherwise it repeats the existing front buffer. Only a freed buffer is restored. This protects incomplete work during overload, including capacity searches. It does not make missed frames disappear: repeated frames must still count against the benchmark's frame deadline.

The initial workspace overlapped the SRAM background copy with both cores' physics. The final handoff deliberately removes that overlap to keep ownership and completion explicit. Moving the source to flash may also contend with instruction fetches. The `ram_background` comparison is therefore useful even when SRAM savings are already known; no runtime speed improvement is presumed.

## Memory accounting and chart interpretation

At 640 × 480 pixels, one 4-bit image occupies `640 × 480 × 4 / 8 = 153,600` bytes. One 1-bit image occupies `640 × 480 / 8 = 38,400` bytes. Both original and final applications use two mutable scanout/draw buffers. The old source header's “153.6 KB” comment describes one buffer and understates the actual double-buffer allocation.

| Change | SRAM released | Ten-byte record equivalent | Interpretation |
| --- | ---: | ---: | --- |
| Two 4-bit buffers → two 1-bit buffers | 230,400 bytes | 23,040 balls | Additional potential record storage, not a tested frame-rate increase |
| SRAM background → flash background | 38,400 bytes | 3,840 balls | Additional potential record storage, not a tested frame-rate increase |
| 24-byte record → 10-byte record, at 20,000 records | 280,000 bytes | 28,000 record equivalents | Savings at an equal record count; do not add this to a different record-size baseline |
| 20-byte earliest record → 10-byte record, at 20,000 records | 200,000 bytes | 20,000 record equivalents | Alternative baseline to the preceding row, not another cumulative saving |

For another consistent comparison, a fixed **200,000-byte record budget** holds 10,000 earliest 20-byte records, 8,333 committed 24-byte records, or 20,000 final 10-byte records. That is +10,000 or +11,667 stored balls respectively. These figures exclude stacks, code/data placed in SRAM, heap, driver state, and other allocations. They also do not alter the program's configured 20,000-ball cap.

The two framebuffer/background savings can be added **only when the starting configuration contains both two 4-bit buffers and an SRAM background**. The earliest repository baseline has no third background allocation: its framebuffer SRAM is 307,200 bytes, the initial dirty workspace's framebuffer-plus-background SRAM is 115,200 bytes, and the final two monochrome buffers require 76,800 bytes. This avoids counting the background saving twice in an earliest-to-final comparison.

## Partner integration

Partner commit `bbae483` supplied a two-core physics split, separate per-core counters, a core-1 button interrupt, a deterministic next-peg idea, and a regulator call. The initial workspace already had working equivalents for several of these. Their intent is retained through the existing semaphore-based workers, per-core accumulators, and separated encoder/button handling.

The partner branch was not adopted verbatim for concrete reasons:

- Its left/right child arrays have only ten entries, while the board contains 136 pegs. Predicting one child also cannot represent arbitrary upward/sideways rebounds. Spatial candidate filtering retains the performance intent while checking geometrically eligible pegs.
- It expands `HISTOGRAM_BINS` before `PEG_ROWS` has been defined, which prevents that source from compiling as written.
- Bounciness is kept as a 0–100 value but fed directly into Q15 multiplication. A value of 100 then means approximately 0.00305 rather than 100% restitution. The final representation uses consistent damping units.
- The A-falling-edge ISR tests A to choose bounciness direction; B is the appropriate phase input. Existing direction handling and debounced mode selection are retained.
- Its `vreg_set_voltage` call follows the clock increase. Final startup raises the core voltage before requesting 300 MHz.

## Voltage and clock choice

`vreg_set_voltage(VREG_VOLTAGE_1_30)` sets the **core supply to 1.30 V** before raising the clock. The voltage limiter remains enabled. This API controls the nominal 1.10 V core rail, not the board's 3.3 V IO rail; `vreg_disable_voltage_limit()` explicitly enables settings beyond the SDK's protected range. [Official Pico SDK regulator API](https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_vreg/include/hardware/vreg.h).

Cornell's RP2350 gravity example uses the same 1.30 V then 300 MHz sequence. That is relevant precedent for this lab, not a guarantee for every board. Raspberry Pi rates RP2350 at 150 MHz, so 300 MHz remains an overclock and needs hardware validation. [Cornell example](https://people.ece.cornell.edu/land/courses/ece4760/RP2350/gravity/index_gravity.html), [Raspberry Pi specifications](https://www.raspberrypi.com/products/rp2350/).

## Measuring ball-count improvements

The benchmark builds compare one disabled optimization at a time against the final `full` build. These are **ablations of the final implementation**, not a historical series of cumulative commits. Their gains cannot simply be added: optimizations interact and may move the bottleneck from physics to drawing, memory bandwidth, or frame synchronization.

| CMake `LAB_BENCH_VARIANT` | Comparison to `full` |
| --- | --- |
| `full` | Final implementation |
| `full_scan` | Check all pegs instead of spatial candidate filtering |
| `single_core` | Execute work on one core |
| `pixel_stamps` | Draw the same circle coverage using individual pixels instead of packed masks |
| `ram_background` | Keep the static background source in SRAM instead of flash |
| `general_damping` | Use general Q15 damping multiplication instead of shift damping |
| `reference_normal` | Use the reference integer distance/normalization path |
| `no_draw_cache` | Stamp every visible ball, including coincident centers |
| `no_text_cache` | Reformat dynamic text each frame |

Use [the benchmark runner](../bench/run_benchmarks.py) and its command-line help to build variants and process captured serial logs. Firmware emits `BENCH` trial records and `CAPACITY` result records on USB serial and UART. The default trial includes 600 warmup frames followed by 600 timed frames; the counts are configurable through `LAB_CALIBRATION_WARMUP_FRAMES` and `LAB_CALIBRATION_FRAMES`. Warmup addresses the original short calibration's synchronized drops and incomplete traversal of the peg board. A finite passing trial remains evidence for that trial, not a guarantee for indefinitely long execution.

```sh
python3 bench/run_benchmarks.py build --variant all
python3 bench/run_benchmarks.py collect bench/logs/full.txt bench/logs/full_scan.txt
python3 bench/run_benchmarks.py charts
```

The sample log paths above must be replaced with real captured files. The script builds UF2 files and imports logs; it does not flash the board. It writes [`optimization_results.csv`](../bench/optimization_results.csv) plus trial/capacity CSVs and regenerates both SVG charts. No measurement rows are synthesized for variants without logs.

1. Use the same Pico 2, SDK/toolchain, supply, VGA resolution/timing, clock, voltage, ball/peg sizes, gravity, restitution setting, audio configuration, and seed policy for every comparable build. Record the actual build revision and parameters.
2. Connect serial capture before reset and capture complete trial/result records. Preserve raw logs. Build success or a host test is not a timing measurement on the microcontroller.
3. Let calibration search for a passing count meeting the frame budget. Its binary search assumes roughly monotonic timing as ball count increases; dense overlaps and caching can violate this assumption. Check counts near the reported boundary before calling it a maximum. A passing value at the configured maximum means **at least that many balls under the test conditions**; it does not reveal the true maximum above the cap.
4. Repeat each variant at least three times and retain individual results. The supplied chart uses the **minimum observed passing capacity across imported boots**, a conservative summary. Review frame overruns and maximum frame times as well as the selected capacity.
5. Plot the measured capacities with optimization/variant on the x-axis and ball count on the y-axis. Compute each one-at-a-time improvement as `capacity(full) - capacity(with optimization disabled)` only when both capacities are uncensored. The tooling leaves the difference blank when a build reaches its configured pool limit. Preserve negative differences if an optimization is slower on the board.
6. For a truly cumulative chart, produce separate staged revisions and measure each one. Restoring old color depth, record sizes, compiler settings, or physics requires a separately controlled historical build; current ablations do not reconstruct those baselines.

Changing radii, damping, filled/outline rendering, fixed-point precision, or RNG sequences changes the workload or simulation behavior. Such comparisons should be labeled as design changes, or benchmarked at matched settings when isolating arithmetic cost. In particular, changing the radii to have a power-of-two sum does **not** make the actual center distance `sqrt(dx*dx + dy*dy)` constant: substituting the radius for that distance would change the collision normal.

Until board logs are collected, the reportable runtime result for each optimization is **not measured**. Use the separate exact memory chart for the quantified storage benefits, and keep the runtime chart's missing values visibly missing.

## Verification performed

`python3 tests/physics_checks.py` compiles the actual production physics functions and compares the optimized implementation with the all-peg scan, reference integer root/normalization, and general damping paths under host Clang's undefined-behavior sanitizer. The passing checks include 200,000 root/normal comparisons, signed-extreme shift cases, 200,000 randomized multiply/damping cases, and 369,096 updater comparisons covering all 136 peg neighborhoods, signed velocities, respawning, histogram edges, and 120,000 sustained trajectory frames. These validate the arithmetic and spatial-filtering equivalence at the final geometry and damping settings; they do not show that those design settings reproduce the old trajectories or establish RP2350 timing.

VGA electrical timing, physical encoder behavior, audio output, and sustained 300 MHz operation still require the connected board. In particular, host raster tests and ARM builds cannot demonstrate 60-frame/s throughput or overclock stability.
