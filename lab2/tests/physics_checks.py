#!/usr/bin/env python3
"""Compile the production physics helpers on the host and check against references.

No Pico SDK, display, or timing claims are involved. The actual updateboid body is
compiled twice: optimized and exhaustive 136-peg/reference-normal/general-damping.
"""

from pathlib import Path
import os
import re
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "animation.c").read_text()


def function(name):
    match = re.search(r"^static (?:inline )?[^\n]+\b" + name + r"\([^;]*?\)\s*\{", SOURCE, re.M)
    if not match:
        raise RuntimeError(f"Production function {name} was not found")
    depth = 1
    end = match.end()
    while depth:
        depth += (SOURCE[end] == "{") - (SOURCE[end] == "}")
        end += 1
    return SOURCE[match.start():end] + "\n"


def section(start, end):
    return SOURCE[SOURCE.index(start):SOURCE.index(end)]


preamble = """
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include <math.h>
#include <string.h>
#include "board_config.h"
#define BOUNCE_OFF_SHIFT 31u
"""
preamble += section("typedef signed int fix15;", "static fix15 peg_x")
preamble += "static fix15 peg_x[PEG_COUNT], peg_y[PEG_COUNT];\n"
preamble += "#define HISTOGRAM_BINS (PEG_ROWS - 1)\n"
preamble += section("typedef struct\n{\n    int16_t x, y;", "static boid boids[")
preamble += section("typedef struct {\n    uint32_t rng;", "static physics_state physics[")
for name in ("initPegs", "packFixed", "nextRandom", "dropboid", "bouncinessShift", "collisionClearance"):
    preamble += function(name)

body = "".join(function(name) for name in ("distanceFix15", "normalComponent", "dampVelocity", "updateboid"))
production = "#define updateboid updateboid_optimized\n" + body + "#undef updateboid\n"
reference = """
#undef LAB_FULL_PEG_SCAN
#undef LAB_REFERENCE_NORMAL
#undef LAB_GENERAL_DAMPING
#define LAB_FULL_PEG_SCAN 1
#define LAB_REFERENCE_NORMAL 1
#define LAB_GENERAL_DAMPING 1
#define distanceFix15 distanceFix15_reference
#define normalComponent normalComponent_reference
#define dampVelocity dampVelocity_reference
#define updateboid updateboid_reference
""" + body + """
#undef distanceFix15
#undef normalComponent
#undef dampVelocity
#undef updateboid
"""

checks = r"""
static uint32_t rng = 0x71d31453u;
static int sample(int low, int high) {
    return low + (int)(nextRandom(&rng) % (unsigned)(high - low + 1));
}

static void compareBall(boid input, unsigned shift) {
    boid a = input, b = input;
    physics_state sa = {.rng = 19}, sb = {.rng = 19};
    updateboid_optimized(&a, &sa, shift);
    updateboid_reference(&b, &sb, shift);
    assert(a.x == b.x && a.y == b.y && a.vx == b.vx && a.vy == b.vy);
    assert(a.last_peg == b.last_peg && a.histogram_recorded == b.histogram_recorded);
    assert(sa.rng == sb.rng && sa.collisions == sb.collisions && sa.fallen == sb.fallen);
    assert(memcmp(sa.histogram, sb.histogram, sizeof(sa.histogram)) == 0);
}

int main(void) {
    initPegs();
    const unsigned shifts[] = {BOUNCE_OFF_SHIFT, 4, 3, 2, 1, 0};
    const fix15 coefficients[] = {0, 2048, 4096, 8192, 16384, 32768};
    for (unsigned j = 0; j < 6; ++j) assert(bouncinessShift(coefficients[j]) == shifts[j]);
    const int32_t edges[] = {INT32_MIN, INT32_MIN + 1, -65537, -32769, -1,
                            0, 1, 32769, 65537, INT32_MAX};
    for (unsigned i = 0; i < sizeof(edges) / sizeof(edges[0]); ++i)
        for (unsigned shift = 0; shift < 32; ++shift)
            assert(shiftRightTrunc32(edges[i], shift) ==
                   (int64_t)edges[i] / (INT64_C(1) << shift));

    for (unsigned i = 0; i < 200000; ++i) {
        fix15 a = sample(-1048576, 1048576), b = sample(-32768, 32768);
        assert(multfix15(a, b) == (int64_t)a * b / 32768);
        assert(collisionClearance(b) == b * 9);
        for (unsigned j = 0; j < 6; ++j)
            assert(dampVelocity(a, shifts[j]) == (int64_t)a * coefficients[j] / 32768);
        fix15 dx = sample(-262143, 262143), dy = sample(-262143, 262143);
        fix15 dist = distanceFix15(dx, dy);
        assert(dist == distanceFix15_reference(dx, dy));
        if (dist) {
            float scale = 32768.0f / (float)dist;
            assert(normalComponent(dx, dist, scale) == divfix(dx, dist));
            assert(normalComponent(dy, dist, scale) == divfix(dy, dist));
        }
    }

    // Coincident centers and deep penetration in every peg row, including the
    // bottom row beyond the partner's original ten-element lookup table.
    for (int peg = 0; peg < PEG_COUNT; ++peg)
        for (int dy = -9; dy <= 9; ++dy)
            for (int dx = -9; dx <= 9; ++dx) {
                boid ball = {.x = packFixed(peg_x[peg] + int2fix15(dx), 10),
                             .y = packFixed(peg_y[peg] + int2fix15(dy), 10),
                             .vx = 0, .vy = 0, .last_peg = NO_PEG};
                compareBall(ball, shifts[(unsigned)(dx + dy + 18) % 6]);
            }

    // Large signed velocities, screen exit/respawn and histogram boundaries.
    for (unsigned i = 0; i < 200000; ++i) {
        boid ball = {.x = (int16_t)sample(-100 * 32, 740 * 32),
                     .y = (int16_t)sample(-20 * 32, 510 * 32),
                     .vx = (int16_t)sample(INT16_MIN, INT16_MAX),
                     .vy = (int16_t)sample(INT16_MIN, INT16_MAX),
                     .last_peg = (uint8_t)sample(0, UINT8_MAX),
                     .histogram_recorded = (nextRandom(&rng) & 1u) != 0};
        compareBall(ball, shifts[i % 6]);
    }

    // Sustained trajectories accumulate state over many frames. Divergence can
    // reveal a missed later collision that a one-frame random sample misses.
    for (unsigned j = 0; j < 6; ++j) {
        physics_state sa = {.rng = 117}, sb = {.rng = 117};
        boid a, b;
        uint32_t spawn = 3;
        dropboid(&a, &spawn);
        b = a;
        for (unsigned frame = 0; frame < 20000; ++frame) {
            updateboid_optimized(&a, &sa, shifts[j]);
            updateboid_reference(&b, &sb, shifts[j]);
            assert(a.x == b.x && a.y == b.y && a.vx == b.vx && a.vy == b.vy);
            assert(a.last_peg == b.last_peg && a.histogram_recorded == b.histogram_recorded);
            assert(sa.rng == sb.rng && sa.collisions == sb.collisions && sa.fallen == sb.fallen);
            assert(memcmp(sa.histogram, sb.histogram, sizeof(sa.histogram)) == 0);
        }
    }
    puts("PASS: signed shifts/damping; 200000 exact roots/normals; 369096 broad-phase/reference updates");
}
"""

with tempfile.TemporaryDirectory(prefix="galton-physics-") as temporary:
    source = Path(temporary) / "physics_checks.c"
    binary = Path(temporary) / "physics_checks"
    source.write_text(preamble + production + reference + checks)
    subprocess.run([os.environ.get("CC", "cc"), "-std=c11", "-O2", "-g",
                    "-fsanitize=undefined", "-fno-sanitize-recover=undefined",
                    "-I", str(ROOT), str(source), "-lm", "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
