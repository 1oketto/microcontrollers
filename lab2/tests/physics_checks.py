#!/usr/bin/env python3
"""Compile the production physics helpers on the host and check against references.

No Pico SDK, display, or timing claims are involved. The actual updateboid body is
compiled twice: optimized and exhaustive 136-peg/reference-normal.
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
#define LAB_COLLISION_PROFILE 1
#include "board_config.h"
"""
preamble += section("typedef signed int fix15;", "static fix15 peg_x")
preamble += "static fix15 peg_x[PEG_COUNT], peg_y[PEG_COUNT];\n"
preamble += "#define HISTOGRAM_BINS (PEG_ROWS - 1)\n"
preamble += section("typedef struct\n{\n    int16_t x, y;", "static boid *boids;")
preamble += section("typedef struct {\n    uint32_t rng;", "static physics_state physics[")
for name in ("initPegs", "packFixed", "nextRandom", "dropboid", "collisionClearance"):
    preamble += function(name)

body = "".join(function(name) for name in ("distanceFix15", "normalComponent", "dampVelocity", "updateboid"))
production = "#define updateboid updateboid_optimized\n" + body + "#undef updateboid\n"
reference = """
#undef LAB_FULL_PEG_SCAN
#undef LAB_REFERENCE_NORMAL
#define LAB_FULL_PEG_SCAN 1
#define LAB_REFERENCE_NORMAL 1
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

static void compareBall(boid input, uint8_t input_state, fix15 bounciness) {
    boid a = input, b = input;
    uint8_t state_a = input_state, state_b = input_state;
    physics_state sa = {.rng = 19}, sb = {.rng = 19};
    updateboid_optimized(&a, &state_a, &sa, bounciness);
    updateboid_reference(&b, &state_b, &sb, bounciness);
    assert(a.x == b.x && a.y == b.y && a.vx == b.vx && a.vy == b.vy);
    assert(state_a == state_b);
    assert(sa.rng == sb.rng && sa.collisions == sb.collisions && sa.fallen == sb.fallen);
    assert(memcmp(sa.histogram, sb.histogram, sizeof(sa.histogram)) == 0);
}

int main(void) {
    initPegs();
    const fix15 coefficients[] = {0, 3277, 6554, 9830, 13107, 16384,
                                  19661, 22938, 26214, 29491, 32768};
    physics_state distribution = {.rng = 0x6d2b79f5u};
    uint32_t distribution_spawn_rng = 0x1b873593u;
    for (unsigned sample_index = 0; sample_index < 16384; ++sample_index) {
        boid sample_ball;
        uint8_t sample_state;
        dropboid(&sample_ball, &sample_state, &distribution_spawn_rng);
        for (unsigned frame = 0; frame < 1000 &&
             sample_state != HISTOGRAM_RECORDED; ++frame)
            updateboid_optimized(&sample_ball, &sample_state, &distribution, coefficients[5]);
        assert(sample_state == HISTOGRAM_RECORDED);
    }
    int histogram_total = 0;
    uint64_t mirror_difference = 0;
    for (int bin = 0; bin < HISTOGRAM_BINS; ++bin) {
        histogram_total += distribution.histogram[bin];
        if (bin < HISTOGRAM_BINS / 2)
            mirror_difference += (uint64_t)abs(distribution.histogram[bin] -
                                      distribution.histogram[HISTOGRAM_BINS - 1 - bin]);
        if (bin < HISTOGRAM_BINS / 2)
            assert(distribution.histogram[bin] <= distribution.histogram[bin + 1]);
        else if (bin > HISTOGRAM_BINS / 2)
            assert(distribution.histogram[bin - 1] >= distribution.histogram[bin]);
    }
    assert(histogram_total >= 16384 * 8 / 10);
    assert(mirror_difference * 10 <= (uint64_t)histogram_total);
    assert(distribution.peg_rows_checked > 0);
    assert(distribution.peg_candidates >= distribution.distance_checks);
    assert(distribution.distance_checks > 0);

    boid separated_ball = {
        .x = packFixed(peg_x[0], 10),
        .y = packFixed(peg_y[0] -
                       int2fix15(BOARD_BALL_RADIUS + BOARD_PEG_RADIUS + 1), 10),
        .vx = 0, .vy = 512
    };
    uint8_t separated_ball_state = 0;
    physics_state separated_state = {.rng = 19};
    updateboid_optimized(&separated_ball, &separated_ball_state,
                         &separated_state, coefficients[5]);
    assert(separated_ball_state == NO_PEG);

    for (unsigned i = 0; i < 200000; ++i) {
        fix15 a = sample(-1048576, 1048576), b = sample(-32768, 32768);
        assert(multfix15(a, b) == (int64_t)a * b / 32768);
        assert(collisionClearance(b) ==
               b * (BOARD_BALL_RADIUS + BOARD_PEG_RADIUS + 1));
        for (unsigned j = 0; j < sizeof(coefficients) / sizeof(coefficients[0]); ++j)
            assert(dampVelocity(a, coefficients[j]) ==
                   (int64_t)a * coefficients[j] / 32768);
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
                             .vx = 0, .vy = 0};
                compareBall(ball, NO_PEG, coefficients[(unsigned)(dx + dy + 18) %
                            (sizeof(coefficients) / sizeof(coefficients[0]))]);
            }

    // Large signed velocities, screen exit/respawn and histogram boundaries.
    for (unsigned i = 0; i < 200000; ++i) {
        boid ball = {.x = (int16_t)sample(-100 * 32, 740 * 32),
                     .y = (int16_t)sample(-20 * 32, 510 * 32),
                     .vx = (int16_t)sample(INT16_MIN, INT16_MAX),
                     .vy = (int16_t)sample(INT16_MIN, INT16_MAX)};
        uint8_t ball_state = (uint8_t)sample(0, UINT8_MAX);
        compareBall(ball, ball_state,
                    coefficients[i % (sizeof(coefficients) / sizeof(coefficients[0]))]);
    }

    // Sustained trajectories accumulate state over many frames. Divergence can
    // reveal a missed later collision that a one-frame random sample misses.
    for (unsigned j = 0; j < sizeof(coefficients) / sizeof(coefficients[0]); ++j) {
        physics_state sa = {.rng = 117}, sb = {.rng = 117};
        uint8_t state_a, state_b;
        boid a, b;
        uint32_t spawn = 3;
        dropboid(&a, &state_a, &spawn);
        b = a;
        state_b = state_a;
        for (unsigned frame = 0; frame < 20000; ++frame) {
            updateboid_optimized(&a, &state_a, &sa, coefficients[j]);
            updateboid_reference(&b, &state_b, &sb, coefficients[j]);
            assert(a.x == b.x && a.y == b.y && a.vx == b.vx && a.vy == b.vy);
            assert(state_a == state_b);
            assert(sa.rng == sb.rng && sa.collisions == sb.collisions && sa.fallen == sb.fallen);
            assert(memcmp(sa.histogram, sb.histogram, sizeof(sa.histogram)) == 0);
        }
    }
    puts("PASS: variable damping; 200000 exact roots/normals; broad-phase/reference updates");
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
