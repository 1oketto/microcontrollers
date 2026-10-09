#ifndef MONO_CIRCLE_H
#define MONO_CIRCLE_H

#include <stdint.h>
#include "../board_config.h"

#define MONO_WIDTH 640
#define MONO_HEIGHT 480
#define MONO_ROW_BYTES (MONO_WIDTH / 8)
#define MONO_CIRCLE_RADIUS 4

/* Radius-four Adafruit midpoint fill raster, in a 9 x 9 bounding box.
 * Precomputing the shape removes circle stepping and individual pixel calls.
 * Peg rows can occupy up to three framebuffer bytes at unaligned x positions.
 */
static const uint16_t mono_circle_rows[9] = {
    0x038, 0x0fe, 0x0fe, 0x1ff, 0x1ff, 0x1ff, 0x0fe, 0x0fe, 0x038
};

static const uint16_t mono_circle_outline_rows[9] = {
    0x038, 0x0c6, 0x082, 0x101, 0x101, 0x101, 0x082, 0x0c6, 0x038
};

static const uint16_t mono_peg_rows[2 * BOARD_PEG_RADIUS + 1] = {
    0x1f0, 0x3f8, 0x7fc, 0xffe, 0x1fff, 0x1fff, 0x1fff,
    0x1fff, 0x1fff, 0xffe, 0x7fc, 0x3f8, 0x1f0
};

static const uint16_t mono_ball_outline_rows[2 * BOARD_BALL_DRAW_RADIUS + 1] = {
    0x1
};

/* parity 0/1 owns even/odd SCREEN rows; parity 2 stamps all rows. */
static inline void mono_stamp_bitmap(uint8_t *buffer, int x, int y, unsigned parity,
                                     int radius, const uint16_t *rows)
{
    int diameter = 2 * radius + 1;
    if (x < -radius || x >= MONO_WIDTH + radius ||
        y < -radius || y >= MONO_HEIGHT + radius)
        return;
    int left = x - radius;
    if (left >= 0 && left < MONO_WIDTH &&
        (left >> 3) + 2 < MONO_ROW_BYTES &&
        y >= radius && y < MONO_HEIGHT - radius) {
        unsigned shift = (unsigned)left & 7u;
        unsigned first = parity < 2 ? ((unsigned)(y - radius) ^ parity) & 1u : 0;
        unsigned step = parity < 2 ? 2 : 1;
        uint8_t *row = buffer + (y - radius + (int)first) * MONO_ROW_BYTES + (left >> 3);
        for (unsigned dy = first; dy < (unsigned)diameter; dy += step, row += step * MONO_ROW_BYTES) {
            uint32_t bits = (uint32_t)rows[dy] << shift;
            row[0] |= (uint8_t)bits;
            row[1] |= (uint8_t)(bits >> 8);
            row[2] |= (uint8_t)(bits >> 16);
        }
        return;
    }
    /* Edge path clips masks before addressing memory. No write can cross a
     * framebuffer row boundary into memory owned by the other core.
     */
    for (int dy = 0; dy < diameter; ++dy) {
        int py = y - radius + dy;
        if ((unsigned)py >= MONO_HEIGHT || (parity < 2 && ((unsigned)py & 1u) != parity))
            continue;
        int row_left = left;
        uint32_t bits = rows[dy];
        if (row_left < 0) { bits >>= -row_left; row_left = 0; }
        int visible_width = MONO_WIDTH - row_left;
        if (visible_width <= 0) continue;
        if (visible_width < diameter)
            bits &= (1u << visible_width) - 1u;
        bits <<= (row_left & 7);
        unsigned byte = (unsigned)row_left >> 3;
        uint8_t *row = buffer + py * MONO_ROW_BYTES;
        row[byte] |= (uint8_t)bits;
        if (byte + 1 < MONO_ROW_BYTES) row[byte + 1] |= (uint8_t)(bits >> 8);
        if (byte + 2 < MONO_ROW_BYTES) row[byte + 2] |= (uint8_t)(bits >> 16);
    }
}

static inline void mono_stamp_circle(uint8_t *buffer, int x, int y, unsigned parity)
{
    mono_stamp_bitmap(buffer, x, y, parity, MONO_CIRCLE_RADIUS, mono_circle_rows);
}

static inline void mono_stamp_circle_outline(uint8_t *buffer, int x, int y,
                                             unsigned parity)
{
    mono_stamp_bitmap(buffer, x, y, parity, MONO_CIRCLE_RADIUS,
                      mono_circle_outline_rows);
}

static inline void mono_stamp_peg(uint8_t *buffer, int x, int y, unsigned parity)
{
    mono_stamp_bitmap(buffer, x, y, parity, BOARD_PEG_RADIUS, mono_peg_rows);
}

static inline void mono_stamp_ball_outline(uint8_t *buffer, int x, int y,
                                           unsigned parity)
{
    mono_stamp_bitmap(buffer, x, y, parity, BOARD_BALL_DRAW_RADIUS,
                      mono_ball_outline_rows);
}

static inline void mono_reference_outline_pixel(uint8_t *buffer, int x, int y,
                                                unsigned parity)
{
    if ((unsigned)x >= MONO_WIDTH || (unsigned)y >= MONO_HEIGHT ||
        (parity < 2 && ((unsigned)y & 1u) != parity))
        return;
    buffer[y * MONO_ROW_BYTES + (x >> 3)] |= (uint8_t)(1u << (x & 7));
}

static inline void mono_reference_circle_outline_radius(uint8_t *buffer, int x0, int y0,
                                                        unsigned parity, int radius)
{
    int f = 1 - radius, ddx = 1, ddy = -2 * radius;
    int x = 0, y = radius;
    mono_reference_outline_pixel(buffer, x0, y0 + y, parity);
    mono_reference_outline_pixel(buffer, x0, y0 - y, parity);
    mono_reference_outline_pixel(buffer, x0 + y, y0, parity);
    mono_reference_outline_pixel(buffer, x0 - y, y0, parity);
    while (x < y) {
        if (f >= 0) { --y; ddy += 2; f += ddy; }
        ++x; ddx += 2; f += ddx;
        mono_reference_outline_pixel(buffer, x0 + x, y0 + y, parity);
        mono_reference_outline_pixel(buffer, x0 - x, y0 + y, parity);
        mono_reference_outline_pixel(buffer, x0 + x, y0 - y, parity);
        mono_reference_outline_pixel(buffer, x0 - x, y0 - y, parity);
        mono_reference_outline_pixel(buffer, x0 + y, y0 + x, parity);
        mono_reference_outline_pixel(buffer, x0 - y, y0 + x, parity);
        mono_reference_outline_pixel(buffer, x0 + y, y0 - x, parity);
        mono_reference_outline_pixel(buffer, x0 - y, y0 - x, parity);
    }
}

static inline void mono_reference_circle_outline(uint8_t *buffer, int x0, int y0,
                                                 unsigned parity)
{
    mono_reference_circle_outline_radius(buffer, x0, y0, parity, MONO_CIRCLE_RADIUS);
}

static inline void mono_reference_ball_outline(uint8_t *buffer, int x, int y,
                                               unsigned parity)
{
    mono_reference_circle_outline_radius(buffer, x, y, parity, BOARD_BALL_DRAW_RADIUS);
}

/* Deliberately generic midpoint reference for the LAB_GENERIC_CIRCLE ablation.
 * It draws the identical filled shape through per-pixel clipping and masks.
 */
static inline void mono_reference_span(uint8_t *buffer, int x, int y0, int y1, unsigned parity)
{
    if ((unsigned)x >= MONO_WIDTH) return;
    for (int y = y0; y <= y1; ++y)
        if ((unsigned)y < MONO_HEIGHT && (parity >= 2 || ((unsigned)y & 1u) == parity))
            buffer[y * MONO_ROW_BYTES + (x >> 3)] |= (uint8_t)(1u << (x & 7));
}

static inline void mono_reference_circle_radius(uint8_t *buffer, int x0, int y0,
                                                unsigned parity, int radius)
{
    int f = 1 - radius, ddx = 1, ddy = -2 * radius;
    int x = 0, y = radius;
    mono_reference_span(buffer, x0, y0 - y, y0 + y, parity);
    while (x < y) {
        if (f >= 0) { --y; ddy += 2; f += ddy; }
        ++x; ddx += 2; f += ddx;
        mono_reference_span(buffer, x0 + x, y0 - y, y0 + y, parity);
        mono_reference_span(buffer, x0 - x, y0 - y, y0 + y, parity);
        mono_reference_span(buffer, x0 + y, y0 - x, y0 + x, parity);
        mono_reference_span(buffer, x0 - y, y0 - x, y0 + x, parity);
    }
}

static inline void mono_reference_circle(uint8_t *buffer, int x0, int y0, unsigned parity)
{
    mono_reference_circle_radius(buffer, x0, y0, parity, MONO_CIRCLE_RADIUS);
}

static inline void mono_reference_peg(uint8_t *buffer, int x, int y, unsigned parity)
{
    mono_reference_circle_radius(buffer, x, y, parity, BOARD_PEG_RADIUS);
}
#endif
