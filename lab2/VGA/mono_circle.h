#ifndef MONO_CIRCLE_H
#define MONO_CIRCLE_H

#include <stdint.h>

#define MONO_WIDTH 640
#define MONO_HEIGHT 480
#define MONO_ROW_BYTES (MONO_WIDTH / 8)
#define MONO_CIRCLE_RADIUS 4

/* Radius-four Adafruit midpoint fill raster, in a 9 x 9 bounding box.
 * Precomputing the shape removes circle stepping and individual pixel calls.
 * A row occupies at most two framebuffer bytes, even at an unaligned x.
 */
static const uint16_t mono_circle_rows[9] = {
    0x038, 0x0fe, 0x0fe, 0x1ff, 0x1ff, 0x1ff, 0x0fe, 0x0fe, 0x038
};

/* parity 0/1 owns even/odd SCREEN rows; parity 2 stamps all rows. */
static inline void mono_stamp_circle(uint8_t *buffer, int x, int y, unsigned parity)
{
    if (x < -4 || x >= MONO_WIDTH + 4 || y < -4 || y >= MONO_HEIGHT + 4)
        return;
    if (x >= 4 && x < MONO_WIDTH - 4 && y >= 4 && y < MONO_HEIGHT - 4) {
        unsigned shift = (unsigned)(x - 4) & 7u;
        unsigned first = parity < 2 ? ((unsigned)(y - 4) ^ parity) & 1u : 0;
        unsigned step = parity < 2 ? 2 : 1;
        uint8_t *row = buffer + (y - 4 + (int)first) * MONO_ROW_BYTES + ((x - 4) >> 3);
        for (unsigned dy = first; dy < 9; dy += step, row += step * MONO_ROW_BYTES) {
            uint16_t bits = (uint16_t)(mono_circle_rows[dy] << shift);
            row[0] |= (uint8_t)bits;
            row[1] |= (uint8_t)(bits >> 8);
        }
        return;
    }
    /* Edge path clips masks before addressing memory. Neither byte may cross
     * the right edge into the next row (owned by the other core).
     */
    for (int dy = 0; dy < 9; ++dy) {
        int py = y - 4 + dy;
        if ((unsigned)py >= MONO_HEIGHT || (parity < 2 && ((unsigned)py & 1u) != parity))
            continue;
        int left = x - 4;
        uint16_t bits = mono_circle_rows[dy];
        if (left < 0) { bits >>= -left; left = 0; }
        bits = (uint16_t)(bits << (left & 7));
        unsigned byte = (unsigned)left >> 3;
        uint8_t *row = buffer + py * MONO_ROW_BYTES;
        row[byte] |= (uint8_t)bits;
        if (byte + 1 < MONO_ROW_BYTES) row[byte + 1] |= (uint8_t)(bits >> 8);
    }
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

static inline void mono_reference_circle(uint8_t *buffer, int x0, int y0, unsigned parity)
{
    int f = 1 - MONO_CIRCLE_RADIUS, ddx = 1, ddy = -2 * MONO_CIRCLE_RADIUS;
    int x = 0, y = MONO_CIRCLE_RADIUS;
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
#endif
