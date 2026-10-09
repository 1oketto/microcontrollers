#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../VGA/mono_circle.h"
#include "../board_config.h"
#include "../VGA/vga_background_data.h"

#define BUFFER_BYTES (MONO_HEIGHT * MONO_ROW_BYTES)
#define GUARD 32
static uint8_t actual[BUFFER_BYTES + GUARD * 2];
static uint8_t expected[BUFFER_BYTES + GUARD * 2];
static uint8_t split[BUFFER_BYTES + GUARD * 2];
static uint8_t initial[BUFFER_BYTES + GUARD * 2];

static void check_position(int x, int y)
{
    for (unsigned p = 0; p < 3; ++p) {
        for (unsigned i = 0; i < sizeof(actual); ++i)
            actual[i] = expected[i] = initial[i] = (uint8_t)(i * 37u + 5u);
        mono_stamp_circle(actual + GUARD, x, y, p);
        mono_reference_circle(expected + GUARD, x, y, p);
        if (memcmp(actual, expected, sizeof(actual))) {
            fprintf(stderr, "raster mismatch at (%d, %d), parity %u\n", x, y, p);
            assert(0);
        }
        assert(!memcmp(actual, initial, GUARD));
        assert(!memcmp(actual + GUARD + BUFFER_BYTES,
                       initial + GUARD + BUFFER_BYTES, GUARD));
        if (p < 2)
            for (int row = (int)(p ^ 1u); row < MONO_HEIGHT; row += 2)
                assert(!memcmp(actual + GUARD + row * MONO_ROW_BYTES,
                               initial + GUARD + row * MONO_ROW_BYTES,
                               MONO_ROW_BYTES));
    }
    memset(actual, 0, sizeof(actual));
    memset(split, 0, sizeof(split));
    mono_stamp_circle(actual + GUARD, x, y, 2);
    mono_stamp_circle(split + GUARD, x, y, 0);
    mono_stamp_circle(split + GUARD, x, y, 1);
    assert(!memcmp(actual, split, sizeof(actual)));
}

int main(void)
{
    // Every horizontal byte alignment at the top/bottom and both clip margins.
    const int ys[] = {-12,-5,-4,-3,-2,-1,0,1,2,3,4,5,127,128,474,475,476,477,478,479,480,481,482,483,484,492};
    for (int x = -5; x <= MONO_WIDTH + 5; ++x)
        for (unsigned y = 0; y < sizeof(ys)/sizeof(ys[0]); ++y)
            check_position(x, ys[y]);
    check_position(-32768, -32768);
    check_position(32767, 32767);
    // All pre-rendered pegs match the production stamp at their exact coordinates.
    memset(expected, 0, sizeof(expected));
    for (int row = 0; row < BOARD_PEG_ROWS; ++row)
        for (int col = 0; col <= row; ++col)
            mono_stamp_circle(expected + GUARD, BOARD_CENTER_X + (2 * col - row) * BOARD_PEG_HORIZONTAL_SPACING / 2,
                              BOARD_TOP_Y + row * BOARD_PEG_VERTICAL_SPACING, 2);
    assert(sizeof(vga_background_flash) == BUFFER_BYTES);
    assert(!memcmp(expected + GUARD + 50 * MONO_ROW_BYTES,
                   vga_background_flash + 50 * MONO_ROW_BYTES,
                   BUFFER_BYTES - 50 * MONO_ROW_BYTES));
    puts("mono circles: clipping, exact reference raster, byte alignment, row ownership, and flash pegs passed");
    return 0;
}
