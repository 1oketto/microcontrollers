#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
// Our assembled programs:
// Each gets the name <pio_filename.pio.h>
#include "hsync.pio.h"
#include "vsync.pio.h"
#include "rgb.pio.h"
// Header file
#include "vga16_graphics_v3.h"
#include "mono_circle.h"
// Font files
#include "font_glcd.c"
#include "font_ascii_characters.h"
#include "font_rom_237_brl4.h"
#include "font_Arial_round_16x24.h"
#include "font_Grotesk16x32.h"
#include "font_Tiny8.h"
#include <string.h>

/*
===================================================================================
Some of the source code is derived from Adafruit GFX library:
Software License Agreement (BSD License)

Copyright (c) 2012 Adafruit Industries.  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

- Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
===================================================================================
Many of these fonts were taken from
http://www.rinkydinkelectronics.com/r_fonts.php
The page Font license:
All fonts on this page are considered Public Domain. This means that
you are free to use them as you see fit in any project, commercial or not.
Commercial projects will still need a commercial license for any libraries used.
====================================================================================
- GPIO 16 ---> VGA Hsync
- GPIO 17 ---> VGA Vsync
- GPIO 18 ---> VGA Green lo-bit --> 470 ohm resistor -->  VGA_Green
- GPIO 19 ---> VGA Green hi_bit --> 330 ohm resistor -->  VGA_Green
- GPIO 20 ---> 330 ohm resistor ---> VGA-Blue
- GPIO 21 ---> 330 ohm resistor ---> VGA-Red
- RP2040 GND ---> VGA-GND

New text commands are re-entrant
DrawPixel is faster
*/

// PIO clocks, pin mapping, and 640 x 480 timing use the known-good 300 MHz setup.
#define H_ACTIVE 655
#define V_ACTIVE 479
#define RGB_ACTIVE 639
#define VGA_ROW_BYTES MONO_ROW_BYTES
#define VGA_BUFFER_COUNT (VGA_ROW_BYTES * MONO_HEIGHT)
#define VGA_DMA_WORDS (VGA_BUFFER_COUNT / 4)

// Two writable 1bpp frames occupy 76,800 bytes of SRAM.
unsigned char vga_buffer_0[VGA_BUFFER_COUNT] __attribute__((aligned(4)));
unsigned char vga_buffer_1[VGA_BUFFER_COUNT] __attribute__((aligned(4)));
char *current_draw_buffer;
static uint8_t *display_buffer;
static int rgb_data_chan;
static volatile uint32_t frame_sequence;
static uint32_t released_sequence, released_frame_start_us;
static uint32_t drawing_sequence, drawing_frame_start_us;
enum draw_state { DRAW_READY, DRAWING, DRAW_SUBMITTED };
static volatile enum draw_state renderer_state;

// For drawLine
#define swap(a, b) { short t = a; a = b; b = t; }

// For writing text -- obsolete!
#define tabspace 4 // number of spaces for a tab

// For accessing the font librarys in flash memory
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define pgm_read_short(addr) (*(const unsigned short *)(addr))

// For drawing characters -- obsolete!
unsigned short cursor_y, cursor_x, textsize ;
char textcolor, textbgcolor, wrap;

// Screen width/height
#define _width 640
#define _height 480

static void __not_in_flash_func(vga_frame_irq)(void)
{
    if (!dma_channel_get_irq1_status(rgb_data_chan)) return;
    dma_channel_acknowledge_irq1(rgb_data_chan);
    ++frame_sequence;
    bool released = renderer_state == DRAW_SUBMITTED;
    if (released) {
        uint8_t *old_display = display_buffer;
        display_buffer = (uint8_t *)current_draw_buffer;
        current_draw_buffer = (char *)old_display;
        released_sequence = frame_sequence;
        released_frame_start_us = time_us_32();
        __dmb();
        renderer_state = DRAW_READY;
    }
    // Sync PIOs keep running at their fixed cadence. This IRQ runs in SRAM and
    // has the vertical blanking interval to refill the RGB FIFO.
    dma_channel_set_read_addr(rgb_data_chan, display_buffer, false);
    dma_channel_set_trans_count(rgb_data_chan, VGA_DMA_WORDS, true);
}

void initVGA(void)
{
    PIO pio = pio0, rgb_pio = pio1;
    uint hsync_sm = 0, vsync_sm = 1, rgb_sm = 0;
    uint hsync_offset = pio_add_program(pio, &hsync_program);
    uint vsync_offset = pio_add_program(pio, &vsync_program);
    uint rgb_offset = pio_add_program(rgb_pio, &rgb_program);
    pio_sm_claim(pio, hsync_sm);
    pio_sm_claim(pio, vsync_sm);
    pio_sm_claim(rgb_pio, rgb_sm);
    hsync_program_init(pio, hsync_sm, hsync_offset, HSYNC);
    vsync_program_init(pio, vsync_sm, vsync_offset, VSYNC);
    rgb_program_init(rgb_pio, rgb_sm, rgb_offset, LO_GRN);

    memset(vga_buffer_0, 0, VGA_BUFFER_COUNT);
    memset(vga_buffer_1, 0, VGA_BUFFER_COUNT);
    display_buffer = vga_buffer_0;
    current_draw_buffer = (char *)vga_buffer_1;
    renderer_state = DRAW_READY;
    frame_sequence = released_sequence = 0;

    // One DMA channel continuously feeds pixels to the RGB PIO.
    rgb_data_chan = dma_claim_unused_channel(true);

    dma_channel_config config = dma_channel_get_default_config(rgb_data_chan);
    channel_config_set_transfer_data_size(&config, DMA_SIZE_32);
    channel_config_set_read_increment(&config, true);
    channel_config_set_write_increment(&config, false);
    channel_config_set_dreq(&config, pio_get_dreq(rgb_pio, rgb_sm, true));
    channel_config_set_high_priority(&config, true);
    dma_channel_configure(rgb_data_chan, &config, &rgb_pio->txf[rgb_sm],
                          display_buffer, VGA_DMA_WORDS, false);

    dma_channel_acknowledge_irq1(rgb_data_chan);
    irq_add_shared_handler(DMA_IRQ_1, vga_frame_irq,
                           PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    dma_channel_set_irq1_enabled(rgb_data_chan, true);
    irq_set_enabled(DMA_IRQ_1, true);

    pio_sm_put_blocking(pio, hsync_sm, H_ACTIVE);
    pio_sm_put_blocking(pio, vsync_sm, V_ACTIVE);
    pio_sm_put_blocking(rgb_pio, rgb_sm, RGB_ACTIVE);
    released_frame_start_us = time_us_32();
    dma_start_channel_mask(1u << rgb_data_chan);
    pio_sm_set_enabled(rgb_pio, rgb_sm, true);
    pio_enable_sm_mask_in_sync(pio, (1u << hsync_sm) | (1u << vsync_sm));
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// ============================== Drawing routines  =================================================
/////////////////////////////////////////////////////////////////////////////////////////////////////

// A function for drawing a pixel with a specified color.
// Note that because information is passed to the PIO state machines through
// a DMA channel, we only need to modify the contents of the array and the
// pixels will be automatically updated on the screen.
void drawPixel(short x, short y, char color)
{
    if ((unsigned)x >= 640 || (unsigned)y >= 480) return;
    uint8_t *pixel = (uint8_t *)current_draw_buffer + y * VGA_ROW_BYTES + (x >> 3);
    uint8_t mask = 1u << (x & 7);
    if (color != BLACK) *pixel |= mask;
    else *pixel &= (uint8_t)~mask;
}

// Peg sprites use the configured 6-pixel visual and physical radius.
void newCircle(short x, short y)
{
    mono_stamp_peg((uint8_t *)current_draw_buffer, x, y, 2);
}

void newCircleRows(short x, short y, unsigned parity)
{
    mono_stamp_peg((uint8_t *)current_draw_buffer, x, y, parity & 1u);
}

void referenceCircleRows(short x, short y, unsigned parity)
{
    mono_reference_peg((uint8_t *)current_draw_buffer, x, y, parity & 1u);
}

// Ball sprites are drawn smaller than their physical collision radius.
void drawWhiteBallRows(short x, short y, unsigned parity)
{
    mono_stamp_ball_outline((uint8_t *)current_draw_buffer, x, y, parity & 1u);
}

void referenceWhiteBallRows(short x, short y, unsigned parity)
{
    mono_reference_ball_outline((uint8_t *)current_draw_buffer, x, y, parity & 1u);
}

// Check status of neighbors
int checkNeighbors(short x, short y) {
    return (isAlive(x-1, y-1) + isAlive(x, y-1) + isAlive(x+1, y-1) +
            isAlive(x-1, y) + isAlive(x+1, y) +
            isAlive(x-1, y+1) + isAlive(x, y+1) + isAlive(x+1, y+1));
}

// VGA routine to draw a cell
void drawCell(short x, short y, char color) {

    drawPixel(x<<1, y<<1, color) ;
    drawPixel((x<<1) + 1, (y<<1), color) ;
    drawPixel((x<<1), (y<<1) + 1, color) ;
    drawPixel((x<<1) + 1, (y<<1) + 1, color) ;

}

// Check if alive
int isAlive(short x, short y) {
    return (readPixel(x<<1, y<<1) & 1) ;
}

// vertical line
void drawVLine(short x, short y, short h, char color) {
    for (short i=y; i<(y+h); i++) {
        drawPixel(x, i, color) ;
    }
}

// horizontal line
// note that this function draws using drawPiexl AND
// directly hitting the buffer memory for speed
void drawHLine(int x, int y, int w, char color)
{
    if ((unsigned)y >= 480 || w <= 0 || x >= 640) return;
    if (x < 0) { w += x; x = 0; }
    if (w <= 0) return;
    if (w > 640 - x) w = 640 - x;
    uint8_t *row = (uint8_t *)current_draw_buffer + y * VGA_ROW_BYTES;
    int end = x + w;
    while ((x & 7) && x < end) drawPixel(x++, y, color);
    int whole_bytes = (end - x) >> 3;
    if (whole_bytes) memset(row + (x >> 3), color != BLACK ? 0xff : 0, whole_bytes);
    x += whole_bytes * 8;
    while (x < end) drawPixel(x++, y, color);
}

// general line drawing
// Bresenham's algorithm - thx wikipedia and thx Bruce!
void drawLine(short x0, short y0, short x1, short y1, char color) {
/* Draw a straight line from (x0,y0) to (x1,y1) with given color
 * Parameters:
 *      x0: x-coordinate of starting point of line. The x-coordinate of
 *          the top-left of the screen is 0. It increases to the right.
 *      y0: y-coordinate of starting point of line. The y-coordinate of
 *          the top-left of the screen is 0. It increases to the bottom.
 *      x1: x-coordinate of ending point of line. The x-coordinate of
 *          the top-left of the screen is 0. It increases to the right.
 *      y1: y-coordinate of ending point of line. The y-coordinate of
 *          the top-left of the screen is 0. It increases to the bottom.
 *      color: 3-bit color value for line
 */
      short steep = abs(y1 - y0) > abs(x1 - x0);
      if (steep) {
        swap(x0, y0);
        swap(x1, y1);
      }

      if (x0 > x1) {
        swap(x0, x1);
        swap(y0, y1);
      }

      short dx, dy;
      dx = x1 - x0;
      dy = abs(y1 - y0);

      short err = dx / 2;
      short ystep;

      if (y0 < y1) {
        ystep = 1;
      } else {
        ystep = -1;
      }

      for (; x0<=x1; x0++) {
        if (steep) {
          drawPixel(y0, x0, color);
        } else {
          drawPixel(x0, y0, color);
        }
        err -= dy;
        if (err < 0) {
          y0 += ystep;
          err += dx;
        }
      }
}

// Draw a rectangle
void drawRect(short x, short y, short w, short h, char color) {
/* Draw a rectangle outline with top left vertex (x,y), width w
 * and height h at given color
 * Parameters:
 *      x:  x-coordinate of top-left vertex. The x-coordinate of
 *          the top-left of the screen is 0. It increases to the right.
 *      y:  y-coordinate of top-left vertex. The y-coordinate of
 *          the top-left of the screen is 0. It increases to the bottom.
 *      w:  width of the rectangle
 *      h:  height of the rectangle
 *      color:  16-bit color of the rectangle outline
 * Returns: Nothing
 */
  drawHLine(x, y, w, color);
  drawHLine(x, y+h-1, w, color);
  drawVLine(x, y, h, color);
  drawVLine(x+w-1, y, h, color);
}

void drawCircle(short x0, short y0, short r, char color) {
/* Draw a circle outline with center (x0,y0) and radius r, with given color
 * Parameters:
 *      x0: x-coordinate of center of circle. The top-left of the screen
 *          has x-coordinate 0 and increases to the right
 *      y0: y-coordinate of center of circle. The top-left of the screen
 *          has y-coordinate 0 and increases to the bottom
 *      r:  radius of circle
 *      color: 16-bit color value for the circle. Note that the circle
 *          isn't filled. So, this is the color of the outline of the circle
 * Returns: Nothing
 */
  short f = 1 - r;
  short ddF_x = 1;
  short ddF_y = -2 * r;
  short x = 0;
  short y = r;

  drawPixel(x0  , y0+r, color);
  drawPixel(x0  , y0-r, color);
  drawPixel(x0+r, y0  , color);
  drawPixel(x0-r, y0  , color);

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    drawPixel(x0 + x, y0 + y, color);
    drawPixel(x0 - x, y0 + y, color);
    drawPixel(x0 + x, y0 - y, color);
    drawPixel(x0 - x, y0 - y, color);
    drawPixel(x0 + y, y0 + x, color);
    drawPixel(x0 - y, y0 + x, color);
    drawPixel(x0 + y, y0 - x, color);
    drawPixel(x0 - y, y0 - x, color);
  }
}

void drawCircleHelper( short x0, short y0, short r, unsigned char cornername, char color) {
// Helper function for drawing circles and circular objects
  short f     = 1 - r;
  short ddF_x = 1;
  short ddF_y = -2 * r;
  short x     = 0;
  short y     = r;

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f     += ddF_y;
    }
    x++;
    ddF_x += 2;
    f     += ddF_x;
    if (cornername & 0x4) {
      drawPixel(x0 + x, y0 + y, color);
      drawPixel(x0 + y, y0 + x, color);
    }
    if (cornername & 0x2) {
      drawPixel(x0 + x, y0 - y, color);
      drawPixel(x0 + y, y0 - x, color);
    }
    if (cornername & 0x8) {
      drawPixel(x0 - y, y0 + x, color);
      drawPixel(x0 - x, y0 + y, color);
    }
    if (cornername & 0x1) {
      drawPixel(x0 - y, y0 - x, color);
      drawPixel(x0 - x, y0 - y, color);
    }
  }
}

// ==================================================
// int sqrt from https://github.com/chmike/fpsqrt/blob/master/fpsqrt.c
int32_t sqrt_i32(int32_t v) {
    uint32_t b = 1<<30, q = 0, r = v;
    while (b > r)
        b >>= 2;
    while( b > 0 ) {
        uint32_t t = q + b;
        q >>= 1;
        if( r >= t ) {
            r -= t;
            q += b;
        }
        b >>= 2;
    }
    return q;
}
// =============================
// fill a circle
// uses simple (but fast) sqrt algorithm
void fillCircle(short x0, short y0, short r, char color) {
  // adding r here just makes a better fit
  int r2 = r * r + r;
  if((y0-r < 0) || (y0+r > 479)) return ;
  for(int i=0; i<=r; i++){
    // adding 2 seems to make the circle more symmetric
    int dx = sqrt_i32(r2 - i*i) ;
    // drawHLine(int x, int y, int w, char color)
    drawHLine(x0-dx, y0+(i), 2*dx, color) ;
    drawHLine(x0-dx, y0-(i), 2*dx, color) ;
  }
}
// ==================================================
// depricated
// void fillCircle(short x0, short y0, short r, char color) {
// /* Draw a filled circle with center (x0,y0) and radius r, with given color
//  * Parameters:
//  *      x0: x-coordinate of center of circle. The top-left of the screen
//  *          has x-coordinate 0 and increases to the right
//  *      y0: y-coordinate of center of circle. The top-left of the screen
//  *          has y-coordinate 0 and increases to the bottom
//  *      r:  radius of circle
//  *      color: 16-bit color value for the circle
//  * Returns: Nothing
//  */

//   drawVLine(x0, y0-r, 2*r+1, color);
//   fillCircleHelper(x0, y0, r, 3, 0, color);
// }

void fillCircleHelper(short x0, short y0, short r, unsigned char cornername, short delta, char color) {
// Helper function for drawing filled circles
  short f     = 1 - r;
  short ddF_x = 1;
  short ddF_y = -2 * r;
  short x     = 0;
  short y     = r;

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f     += ddF_y;
    }
    x++;
    ddF_x += 2;
    f     += ddF_x;

    if (cornername & 0x1) {
      drawVLine(x0+x, y0-y, 2*y+1+delta, color);
      drawVLine(x0+y, y0-x, 2*x+1+delta, color);
    }
    if (cornername & 0x2) {
      drawVLine(x0-x, y0-y, 2*y+1+delta, color);
      drawVLine(x0-y, y0-x, 2*x+1+delta, color);
    }
  }
}


// Draw a rounded rectangle
void drawRoundRect(short x, short y, short w, short h, short r, char color) {
/* Draw a rounded rectangle outline with top left vertex (x,y), width w,
 * height h and radius of curvature r at given color
 * Parameters:
 *      x:  x-coordinate of top-left vertex. The x-coordinate of
 *          the top-left of the screen is 0. It increases to the right.
 *      y:  y-coordinate of top-left vertex. The y-coordinate of
 *          the top-left of the screen is 0. It increases to the bottom.
 *      w:  width of the rectangle
 *      h:  height of the rectangle
 *      color:  16-bit color of the rectangle outline
 * Returns: Nothing
 */
  // smarter version
  drawHLine(x+r  , y    , w-2*r, color); // Top
  drawHLine(x+r  , y+h-1, w-2*r, color); // Bottom
  drawVLine(x    , y+r  , h-2*r, color); // Left
  drawVLine(x+w-1, y+r  , h-2*r, color); // Right
  // draw four corners
  drawCircleHelper(x+r    , y+r    , r, 1, color);
  drawCircleHelper(x+w-r-1, y+r    , r, 2, color);
  drawCircleHelper(x+w-r-1, y+h-r-1, r, 4, color);
  drawCircleHelper(x+r    , y+h-r-1, r, 8, color);
}

// =================================================
void fillRoundRect(short x, short y, short w, short h, short r, char color) {
  // smarter version
  fillRect(x, y+r, w, h-2*r, color);
  fillRect(x+r, y, w-2*r, r, color);
  fillRect(x+r, y+h-r, w-2*r, r, color);

  // draw four corners
  fillCircle(x+w-r, y+r, r-1, color);
  fillCircle(x+r  , y+r, r-1, color);
  fillCircle(x+w-r, y+h-r, r-1, color);
  fillCircle(x+r,   y+h-r, r-1, color);
}

// // =================================================
// // Fill a rounded rectangle
// void fillRoundRect(short x, short y, short w, short h, short r, char color) {
//   // smarter version
//   fillRect(x+r, y, w-2*r, h, color);

//   // draw four corners
//   fillCircleHelper(x+w-r-1, y+r, r, 1, h-2*r-1, color);
//   fillCircleHelper(x+r    , y+r, r, 2, h-2*r-1, color);
// }


// fill a rectangle
void fillRect(short x, short y, short w, short h, char color)
{
    int top = y < 0 ? 0 : y;
    int bottom = (int)y + h;
    if (bottom > 480) bottom = 480;
    if (w <= 0 || h <= 0) return;
    for (int row = top; row < bottom; ++row) drawHLine(x, row, w, color);
}

/////////////////////////////////////////////////////////////////////
// copied with minor mods from
// https://ece4760.github.io/Projects/Fall2023/av522_dy245/code.html
/////////////////////////////////////////////////////////////////////
// Draw a filled triangle
// uses top-right rasterization rule to leave no holes between triangles
// (see https://en.wikipedia.org/wiki/Rasterisation)
void fillTri(float x0, float y0, float x1, float y1, float x2, float y2, char color) {
  //
  // sort verts so y0 <= y1 <= y2 (p0 = top, p1 = middle, p2 = bottom)
  if (y1 < y0) {
    swap(x0, x1);
    swap(y0, y1);
  }
  if (y2 < y0) {
    swap(x0, x2);
    swap(y0, y2);
  }
  if (y2 < y1) {
    swap(x1, x2);
    swap(y1, y2);
  }

  // calculate slopes of each edge, in fix15 (don't divide by 0)
  float dxdy_01 = y1 == y0 ? 0 :  (x1 - x0) / (y1 - y0);
  float dxdy_02 = y2 == y0 ? 0 :  (x2 - x0) / (y2 - y0);
  float dxdy_12 = y2 == y1 ? 0 :  (x2 - x1) / (y2 - y1);
  // same for z
  //s15x16 dzdy_01 = y1 == y0 ? 0 : divs15x16(int_to_s15x16(z1 - z0), y1 - y0);
 // s15x16 dzdy_02 = y2 == y0 ? 0 : divs15x16(int_to_s15x16(z2 - z0), y2 - y0);
 // s15x16 dzdy_12 = y2 == y1 ? 0 : divs15x16(int_to_s15x16(z2 - z1), y2 - y1);

  // figure out whether p1 is on the left or right side of the triangle
  bool flat_top = (y0 == y1);
  bool flat_bottom = (y1 == y2);
  bool p1_is_left = flat_top ? (x0 > x1) : (dxdy_02 > dxdy_01);

  // starting at p0, we draw horizontal scanlines (from x_left to x_right, at height y)
  float x_left = x0;
  float x_right = x0;
  float y = y0;

  // similarly, we have interpolators for z coordinates
  //s15x16 z_left = int_to_s15x16(z0) + zeropt5;
  //s15x16 z_right = int_to_s15x16(z0) + zeropt5;

  // x_left and x_right are moved based on slopes of left/right edges
  float dx_left, dx_right;
  float dz_left, dz_right;
  if (p1_is_left) {
    dx_left = dxdy_01;
    dx_right = dxdy_02;
   // dz_left = dzdy_01;
   // dz_right = dzdy_02;
  } else {
    dx_left = dxdy_02;
    dx_right = dxdy_01;
    //dz_left = dzdy_02;
    //dz_right = dzdy_01;
  }

  // macro function to move the scanline down, and update its endpoints
  #define moveScanline() {\
    y += 1 ;\
    x_left += dx_left;\
    x_right += dx_right;\
  }
    //z_left += dz_left;\
   // z_right += dz_right;\
  }

  // draw top half of triangle; skipped for flat top case
  while (y < y1) {
    //drawScanline((short)fix2int15(y), (short)fix2int15(x_left), (short)fix2int15(x_right), z_left, z_right, color);
    drawHLine((int) (x_left), (int) (y), abs((int) (x_right-x_left)), color);

    moveScanline();
  }

  // flat bottom triangles skip the rest
  if (flat_bottom)
    return;

  // reconfigure one end of the scanline so it goes from p1 to p2
  if (p1_is_left) {
    x_left = x1;
    dx_left = dxdy_12;
    //z_left = int2fix15(z1) + zeropt5;
   // dz_left = dzdy_12;
  } else {
    x_right = x1;
    dx_right = dxdy_12;
   // z_right = int2fix15(z1) + zeropt5;
   // dz_right = dzdy_12;
  }

  // draw horizontal line through p1 (middle)
  //drawScanline((short)fix2int15(y), (short)fix2int15(x_left), (short)fix2int15(x_right), z_left, z_right, color);
  drawHLine((int) (x_left), (int) (y), (int) (x_right-x_left), color);
  // draw bottom half of triangle; skipped for flat bottom case
  while (y < y2) {
    moveScanline();
    //drawScanline((short)fix2int15(y), (short)fix2int15(x_left), (short)fix2int15(x_right), z_left, z_right, color);
    drawHLine((int) (x_left), (int) (y), (int) (x_right-x_left), color);
  }
}

// =============================================
// end copied code
// =============================================
// application builds an array of
// short point_list[numlines][2]

// multiline draw
void drawMultiLine(int num_lines,  short point_list[][2], char color){
  for(int i=1; i<num_lines; i++){
    drawLine(point_list[i-1][0], point_list[i-1][1], point_list[i][0], point_list[i][1], color);
  }
}

// === text stuff ===========================
// NOTE!!!
// depricated
// do not USE these, they are NOT re-entrant .
// see below for the new text routines
// the OLDER implementation is here for back compatabioity
// Draw a character
void drawChar(short x, short y, unsigned char c, char color, char bg, unsigned char size) {
    char i, j;
  if((x >= _width)            || // Clip right
     (y >= _height)           || // Clip bottom
     ((x + 6 * size - 1) < 0) || // Clip left
     ((y + 8 * size - 1) < 0))   // Clip top
    return;

  for (i=0; i<6; i++ ) {
    unsigned char line;
    if (i == 5)
      line = 0x0;
    else
      line = pgm_read_byte(font+(c*5)+i);
    for ( j = 0; j<8; j++) {
      if (line & 0x1) {
        if (size == 1) // default size
          drawPixel(x+i, y+j, color);
        else {  // big size
          fillRect(x+(i*size), y+(j*size), size, size, color);
        }
      } else if (bg != color) {
        if (size == 1) // default size
          drawPixel(x+i, y+j, bg);
        else {  // big size
          fillRect(x+i*size, y+j*size, size, size, bg);
        }
      }
      line >>= 1;
    }
  }
}
// depricated
inline void setCursor(short x, short y) {
/* Set cursor for text to be printed
 * Parameters:
 *      x = x-coordinate of top-left of text starting
 *      y = y-coordinate of top-left of text starting
 * Returns: Nothing
 */
  cursor_x = x;
  cursor_y = y;
}
// depricated
inline void setTextSize(unsigned char s) {
/*Set size of text to be displayed
 * Parameters:
 *      s = text size (1 being smallest)
 * Returns: nothing
 */
  textsize = (s > 0) ? s : 1;
}
// depricated
inline void setTextColor(char c) {
  // For 'transparent' background, we'll set the bg
  // to the same as fg instead of using a flag
  textcolor = textbgcolor = c;
}
// depricated
inline void setTextColor2(char c, char b) {
/* Set color of text to be displayed
 * Parameters:
 *      c = 16-bit color of text
 *      b = 16-bit color of text background
 */
  textcolor   = c;
  textbgcolor = b;
}
// depricated
inline void setTextWrap(char w) {
  wrap = w;
}

// depricated
void tft_write(unsigned char c){
  if (c == '\n') {
    cursor_y += textsize*8;
    cursor_x  = 0;
  } else if (c == '\r') {
    // skip em
  } else if (c == '\t'){
      int new_x = cursor_x + tabspace;
      if (new_x < _width){
          cursor_x = new_x;
      }
  } else {
    drawChar(cursor_x, cursor_y, c, textcolor, textbgcolor, textsize);
    cursor_x += textsize*6;
    if (wrap && (cursor_x > (_width - textsize*6))) {
      cursor_y += textsize*8;
      cursor_x = 0;
    }
  }
}
// depricated
inline void writeString(char* str){
/* Print text onto screen
 * Call tft_setCursor(), tft_setTextColor(), tft_setTextSize()
 *  as necessary before printing
 */
    while (*str){
        tft_write(*str++);
    }
}

//=================================================
// added 10/16/2023 brl4
// depricated
inline void setTextColorBig(char color, char background) {
/* Set color of text to be displayed
 * Parameters:
 *      color = 16-bit color of text
 *      b = 16-bit color of text background
 *      background ==-1 means trasnparten background
 */
  textcolor   = color;
  textbgcolor = background;
}
//=================================================
// added 10/11/2023 brl4
// Draw a character
// depricated
void drawCharBig(short x, short y, unsigned char c, char color, char bg) {
  char i, j ;
  unsigned char line;
  for (i=0; i<15; i++ ) {
    line = pgm_read_byte(bigFont+((int)c*16)+i);
    for ( j = 0; j<8; j++) {
      if (line & 0x80) {
        drawPixel(x+j, y+i, color);
      } else if (bg!=color){
        drawPixel(x+j, y+i, bg);
      }
      line <<= 1;
    }
  }
}
// depricated
inline void writeStringBig(char* str){
/* Print text onto screen
 * Call tft_setCursor(), tft_setTextColorBig()
 *  as necessary before printing
 */
    while (*str){
      char c = *str++;
        drawCharBig(cursor_x, cursor_y, c, textcolor, textbgcolor);
        cursor_x += 8 ;
    }
}
// depricated
inline void writeStringBold(char* str){
/* Print text onto screen
 * Call tft_setCursor(), tft_setTextColorBig()
 *  as necessary before printing
 */
   /* Print text onto screen
 * Call tft_setCursor(), tft_setTextColor(), tft_setTextSize()
 *  as necessary before printing
 */
    char temp_bg ;
    temp_bg = textbgcolor;
    while (*str){
        char c = *str++;
        drawChar(cursor_x, cursor_y, c, textcolor, textbgcolor, textsize);
        drawChar(cursor_x+1, cursor_y, c, textcolor, textcolor, textsize);
        cursor_x += 7 * textsize ;
    }
    textbgcolor = temp_bg ;
}

// ===============================================
//Re-entrant text -- >>USE THESE!<<
//
// //GLCD font Adafruit and Hunter
// returns num chars drawn
// Row-major bitmap font renderer. Font bytes are MSB-first; the framebuffer
// is LSB-first. Nonzero colors become white, and negative backgrounds are transparent.
static int drawMonoBitmapText(short x, short y, char *str, char color, char bgcolor,
                              const char *bitmap, int width, int height,
                              int glyph_stride, int first_char, int glyph_count)
{
    if (x < 0 || y < 0 || y + height > 480) return 0;
    int count = 0;
    int row_bytes = (width + 7) / 8;
    while (*str && x + width <= 640) {
        unsigned c = (unsigned char)*str++;
        int glyph = (int)c - first_char;
        if (glyph < 0 || glyph >= glyph_count) glyph = '?' - first_char;
        if (glyph < 0 || glyph >= glyph_count) glyph = 0;
        const uint8_t *data = (const uint8_t *)bitmap + glyph * glyph_stride;
        for (int row = 0; row < height; ++row) {
            for (int col = 0; col < width; ++col) {
                bool set = data[row * row_bytes + (col >> 3)] & (0x80u >> (col & 7));
                if (set) drawPixel(x + col, y + row, color);
                else if ((int8_t)bgcolor >= 0) drawPixel(x + col, y + row, bgcolor);
            }
        }
        x += width;
        ++count;
    }
    return count;
}

int drawTextGLCD(short x, short y, char *str, char color, char bgcolor)
{
    if (x < 0 || y < 0 || y + 8 > 480) return 0;
    int count = 0;
    while (*str && x + 6 <= 640) {
        unsigned c = (unsigned char)*str++;
        for (int col = 0; col < 6; ++col) {
            uint8_t bits = col < 5 ? pgm_read_byte(font + c * 5 + col) : 0;
            for (int row = 0; row < 8; ++row) {
                if (bits & (1u << row)) drawPixel(x + col, y + row, color);
                else if ((int8_t)bgcolor >= 0) drawPixel(x + col, y + row, bgcolor);
            }
        }
        x += 6;
        ++count;
    }
    return count;
}

// ASCII from Designed by: David Perez de la Cruz,and Ed Lau
// see: https://people.ece.cornell.edu/land/courses/ece4760/FinalProjects/s2005/dp93/index.html
//
int drawTextAscii(short x, short y, char *str, char color, char bgcolor)
{
    return drawMonoBitmapText(x, y, str, color, bgcolor, asciifont, 6, 7,
                              7, 0, sizeof(asciifont) / 7);
}

//
// TinyFont from http://www.rinkydinkelectronics.com/r_fonts.php
//
int drawTextTiny8(short x, short y, char *str, char color, char bgcolor)
{
    return drawMonoBitmapText(x, y, str, color, bgcolor, TinyFont, 8, 8,
                              8, 32, sizeof(TinyFont) / 8);
}

//  VGA437 from Code Block 437 IBM font 1982
int drawTextVGA437(short x, short y, char *str, char color, char bgcolor)
{
    return drawMonoBitmapText(x, y, str, color, bgcolor, bigFont, 8, 16,
                              16, 0, sizeof(bigFont) / 16);
}
//
// Arial_round_16x24  bypasses the general drawPixel becuase of the
// packed nature of the draw buffer access
// http://www.rinkydinkelectronics.com/r_fonts.php
int drawTextArial24(short x, short y, char *str, char color, char bgcolor)
{
    return drawMonoBitmapText(x, y, str, color, bgcolor, Arial_round_16x24, 16, 24,
                              48, 32, sizeof(Arial_round_16x24) / 48);
}
// Grotesk16x32
// http://www.rinkydinkelectronics.com/r_fonts.php
int drawTextGrotesk32(short x, short y, char *str, char color, char bgcolor)
{
    return drawMonoBitmapText(x, y, str, color, bgcolor, Grotesk16x32, 16, 32,
                              64, 32, sizeof(Grotesk16x32) / 64);
}
// ======================================================
// depricated
// !!!Dont use!!!!!! slow and is superceeded by Tiny8
void drawBoldTextGLCD(short x, short y, char * str, char textcolor, char textbgcolor, char size){
   char temp_bg ;
    temp_bg = textbgcolor;
    while (*str){
        char c = *str++;
        drawChar(x, y, c, textcolor, textbgcolor, size);
        drawChar(x+1, y, c, textcolor, textcolor, size);
        x += 7 * size ;
    }
    textbgcolor = temp_bg ;
}

// /////////////////////////////////////////////
// Fast erase functions
// NOTE that there is NO RANGE check on these funcitons
// They will clobber memory if x,y falls outside
// the vga display boundaries (0,0) to (640,480)
void clearRect(short x1, short y1, short x2, short y2, short c)
{
    fillRect(x1, y1, x2 - x1, y2 - y1, c);
}
//
void clearLowFrame(short top, short c)
{
    if (top < 0) top = 0;
    if (top >= 480) return;
    memset(current_draw_buffer + VGA_ROW_BYTES * top, c != BLACK ? 0xff : 0,
           VGA_BUFFER_COUNT - VGA_ROW_BYTES * top);
}
// region from y1 to y2 with y1 < y2
void clearRegion(short y1, short y2, short c)
{
    if (y1 < 0) y1 = 0;
    if (y2 > 480) y2 = 480;
    if (y1 >= y2) return;
    memset(current_draw_buffer + VGA_ROW_BYTES * y1, c != BLACK ? 0xff : 0,
           VGA_ROW_BYTES * (y2 - y1));
}

// ======================================
// buffer copy utilities
#ifndef DOUBLE_BUFFER_NONE
  void copy_buffer0to1(void){
    memcpy(vga_buffer_1, vga_buffer_0, VGA_BUFFER_COUNT) ;
  }

  void copy_buffer1to0(void){
    memcpy(vga_buffer_0, vga_buffer_1, VGA_BUFFER_COUNT) ;
  }

  void copy_buffer_to_other(void) {
      if(current_draw_buffer == (char *)vga_buffer_1)
        memcpy(vga_buffer_0, vga_buffer_1, VGA_BUFFER_COUNT) ;
      else
        memcpy(vga_buffer_1, vga_buffer_0, VGA_BUFFER_COUNT) ;
  }
#endif

// ====================================
// driver communication with thread
// Acquire a restored back buffer. Call on the same core as initVGA().
int draw_start_signal(void)
{
    uint32_t irq_state = save_and_disable_interrupts();
    bool ready = renderer_state == DRAW_READY;
    if (ready) {
        __dmb();
        drawing_sequence = released_sequence;
        drawing_frame_start_us = released_frame_start_us;
        renderer_state = DRAWING;
    }
    restore_interrupts(irq_state);
    return ready;
}

// The caller joins the worker core and finishes all UI before publishing.
// The next scanout boundary swaps buffers; no writes are allowed until the
// next draw_start_signal() acquisition. Overload never exposes partial frames.
void vga_frame_complete(void)
{
    uint32_t irq_state = save_and_disable_interrupts();
    if (renderer_state == DRAWING) {
        __dmb();
        renderer_state = DRAW_SUBMITTED;
    }
    restore_interrupts(irq_state);
}

uint32_t draw_frame_start_us(void) { return drawing_frame_start_us; }

int draw_frame_expired(void)
{
    return frame_sequence != drawing_sequence ||
           dma_channel_get_irq1_status(rgb_data_chan);
}

int get_buffer_type(void) { return 1; } // double-buffered; one submission per refresh

//////////////////////////////////////////////////
// read back from VGA
// get the color of a pixel
// but remember there are two buffers!
short readPixel(short x, short y)
{
    if ((unsigned)x >= 640 || (unsigned)y >= 480) return BLACK;
    uint8_t byte = ((uint8_t *)current_draw_buffer)[y * VGA_ROW_BYTES + (x >> 3)];
    return (byte & (1u << (x & 7))) ? WHITE : BLACK;
}

///////////////////////////////////////////////
void crosshair(short x, short y, short c){
  drawPixel(x,y,c);
  drawPixel(x-1,y,c);
  drawPixel(x+1,y,c);
  drawPixel(x,y-1,c);
  drawPixel(x,y+1,c);
  drawPixel(x-2,y,c);
  drawPixel(x+2,y,c);
  drawPixel(x,y-2,c);
  drawPixel(x,y+2,c);
}

///////////////////////////////////////////////
