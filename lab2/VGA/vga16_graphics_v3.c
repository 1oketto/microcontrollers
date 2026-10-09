#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
// Our assembled programs:
// Each gets the name <pio_filename.pio.h>
#include "hsync.pio.h"
#include "vsync.pio.h"
#include "rgb.pio.h"
// Header file
#include "vga16_graphics_v3.h"
// Font files
#include "font_glcd.c"
#include "font_ascii_characters.h"
#include "font_rom_237_brl4.h"
#include "font_Arial_round_16x24.h"
#include "font_Grotesk16x32.h"
#include "font_Tiny8.h"
#include <string.h>

// VGA timing constants
#define H_ACTIVE   655    // (active + frontporch - 1) - one cycle delay for mov
#define V_ACTIVE   479    // (active - 1)

// CHANGED: In 1-bit mode, the PIO state machine shifts 640 bits (1 bit per pixel)
// across a row instead of 320 4-bit nibbles.
#define RGB_ACTIVE 639

// CHANGED: Added row byte count (640 pixels / 8 bits per byte = 80 bytes per row)
#define VGA_ROW_BYTES 80

// CHANGED: Updated buffer size from 153,600 to 38,400 bytes (640 * 480 / 8 bits per byte)
#define VGA_BUFFER_COUNT 38400

// ===============================
// Define exactly ONE of
// DOUBLE_BUFFER_60, DOUBLE_BUFFER_30, DOUBLE_BUFFER_NONE
#define DOUBLE_BUFFER_60
// ===============================

// Pixel color array that is DMAed to the PIO machines
unsigned char vga_buffer_0[VGA_BUFFER_COUNT];
char * pointer_vga_buffer_0 = &vga_buffer_0[0];

#ifndef DOUBLE_BUFFER_NONE
  unsigned char vga_buffer_1[VGA_BUFFER_COUNT];
  char * pointer_vga_buffer_1 = &vga_buffer_1[0];
#endif

char * current_draw_buffer;
char * pointer_current_display_buffer;

char * draw_buffer[4]  __attribute__ ((aligned (16)));
char * pointer_display_buffer[4] __attribute__ ((aligned (16)));
int    start_flag_array[4] __attribute__ ((aligned (16)));

int start_flag = 0;
int buffer_type;

// CHANGED: Removed TOPMASK and BOTTOMMASK definitions since we no longer work with 4-bit nibbles.

#define swap(a, b) { short t = a; a = b; b = t; }
#define tabspace 4

#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define pgm_read_short(addr) (*(const unsigned short *)(addr))

unsigned short cursor_y, cursor_x, textsize;
char textcolor, textbgcolor, wrap;

#define _width 640
#define _height 480

void initVGA() {
    PIO pio = pio0;

    uint hsync_offset = pio_add_program(pio, &hsync_program);
    uint vsync_offset = pio_add_program(pio, &vsync_program);
    uint rgb_offset = pio_add_program(pio, &rgb_program);

    uint hsync_sm = 0;
    uint vsync_sm = 1;
    uint rgb_sm = 2;
    pio_sm_claim (pio, hsync_sm);
    pio_sm_claim (pio, vsync_sm);
    pio_sm_claim (pio, rgb_sm);

    hsync_program_init(pio, hsync_sm, hsync_offset, HSYNC);
    vsync_program_init(pio, vsync_sm, vsync_offset, VSYNC);
    rgb_program_init(pio, rgb_sm, rgb_offset, LO_GRN);

    current_draw_buffer = vga_buffer_0;    
    
    #ifdef DOUBLE_BUFFER_60
      buffer_type = 1;
      draw_buffer[0] = vga_buffer_1;
      draw_buffer[1] = vga_buffer_0; 
      draw_buffer[2] = vga_buffer_1;
      draw_buffer[3] = vga_buffer_0;
      
      pointer_display_buffer[0] = pointer_vga_buffer_0;
      pointer_display_buffer[1] = pointer_vga_buffer_1;  
      pointer_display_buffer[2] = pointer_vga_buffer_0; 
      pointer_display_buffer[3] = pointer_vga_buffer_1;
      
      start_flag_array[0] = 1;
      start_flag_array[1] = 1;
      start_flag_array[2] = 1;
      start_flag_array[3] = 1;
    #endif

    #ifdef DOUBLE_BUFFER_30
      buffer_type = 2;
      draw_buffer[0] = vga_buffer_0;
      draw_buffer[1] = vga_buffer_0; 
      draw_buffer[2] = vga_buffer_1;
      draw_buffer[3] = vga_buffer_1;
      
      pointer_display_buffer[0] = pointer_vga_buffer_1;
      pointer_display_buffer[1] = pointer_vga_buffer_1;  
      pointer_display_buffer[2] = pointer_vga_buffer_0; 
      pointer_display_buffer[3] = pointer_vga_buffer_0;
      
      start_flag_array[0] = 2;
      start_flag_array[1] = 0;
      start_flag_array[2] = 2;
      start_flag_array[3] = 0;
    #endif

    #ifdef DOUBLE_BUFFER_NONE
      buffer_type = 3;
      draw_buffer[0] = vga_buffer_0;
      draw_buffer[1] = vga_buffer_0; 
      draw_buffer[2] = vga_buffer_0;
      draw_buffer[3] = vga_buffer_0;
      
      pointer_display_buffer[0] = pointer_vga_buffer_0;
      pointer_display_buffer[1] = pointer_vga_buffer_0;  
      pointer_display_buffer[2] = pointer_vga_buffer_0; 
      pointer_display_buffer[3] = pointer_vga_buffer_0;
      
      start_flag_array[0] = 3;
      start_flag_array[1] = 3;
      start_flag_array[2] = 3;
      start_flag_array[3] = 3;
    #endif

    int rgb_data_chan = dma_claim_unused_channel(true);
    int set_disp_chan = dma_claim_unused_channel(true);
    int set_draw_chan = dma_claim_unused_channel(true);
    int set_start_chan = dma_claim_unused_channel(true);

    #define rgb_high_priority false

    // CHANGED: Changed DMA size from DMA_SIZE_8 to DMA_SIZE_32 so DMA transfers
    dma_channel_config c0 = dma_channel_get_default_config(rgb_data_chan);
    channel_config_set_transfer_data_size(&c0, DMA_SIZE_32); // 32 bits (32 monochrome pixels) directly into the PIO TX FIFO at a time.
    channel_config_set_read_increment(&c0, true);
    channel_config_set_write_increment(&c0, false);
    channel_config_set_dreq(&c0, DREQ_PIO0_TX2);
    channel_config_set_chain_to(&c0, set_disp_chan);
    channel_config_set_high_priority(&c0, rgb_high_priority);

    // CHANGED: Updated transfer count from VGA_BUFFER_COUNT (bytes) to VGA_BUFFER_COUNT / 4 (32-bit words).
    dma_channel_configure(
        rgb_data_chan,
        &c0,
        &pio->txf[rgb_sm],
        &vga_buffer_0,
        VGA_BUFFER_COUNT / 4,
        false
    );

    dma_channel_config c1 = dma_channel_get_default_config(set_disp_chan);
    channel_config_set_transfer_data_size(&c1, DMA_SIZE_32);
    channel_config_set_read_increment(&c1, true);
    channel_config_set_write_increment(&c1, false);
    channel_config_set_chain_to(&c1, set_draw_chan);
    channel_config_set_ring(&c1, false, 4);

    dma_channel_configure(
        set_disp_chan,
        &c1,
        &dma_hw->ch[rgb_data_chan].read_addr,
        pointer_display_buffer,
        1,
        false
    );

    c1 = dma_channel_get_default_config(set_draw_chan);
    channel_config_set_transfer_data_size(&c1, DMA_SIZE_32);
    channel_config_set_read_increment(&c1, true);
    channel_config_set_write_increment(&c1, false);
    channel_config_set_chain_to(&c1, set_start_chan);
    channel_config_set_ring(&c1, false, 4);

    dma_channel_configure(
        set_draw_chan,
        &c1,
        &current_draw_buffer,
        draw_buffer,
        1,
        false
    );

    c1 = dma_channel_get_default_config(set_start_chan);
    channel_config_set_transfer_data_size(&c1, DMA_SIZE_32);
    channel_config_set_read_increment(&c1, true);
    channel_config_set_write_increment(&c1, false);
    channel_config_set_chain_to(&c1, rgb_data_chan);
    channel_config_set_ring(&c1, false, 4);

    dma_channel_configure(
        set_start_chan,
        &c1,
        &start_flag,
        start_flag_array,
        1,
        false
    );

    pio_sm_put_blocking(pio, hsync_sm, H_ACTIVE);
    pio_sm_put_blocking(pio, vsync_sm, V_ACTIVE);
    pio_sm_put_blocking(pio, rgb_sm, RGB_ACTIVE);

    pio_enable_sm_mask_in_sync(pio, ((1u << hsync_sm) | (1u << vsync_sm) | (1u << rgb_sm)));
    dma_start_channel_mask((1u << rgb_data_chan));
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// ============================== Drawing routines  =================================================
/////////////////////////////////////////////////////////////////////////////////////////////////////

// CHANGED: Updated drawPixel for 1-bit packing (8 pixels per byte)
void drawPixel(short x, short y, char color) {
    if ((unsigned)x >= 640 || (unsigned)y >= 480) return;
    
    // Find byte index: (y * 80) + (x / 8)
    uint8_t *pixel = (uint8_t *)current_draw_buffer + (y * VGA_ROW_BYTES) + (x >> 3);
    uint8_t mask = 1u << (x & 7); // Bit offset within the byte

    if (color != 0) {
        *pixel |= mask;   // Set bit to 1 (White)
    } else {
        *pixel &= ~mask;  // Clear bit to 0 (Black)
    }
}

int checkNeighbors(short x, short y) {
    return (isAlive(x-1, y-1) + isAlive(x, y-1) + isAlive(x+1, y-1) +
            isAlive(x-1, y) + isAlive(x+1, y) +
            isAlive(x-1, y+1) + isAlive(x, y+1) + isAlive(x+1, y+1));
}

void drawCell(short x, short y, char color) {
    drawPixel(x<<1, y<<1, color);
    drawPixel((x<<1) + 1, (y<<1), color);
    drawPixel((x<<1), (y<<1) + 1, color);
    drawPixel((x<<1) + 1, (y<<1) + 1, color);
}

int isAlive(short x, short y) {
    return (readPixel(x<<1, y<<1) & 1);
}

void drawVLine(short x, short y, short h, char color) {
    for (short i=y; i<(y+h); i++) {
        drawPixel(x, i, color);
    }
}

// CHANGED: Simplified drawHLine to safely work with 1-bit pixels via drawPixel
void drawHLine(int x, int y, int w, char color) {
    if ((unsigned)y >= 480 || w <= 0 || x >= 640) return;
    if (x < 0) { w += x; x = 0; }
    if (w > 640 - x) w = 640 - x;

    for (int i = 0; i < w; i++) {
        drawPixel(x + i, y, color);
    }
}

void drawLine(short x0, short y0, short x1, short y1, char color) {
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

void drawRect(short x, short y, short w, short h, char color) {
    drawHLine(x, y, w, color);
    drawHLine(x, y+h-1, w, color);
    drawVLine(x, y, h, color);
    drawVLine(x+w-1, y, h, color);
}

// Helper: Only writes the pixel if its Y coordinate matches the core's parity
static inline void drawPixelParity(short x, short y, char color, unsigned parity) {
    if (((unsigned)y & 1u) == (parity & 1u)) {
        drawPixel(x, y, color);
    }
}

// Hollow circle renderer filtered by scanline parity (0 = even Y rows, 1 = odd Y rows)
void drawCircleParity(short x0, short y0, short r, char color, unsigned parity) {
    short f = 1 - r;
    short ddF_x = 1;
    short ddF_y = -2 * r;
    short x = 0;
    short y = r;

    drawPixelParity(x0,     y0 + r, color, parity);
    drawPixelParity(x0,     y0 - r, color, parity);
    drawPixelParity(x0 + r, y0,     color, parity);
    drawPixelParity(x0 - r, y0,     color, parity);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        drawPixelParity(x0 + x, y0 + y, color, parity);
        drawPixelParity(x0 - x, y0 + y, color, parity);
        drawPixelParity(x0 + x, y0 - y, color, parity);
        drawPixelParity(x0 - x, y0 - y, color, parity);
        drawPixelParity(x0 + y, y0 + x, color, parity);
        drawPixelParity(x0 - y, y0 + x, color, parity);
        drawPixelParity(x0 + y, y0 - x, color, parity);
        drawPixelParity(x0 - y, y0 - x, color, parity);
    }
}

void drawCircle(short x0, short y0, short r, char color) {
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

void drawCircleHelper(short x0, short y0, short r, unsigned char cornername, char color) {
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

void fillCircle(short x0, short y0, short r, char color) {
    int r2 = r * r + r;
    if((y0-r < 0) || (y0+r > 479)) return;
    for(int i=0; i<=r; i++){
        int dx = sqrt_i32(r2 - i*i);
        drawHLine(x0-dx, y0+(i), 2*dx, color);
        drawHLine(x0-dx, y0-(i), 2*dx, color);
    }  
}

void fillCircleHelper(short x0, short y0, short r, unsigned char cornername, short delta, char color) {
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

void drawRoundRect(short x, short y, short w, short h, short r, char color) {
    drawHLine(x+r  , y    , w-2*r, color);
    drawHLine(x+r  , y+h-1, w-2*r, color);
    drawVLine(x    , y+r  , h-2*r, color);
    drawVLine(x+w-1, y+r  , h-2*r, color);
    
    drawCircleHelper(x+r    , y+r    , r, 1, color);
    drawCircleHelper(x+w-r-1, y+r    , r, 2, color);
    drawCircleHelper(x+w-r-1, y+h-r-1, r, 4, color);
    drawCircleHelper(x+r    , y+h-r-1, r, 8, color);
}

void fillRoundRect(short x, short y, short w, short h, short r, char color) {
    fillRect(x, y+r, w, h-2*r, color);
    fillRect(x+r, y, w-2*r, r, color);
    fillRect(x+r, y+h-r, w-2*r, r, color);

    fillCircle(x+w-r, y+r, r-1, color);
    fillCircle(x+r  , y+r, r-1, color);
    fillCircle(x+w-r, y+h-r, r-1, color);
    fillCircle(x+r,   y+h-r, r-1, color);
}

void fillRect(short x, short y, short w, short h, char color) {
    if((y + h - 1) >= _height) h = _height - y - 1;

    for(int j=y; j<(y+h); j++) {
        drawHLine(x, j, w, color);
    }
}

void fillTri(float x0, float y0, float x1, float y1, float x2, float y2, char color) {
    if (y1 < y0) { swap(x0, x1); swap(y0, y1); }
    if (y2 < y0) { swap(x0, x2); swap(y0, y2); }
    if (y2 < y1) { swap(x1, x2); swap(y1, y2); }

    float dxdy_01 = y1 == y0 ? 0 : (x1 - x0) / (y1 - y0);
    float dxdy_02 = y2 == y0 ? 0 : (x2 - x0) / (y2 - y0);
    float dxdy_12 = y2 == y1 ? 0 : (x2 - x1) / (y2 - y1);

    bool flat_top = (y0 == y1);
    bool flat_bottom = (y1 == y2);
    bool p1_is_left = flat_top ? (x0 > x1) : (dxdy_02 > dxdy_01);

    float x_left = x0;
    float x_right = x0;
    float y = y0;

    float dx_left, dx_right;
    if (p1_is_left) {
        dx_left = dxdy_01;
        dx_right = dxdy_02;
    } else {
        dx_left = dxdy_02;
        dx_right = dxdy_01;
    }

    #define moveScanline() {\
        y += 1;\
        x_left += dx_left;\
        x_right += dx_right;\
    }
  
    while (y < y1) {
        drawHLine((int) (x_left), (int) (y), abs((int) (x_right-x_left)), color);
        moveScanline();
    }

    if (flat_bottom) return;

    if (p1_is_left) {
        x_left = x1;
        dx_left = dxdy_12;
    } else {
        x_right = x1;
        dx_right = dxdy_12;
    }

    drawHLine((int) (x_left), (int) (y), (int) (x_right-x_left), color);
    
    while (y < y2) {
        moveScanline();
        drawHLine((int) (x_left), (int) (y), (int) (x_right-x_left), color);
    }
}

void drawMultiLine(int num_lines, short point_list[][2], char color){
    for(int i=1; i<num_lines; i++){
        drawLine(point_list[i-1][0], point_list[i-1][1], point_list[i][0], point_list[i][1], color);
    }
}

// CHANGED: Cleaned up fast clear functions to use single-bit row offset (y * 80)
void clearRect(short x1, short y1, short x2, short y2, short c) {
    for(int i=y1; i<y2; i++){
        memset(current_draw_buffer + VGA_ROW_BYTES * i + (x1 >> 3), c ? 0xFF : 0x00, (x2 - x1) >> 3);
    }
}

void clearLowFrame(short top, short c) {
    if (top < 0) top = 0;
    if (top >= 480) return;
    memset(current_draw_buffer + VGA_ROW_BYTES * top, c ? 0xFF : 0x00, VGA_BUFFER_COUNT - (VGA_ROW_BYTES * top));
}

void clearRegion(short y1, short y2, short c) {
    if (y1 < 0) y1 = 0;
    if (y2 > 480) y2 = 480;
    if (y1 >= y2) return;
    memset(current_draw_buffer + VGA_ROW_BYTES * y1, c ? 0xFF : 0x00, VGA_ROW_BYTES * (y2 - y1));
}

#ifndef DOUBLE_BUFFER_NONE
  void copy_buffer0to1(void){
    memcpy(vga_buffer_1, vga_buffer_0, VGA_BUFFER_COUNT);
  }

  void copy_buffer1to0(void){
    memcpy(vga_buffer_0, vga_buffer_1, VGA_BUFFER_COUNT);
  }

  void copy_buffer_to_other(void) {
      if(current_draw_buffer == (char *)vga_buffer_1)
        memcpy(vga_buffer_0, vga_buffer_1, VGA_BUFFER_COUNT);
      else
        memcpy(vga_buffer_1, vga_buffer_0, VGA_BUFFER_COUNT);
  }
#endif

int draw_start_signal(void){
  if(start_flag==1) {
    start_flag = 0;
    return 1;
  }
  else {
    return 0;
  }
}

int get_buffer_type(void){
  return buffer_type;
}

// CHANGED: Updated readPixel to check the single-bit state for 1-bit monochrome
short readPixel(short x, short y) {
    if ((unsigned)x >= 640 || (unsigned)y >= 480) return 0;
    
    uint8_t byte = ((uint8_t *)current_draw_buffer)[y * VGA_ROW_BYTES + (x >> 3)];
    return (byte & (1u << (x & 7))) ? 1 : 0;
}
  
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
