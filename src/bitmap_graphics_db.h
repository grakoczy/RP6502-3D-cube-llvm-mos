// ---------------------------------------------------------------------------
// bitmap_graphics_db.h
//
// This library was written by tonyvr
// and upgraded by WojciechGw for double buffering 
// to simplify bitmap graphics programming
// of the RP6502 picocomputer designed by Rumbledethumps.
//
// This code is an adaptation of the vga_graphics library written by V. Hunter Adams
// from Cornell University, for his excellent RP2040 microcontroller programming course.
//
// https://github.com/vha3/Hunter-Adams-RP2040-Demos/tree/master/VGA_Graphics/VGA_Graphics_Primitives
//
// There doesn't seem to be a copyright or a license associated with his code.
// I don't care what you do with my version either -- have fun!
// ---------------------------------------------------------------------------

#ifndef BITMAP_GRAPHICS_DB_H
#define BITMAP_GRAPHICS_DB_H

#include <stdbool.h>
#include <stdint.h>

#define swap(a, b) { int16_t t = a; a = b; b = t; }

// For writing text
#define TABSPACE 4 // number of spaces for a tab

// For accessing the font library
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))

void init_bitmap_graphics(uint16_t canvas_struct_address,
                          uint16_t canvas_data_address,
                          uint8_t  canvas_plane,
                          uint8_t  canvas_type,
                          uint16_t canvas_width,
                          uint16_t canvas_height,
                          uint8_t  bits_per_pixel);
uint16_t canvas_width(void);
uint16_t canvas_height(void);
uint8_t bits_per_pixel(void);

uint16_t random(uint16_t low_limit, uint16_t high_limit);

void set_cursor(uint16_t x, uint16_t y);
void set_text_multiplier(uint8_t mult);
void set_text_color(uint16_t color); // transparent background
void set_text_colors(uint16_t color, uint16_t background);
void set_text_wrap(bool w);

void switch_buffer(uint16_t buffer_data_address);
void erase_buffer(uint16_t buffer_data_address);
void draw_pixel2buffer(uint16_t color, uint16_t x, uint16_t y, uint16_t buffer_data_address);
void draw_line2buffer(uint16_t color, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t buffer_data_address);
void draw_vline2buffer(uint16_t color, uint16_t x, uint16_t y, uint16_t h, uint16_t buffer_data_address);
void draw_hline2buffer(uint16_t color, uint16_t x, uint16_t y, uint16_t w, uint16_t buffer_data_address);
void draw_rect2buffer(uint16_t color, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t buffer_data_address);
void fill_rect2buffer(uint16_t color, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t buffer_data_address);
void draw_circle2buffer(uint16_t color, uint16_t x0, uint16_t y0, uint16_t r, uint16_t buffer_data_address);
void fill_circle2buffer(uint16_t color, uint16_t x0, uint16_t y0, uint16_t r, uint16_t buffer_data_address);
void draw_rounded_rect2buffer(uint16_t color, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t r, uint16_t buffer_data_address);
void fill_rounded_rect2buffer(uint16_t color, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t r, uint16_t buffer_data_address);
void draw_char2buffer(char chr, uint16_t x, uint16_t y, uint16_t buffer_data_address);
void draw_string2buffer(char * str, uint16_t buffer_data_address);

#endif // BITMAP_GRAPHICS_DB_H
