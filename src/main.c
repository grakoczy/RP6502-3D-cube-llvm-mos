#include <rp6502.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include "colors.h"
#include "usb_hid_keys.h"
#include "bitmap_graphics_db.h"

// XRAM locations
#define KEYBOARD_INPUT 0xFF10 // KEYBOARD_BYTES of bitmask data

// 256 bytes HID code max, stored in 32 uint8
#define KEYBOARD_BYTES 32
uint8_t keystates[KEYBOARD_BYTES] = {0};

// keystates[code>>3] gets contents from correct byte in array
// 1 << (code&7) moves a 1 into proper position to mask with byte contents
// final & gives 1 if key is pressed, 0 if not
#define key(code) (keystates[code >> 3] & (1 << (code & 7)))

// Simple 3D to 2D projection (scaled)
uint8_t scale = 64;
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240

// Precompute sine and cosine values for rotation
#define NUM_POINTS 360
int16_t sine_values[NUM_POINTS];
int16_t cosine_values[NUM_POINTS];

// for double buffering
uint16_t buffers[2];
uint8_t active_buffer = 0;

// Constants for fixed-point trigonometry
enum {cA1 = 3370945099UL, cB1 = 2746362156UL, cC1 = 292421UL};
enum {n = 13, p = 32, q = 31, r = 3, a = 12};

// Function to calculate fixed-point sine
// https://www.nullhardware.com/blog/fixed-point-sine-and-cosine-for-embedded-systems/
//
int16_t fpsin(int16_t i) {
    i <<= 1;
    uint8_t c = i < 0; // Set carry for output pos/neg

    if(i == (i | 0x4000)) // Flip input value to corresponding value in range [0..8192]
        i = (1 << 15) - i;
    i = (i & 0x7FFF) >> 1;

    uint32_t y = (cC1 * ((uint32_t)i)) >> n;
    y = cB1 - (((uint32_t)i * y) >> r);
    y = (uint32_t)i * (y >> n);
    y = (uint32_t)i * (y >> n);
    y = cA1 - (y >> (p - q));
    y = (uint32_t)i * (y >> n);
    y = (y + (1UL << (q - a - 1))) >> (q - a); // Rounding

    return c ? -y : y;
}

// Function to calculate fixed-point cosine
#define fpcos(i) fpsin((int16_t)(((uint16_t)(i)) + 8192U))

// Cube vertices in 3D space (8 corners of a cube)
int16_t cube_vertices[8][3] = {
    {-4096, -4096, -4096}, {4096, -4096, -4096}, {4096, 4096, -4096}, {-4096, 4096, -4096},  // Back face
    {-4096, -4096,  4096}, {4096, -4096,  4096}, {4096, 4096,  4096}, {-4096, 4096,  4096}   // Front face
};

void precompute_sin_cos() {
    int16_t angle_step = 32768 / NUM_POINTS; // 32768 is 2^15, representing 2*pi

    for (int i = 0; i < NUM_POINTS; i++) {
        sine_values[i] = fpsin(i * angle_step);
        cosine_values[i] = fpcos(i * angle_step);
    }
}

// end coordinates in 2d space in the middle of canvas
void project_3d_to_2d(int16_t vertex[3], int16_t* x2d, int16_t* y2d) {
    *x2d = (vertex[0] / scale) + SCREEN_WIDTH / 2;
    *y2d = (vertex[1] / scale) + SCREEN_HEIGHT / 2;
}

// Rotate around X axis
void rotate_x(int16_t* y, int16_t* z, int angle) {
    int16_t tmp_y = *y;
    int16_t tmp_z = *z;
    *y = ((long)tmp_y * (long)cosine_values[angle] - (long)tmp_z * (long)sine_values[angle])>> 12;
    *z = ((long)tmp_y * (long)sine_values[angle] + (long)tmp_z * (long)cosine_values[angle])>> 12;
}

// Rotate around Y axis
void rotate_y(int16_t* x, int16_t* z, int angle) {
    int16_t tmp_x = *x;
    int16_t tmp_z = *z;
    *x = ((long)tmp_x * (long)cosine_values[angle] + (long)tmp_z * (long)sine_values[angle])>> 12;
    *z = ((long)tmp_z * (long)cosine_values[angle] - (long)tmp_x * (long)sine_values[angle])>> 12;
}

// Rotate around Z axis
void rotate_z(int16_t* x, int16_t* y, int angle) {
    int16_t tmp_x = *x;
    int16_t tmp_y = *y;
    *x = (tmp_x * (long)cosine_values[angle] - (long)tmp_y * (long)sine_values[angle])>> 12;
    *y = (tmp_x * (long)sine_values[angle] + (long)tmp_y * (long)cosine_values[angle])>> 12;
}

// Draw the cube by connecting the vertices with lines
void draw_cube(int angleX, int angleY, int angleZ, int16_t color, uint8_t mode, uint16_t buffer_data_address) {
    int16_t x2d[8], y2d[8];
    // Rotate and project all vertices
    for (uint8_t i = 0; i < 8; i++) {

        int16_t vertex[3] = {cube_vertices[i][0], cube_vertices[i][1], cube_vertices[i][2]};

        // Apply rotations
        rotate_x(&vertex[1], &vertex[2], angleX);
        rotate_y(&vertex[0], &vertex[2], angleY);
        rotate_z(&vertex[0], &vertex[1], angleZ);

        // Project the 3D vertex to 2D
        project_3d_to_2d(vertex, &x2d[i], &y2d[i]);
        
    }

    // Connect the vertices with lines to draw the cube (front and back faces)
    if (mode == 0) {
    draw_line2buffer(color, x2d[0], y2d[0], x2d[1], y2d[1], buffer_data_address);
    draw_line2buffer(color, x2d[1], y2d[1], x2d[2], y2d[2], buffer_data_address);
    draw_line2buffer(color, x2d[2], y2d[2], x2d[3], y2d[3], buffer_data_address);
    draw_line2buffer(color, x2d[3], y2d[3], x2d[0], y2d[0], buffer_data_address);
    draw_line2buffer(color, x2d[4], y2d[4], x2d[5], y2d[5], buffer_data_address);
    draw_line2buffer(color, x2d[5], y2d[5], x2d[6], y2d[6], buffer_data_address);
    draw_line2buffer(color, x2d[6], y2d[6], x2d[7], y2d[7], buffer_data_address);
    draw_line2buffer(color, x2d[7], y2d[7], x2d[4], y2d[4], buffer_data_address);
    draw_line2buffer(color, x2d[0], y2d[0], x2d[4], y2d[4], buffer_data_address);
    draw_line2buffer(color, x2d[1], y2d[1], x2d[5], y2d[5], buffer_data_address);
    draw_line2buffer(color, x2d[2], y2d[2], x2d[6], y2d[6], buffer_data_address);
    draw_line2buffer(color, x2d[3], y2d[3], x2d[7], y2d[7], buffer_data_address);
    }
    if (mode == 1) {
        draw_pixel2buffer(color, x2d[0], y2d[0], buffer_data_address);
        draw_pixel2buffer(color, x2d[1], y2d[1], buffer_data_address);
        draw_pixel2buffer(color, x2d[2], y2d[2], buffer_data_address);
        draw_pixel2buffer(color, x2d[3], y2d[3], buffer_data_address);
        draw_pixel2buffer(color, x2d[4], y2d[4], buffer_data_address);
        draw_pixel2buffer(color, x2d[5], y2d[5], buffer_data_address);
        draw_pixel2buffer(color, x2d[6], y2d[6], buffer_data_address);
        draw_pixel2buffer(color, x2d[7], y2d[7], buffer_data_address);
        draw_pixel2buffer(color, x2d[0], y2d[0], buffer_data_address);
        draw_pixel2buffer(color, x2d[1], y2d[1], buffer_data_address);
        draw_pixel2buffer(color, x2d[2], y2d[2], buffer_data_address);
        draw_pixel2buffer(color, x2d[3], y2d[3], buffer_data_address);
    }

}

int main() {
    
    bool handled_key = false;
    bool paused = true;
    bool show_buffers_indicators = false;
    uint8_t mode = 0;
    uint8_t i = 0;

    init_bitmap_graphics(0xFF00, buffers[active_buffer], 0, 1, SCREEN_WIDTH, SCREEN_HEIGHT, 1);
    // assign address for each buffer
    buffers[0] = 0x0000;
    buffers[1] = 0x2580;
    erase_buffer(buffers[active_buffer]);
    erase_buffer(buffers[!active_buffer]);

    active_buffer = 0;
    switch_buffer(buffers[active_buffer]);

    // Precompute sine and cosine values
    set_cursor(10, 220);
    draw_string2buffer("Precomputing sine and cosine values...", buffers[active_buffer]);
    precompute_sin_cos();
    erase_buffer(buffers[active_buffer]);

    set_cursor(10, 220);
    draw_string2buffer("Press SPACE to start/stop", buffers[active_buffer]);
    set_cursor(10, 230);
    draw_string2buffer("Press 1 or 2 to change drawing style", buffers[active_buffer]);

    // start angles
    int angleX = 30, angleY = 30, angleZ = 30;
    draw_cube(angleX, angleY, angleZ, WHITE, mode, buffers[active_buffer]);

    while (true) {

        if (!paused) {

           // Update rotation angles
            angleX = (angleX + 1) % NUM_POINTS;
            angleY = (angleY + 1) % NUM_POINTS;
            angleZ = (angleZ + 1) % NUM_POINTS;

            // draw on inactive buffer
            erase_buffer(buffers[!active_buffer]);
            draw_cube(angleX, angleY, angleZ, WHITE, mode, buffers[!active_buffer]);
            if(show_buffers_indicators){
                draw_circle2buffer(WHITE, (active_buffer ? SCREEN_WIDTH - 20 : 20), 20, 8, buffers[!active_buffer]);
                set_cursor((active_buffer ? SCREEN_WIDTH - 22 : 18), 17);
                draw_string2buffer((active_buffer ? "0" : "1"), buffers[!active_buffer]);
            }
            switch_buffer(buffers[!active_buffer]);
            // change active buffer
            active_buffer = !active_buffer;

        }

        xregn( 0, 0, 0, 1, KEYBOARD_INPUT);
        RIA.addr0 = KEYBOARD_INPUT;
        RIA.step0 = 0;

        // fill the keystates bitmask array
        for (uint8_t i = 0; i < KEYBOARD_BYTES; i++) {
            uint8_t j, new_keys;
            RIA.addr0 = KEYBOARD_INPUT + i;
            new_keys = RIA.rw0;

            // check for change in any and all keys
            for (j = 0; j < 8; j++) {
                uint8_t new_key = (new_keys & (1<<j));
                if ((((i<<3)+j)>3) && (new_key != (keystates[i] & (1<<j)))) {
                    // printf( "key %d %s\n", ((i<<3)+j), (new_key ? "pressed" : "released"));
                }
            }

            keystates[i] = new_keys;
        }

        // check for a key down
        if (!(keystates[0] & 1)) {
            if (!handled_key) { // handle only once per single keypress
                // handle the keystrokes
                if (key(KEY_SPACE)) {
                    paused = !paused;
                    if(paused){
                        set_cursor(10, 220);
                        draw_string2buffer("Press SPACE to start", buffers[active_buffer]);
                    }
                }
                if (key(KEY_1)) {
                    mode = 0;
                }
                if (key(KEY_2)) {
                    mode = 1;
                }
                if (key(KEY_3)) {
                    show_buffers_indicators = !show_buffers_indicators;
                }
                if (key(KEY_EQUAL)){
                    if (scale >= 64){
                        scale-=5;
                    }
                }
                if (key(KEY_MINUS)){
                    if (scale <= 200){
                        scale+=5;
                    }
                }
                if (key(KEY_ESC)) {
                    break;
                }
                handled_key = true;
            }
        } else { // no keys down
            handled_key = false;
        }

    }

    return 0;
    
}
