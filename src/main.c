// Simple rotating 3D cube
//
// Original code by Grzegorz Rakoczy
// fun stuff with coordinates caching and double buffering 
// added by WojciechGw
//

#include <rp6502.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include "colors.h"
#include "usb_hid_keys.h"
#include "bitmap_graphics_db.h"

// #define HIRES
#define NUM_MODES 4

// Screen related
//
#ifdef HIRES
    #define SCALE 48
    #define SCREEN_WIDTH 640
    #define SCREEN_HEIGHT 360
    #define OFFSET_X 60
    #define OFFSET_Y 0
    #define NUM_POINTS 270
#else
    #define SCALE 96
    #define SCREEN_WIDTH 320
    #define SCREEN_HEIGHT 240
    #define OFFSET_X 30
    #define OFFSET_Y 0
    #define NUM_POINTS 270
#endif

// for double buffering
uint16_t buffers[2];
uint8_t active_buffer = 0;
int16_t distance = 1000; // for perspective calculations
char *buf[] = {"                                                                  "};

bool paused = true;
bool show_indicators = false;
bool show_vertex_coordinates = false;
bool first_run = true;

// Keyboard related
//
// XRAM locations
#define KEYBOARD_INPUT 0xFF10 // KEYBOARD_BYTES of bitmask data
// 256 bytes HID code max, stored in 32 uint8
#define KEYBOARD_BYTES 32
uint8_t keystates[KEYBOARD_BYTES] = {0};
// keystates[code>>3] gets contents from correct byte in array
// 1 << (code&7) moves a 1 into proper position to mask with byte contents
// final & gives 1 if key is pressed, 0 if not
#define key(code) (keystates[code >> 3] & (1 << (code & 7)))

// Fixed-point arithmetics
//
// Precompute sine and cosine values for rotation
int16_t sine_values[NUM_POINTS];
int16_t cosine_values[NUM_POINTS];
int16_t cube_vertices_precalculated[NUM_POINTS][8][3];
uint32_t cube_position = 0;
bool calculations_completed = false;

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

void WaitForAnyKey(){

    bool handled_key = true;
    while (1){
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
                //if ((((i<<3)+j)>3) && (new_key != (keystates[i] & (1<<j)))) {
                    // printf( "key %d %s\n", ((i<<3)+j), (new_key ? "pressed" : "released"));
                //}
            }
            keystates[i] = new_keys;
        }
        // check for a key down
        if (!(keystates[0] & 1)) {
            if (!handled_key) { // handle only once per single keypress
                // handle the keystrokes
                handled_key = true;
                break;
            }
        } else { // no keys down
            handled_key = false;
        }
    }
}

// Recalculate coordinates for perspective view
//
/*
    int16_t addPerspective(int32_t coordinate, int32_t z, int32_t distance){
    int32_t denominator = distance + z;
    int32_t scaleFactor = 1000;
    int32_t scaled_distance = distance * scaleFactor;
    int32_t ratio = scaled_distance / denominator;
    return (int16_t)((coordinate * ratio) / scaleFactor);
}
*/

// Draw the cube by connecting the vertices with lines
void drawCube(int angleX, int angleY, int angleZ, int16_t color, uint8_t mode, uint32_t position, uint16_t buffer_data_address) {

    int16_t x2d[8], y2d[8], z2d[8];
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;

    // Rotate and project all vertices
    for (uint8_t i = 0; i < 8; i++) {

        if(!calculations_completed || first_run){

            int16_t vertex[3] = {cube_vertices[i][0], cube_vertices[i][1], cube_vertices[i][2]};
            x = vertex[0];
            y = vertex[1];
            z = vertex[2];

            // rotate y
            int16_t rotx = ((long)x * (long)cosine_values[angleY] + (long)z * (long)sine_values[angleY])>> 12;
            int16_t roty = (long)y;
            int16_t rotz = ((long)z * (long)cosine_values[angleY] - (long)x * (long)sine_values[angleY])>> 12;
            // rotate x
            int16_t rotxx = (long)rotx;
            int16_t rotyy = ((long)roty * (long)cosine_values[angleX] - (long)rotz * (long)sine_values[angleX])>> 12;
            int16_t rotzz = ((long)roty * (long)sine_values[angleX] + (long)rotz * (long)cosine_values[angleX])>> 12;
            // rotate z
            int16_t rotxxx = ((long)rotxx * (long)cosine_values[angleZ] - (long)rotyy * (long)sine_values[angleZ])>> 12;
            int16_t rotyyy = ((long)rotxx * (long)sine_values[angleZ] + (long)rotyy * (long)cosine_values[angleZ])>> 12;
            int16_t rotzzz = (long)rotzz;

            // add perspective
            int16_t rotxxx_p = rotxxx; // addPerspective((int32_t)rotxxx, (int32_t)rotzzz, (int32_t)distance);
            int16_t rotyyy_p = rotyyy; // addPerspective((int32_t)rotyyy, (int32_t)rotzzz, (int32_t)distance);

            // send to center of the screen (with optional offset)
            x = (rotxxx_p / SCALE) + SCREEN_WIDTH / 2 + OFFSET_X;
            y = (rotyyy_p / SCALE) + SCREEN_HEIGHT / 2 + OFFSET_Y;
            z = (rotzzz / SCALE);

            cube_vertices_precalculated[position][i][0] = x;
            cube_vertices_precalculated[position][i][1] = y;
            cube_vertices_precalculated[position][i][2] = z;

        } else {

            x = cube_vertices_precalculated[position][i][0];
            y = cube_vertices_precalculated[position][i][1];
            z = cube_vertices_precalculated[position][i][2];

        }

        x2d[i] = x;
        y2d[i] = y;
        z2d[i] = z;
        
    }

    // show additional infos
    if (show_vertex_coordinates){
        for (uint8_t i = 0; i < 8; i++) {
            // set_cursor(10,10);
            // sprintf(*buf,"distance: %d", distance);
            // draw_string2buffer(*buf, buffer_data_address);
            set_cursor(20, 40 + i * 10);
            sprintf(*buf,"%d (%d,%d,%d)", i, x2d[i], y2d[i], z2d[i]);
            draw_string2buffer(*buf, buffer_data_address);
        }
    }

    if(calculations_completed || first_run){
        // Connect the vertices with lines to draw the cube (front and back faces)
        if (mode == 0 || mode > 3) {
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
            // additional cross to indicate front side
            if(mode == 4){
                draw_line2buffer(color, x2d[2], y2d[2], x2d[7], y2d[7], buffer_data_address);
                draw_line2buffer(color, x2d[3], y2d[3], x2d[6], y2d[6], buffer_data_address);
            }
        }

        if (mode == 1) {
            for(int v = 0; v < 8; v++){
                draw_pixel2buffer(color, x2d[v], y2d[v], buffer_data_address);
            }
        }

        if (mode > 1){
            for(int v = 0; v < 8; v++){
                // if(z2d[v] <= 0) draw_circle2buffer(color, x2d[v], y2d[v], 3, buffer_data_address);
                set_cursor(x2d[v] + 3, y2d[v] + 3);
                // sprintf(*buf,"%d(%d,%d,%d)", v, x2d[v], y2d[v], z2d[v]);
                set_text_multiplier((z2d[v] < 0) ? ((z2d[v] < -90) ? 3 : 2) : 1);
                sprintf(*buf,"%d", v);
                draw_string2buffer(*buf, buffer_data_address);
                set_text_multiplier(1);
            }
        }
    }
}

int main() {
    
    // Precompute sine and cosine values
    precompute_sin_cos();

    bool handled_key = false;
    uint8_t mode = 0;
    uint8_t i = 0;
    buffers[0] = 0x0000;
#ifdef HIRES
    buffers[1] = 0x7080;
#else
    buffers[1] = 0x2580;
#endif
    erase_buffer(buffers[0]);
    erase_buffer(buffers[1]);

#ifdef HIRES
    init_bitmap_graphics(0xFF00, buffers[0], 0, 4, SCREEN_WIDTH, SCREEN_HEIGHT, 1);
#else
    init_bitmap_graphics(0xFF00, buffers[0], 0, 1, SCREEN_WIDTH, SCREEN_HEIGHT, 1);
#endif

    // force 1st buffer
    active_buffer = 0;
    switch_buffer(buffers[active_buffer]);

    // start angles
    int start_angleX = 30, start_angleY = 30, start_angleZ = 15;
    cube_position = 0;
    drawCube(start_angleX, start_angleY, start_angleZ, WHITE, mode, cube_position, buffers[active_buffer]);
    int angleX = start_angleX;
    int angleY = start_angleY;
    int angleZ = start_angleZ;
    mode = 1;

    set_text_multiplier(4);
    set_cursor(10, 10);
    draw_string2buffer("3D cube", buffers[active_buffer]);
    set_text_multiplier(1);

    set_cursor(10, SCREEN_HEIGHT - 60);
    draw_string2buffer("[SPACE] start/stop", buffers[active_buffer]);
    set_cursor(10, SCREEN_HEIGHT - 50);
    draw_string2buffer("[M]     cycle thru drawing modes", buffers[active_buffer]);
    set_cursor(10, SCREEN_HEIGHT - 40);
    draw_string2buffer("[B]     show/hide buffer indicator", buffers[active_buffer]);
    set_cursor(10, SCREEN_HEIGHT - 30);
    draw_string2buffer("[ESC]   exit", buffers[active_buffer]);

    set_cursor(10, SCREEN_HEIGHT - 10);
    draw_string2buffer("PRESS ANY KEY TO START", buffers[active_buffer]);
    WaitForAnyKey();

    while (true) {

        if(!paused || first_run){
           // Update rotation angles
            angleX = (angleX + (360 / NUM_POINTS)) % NUM_POINTS;
            angleY = (angleY + (360 / NUM_POINTS)) % NUM_POINTS;
            angleZ = (angleZ + (360 / NUM_POINTS)) % NUM_POINTS;
            if(angleX == start_angleX && angleY == start_angleY && angleZ == start_angleZ)
            {
                if(!calculations_completed){
                    calculations_completed = true;
                    first_run = false;
                    paused = false;
                    mode = 0;
                    // printf("number of calculated cube positions: %lu\n",cube_position + 1);
                }
                cube_position = 0;
            } else {
                cube_position++;
            }
            // screen double buffering magic
            // draw on inactive buffer
            erase_buffer(buffers[!active_buffer]);
            drawCube(angleX, angleY, angleZ, WHITE, mode, cube_position, buffers[!active_buffer]);

            if(!calculations_completed){
                set_cursor(0,15);
                draw_string2buffer("PRECOMPUTING COORDINATES, PLEASE WAIT ...", buffers[!active_buffer]);
                draw_rect2buffer(WHITE, 0, 0, (NUM_POINTS) + 4                  , 10, buffers[!active_buffer]);
                fill_rect2buffer(WHITE, 2, 2, (NUM_POINTS) - (cube_position + 1),  6, buffers[!active_buffer]);
            }
            if(show_indicators){
                draw_circle2buffer(WHITE, (active_buffer ? SCREEN_WIDTH - 20 : 20), 20, 8, buffers[!active_buffer]);
                set_cursor((active_buffer ? SCREEN_WIDTH - 22 : 18), 17);
                draw_string2buffer((active_buffer ? "0" : "1"), buffers[!active_buffer]);
            }
           
            // switch to updated buffer
            switch_buffer(buffers[!active_buffer]);
            // switch active buffer index for next loop
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
                //if ((((i<<3)+j)>3) && (new_key != (keystates[i] & (1<<j)))) {
                    // printf( "key %d %s\n", ((i<<3)+j), (new_key ? "pressed" : "released"));
                //}
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
                        set_text_multiplier(4);
                        set_cursor(10, 10);
                        draw_string2buffer("3D cube", buffers[active_buffer]);
                        set_text_multiplier(1);
                        set_cursor(10, SCREEN_HEIGHT - 60);
                        draw_string2buffer("[SPACE] start/stop", buffers[active_buffer]);
                        set_cursor(10, SCREEN_HEIGHT - 50);
                        draw_string2buffer("[M]     cycle thru drawing modes", buffers[active_buffer]);
                        set_cursor(10, SCREEN_HEIGHT - 40);
                        draw_string2buffer("[B]     show/hide buffer indicator", buffers[active_buffer]);
                        set_cursor(10, SCREEN_HEIGHT - 30);
                        draw_string2buffer("[ESC]   exit", buffers[active_buffer]);
                        set_cursor(10, SCREEN_HEIGHT - 10);
                        draw_string2buffer("Press SPACE to continue", buffers[active_buffer]);
                    }
                }
                if (key(KEY_B)) {
                    show_indicators = !show_indicators;
                }
                if (key(KEY_M)) {
                    mode = ((mode + 1) > NUM_MODES ? 0 : (mode + 1));
                }
                if (key(KEY_C)) {
                    show_vertex_coordinates = !show_vertex_coordinates;
                }
                if (key(KEY_UP)) {
                    distance = ((distance - 50) < 100 ? 100 : (distance - 50));
                }
                if (key(KEY_DOWN)) {
                    distance = ((distance + 50) > 1000 ? 1000 : (distance + 50));
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
