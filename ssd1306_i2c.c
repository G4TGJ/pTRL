/**
 * SSD1306 OLED driver over I2C
 * 
 * Original Copyright (c) 2021 Raspberry Pi (Trading) Ltd.
 *
 * Modified by Richard Tomlinson G4TGJ 2024
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "config.h"
#include "ssd1306.h"
#include "font.h"

#ifdef OLED_DISPLAY
/* Example code to talk to an SSD1306-based OLED display

   The SSD1306 is an OLED/PLED driver chip, capable of driving displays up to
   128x64 pixels.

   NOTE: Ensure the device is capable of being driven at 3.3v NOT 5v. The Pico
   GPIO (and therefore I2C) cannot be used at 5v.

   You will need to use a level shifter on the I2C lines if you want to run the
   board at 5v.

   Connections on Raspberry Pi Pico board, other boards may vary.

   GPIO PICO_DEFAULT_I2C_SDA_PIN (on Pico this is GP4 (pin 6)) -> SDA on display
   board
   GPIO PICO_DEFAULT_I2C_SCL_PIN (on Pico this is GP5 (pin 7)) -> SCL on
   display board
   3.3v (pin 36) -> VCC on display board
   GND (pin 38)  -> GND on display board
*/

#define SSD1306_I2C_ADDR            _u(0x3C)

// commands (see datasheet)
#define SSD1306_SET_MEM_MODE        _u(0x20)
#define SSD1306_SET_COL_ADDR        _u(0x21)
#define SSD1306_SET_PAGE_ADDR       _u(0x22)
#define SSD1306_SET_HORIZ_SCROLL    _u(0x26)
#define SSD1306_SET_SCROLL          _u(0x2E)

#define SSD1306_SET_DISP_START_LINE _u(0x40)

#define SSD1306_SET_CONTRAST        _u(0x81)
#define SSD1306_SET_CHARGE_PUMP     _u(0x8D)

#define SSD1306_SET_SEG_REMAP       _u(0xA0)
#define SSD1306_SET_ENTIRE_ON       _u(0xA4)
#define SSD1306_SET_ALL_ON          _u(0xA5)
#define SSD1306_SET_NORM_DISP       _u(0xA6)
#define SSD1306_SET_INV_DISP        _u(0xA7)
#define SSD1306_SET_MUX_RATIO       _u(0xA8)
#define SSD1306_SET_DISP            _u(0xAE)
#define SSD1306_SET_COM_OUT_DIR     _u(0xC0)
#define SSD1306_SET_COM_OUT_DIR_FLIP _u(0xC0)

#define SSD1306_SET_DISP_OFFSET     _u(0xD3)
#define SSD1306_SET_DISP_CLK_DIV    _u(0xD5)
#define SSD1306_SET_PRECHARGE       _u(0xD9)
#define SSD1306_SET_COM_PIN_CFG     _u(0xDA)
#define SSD1306_SET_VCOM_DESEL      _u(0xDB)

#define SSD1306_PAGE_HEIGHT         _u(8)
#define SSD1306_NUM_PAGES           (OLED_HEIGHT / SSD1306_PAGE_HEIGHT)
#define SSD1306_BUF_LEN             (SSD1306_NUM_PAGES * OLED_WIDTH)

#define SSD1306_WRITE_MODE         _u(0xFE)
#define SSD1306_READ_MODE          _u(0xFF)

#define CHAR_WIDTH 8
#define CHAR_HEIGHT 8

#define ROW_HEIGHT 8

struct render_area {
    uint8_t start_col;
    uint8_t end_col;
    uint8_t start_page;
    uint8_t end_page;

    int buflen;
};

// There are two buffers representing the OLED
// We write to one and then compare it to the other
// when deciding what to write to the screen
// This speeds things up and reduces I2C traffic

// We write to this buffer
static uint8_t oledBuf[SSD1306_BUF_LEN];

// This represents what is on the screen
static uint8_t screenOledBuf[SSD1306_BUF_LEN];

static struct render_area frame_area =
{
    start_col: 0,
    end_col : OLED_WIDTH - 1,
    start_page : 0,
    end_page : SSD1306_NUM_PAGES - 1
};

static void calc_render_area_buflen(struct render_area *area) {
    // calculate how long the flattened buffer will be for a render area
    area->buflen = (area->end_col - area->start_col + 1) * (area->end_page - area->start_page + 1);
}

#ifdef i2c_default

static void SSD1306_send_cmd(uint8_t cmd) {
    // I2C write process expects a control byte followed by data
    // this "data" can be a command or data to follow up a command
    // Co = 1, D/C = 0 => the driver expects a command
    uint8_t buf[2] = {0x80, cmd};
    i2c_write_blocking(i2c_default, SSD1306_I2C_ADDR, buf, 2, false);
}

static void SSD1306_send_cmd_list(uint8_t *buf, int num) {
    for (int i=0;i<num;i++)
        SSD1306_send_cmd(buf[i]);
}

static void SSD1306_send_buf(uint8_t buf[], int buflen) {
    // in horizontal addressing mode, the column address pointer auto-increments
    // and then wraps around to the next page, so we can send the entire frame
    // buffer in one gooooooo!

    // copy our frame buffer into a new buffer because we need to add the control byte
    // to the beginning

    uint8_t *temp_buf = malloc(buflen + 1);

    temp_buf[0] = 0x40;
    memcpy(temp_buf+1, buf, buflen);

    i2c_write_blocking(i2c_default, SSD1306_I2C_ADDR, temp_buf, buflen + 1, false);

    free(temp_buf);
}

void SSD1306_scroll(bool on) {
    // configure horizontal scrolling
    uint8_t cmds[] = {
        SSD1306_SET_HORIZ_SCROLL | 0x00,
        0x00, // dummy byte
        0x00, // start page 0
        0x00, // time interval
        0x03, // end page 3 SSD1306_NUM_PAGES ??
        0x00, // dummy byte
        0xFF, // dummy byte
        SSD1306_SET_SCROLL | (on ? 0x01 : 0) // Start/stop scrolling
    };

    SSD1306_send_cmd_list(cmds, count_of(cmds));
}

static void render(uint8_t *buf, struct render_area *area)
{
    // update a portion of the display with a render area
    uint8_t cmds[] = {
        SSD1306_SET_COL_ADDR,
        area->start_col,
        area->end_col,
        SSD1306_SET_PAGE_ADDR,
        area->start_page,
        area->end_page
    };
    
    SSD1306_send_cmd_list(cmds, count_of(cmds));
    SSD1306_send_buf(buf, area->buflen);
}

// Render the screen but only write area that has changed
static void renderScreen( void )
{
    // Compare the two buffers, page by page
    // We will render the part of each page that has changed
    for( int page = 0 ; page < SSD1306_NUM_PAGES ; page++ )
    {
        // Start with the render area set to extremes
        // We will set this to sensible values as we find differences
        struct render_area area =
        {
            start_col : OLED_WIDTH-1,
            end_col : 0,
            start_page : page,
            end_page : page
        };

        // See if any bytes on this page have changed
        bool different = false;
        for( int x = 0 ; x < OLED_WIDTH ; x++ )
        {
            // Work out the byte in the buffer that represents the page and horizontal position
            int byte = page * OLED_WIDTH + x;
            if( oledBuf[byte] != screenOledBuf[byte] )
            {
                if( x < area.start_col )
                {
                    area.start_col = x;
                }
                if( x > area.end_col )
                {
                    area.end_col = x;
                }

                // Must also copy the byte over to the screen buffer
                screenOledBuf[byte] = oledBuf[byte];
                different = true;
            }
        }

        // Render the different part of the page
        if( different )
        {
            calc_render_area_buflen(&area);
            render(&oledBuf[area.start_col + area.start_page*OLED_WIDTH], &area);
        }
    }
}

static void SetPixel( uint8_t *buf, int x, int y, bool on )
{
    if( x >= 0 && x < OLED_WIDTH && y >=0 && y < OLED_HEIGHT )
    {
        // The calculation to determine the correct bit to set depends on which address
        // mode we are in. This code assumes horizontal

        // The video ram on the SSD1306 is split up in to 8 rows, one bit per pixel.
        // Each row is 128 long by 8 pixels high, each byte vertically arranged, so byte 0 is x=0, y=0->7,
        // byte 1 is x = 1, y=0->7 etc

        // This code could be optimised, but is like this for clarity. The compiler
        // should do a half decent job optimising it anyway.

        const int BytesPerRow = OLED_WIDTH ; // x pixels, 1bpp, but each row is 8 pixel high, so (x / 8) * 8

        int byte_idx = (y / 8) * BytesPerRow + x;
        uint8_t byte = buf[byte_idx];

        if (on)
        {
            byte |=  1 << (y % 8);
        }
        else
        {
            byte &= ~(1 << (y % 8));
        }

        buf[byte_idx] = byte;
    }
}

// Basic Bresenhams.
void oledDrawLine(int x0, int y0, int x1, int y1, bool on, bool render)
{
    int dx =  abs(x1-x0);
    int sx = x0<x1 ? 1 : -1;
    int dy = -abs(y1-y0);
    int sy = y0<y1 ? 1 : -1;
    int err = dx+dy;
    int e2;

    while (true)
    {
        SetPixel(oledBuf, x0, y0, on);
        if (x0 == x1 && y0 == y1)
        {
            break;
        }
        e2 = 2*err;

        if (e2 >= dy)
        {
            err += dy;
            x0 += sx;
        }
        
        if (e2 <= dx)
        {
            err += dx;
            y0 += sy;
        }
    }

    if( render )
    {
        renderScreen();
    }
}

// Draw a solid rectangle. Often used with on=false to clear an area.
void oledDrawRectangle(int x0, int y0, int width, int height, bool on, bool render)
{
    for( int y = y0 ; y < y0 + height ; y++ )
    {
        oledDrawLine( x0, y, x0 + width, y, on, false );
    }

    if( render )
    {
        renderScreen();
    }
}

// Write a string to the oled starting at pixel x,y
void oledWriteString(int x, int y, const char *str, int fontNum, bool render)
{
    const uint8_t *font = getFont(fontNum);

    const int fontHeight = getFontHeight(fontNum);
    const int fontWidth = getFontWidth(fontNum);
    const int fontFirstChar = getFontFirstChar(fontNum);
    const int fontNumChars = getFontNumChars(fontNum);
    const int fontLastChar = fontFirstChar + fontNumChars - 1;

    // Process each character
    int len = strlen(str);
    for( int i = 0 ; i < len ; i++)
    {
        uint8_t ch = str[i];

        // Check the character is in the font
        if( ch >= fontFirstChar && ch <= fontLastChar )
        {
            int fontIndex = (ch - fontFirstChar) * fontHeight * fontWidth/8;
            for( int i = 0 ; i < fontHeight ; i++ )
            {
                for( int pixel = 0 ; pixel < fontWidth ; pixel++ )
                {
                    int byte = pixel / 8;
                    int bit = pixel % 8;
                    SetPixel( oledBuf, x + pixel, y + i, font[fontIndex + i*fontWidth/8 + byte] & (0x80>>bit) );
                }
            }
        }

        x += fontWidth;
    }

    if( render )
    {
        renderScreen();
    }
}

void oledInit() {
    // Some of these commands are not strictly necessary as the reset
    // process defaults to some of these but they are shown here
    // to demonstrate what the initialization sequence looks like
    // Some configuration values are recommended by the board manufacturer

    uint8_t cmds[] = {
        SSD1306_SET_DISP,               // set display off
        /* memory mapping */
        SSD1306_SET_MEM_MODE,           // set memory address mode 0 = horizontal, 1 = vertical, 2 = page
        0x00,                           // horizontal addressing mode
        /* resolution and layout */
        SSD1306_SET_DISP_START_LINE,    // set display start line to 0
        SSD1306_SET_SEG_REMAP | 0x01,   // set segment re-map, column address 127 is mapped to SEG0
        SSD1306_SET_MUX_RATIO,          // set multiplex ratio
        OLED_HEIGHT - 1,             // Display height - 1
        SSD1306_SET_COM_OUT_DIR | 0x08, // set COM (common) output scan direction. Scan from bottom up, COM[N-1] to COM0
        SSD1306_SET_DISP_OFFSET,        // set display offset
        0x00,                           // no offset
        SSD1306_SET_COM_PIN_CFG,        // set COM (common) pins hardware configuration. Board specific magic number. 
                                        // 0x02 Works for 128x32, 0x12 Possibly works for 128x64. Other options 0x22, 0x32
#if ((OLED_WIDTH == 128) && (OLED_HEIGHT == 32))
        0x02,                           
#elif ((OLED_WIDTH == 128) && (OLED_HEIGHT == 64))
        0x12,
#else
        0x02,
#endif
        /* timing and driving scheme */
        SSD1306_SET_DISP_CLK_DIV,       // set display clock divide ratio
        0x00,                           // div ratio of 1, lowest freq
        SSD1306_SET_PRECHARGE,          // set pre-charge period
        0xF1,                           // Vcc internally generated on our board
        SSD1306_SET_VCOM_DESEL,         // set VCOMH deselect level
        0x30,                           // 0.83xVcc
        /* display */
        SSD1306_SET_CONTRAST,           // set contrast control
        0xFF,
        SSD1306_SET_ENTIRE_ON,          // set entire display on to follow RAM content
        SSD1306_SET_NORM_DISP,           // set normal (not inverted) display
        SSD1306_SET_CHARGE_PUMP,        // set charge pump
        0x14,                           // Vcc internally generated on our board
        SSD1306_SET_SCROLL | 0x00,      // deactivate horizontal scrolling if set. This is necessary as memory writes will corrupt if scrolling was enabled
        SSD1306_SET_DISP | 0x01, // turn display on
    };

    SSD1306_send_cmd_list(cmds, count_of(cmds));

    // Initialize render area for entire frame (OLED_WIDTH pixels by SSD1306_NUM_PAGES pages)
    calc_render_area_buflen(&frame_area);

    // Clear the screen
    render(oledBuf, &frame_area);
}

#endif
#endif
