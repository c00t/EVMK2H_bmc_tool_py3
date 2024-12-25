/******************************************************************************
 *
 * File	Name:       bmc_lcd.h
 *
 * Description: This contains includes, defines, and function prototypes for 
 *              BMC LCD module.
 * 
 ******************************************************************************/
/****************************************************************************
 * Copyright (c) 2012 Texas Instruments Incorporated - http://www.ti.com
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#ifndef BMC_LCD_H_
#define BMC_LCD_H_

/* ===================== Includes ================== */
#include "bmc_map.h"
#include "bmc.h"
/* ============================= Defines ================================== */
#define high4(val)      (((val) & 0xF0) >> 4)   // Access the four most significant bits of the value(val).
#define low4(val)       ((val) & 0x0F)          // Access the four least significant bits of the value(val).
#define MAX_CHAR        16                      // The maximum number of characters that can fit on one line.
#define MAX_LINES       (MAX_PAGE/2)            // The maximum number of lines of text that can fit on the LCD.

//Commands:

    //User Commands for the LCD:
    #define ADC_NORM            0xA0            // Sets the segment driver direction to normal.
    #define ADC_RVRSE           0xA1            // Sets the segment driver direction to reverse.
    #define COM_OUT_NORM        0xC0            // Sets the common output mode to normal.
    #define COM_OUT_RVRSE       0xC8            // Sets the common output mode to reverse.
    #define LCD_ON              0xAF            // Turns the LCD on.
    #define LCD_OFF             0xAE            // Turns the LCD off.
    #define ALL_PIXELS_ON       0xA5            // Turns every pixel on.
    #define PIXELS_NORM         0xA4            // Returns the pixels to normal after ALL_PIXELS_ON.
    #define DPY_NORM            0xA6            // Sets 1 to be on and 0 to be off for pixels.
    #define DPY_RVRSE           0xA7            // Sets 0 to be on and 1 to be off for pixels.
    #define READ_MOD_WRT        0xE0            // Sets the column to increment on write, but not on read.
    #define READ_MOD_WRT_END    0xEE            // Ends READ_MOD_WRT and sets the column back to where it began.
    #define INTRNL_RST          0xE2            // Resets the LCD.
    #define SLEEP_ON            0xAC            // Makes the LCD sleep.
    #define SLEEP_OFF           0xAD            // Makes the LCD stop sleeping.
    #define NOP                 0xE3            // Non-operation Command.

    //Initialization Commands for the LCD:
    #define SET_OP_MODE             0x2F
    #define SET_BIAS                0xA2        // Sets the voltage bias ratio.
    #define SET_SET_V0_VOL_REG_MODE 0x81        // Sets the V0 electronic volume.(Part 1)
    #define SET_V0_VOL_REG          0x2F        // Sets the V0 electronic volume.(Part 2)

    /* Note: The following value is different from the one given by the LCD manufacturer.
     * Our testing seemed to indicate that the manufacturer gives the wrong value(0x26 instead of 0x21)
     * and the LCD will not function correctly if that value is used.
     */

    #define SET_OHM_RATIO   0x21                // Sets the internal resistor ratio.
    #define SET_START_LINE  0x40                // Sets the start line to 0.

//Values
#define LCD_PAGES       16                   // The number of pages.
#define MAX_PAGE        4                    // The number of pixel pages.
#define MAX_COL         128                  // The number of pixel columns.
#define PIX_PER_PIXPAGE 13                   // The number of pixels per pixel page.
#define TOGGLE_DELAY    6000                 // The amount of delay for visual toggling in microseconds.

#define LCD_SOCBELOW    0                    // SOC is below LCD from this view (edge of board is up)
#define LCD_SOCABOVE    1                    // SOC is above LCD from this view (edge of board is down);
#define LCD_DIRECTION   LCD_SOCABOVE         // Direction of LCD, set to LCD_SOCBELOW or LCD_SOCABOVE

#if (LCD_DIRECTION == LCD_SOCBELOW)
#define LCD_COMPARE(x, y)     x < y
#else
#define LCD_COMPARE(x, y)     x >= y
#endif

//Pages:
#define BOARD_INFO      0                    // Page 0 of LCD
#define SETUP_STATE     1                    // Page 1 of LCD
#define FAN_TEMP        2                    // Page 2 of LCD
#define PWR_USAGE       3                    // Page 3 of LCD
#define CLK_PAGE        4                    // Page 4 of LCD
#define BOOTMODE_PAGE   5                    // Page 5 of LCD
#define BMC_NOTICE      6                    // Page 6 of LCD

/* ============================= Function Prototypes ================================== */
extern void LCD_UpdatePage(char *strPrint, int page, int line);
extern void LCD_Printf(int page, int line, const char* format, ...);

#endif /* BMC_LCD_H_ */
