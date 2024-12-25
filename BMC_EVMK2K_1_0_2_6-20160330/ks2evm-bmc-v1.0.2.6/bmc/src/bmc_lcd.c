/******************************************************************************
 *
 * File    Name:       bmc_lcd.c
 *
 * Description: This contains the internal and external functions for the BMC
 *              lcd module.
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

#include "bmc_map.h"
#include "bmc_lcd.h"
#include "bmc_font.h"
#include "bmc_heap.h"
#include "spi_driver.h"
#include "gpio_driver.h"

#include <xdc/runtime/Types.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/family/arm/lm3/TimeStampProvider.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#define NUM_LCD_SLOTS           4

typedef struct LCD_Page
{
    const int id;
    char text[MAX_LINES][MAX_CHAR+1];
    const tBoolean priority;
    const char *title;
} LCD_Page;

typedef struct LCD_Queue_Elem
{
    Queue_Elem elem;
    LCD_Page *page;
} LCD_Queue_Elem;

/* typedef struct LCD_Slot
{
    LCD_Page *page;
    unsigned long delay_time;
} LCD_Slot; */

/* ====================== Globals ====================== */
extern Swi_Handle LCD_swi;
extern Semaphore_Handle PAGE_sem;
extern Semaphore_Handle CONFIG_sem;
extern Semaphore_Handle LCD_sem;
extern Semaphore_Handle MESSAGE_sem;
extern Semaphore_Handle LCD_Print_Sem;
static Queue_Handle priority_Queue, changed_Queue;
static LCD_Page *idlePtr, *displayed, *priorityPtr, *changedPtr;
static unsigned char LCD_Image[MAX_LINES*PIX_PER_PIXPAGE][MAX_CHAR];
static unsigned char initBytes[11] =
{
    ADC_NORM,
    LCD_OFF,
    COM_OUT_RVRSE,
    SET_BIAS,
    SET_OP_MODE,
    SET_OHM_RATIO,
    SET_SET_V0_VOL_REG_MODE,
    SET_V0_VOL_REG,
    SET_START_LINE,
    PIXELS_NORM,
    LCD_ON
};
static LCD_Page pages[LCD_PAGES] =
{
    {   0,
        {{0},{0}},
        false,
        ""
    },
    {   1,
        {{0},{0}},
        false,
        ""
    },
    {   2,
        {{0},{0}},
        false,
        ""
    },
    {   3,
        {{0},{0}},
        false,
        ""
    },
    {   4,
        {{0},{0}},
        true,
        ""
    },
    {   5,
        {{0},{0}},
        false,
        ""
    },
    {   6,
        {{0},{0}},
        false,
        ""
    },
    {   7,
        {{0},{0}},
        false,
        ""
    },
    {   8,
        {{0},{0}},
        false,
        ""
    },
    {   9,
        {{0},{0}},
        false,
        ""
    },
    {   10,
        {{0},{0}},
        false,
        ""
    },
    {   11,
        {{0},{0}},
        false,
        ""
    },
    {   12,
        {{0},{0}},
        false,
        ""
    },
    {   13,
        {{0},{0}},
        false,
        ""
    },
    {   14,
        {{0},{0}},
        false,
        ""
    },
    {   15,
        {{0},{0}},
        false,
        ""
    }
};


/* ====================== Local Functions ======================= */
static int isBlank(int page);
static void string_draw(int x, int y, int n, char *string);
static void image_blt(int x, int y, int w, int h, const unsigned char *data);
static void print_image(unsigned char image[][MAX_COL/8]);
static void cls();
static void clear_image();
static void setcol(int col);
static void setpage(int page);
static void sendcmd(unsigned long ulCmd);
static void senddata(unsigned long ulData);

/*
 * Resets the LCD and performs other necessary setup.
 */
static void LCD_Setup()
{
    int count;
    unsigned long lcd_gpios;
    
    /* Reset the LCD */
    lcd_gpios = GPIO_Start(LCD_A0_PORT | LCD_RST_PORT);
    GPIO_SetType(LCD_A0_PORT, LCD_A0, LCD_A0, lcd_gpios);
    GPIO_SetType(LCD_RST_PORT, LCD_RST, LCD_RST, lcd_gpios);
    GPIO_RMW(LCD_A0_PORT, LCD_A0, 0, lcd_gpios);
    GPIO_RMW(LCD_RST_PORT, LCD_RST, 0, lcd_gpios);
    GPIO_Update(lcd_gpios);
    
    delay(100);
    
    lcd_gpios = GPIO_Start(LCD_RST_PORT);
    GPIO_RMW(LCD_RST_PORT, LCD_RST, LCD_RST, lcd_gpios);
    GPIO_Update(lcd_gpios);
    
    delay(100);

    for(count = 0; count < 11; count++)
    {
        sendcmd(initBytes[count]);
    }
    delay(100);
}

/*
 * Cycles through pages and prints them to the LCD
 */
void LCD_Pages(){
    
    int line, count;
    LCD_Queue_Elem *elemPtr;
    
    LCD_Setup();
    
    /* Create the queues */
    priority_Queue = Queue_create(NULL, NULL);
    changed_Queue = Queue_create(NULL, NULL);
    idlePtr = pages;
    displayed = NULL;
    priorityPtr = NULL;
    changedPtr = NULL;
    count = 0;

    /* Clear the LCD */
    cls();
    while(1)
    {
        if(Queue_empty(priority_Queue) && Queue_empty(changed_Queue) &&
            priorityPtr == NULL && changedPtr == NULL)
        {
            count = 0;
            while(isBlank(idlePtr->id))
            {
                if(idlePtr->id == LCD_PAGES-1)
                    idlePtr = pages;
                else
                    idlePtr++;
            }
            
            displayed = idlePtr;
            
            if(idlePtr->id == LCD_PAGES-1)
                idlePtr = pages;
            else
                idlePtr++;
            
            clear_image();
            Semaphore_pend(PAGE_sem, BIOS_WAIT_FOREVER);
            for(line = 0; line < MAX_LINES; line++)
                string_draw(0, line*font_num_lines, MAX_CHAR, &(displayed->text[line][0]));
            
            Semaphore_post(PAGE_sem);
            Semaphore_post(LCD_Print_Sem);
            Semaphore_pend(MESSAGE_sem, 2000);
        }
        else
        {
            Semaphore_pend(MESSAGE_sem, BIOS_NO_WAIT);
            
            if(!Queue_empty(priority_Queue))
            {
                changedPtr = NULL;
                
                elemPtr = (LCD_Queue_Elem*)Queue_dequeue(priority_Queue);
                priorityPtr = elemPtr->page;
                BMC_Free(elemPtr, NULL);
                displayed = priorityPtr;
            }
            else if(priorityPtr)
            {
                changedPtr = NULL;
                displayed = priorityPtr;
            }
            else if(!Queue_empty(changed_Queue))
            {
                priorityPtr = NULL;
                
                elemPtr = (LCD_Queue_Elem*)Queue_dequeue(changed_Queue);
                changedPtr = elemPtr->page;
                BMC_Free(elemPtr, NULL);
                displayed = changedPtr;
            }
            else if(changedPtr)
            {
                priorityPtr = NULL;
                displayed = changedPtr;
            }
            
            if(++count >= 3)
            {
                changedPtr = NULL;
                priorityPtr = NULL;
            }
            
            clear_image();
            Semaphore_pend(PAGE_sem, BIOS_WAIT_FOREVER);
            for(line = 0; line < MAX_LINES; line++)
            {
                string_draw(0, line*font_num_lines, MAX_CHAR, &(displayed->text[line][0]));
            }
            Semaphore_post(PAGE_sem);
            Semaphore_post(LCD_Print_Sem);
            
            delay(1000);
        }
    }
}

/*
 * Task that waits for a semaphore and prints
 */
void LCD_Print_Task()
{
    while(1)
    {
        Semaphore_pend(LCD_Print_Sem, BIOS_WAIT_FOREVER);
        print_image(LCD_Image);
    }
}

/*
 * SWI that updates the current display if it changes
 */
void LCD_SWI()
{
    int line;
    
    clear_image();
    for(line = 0; line < MAX_LINES; line++)
    {
        string_draw(0, line*font_num_lines, MAX_CHAR, &(displayed->text[line][0]));
    }
    
    Semaphore_post(LCD_Print_Sem);
}

/*
 * Changes the value of pages[page][line] to strPrint
 * 
 * Note: strPrint must be less than MAX_CHAR, page
 *       must be less than LCD_PAGES, and line must
 *       be less than MAX_LINES
 */
void LCD_UpdatePage(char *strPrint, int page, int line)
{
    if (line >= MAX_LINES)
        return;
        
    if (page >= LCD_PAGES)
        return;
    
    LCD_Queue_Elem *elemPtr;
    
    Semaphore_pend(PAGE_sem, BIOS_WAIT_FOREVER);
    strncpy(pages[page].text[line], strPrint, MAX_CHAR+1);
    Semaphore_post(PAGE_sem);
    
    elemPtr = (LCD_Queue_Elem*)BMC_Alloc(sizeof(LCD_Queue_Elem), NULL);
    if(elemPtr == NULL)
        return;
    
    elemPtr->page = &pages[page];
    
    if(pages[page].priority)
    {
        Queue_enqueue(priority_Queue, &(elemPtr->elem));
    }
    else
    {
        Queue_enqueue(changed_Queue, &(elemPtr->elem));
    }
    
    if(displayed->id == page)
    {
        Swi_post(LCD_swi);
    }
    
    Semaphore_post(MESSAGE_sem);
}

/*
 * LCD Printf: a printf for LCD, page and line work exactly like UpdatePage
 */
void LCD_Printf(int page, int line, const char *format, ...)
{
    char csLine[MAX_CHAR+1];
    
    va_list arglist;
    va_start(arglist, format);
    vsnprintf(csLine, MAX_CHAR+1, format, arglist);
    va_end(arglist);
    LCD_UpdatePage(csLine, page, line);
}

/*
 * Checks to see if the page(page_num) is blank.
 */
int isBlank(int page_num){
    int line;
    int c;
    int blank = true;

    Semaphore_pend(PAGE_sem, BIOS_WAIT_FOREVER);
    for(line = 0; line < MAX_LINES && blank; line++){
        for(c = 0; c < MAX_CHAR; c++){
            if(pages[page_num].text[line][c] == 0){
                break;
            }
            else if((pages[page_num].text[line][c] != ' ')){
                blank = false;
                break;
            }
        }
    }
    Semaphore_post(PAGE_sem);
    
    return blank;
}

/*
 * Writes a string into the image at a specified location(x,y) in pixels given the max length of the
 * string(n). This function does not write to location char x and line y. If these specific locations
 * are desired, they must be passed to the function in pixel form.
 */
void string_draw(int x, int y, int n, char *string)
{
    int i;
    char c;

    for(i = 0; i < n && *string != '\0'; i++, string++){
        c = *string;
        if (c > 0x7F) {
            c = ' ';
        }
        image_blt(x + i*font_num_cols, y, font_num_cols, font_num_lines, &fontdata[c * font_num_lines]);
    }
}

/*
 * Puts the data into the image at location(x,y). Also requires the width(w) and
 * height(h) in pixels of the data being passed.
 */
void image_blt(int x, int y, int w, int h, const unsigned char *data)
{
    int xb = x / 8;
    int bs = x % 8;
    int xo;
    int yo;
    unsigned char byte;
    unsigned char byte0;
    unsigned char byte1;

    for(yo = 0; yo < h; yo++){
        for(xo = 0; xo < (w+7)/8; xo++){
            byte = *data;
            data++;
            byte0 = byte >> bs;
            byte1 = byte << (8-bs);
            LCD_Image[y+yo][xb+xo] |= byte0;
            LCD_Image[y+yo][xb+xo+1] |= byte1;
        }
    }
}

/*
 * Translates the data given from the horizontal coding that our font has into the vertical coding that
 * the LCD uses and prints the image.
 */
void print_image(unsigned char image[][MAX_COL/8])
{
    int j;
    int k;
    int i;
    unsigned char tstmask;
    unsigned char setmask;
    unsigned char out;
    unsigned char tst;
    int fontline;
    
    /* Variables used to tell orientation of LCD */
    int lineStart, lineEnd, lineDir;
    int colStart, colEnd, colDir;
    int rowMult;
    
    if(LCD_DIRECTION == LCD_SOCBELOW)
    {
        lineStart = 0;
        lineEnd = (MAX_LINES*PIX_PER_PIXPAGE);
        lineDir = font_num_cols;
        
        colStart = 0;
        colEnd = MAX_COL;
        colDir = 1;
        
        rowMult = 1;
    }
    else
    {
        lineStart = (MAX_LINES*PIX_PER_PIXPAGE) - 1;
        lineEnd = 0;
        lineDir = 0-(font_num_cols);
        
        colStart = MAX_COL-1;
        colEnd = 0;
        colDir = -1;
        
        rowMult = -1;
    }

    for(fontline = lineStart, i = 0; LCD_COMPARE(fontline, lineEnd); i++, fontline += lineDir){
        setpage(0 + i);
        setcol(0);
        for(j = colStart; LCD_COMPARE(j, colEnd); j += colDir){
            out = 0;
            tstmask = 0x80 >> (j % 8);
            for(k = 0; k < font_num_cols; k++){
                if (LCD_COMPARE((fontline - k), lineEnd)){
                    setmask = 0x01 << k;
                    tst = image[fontline + (k*rowMult)][j / 8];
                    out |= (tst & tstmask ? setmask : 0);
                }
            }
            senddata(out);
        }
    }
}

/*
 * Clears the screen.
 */
void cls()
{
    /* unsigned char Blank_Image[MAX_PAGE*8][MAX_COL/8];
    memset(Blank_Image, 0, sizeof(Blank_Image));
    print_image(Blank_Image); */
    clear_image();
    Semaphore_post(LCD_Print_Sem);
}

/*
 * Clears the image.
 */
void clear_image()
{
    memset(LCD_Image, 0, sizeof(LCD_Image));
}

/*
 * Sets the LCD column.
 */
void setcol(int col)
{
    sendcmd(0x10 | high4(col));
    sendcmd(0x00 | low4(col));
}

/*
 * Sets the Pixel Page.
 */
void setpage(int page)
{
    sendcmd(0xB0 | page);
}

/*
 * Sends a command to the LCD.
 */
void sendcmd(unsigned long ulCmd)
{
    SPI_DEVICE *lcd_spi;
    unsigned long lcd_gpios;
    unsigned char write, read;
    
    write = (unsigned char) ulCmd;
    read = 0;
    
    lcd_spi = getSPIDevice(LCD_SPI);
    
    lcd_gpios = GPIO_Start(LCD_A0_PORT);
    GPIO_RMW(LCD_A0_PORT, LCD_A0, 0, lcd_gpios);
    GPIO_Update(lcd_gpios);
    
    SPI_Start(lcd_spi);
    SPI_RW(&write, &read, 1, lcd_spi);
    SPI_Stop(lcd_spi);
}

/*
 * Sends data to the LCD.
 */
void senddata(unsigned long ulData)
{
    SPI_DEVICE *lcd_spi;
    unsigned long lcd_gpios;
    unsigned char write, read;
    
    write = (unsigned char) ulData;
    read = 0;
    
    lcd_spi = getSPIDevice(LCD_SPI);
    
    lcd_gpios = GPIO_Start(LCD_A0_PORT);
    GPIO_RMW(LCD_A0_PORT, LCD_A0, LCD_A0, lcd_gpios);
    GPIO_Update(lcd_gpios);
    
    SPI_Start(lcd_spi);
    SPI_RW(&write, &read, 1, lcd_spi);
    SPI_Stop(lcd_spi);
}
