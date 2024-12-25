/******************************************************************************
 *
 * File	Name:       bmc_uart.c
 *
 * Description: This contains the internal and external functions for the BMC
 *              UART modules.
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include <xdc/runtime/Types.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>

#include "bmc_uart.h"

#include "inc/hw_uart.h"

#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"


#include "bmc_queue.h"
#include "bmc_heap.h"
#include "bmc_lcd.h"

#define UART_RX_EVENT          Event_Id_00
#define UART_QUEUE_EVENT       Event_Id_01

#define SOC_MESSAGE_MAX_SIZE   20
#define MAX_BUFFER_SIZE        512

const char *const SOC_LCD_MESSAGE = "#>>>>> LCD ";

char csCUARTBuffer[MAX_BUFFER_SIZE];
int g_bufferIndex = 0;

typedef struct UART_Handle
{
    char buffer[MAX_CMD][MAX_SIZE];
    Queue_Handle message_queue;
    UInt bufferPosY, cmdPosY;
    UInt bufferPosX, promptPosX;
    int prompt;
} UART_Handle;

typedef struct
{
    unsigned char seconds;
    unsigned char minutes;
    unsigned char hours;
} UART_CLOCK;

/* ============================== Globals ========================== */
extern void parse_cmd();
extern Swi_Handle UART_CLOCK_SWI;
extern Swi_Handle SOC_UART_SWI;
extern Event_Handle UART_events;
extern Semaphore_Handle CMD_sem;
extern Semaphore_Handle CMD_Start_sem;
static UART_Handle consoleUART;
static UART_Handle *const hUART = &consoleUART;
static UART_CLOCK uart_clock;
static UART_CLOCK *const hUClock = &uart_clock;
static char SOC_UART_Buffer[SOC_MESSAGE_MAX_SIZE];

/* ============================== Local Function Prototypes ============================ */
static void UART_Timestamp();
static void UART_Send(char *csData);
static void UART_PlacePrompt(UInt ulPrompt);
static void UART_ClearPrompt();

/*
 * UART Setup: Initialize variables and enable UART
 */ 
void CONSOLE_UART_Setup()
{
    hUART->bufferPosY = 0;
    hUART->bufferPosX = 0;
    hUART->cmdPosY = 0;
    hUART->promptPosX = 0;
    hUART->prompt = false;
    hUART->message_queue = Queue_create(NULL, NULL);
    
    hUClock->seconds = 0;
    hUClock->minutes = 0;
    hUClock->hours = 0;
}

void SOC_UART_Setup()
{
    memset(SOC_UART_Buffer, NULL, SOC_MESSAGE_MAX_SIZE);
}

/*
 * Posts to the UART Clock Swi
 */
void Swi_Trigger()
{
    Swi_post(UART_CLOCK_SWI);
}

/*
 * UART Clock Swi: increment clock
 */
void UART_Clock_Swi()
{
    hUClock->seconds++;
    if(hUClock->seconds == 60)
    {
        hUClock->seconds = 0;
        hUClock->minutes++;
        if(hUClock->minutes == 60)
        {
            hUClock->minutes = 0;
            hUClock->hours++;
            if(hUClock->hours > 99)
                hUClock->hours = 0;
        }
    }
}

/*
 * UART Swi: This SWI handles the SOC UART and posts to the LCD
 *           if the format is correct.
 */
void UART_Swi()
{
    static int i = 0;
    static int page = -1;
    static int line = 0;
    static tBoolean message_detect = false;
    long result;
    
    if(i >= SOC_MESSAGE_MAX_SIZE)
    {
        i = 0;
        memset(SOC_UART_Buffer, NULL, SOC_MESSAGE_MAX_SIZE);
    }
    
    result = ROM_UARTCharGetNonBlocking(SOC_UART_BASE);
    
    if(message_detect)
    {
        if(result == '\n')
        {
            if(line == 0 && page >= 0)
            {
                LCD_Printf(page, line++, SOC_UART_Buffer);
                CONSOLE_UART_Timestamp();
                CONSOLE_UART_Printf("%s on page %i line %i\r\n", SOC_UART_Buffer, page, line-1);
            }
            else if(page >= 0)
            {
                LCD_Printf(page, line--, SOC_UART_Buffer);
                CONSOLE_UART_Timestamp();
                CONSOLE_UART_Printf("%s on page %i line %i\r\n", SOC_UART_Buffer, page, line+1);
                page = -1;
                message_detect = false;
            }
            else
            {
                message_detect = false;
            }
            i = 0;
            memset(SOC_UART_Buffer, NULL, SOC_MESSAGE_MAX_SIZE);
        }
        else
        {
            SOC_UART_Buffer[i++] = result;
        }
    }
    else
    {
        if(i == 0 && result != '#')
        {
            SOC_UART_EnableRX();
            return;
        }
        if(result == '\n')
        {
            if(strncmp(SOC_UART_Buffer, SOC_LCD_MESSAGE, 11) == 0 && page < 0)
            {
                page = atoi(&SOC_UART_Buffer[11]);
                if(page < 12 || page > 15)
                {
                    page = -1;
                    i = 0;
                    memset(SOC_UART_Buffer, NULL, SOC_MESSAGE_MAX_SIZE);
                    SOC_UART_EnableRX();
                    return;
                }
                message_detect = true;
            }
            memset(SOC_UART_Buffer, NULL, SOC_MESSAGE_MAX_SIZE);
            i = 0;
        }
        else
        {
            SOC_UART_Buffer[i++] = (char)result;
        }
        
    }
    
    SOC_UART_EnableRX();
}

/*
 * UART Task: The SYS/BIOS Task that handles UART TX/RX functionality.
 */
void UART_Task()
{
    char nxtChar;
    UInt events;
    long result;
    
    Message *p_message;
    
    UART_Send("\033[2J");
    UART_Send("\r");
    UART_Send("\033[100B");
    UART_Send(UART_PROMPT);
    
    hUART->prompt = true;
    
    /* Main Task Loop */
    while(1)
    {
        /* Wait for the specified events */
        events = Event_pend(UART_events, Event_Id_NONE,
            (UART_RX_EVENT + UART_QUEUE_EVENT),
            BIOS_WAIT_FOREVER);
        
        /* Keep processing while there is data in the queue and rx buffer */
        while(!Queue_empty(hUART->message_queue) || (events & UART_RX_EVENT))
        {
            /* Process UART RX buffer */
            if(events & UART_RX_EVENT)
            {
                result = ROM_UARTCharGetNonBlocking(CONSOLE_UART_BASE);
                while(result > 0)
                {
                    nxtChar = (char) result;
                    /* Is the new value enter/return? */
                    if(nxtChar == '\r')
                    {
                        hUART->bufferPosY++;
                        hUART->bufferPosX = 0;
                        if(hUART->bufferPosY >= MAX_CMD) hUART->bufferPosY = 0;
                        memset(hUART->buffer[hUART->bufferPosY], '\0', MAX_SIZE);
                    }
                    else if(nxtChar == 0x7F || nxtChar == '\b')
                    {
                        if(hUART->bufferPosX > 0)
                        {
                            nxtChar = '\b';
                            hUART->buffer[hUART->bufferPosY][--hUART->bufferPosX] = '\0'; 
                        }
                    }
                    else if(nxtChar == '\t')
                    {
                        /* TODO add spaces mod 4 or 8, for now treat as a space */
                        nxtChar = ' ';
                        hUART->buffer[hUART->bufferPosY][hUART->bufferPosX++] = ' ';
                    }
                    else if(nxtChar == '\033')
                    {
                        result = ROM_UARTCharGet(CONSOLE_UART_BASE);
                        nxtChar = (unsigned char) result;
                        
                        if(nxtChar == '[')
                        {
                            result = ROM_UARTCharGet(CONSOLE_UART_BASE);
                            nxtChar = (unsigned char) result;
                            if(nxtChar == 'A')
                            {
                                if(hUART->cmdPosY == hUART->bufferPosY && hUART->cmdPosY > 0)
                                    hUART->cmdPosY--;
                                if(hUART->bufferPosY > 0)
                                    hUART->bufferPosY--;
                                hUART->bufferPosX = (UInt)strlen(hUART->buffer[hUART->bufferPosY]);
                                hUART->promptPosX = hUART->bufferPosX;
                                UART_ClearPrompt();
                                UART_PlacePrompt(hUART->bufferPosY);
                            }
                        }
                    }
                    else if(nxtChar >= 0x20 && nxtChar <= 0x7E)
                    {
                        hUART->buffer[hUART->bufferPosY][hUART->bufferPosX++] = nxtChar;
                    }
                    result = ROM_UARTCharGetNonBlocking(CONSOLE_UART_BASE);
                }
                
                if(Semaphore_pend(CMD_sem, BIOS_NO_WAIT)) /* The command task is not running */
                {
                    /* Is there a command that needs to be processed? */
                    if(hUART->bufferPosY != hUART->cmdPosY)
                    {
                        /* Clear the prompt and output the command */
                        UART_ClearPrompt();
                        UART_Timestamp();
                        UART_PlacePrompt(hUART->cmdPosY);
                        UART_Send("\r\n");
                        hUART->prompt = false;
                        hUART->promptPosX = 0;
                        
                        /* Parse the Command */
                        parse_cmd(hUART->buffer[hUART->cmdPosY++]);
                        if(hUART->cmdPosY >= MAX_CMD) hUART->cmdPosY = 0;
                        
                        /* Start the Command Task */
                        Semaphore_post(CMD_Start_sem);
                    }
                    /* Place prompt on screen if not already there */
                    if(!hUART->prompt)
                    {
                        UART_Send(UART_PROMPT);
                        hUART->prompt = true;
                    }
                    
                    /* Output all characters in buffer */
                    while(hUART->promptPosX < hUART->bufferPosX)
                    {
                        ROM_UARTCharPut(CONSOLE_UART_BASE, (unsigned char)hUART->buffer[hUART->bufferPosY][hUART->promptPosX++]);
                    }
                    
                    /* Backspace */
                    while(hUART->promptPosX > hUART->bufferPosX)
                    {
                        UART_Send("\b \b");
                        hUART->promptPosX--;
                    }
                    Semaphore_post(CMD_sem);
                }
            }
            
            /* Process queue */
            if(!Queue_empty(hUART->message_queue))
            {
                /* Clear the prompt */
                UART_ClearPrompt();
                p_message = Queue_dequeue(hUART->message_queue);
                if(p_message)
                {
                    UART_Send(p_message->message);
                    BMC_Free(p_message->message, NULL);
                    BMC_Free(p_message, NULL);
                }
                UART_PlacePrompt(hUART->bufferPosY);
            }
            
            events = Event_pend(UART_events, UART_RX_EVENT, Event_Id_NONE, BIOS_NO_WAIT);
        }
    }
}

/*
 * UART printf: a printf for the UART console
 */
void CONSOLE_UART_Printf(const char *format, ...)
{
    char *csLine = (char*)BMC_Alloc(UART_MAX_CHAR, NULL);
    Message *newMessage = (Message*)BMC_Alloc(sizeof(Message), NULL);
    
    if(csLine == NULL || newMessage == NULL)
        return;
    
    va_list arglist;
    va_start(arglist, format);
    vsnprintf(csLine, UART_MAX_CHAR, format, arglist);
    va_end(arglist);
    newMessage->message = csLine;
    Queue_enqueue(hUART->message_queue, &(newMessage->elem));
    Event_post(UART_events, UART_QUEUE_EVENT);
}

/*
 * UART Timestamp: Print a timestamp out on the uart
 * external version, does uartprintf
 */
void CONSOLE_UART_Timestamp()
{
    CONSOLE_UART_Printf("[%02u:%02u:%02u]  ", hUClock->hours, hUClock->minutes, hUClock->seconds);
}

/*
 * UART Timestamp: Print a timestamp out on the uart
 * local version, directly prints
 */
static void UART_Timestamp()
{
    char timestamp_str[20];
    
    snprintf(timestamp_str, 20, "\r[%02u:%02u:%02u]  ", hUClock->hours, hUClock->minutes, hUClock->seconds);
    
    UART_Send(timestamp_str);
}

void CONSOLE_UART_ISR()
{
    unsigned long ulMaskIntStatus;
    //unsigned long result;

    ulMaskIntStatus = ROM_UARTIntStatus(CONSOLE_UART_BASE, true);     // Get the Interrupt Status

    ROM_UARTIntClear(CONSOLE_UART_BASE, ulMaskIntStatus);            // Clear the Interrupt
    
    /* result = ROM_UARTCharGetNonBlocking(CONSOLE_UART_BASE);
    if(result > 0)
    {
        csCUARTBuffer[g_bufferIndex++] = (char)(result & 0xFF);
        if(g_bufferIndex >= MAX_BUFFER_SIZE) g_bufferIndex = 0;
    } */
    
    if(ulMaskIntStatus > 0)
        Event_post(UART_events, UART_RX_EVENT);
}

void SOC_UART_ISR()
{
    unsigned long ulMaskIntStatus;
    
    ulMaskIntStatus = ROM_UARTIntStatus(SOC_UART_BASE, true);   // Get the Interrupt Status
    
    SOC_UART_DisableRX();
    
    ROM_UARTIntClear(SOC_UART_BASE, ulMaskIntStatus);           // Clear the Interrupt
    
    if(ulMaskIntStatus > 0)
        Swi_post(SOC_UART_SWI);
}

void CONSOLE_UART_EnableRX()
{
    ROM_UARTIntEnable(CONSOLE_UART_BASE, UART_INT_RT | UART_INT_RX);
}

void CONSOLE_UART_DisableRX()
{
    ROM_UARTIntDisable(CONSOLE_UART_BASE, UART_INT_RT | UART_INT_RX);
}

void SOC_UART_EnableRX()
{
    ROM_UARTIntEnable(SOC_UART_BASE, UART_INT_RT | UART_INT_RX);
}

void SOC_UART_DisableRX()
{
    ROM_UARTIntDisable(SOC_UART_BASE, UART_INT_RT | UART_INT_RX);
}

/*======================================= Local Functions =======================================*/

/*
 * UART Send: Sends the cstring out through the UART, will block if UART TX FIFO is full
 */
static void UART_Send(char *csData)
{
    while(*csData)
    {
        ROM_UARTCharPut(CONSOLE_UART_BASE, *csData++);
    }
}

/*
 * Place Prompt: Places the full prompt onto the current line of the UART
 */
static void UART_PlacePrompt(UInt ulPrompt)
{
    UART_Send(UART_PROMPT);
    hUART->prompt = true;
    UART_Send(hUART->buffer[ulPrompt]);
}

/*
 * Clear Prompt: Removes the prompt from the screen.
 */
static void UART_ClearPrompt()
{
    int i;
    
    for(i = 0; i < (hUART->promptPosX + sizeof(UART_PROMPT) - 1); i++)
    {
        UART_Send("\b");
        UART_Send(" ");
        UART_Send("\b");
    }
    hUART->prompt = false;
}
