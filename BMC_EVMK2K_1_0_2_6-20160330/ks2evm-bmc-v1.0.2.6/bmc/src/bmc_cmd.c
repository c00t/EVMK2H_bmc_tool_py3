/******************************************************************************
 *
 * File	Name:       bmc_cmd.c
 *
 * Description: This file contains command_line functions for BMC.
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

#include "bmc_map.h"
#include "bmc.h"

#include "inc/hw_flash.h"
#include "driverlib/flash.h"

#include "bmc_commands.h"
#include "bmc_uart.h"
#include "bmc_clocks.h"
#include "bmc_lcd.h"
#include "bmc_state.h"
#include "bmc_heap.h"
#include "evm_types.h"
#include "spi_driver.h"
#include "i2c_driver.h"
#include "gpio_driver.h"

enum BOOT_TYPE { WARM, FULL, POR, PWR };

/* ================ Globals ================ */
extern Semaphore_Handle CMD_Start_sem;
extern Semaphore_Handle CMD_sem;
extern Semaphore_Handle STATE_MACHINE_sem;
extern Semaphore_Handle POLLING_sem;
static char cmdCpy[MAX_SIZE];
static char *buffer, *strBuf;
static char *argv[MAX_ARGS];
static int argc;

static const char *STATE_STRS[11] = {
    "Initialization",
    "Main Power Off",
    "Main Power On",
    "SOC State",
    "Error",
    "Board State",
    "SOC Power Off",
    "SOC Power On",
    "SOC Running",
    "SOC PWR Error",
    "Reset Error"
};
/* ======================= Function Prototypes ========================= */
static void convertLower(char *string);

/*
 * Command Task: Waits for signal from uart task and command processing
 */
void CMD_Task()
{
    int i;
    
    /* Output prompt */
    CONSOLE_UART_Printf("\r             \r\n\r\nBMC VERSION %s\r\nBUILT %s %s\r\n---------------------\r\n\r\n",
        stringVer, __DATE__, __TIME__ );
    
    while(1)
    {
        Semaphore_pend(CMD_Start_sem, BIOS_WAIT_FOREVER);
        Semaphore_pend(CMD_sem, BIOS_NO_WAIT);
        if(argv[0] != NULL)
        {
            for(i = 0; i < total_commands; i++)
            {
                if((strcmp(argv[0], command_list[i].title) == 0) && (!command_list[i].disabled))
                {
                    CONSOLE_UART_Timestamp();
                    CONSOLE_UART_Printf("Executing command \"%s\"\r\n", command_list[i].title);
                    (*command_list[i].function_call)(argc, argv);
                    break;
                }
            }
            if(i >= total_commands)
                CONSOLE_UART_Printf("Error: \"%s\" is not enabled or is not a command\r\nTo enable a hidden command use the 'hwdg' command\r\nType 'help hwdbg' for more information.\r\n", argv[0]);
        }
        Semaphore_post(CMD_sem);
    }
}

/*
 *  Parses the current command buffer into a command and arguments.
 */
void parse_cmd(char *command)
{
    /* Clear arguments */
    memset(argv, 0, sizeof(char*)*MAX_ARGS);
    argc = 0;
    
    if(command == NULL)
        return;

    strcpy(cmdCpy, command);
    
    convertLower(cmdCpy);

    // Get the command
    argv[argc++] = strtok_r(cmdCpy, " ", &strBuf);
    if (argv[0] != NULL)
    {
        // Get the arguments
        while (buffer = strtok_r(NULL, " ", &strBuf))
        {
            argv[argc++] = buffer;
        }
    }
}

static void convertLower(char *string)
{
    char *ptr = string;
    while(*ptr)
    {
        if(*ptr > 0x40 && *ptr < 0x5B)
        {
            *ptr |= 0x20;
        }
        ptr++;
    }
}

void CMD_hide(Command *cmd)
{
    cmd->hidden = true;
}

void CMD_unhide(Command *cmd)
{
    cmd->hidden = false;
}

void CMD_enable(Command *cmd)
{
    cmd->disabled = false;
}

void CMD_disable(Command *cmd)
{
    cmd->hidden = true;
    cmd->disabled = true;
}

/*
 * Command Functions: The following functions are used to implement
 * the various commands.
 */

/*
 * command_lcd: prints the two strings given to the LCD display
 */
void command_lcd(int argc, char **argv)
{
    int page, line;
    
    page = atoi(argv[2]);
    line = atoi(argv[3]);
    
    CONSOLE_UART_Timestamp();
    CONSOLE_UART_Printf("Adding \"%s\" to page %i line %i...\r\n", argv[1], page, line);
    LCD_UpdatePage(argv[1], page, line);
    CONSOLE_UART_Timestamp();
    CONSOLE_UART_Printf("Added to LCD\r\n");
}

/*
 * command_wait: delays umSec microseconds
 */
void command_wait(int argc, char **argv)
{
    unsigned long mSec;
    
    mSec = strtoul(argv[1], (char **) NULL, 10);
    
    CONSOLE_UART_Timestamp();
    CONSOLE_UART_Printf("Delaying %i milliseconds...\r\n", mSec);   
    delay(mSec);
    CONSOLE_UART_Timestamp();
    CONSOLE_UART_Printf("Finished\r\n");
}

/*
 * command_gpio: performs peek and poke operations
 * on the MCU and GPIO expander pins.
 */
void command_gpio(int argc, char **argv)
{
    int i, j;
    int expander = (strncmp(argv[1], "x", 1) == 0) ? 1 : 0;
    unsigned long port_key, ports;
    char *port;

    if(expander) // GPIO expander
    {
        switch (argv[1][1]) {
        case 'a':
            port = "GPIO expander A";
            port_key = GPIO_PORT_XA;
            break;
        case 'b':
            port = "GPIO expander B";
            port_key = GPIO_PORT_XB;
            break;
        case 'c':
            port = "GPIO expander C";
            port_key = GPIO_PORT_XC;
            break;
        case 'd':
            port = "GPIO expander D";
            port_key = GPIO_PORT_XD;
            break;
        default:
            return;
        }
    }
    else // MCU GPIO pin
    {
        /* Get port base */
        switch(*(argv[1])){
        case 'a':
            port = "port A";
            port_key = GPIO_PORT_A;
            break;
        case 'b':
            port = "port B";
            port_key = GPIO_PORT_B;
            break;
        case 'c':
            port = "port C";
            port_key = GPIO_PORT_C;
            break;
        case 'd':
            port = "port D";
            port_key = GPIO_PORT_D;
            break;
        case 'e':
            port = "port E";
            port_key = GPIO_PORT_E;
            break;
        case 'f':
            port = "port F";
            port_key = GPIO_PORT_F;
            break;
        case 'g':
            port = "port G";
            port_key = GPIO_PORT_G;
            break;
        case 'h':
            port = "port H";
            port_key = GPIO_PORT_H;
            break;
        case 'j':
            port = "port J";
            port_key = GPIO_PORT_J;
            break;
        default:
            return;
        }
    }

    if(argc > 2) // Write/input select
    {
        char *realValue, *nextChunk, *strBuf;
        unsigned short setBitMask, resetBitMask, inputBitMask;

        setBitMask = 0;
        resetBitMask = 0;
        inputBitMask = 0;

        /* Remove all '_' from value */
        realValue = strtok_r(argv[2], "_", &strBuf);
        while(nextChunk = strtok_r(NULL, "_", &strBuf))
            realValue = strcat(realValue, nextChunk);

        int valueSize = strlen(realValue);

        /* Set bit masks */
        if (valueSize > 16)
        {
            return;
        }

        for(i = 0; i < valueSize; i++)
        {
            switch (realValue[i]) {
            case '0':
                resetBitMask |= (1 << (valueSize - (i + 1)));
                break;
            case '1':
                setBitMask |= (1 << (valueSize - (i + 1)));
                break;
            case 'z':
                inputBitMask |= (1 << (valueSize - (i + 1)));
                break;
            default:
                break;
            }
        }
        
        CONSOLE_UART_Timestamp();
        CONSOLE_UART_Printf("Writing to %s...\r\n", port);        
        /* Check for conflict with UART */
        if(port_key == GPIO_PORT_A)
        {
            CONSOLE_UART_Timestamp();
            CONSOLE_UART_Printf("Cannot alter Bit 0 or 1 of Port A\r\n");
            resetBitMask &= ~(0x3);
            setBitMask &= ~(0x3);
            inputBitMask &= ~(0x3);
        }
        
        CONSOLE_UART_Timestamp();
        CONSOLE_UART_Printf("Inputs: 0x%04X\r\nHigh Outputs: 0x%04X\r\nLow Outputs: 0x%04X\r\n", inputBitMask, setBitMask, resetBitMask);

        ports = GPIO_Start(port_key);
        GPIO_SetType(port_key, (inputBitMask | setBitMask | resetBitMask), (setBitMask | resetBitMask), ports);
        GPIO_RMW(port_key, (setBitMask | resetBitMask), (setBitMask & ~resetBitMask), ports);
        GPIO_Update(ports);
        
        CONSOLE_UART_Timestamp();
        CONSOLE_UART_Printf("Finished setting %s...\r\n", port);
    }
    else // Read
    {
        CONSOLE_UART_Timestamp();
        CONSOLE_UART_Printf("Reading from %s...\r\n", port);
        unsigned short outmask, pin_values;
        char inResult[20], outResult[20];
        int size;
        
        memset(inResult, '\0', 20);
        memset(outResult, '\0', 20);
        
        ports = GPIO_Start(port_key);
        outmask = GPIO_GetType(port_key, ports);
        pin_values = GPIO_Get(port_key, ports);
        GPIO_Update(ports);
        
        if(expander)
            size = 15;
        else
            size = 7;
        
        j = 0;
        for(i = size; i >= 0; i--)
        {
            if(!(outmask & (1 << i)))
            {
                outResult[j] = 'X';
                if(pin_values & (1 << i))
                {
                    inResult[j] = '1';
                }
                else
                {
                    inResult[j] = '0';
                }
            }
            else
            {
                inResult[j] = 'X';
                if(pin_values & (1 << i))
                {
                    outResult[j] = '1';
                }
                else
                {
                    outResult[j] = '0';
                }
            }
            j++;
            if (i % 4 == 0 && i != 0)
            {
                outResult[j] = '_';
                inResult[j++] = '_';
            }
        }
        CONSOLE_UART_Timestamp();
        CONSOLE_UART_Printf("Inputs:  %s\r\n", inResult);
        CONSOLE_UART_Timestamp();
        CONSOLE_UART_Printf("Outputs: %s\r\n", outResult);
    }
}

/*
 * command_spi: Performs read and write operations on SPI.
 */
void command_spi(int argc, char **argv)
{
    int i, size;
    unsigned char key;
    unsigned char *write, *read;
    char *strBuf, **value;
    SPI_DEVICE *spi_device;
    
    size = argc - 2;
    key = (strtoul(strtok_r(argv[1], ".", &strBuf), (char **)NULL, 10) << 4) | 
          (strtoul(strtok_r(NULL, ".", &strBuf), (char **)NULL, 10));
    
    spi_device = getSPIDevice(key);
    if(spi_device == NULL)
    {
        CONSOLE_UART_Timestamp();
        CONSOLE_UART_Printf("ERROR: SPI Device not found.\r\n");
        return;
    }
    
    write = read = NULL;
    
    while(write == NULL)
    {
        write = (unsigned char*)BMC_Alloc(sizeof(unsigned char)*(size), NULL);
        delay(1);
    }
    while(read == NULL)
    {
        read = (unsigned char*)BMC_Alloc(sizeof(unsigned char)*(size), NULL);
        delay(1);
    }
    
    memset(write, NULL, size);
    memset(read, NULL, size);
    
    value = &argv[2];
    i = 0;
    while(i < size)
    {
        write[i++] = (unsigned char)strtoul(*value++, (char**)NULL, 16);
    }
    
    SPI_Start(spi_device);
    SPI_RW(write, read, size, spi_device);
    SPI_Stop(spi_device);
    
    CONSOLE_UART_Timestamp();
    CONSOLE_UART_Printf("Results: ");
    for(i = 0; i < size; i++)
    {
        CONSOLE_UART_Printf("0x%02X ", (char)read[i]);
    }
    CONSOLE_UART_Printf("\r\n");
    
    BMC_Free(write, NULL);
    BMC_Free(read, NULL);
}

/*
 * i2c Error Handling
 */
static void i2cErrorHandle(unsigned char errors, int number)
{
    if(errors == I2C_MASTER_ERROR_NONE)
        return;
    
    CONSOLE_UART_Timestamp();
    CONSOLE_UART_Printf("The following errors occurred on I2C operations number %d:\r\n", number);
    
    if(errors & I2C_MASTER_BUS_TIMEOUT_ERR)
    {
        CONSOLE_UART_Timestamp();
        CONSOLE_UART_Printf("I2C Bus Timeout\r\n");
    }
    
    if(errors & I2C_MASTER_TIMEOUT_ERR)
    {
        CONSOLE_UART_Timestamp();
        CONSOLE_UART_Printf("I2C Operation Timeout\r\n");
    }
    if(errors & I2C_MASTER_ERROR_ADDR_ACK)
    {
        CONSOLE_UART_Timestamp();
        CONSOLE_UART_Printf("I2C Address Ack Error\r\n");
    }
    
    if(errors & I2C_MASTER_ERROR_DATA_ACK)
    {
        CONSOLE_UART_Timestamp();
        CONSOLE_UART_Printf("I2C Data Ack Error\r\n");
    }
    if(errors & I2C_MASTER_ERROR_ARB_LOST)
    {
        CONSOLE_UART_Timestamp();
        CONSOLE_UART_Printf("I2C Arbitration Lost\r\n");    
    }
}

/*
 * command_i2c: perform I2C communications.
 */
void command_i2c(int argc, char **argv)
{
    unsigned char address;
    unsigned long i2c_base;
    const I2C_ENABLE *enable;
    const GateMutexPri_Handle *mutex;
    const Semaphore_Handle *semaphore;
    int base, size, i;
    char *strBuf, **iteratePtr;
    I2C_OP **operations, *opPtr;
    
    I2C_PORT custom_port = { 0, 0, 0, NULL, NULL, NULL };
    
    base = atoi(strtok_r(argv[1], ".", &strBuf));
    address = (unsigned char)strtoul(strtok_r(NULL, ".", &strBuf), (char**)NULL, 16);
    size = argc - 2;
    
    i2c_base = getI2CBase(base);
    enable = getI2CEnable(base, address);
    mutex = getI2CMutex(base);
    semaphore = getI2CSemaphore(base);
    
    custom_port.base = i2c_base;
    custom_port.address = address;
    custom_port.gateKey = NULL;
    custom_port.enable = enable;
    custom_port.mutex = mutex;
    custom_port.semaphore = semaphore;
    
    if(custom_port.enable == NULL || custom_port.mutex == NULL || custom_port.semaphore == NULL)
    {
        CONSOLE_UART_Timestamp();
        CONSOLE_UART_Printf("ERROR: Could not find specified i2c port\r\n");
        return;
    }
    
    operations = NULL;
    
    /* Need to add timeout */
    while(operations == NULL)
    {
        operations = (I2C_OP**)BMC_Calloc(size, sizeof(I2C_OP*), NULL);
        delay(1);
    }
    
    iteratePtr = &argv[2];
    for(i = 0; i < size; i++)
    {
        operations[i] = (I2C_OP*)BMC_Alloc(sizeof(I2C_OP), NULL);
        
        opPtr = operations[i];
        if(strcmp(*iteratePtr, "r") == 0)
        {
            opPtr->operand = I2C_READ;
            opPtr->data = 0;
        }
        else
        {
            opPtr->operand = I2C_WRITE;
            opPtr->data = (unsigned char)strtoul(*iteratePtr, (char**)NULL, 16);
        }
        opPtr->error = 0;
        iteratePtr++;
    }
    
    I2C_Start(&custom_port);
    I2C_Operation(operations, size, &custom_port);
    I2C_Stop(&custom_port);
    
    for(i = 0; i < size; i++)
    {
        i2cErrorHandle(operations[i]->error, i);
    }
    
    CONSOLE_UART_Timestamp();
    CONSOLE_UART_Printf("Read results: ");
    for(i = 0; i < size; i++)
    {
        if(operations[i]->operand == I2C_READ)
            CONSOLE_UART_Printf("0x%02X ", operations[i]->data);
        
        BMC_Free(operations[i], NULL);
    }
    CONSOLE_UART_Printf("\r\n");
    
    BMC_Free(operations, NULL);
}

/*
 * command: clkreg, reads or writes to specified register on clock
 */
void command_clkreg(int argc, char **argv)
{
    int clock;
    unsigned int cReg, clock_select;
    unsigned short data;
    char *strBuf;

    /* Get clock and register values */
    clock = atoi(strtok_r(argv[1], ".", &strBuf));
    cReg = (unsigned int) strtoul(strtok_r(NULL, ".", &strBuf), (char **) NULL, 0);

    /* Get chip select based on clock */
    switch(clock)
    {
    case 1:
        clock_select = CLOCK_GEN_1;
        break;
    case 2:
        clock_select = CLOCK_GEN_2;
        break;
    case 3:
        clock_select = CLOCK_GEN_3;
        break;
    default:
        return;
    }

    if(argv[2]) // write
    {
        /* Get the data to be written */
        data = (unsigned int) strtoul(argv[2], (char **) NULL, 0);
        
        CONSOLE_UART_Timestamp();
        CONSOLE_UART_Printf("Writing 0x%04X to register %i on clock module %i...\r\n", data, cReg, clock);

        /* Write data */
        CLOCK_RegWrite(clock_select, cReg, data);
        
        CONSOLE_UART_Timestamp();
        CONSOLE_UART_Printf("Finished\r\n");
    }
    else // read
    {
        CONSOLE_UART_Timestamp();
        CONSOLE_UART_Printf("Reading from register %i on clock module %i...\r\n", cReg, clock);
        
        /* Get Data */
        data = CLOCK_RegRead(clock_select, cReg);
        
        CONSOLE_UART_Timestamp();
        CONSOLE_UART_Printf("Data: 0x%04X\r\n", data);
    }
}

/*
 * command: readall, reads the value of every gpio pin and expander
 */
void command_readall(int argc, char **argv)
{
}

/*
 * command: EEPROM, writes and reads to eeprom
 *
 */
void command_eeprom(int argc, char **argv)
{
    int i, size;
    unsigned long usAddr;
    I2C_OP **operations, *opPtr;
    I2C_PORT *eeprom_port;
    
    usAddr = strtoul(argv[1], (char**) NULL, 16);

    if(usAddr > 0x1FFFF)
        return;
    
    eeprom_port = ((usAddr & 0x10000) > 0) ? getPort(EEPROM51) : getPort(EEPROM50);
    
    if(eeprom_port == NULL)
    {
        CONSOLE_UART_Timestamp();
        CONSOLE_UART_Printf("ERROR: Could not find EEPROM I2C\r\n");
        return;
    }
    
    size = argc - 2;
    operations = NULL;
    
    if(size > 0)
    {
        while(operations == NULL)
        {
            operations = (I2C_OP**)BMC_Calloc((size+2), sizeof(I2C_OP*), NULL);
            delay(1);
        }
        opPtr = operations[0];
        char **iteratePtr = &argv[2];
        
        operations[0] = (I2C_OP*)BMC_Alloc(sizeof(I2C_OP), NULL);
        operations[0]->operand = I2C_WRITE;
        operations[0]->data = (unsigned char)((usAddr & 0xFF00) >> 8);
        operations[0]->error = 0;
        
        operations[1] = (I2C_OP*)BMC_Alloc(sizeof(I2C_OP), NULL);
        operations[1]->operand = I2C_WRITE;
        operations[1]->data = (unsigned char)(usAddr & 0xFF);
        operations[1]->error = 0;
        
        for(i = 0; i < size; i++)
        {
            operations[i+2] = (I2C_OP*)BMC_Alloc(sizeof(I2C_OP), NULL);
            opPtr = operations[i+2];
            opPtr->operand = I2C_WRITE;
            opPtr->data = (unsigned char)strtoul(*iteratePtr++, (char**)NULL, 16);
            opPtr->error = 0;
        }
    }
    else
    {
        size = 16;
        operations = (I2C_OP**)BMC_Calloc((size+2), sizeof(I2C_OP*), NULL);
        
        operations[0] = (I2C_OP*)BMC_Alloc(sizeof(I2C_OP), NULL);
        operations[0]->operand = I2C_WRITE;
        operations[0]->data = (unsigned char)((usAddr & 0xFF00) >> 8);
        operations[0]->error = 0;
        
        operations[1] = (I2C_OP*)BMC_Alloc(sizeof(I2C_OP), NULL);
        operations[1]->operand = I2C_WRITE;
        operations[1]->data = (unsigned char)(usAddr & 0xFF);
        operations[1]->error = 0;
        
        for(i = 0; i < size; i++)
        {
            operations[i+2] = (I2C_OP*)BMC_Alloc(sizeof(I2C_OP), NULL);
            opPtr = operations[i+2];
            opPtr->operand = I2C_READ;
            opPtr->data = 0;
            opPtr->error = 0;
        }
    }
    
    I2C_Start(eeprom_port);
    I2C_Operation(operations, size+2, eeprom_port);
    I2C_Stop(eeprom_port);
    
    for(i = 0; i < size; i++)
    {
        i2cErrorHandle(operations[i]->error, i);
    }
    
    BMC_Free(operations[0], NULL);
    BMC_Free(operations[1], NULL);
    if(argc <= 2)
    {
        CONSOLE_UART_Timestamp();
        CONSOLE_UART_Printf("Results: ");
        for(i = 0; i < size; i++)
        {
            CONSOLE_UART_Printf("0x%02X ", operations[i+2]->data);
        }
        CONSOLE_UART_Printf("\r\n");
        BMC_Free(operations[i+2], NULL);
    }
    
    BMC_Free(operations, NULL);
}

/*
 * Displays information on various commands
 */
void command_help(int argc, char** argv)
{
    int i;
    const Command *cmdPtr;
    const char *strPtr;
    
    if(argc > 1)
    {
        cmdPtr = NULL;
        for(i = 0; i < total_commands; i++)
        {
            if(strcmp(argv[1], command_list[i].title) == 0)
            {
                cmdPtr = &command_list[i];
                break;
            }
        }
        if(cmdPtr == NULL)
        {
            CONSOLE_UART_Timestamp();
            CONSOLE_UART_Printf("Error - Command not recognized.\r\n");
            return;
        }
        if(cmdPtr->disabled)
        {
            CONSOLE_UART_Timestamp();
            CONSOLE_UART_Printf("Error - This command is disabled, please use hwdbg to enable the command\r\n");
            return;
        }
        
        CONSOLE_UART_Printf("\r\n");        
        for(i = 0; i < (int)strlen(cmdPtr->long_help); i+= UART_MAX_CHAR-1)
        {
            strPtr = &cmdPtr->long_help[i];
            CONSOLE_UART_Printf(strPtr);
        }
        
        CONSOLE_UART_Printf("\r\n\r\n");
        return;
    }
    
    CONSOLE_UART_Printf("\r\n");
    for(i = 0; i < total_commands; i++)
    {
        cmdPtr = &command_list[i];
        
        if(!cmdPtr->hidden)
        {
            CONSOLE_UART_Printf("%s - %s\r\n", cmdPtr->title, cmdPtr->sum_help);
        }
    }
    CONSOLE_UART_Printf("\r\n");
}

/*
 * Hardware Debug: controls various debug aspects
 */
void command_hwdbg(int argc, char** argv)
{
    CONSOLE_UART_Timestamp();
    if(strcmp(argv[1], "on") == 0)
    {
        CONSOLE_UART_Printf("Setting Hardware Debug Mode to ON\r\n");
        SetDebugMode(ON);
    }
    else if(strcmp(argv[1], "off") == 0)
    {
        CONSOLE_UART_Printf("Setting Hardware Debug Mode to OFF\r\n");
        SetDebugMode(OFF);
    }
    else if(strcmp(argv[1], "continue") == 0)
    {
        CONSOLE_UART_Printf("Setting Hardware Debug Mode to CONTINUE\r\n");
        SetDebugMode(CONTINUE);
    }
    else if(strcmp(argv[1], "polling") == 0)
    {
        if (argc < 2)
        {
            CONSOLE_UART_Timestamp();
            CONSOLE_UART_Printf("Error, incorrect use of hwdbg command. Type help hwdbg for proper syntax.\r\n");
            return;
        }
        if (strcmp(argv[2], "enable") == 0)
        {
            CONSOLE_UART_Timestamp();
            CONSOLE_UART_Printf("Enabling Polling\r\n");
            Semaphore_post(POLLING_sem);
        }
        else if(strcmp(argv[2], "disable") == 0)
        {
            CONSOLE_UART_Printf("Disabling Polling. WARNING - This will prevent proper shutdown in\r\n");
            CONSOLE_UART_Timestamp();
            CONSOLE_UART_Printf("the event of a power failure. This should only be done to update UCD modules.\r\n");
            
            Semaphore_pend(POLLING_sem, START_TIMEOUT);
            CONSOLE_UART_Timestamp();
            CONSOLE_UART_Printf("Polling disabled.\r\n");
        }
    }
    else if(strcmp(argv[1], "cmd") == 0)
    {
        Command *cmd;
        int i;
        if(argv[2])
        {
            if(strcmp(argv[2], "show") == 0)
            {
                CONSOLE_UART_Printf("Enabling and showing all commands\r\n");
                for(i = 0; i < total_commands; i++)
                {
                    cmd = &command_list[i];
                    CMD_enable(cmd);
                    CMD_unhide(cmd);
                }
            }
            else
            {
                for(i = 0; i < total_commands; i++)
                {
                    cmd = &command_list[i];
                    if(strcmp(argv[2], cmd->title) == 0)
                    {
                        CONSOLE_UART_Printf("Enabling command: %s\r\n", cmd->title);
                        CMD_enable(cmd);
                        if(strcmp(argv[3], "show") == 0)
                            CMD_unhide(cmd);
                        break;
                    }
                }
                if(i >= total_commands)
                    CONSOLE_UART_Printf("Command %s not recognized\r\n", argv[2]);
            }
        }
        else
        {
            /* Enable all commands, but do not unhide */
            CONSOLE_UART_Printf("Enabling all commands\r\n");
            for(i = 0; i < total_commands; i++)
            {
                cmd = &command_list[i];
                CMD_enable(cmd);
            }
        }
    }
    else
    {
        CONSOLE_UART_Printf("Hardware Debug Mode not recognized\r\n");
    }
}

/*
 * WP - used to easily view and control write protects
 */
void command_wp(int argc, char** argv)
{
    int i;
    unsigned long ports;
    char *wp_enabled = "ON";
    char *wp_disabled = "OFF";
    char *eeprom_wp, *nor_wp, *nand_wp;
    
    ports = GPIO_Start(NAND_WPz_PORT | EEPROM_WP_PORT | NOR_WP_PORT);
    
    if(argc > 1)
    {
        int enable = (strcmp(argv[1], "on") == 0);
        if(argc > 2)
        {            
            for(i = 2; i < argc; i++)
            {
                if(strcmp(argv[i], "eeprom") == 0)
                {
                    GPIO_RMW(EEPROM_WP_PORT, EEPROM_WP, (enable) ? EEPROM_WP : 0, ports);
                }
                else if(strcmp(argv[i], "nor") == 0)
                {
                    GPIO_RMW(NOR_WP_PORT, NOR_WP, (enable) ? NOR_WP : 0, ports);
                }
                else if(strcmp(argv[i], "nand") == 0)
                {
                    GPIO_RMW(NAND_WPz_PORT, NAND_WPz, (enable) ? 0 : NAND_WPz, ports);
                }
                else
                {
                    CONSOLE_UART_Timestamp();
                    CONSOLE_UART_Printf("Error - %s is not a recognized device\r\n", argv[i]);
                }
            }
        }
        else
        {
            if(enable)
            {
                GPIO_RMW(EEPROM_WP_PORT, EEPROM_WP, EEPROM_WP, ports);
                GPIO_RMW(NOR_WP_PORT, NOR_WP, NOR_WP, ports);
                GPIO_RMW(NAND_WPz_PORT, NAND_WPz, 0, ports);
            }
            else
            {
                GPIO_RMW(EEPROM_WP_PORT, EEPROM_WP, 0, ports);
                GPIO_RMW(NOR_WP_PORT, NOR_WP, 0, ports);
                GPIO_RMW(NAND_WPz_PORT, NAND_WPz, NAND_WPz, ports);
            }
        }
    }
    eeprom_wp = (GPIO_Get(EEPROM_WP_PORT, ports) & EEPROM_WP) ? wp_enabled : wp_disabled;
    nor_wp = (GPIO_Get(NOR_WP_PORT, ports) & NOR_WP) ? wp_enabled : wp_disabled;
    nand_wp = (GPIO_Get(NAND_WPz_PORT, ports) & NAND_WPz) ? wp_disabled : wp_enabled;
    
    CONSOLE_UART_Timestamp();
    CONSOLE_UART_Printf("Current WP settings:\r\n");
    CONSOLE_UART_Timestamp();
    CONSOLE_UART_Printf("EEPROM WP: %s\r\n", eeprom_wp);
    CONSOLE_UART_Timestamp();
    CONSOLE_UART_Printf("NOR WP: %s\r\n", nor_wp);
    CONSOLE_UART_Timestamp();
    CONSOLE_UART_Printf("NAND WP: %s\r\n", nand_wp);
    
    GPIO_Update(ports);
}

/*
 * Local function used by reboot and shutdown to change the 
 * goal state of the State Machine.
 */
static void bootChange(int argc, char** argv, tBoolean reboot)
{
    int i;
    enum BOOT_TYPE stateChange = (reboot) ? FULL : PWR;
    tBoolean graceful = true;
    SOC *soc;
    
    if(argc > 1)
    {
        if(strcmp(argv[1], "warm") == 0)
            stateChange = WARM;
        else if(strcmp(argv[1], "por") == 0)
            stateChange = POR;
        else if (strcmp(argv[1], "pwr") == 0)
            stateChange = PWR;
        
        if(argc > 2)
            if(strcmp(argv[2], "force") == 0)
                graceful = false;
    }
    
    for(i = 0; i < num_socs; i++)
    {
        soc = &soc_list[i];
        Semaphore_pend(*soc->state_sem, BIOS_WAIT_FOREVER);
        
        soc->graceful = graceful;
        soc->reboot = reboot;
        if(stateChange == WARM)
            soc->warm = true;
        else
            soc->warm = false;
        
        if(stateChange == WARM || stateChange == FULL)
            soc->goal_state = SOC_POWER_ON;
        else if(stateChange == POR)
            soc->goal_state = SOC_POWER_OFF;
        else
            soc->goal_state = BOARD_STATE;
        
        Semaphore_post(*soc->state_sem);
    }
    SetReboot(reboot);
    if(stateChange == PWR)
    {
        SetBoardGoalState(MAIN_POWER_OFF);
    }
    else    
        Semaphore_post(STATE_MACHINE_sem);
}

/*
 * Reboot - places the system into the specified state
 * and then brings it back to the "running" state.
 */
void command_reboot(int argc, char** argv)
{
    Semaphore_pend(POLLING_sem, START_TIMEOUT);
    bootChange(argc, argv, true);
}

/*
 * Shudown - places the system/soc in the specified
 * shutdown state.
 */
void command_shutdown(int argc, char** argv)
{
    Semaphore_pend(POLLING_sem, START_TIMEOUT);
    bootChange(argc, argv, false);
}

/*
 * Run - brings the system from wherever it currently is
 * to the running state.
 */
void command_run(int argc, char** argv)
{
    int i;
    SOC *soc;
    
    for(i = 0; i < num_socs; i++)
    {
        soc = &soc_list[i];
        SetSOCGoalState(soc, RUNNING);
    }
    
    SetBoardGoalState(SOC_STATE);
}

/*
 * Status - displays the status of the system in a formatted
 * output.
 */
void command_status(int argc, char** argv)
{
    const char *board_state, *soc_state;
    char dipSW[5], SOC_request[3];
    unsigned long ports;
    unsigned short dipSW_raw, SOC_req_raw;
    
    switch(GetBoardState())
    {
        case INIT:
            board_state = STATE_STRS[0];
            break;
        case MAIN_POWER_OFF:
            board_state = STATE_STRS[1];
            break;
        case MAIN_POWER_ON:
            board_state = STATE_STRS[2];
            break;
        case SOC_STATE:
            board_state = STATE_STRS[3];
            break;
        case ERROR:
            board_state = STATE_STRS[4];
            break;
    }
    switch(soc_list[0].current_state)
    {
        case BOARD_STATE:
            soc_state = STATE_STRS[5];
            break;
        case SOC_POWER_OFF:
            soc_state = STATE_STRS[6];
            break;
        case SOC_POWER_ON:
            soc_state = STATE_STRS[7];
            break;
        case RUNNING:
            soc_state = STATE_STRS[8];
            break;
        case SOC_ERROR:
            soc_state = STATE_STRS[9];
            break;
        case RESET_ERROR:
            soc_state = STATE_STRS[10];
            break;
    }
    
    ports = GPIO_Start(DIP_SW_PORT | SOC_LOW_GPIO_PORT);
    
    dipSW_raw = GPIO_Get(DIP_SW_PORT, ports) & DIP_SW_PINS;
    SOC_req_raw = GPIO_Get(SOC_LOW_GPIO_PORT, ports) & (SOC_GPIO_06 | SOC_GPIO_07);
    
    dipSW[0] = ((dipSW_raw & DIP_SW_B0) ? '0' : '1');
    dipSW[1] = ((dipSW_raw & DIP_SW_B1) ? '0' : '1');
    dipSW[2] = ((dipSW_raw & DIP_SW_B2) ? '0' : '1');
    dipSW[3] = ((dipSW_raw & DIP_SW_B3) ? '0' : '1');
    dipSW[4] = NULL;
    
    SOC_request[0] = ((SOC_req_raw & SOC_GPIO_06) ? '1' : '0');
    SOC_request[1] = ((SOC_req_raw & SOC_GPIO_07) ? '1' : '0');
    SOC_request[2] = NULL;
    
    CONSOLE_UART_Timestamp();
    CONSOLE_UART_Printf("Board State      SOC State    DIP SW    SOC GPIO 6 & 7\r\n");
    CONSOLE_UART_Timestamp();
    CONSOLE_UART_Printf("%-14s %-15s %-13s %s\r\n", board_state, soc_state, dipSW, SOC_request);
}

/*
 * BootMode - used to display and set variouse aspects of the bootmode
 */
void command_bootmode(int argc, char** argv)
{
    BOOTMODE mode;
    int mode_index, i;
    tBoolean nonvolatile;
    if (argc < 2)
    {
        //output the current bootmode
        getCurrentBootMode(&mode);
        CONSOLE_UART_Timestamp();
        CONSOLE_UART_Printf("Current Bootmode is 0x%08X%08X   %s\r\n", mode.hi_val, mode.lo_val, mode.title);
    }
    else if(strcmp(argv[1], "all") == 0)
    {
        //output all bootmodes and note the current bootmode
        for(i = 0; i < 16; i++)
        {
            CONSOLE_UART_Timestamp();
            if(getBootModeByIndex(i, &mode))
            {                
                CONSOLE_UART_Printf("%2i 0x%08X%08X   %s......Current Bootmode\r\n", i, mode.hi_val, mode.lo_val, mode.title);
            }
            else
            {
                CONSOLE_UART_Printf("%2i 0x%08X%08X   %s\r\n", i, mode.hi_val, mode.lo_val, mode.title);
            }
        }
    }
    else if(strcmp(argv[1], "read") == 0)
    {
        //re-read the DIP and set the bootmode to the correct value
        resetBootMode();
    }
    else if(*argv[1] == '#' && argc < 3)
    {
        //Set the current bootmode the number directly following x
        mode_index = atoi(&(argv[1][1]));
        CONSOLE_UART_Timestamp();
        getBootModeByIndex(mode_index, &mode);
        CONSOLE_UART_Printf("Changing to bootmode %d: %s\r\n", mode_index, mode.title);
        setBootModeByIndex(mode_index);
    }
    else
    {        
        mode.hi_val = strtoul(argv[2], (char**)NULL, 16);
        mode.lo_val = strtoul(argv[3], (char**)NULL, 16);
        
        memset(mode.title, NULL, MAX_TITLE_SIZE);        
        for(i = 4; i < argc; i++)
        {
            strcat(mode.title, argv[i]);
            strcat(mode.title, " ");
        }
        
        if(*argv[1] == '#')
        {
            nonvolatile = true;
            mode_index = atoi(&(argv[1][1]));
        }
        else
        {
            nonvolatile = false;
            mode_index = atoi(argv[1]);
        }
        
        if(mode_index < 8 || mode_index > 15)
        {
            CONSOLE_UART_Timestamp();
            CONSOLE_UART_Printf("Error: Bootmode of %i read. Only values of 8 - 15 may be used by this command.\r\n", mode_index);
            return;
        }
        
        CONSOLE_UART_Timestamp();
        if(nonvolatile)
            CONSOLE_UART_Printf("Saving User Bootmode %d to 0x%08X%08X %s, this change is non-volatile\r\n", mode_index, mode.hi_val, mode.lo_val, mode.title);
        else
            CONSOLE_UART_Printf("Changing User Bootmode %d to 0x%08X%08X %s, this change is volatile\r\n", mode_index, mode.hi_val, mode.lo_val, mode.title);
        
        saveBootModeByIndex(mode_index, &mode, nonvolatile);
    }
}

/*
 * prog_flash_reg: local function used to program flash registers
 */
static int prog_flash_reg(unsigned long data, unsigned long reg, unsigned long address)
{
    int timeout = 0;
    
    HWREG(reg) = data;
    HWREG(FLASH_FMA) = address;
    HWREG(FLASH_FMC) |= FLASH_FMC_WRKEY | FLASH_FMC_COMT;
    while((HWREG(FLASH_FMC) & FLASH_FMC_COMT) && timeout++ < 50000);
    if(timeout >= 50000) return -1;
    return 0;
}

/*
 * Commission - used to store board information and setup in-field updates
 */
void command_commission(int argc, char** argv)
{
    int i;
    unsigned long board_type, board_version, board_sn, board_bootcfg;
    unsigned char temp[4];
    char *strBuf, *tempStr;
    char temp1[20], temp2[20];
    const EVM_TYPE *evm_type = NULL;
    
    if(!(HWREG(BOARD_TYPE_REG) & FLASH_USERREG0_NW))
    {
        CONSOLE_UART_Timestamp();
        CONSOLE_UART_Printf("EVM Board Type has already been programmed. Skipping.\r\n");
    }
    else if (argc > 1)
    {
        for(i = 0; i < num_evm_types; i++)
        {
            strncpy(temp1, evm_types_list[i].specific_name, 20);
            convertLower(temp1);
            strncpy(temp2, evm_types_list[i].id_str, 20);
            convertLower(temp2);
            if((strcmp(argv[1], temp1) == 0) ||
               (strcmp(argv[1], temp2) == 0))
            {
                evm_type = &evm_types_list[i];
                board_type = (unsigned long) i;
                break;
            }
        }
            
        if(evm_type == NULL)
            board_type = ~((unsigned long)0);
        
        prog_flash_reg(board_type, BOARD_TYPE_REG, BOARD_TYPE_ADDR);
        
        CONSOLE_UART_Timestamp();
        CONSOLE_UART_Printf("Successfully programmed EVM Baord Type.\r\n");
    }
    
    if(!(HWREG(BOARD_VER_REG) & FLASH_USERREG1_NW))
    {
        CONSOLE_UART_Timestamp();
        CONSOLE_UART_Printf("EVM Board Version has already been programmed. Skipping.\r\n");
    }
    else if (argc > 2)
    {
        memset(temp, NULL, 4);
        i = 1;
        temp[0] = (unsigned char) strtoul(strtok_r(argv[2], ".", &strBuf), NULL, 10);
        while(tempStr = strtok_r(NULL, ".", &strBuf))
            temp[i++] = (unsigned char) strtoul(tempStr, NULL, 10);
        
        board_version = (((unsigned long)temp[0]) << 0x18) |
                        (((unsigned long)temp[1]) << 0x10) |
                        (((unsigned long)temp[2]) << 0x08) |
                        (((unsigned long)temp[3]) << 0x00);
        
        prog_flash_reg(board_version, BOARD_VER_REG, BOARD_VER_ADDR);
        
        CONSOLE_UART_Timestamp();
        CONSOLE_UART_Printf("Successfully programmed EVM Board Version.\r\n");
    }
    
    if(!(HWREG(BOARD_SN_REG) & FLASH_USERREG2_NW))
    {
        CONSOLE_UART_Timestamp();
        CONSOLE_UART_Printf("EVM Board S/N has already been programmed. Skipping.\r\n");
    }
    else if (argc > 3)
    {
        board_sn = strtoul(argv[3], NULL, 10);
        prog_flash_reg(board_sn, BOARD_SN_REG, BOARD_SN_ADDR);
        
        CONSOLE_UART_Timestamp();
        CONSOLE_UART_Printf("Successfully programmed EVM Serial Number.\r\n");
    }
    
    if(!(HWREG(FLASH_BOOTCFG) & FLASH_BOOTCFG_NW))
    {
        CONSOLE_UART_Timestamp();
        CONSOLE_UART_Printf("In-field updating has already been enabled. Skipping.\r\n");
    }
    else
    {
        board_bootcfg = BOOTCFG_PORT | BOOTCFG_PIN | BOOTCFG_POL | FLASH_BOOTCFG_DBG1;
        prog_flash_reg(board_bootcfg, FLASH_FMD, BOOTCFG_ADDR);
        
        CONSOLE_UART_Timestamp();
        CONSOLE_UART_Printf("Successfully enabled in-field updating.\r\n");
    }
    
    CONSOLE_UART_Timestamp();
    CONSOLE_UART_Printf("Please power cycle the board for these changes to take affect.\r\n");
}

void UCD_ver(char *ucd_ver)
{
    const int size = 12;
    int i;
    I2C_OP **operations;
    I2C_PORT *pm_bus;
    
    pm_bus = getPort(UCD_9090);
    
    if(pm_bus == NULL)
    {
        return;
    }
    
    operations = NULL;
    while(operations == NULL)
    {
        operations = (I2C_OP**)BMC_Calloc((size), sizeof(I2C_OP*), NULL);
        delay(1);
    }
    
    operations[0] = (I2C_OP*)BMC_Alloc(sizeof(I2C_OP), NULL);
    operations[0]->operand = I2C_WRITE;
    operations[0]->data = 0x9B;
    operations[0]->error = 0;
    
    for(i = 1; i < size; i++)
    {
        operations[i] = (I2C_OP*)BMC_Alloc(sizeof(I2C_OP), NULL);
        operations[i]->operand = I2C_READ;
        operations[i]->data = 0;
        operations[i]->error = 0;
    }
    
    I2C_Start(pm_bus);
    I2C_Operation(operations, size, pm_bus);
    I2C_Stop(pm_bus);
    
    for(i = 6; i < size; i++)
    {
        if(operations[i]->error == I2C_MASTER_ERROR_NONE)
        {
            if(operations[i]->data > 0x1F && operations[i]->data < 0x7F)
            ucd_ver[i-6] = operations[i]->data;
        }
    }
    
    for(i = 0; i < size; i++)
        BMC_Free(operations[i], NULL);
    
    BMC_Free(operations, NULL);
}

/*
 * Ver - displays version information.
 */
void command_ver(int argc, char** argv)
{
    unsigned long board_type, board_version, board_sn;
    unsigned char verChars[4];
    char version_str[20];
    char ucd_ver[10];
    const EVM_TYPE *evm_type;
    
    memset(ucd_ver, NULL, 10);
    
    EVM_TYPE type = {
        "X.X.X.X",
        "UNKNOWN",
        "UNKNOWN"
    };
    
    board_type = HWREG(BOARD_TYPE_REG);
    if(board_type < num_evm_types)
        evm_type = &evm_types_list[board_type];
    else
        evm_type = &type;
    
    board_version = HWREG(BOARD_VER_REG);
    verChars[0] = (unsigned char) ((board_version & 0xFF000000) >> 0x18);
    verChars[1] = (unsigned char) ((board_version & 0x00FF0000) >> 0x10);
    verChars[2] = (unsigned char) ((board_version & 0x0000FF00) >> 0x08);
    verChars[3] = (unsigned char) ((board_version & 0x000000FF) >> 0x00);
    
    if(verChars[3] == 0)
    {
        if(verChars[2] == 0)
        {
            snprintf(version_str, 20, "%u.%u", verChars[0], verChars[1]);
        }
        else
        {
            snprintf(version_str, 20, "%u.%u.%u", verChars[0], verChars[1], verChars[2]);
        }
    }
    else
    {
        snprintf(version_str, 20, "%u.%u.%u.%u", verChars[0], verChars[1], verChars[2], verChars[3]);
    }
    
    board_sn = HWREG(BOARD_SN_REG);
    UCD_ver(ucd_ver);
    
    CONSOLE_UART_Timestamp();
    CONSOLE_UART_Printf("\r\n");
    CONSOLE_UART_Timestamp();
    CONSOLE_UART_Printf("BMC Version    EVM Type    EVM Superset    EVM Subset    EVM Version    EVM S/N        UCD Ver\r\n");
    CONSOLE_UART_Timestamp();
    CONSOLE_UART_Printf("%-14.14s %-11.11s %-15.15s %-13.13s %-14.14s %-14u %s\r\n", stringVer, evm_type->id_str,
        evm_type->generic_name, evm_type->specific_name, version_str, board_sn, ucd_ver);
    CONSOLE_UART_Timestamp();
    CONSOLE_UART_Printf("\r\n");
}

/*
 * PCIE - enable or disable the PCIE mux
 */
void command_pcie(int argc, char** argv)
{
    unsigned long ports;
    unsigned char ucSelect;
    
    ports = GPIO_Start(PCIECLK_MUX_SEL_PORT);
    
    // Get the current select
    ucSelect = (GPIO_Get(PCIECLK_MUX_SEL_PORT, ports) & PCIECLK_MUX_SEL) ? 1 : 0;
    
    CONSOLE_UART_Timestamp();
    CONSOLE_UART_Printf("The PCIE CLK Mux select is currently %u\r\n", ucSelect);
    
    if(argc < 2)
        return;
    
    // Get arguments
    ucSelect = (unsigned char)atoi(argv[1]);
    
    CONSOLE_UART_Timestamp();
    CONSOLE_UART_Printf("Setting select to %u\r\n", ucSelect);
    
    GPIO_RMW(PCIECLK_MUX_SEL_PORT, PCIECLK_MUX_SEL, (ucSelect ? PCIECLK_MUX_SEL : 0), ports);
    GPIO_Update(ports);
}
