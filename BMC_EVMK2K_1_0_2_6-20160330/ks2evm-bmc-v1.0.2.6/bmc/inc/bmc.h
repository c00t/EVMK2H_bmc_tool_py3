/******************************************************************************
 *
 * File	Name:       bmc.h
 *
 * Description: This contains general defines and includes for bmc.c
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

#ifndef BMC_H_
#define BMC_H_

#include <ti/sysbios/BIOS.h>
#include "bmc_commands.h"

/* ========== Defines ========== */

//Version
#define version      01.00.02.06
#define stringVer    "1.0.2.6"

//Values:
#define GPIOE_SPI_SPEED 4000000               // GPIO Expander Cock rate
#define LCD_SPI_SPEED   10000000              // LCD Clock rate
#define BAUD_RATE       115200                // Baud rate for UART
#define MAX_ARGS        50                    // Maximum number of arguments
#define TIMEOUT         30000                 // Timeout value for i2c
#define START_TIMEOUT   50                    // Timeout for Startup

/* ================================= Function Prototypes ================================= */
extern void delay(unsigned long mSec);
extern void CMD_enable(Command *cmd);
extern void CMD_disable(Command *cmd);
extern char *strtok_r(char *s, const char *delim, char **last);
extern void progBootCFG();
extern void setBootCFG(unsigned int bootMode);
extern void Error_Control(int enable);

#endif /* BMC_H_ */
