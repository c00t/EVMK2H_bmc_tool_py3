/******************************************************************************
 *
 * File	Name:       bmc_uart.h
 *
 * Description: This contains includes, defines, and function prototypes for 
 *              BMC UART modules.
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

#ifndef BMC_UART_H_
#define BMC_UART_H_

/* ===================== Includes ================== */
#include "bmc_map.h"
#include "bmc.h"

/* ============================= Defines ============================= */
#define CONSOLE_UART_SELECT     0x01                    // Selects the console UART
#define SOC_UART_SELECT         0x02                    // Selects the SOC UART

#define MAX_SIZE        128                             // Command size
#define MAX_CMD         10

/* Values */
#define UART_MAX_CHAR          128                     // Maximum characters per print for uart
#define UART_PROMPT            "BMC>"                  // BMC prompt

/* Interrupt Handles */
#define UART_NO_HANDLE         0x00                    // No handle (disable UART interrupts)
#define UART_CMD_HANDLE        0x01                    // Command Line UART interrupt handle

/* ==================================== Function Prototypes ================================= */
extern void CONSOLE_UART_Setup();
extern void CONSOLE_UART_Printf(const char *format, ...);
extern void CONSOLE_UART_Timestamp();
extern void CONSOLE_UART_EnableRX();
extern void CONSOLE_UART_DisableRX();

extern void SOC_UART_Setup();
extern void SOC_UART_EnableRX();
extern void SOC_UART_DisableRX();

#endif /* BMC_UART_H_ */
