/******************************************************************************
 *
 * File	Name:       bmc_commands.h
 *
 * Description: This contains initialization of variables in command.
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

#ifndef BMC_COMMAND_H_
#define BMC_COMMAND_H_

#include "inc/hw_types.h"

typedef struct Command
{
    const char *title;                      // Command name
    void (*const function_call)(int, char**);     // Function to call for the command
    const char *sum_help;                   // A short summary of the command
    const char *long_help;                  // A long description of the command
    tBoolean hidden;
    tBoolean disabled;
} Command;

extern const int total_commands;
extern Command command_list[];

/* Command functions */
extern void command_lcd(int, char**);
extern void command_wait(int, char**);
extern void command_gpio(int, char**);
extern void command_spi(int, char**);
extern void command_i2c(int, char**);
extern void command_clkreg(int, char**);
extern void command_readall(int, char**);
extern void command_eeprom(int, char**);
extern void command_help(int, char**);
extern void command_hwdbg(int, char**);
extern void command_wp(int, char**);
extern void command_reboot(int, char**);
extern void command_shutdown(int, char**);
extern void command_run(int, char**);
extern void command_status(int, char**);
extern void command_bootmode(int, char**);
extern void command_commission(int, char**);
extern void command_ver(int, char**);
extern void command_pcie(int, char**);

#endif
