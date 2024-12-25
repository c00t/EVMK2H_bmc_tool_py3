/******************************************************************************
 *
 * File	Name:       bmc_state.h
 *
 * Description: This contains includes, defines, and function prototypes for 
 *              BMC STATE modules.
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

#ifndef BMC_STATE_H_
#define BMC_STATE_H_

/* ===================== Includes ================== */
#include "bmc_map.h"
#include <ti/sysbios/knl/Semaphore.h>

/* ============================= Defines ============================= */
#define NUM_USER_BOOTMODES  8
#define MAX_TITLE_SIZE      32

enum BOARD_STATE { INIT, MAIN_POWER_OFF, MAIN_POWER_ON, SOC_STATE, ERROR };
enum SOC_STATE { BOARD_STATE, SOC_POWER_OFF, SOC_POWER_ON, RUNNING, SOC_ERROR, RESET_ERROR };
enum DEBUG_MODE { ON, OFF, HWDBG, CONTINUE };

typedef struct BOOTMODE {
    unsigned int hi_val;
    unsigned int lo_val;
    char title[MAX_TITLE_SIZE];
} BOOTMODE;

typedef struct SOC {
    BOOTMODE boot_mode;
    enum SOC_STATE current_state;
    enum SOC_STATE goal_state;
    tBoolean warm;
    tBoolean graceful;
    tBoolean reboot;
    const unsigned char i2c_key;
    const Semaphore_Handle *state_sem;
} SOC;

extern const int num_user_bootmodes;
extern const int num_socs;
extern SOC soc_list[];

/* ==================================== Function Prototypes ================================= */
extern void SetBoardGoalState(enum BOARD_STATE new_state);
extern void SetSOCGoalState(SOC *soc, enum SOC_STATE new_state);
extern enum BOARD_STATE GetBoardState();
extern void SetGracefulMode(tBoolean new_mode);
extern void SetReboot(tBoolean reboot);
extern void SetDebugMode(enum DEBUG_MODE mode);
extern enum DEBUG_MODE GetDebugMode();
extern unsigned char getDIPSwitch();
extern tBoolean getBootModeByIndex(int index, BOOTMODE *mode);
extern void resetBootMode();
extern void setBootModeByIndex(int index);
extern void saveBootModeByIndex(int index, BOOTMODE *mode, tBoolean permanent);
extern void getCurrentBootMode(BOOTMODE *mode);

#endif
