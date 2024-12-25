/******************************************************************************
 *
 * File	Name:       bmc_state.c
 *
 * Description: This file contains the automated startup task for BMC.
 *
 * Currently contains EVM specific code, consider macro of GPIOE pins.
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

#include <string.h>
#include <stdarg.h>

#include "bmc_state.h"
#include "bmc.h"

#include "inc/hw_flash.h"

#include "driverlib/flash.h"

#include "bmc_uart.h"
#include "bmc_lcd.h"
#include "bmc_clocks.h"
#include "bmc_commands.h"
#include "evm_types.h"

#include "gpio_driver.h"

#define SETUP_TIMEOUT           500                     // Represents 5 seconds, poll GPIO pin every 1 ms

const int num_user_bootmodes = 8;

static unsigned long PLL_LOCK_PORT[3] =
{
    PLL_LOCK1_PORT,
    PLL_LOCK2_PORT,
    PLL_LOCK3_PORT
};

static unsigned short PLL_LOCK[3] =
{
    PLL_LOCK1,
    PLL_LOCK2,
    PLL_LOCK3
};

static BOOTMODE BOOT_MODES[16] =
{                                                                // DIP
    {0x00000000, 0x00110CE7, "ARM NAND"},                        // 0000
    {0x00000000, 0x00100001, "DSP No-Boot"},                     // 0001
    {0x00000000, 0x00112005, "ARM SPI"},                         // 0010
    {0x00000000, 0x00100003, "ARM I2C"},                         // 0011
    {0x00000000, 0x00100CEF, "ARM UART"},                        // 0100
    {0x00000000, 0x00115EEB, "ARM RBL ENET"},                    // 0101
    {0x00000000, 0x001010E1, "SLEEP W/ MAX PLL & ARM BYPASS"},   // 0110
    {0x00000000, 0x00103EE1, "SLEEP W/ MAX PLL"},                // 0111
    // USER MODE BOOTMODES
    {0x00000000, 0x001101E7, "DSP NAND"},                        // 1000
    {0x00000000, 0x001010C1, "SLEEP W/ SLOW PLL & ARM BYPASS"},  // 1001
    {0x00000000, 0x00112105, "DSP SPI"},                         // 1010
    {0x00000000, 0x00100103, "DSP I2C"},                         // 1011
    {0x00000000, 0x20112005, "ARM SPI 10MHz"},                   // 1100
    {0x00000000, 0x001111EB, "DSP RBL ENET"},                    // 1101
    {0x00000000, 0x00103CC1, "SLEEP W/ SLOW PLL & SLOW ARM PLL"},// 1110
    {0x00000000, 0x00100DEF, "DSP UART"}                         // 1111
};

static const unsigned long BOOTMODE_LOCATION = BOOT_LOCATION;
static const unsigned long CHECK_BYTE_OFFSET = CHECK_OFFSET;

static const char *const ERROR_MESSAGE = "EVM setup has encountered the following error: %s\r\n";
static const char *const CHANGE_MESSAGE = "To alter this behavior, please use the hwdbg command.\r\n";
static const char *const HWDBG_ON = "HWDBG is set to ON, evm setup will now halt.\r\n";
static const char *const HWDBG_OFF = "HWDBG is set to OFF, evm setup will attempt to re-execute the previous step.\r\n";
static const char *const HWDBG_CONT = "HWDBG is set to CONTINUE, evm setup will continue despite this error.\r\n";

static const char *const MAIN_POWER_FAIL_MESSAGE = "The signal MAIN_POWER_GOOD has failed to assert";
static const char *const SOC_POWER_FAIL_MESSAGE = "The signal SOC_POWER_GOOD has failed to assert";
static const char *const CLK_GEN_SETUP_FAIL_MESSAGE = "The PLL_LOCK signal from one or more clock generators failed to assert";
static const char *const SOC_RESET_FAIL_MESSAGE = "The signal SOC_RESETSTATz has failed to assert";

/*=============== External Variables =================*/
extern Semaphore_Handle STATE_MACHINE_sem;
extern Semaphore_Handle BOARD_GOAL_sem;
extern Semaphore_Handle GRACEFUL_sem;
extern Semaphore_Handle REBOOT_sem;
extern Semaphore_Handle DEBUG_MODE_sem;
extern Semaphore_Handle POLLING_sem;

/*=================== Local Variables ==================*/
static enum BOARD_STATE current_board_state = INIT;
static enum BOARD_STATE goal_board_state = SOC_STATE;
static tBoolean g_graceful = true;
static tBoolean g_reboot = false;
static enum DEBUG_MODE g_dbgMode = ON;
static BOOTMODE g_soc_bootmode;


/*==================== Local Functions ========================*/
static void setBoardCurrentState(enum BOARD_STATE new_state);
static void setSOCCurrentState(SOC *soc, enum SOC_STATE new_state);

static void bmc_init();
static tBoolean mainPowerStart();
static void mainPowerStop(tBoolean graceful);

static int socStateMachine();
static tBoolean checkSocStates();
static tBoolean socPowerStart(SOC *soc);
static void socPowerStop(SOC *soc, tBoolean graceful);
static tBoolean socOutOfRST(SOC *soc);

static void socWarmReset(SOC *soc);
static void socFullReset(SOC *soc);
static void socPOR(SOC *soc);

static void reset_gpio_expanders();
static BOOTMODE get_bootmode();
static void program_bootmode(SOC *soc);
static void flash_bootmode(BOOTMODE *modeStart);
void alter_bootmode(int mode, BOOTMODE *newMode);
void read_bootmode(int mode, BOOTMODE *modePtr);

static void Setup_GPIOInit();
static void Setup_GPIOMainPwr(tBoolean);
static tBoolean Setup_ClkGen(tBoolean);
static void Setup_GPIOSOCPwr(tBoolean);
static void Setup_GPIORST(tBoolean);

static void writeBootModeToFlash(BOOTMODE*, unsigned long);
static void readBootModeFromFlash(BOOTMODE*, unsigned long);

static void console_printf(int lPage, int lLine, const char *format, ...);

/*
 * The State Machine for the EVM. Performs startup by default.
 */
void State_Machine()
{
    /* Delay to let other tasks initialize */
    delay(10);

    while(1)
    {
        if(current_board_state == goal_board_state && (current_board_state != SOC_STATE || checkSocStates()))
        {
            if(g_reboot)
            {
                SetBoardGoalState(SOC_STATE);
                SetReboot(false);
            }

            Semaphore_pend(STATE_MACHINE_sem, BIOS_WAIT_FOREVER);
        }
        Semaphore_pend(BOARD_GOAL_sem, BIOS_WAIT_FOREVER);
        switch(current_board_state)
        {
            case INIT:
                bmc_init();
                setBoardCurrentState(MAIN_POWER_OFF);
                break;
            case MAIN_POWER_OFF:
                if(goal_board_state == MAIN_POWER_OFF)
                    break;
                if(mainPowerStart())
                {
                    setBoardCurrentState(MAIN_POWER_ON);
                }
                else
                {
                    Semaphore_pend(DEBUG_MODE_sem, BIOS_WAIT_FOREVER);

                    CONSOLE_UART_Timestamp();
                    CONSOLE_UART_Printf(ERROR_MESSAGE, MAIN_POWER_FAIL_MESSAGE);

                    if(g_dbgMode != HWDBG)
                        mainPowerStop(false);
                    if(g_dbgMode != OFF)
                    {
                        CONSOLE_UART_Timestamp();
                        CONSOLE_UART_Printf(HWDBG_ON);
                        CONSOLE_UART_Timestamp();
                        CONSOLE_UART_Printf(CHANGE_MESSAGE);
                        setBoardCurrentState(ERROR);
                        goal_board_state = ERROR;
                    }
                    else
                    {
                        CONSOLE_UART_Timestamp();
                        CONSOLE_UART_Printf(HWDBG_OFF);
                        CONSOLE_UART_Timestamp();
                        CONSOLE_UART_Printf(CHANGE_MESSAGE);
                    }

                    LCD_Printf(SOC_STATE, 0, "MAIN POWER");
                    LCD_Printf(SOC_STATE, 1, "FAILURE");

                    delay(1000);

                    Semaphore_post(DEBUG_MODE_sem);
                }
                break;
            case MAIN_POWER_ON:
                if(goal_board_state == MAIN_POWER_OFF)
                {
                    Semaphore_pend(GRACEFUL_sem, BIOS_WAIT_FOREVER);
                    mainPowerStop(g_graceful);
                    Semaphore_post(GRACEFUL_sem);
                    setBoardCurrentState(MAIN_POWER_OFF);
                }
                else if(goal_board_state == SOC_STATE)
                {
                    setBoardCurrentState(SOC_STATE);
                }
                break;
            case SOC_STATE:
                switch(socStateMachine())
                {
                    case 0:
                        setBoardCurrentState(MAIN_POWER_ON);
                        break;
                    case 1:
                        Semaphore_pend(DEBUG_MODE_sem, BIOS_WAIT_FOREVER);

                        CONSOLE_UART_Timestamp();
                        CONSOLE_UART_Printf("Error Detected\r\n");

                        if(g_dbgMode != HWDBG)
                            mainPowerStop(false);
                        if(g_dbgMode != OFF)
                        {
                            setBoardCurrentState(ERROR);
                        }
                        else
                        {
                            setBoardCurrentState(MAIN_POWER_OFF);
                        }

                        Semaphore_post(DEBUG_MODE_sem);
                        break;
                    default:
                        break;
                }
                break;
            case ERROR:
                if(goal_board_state != ERROR)
                {
                    setBoardCurrentState(MAIN_POWER_OFF);
                }
                break;
        }
        Semaphore_post(BOARD_GOAL_sem);
    }
}

/*
 * The SoC state machine. Cycles through each SoC on the system and performs startup tasks.
 */
static int socStateMachine()
{
    int i, board_state_count;
    SOC *soc;

    board_state_count = 0;
    for(i = 0; i < num_socs; i++)
    {
        soc = &soc_list[i];
        Semaphore_pend(*soc->state_sem, BIOS_WAIT_FOREVER);
        if(soc->current_state == soc->goal_state)
        {
            if(soc->current_state == BOARD_STATE)
                board_state_count++;

            Semaphore_post(*soc->state_sem);
            continue;
        }
        switch(soc->current_state)
        {
            case BOARD_STATE:
                soc->boot_mode = g_soc_bootmode;
                setSOCCurrentState(soc, SOC_POWER_OFF);
                break;
            case SOC_POWER_OFF:
                if(soc->goal_state != BOARD_STATE)
                {
                    if(socPowerStart(soc))
                    {
                        setSOCCurrentState(soc, SOC_POWER_ON);
                    }
                    else
                    {
                        Semaphore_pend(DEBUG_MODE_sem, BIOS_WAIT_FOREVER);

                        CONSOLE_UART_Timestamp();
                        CONSOLE_UART_Printf(ERROR_MESSAGE, SOC_POWER_FAIL_MESSAGE);

                        if(g_dbgMode != HWDBG)
                            socPowerStop(soc, false);
                        if(g_dbgMode != OFF)
                        {
                            CONSOLE_UART_Timestamp();
                            CONSOLE_UART_Printf(HWDBG_ON);
                            CONSOLE_UART_Timestamp();
                            CONSOLE_UART_Printf(CHANGE_MESSAGE);
                            setSOCCurrentState(soc, SOC_ERROR);
                            soc->goal_state = SOC_ERROR;
                        }
                        else
                        {
                            CONSOLE_UART_Timestamp();
                            CONSOLE_UART_Printf(HWDBG_OFF);
                            CONSOLE_UART_Timestamp();
                            CONSOLE_UART_Printf(CHANGE_MESSAGE);
                        }

                        LCD_Printf(SOC_STATE, 0, "SOC POWER");
                        LCD_Printf(SOC_STATE, 1, "FAILURE");

                        delay(1000);

                        Semaphore_post(DEBUG_MODE_sem);
                    }
                }
                else
                {
                    setSOCCurrentState(soc, BOARD_STATE);
                }
                break;
            case SOC_POWER_ON:
                soc->boot_mode = g_soc_bootmode;
                LCD_Printf(BOOTMODE_PAGE, 1, "%s", g_soc_bootmode.title);
                if(socOutOfRST(soc))
                {
                    CONSOLE_UART_Timestamp();
                    console_printf(SOC_STATE, 0, "BOOT COMPLETE");
                    console_printf(SOC_STATE, 1, "\r\n");
                    setSOCCurrentState(soc, RUNNING);
                    Semaphore_post(POLLING_sem);
                }
                else
                {
                    Semaphore_pend(DEBUG_MODE_sem, BIOS_WAIT_FOREVER);

                    if(g_dbgMode != HWDBG)
                        socFullReset(soc);
                    if(g_dbgMode != OFF)
                    {
                        CONSOLE_UART_Timestamp();
                        CONSOLE_UART_Printf(HWDBG_ON);
                        CONSOLE_UART_Timestamp();
                        CONSOLE_UART_Printf(CHANGE_MESSAGE);
                        setSOCCurrentState(soc, RESET_ERROR);
                        soc->goal_state = RESET_ERROR;
                    }
                    else
                    {
                        CONSOLE_UART_Timestamp();
                        CONSOLE_UART_Printf(HWDBG_OFF);
                        CONSOLE_UART_Timestamp();
                        CONSOLE_UART_Printf(CHANGE_MESSAGE);
                    }

                    LCD_Printf(SOC_STATE, 0, "SOC RESET");
                    LCD_Printf(SOC_STATE, 1, "FAILURE");

                    delay(1000);

                    Semaphore_post(DEBUG_MODE_sem);
                }
                break;
            case RUNNING:
                switch(soc->goal_state)
                {
                    case SOC_POWER_ON: //RST command
                        if(soc->warm)
                            socWarmReset(soc);
                        else
                            socFullReset(soc);
                        setSOCCurrentState(soc, SOC_POWER_ON);
                        break;
                    case SOC_POWER_OFF: //POR command
                        socFullReset(soc);
                        socPOR(soc);
                        socPowerStop(soc, soc->graceful);
                        setSOCCurrentState(soc, SOC_POWER_OFF);
                        break;
                    case BOARD_STATE:
                        socFullReset(soc);
                        socPOR(soc);
                        socPowerStop(soc, soc->graceful);
                        setSOCCurrentState(soc, BOARD_STATE);
                        break;
                }
                break;
            case SOC_ERROR:
                if(soc->goal_state != SOC_ERROR)
                {
                    setSOCCurrentState(soc, SOC_POWER_OFF);
                }
                break;
            case RESET_ERROR:
                if(soc->goal_state != RESET_ERROR)
                {
                    switch(soc->goal_state)
                    {
                        case BOARD_STATE:
                            socFullReset(soc);
                            socPOR(soc);
                            socPowerStop(soc, soc->graceful);
                            setSOCCurrentState(soc, BOARD_STATE);
                            break;
                        case SOC_POWER_OFF:
                            socFullReset(soc);
                            socPOR(soc);
                            socPowerStop(soc, soc->graceful);
                            setSOCCurrentState(soc, SOC_POWER_OFF);
                            break;
                        case SOC_POWER_ON:
                        case RUNNING:
                        default:
                            socFullReset(soc);
                            setSOCCurrentState(soc, SOC_POWER_ON);
                            break;
                    }
                }
                break;
        }
        Semaphore_post(*soc->state_sem);
    }
    if(board_state_count == num_socs)
    {
        return 0;
    }
    else
        return 2;
}

/*
 * Sets the goal state for the board.
 */
void SetBoardGoalState(enum BOARD_STATE new_state)
{
    Semaphore_pend(BOARD_GOAL_sem, BIOS_WAIT_FOREVER);
    goal_board_state = new_state;
    Semaphore_post(BOARD_GOAL_sem);
    Semaphore_post(STATE_MACHINE_sem);
}

/*
 * Sets the goal state of the given SoC.
 */
void SetSOCGoalState(SOC *soc, enum SOC_STATE new_state)
{
    Semaphore_pend(*soc->state_sem, BIOS_WAIT_FOREVER);
    soc->goal_state = new_state;
    Semaphore_post(*soc->state_sem);
    Semaphore_post(STATE_MACHINE_sem);
}

/*
 * Sets the current state of the board.
 */
static void setBoardCurrentState(enum BOARD_STATE new_state)
{
	current_board_state = new_state;
}

/*
 * Sets the current state of the given SoC.
 */
static void setSOCCurrentState(SOC *soc, enum SOC_STATE new_state)
{
    soc->current_state = new_state;
}

/*
 * Returns the current board state
 */
enum BOARD_STATE GetBoardState()
{
    return current_board_state;
}

/*
 * Sets the debug mode.
 */
void SetDebugMode(enum DEBUG_MODE mode)
{
    Semaphore_pend(DEBUG_MODE_sem, BIOS_WAIT_FOREVER);
    g_dbgMode = mode;
    Semaphore_post(DEBUG_MODE_sem);
}

/*
 * Returns the debug mode.
 */
enum DEBUG_MODE GetDebugMode()
{
    return g_dbgMode;
}

/*
 * Sets the graceful variable, used to determine whether to perform
 * a graceful or forced shutdown.
 */
void SetGracefulMode(tBoolean new_mode)
{
    Semaphore_pend(GRACEFUL_sem, BIOS_WAIT_FOREVER);
    g_graceful = new_mode;
    Semaphore_post(GRACEFUL_sem);
}

/*
 * Sets the reboot variable, used to determine whether to reboot
 * after shutdown or to stay in a power off state.
 */
void SetReboot(tBoolean reboot)
{
    Semaphore_pend(REBOOT_sem, BIOS_WAIT_FOREVER);
    g_reboot = reboot;
    Semaphore_post(REBOOT_sem);
}

/*
 * Returrns the current value of the DIP switch.
 */
unsigned char getDIPSwitch()
{
    unsigned char dipSwitch;
    unsigned long ports;

    ports = GPIO_Start(DIP_SW_PORT);
    dipSwitch = (unsigned char)GPIO_Get(DIP_SW_PORT, ports) &
                (unsigned char)(DIP_SW_PINS);

    GPIO_Update(ports);

    dipSwitch = (~dipSwitch);
    dipSwitch = (dipSwitch & 0xA) >> 1 | (dipSwitch & 0x5) << 1;
    dipSwitch = (dipSwitch & 0xC) >> 2 | (dipSwitch & 0x3) << 2;

    return dipSwitch;
}

/*
 * Check all socs and their current states
 * if current states equal goal states, return true
 * otherwise return false
 */
static tBoolean checkSocStates()
{
    int i;
    SOC *soc;

    for(i = 0; i < num_socs; i++)
    {
        soc = &soc_list[i];
        Semaphore_pend(*soc->state_sem, BIOS_WAIT_FOREVER);

        if(soc->current_state != soc->goal_state)
        {
            Semaphore_post(*soc->state_sem);
            return false;
        }

        if(soc->reboot)
        {
            soc->reboot = false;
            soc->goal_state = RUNNING;
            Semaphore_post(*soc->state_sem);
            return false;
        }
        Semaphore_post(*soc->state_sem);
    }

    return true;
}

/*
 * Returns the bootmode at BOOT_MODES[index]
 */
tBoolean getBootModeByIndex(int index, BOOTMODE *mode)
{
    if(index > 15)
        return false;

    mode->hi_val = BOOT_MODES[index].hi_val;
    mode->lo_val = BOOT_MODES[index].lo_val;
    strcpy(mode->title, BOOT_MODES[index].title);

    if(mode->hi_val == g_soc_bootmode.hi_val &&
       mode->lo_val == g_soc_bootmode.lo_val &&
       (strcmp(mode->title, g_soc_bootmode.title) == 0))
    {
        return true;
    }
    return false;
}

/*
 * Resets the current bootmode based on the DIP switch.
 */
void resetBootMode()
{
    g_soc_bootmode = get_bootmode();
    LCD_Printf(BOOTMODE_PAGE, 0, "%s", g_soc_bootmode.title);
}

/*
 * Sets the bootmode to what is represented by BOOT_MODES[index]
 */
void setBootModeByIndex(int index)
{
    g_soc_bootmode.hi_val = BOOT_MODES[index].hi_val;
    g_soc_bootmode.lo_val = BOOT_MODES[index].lo_val;
    strcpy(g_soc_bootmode.title, BOOT_MODES[index].title);
    LCD_Printf(BOOTMODE_PAGE, 0, "%s", g_soc_bootmode.title);
}

/*
 * Sets the value of BOOT_MODES[index] to *mode. Stores the value to
 * flash if permanent is true.
 */
void saveBootModeByIndex(int index, BOOTMODE *mode, tBoolean permanent)
{
    if(index < 8 || index > 15)
        return;

    BOOT_MODES[index].hi_val = mode->hi_val;
    BOOT_MODES[index].lo_val = mode->lo_val;
    strcpy(BOOT_MODES[index].title, mode->title);

    if(permanent)
    {
        alter_bootmode(index, &BOOT_MODES[index]);
    }
}

/*
 * Get the current bootmode
 */
void getCurrentBootMode(BOOTMODE *mode)
{
    mode->hi_val = g_soc_bootmode.hi_val;
    mode->lo_val = g_soc_bootmode.lo_val;
    strcpy(mode->title, g_soc_bootmode.title);
}

/*
 * Initializes BMC
 */
static void bmc_init()
{
    int i;
    unsigned long board_type, board_version, board_sn;
    unsigned char verChars[4];
    char temp[20];
    const EVM_TYPE *evm_type, default_type = { "X.X.X.X", "UNKNOWN", "UNKNOWN" };

    CONSOLE_UART_Timestamp();
    console_printf(SOC_STATE, 0, "BMC Init ");
    console_printf(SOC_STATE, 1, "Begin\r\n");

    CONSOLE_UART_Timestamp();
    console_printf(BMC_NOTICE, 0, "BMC Version ");
    console_printf(BMC_NOTICE, 1, stringVer);
    CONSOLE_UART_Printf("\r\n");

    /* GPIO Initialization */
    Setup_GPIOInit();

    if(!(HWREG(BOARD_TYPE_REG) & FLASH_USERREG0_NW))
    {
        evm_type = &default_type;

        board_type = HWREG(BOARD_TYPE_REG);
        board_version = HWREG(BOARD_VER_REG);
        board_sn = HWREG(BOARD_SN_REG);

        if(board_type < num_evm_types)
            evm_type = &evm_types_list[board_type];

        CONSOLE_UART_Timestamp();
        console_printf(BOARD_INFO, 0, strcat(strcpy(temp, evm_type->specific_name), "\r\n"));

        verChars[0] = (unsigned char) ((board_version & 0xFF000000) >> 0x18);
        verChars[1] = (unsigned char) ((board_version & 0x00FF0000) >> 0x10);
        verChars[2] = (unsigned char) ((board_version & 0x0000FF00) >> 0x08);
        verChars[3] = (unsigned char) ((board_version & 0x000000FF) >> 0x00);
        CONSOLE_UART_Timestamp();
        if(verChars[3] == 0)
        {
            if(verChars[2] == 0)
                CONSOLE_UART_Printf("%u.%u\r\n", verChars[0], verChars[1]);
            else
                CONSOLE_UART_Printf("%u.%u.%u\r\n", verChars[0], verChars[1], verChars[2]);
        }
        else
            CONSOLE_UART_Printf("%u.%u.%u.%u\r\n", verChars[0], verChars[1], verChars[2], verChars[3]);

        CONSOLE_UART_Timestamp();
        CONSOLE_UART_Printf("S/N: %u\r\n", board_sn);
        LCD_Printf(BOARD_INFO, 1, "S/N: %u\r\n", board_sn);
    }

    if((!(HWREG(BOARD_TYPE_REG) & FLASH_USERREG0_NW)) &&
       (!(HWREG(BOARD_VER_REG) & FLASH_USERREG1_NW)) &&
       (!(HWREG(BOARD_SN_REG) & FLASH_USERREG2_NW)))
    {
        for(i = 0; i < total_commands; i++)
        {
            if(strcmp("commission", command_list[i].title) == 0)
            {
                command_list[i].hidden = 1;
            }
        }
    }

    if(*((unsigned char*)(BOOTMODE_LOCATION + CHECK_BYTE_OFFSET)) != 0)
    {
        flash_bootmode(&BOOT_MODES[num_user_bootmodes]);
    }
    else
    {
        for(i = 0; i < num_user_bootmodes; i++)
        {
            read_bootmode(i, &BOOT_MODES[i + num_user_bootmodes]);
        }
    }

    //MMC_PS_N0
    //VCC3V3_MP_ALT_DET

    g_soc_bootmode = get_bootmode();
    LCD_Printf(BOOTMODE_PAGE, 0, "%s", g_soc_bootmode.title);

    CONSOLE_UART_Timestamp();
    console_printf(SOC_STATE, 0, "BMC Init ");
    console_printf(SOC_STATE, 1, "Complete\r\n");
}

/*
 * Performs tasks necessary to start main power.
 */
static tBoolean mainPowerStart()
{
    unsigned long ports;
    unsigned char result;
    int lTimeout;

    CONSOLE_UART_Timestamp();
    console_printf(SOC_STATE, 0, "Main PWR Start ");
    console_printf(SOC_STATE, 1, "Begin\r\n");

    ports = GPIO_Start(MAIN_POWER_START_PORT);
    GPIO_RMW(MAIN_POWER_START_PORT, MAIN_POWER_START, MAIN_POWER_START, ports);
    GPIO_Update(ports);

    for(lTimeout = 0; lTimeout < SETUP_TIMEOUT; lTimeout++)
    {
        ports = GPIO_Start(MAIN_POWER_GOOD_PORT);
        result = (unsigned char)GPIO_Get(MAIN_POWER_GOOD_PORT, ports);
        GPIO_Update(ports);
        if(result & MAIN_POWER_GOOD)
            break;

        delay(1);
    }

    if(lTimeout >= SETUP_TIMEOUT)
    {
        if(g_dbgMode == CONTINUE)
        {
            CONSOLE_UART_Timestamp();
            CONSOLE_UART_Printf(ERROR_MESSAGE, MAIN_POWER_FAIL_MESSAGE);
            CONSOLE_UART_Timestamp();
            CONSOLE_UART_Printf(HWDBG_CONT);
            CONSOLE_UART_Timestamp();
            CONSOLE_UART_Printf(CHANGE_MESSAGE);
        }
        else
            return false;
    }

    Setup_GPIOMainPwr(true);

    CONSOLE_UART_Timestamp();
    console_printf(SOC_STATE, 0, "Main PWR Start ");
    console_printf(SOC_STATE, 1, "Complete\r\n");

    return true;
}

/*
 * Stops main power.
 */
static void mainPowerStop(tBoolean graceful)
{
    unsigned long ports;

    CONSOLE_UART_Timestamp();
    console_printf(SOC_STATE, 0, "Main PWR Stop ");
    console_printf(SOC_STATE, 1, "Begin\r\n");

    //if(graceful)
    //{
        //Currently no graceful shutdown
    //}
    //else
    //{
        ports = GPIO_Start(MAIN_POWER_START_PORT);
        GPIO_RMW(MAIN_POWER_START_PORT, MAIN_POWER_START, 0, ports);
        GPIO_Update(ports);
    //}

    Setup_GPIOMainPwr(false);

    CONSOLE_UART_Timestamp();
    console_printf(SOC_STATE, 0, "Main PWR Stop ");
    console_printf(SOC_STATE, 1, "Complete\r\n");
}

/*
 * Performs tasks necessary to start SoC power.
 */
static tBoolean socPowerStart(SOC *soc)
{
    unsigned long ports;
    unsigned char result;
    int lTimeout;

    CONSOLE_UART_Timestamp();
    console_printf(SOC_STATE, 0, "SOC PWR Start ");
    console_printf(SOC_STATE, 1, "Begin\r\n");

    ports = GPIO_Start(SOC_POWER_START_PORT);
    GPIO_RMW(SOC_POWER_START_PORT, SOC_POWER_START, SOC_POWER_START, ports);
    GPIO_Update(ports);

    for(lTimeout = 0; lTimeout < SETUP_TIMEOUT; lTimeout++)
    {
        ports = GPIO_Start(SOC_POWER_GOOD_PORT);
        result = (unsigned char)GPIO_Get(SOC_POWER_GOOD_PORT, ports);
        GPIO_Update(ports);
        if(result & SOC_POWER_GOOD)
            break;

        delay(1);
    }

    if(lTimeout >= SETUP_TIMEOUT)
    {
        if(g_dbgMode == CONTINUE)
        {
            CONSOLE_UART_Timestamp();
            CONSOLE_UART_Printf(ERROR_MESSAGE, SOC_POWER_FAIL_MESSAGE);
            CONSOLE_UART_Timestamp();
            CONSOLE_UART_Printf(HWDBG_CONT);
            CONSOLE_UART_Timestamp();
            CONSOLE_UART_Printf(CHANGE_MESSAGE);
        }
        else
            return false;
    }

    reset_gpio_expanders();

    if(!Setup_ClkGen(true))
    {
        if(g_dbgMode == CONTINUE)
        {
            CONSOLE_UART_Timestamp();
            CONSOLE_UART_Printf(ERROR_MESSAGE, CLK_GEN_SETUP_FAIL_MESSAGE);
            CONSOLE_UART_Timestamp();
            CONSOLE_UART_Printf(HWDBG_CONT);
            CONSOLE_UART_Timestamp();
            CONSOLE_UART_Printf(CHANGE_MESSAGE);
        }
        else
            return false;
    }

    Setup_GPIOSOCPwr(true);

    CONSOLE_UART_Timestamp();
    console_printf(SOC_STATE, 0, "SOC PWR Start ");
    console_printf(SOC_STATE, 1, "Complete\r\n");

    return true;
}

/*
 * Stops SoC power.
 */
static void socPowerStop(SOC *soc, tBoolean graceful)
{
    unsigned long ports;

    CONSOLE_UART_Timestamp();
    console_printf(SOC_STATE, 0, "SOC PWR Stop ");
    console_printf(SOC_STATE, 1, "Begin\r\n");

    //if(graceful)
    //{
        //Currently no graceful shutdown
    //}
    //else
    //{
        ports = GPIO_Start(SOC_POWER_START_PORT);
        GPIO_RMW(SOC_POWER_START_PORT, SOC_POWER_START, 0, ports);
        GPIO_Update(ports);
    //}

    Setup_GPIOSOCPwr(false);
    Setup_ClkGen(false);

    CONSOLE_UART_Timestamp();
    console_printf(SOC_STATE, 0, "SOC PWR Stop ");
    console_printf(SOC_STATE, 1, "Complete\r\n");
}

/*
 * Performs task necessary to bring SoC out of reset.
 */
static tBoolean socOutOfRST(SOC *soc)
{
    unsigned long ports;
    unsigned char result;
    int lTimeout;

    CONSOLE_UART_Timestamp();
    console_printf(SOC_STATE, 0, "SOC RST ");
    console_printf(SOC_STATE, 1, "Begin\r\n");

    program_bootmode(soc);

    ports = GPIO_Start(SOC_RESETZ_PORT);
    GPIO_RMW(SOC_RESETZ_PORT, SOC_RESETZ, SOC_RESETZ, ports);
    GPIO_Update(ports);
    delay(50);

    ports = GPIO_Start(SOC_RESETFULLZ_PORT | SOC_NMIZ_PORT | SOC_LRESETZ_PORT | SOC_LRESETNMIENZ_PORT);
    GPIO_RMW(SOC_RESETFULLZ_PORT, SOC_RESETFULLZ, SOC_RESETFULLZ, ports);
    GPIO_RMW(SOC_NMIZ_PORT, SOC_NMIZ, SOC_NMIZ, ports);
    GPIO_RMW(SOC_LRESETZ_PORT, SOC_LRESETZ, SOC_LRESETZ, ports);
    GPIO_RMW(SOC_LRESETNMIENZ_PORT, SOC_LRESETNMIENZ, SOC_LRESETNMIENZ, ports);
    GPIO_Update(ports);

    for(lTimeout = 0; lTimeout < SETUP_TIMEOUT; lTimeout++)
    {
        ports = GPIO_Start(SOC_RESETSTATZ_PORT);
        result = (unsigned char)GPIO_Get(SOC_RESETSTATZ_PORT, ports);
        GPIO_Update(ports);
        if(result & SOC_RESETSTATZ)
            break;

        delay(1);
    }

    ports = GPIO_Start(SOC_GPIO_PORT | SOC_CORESEL_PORT);
    GPIO_SetType(SOC_LOW_GPIO_PORT, SOC_LOW_GPIO_PINS, 0, ports);
    GPIO_SetType(SOC_HIGH_GPIO_PORT, SOC_HIGH_GPIO_PINS, 0, ports);
    GPIO_RMW(SOC_CORESEL_PORT, SOC_CORESEL_PINS, 0, ports);
    GPIO_Update(ports);

    if(lTimeout >= SETUP_TIMEOUT)
    {
        if(g_dbgMode == CONTINUE)
        {
            CONSOLE_UART_Timestamp();
            CONSOLE_UART_Printf(ERROR_MESSAGE, SOC_RESET_FAIL_MESSAGE);
            CONSOLE_UART_Timestamp();
            CONSOLE_UART_Printf(HWDBG_CONT);
            CONSOLE_UART_Timestamp();
            CONSOLE_UART_Printf(CHANGE_MESSAGE);
        }
        else
            return false;
    }

    Setup_GPIORST(true);

    CONSOLE_UART_Timestamp();
    console_printf(SOC_STATE, 0, "SOC RST ");
    console_printf(SOC_STATE, 1, "Complete\r\n");
    return true;
}

/*
 * Puts the SoC in warm reset.
 */
static void socWarmReset(SOC *soc)
{
    unsigned long ports;

    /* Need to add graceful reset */
    CONSOLE_UART_Timestamp();
    console_printf(SOC_STATE, 0, "Warm Reset ");
    console_printf(SOC_STATE, 1, "Begin\r\n");

    ports = GPIO_Start(SOC_RESETZ_PORT);
    GPIO_RMW(SOC_RESETZ_PORT, SOC_RESETZ, 0, ports);
    GPIO_Update(ports);

    CONSOLE_UART_Timestamp();
    console_printf(SOC_STATE, 0, "Warm Reset ");
    console_printf(SOC_STATE, 1, "Complete\r\n");
}

/*
 * Puts the SoC in full reset.
 */
static void socFullReset(SOC *soc)
{
    unsigned long ports;

    /* Need to add graceful reset */
    CONSOLE_UART_Timestamp();
    console_printf(SOC_STATE, 0, "Full Reset ");
    console_printf(SOC_STATE, 1, "Begin\r\n");

    ports = GPIO_Start(SOC_RESETZ_PORT | SOC_RESETFULLZ_PORT);
    GPIO_RMW(SOC_RESETZ_PORT, SOC_RESETZ, 0, ports);
    GPIO_RMW(SOC_RESETFULLZ_PORT, SOC_RESETFULLZ, 0, ports);
    GPIO_Update(ports);

    CONSOLE_UART_Timestamp();
    console_printf(SOC_STATE, 0, "Full Reset ");
    console_printf(SOC_STATE, 1, "Complete\r\n");
}

/*
 * Performs a power on reset for the SoC.
 */
static void socPOR(SOC *soc)
{
    unsigned long ports;

    /* Need to add graceful reset */
    CONSOLE_UART_Timestamp();
    console_printf(SOC_STATE, 0, "POR ");
    console_printf(SOC_STATE, 1, "Begin\r\n");

    ports = GPIO_Start(SOC_PORZ_PORT);
    GPIO_RMW(SOC_PORZ_PORT, SOC_PORZ, 0, ports);
    GPIO_Update(ports);

    CONSOLE_UART_Timestamp();
    console_printf(SOC_STATE, 0, "POR ");
    console_printf(SOC_STATE, 1, "Complete\r\n");
}

/*
 * Toggles the reset pin on the GPIO Expanders.
 */
static void reset_gpio_expanders()
{
    unsigned long ports;

    ports = GPIO_Start(SPI_GPIO_RESET_PORT);
    GPIO_RMW(SPI_GPIO_RESET_PORT, SPI_GPIO_RESET, 0, ports);
    GPIO_Update(ports);

    delay(1);

    ports = GPIO_Start(SPI_GPIO_RESET_PORT);
    GPIO_RMW(SPI_GPIO_RESET_PORT, SPI_GPIO_RESET, SPI_GPIO_RESET, ports);
    GPIO_Update(ports);

    delay(1);
}

/*
 * Retrieves the bootmode based on the DIP switch configuration.
 */
static BOOTMODE get_bootmode()
{
    unsigned char dipSwitch;
    BOOTMODE mode;

    dipSwitch = getDIPSwitch();

    mode.hi_val = BOOT_MODES[dipSwitch].hi_val;
    mode.lo_val = BOOT_MODES[dipSwitch].lo_val;
    strcpy(mode.title, BOOT_MODES[dipSwitch].title);

    return mode;
}

/*
 * Places the current bootmode on the SoC.
 */
static void program_bootmode(SOC *soc)
{
    unsigned long ports;
    unsigned int low_boot;

    CONSOLE_UART_Timestamp();
    CONSOLE_UART_Printf("Current BootMode is set to %s\r\n", soc->boot_mode.title);

    low_boot = soc->boot_mode.lo_val;

    ports = GPIO_Start(SOC_GPIO_PORT | SOC_CORESEL_PORT | SOC_CORECLKSEL_PORT | SOC_PACLKSEL_PORT);

    GPIO_SetType(SOC_LOW_GPIO_PORT, SOC_LOW_GPIO_PINS, SOC_LOW_GPIO_PINS, ports);
    GPIO_SetType(SOC_HIGH_GPIO_PORT, SOC_HIGH_GPIO_PINS, SOC_HIGH_GPIO_PINS, ports);

    GPIO_RMW(SOC_LOW_GPIO_PORT, SOC_GPIO_00 | SOC_GPIO_01 | SOC_GPIO_02 | SOC_GPIO_03 |
                                SOC_GPIO_04 | SOC_GPIO_05 | SOC_GPIO_06 | SOC_GPIO_07 |
                                SOC_GPIO_08 | SOC_GPIO_09 | SOC_GPIO_10 | SOC_GPIO_11 |
                                SOC_GPIO_12 | SOC_GPIO_13, low_boot, ports);
    GPIO_RMW(SOC_LOW_GPIO_PORT, SOC_GPIO_14 | SOC_GPIO_15, (low_boot >> 4), ports);
    GPIO_RMW(SOC_HIGH_GPIO_PORT, SOC_GPIO_16, (low_boot >> 0x10), ports);
    GPIO_RMW(SOC_CORESEL_PORT, SOC_CORESEL_PINS, (low_boot >> 6), ports);
    GPIO_RMW(SOC_CORECLKSEL_PORT, SOC_CORECLKSEL, (low_boot >> 0x18), ports);
    GPIO_RMW(SOC_PACLKSEL_PORT, SOC_PACLKSEL, (low_boot >> 0x18), ports);
    GPIO_Update(ports);
}

/*
 * Stores the user bootmodes in flash.
 */
static void flash_bootmode(BOOTMODE *modeStart)
{
    int i;
    unsigned long check_byte;

    check_byte = 0;

    FlashErase(BOOTMODE_LOCATION);

    for(i = 0; i < num_user_bootmodes; i++)
    {
        writeBootModeToFlash(modeStart++, BOOTMODE_LOCATION + sizeof(BOOTMODE)*(i));
    }

    FlashProgram(&check_byte, (BOOTMODE_LOCATION+CHECK_BYTE_OFFSET), (unsigned long)4);
}

/*
 * Stores the given bootmode in flash at index mode.
 */
void alter_bootmode(int mode, BOOTMODE *newMode)
{
    int i;
    BOOTMODE userModes[NUM_USER_BOOTMODES];

    mode -= 8;

    for(i = 0; i < num_user_bootmodes; i++)
    {
        readBootModeFromFlash(&userModes[i], (BOOTMODE_LOCATION + (sizeof(BOOTMODE) * i)));
    }

    userModes[mode].hi_val = newMode->hi_val;
    userModes[mode].lo_val = newMode->lo_val;
    strcpy(userModes[mode].title, newMode->title);

    flash_bootmode(userModes);
}

/*
 * Reads the bootmode indexed by mode and stores it in modePtr.
 *
 * NOTE: Index ranges from 0 - 7, not 8 - 15.
 */
void read_bootmode(int mode, BOOTMODE *modePtr)
{
    readBootModeFromFlash(modePtr, (BOOTMODE_LOCATION + (sizeof(BOOTMODE) * mode)));
}

/*
 * Initial GPIO setup.
 */
static void Setup_GPIOInit()
{
    unsigned long ports;

    ports = GPIO_Start(MAIN_POWER_START_PORT | SOC_POWER_START_PORT |
                       MMC_RED_LED_PORT      | MMC_BLUE_LED_PORT    );

    GPIO_SetType(MAIN_POWER_START_PORT, MAIN_POWER_START, MAIN_POWER_START, ports);
    GPIO_SetType(SOC_POWER_START_PORT, SOC_POWER_START, SOC_POWER_START, ports);
    GPIO_SetType(MMC_RED_LED_PORT, MMC_RED_LED, MMC_RED_LED, ports);
    GPIO_SetType(MMC_BLUE_LED_PORT, MMC_BLUE_LED, MMC_BLUE_LED, ports);

    GPIO_RMW(MAIN_POWER_START_PORT, MAIN_POWER_START, 0, ports);
    GPIO_RMW(SOC_POWER_START_PORT, SOC_POWER_START, 0, ports);
    GPIO_RMW(MMC_RED_LED_PORT, MMC_RED_LED, MMC_RED_LED, ports);
    GPIO_RMW(MMC_BLUE_LED_PORT, MMC_BLUE_LED, MMC_BLUE_LED, ports);

    GPIO_Update(ports);
}

/*
 * Main Power GPIO setup.
 */
static void Setup_GPIOMainPwr(tBoolean dir)
{
    unsigned long ports;

    ports = GPIO_Start(PHY_RST_PORT | NOR_WP_PORT | PCIECLK_MCU_PD_PORT | PCIECLK_OE_PORT |
                       SPI_GPIO_RESET_PORT | PCIECLK_MUX_SEL_PORT);

    GPIO_SetType(PHY_RST_PORT, PHY_RST, (dir) ? PHY_RST : 0, ports);
    GPIO_SetType(NOR_WP_PORT, NOR_WP, (dir) ? NOR_WP : 0, ports);
    GPIO_SetType(PCIECLK_MCU_PD_PORT, PCIECLK_MCU_PD, (dir) ? PCIECLK_MCU_PD : 0, ports);
    GPIO_SetType(PCIECLK_OE_PORT, PCIECLK_OE, (dir) ? PCIECLK_OE : 0, ports);
    GPIO_SetType(SPI_GPIO_RESET_PORT, SPI_GPIO_RESET, (dir) ? SPI_GPIO_RESET : 0, ports);
    GPIO_SetType(PCIECLK_MUX_SEL_PORT, PCIECLK_MUX_SEL, (dir) ? PCIECLK_MUX_SEL : 0, ports);

    if(dir)
    {
        //GPIO_RMW(PHY_RST_PORT, PHY_RST, 0, ports);
        GPIO_RMW(NOR_WP_PORT, NOR_WP, 0, ports);
        GPIO_RMW(PCIECLK_MCU_PD_PORT, PCIECLK_MCU_PD, 0, ports);
        GPIO_RMW(PCIECLK_OE_PORT, PCIECLK_OE, 0, ports);
        GPIO_RMW(SPI_GPIO_RESET_PORT, SPI_GPIO_RESET, SPI_GPIO_RESET, ports);
        GPIO_RMW(PCIECLK_MUX_SEL_PORT, PCIECLK_MUX_SEL, 0, ports);
    }

    GPIO_Update(ports);
}

/*
 * Clock Generator (CDCM) setup.
 */
static tBoolean Setup_ClkGen(tBoolean dir)
{
    unsigned long ports;

    ports = GPIO_Start(MCU_EMU_DET_PORT | REFCLK1_PD_PORT | REFCLK2_PD_PORT | REFCLK3_PD_PORT |
                       CLK_RSTz_PORT | CLK3_REF_SEL_PORT | CLK2_REF_SEL_PORT | CLK1_REF_SEL_PORT);

    GPIO_SetType(MCU_EMU_DET_PORT, MCU_EMU_DET, (dir) ? MCU_EMU_DET : 0, ports);
    GPIO_SetType(REFCLK1_PD_PORT, REFCLK1_PD, (dir) ? REFCLK1_PD : 0, ports);
    GPIO_SetType(REFCLK2_PD_PORT, REFCLK2_PD, (dir) ? REFCLK2_PD : 0, ports);
    GPIO_SetType(REFCLK3_PD_PORT, REFCLK3_PD, (dir) ? REFCLK3_PD : 0, ports);
    GPIO_SetType(CLK_RSTz_PORT, CLK_RSTz, (dir) ? CLK_RSTz : 0, ports);
    GPIO_SetType(CLK3_REF_SEL_PORT, CLK3_REF_SEL, (dir) ? CLK3_REF_SEL : 0, ports);
    GPIO_SetType(CLK2_REF_SEL_PORT, CLK2_REF_SEL, (dir) ? CLK2_REF_SEL : 0, ports);
    GPIO_SetType(CLK1_REF_SEL_PORT, CLK1_REF_SEL, (dir) ? CLK1_REF_SEL : 0, ports);

    if(dir)
    {
        GPIO_RMW(MCU_EMU_DET_PORT, MCU_EMU_DET, MCU_EMU_DET, ports);
        GPIO_RMW(REFCLK1_PD_PORT, REFCLK1_PD, 0, ports);
        GPIO_RMW(REFCLK2_PD_PORT, REFCLK2_PD, 0, ports);
        GPIO_RMW(REFCLK3_PD_PORT, REFCLK3_PD, 0, ports);
        GPIO_RMW(CLK_RSTz_PORT, CLK_RSTz, 0, ports);
        GPIO_RMW(CLK3_REF_SEL_PORT, CLK3_REF_SEL, 0, ports);
        GPIO_RMW(CLK2_REF_SEL_PORT, CLK2_REF_SEL, 0, ports);
        GPIO_RMW(CLK1_REF_SEL_PORT, CLK1_REF_SEL, 0, ports);
    }

    GPIO_Update(ports);

    if(dir)
    {
        int clk, lTimeout;
        unsigned int reg;
        unsigned short value;
        const unsigned short *clkgenRegPtr, *clkgenPtr;

        delay(10);
        ports = GPIO_Start(REFCLK1_PD_PORT | REFCLK2_PD_PORT | REFCLK3_PD_PORT);
        GPIO_RMW(REFCLK1_PD_PORT, REFCLK1_PD, REFCLK1_PD, ports);
        GPIO_RMW(REFCLK2_PD_PORT, REFCLK2_PD, REFCLK2_PD, ports);
        GPIO_RMW(REFCLK3_PD_PORT, REFCLK3_PD, REFCLK3_PD, ports);
        GPIO_Update(ports);
        delay(10);

        ports = GPIO_Start(CLK_RSTz_PORT | CLK3_REF_SEL_PORT | CLK2_REF_SEL_PORT | CLK1_REF_SEL_PORT);
        GPIO_RMW(CLK_RSTz_PORT, CLK_RSTz, CLK_RSTz, ports);
        GPIO_RMW(CLK3_REF_SEL_PORT, CLK3_REF_SEL, CLK3_REF_SEL, ports);
        GPIO_RMW(CLK2_REF_SEL_PORT, CLK2_REF_SEL, CLK2_REF_SEL, ports);
        GPIO_RMW(CLK1_REF_SEL_PORT, CLK1_REF_SEL, CLK1_REF_SEL, ports);
        GPIO_Update(ports);
        delay(10);

        clkgenPtr = (g_soc_bootmode.lo_val & 0x20000000) ? (clkgen[1]) : clkgen[0];

        for(clk = 0; clk < clk_gen_amount; clk++)
        {
            if(clk != 0) clkgenPtr = clkgen[clk+1];

            clkgenRegPtr = clkgenPtr;
            for(reg = 0; reg < clk_gen_reg_amount; reg++)
            {
                value = (*clkgenRegPtr++);
                CLOCK_RegWrite(clk, reg, value);
                if(reg == 0)
                    CLOCK_RegWrite(clk, reg, value);
#ifdef _DEBUG
                unsigned short check;
                check = CLOCK_RegRead(clk, reg);
                CONSOLE_UART_Timestamp();
                CONSOLE_UART_Printf("Testing register %i of clock %i...", reg, clk+1);
                if(value !=  check)
                    CONSOLE_UART_Printf(" Failed: Wanted 0x%04X, got 0x%04X\r\n", value, check);
                else
                    CONSOLE_UART_Printf(" Passed.\r\n");
                delay(5);
#endif
            }

            CONSOLE_UART_Timestamp();
            CONSOLE_UART_Printf("Clock %i\r\n", clk+1);
            CLOCK_RegWrite(clk, 3, (clkgenPtr[3] & ~(0x40)));
            delay(10);
            CLOCK_RegWrite(clk, 3, (clkgenPtr[3] | 0x40));
            delay(10);

            for(lTimeout = 0; lTimeout < (SETUP_TIMEOUT*5); lTimeout++)
            {
                ports = GPIO_Start(PLL_LOCK_PORT[clk]);
                if(GPIO_Get(PLL_LOCK_PORT[clk], ports) & PLL_LOCK[clk])
                {
                    GPIO_Update(ports);
                    break;
                }
                GPIO_Update(ports);
                delay(1);
            }
            if(lTimeout >= (SETUP_TIMEOUT*5))
            {
                CONSOLE_UART_Timestamp();
                CONSOLE_UART_Printf("Failed\r\n");

                return false;
            }
            else
            {
                CONSOLE_UART_Timestamp();
                CONSOLE_UART_Printf("Passed\r\n");
            }
        }
    }

    return true;
}

/*
 * SoC Power GPIO setup.
 */
static void Setup_GPIOSOCPwr(tBoolean dir)
{
    unsigned long ports;

    ports = GPIO_Start(PCIECLK_MCU_PD_PORT | PCIECLK_OE_PORT | PHY_RST_PORT | SOC_PORZ_PORT |
                       SOC_GPIO_PORT | PLLLOCK_LED_PORT | NAND_WPz_PORT | EEPROM_WP_PORT |
                       SOC_CORECLKSEL_PORT | SOC_PACLKSEL_PORT | SOC_NMIZ_PORT | SOC_LRESETZ_PORT |
                       SOC_LRESETNMIENZ_PORT | SOC_CORESEL_PORT | TIMI_MUX_OEz_PORT |
                       SOC_RESETFULLZ_PORT | SOC_RESETZ_PORT | MCU_EMU_DET_PORT | SOC_VPPB_EN_PORT);

    GPIO_SetType(SOC_PORZ_PORT, SOC_PORZ, (dir) ? SOC_PORZ : 0, ports);
    GPIO_SetType(SOC_LOW_GPIO_PORT, SOC_LOW_GPIO_PINS, (dir) ? SOC_LOW_GPIO_PINS : 0, ports);
    GPIO_SetType(SOC_HIGH_GPIO_PORT, SOC_HIGH_GPIO_PINS, (dir) ? SOC_HIGH_GPIO_PINS : 0, ports);
    GPIO_SetType(PLLLOCK_LED_PORT, PLLLOCK_LED, (dir) ? PLLLOCK_LED : 0, ports);
    GPIO_SetType(NAND_WPz_PORT, NAND_WPz, (dir) ? NAND_WPz : 0, ports);
    GPIO_SetType(EEPROM_WP_PORT, EEPROM_WP, (dir) ? EEPROM_WP : 0, ports);
    GPIO_SetType(SOC_CORECLKSEL_PORT, SOC_CORECLKSEL, (dir) ? SOC_CORECLKSEL : 0, ports);
    GPIO_SetType(SOC_PACLKSEL_PORT, SOC_PACLKSEL, (dir) ? SOC_PACLKSEL : 0, ports);
    GPIO_SetType(SOC_NMIZ_PORT, SOC_NMIZ, (dir) ? SOC_NMIZ : 0, ports);
    GPIO_SetType(SOC_LRESETZ_PORT, SOC_LRESETZ, (dir) ? SOC_LRESETZ : 0, ports);
    GPIO_SetType(SOC_LRESETNMIENZ_PORT, SOC_LRESETNMIENZ, (dir) ? SOC_LRESETNMIENZ : 0, ports);
    GPIO_SetType(SOC_CORESEL_PORT, SOC_CORESEL_PINS, (dir) ? SOC_CORESEL_PINS : 0, ports);
    GPIO_SetType(TIMI_MUX_OEz_PORT, TIMI_MUX_OEz, (dir) ? TIMI_MUX_OEz : 0, ports);
    GPIO_SetType(SOC_RESETFULLZ_PORT, SOC_RESETFULLZ, (dir) ? SOC_RESETFULLZ : 0, ports);
    GPIO_SetType(SOC_RESETZ_PORT, SOC_RESETZ, (dir) ? SOC_RESETZ : 0, ports);
    GPIO_SetType(SOC_VPPB_EN_PORT, SOC_VPPB_EN, (dir) ? SOC_VPPB_EN : 0, ports);

    if(dir)
    {
        GPIO_RMW(PCIECLK_MCU_PD_PORT, PCIECLK_MCU_PD, PCIECLK_MCU_PD, ports);
        GPIO_RMW(PCIECLK_OE_PORT, PCIECLK_OE, PCIECLK_OE, ports);
        GPIO_RMW(PHY_RST_PORT, PHY_RST, PHY_RST, ports);
        GPIO_RMW(SOC_PORZ_PORT, SOC_PORZ, SOC_PORZ, ports);
        GPIO_RMW(PLLLOCK_LED_PORT, PLLLOCK_LED, 0, ports);
        GPIO_RMW(NAND_WPz_PORT, NAND_WPz, NAND_WPz, ports);
        GPIO_RMW(EEPROM_WP_PORT, EEPROM_WP, 0, ports);
        GPIO_RMW(SOC_CORECLKSEL_PORT, SOC_CORECLKSEL, 0, ports);
        GPIO_RMW(SOC_PACLKSEL_PORT, SOC_PACLKSEL, 0, ports);
        GPIO_RMW(SOC_NMIZ_PORT, SOC_NMIZ, 0, ports);
        GPIO_RMW(SOC_LRESETZ_PORT, SOC_LRESETZ, 0, ports);
        GPIO_RMW(SOC_LRESETNMIENZ_PORT, SOC_LRESETNMIENZ, 0, ports);
        GPIO_RMW(SOC_CORESEL_PORT, SOC_CORESEL_PINS, 0, ports);
        GPIO_RMW(TIMI_MUX_OEz_PORT, TIMI_MUX_OEz, 0, ports);
        GPIO_RMW(SOC_RESETFULLZ_PORT, SOC_RESETFULLZ, 0, ports);
        GPIO_RMW(SOC_RESETZ_PORT, SOC_RESETZ, 0, ports);
        GPIO_RMW(SOC_VPPB_EN_PORT, SOC_VPPB_EN, 0, ports);
        GPIO_RMW(MCU_EMU_DET_PORT, MCU_EMU_DET, 0, ports);
    }

    GPIO_Update(ports);
}

/*
 * SoC Reset GPIO setup.
 */
static void Setup_GPIORST(tBoolean dir)
{
    unsigned long ports;

    if(dir)
    {
        ports = GPIO_Start(SOC_VPPB_EN_PORT);
        GPIO_RMW(SOC_VPPB_EN_PORT, SOC_VPPB_EN, 0, ports);
        GPIO_Update(ports);
    }
}

/*
 * Prints a formatted string to both the UART and LCD.
 */
static void console_printf(int lPage, int lLine, const char *format, ...)
{
    va_list argList;
    va_start(argList, format);
    CONSOLE_UART_Printf(format, argList);
    LCD_Printf(lPage, lLine, format, argList);
    va_end(argList);
}

/*
 * Writes the given mode to the specified address.
 */
static void writeBootModeToFlash(BOOTMODE *mode, unsigned long ulAddress)
{
    unsigned long size;

    // size must be a multiple of 4
    size = sizeof(BOOTMODE) + 3;
    size -= size%4;

    FlashProgram((unsigned long*)mode, ulAddress, size);
}

/*
 * Reads the bootmode at the specified address.
 */
static void readBootModeFromFlash(BOOTMODE *mode, unsigned long ulAddress)
{
    memcpy(mode, (void*)ulAddress, sizeof(BOOTMODE));
}
