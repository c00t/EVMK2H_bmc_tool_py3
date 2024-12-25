/******************************************************************************
 *
 * File    Name:       bmc_polling.c
 *
 * Description: This contains the Task to periodically check the status of the 
 *              BMC system.
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

#include <ti/sysbios/knl/Semaphore.h>

#include "bmc_map.h"
#include "bmc.h"
#include "bmc_state.h"

#include "bmc_uart.h"
#include "bmc_lcd.h"

#include "gpio_driver.h"

#define MAX_COUNT   10

extern Semaphore_Handle POLLING_sem;
extern Semaphore_Handle STATE_MACHINE_sem;

static void Output_SOC_GPIO(unsigned long ports)
{
    unsigned short dipSW, pwrButton, attnButton, soc_gpio_result;
    
    dipSW = GPIO_Get(DIP_SW_PORT, ports) & (DIP_SW_B0 | DIP_SW_B1 | DIP_SW_B2 | DIP_SW_B3);
    pwrButton = GPIO_Get(FULL_RESET_PORT, ports) & FULL_RESET;
    attnButton = GPIO_Get(WARM_RESET_PORT, ports) & WARM_RESET;
    
    soc_gpio_result = ((dipSW & DIP_SW_B0) ? SOC_GPIO_00 : 0) |
                      ((dipSW & DIP_SW_B1) ? SOC_GPIO_01 : 0) |
                      ((dipSW & DIP_SW_B2) ? SOC_GPIO_02 : 0) |
                      ((dipSW & DIP_SW_B3) ? SOC_GPIO_03 : 0) |
                      ((pwrButton & FULL_RESET) ? SOC_GPIO_04 : 0) |
                      ((attnButton & WARM_RESET) ? SOC_GPIO_05 : 0);
    
    GPIO_SetType(SOC_LOW_GPIO_PORT, SOC_GPIO_00 | SOC_GPIO_01 |
                                    SOC_GPIO_02 | SOC_GPIO_03 |
                                    SOC_GPIO_04 | SOC_GPIO_04, 0xFF, ports);
    
    GPIO_RMW(SOC_LOW_GPIO_PORT, SOC_GPIO_00 | SOC_GPIO_01 |
                                SOC_GPIO_02 | SOC_GPIO_03 |
                                SOC_GPIO_04 | SOC_GPIO_04, soc_gpio_result, ports);
    
    //Inputs
    GPIO_SetType(SOC_LOW_GPIO_PORT, SOC_GPIO_08 | SOC_GPIO_09 | SOC_GPIO_10 |
                                    SOC_GPIO_11 | SOC_GPIO_12 | SOC_GPIO_13 |
                                    SOC_GPIO_14 | SOC_GPIO_15, 0, ports);
}

void Polling_Task()
{
    unsigned long ports;
    unsigned short soc_request, former_state;
    int main_pwr_count, soc_pwr_count, resetstatz_count, full_reset_pressed_count, full_reset_released_count;
    int i, number_pressed;
    SOC *soc;
    enum DEBUG_MODE hwdbg_mode;
    tBoolean reboot, dc_detect;
    
    main_pwr_count = 0;
    soc_pwr_count = 0;
    resetstatz_count = 0;
    full_reset_pressed_count = 0;
    full_reset_released_count = 0;
    
    number_pressed = 0;
    former_state = 0;
    
    dc_detect = false;
    
    while(1)
    {
        Semaphore_pend(POLLING_sem, BIOS_WAIT_FOREVER);
        
        ports = GPIO_Start(MAIN_POWER_GOOD_PORT | SOC_POWER_GOOD_PORT | SOC_RESETSTATZ_PORT |
                           WARM_RESET_PORT | FULL_RESET_PORT | SOC_GPIO_16_PORT | SOC_GPIO_06_PORT |
                           SOC_GPIO_07_PORT | SOC_GPIO_PORT | DIP_SW_PORT | SOC_PLLLOCK_PORT | PLLLOCK_LED_PORT);
        
        if((GPIO_Get(MAIN_POWER_GOOD_PORT, ports) & MAIN_POWER_GOOD) == 0)
        {
            if(main_pwr_count++ > MAX_COUNT)
            {
                CONSOLE_UART_Printf("\r\n");
                CONSOLE_UART_Timestamp();
                CONSOLE_UART_Printf("Error: MAIN_POWER_GOOD signal has de-asserted. Shutting down board.\r\n\r\n");
                
                hwdbg_mode = GetDebugMode();
                reboot = (hwdbg_mode != ON);
                
                for(i = 0; i < num_socs; i++)
                {
                    soc = &soc_list[i];
                    Semaphore_pend(*soc->state_sem, BIOS_WAIT_FOREVER);
                    
                    /* TODO: adjust these based on hwdbg settings */
                    soc->graceful = false;
                    soc->reboot = reboot;
                    soc->warm = false;
                    soc->goal_state = BOARD_STATE;
                    
                    Semaphore_post(*soc->state_sem);
                }
                
                SetReboot(reboot);
                SetBoardGoalState(MAIN_POWER_OFF);
                main_pwr_count = 0;
                Semaphore_pend(POLLING_sem, BIOS_NO_WAIT);
            }
            else Semaphore_post(POLLING_sem);
            GPIO_Update(ports);
            delay(10);
            continue;
        }
        else main_pwr_count = 0;
        
        if((GPIO_Get(SOC_POWER_GOOD_PORT, ports) & SOC_POWER_GOOD) == 0)
        {
            if(soc_pwr_count++ > MAX_COUNT)
            {
                CONSOLE_UART_Printf("\r\n");
                CONSOLE_UART_Timestamp();
                CONSOLE_UART_Printf("Error: SOC_POWER_GOOD signal has de-asserted. Shutting down soc power.\r\n\r\n");
                
                hwdbg_mode = GetDebugMode();
                reboot = (hwdbg_mode != ON);
                
                for(i = 0; i < num_socs; i++)
                {
                    soc = &soc_list[i];
                    Semaphore_pend(*soc->state_sem, BIOS_WAIT_FOREVER);
                    
                    /* TODO: adjust these based on hwdbg settings */
                    soc->graceful = false;
                    soc->reboot = reboot;
                    soc->warm = false;
                    soc->goal_state = SOC_POWER_OFF;
                    
                    Semaphore_post(*soc->state_sem);
                }
                
                SetReboot(reboot);
                Semaphore_post(STATE_MACHINE_sem);
                soc_pwr_count = 0;
                Semaphore_pend(POLLING_sem, BIOS_NO_WAIT);
            }
            else Semaphore_post(POLLING_sem);
            GPIO_Update(ports);
            delay(10);
            continue;
        }
        else soc_pwr_count = 0;
        
        if((GPIO_Get(SOC_RESETSTATZ_PORT, ports) & SOC_RESETSTATZ) == 0)
        {
            if(resetstatz_count++ > MAX_COUNT)
            {
                CONSOLE_UART_Printf("\r\n");
                CONSOLE_UART_Timestamp();
                CONSOLE_UART_Printf("Error: RESETSTATz has de-asserted, placing SOC in reset.\r\n\r\n");
                
                hwdbg_mode = GetDebugMode();
                reboot = (hwdbg_mode != ON);
                
                for(i = 0; i < num_socs; i++)
                {
                    soc = &soc_list[i];
                    
                    Semaphore_pend(*soc->state_sem, BIOS_WAIT_FOREVER);
                    
                    soc->graceful = false;
                    soc->reboot = reboot;
                    soc->warm = false;
                    
                    soc->goal_state = SOC_POWER_ON;
                }
                Semaphore_post(STATE_MACHINE_sem);
                resetstatz_count = 0;
                Semaphore_pend(POLLING_sem, BIOS_NO_WAIT);
            }
            else Semaphore_post(POLLING_sem);
            GPIO_Update(ports);
            delay(10);
            continue;
        }
        else resetstatz_count = 0;
        
        if((GPIO_Get(SOC_GPIO_16_PORT, ports) & SOC_GPIO_16) != 0)
        {
            if(!dc_detect)
            {
                dc_detect = true;
                
                CONSOLE_UART_Printf("\r\n");
                CONSOLE_UART_Timestamp();
                CONSOLE_UART_Printf("Daughtercard detected, setting all SOC GPIO pins to input\r\n\r\n");
                
                GPIO_SetType(SOC_LOW_GPIO_PORT, SOC_LOW_GPIO_PINS, 0, ports);
                GPIO_SetType(SOC_HIGH_GPIO_PORT, SOC_HIGH_GPIO_PINS, 0, ports);
            }
        }
        else
        {
            dc_detect = false;
            soc_request = (GPIO_Get(SOC_GPIO_07_PORT, ports) & SOC_GPIO_07) |
                          (GPIO_Get(SOC_GPIO_06_PORT, ports) & SOC_GPIO_06);
            
            if(soc_request != former_state)
            {
                former_state = soc_request;
                switch(soc_request)
                {
                    case 0:
                        GPIO_SetType(SOC_LOW_GPIO_PORT, SOC_LOW_GPIO_PINS, 0, ports);
                        GPIO_SetType(SOC_HIGH_GPIO_PORT, SOC_HIGH_GPIO_PINS, 0, ports);
                        break;
                    case SOC_GPIO_06:
                        CONSOLE_UART_Timestamp();
                        CONSOLE_UART_Printf("SOC has requested a reboot\r\n");
                        LCD_Printf(BMC_NOTICE, 0, "SOC requested");
                        LCD_Printf(BMC_NOTICE, 1, "reboot");
                        for(i = 0; i < num_socs; i++)
                            {
                                soc = &soc_list[i];
                                Semaphore_pend(*soc->state_sem, BIOS_WAIT_FOREVER);
                                
                                /* TODO: adjust these based on hwdbg settings */
                                soc->graceful = true;
                                soc->reboot = true;
                                soc->warm = false;
                                soc->goal_state = SOC_POWER_OFF;
                                
                                Semaphore_post(*soc->state_sem);
                            }
                        
                        Output_SOC_GPIO(ports);
                        
                        break;
                    case SOC_GPIO_07:
                        Output_SOC_GPIO(ports);
                        break;
                    case (SOC_GPIO_07 | SOC_GPIO_06):
                        CONSOLE_UART_Timestamp();
                        CONSOLE_UART_Printf("SOC has requested a shutdown\r\n");
                        LCD_Printf(BMC_NOTICE, 0, "SOC requested");
                        LCD_Printf(BMC_NOTICE, 1, "shutdown");
                        for(i = 0; i < num_socs; i++)
                            {
                                soc = &soc_list[i];
                                Semaphore_pend(*soc->state_sem, BIOS_WAIT_FOREVER);
                                
                                /* TODO: adjust these based on hwdbg settings */
                                soc->graceful = true;
                                soc->reboot = false;
                                soc->warm = false;
                                soc->goal_state = BOARD_STATE;
                                
                                Semaphore_post(*soc->state_sem);
                            }
                            
                            SetReboot(false);
                            SetGracefulMode(true);
                            SetBoardGoalState(MAIN_POWER_OFF);
                        Output_SOC_GPIO(ports);
                        break;
                }
            }
        }
        
        if((GPIO_Get(FULL_RESET_PORT, ports) & FULL_RESET) == 0)
        {
            if(full_reset_released_count > 0)
            {
                full_reset_released_count = 0;
                full_reset_pressed_count = 0;
            }
            if(full_reset_pressed_count++ == 300)
            {
                full_reset_pressed_count = 0;
                full_reset_released_count = 0;
                number_pressed = 0;
                CONSOLE_UART_Timestamp();
                CONSOLE_UART_Printf("FULL SHUTDOWN\r\n");
                LCD_Printf(BMC_NOTICE, 0, "FULL SHUTDOWN");
                for(i = 0; i < num_socs; i++)
                {
                    soc = &soc_list[i];
                    Semaphore_pend(*soc->state_sem, BIOS_WAIT_FOREVER);
                    
                    /* TODO: adjust these based on hwdbg settings */
                    soc->graceful = false;
                    soc->reboot = false;
                    soc->warm = false;
                    soc->goal_state = BOARD_STATE;
                    
                    Semaphore_post(*soc->state_sem);
                }
                
                SetReboot(false);
                SetGracefulMode(false);
                SetBoardGoalState(MAIN_POWER_OFF);
                Semaphore_pend(POLLING_sem, BIOS_NO_WAIT);
            }
            else Semaphore_post(POLLING_sem);
            GPIO_Update(ports);
            delay(10);
            continue;
        }
        else if(full_reset_pressed_count >= 3)
        {
            if(full_reset_released_count >= 100)
            {
                full_reset_pressed_count = 0;
                full_reset_released_count = 0;
                switch(number_pressed)
                {
                    case 1:
                        // Safe Shutdown
                        CONSOLE_UART_Timestamp();
                        CONSOLE_UART_Printf("Shutting down EVM...\r\n");
                        LCD_Printf(BMC_NOTICE, 1, "Begin");
                        for(i = 0; i < num_socs; i++)
                        {
                            soc = &soc_list[i];
                            Semaphore_pend(*soc->state_sem, BIOS_WAIT_FOREVER);
                            
                            /* TODO: adjust these based on hwdbg settings */
                            soc->graceful = true;
                            soc->reboot = false;
                            soc->warm = false;
                            soc->goal_state = BOARD_STATE;
                            
                            Semaphore_post(*soc->state_sem);
                        }
                        
                        SetReboot(false);
                        SetGracefulMode(true);
                        SetBoardGoalState(MAIN_POWER_OFF);
                        break;
                    case 2:
                        // Warm Reset
                        CONSOLE_UART_Timestamp();
                        CONSOLE_UART_Printf("Initiating Warm Reset...\r\n");
                        LCD_Printf(BMC_NOTICE, 1, "Begin");
                        for(i = 0; i < num_socs; i++)
                        {
                            soc = &soc_list[i];
                            Semaphore_pend(*soc->state_sem, BIOS_WAIT_FOREVER);
                            
                            /* TODO: adjust these based on hwdbg settings */
                            soc->graceful = true;
                            soc->reboot = true;
                            soc->warm = true;
                            soc->goal_state = SOC_POWER_ON;
                            
                            Semaphore_post(*soc->state_sem);
                        }
                        Semaphore_post(STATE_MACHINE_sem);
                        break;
                    case 3:
                        // Full Reset
                        CONSOLE_UART_Timestamp();
                        CONSOLE_UART_Printf("Initiating Full Reset...\r\n");
                        LCD_Printf(BMC_NOTICE, 1, "Begin");
                        for(i = 0; i < num_socs; i++)
                        {
                            soc = &soc_list[i];
                            Semaphore_pend(*soc->state_sem, BIOS_WAIT_FOREVER);
                            
                            /* TODO: adjust these based on hwdbg settings */
                            soc->graceful = true;
                            soc->reboot = true;
                            soc->warm = false;
                            soc->goal_state = SOC_POWER_ON;
                            
                            Semaphore_post(*soc->state_sem);
                        }
                        Semaphore_post(STATE_MACHINE_sem);
                        break;
                    case 4:
                        // Cancel
                        LCD_Printf(BMC_NOTICE, 0, "Cancelled");
                        number_pressed = 0;
                        Semaphore_post(POLLING_sem);
                        GPIO_Update(ports);
                        delay(10);
                        continue;
                }
                number_pressed = 0;
                Semaphore_pend(POLLING_sem, BIOS_NO_WAIT);
            }
            else
            {
                if(full_reset_released_count == 0)
                {
                    if(number_pressed < 4)
                    {
                        number_pressed++;
                    }
                    switch(number_pressed)
                    {
                        case 1:
                            LCD_Printf(BMC_NOTICE, 0, "SAFE SHUTDOWN");
                            break;
                        case 2:
                            LCD_Printf(BMC_NOTICE, 0, "WARM RESET");
                            break;
                        case 3:
                            LCD_Printf(BMC_NOTICE, 0, "FULL RESET");
                            break;
                        case 4:
                            LCD_Printf(BMC_NOTICE, 0, "CANCELLING");
                            break;
                    }
                    if(number_pressed < 4) LCD_Printf(BMC_NOTICE, 1, "PUSH TO CANCEL");
                    else LCD_Printf(BMC_NOTICE, 1, "");
                }
                full_reset_released_count++;
                Semaphore_post(POLLING_sem);
            }
            GPIO_Update(ports);
            delay(10);
            continue;
        }
        else full_reset_pressed_count = full_reset_released_count = number_pressed = 0;
        
        GPIO_RMW(PLLLOCK_LED_PORT, PLLLOCK_LED,
            (GPIO_Get(SOC_PLLLOCK_PORT, ports) & SOC_PLLLOCK) ?
            PLLLOCK_LED : 0, ports);
        
        GPIO_Update(ports);
        Semaphore_post(POLLING_sem);        
        delay(10);
    }
}
