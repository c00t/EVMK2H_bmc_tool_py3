/******************************************************************************
 *
 * File	Name:       spi_local.c
 *
 * Description: This device contains device specific information for SPI
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

#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/ssi.h"
#include "driverlib/rom.h"

#include "spi_driver.h"

#include "bmc_map.h"
#include "bmc.h"

extern GateMutexPri_Handle SPI0_Mutex;
extern GateMutexPri_Handle SPI1_Mutex;

extern Semaphore_Handle SPI0_Semaphore;
extern Semaphore_Handle SPI1_Semaphore;

const int num_spi_devices = 8;

SPI_DEVICE spi_devices[] = {
    { /* Clock Gen 1: SPI 0, CS 0 */
        SSI0_BASE,
        GPIO_PORTA_BASE,
        NULL,
        CLOCK_GEN_1,
        GPIO_PIN_3,
        &SPI0_Mutex,
        &SPI0_Semaphore
    },
    { /* Clock Gen 2: SPI 0, CS 1 */
        SSI0_BASE,
        GPIO_PORTC_BASE,
        NULL,
        CLOCK_GEN_2,
        GPIO_PIN_4,
        &SPI0_Mutex,
        &SPI0_Semaphore
    },
    { /* Clock Gen 3: SPI 0, CS 2 */
        SSI0_BASE,
        GPIO_PORTC_BASE,
        NULL,
        CLOCK_GEN_3,
        GPIO_PIN_5,
        &SPI0_Mutex,
        &SPI0_Semaphore
    },
    { /* GPIO Expander A: SPI 0, CS 3 */
        SSI0_BASE,
        GPIO_PORTC_BASE,
        NULL,
        GPIO_EXPANDER_A,
        GPIO_PIN_6,
        &SPI0_Mutex,
        &SPI0_Semaphore
    },
    { /* GPIO Expander B: SPI 0, CS 4 */
        SSI0_BASE,
        GPIO_PORTC_BASE,
        NULL,
        GPIO_EXPANDER_B,
        GPIO_PIN_7,
        &SPI0_Mutex,
        &SPI0_Semaphore
    },
    { /* GPIO Expander C: SPI 0, CS 5 */
        SSI0_BASE,
        GPIO_PORTF_BASE,
        NULL,
        GPIO_EXPANDER_C,
        GPIO_PIN_2,
        &SPI0_Mutex,
        &SPI0_Semaphore
    },
    { /* GPIO Expander D: SPI 0, CS 6 */
        SSI0_BASE,
        GPIO_PORTF_BASE,
        NULL,
        GPIO_EXPANDER_D,
        GPIO_PIN_3,
        &SPI0_Mutex,
        &SPI0_Semaphore
    },
    { /* LCD Display: SPI 1, CS 0 */
        SSI1_BASE,
        GPIO_PORTE_BASE,
        NULL,
        LCD_SPI,
        GPIO_PIN_1,
        &SPI1_Mutex,
        &SPI1_Semaphore
    }
};

void SPI_Setup()
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
    
    // SSI0
    GPIOPinConfigure(GPIO_CONFIG_SSI0CLK);
    GPIOPinConfigure(GPIO_CONFIG_SSI0RX);
    GPIOPinConfigure(GPIO_CONFIG_SSI0TX);

    // SSI1
    GPIOPinConfigure(GPIO_CONFIG_SSI1CLK);
    GPIOPinConfigure(GPIO_CONFIG_SSI1RX);
    GPIOPinConfigure(GPIO_CONFIG_SSI1TX);
    
    ROM_GPIOPinTypeSSI(GPIO_PORTA_BASE, (GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5));     // SPI0
    ROM_GPIOPinTypeSSI(GPIO_PORTE_BASE, (GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3));                         // SPI1
    
    ROM_SSIConfigSetExpClk(SSI0_BASE,                     // SPI0
                           SysCtlClockGet(),              // System Clock for SSI clock
                           SSI_FRF_MOTO_MODE_0,           // Data frame format (polarity 0, phase 0)
                           SSI_MODE_MASTER,               // Configure as Master
                           GPIOE_SPI_SPEED,               // Set speed
                           8);                            // Byte-long word length

    ROM_SSIConfigSetExpClk(SSI1_BASE,                     // SPI1
                           SysCtlClockGet(),              // System Clock for SSI clock
                           SSI_FRF_MOTO_MODE_0,           // Data frame format (polarity 0, phase 0)
                           SSI_MODE_MASTER,               // Configure as Master
                           LCD_SPI_SPEED,                 // Set speed
                           8);                            // Byte-long word length
    
    /* Set all CS pins to outputs and assert them */
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, (GPIO_PIN_3));
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, (GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7));
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, (GPIO_PIN_1));
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, (GPIO_PIN_2 | GPIO_PIN_3));
    
    
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, (GPIO_PIN_3), (GPIO_PIN_3));
    ROM_GPIOPinWrite(GPIO_PORTC_BASE, (GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7), (GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7));
    ROM_GPIOPinWrite(GPIO_PORTE_BASE, (GPIO_PIN_1), (GPIO_PIN_1));
    ROM_GPIOPinWrite(GPIO_PORTF_BASE, (GPIO_PIN_2 | GPIO_PIN_3), (GPIO_PIN_2 | GPIO_PIN_3));
    
    
    /* Enable SSIs */
    ROM_SSIEnable(SSI0_BASE);
    ROM_SSIEnable(SSI1_BASE);
}

const GateMutexPri_Handle *getSPIMutex(int base)
{
    const GateMutexPri_Handle *mutex;
    
    switch(base)
    {
        case 0:
            mutex = &SPI0_Mutex;
            break;
        case 1:
            mutex = &SPI1_Mutex;
            break;
        default:
            mutex = NULL;
    }
    
    return mutex;
}

const Semaphore_Handle *getSPISemaphore(int base)
{
    const Semaphore_Handle *semaphore;
    
    switch(base)
    {
        case 0:
            semaphore = &SPI0_Semaphore;
            break;
        case 1:
            semaphore = &SPI1_Semaphore;
            break;
        default:
            semaphore = NULL;
    }
    
    return semaphore;
}

void SPI_0_HWI()
{
    unsigned long ulMaskIntStatus;
    
    ulMaskIntStatus = ROM_SSIIntStatus(SSI0_BASE, true);
    
    ROM_SSIIntClear(SSI0_BASE, ulMaskIntStatus);
    ROM_SSIIntDisable(SSI0_BASE, SSI_TXFF);
    
    if(ulMaskIntStatus)
        Semaphore_post(SPI0_Semaphore);
}

void SPI_1_HWI()
{
    unsigned long ulMaskIntStatus;
    
    ulMaskIntStatus = ROM_SSIIntStatus(SSI1_BASE, true);
    
    ROM_SSIIntClear(SSI1_BASE, ulMaskIntStatus);
    ROM_SSIIntDisable(SSI1_BASE, SSI_TXFF);
    
    if(ulMaskIntStatus)
        Semaphore_post(SPI1_Semaphore);
}
