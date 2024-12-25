/******************************************************************************
 *
 * File	Name:       i2c_local.c
 *
 * Description: This device contains device specific information for I2C
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
#include "driverlib/i2c.h"
#include "driverlib/rom.h"

#include "i2c_driver.h"

#include "bmc_map.h"
#include "bmc.h"

#define SOC_I2C_ADDR        0x00
#define EEPROM50_ADDR       0x50
#define EEPROM51_ADDR       0x51
#define PM_BUS_ADDR_9090    0x68
#define PM_BUS_ADDR_9244_0  0x34
#define PM_BUS_ADDR_9244_1  0x4E

extern GateMutexPri_Handle I2C1_Mutex;

extern Semaphore_Handle I2C1_Semaphore;

const int num_i2c_ports = 6;

const I2C_ENABLE soc_i2c_enable = {
    GPIO_PORTE_BASE,
    GPIO_PIN_4
};

const I2C_ENABLE pw_seq_i2c_enable = {
	GPIO_PORTE_BASE,
    GPIO_PIN_5
};

I2C_PORT i2c_ports[] = {
    { /* SOC I2C */
        I2C1_MASTER_BASE,
        NULL,
        SOC_I2C,
        SOC_I2C_ADDR,
        &soc_i2c_enable,
        &I2C1_Mutex,
        &I2C1_Semaphore
    },
    { /* EEPROM50 */
        I2C1_MASTER_BASE,
        NULL,
        EEPROM50,
        EEPROM50_ADDR,
        &soc_i2c_enable,
        &I2C1_Mutex,
        &I2C1_Semaphore
    },
    { /* EEPROM51 */
        I2C1_MASTER_BASE,
        NULL,
        EEPROM51,
        EEPROM51_ADDR,
        &soc_i2c_enable,
        &I2C1_Mutex,
        &I2C1_Semaphore
    },
    { /* PM BUS (UCD9090) */
        I2C1_MASTER_BASE,
        NULL,
        UCD_9090,
        PM_BUS_ADDR_9090,
        &pw_seq_i2c_enable,
        &I2C1_Mutex,
        &I2C1_Semaphore
    },
    { /* PM BUS (UCD9244_0) */
        I2C1_MASTER_BASE,
        NULL,
        UCD_9244_0,
        PM_BUS_ADDR_9244_0,
        &pw_seq_i2c_enable,
        &I2C1_Mutex,
        &I2C1_Semaphore
    },
    { /* PM BUS (UCD9244_1) */
        I2C1_MASTER_BASE,
        NULL,
        UCD_9244_1,
        PM_BUS_ADDR_9244_1,
        &pw_seq_i2c_enable,
        &I2C1_Mutex,
        &I2C1_Semaphore
    }
};

void I2C_Setup()
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);     // I2C0
    
    // I2C1
    GPIOPinConfigure(GPIO_CONFIG_I2C1SCL);
    GPIOPinConfigure(GPIO_CONFIG_I2C1SDA);
    
    ROM_GPIOPinTypeI2C(GPIO_PORTA_BASE, (GPIO_PIN_6 | GPIO_PIN_7));                                     // I2C1
    
    ROM_I2CMasterInitExpClk(I2C1_MASTER_BASE,          // SOC/EEPROM/PMBUS Master I2C
                            ((SysCtlClockGet()/80)*120),  // System Clock for I2C clock
                            false);                       // Not setup for fast data transfers (transfer rate of 100 kbps)
    
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, (GPIO_PIN_4 | GPIO_PIN_5));
    
    ROM_GPIOPinWrite(GPIO_PORTE_BASE, (GPIO_PIN_4 | GPIO_PIN_5), 0);
    
    /* Enable I2C */
    ROM_I2CMasterEnable(I2C1_MASTER_BASE);
}

const I2C_ENABLE *getI2CEnable(int base, unsigned char address)
{
    const I2C_ENABLE *enable;
    
    switch(base)
    {
        case 1:
            if(address == PM_BUS_ADDR_9090   ||
               address == PM_BUS_ADDR_9244_0 ||
               address == PM_BUS_ADDR_9244_1)
            {
                enable = &pw_seq_i2c_enable;
            }
            else enable = &soc_i2c_enable; 
            break;
        default:
            enable = NULL;
    }
    
    return enable;
}

const GateMutexPri_Handle *getI2CMutex(int base)
{
    const GateMutexPri_Handle *mutex;
    
    switch(base)
    {
        case 1:
            mutex = &I2C1_Mutex;
            break;
        default:
            mutex = NULL;
    }
    
    return mutex;
}

const Semaphore_Handle *getI2CSemaphore(int base)
{
    const Semaphore_Handle *semaphore;
    
    switch(base)
    {
        case 1:
            semaphore = &I2C1_Semaphore;
            break;
        default:
            semaphore = NULL;
    }
    
    return semaphore;
}

unsigned long getI2CBase(int base)
{
    unsigned long i2c_base;
    
    switch(base)
    {
        case 1:
            i2c_base = I2C1_MASTER_BASE;
            break;
        default:
            i2c_base = 0;
    }
    
    return i2c_base;
}

void I2C_1_HWI()
{
    tBoolean ulMaskIntStatus;
    
    ulMaskIntStatus = ROM_I2CMasterIntStatus(I2C1_MASTER_BASE, true);
    
    ROM_I2CMasterIntClear(I2C1_MASTER_BASE);
    ROM_I2CMasterIntDisable(I2C1_MASTER_BASE);
    
    if(ulMaskIntStatus)
        Semaphore_post(I2C1_Semaphore);
}
