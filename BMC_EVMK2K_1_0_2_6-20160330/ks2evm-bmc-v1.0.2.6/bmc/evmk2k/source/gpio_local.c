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
#include "driverlib/uart.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"

#include "gpio_driver.h"

#include "bmc_map.h"
#include "bmc.h"

extern GateMutexPri_Handle GPIOA_Mutex;
extern GateMutexPri_Handle GPIOB_Mutex;
extern GateMutexPri_Handle GPIOC_Mutex;
extern GateMutexPri_Handle GPIOD_Mutex;
extern GateMutexPri_Handle GPIOE_Mutex;
extern GateMutexPri_Handle GPIOF_Mutex;
extern GateMutexPri_Handle GPIOG_Mutex;
extern GateMutexPri_Handle GPIOH_Mutex;
extern GateMutexPri_Handle GPIOJ_Mutex;

extern GateMutexPri_Handle GPIOXA_Mutex;
extern GateMutexPri_Handle GPIOXB_Mutex;
extern GateMutexPri_Handle GPIOXC_Mutex;
extern GateMutexPri_Handle GPIOXD_Mutex;

const int num_gpio_ports = 13;

GPIO_PORT gpio_ports[] = {
    { /* GPIO Port A */
        GPIO_PORT_A,
        GPIO_PORTA_BASE,
        NULL,
        (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3),
        (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3), 0,
        0, 0,
        false, false,
        &GPIOA_Mutex
    },
    { /* GPIO Port B */
        GPIO_PORT_B,
        GPIO_PORTB_BASE,
        NULL,
        0,
        0, 0,
        0, 0,
        false, false,
        &GPIOB_Mutex
    },
    { /* GPIO Port C */
        GPIO_PORT_C,
        GPIO_PORTC_BASE,
        NULL,
        (GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7),
        (GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7), 0,
        0, 0,
        false, false,
        &GPIOC_Mutex
    },
    { /* GPIO Port D */
        GPIO_PORT_D,
        GPIO_PORTD_BASE,
        NULL,
        0,
        0, 0,
        0, 0,
        false, false,
        &GPIOD_Mutex
    },
    { /* GPIO Port E */
        GPIO_PORT_E,
        GPIO_PORTE_BASE,
        NULL,
        (GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5),
        (GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5), 0,
        0, 0,
        false, false,
        &GPIOE_Mutex
    },
    { /* GPIO Port F */
        GPIO_PORT_F,
        GPIO_PORTF_BASE,
        NULL,
        (GPIO_PIN_2 | GPIO_PIN_3),
        (GPIO_PIN_2 | GPIO_PIN_3), 0,
        0, 0,
        false, false,
        &GPIOF_Mutex
    },
    { /* GPIO Port G */
        GPIO_PORT_G,
        GPIO_PORTG_BASE,
        NULL,
        0,
        0, 0,
        0, 0,
        false, false,
        &GPIOG_Mutex
    },
    { /* GPIO Port H */
        GPIO_PORT_H,
        GPIO_PORTH_BASE,
        NULL,
        0,
        0, 0,
        0, 0,
        false, false,
        &GPIOH_Mutex
    },
    { /* GPIO Port J */
        GPIO_PORT_J,
        GPIO_PORTJ_BASE,
        NULL,
        0,
        0, 0,
        0, 0,
        false, false,
        &GPIOJ_Mutex
    },
    { /* GPIO Port XA */
        GPIO_PORT_XA,
        GPIO_EXPANDER_A,
        NULL,
        0,
        0, 0,
        0, 0,
        false, false,
        &GPIOXA_Mutex
    },
    { /* GPIO Port XB */
        GPIO_PORT_XB,
        GPIO_EXPANDER_B,
        NULL,
        0,
        0, 0,
        0, 0,
        false, false,
        &GPIOXB_Mutex
    },
    { /* GPIO Port XC */
        GPIO_PORT_XC,
        GPIO_EXPANDER_C,
        NULL,
        0,
        0, 0,
        0, 0,
        false, false,
        &GPIOXC_Mutex
    },
    { /* GPIO Port XA */
        GPIO_PORT_XD,
        GPIO_EXPANDER_D,
        NULL,
        0,
        0, 0,
        0, 0,
        false, false,
        &GPIOXD_Mutex
    }
};

void GPIO_Setup()
{
    /* GPIO Port enables */
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);    // GPIO port A
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);    // GPIO port B
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);    // GPIO port C
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);    // GPIO port D
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);    // GPIO port E
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);    // GPIO port F
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);    // GPIO port G
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);    // GPIO port H
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);    // GPIO port J
    
    /* Set all pins to Input initially */
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, (unsigned char)ALL_PINS);
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, (unsigned char)ALL_PINS);
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, (unsigned char)ALL_PINS);
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, (unsigned char)ALL_PINS);
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, (unsigned char)ALL_PINS);
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, (unsigned char)ALL_PINS);
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTG_BASE, (unsigned char)ALL_PINS);
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTH_BASE, (unsigned char)ALL_PINS);
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, (unsigned char)ALL_PINS);
    
    /* Set signal strengths */
    ROM_GPIOPadConfigSet(GPIO_PORTA_BASE, (unsigned char)(ALL_PINS) & ~(I2C1_SCL | I2C1_SDA), GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);
    ROM_GPIOPadConfigSet(GPIO_PORTB_BASE, (unsigned char)(ALL_PINS) & ~(I2C0_SCL | I2C0_SDA), GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);
    ROM_GPIOPadConfigSet(GPIO_PORTC_BASE, (unsigned char)(ALL_PINS), GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);
    ROM_GPIOPadConfigSet(GPIO_PORTD_BASE, (unsigned char)(ALL_PINS), GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);
    ROM_GPIOPadConfigSet(GPIO_PORTE_BASE, (unsigned char)(ALL_PINS), GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);
    ROM_GPIOPadConfigSet(GPIO_PORTF_BASE, (unsigned char)(ALL_PINS), GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);
    ROM_GPIOPadConfigSet(GPIO_PORTG_BASE, (unsigned char)(ALL_PINS), GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);
    ROM_GPIOPadConfigSet(GPIO_PORTH_BASE, (unsigned char)(ALL_PINS), GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);
    ROM_GPIOPadConfigSet(GPIO_PORTJ_BASE, (unsigned char)(ALL_PINS), GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);
    
    /* UART Setup */
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);    // UART0
    
    // UART0
    ROM_GPIOPinConfigure(GPIO_CONFIG_U0RX);
    ROM_GPIOPinConfigure(GPIO_CONFIG_U0TX);
    
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, (CONSOLE_UART_RX | CONSOLE_UART_TX));
    
    ROM_UARTConfigSetExpClk(CONSOLE_UART_BASE,            // CONSOLE UART
                            SysCtlClockGet(),             // System Clock for UART Clock
                            BAUD_RATE,                    // BAUD RATE
                            (UART_CONFIG_WLEN_8    |      // 8 bit word length
                             UART_CONFIG_STOP_ONE  |      // 1 stop bit
                             UART_CONFIG_PAR_NONE));      // no parity bit

    ROM_UARTFIFOLevelSet(CONSOLE_UART_BASE, NULL, UART_FIFO_RX1_8);
    
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);    // UART1
    
    // UART1
    ROM_GPIOPinConfigure(GPIO_CONFIG_U1RX);
    ROM_GPIOPinConfigure(GPIO_CONFIG_U1TX);
    
    ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, (SOC_UART_RX | SOC_UART_TX));
    
    ROM_UARTConfigSetExpClk(SOC_UART_BASE,                // SOC UART
                            SysCtlClockGet(),             // System Clock for UART Clock
                            BAUD_RATE,                    // BAUD RATE
                            (UART_CONFIG_WLEN_8    |      // 8 bit word length
                             UART_CONFIG_STOP_ONE  |      // 1 stop bit
                             UART_CONFIG_PAR_NONE));      // no parity bit
    
    ROM_UARTFIFOLevelSet(SOC_UART_BASE, NULL, UART_FIFO_RX1_8);

    /* Enable UART */
    ROM_UARTEnable(CONSOLE_UART_BASE);
    ROM_UARTEnable(SOC_UART_BASE);
}
