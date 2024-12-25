/******************************************************************************
 *
 * File	Name:       gpio_driver.c
 *
 * Description: This contains source code for controlling access to the GPIO
 *              ports on the BMC microcontroller.
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

#include "gpio_driver.h"
#include "spi_driver.h"

#include "inc/hw_gpio.h"

#include "driverlib/gpio.h"
#include "driverlib/rom.h"

//Register Addresses (IOCON.BANK = 0)                                                            Default Value
#define GPIOE_IODIRA       0x00                      // I/O Direction for Port A                   1111 1111
#define GPIOE_IODIRB       0x01                      // I/O Direction for Port B                   1111 1111
#define GPIOE_IPOLA        0x02                      // Polarity of Port A                         0000 0000
#define GPIOE_IPOLB        0x03                      // Polarity of Port B                         0000 0000
#define GPIOE_GPINTENA     0x04                      // Interrupt-on-change enable Port A          0000 0000
#define GPIOE_GPINTENB     0x05                      // Interrupt-on-change enable Port B          0000 0000
#define GPIOE_DEFVALA      0x06                      // Default value reg Port A                   0000 0000
#define GPIOE_DEFVALB      0x07                      // Default value reg Port B                   0000 0000
#define GPIOE_INTCONA      0x08                      // Interrupt-on-change control reg Port A     0000 0000
#define GPIOE_INTCONB      0x09                      // Interrupt-on-change control reg Port B     0000 0000
#define GPIOE_IOCON        0x0A                      // Configuration Register                     0000 0000
#define GPIOE_GPPUA        0x0C                      // Pull-up resistor config Port A             0000 0000
#define GPIOE_GPPUB        0x0D                      // Pull-up resistor config Port B             0000 0000
#define GPIOE_INTFA        0x0E                      // Interrupt flag reg Port A                  0000 0000
#define GPIOE_INTFB        0x0F                      // Interrupt flag reg Port B                  0000 0000
#define GPIOE_INTCAPA      0x10                      // Interrupt captured value Port A            0000 0000
#define GPIOE_INTCAPB      0x11                      // Interrupt captured value Port B            0000 0000
#define GPIOE_GPIOA        0x12                      // GPIO reg for Port A                        0000 0000
#define GPIOE_GPIOB        0x13                      // GPIO reg for Port B                        0000 0000
#define GPIOE_OLATA        0x14                      // Latch reg for Port A                       0000 0000
#define GPIOE_OLATB        0x15                      // Latch reg for Port B                       0000 0000

#define GPIOE_READ         0x41                      // Sequence to signal a READ command to GPIO expander
#define GPIOE_WRITE        0x40                      // Sequence to signal a WRITE command to GPIO expander

extern const int num_gpio_ports;
extern GPIO_PORT gpio_ports[];

static unsigned short GPIOE_Read(unsigned long spi_key);
static int GPIOE_Write(unsigned long spi_key, unsigned short value);
// static unsigned short GPIOE_GetType(unsigned long spi_key);
static int GPIOE_SetType(unsigned long spi_key, unsigned short inout_mask);

/*
 * Gets the port in ports based on port_keys.
 */
static GPIO_PORT *getPort(unsigned long port_key)
{
    int i;
    GPIO_PORT *port;
    
    port = NULL;
    for(i = 0; i < num_gpio_ports; i++)
    {
        if(gpio_ports[i].key == port_key)
        {
            port = &gpio_ports[i];
            break;
        }
    }
    return port;
}

/*
 * Starts the GPIO access to all listed ports.
 */
unsigned long GPIO_Start(unsigned long port_keys)
{
    int i;
    GPIO_PORT *port;
    
    for(i = 0; i < num_gpio_ports; i++)
    {
        if(gpio_ports[i].key & port_keys)
        {
            port = &gpio_ports[i];
            port->gateKey = GateMutexPri_enter(*(port->mutex));
            port->dir_mod = false;
            port->val_mod = false;
            if(EXPANDER(port->key))
            {
                port->pin_values = GPIOE_Read(port->port_base);
            }
            else
            {
                port->pin_values = (unsigned short)ROM_GPIOPinRead(port->port_base, (unsigned char)0xFF);
            }
        }
    }
    
    return port_keys;
}

/*
 * Returns the pin direction (intput = 0, output = 1) of all pins on the given hardware port.
 */
unsigned short GPIO_GetType(unsigned long port_key, unsigned long ports)
{
    if(!(ports & port_key))
        return 0;
    
    GPIO_PORT *port;
    
    port = getPort(port_key);
    if(!port)
        return 0;
    
    return port->pin_dirs;
}

/*
 * Sets the type (input = 0, output = 1) of the given port based on the pin_mask.
 */
int GPIO_SetType(unsigned long port_key, unsigned short pin_mask, unsigned short inout_mask, unsigned long ports)
{
    if(!(ports & port_key))
        return -1;
    
    unsigned short nomod_mask;
    GPIO_PORT *port;
    
    port = getPort(port_key);
    if(!port)
        return -2;
    
    nomod_mask = pin_mask & ~(port->donot_modify);
    port->dir_mask |= nomod_mask;
    port->pin_dirs &= ~(nomod_mask);
    inout_mask &= nomod_mask;
    port->pin_dirs |= inout_mask;
    
    port->dir_mod = true;
    
    return 0;
}

/*
 * Returns the current values of the pins on the given port.
 */
unsigned short GPIO_Get(unsigned long port_key, unsigned long ports)
{
    if(!(ports & port_key))
        return 0;
    
    GPIO_PORT *port;
    
    port = getPort(port_key);
    if(!port)
        return 0;
    
    return port->pin_values;
}

/*
 * Read, modify, write. Sets the current pins on the specified port using pin_mask and value.
 */
int GPIO_RMW(unsigned long port_key, unsigned short pin_mask, unsigned short value, unsigned long ports)
{
    if(!(ports & port_key))
        return -1;
    
    unsigned short nomod_mask;
    GPIO_PORT *port;
    
    port = getPort(port_key);
    if(!port)
        return -2;
    
    nomod_mask = pin_mask & ~(port->donot_modify);
    port->val_mask |= nomod_mask;
    port->pin_values &= ~(nomod_mask);
    value &= nomod_mask;
    port->pin_values |= value;
    
    port->val_mod = true;
    
    return 0;
}

/*
 * Places the values set in RMW on the pins and releases control of the port
 */
int GPIO_Update(unsigned long ports)
{
    if(!ports)
        return -1;
    
    int i;
    GPIO_PORT *port;
    
    for(i = 0; i < num_gpio_ports; i++)
    {
        if(gpio_ports[i].key & ports)
        {
            port = &gpio_ports[i];
            if(EXPANDER(port->key))
            {
                if(port->dir_mod)
                    GPIOE_SetType(port->port_base, port->pin_dirs);
                if(port->val_mod)
                    GPIOE_Write(port->port_base, port->pin_values);
            }
            else
            {
                if(port->dir_mod)
                {
                    ROM_GPIOPinTypeGPIOOutput(port->port_base, (unsigned char)(port->pin_dirs & port->dir_mask));
                    ROM_GPIOPinTypeGPIOInput(port->port_base, (unsigned char)(~(port->pin_dirs) & port->dir_mask));
                }
                if(port->val_mod)
                    ROM_GPIOPinWrite(port->port_base, port->val_mask, port->pin_values);
            }
            port->pin_values = 0;
            port->dir_mod = false;
            port->val_mod = false;
            port->val_mask = 0;
            port->dir_mask = 0;
            GateMutexPri_leave(*port->mutex, port->gateKey);
        }
    }
    ports = 0;
    
    return 0;
}

/* Used for expander reads */
static unsigned short GPIOE_Read(unsigned long spi_key)
{
    SPI_DEVICE *device;
    unsigned char write[4] = {GPIOE_READ, GPIOE_GPIOA, 0, 0};
    unsigned char read[4];
    
    device = getSPIDevice(spi_key);
    
    if(device == NULL)
        return 0;
    
    SPI_Start(device);

    SPI_RW(write, read, 4, device);
    
    SPI_Stop(device);
    
    return ((((unsigned short)read[3]) << 8) | (unsigned short)read[2]);
}

/* Used for expander writes */
static int GPIOE_Write(unsigned long spi_key, unsigned short value)
{
    SPI_DEVICE *device;
    unsigned char write[4] = {GPIOE_WRITE, GPIOE_GPIOA, 0, 0};
    unsigned char read[4];
    
    write[2] = (unsigned char)(value & 0xFF);
    write[3] = (unsigned char)((value & 0xFF00) >> 8);
    
    device = getSPIDevice(spi_key);

    if(device == NULL)
        return 0;

    SPI_Start(device);
    
    SPI_RW(write, read, 4, device);
    
    SPI_Stop(device);
    
    return 0;
}

/* Used to get the types of all pins on an expander - Not actually used by driver*/
/* static unsigned short GPIOE_GetType(unsigned long spi_key)
{
    SPI_DEVICE *device;
    unsigned char write[4] = {GPIOE_READ, GPIOE_IODIRA, 0, 0};
    unsigned char read[4];
    
    device = getSPIDevice(spi_key);

    if(device == NULL)
        return 0;

    SPI_Start(device);
    
    SPI_RW(write, read, 4, device);
    
    SPI_Stop(device);
    
    return ((((unsigned short)read[3]) << 8) | (unsigned short)read[2]);
} */

/* Sets the type of all the pins on an expander */
static int GPIOE_SetType(unsigned long spi_key, unsigned short inout_mask)
{
    SPI_DEVICE *device;
    unsigned char write[4] = {GPIOE_WRITE, GPIOE_IODIRA, 0, 0};
    unsigned char read[4];
    
    inout_mask = ~inout_mask;
    
    write[2] = (unsigned char)(inout_mask & 0xFF);
    write[3] = (unsigned char)((inout_mask & 0xFF00) >> 8);

    device = getSPIDevice(spi_key);

    if(device == NULL)
        return 0;

    SPI_Start(device);
    
    SPI_RW(write, read, 4, device);
    
    SPI_Stop(device);
    
    return 0;
}
