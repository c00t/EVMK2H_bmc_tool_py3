/******************************************************************************
 *
 * File	Name:       i2c_driver.c
 *
 * Description: This contains source code for controlling access to the I2C
 *              peripherals attached to the BMC microcontroller.
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

#include "i2c_driver.h"

#include "inc/hw_i2c.h"

#include "driverlib/i2c.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"

#define I2C_BUS_TIMEOUT                 5000
#define I2C_TIMEOUT                     250

extern const int num_i2c_ports;
extern I2C_PORT i2c_ports[];

static unsigned char waitForBus(I2C_PORT *port);
static unsigned char waitForMaster(I2C_PORT *port);

/*
 * Retrieves the I2C port indicated by the key given
 */
I2C_PORT *getPort(unsigned char port_key)
{
    I2C_PORT *port = NULL;
    int i;
    for(i = 0; i < num_i2c_ports; i++)
    {
        if(i2c_ports[i].key == port_key)
        {
            port = &i2c_ports[i];
            break;
        }
    }
    return port;
}

/*
 * Start function, enters the gate and prevents access of the specified
 * port by other threads. Also sets the address of the port.
 */
int I2C_Start(I2C_PORT *port)
{
    if(port == NULL)
        return -1;
        
    port->gateKey = GateMutexPri_enter(*port->mutex);
    
    return 0;
}

/*
 * Performs a series of operations indicated by operation.
 */
int I2C_Operation(I2C_OP **operation, int size, I2C_PORT *port)
{
    if(port == NULL || size <= 0)
        return -1;
    
    int i;
    unsigned long ulCmd, ulBase;
    I2C_OP *opPtr;
    
    opPtr = operation[0];
    ulBase = port->base;
    
    /* Enable the I2C */
    ROM_GPIOPinWrite(port->enable->gpio_port, port->enable->gpio_pin, port->enable->gpio_pin);
    
    /* Set address and initial operation */
    ROM_I2CMasterSlaveAddrSet(ulBase, port->address, opPtr->operand);
    
    /* wait for the bus to be ready */
    opPtr->error |= waitForBus(port);
    
    /* Need to send start command */
    ulCmd = I2C_MASTER_CMD_BURST_SEND_START;
    for(i = 0; i < size; i++)
    {
        opPtr = operation[i];
        if(opPtr->operand == I2C_READ)
        {
            if(HWREG(ulBase + I2C_O_MSA) & I2C_MSA_RS) // Former state was receive
            {
                /* Continue receiving */
                ulCmd |= I2C_MASTER_CMD_BURST_RECEIVE_CONT;
            }
            else // Fomer state was send
            {
                /* Switch to receive and restart */
                HWREG(ulBase + I2C_O_MSA) |= I2C_MSA_RS;
                ulCmd |= I2C_MASTER_CMD_BURST_RECEIVE_START;
            }
            // Final operation.
            if((i + 1) == size)
            {
                ulCmd |= 0x4;    // Set the stop bit.
                ulCmd &= ~(0x8); //Clear the Ack bit
            }
            /* Place command on register */
            ROM_I2CMasterControl(ulBase, ulCmd);
            
            /* Wait for master */
            opPtr->error |= waitForMaster(port);
            
            /* Check for I2C error */
            opPtr->error |= ROM_I2CMasterErr(ulBase);
            
            /* Get Result */
            opPtr->data = ROM_I2CMasterDataGet(ulBase);
        }
        else
        {
            if(HWREG(ulBase + I2C_O_MSA) & I2C_MSA_RS) // Former state was receive
            {
                /* Switch to send and restart */
                HWREG(ulBase + I2C_O_MSA) &= ~(I2C_MSA_RS);
                ulCmd |= I2C_MASTER_CMD_BURST_SEND_START;
            }
            else // Fomer state was send
            {
                /* Continue sending */
                ulCmd |= I2C_MASTER_CMD_BURST_SEND_CONT;
            }
            
            ulCmd |= ((i+1) == size) ? 0x4 : 0; // Set the stop bit if this is the last operation.
            
            /* Place data on I2C */
            ROM_I2CMasterDataPut(ulBase, opPtr->data);
            
            /* Send command */
            ROM_I2CMasterControl(ulBase, ulCmd);
            
            /* Wait for send */
            opPtr->error |= waitForMaster(port);
            
            /* Get I2C Errors */
            opPtr->error |= ROM_I2CMasterErr(ulBase);
        }
        ulCmd = 0;
    }
    
    /* Disable the I2C */
    ROM_GPIOPinWrite(port->enable->gpio_port, port->enable->gpio_pin, 0);
    
    return 0;
}

/*
 * Stops access to the I2C Port.
 */
int I2C_Stop(I2C_PORT *port)
{
    if(port == NULL)
        return -1;
    
    GateMutexPri_leave(*port->mutex, port->gateKey);
    port = NULL;
    
    return 0;
}

/* Local functions */

static unsigned char waitForBus(I2C_PORT *port)
{
    int busy = 0;
    while(ROM_I2CMasterBusBusy(port->base) && busy++ < I2C_BUS_TIMEOUT);
    return (busy >= I2C_BUS_TIMEOUT) ? 1 : 0;
}

static unsigned char waitForMaster(I2C_PORT *port)
{
    int busy = 0;
    while(ROM_I2CMasterBusy(port->base) && busy++ < I2C_BUS_TIMEOUT);
    return (busy >= I2C_BUS_TIMEOUT) ? (1 << 1) : 0;
    // I2CMasterIntEnable(port->base);
    // if(!Semaphore_pend(*port->semaphore, I2C_TIMEOUT))
    // {
        // I2CMasterIntDisable(port->base);
        // return (1 << 1);
    // }
    // return 0;
}
