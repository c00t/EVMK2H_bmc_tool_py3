/******************************************************************************
 *
 * File	Name:       spi_driver.c
 *
 * Description: This contains source code for controlling access to the SPI
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

#include "spi_driver.h"

#include "inc/hw_ssi.h"

#include "driverlib/gpio.h"
#include "driverlib/ssi.h"
#include "driverlib/rom.h"

#define SPI_TIMEOUT                 250
#define SPI_RX_TIMEOUT              5000

extern const int num_spi_devices;
extern SPI_DEVICE spi_devices[];

/* Local function used to get the device based on the key given */
SPI_DEVICE *getSPIDevice(unsigned char device_key)
{
    SPI_DEVICE *device = NULL;
    int i;
    for(i = 0; i < num_spi_devices; i++)
    {
        if(spi_devices[i].key == device_key)
        {
            device = &spi_devices[i];
            break;
        }
    }
    return device;
}

/*
 * Start function, finds the device being asked for and enters
 * the gate for that SPI port. Once this function is used no
 * other thread may access the same SPI port until SPI_Stop
 * is called.
 */
int SPI_Start(SPI_DEVICE *device)
{
    if(device == NULL)
        return -1;
    
    device->gateKey = GateMutexPri_enter(*device->mutex);
    
    return 0;
}

/*
 * Performs SPI read/write sequence based on size. Write should contain a series
 * of bytes to send; received values will be stored in read.
 */
int SPI_RW(unsigned char *write, unsigned char *read, int size, SPI_DEVICE *device)
{
    if(device == NULL)
        return -1;
    
    int i, count;
    unsigned long receive;
    
    ROM_GPIOPinWrite(device->cs_base, device->cs_pin, 0);
    
    while(ROM_SSIDataGetNonBlocking(device->spi_base, &receive));
    
    for(i = 0; i < size; i++)
    {
        ROM_SSIIntEnable(device->spi_base, SSI_TXFF);
        if(!Semaphore_pend(*device->semaphore, SPI_TIMEOUT))
        {
            ROM_SSIIntDisable(device->spi_base, SSI_TXFF);
            ROM_GPIOPinWrite(device->cs_base, device->cs_pin,  device->cs_pin);
            return -2;
        }
        ROM_SSIDataPut(device->spi_base, write[i]);
        
        count = 0;
        while(!(ROM_SSIDataGetNonBlocking(device->spi_base, &receive)) && count++ < SPI_RX_TIMEOUT);
        if(count < SPI_RX_TIMEOUT)
        {
            read[i] = (unsigned char) receive;
        }
    }
    ROM_GPIOPinWrite(device->cs_base, device->cs_pin,  device->cs_pin);
    
    return 0;
}

/*
 * Stops access to the specified spi device.
 * Leaves the gate and prevents future access to the device
 */
int SPI_Stop(SPI_DEVICE *device)
{
    if(device == NULL)
        return -1;
    
    GateMutexPri_leave(*device->mutex, device->gateKey);
    device = NULL;
    
    return 0;
}
