/******************************************************************************
 *
 * File	Name:       spi_driver.h
 *
 * Description: Header file used for SPI peripheral access.
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
 
#ifndef _SPI_DRIVER_H_
#define _SPI_DRIVER_H_

#include "bmc_map.h"
#include <ti/sysbios/gates/GateMutexPri.h>
#include <ti/sysbios/knl/Semaphore.h>

typedef struct SPI_DEVICE {
    const unsigned long spi_base, cs_base;
    unsigned int gateKey;
    const unsigned char key;
    const unsigned char cs_pin;
    const GateMutexPri_Handle *mutex;
    const Semaphore_Handle *semaphore;
} SPI_DEVICE;

extern void SPI_Setup();

extern const GateMutexPri_Handle *getSPIMutex(int base);
extern const Semaphore_Handle *getSPISemaphore(int base);

extern SPI_DEVICE *getSPIDevice(unsigned char device_key);
extern int SPI_Start(SPI_DEVICE *device);
extern int SPI_RW(unsigned char *write, unsigned char *read, int size, SPI_DEVICE *device);
extern int SPI_Stop(SPI_DEVICE *device);

#endif
