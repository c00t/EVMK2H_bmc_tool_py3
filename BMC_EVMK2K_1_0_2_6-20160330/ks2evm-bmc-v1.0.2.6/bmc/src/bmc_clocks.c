/******************************************************************************
 *
 * File	Name:       bmc_clocks.c
 *
 * Description: This contains the internal and external functions for the BMC
 *              clock modules.
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
 
#include "bmc_clocks.h"
#include "spi_driver.h"

/* ============================== Local Functions ====================== */

/*
 * Writes the given data to the specified clock register
 */
void CLOCK_RegWrite(unsigned char select, unsigned int address, unsigned short data)
{
    SPI_DEVICE *clock_gen;
    unsigned char write[4], read[4];
    
    /* Fill in write buffer */
    write[0] = (unsigned char)((address & 0x0F00) >> 8);
    write[1] = (unsigned char)(address & 0x00FF);
    write[2] = (unsigned char)((data & 0xFF00) >> 8);
    write[3] = (unsigned char)(data & 0x00FF);
    
    /* Get the device */
    clock_gen = getSPIDevice(select);

    /* SPI operation */
    SPI_Start(clock_gen);
    SPI_RW(write, read, 4, clock_gen);
    SPI_Stop(clock_gen);
}

/*
 * Reads the data from the specified clock register and returns the data stored there
 */
unsigned short CLOCK_RegRead(unsigned char select, unsigned int address)
{
    SPI_DEVICE *clock_gen;
    unsigned char write[4], read[4] = {0, 0, 0, 0};
    
    /* Fill in write buffer */
    write[0] = (unsigned char)(((address & 0x0F00) | 0x8000) >> 8);
    write[1] = (unsigned char)(address & 0x00FF);
    write[2] = 0;
    write[3] = 0;
    
    /* Get the device */
    clock_gen = getSPIDevice(select);

    /* SPI operation */
    SPI_Start(clock_gen);
    SPI_RW(write, read, 4, clock_gen);
    SPI_Stop(clock_gen);
    
    /* Return the result */
    return ((((unsigned short)read[2]) << 8) | ((unsigned short)read[3]));
}
