/******************************************************************************
 *
 * File	Name:       i2c_driver.h
 *
 * Description: Header file used for I2C peripheral access.
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

#ifndef _I2C_DRIVER_H_
#define _I2C_DRIVER_H_

#include "bmc_map.h"
#include <ti/sysbios/gates/GateMutexPri.h>
#include <ti/sysbios/knl/Semaphore.h>

#define I2C_MASTER_ERROR_NONE       0X00
#define I2C_MASTER_BUS_TIMEOUT_ERR  0x01
#define I2C_MASTER_TIMEOUT_ERR      0x02
#define I2C_MASTER_ERROR_ADDR_ACK   0X04
#define I2C_MASTER_ERROR_DATA_ACK   0X08
#define I2C_MASTER_ERROR_ARB_LOST   0X10

typedef struct I2C_ENABLE {
    const unsigned long gpio_port;
    const unsigned char gpio_pin;
} I2C_ENABLE;

typedef struct I2C_PORT {
    unsigned long base;
    unsigned int gateKey;
    const unsigned char key;
    unsigned char address;
    const I2C_ENABLE *enable;
    const GateMutexPri_Handle *mutex;
    const Semaphore_Handle *semaphore;
} I2C_PORT;

typedef struct I2C_OP {
    enum OPERAND { I2C_WRITE=0, I2C_READ=1 } operand;
    unsigned char data;
    unsigned char error;
} I2C_OP;

extern void I2C_Setup();

extern const I2C_ENABLE *getI2CEnable(int base, unsigned char address);
extern const GateMutexPri_Handle *getI2CMutex(int base);
extern const Semaphore_Handle *getI2CSemaphore(int base);
extern unsigned long getI2CBase(int base);

extern I2C_PORT *getPort(unsigned char port_key);
extern int I2C_Start(I2C_PORT *port);
extern int I2C_Operation(I2C_OP **operation, int size, I2C_PORT *port);
extern int I2C_Stop(I2C_PORT *port);

#endif
