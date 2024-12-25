/******************************************************************************
 *
 * File	Name:       gpio_driver.h
 *
 * Description: Header file used for GPIO access.
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

#ifndef _GPIO_DRIVER_H_
#define _GPIO_DRIVER_H_

#include "bmc_map.h"
#include <ti/sysbios/gates/GateMutexPri.h>

#define EXPANDER(x) (x >= 0x10000)
#define EXPANDER_DEVICE(x) (unsigned char)(x >> 16)

typedef struct GPIO_PORT {
    const unsigned long key;
    const unsigned long port_base;
    unsigned int gateKey;
    const unsigned short donot_modify;
    unsigned short pin_dirs, pin_values;
    unsigned short dir_mask, val_mask;
    tBoolean dir_mod, val_mod;
    const GateMutexPri_Handle *mutex;
} GPIO_PORT;

extern void GPIO_Setup();

extern unsigned long GPIO_Start(unsigned long port_keys);
extern unsigned short GPIO_GetType(unsigned long port_key, unsigned long ports);
extern int GPIO_SetType(unsigned long port_key, unsigned short pin_mask, unsigned short inout_mask, unsigned long ports);
extern unsigned short GPIO_Get(unsigned long port_key, unsigned long ports);
extern int GPIO_RMW(unsigned long port_key, unsigned short pin_mask, unsigned short value, unsigned long ports);
extern int GPIO_Update(unsigned long ports);

#endif
