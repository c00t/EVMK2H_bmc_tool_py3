/******************************************************************************
 *
 * File	Name:       bmc_clk_gen.c
 *
 * Description: This contains a lookup table used for the clock generator registers.
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

const int clk_gen_amount = 3;

const int clk_gen_reg_amount = 21;

const unsigned short clkgen1[] = {
/*         0       1       2       3       4       5       6       7       8       9  */
/*0x*/   0x0009, 0x0000, 0x001F, 0x00F5, 0x30AF, 0x0023, 0x0004, 0x0023, 0x0004, 0x0003,
/*1x*/   0x01F0, 0x0000, 0x0003, 0x01F0, 0x0000, 0x0001, 0x0000, 0x0000, 0x0001, 0x0000,
/*2x*/   0x0000
};

const unsigned short clkgen1_10MHz[] = {
/*         0       1       2       3       4       5       6       7       8       9  */
/*0x*/   0x01B8, 0x0060, 0x05FF, 0x00F5, 0x20Af, 0x0023, 0x0004, 0x0023, 0x0004, 0x0003,
/*1x*/   0x01F0, 0x0000, 0x0003, 0x01F0, 0x0000, 0x0001, 0x0000, 0x0000, 0x0001, 0x0000,
/*2x*/   0x0000
};

const unsigned short clkgen2[] = {
/*         0       1       2       3       4       5       6       7       8       9  */
/*0x*/   0x03F1, 0x017C, 0x187C, 0x00F5, 0x20AF, 0x0023, 0x0003, 0x0001, 0x0031, 0x0203,
/*1x*/   0x01E4, 0x0000, 0x0001, 0x0000, 0x0000, 0x0001, 0x0000, 0x0000, 0x0011, 0x0310,
/*2x*/   0x0000
};

const unsigned short clkgen3[] = {
/*         0       1       2       3       4       5       6       7       8       9  */
/*0x*/   0x03F1, 0x0000, 0x0018, 0x00F5, 0x306F, 0x0023, 0x0001, 0x0023, 0x0004, 0x0203,
/*1x*/   0x0022, 0x0000, 0x0203, 0x0022, 0x0000, 0x0203, 0x0022, 0x0000, 0x0203, 0x0022,
/*2x*/   0x0000
};

const unsigned short *clkgen[] = {
    clkgen1,
    clkgen1_10MHz,
    clkgen2,
    clkgen3
};
