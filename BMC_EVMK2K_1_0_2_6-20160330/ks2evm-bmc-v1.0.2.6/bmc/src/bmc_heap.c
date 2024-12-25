/******************************************************************************
 *
 * File    Name:       bmc_heap.c
 *
 * Description: This contains functions to allocate and free memory from the BMC
 *              Heap.
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

 #include <string.h>
 
#include <ti/sysbios/heaps/HeapMultiBuf.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/Error.h>

#include "bmc_heap.h"

extern HeapMultiBuf_Handle bmc_heap;
static Memory_Stats bmc_heap_stats;
static Memory_Stats *stat_ptr = &bmc_heap_stats;
static Error_Block eb;

void BMC_Heap_Init()
{
    Error_init(&eb);
}

void* BMC_Alloc(SizeT size, SizeT align)
{
    HeapMultiBuf_getStats(bmc_heap, stat_ptr);
    if(size > stat_ptr->largestFreeSize)
        return NULL;
    
    return HeapMultiBuf_alloc(bmc_heap, size, align, &eb);
}

void* BMC_Calloc(SizeT num, SizeT size, SizeT align)
{
    void *ptr;
    SizeT real_size = num*size;
    
    /* Allocate block */
    ptr = BMC_Alloc(real_size, align);
    if (ptr == NULL) return NULL;
    
    /* Set all values to NULL */
    memset(ptr, NULL, num);
    
    return ptr;
}

void BMC_Free(void* block, SizeT size)
{
    HeapMultiBuf_free(bmc_heap, block, size);
}
