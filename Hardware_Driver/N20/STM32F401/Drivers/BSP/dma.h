#ifndef __DMA_H
#define	__DMA_H

#include "main.h"

extern DMA_HandleTypeDef g_dma_handle; 
void dma_init(DMA_Stream_TypeDef *dma_stream_handle, uint32_t ch);  /* ≈‰÷√DMAx_CHx */

#endif






























