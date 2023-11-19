#ifndef __DATA_PRTOCOL_H
#define __DATA_PRTOCOL_H

#include "main.h"
#include "dma.h"
#include "usart.h"

/**
 * @brief 发送数据结构体
*/
typedef struct __Scope_Data_t {
    float Data;
    unsigned char Channel;
}Scope_Data_t;

extern unsigned char DataScope_OutPut_Buffer[42];	                    //待发送帧数据缓存区
void DataScope_Get_Channel_Data(float Data,unsigned char Channel);      // 写通道数据至 待发送帧数据缓存区
unsigned char DataScope_Data_Generate(unsigned char Channel_Number);    // 发送帧数据生成函数 
void DataScope_DMA_Send(unsigned char channel_nums);                    // DMA发送函数

#endif 





