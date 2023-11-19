#include "DataScope_DP.h"

unsigned char DataScope_OutPut_Buffer[42] = {0};	   //串口发送缓冲区

/**************************************************************************
上位机使用的是ACE-TECH提供的开源上位机，十分感谢！所以在此注明出处。
**************************************************************************/

/**
 * @brief  将单精度浮点数据转成4字节数据并存入指定地址
 * @param  target:目标单精度数据
 * @param  buf:待写入数组
 * @param  beg:指定从数组第几个元素开始写入
 * @retval None
*/
void Float2Byte(float *target,unsigned char *buf,unsigned char beg)
{
    unsigned char *point;
    point = (unsigned char*)target;	  //得到float的地址
    buf[beg]   = point[0];
    buf[beg+1] = point[1];
    buf[beg+2] = point[2];
    buf[beg+3] = point[3];
}

/**
 * @brief  将待发送通道的单精度浮点数据写入发送缓冲区
 * @param  Data:通道数据
 * @param  Channel:选择通道（1-10）
 * @retval None
*/
void DataScope_Get_Channel_Data(float Data,unsigned char Channel)
{
	if ( (Channel > 10) || (Channel == 0) ) return;  //通道个数大于10或等于0，直接跳出，不执行函数
	else {
		switch (Channel) {
			case 1:  Float2Byte(&Data,DataScope_OutPut_Buffer,1); break;
			case 2:  Float2Byte(&Data,DataScope_OutPut_Buffer,5); break;
			case 3:  Float2Byte(&Data,DataScope_OutPut_Buffer,9); break;
			case 4:  Float2Byte(&Data,DataScope_OutPut_Buffer,13); break;
			case 5:  Float2Byte(&Data,DataScope_OutPut_Buffer,17); break;
			case 6:  Float2Byte(&Data,DataScope_OutPut_Buffer,21); break;
			case 7:  Float2Byte(&Data,DataScope_OutPut_Buffer,25); break;
			case 8:  Float2Byte(&Data,DataScope_OutPut_Buffer,29); break;
			case 9:  Float2Byte(&Data,DataScope_OutPut_Buffer,33); break;
			case 10: Float2Byte(&Data,DataScope_OutPut_Buffer,37); break;
		}
	}	 
}

/**
 * @brief  生成 DataScopeV1.0 能正确识别的帧格式
 * @param  Channel_Number:需要发送的通道个数
 * @retval 返回发送缓冲区数据个数，返回0表示帧格式生成失败
*/
unsigned char DataScope_Data_Generate(unsigned char Channel_Number)
{
	if ( (Channel_Number > 10) || (Channel_Number == 0) ) { return 0; }  //通道个数大于10或等于0，直接跳出，不执行函数
	else {	
		DataScope_OutPut_Buffer[0] = '$';  //帧头
		switch(Channel_Number) { 
			case 1:   DataScope_OutPut_Buffer[5]  =  5; return  6;  
			case 2:   DataScope_OutPut_Buffer[9]  =  9; return 10;
			case 3:   DataScope_OutPut_Buffer[13] = 13; return 14; 
			case 4:   DataScope_OutPut_Buffer[17] = 17; return 18;
			case 5:   DataScope_OutPut_Buffer[21] = 21; return 22;  
			case 6:   DataScope_OutPut_Buffer[25] = 25; return 26;
			case 7:   DataScope_OutPut_Buffer[29] = 29; return 30; 
			case 8:   DataScope_OutPut_Buffer[33] = 33; return 34; 
			case 9:   DataScope_OutPut_Buffer[37] = 37; return 38;
			case 10:  DataScope_OutPut_Buffer[41] = 41; return 42; 
		}	 
	}
	return 0;
}

void DataScope_DMA_Send(unsigned char channel_nums) {
	unsigned char count = DataScope_Data_Generate(channel_nums);
	HAL_UART_Transmit_DMA(&g_uart1_handle, DataScope_OutPut_Buffer, count);
	while(1) {
        if (__HAL_DMA_GET_FLAG(&g_dma_handle, DMA_FLAG_TCIF3_7))        /*等待传输完成*/
        {
            __HAL_DMA_CLEAR_FLAG(&g_dma_handle, DMA_FLAG_TCIF3_7);      /*清除传输完成标志*/
            HAL_UART_DMAStop(&g_uart1_handle);                          /*传输完毕关闭DMA*/
            break;
        }
    }
}









