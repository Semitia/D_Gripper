#include "test.h"

void LED_test(void){
    while(1)
    {
        //HAL_UART_Transmit(&huart1,USART1_BUF,sizeof(USART1_BUF),0xffff);
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
        osDelay(1000);
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
        osDelay(1000);
    }
}

void PWM_test(void){
    uint8_t pwm = 0;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 20);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 40);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 60);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 80);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 30);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 70);
    while(1){
        pwm += 20;
        pwm %= 100;
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm);
        printf("pwm: %d\r\n",pwm); 
        osDelay(1000);
    }

}

void USART_RX_test(void){
    if(g_usart_rx_sta & 0x8000)
    {
        printf("got: %s \r\n",g_usart_rx_buf);
        g_usart_rx_sta = 0;
    }
    osDelay(100);
}

void USART1_dmaTX_test(void) {
    static uint8_t string_to_send[] = "Hello FreeRTOSHello FreeRTOSHello FreeRTOSHello FreeRTOSHello FreeRTOSHello FreeRTOSHello FreeRTOS\r\n";
	while(1){
    HAL_UART_Transmit_DMA(&g_uart1_handle, string_to_send, sizeof(string_to_send));
    while(1) {
        if (__HAL_DMA_GET_FLAG(&g_dma_handle, DMA_FLAG_TCIF3_7))        /*等待传输完成*/
        {
            __HAL_DMA_CLEAR_FLAG(&g_dma_handle, DMA_FLAG_TCIF3_7);      /*清除传输完成标志*/
            HAL_UART_DMAStop(&g_uart1_handle);                          /*传输完毕关闭DMA*/
            break;
        }
    }
    osDelay(500);
	}
}


/**
 * @brief       串口指令集
 *  byte 0:     指令类型
 *      0x00:   6通道PWM占空比调节
 *          byte 1:     通道序号
 *          byte 2:     占空比(0~100)     
 *      0x01:   读取编码器信息
 * 
 * @param       无
 * 
*/
void CMD_ctrl(void){
    if(g_usart_rx_sta & 0x8000)
    {
        switch(g_usart_rx_buf[0]){
            case 0x00:
                switch(g_usart_rx_buf[1]){
                    case 0x00:
                            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, g_usart_rx_buf[2]);
                            break;
                    case 0x01:
                            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, g_usart_rx_buf[2]);
                            break;
                    case 0x02:
                            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, g_usart_rx_buf[2]);
                            break;
                    case 0x03:
                            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, g_usart_rx_buf[2]);
                            break;
                    case 0x04:
                            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, g_usart_rx_buf[2]);
                            break;
                    case 0x05:
                            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, g_usart_rx_buf[2]);
                            break;
                    default:
                    printf("wrong channel number\r\n");
                            break;
                }
                printf("set channel: %d, pwm: %d\r\n",g_usart_rx_buf[1],g_usart_rx_buf[2]);
                break;
            case 0x01:
								break;
            default:
                printf("wrong command\r\n");
                break;
        }
        g_usart_rx_sta = 0;
    }
    else
    {return;}
}






