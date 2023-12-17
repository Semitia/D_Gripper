#include "sys.h"
#include "usart.h"
#include "TIM.h"
#include "dma.h"
#include "N20.h"
#include "adc.h"
#include "DataScope_DP.h"

uint8_t USART1_BUF[] = "Hello FreeRTOS\r\n";
N20_t n20[3];

/* START_TASK 任务 配置 */
#define START_TASK_PRIO 1                   /* 任务优先级 数字越大优先级越高*/
#define START_STK_SIZE  128                 /* 任务堆栈大小 */
TaskHandle_t            StartTask_Handler;  /* 任务句柄 */
void start_task(void *pvParameters);        /* 任务函数 */

/* TASK1--INFO 任务 配置 */
#define TASK1_PRIO      2                   /* 任务优先级 */
#define TASK1_STK_SIZE  128                 /* 任务堆栈大小 */
TaskHandle_t            Task1Task_Handler;  /* 任务句柄 */
void N20_Task(void *pvParameters);         /* 任务函数 */

/* TASK2--CMD 任务 配置 */
#define TASK2_PRIO      1                   /* 任务优先级 */
#define TASK2_STK_SIZE  128                 /* 任务堆栈大小 */
TaskHandle_t            Task2Task_Handler;  /* 任务句柄 */
void CMD_Task(void *pvParameters);          /* 任务函数 */

/* TASK3--ADC 采集 配置*/
#define TASK3_PRIO      3                   
#define TASK3_STK_SIZE  128
TaskHandle_t            Task3Task_Handler;
void ADC_Task(void *pvParameters);

/**
  * @brief  FreeRTOS initialization
  */
void MX_FREERTOS_Init(void) {
	xTaskCreate((TaskFunction_t )start_task,          /* 任务函数 */
						(const char*    )"start_task",          /* 任务名称 */
						(uint16_t       )START_STK_SIZE,        /* 任务堆栈大小 */
						(void*          )NULL,                  /* 传入给任务函数的参数 */
						(UBaseType_t    )START_TASK_PRIO,       /* 任务优先级 */
						(TaskHandle_t*  )&StartTask_Handler);   /* 任务句柄 */
}

/**
 * @brief       start_task
 * @param       pvParameters : 传入参数(未用到)
 * @retval      无
 */
void start_task(void *pvParameters)
{
  taskENTER_CRITICAL();           /* 进入临界区 */
  /* 创建任务1 */
  xTaskCreate((TaskFunction_t )N20_Task,
              (const char*    )"infoTask",
              (uint16_t       )TASK1_STK_SIZE,
              (void*          )NULL,
              (UBaseType_t    )TASK1_PRIO,
              (TaskHandle_t*  )&Task1Task_Handler);
  /* 创建任务2 */
  xTaskCreate((TaskFunction_t )CMD_Task,
              (const char*    )"CMDTask",
              (uint16_t       )TASK2_STK_SIZE,
              (void*          )NULL,
              (UBaseType_t    )TASK2_PRIO,
              (TaskHandle_t*  )&Task2Task_Handler);
  /* 创建任务3 */
  xTaskCreate((TaskFunction_t )ADC_Task,
              (const char*    )"ADCTask",
              (uint16_t       )TASK3_STK_SIZE,
              (void*          )NULL,
              (UBaseType_t    )TASK3_PRIO,
              (TaskHandle_t*  )&Task3Task_Handler);

							
	n20[0].htim_ENC = N0_ENC_TIM;
	n20[0].htim_PWM = N0_PWM_TIM;
	n20[0].channel[0] = N0_PWM_CHANNEL1;
	n20[0].channel[1] = N0_PWM_CHANNEL2;
	initN20(&n20[0], 0);
	n20[1].htim_ENC = N1_ENC_TIM;
	n20[1].htim_PWM = N1_PWM_TIM;
	n20[1].channel[0] = N1_PWM_CHANNEL1;
	n20[1].channel[1] = N1_PWM_CHANNEL2;
	initN20(&n20[1], 1);
	n20[2].htim_ENC = N2_ENC_TIM;
	n20[2].htim_PWM = N2_PWM_TIM;
	n20[2].channel[0] = N2_PWM_CHANNEL1;
	n20[2].channel[1] = N2_PWM_CHANNEL2;
	initN20(&n20[2], 2);
							
  vTaskDelete(StartTask_Handler); /* 删除开始任务 */
  taskEXIT_CRITICAL();            /* 退出临界区 */
}


int ccnt=0,ccnt1=0,ccnt2=0;
void N20_Task(void *argument)
{  
  // while(1)
  // {
	// 	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
  //   osDelay(1000);
	// 	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
	// 	osDelay(1000);
  // }
	osDelay(10);
  while(1) {
		// ccnt = __HAL_TIM_GET_COUNTER(&htim3);
    // ccnt1 = __HAL_TIM_GET_COUNTER(&htim4);
    // ccnt2 = __HAL_TIM_GET_COUNTER(&htim5);
    updateN20(&n20[0]);
    updateN20(&n20[1]);
    updateN20(&n20[2]);
    setSpd(&n20[0], 3);
    setSpd(&n20[1], 3);
    setSpd(&n20[2], 3);
    setPWM(&n20[0]);
    setPWM(&n20[1]);
    setPWM(&n20[2]);
    osDelay(10);
  }

}

uint32_t adc_val[3] = {0};
void ADC_Task(void *argument)
{
  int i;
  while(1) {
    for(i=0; i<3; i++) {
      HAL_ADC_Start(&hadc1);
      HAL_ADC_PollForConversion(&hadc1, 1000);
      adc_val[i] = HAL_ADC_GetValue(&hadc1);
      HAL_ADC_Stop(&hadc1);
    }
    osDelay(50);
  }
}

void sendMsg(uint8_t *buf, uint8_t size) {
  buf[size++] = 0x41;
  buf[size++] = 0x49;
  HAL_UART_Transmit_DMA(&huart1, buf, size);
  while(1) {
      if (__HAL_DMA_GET_FLAG(&g_dma_handle, DMA_FLAG_TCIF3_7))        
      {
          __HAL_DMA_CLEAR_FLAG(&g_dma_handle, DMA_FLAG_TCIF3_7);      
          HAL_UART_DMAStop(&huart1);                          
          break;
      }
  }
}

uint8_t send_buf[8];
float recv_spd, recv_pos;
void CMD_Task(void *argument)
{
  // // DataScope test
  // double j=0;
  // while(1)	
	// {
	// 		j+=0.1;
	// 		if(j>3.14)  j=-3.14; 
	//    DataScope_Get_Channel_Data(10*j, 1 );
	// 		DataScope_Get_Channel_Data(10*j, 2 );
	// 		DataScope_Get_Channel_Data(2*j, 3 );
	// 		DataScope_DMA_Send(3);
	// 		delay_ms(50); //20HZ 
	// } 
  uint8_t id;
  while(1) {
		id=0;
    if(g_usart_rx_sta & 0x8000) {
      id = g_usart_rx_buf[0];
      switch(g_usart_rx_buf[1]) {
        case 0x01: {//速度控制指令
          recv_spd = (float)(g_usart_rx_buf[2] << 8 | g_usart_rx_buf[3]) / SPD_SEND_SCALE;
          n20[id].spd_tar = recv_spd;
          break;
        }
        case 0x02: {//位置控制指令
          recv_pos = (float)(g_usart_rx_buf[2] << 8 | g_usart_rx_buf[3]) / POS_SEND_SCALE;
          break;
        }
        case 0x03: {//向上位机发送速度
          int16_t send_spd;
          send_buf[0] = id;
          send_buf[1] = 0x03;
          send_spd = (int16_t)(n20[id].spd * SPD_SEND_SCALE);
          send_buf[2] = send_spd >> 8;
          send_buf[3] = send_spd & 0xff;
          sendMsg(send_buf, 4);
          break;
        }
        case 0x04: {//向上位机发送位置
          int16_t send_pos;
          send_buf[0] = id;
          send_buf[1] = 0x04;
          send_pos = (int16_t)(n20[id].pos * POS_SEND_SCALE);
          send_buf[2] = send_pos >> 8;
          send_buf[3] = send_pos & 0xff;
          sendMsg(send_buf, 4);
          break;
        }
        case 0x05:{ //停转
          n20[0].spd_tar = 0;
          n20[1].spd_tar = 0;
          n20[2].spd_tar = 0;
          break;
        }
        case 0x07: {//向上位机发送角度传感器值
          break;
        }
        default: {
          break;
        }
      }
      g_usart_rx_sta = 0;
    }
  }

}
