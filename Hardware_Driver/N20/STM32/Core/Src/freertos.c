#include "sys.h"
#include "usart.h"
#include "TIM.h"
#include "dma.h"
#include "N20.h"
#include "DataScope_DP.h"

uint8_t USART1_BUF[] = "Hello FreeRTOS\r\n";

/* START_TASK 任务 配置 */
#define START_TASK_PRIO 1                   /* 任务优先级 数字越大优先级越高*/
#define START_STK_SIZE  128                 /* 任务堆栈大小 */
TaskHandle_t            StartTask_Handler;  /* 任务句柄 */
void start_task(void *pvParameters);        /* 任务函数 */

/* TASK1--INFO 任务 配置 */
#define TASK1_PRIO      2                   /* 任务优先级 */
#define TASK1_STK_SIZE  128                 /* 任务堆栈大小 */
TaskHandle_t            Task1Task_Handler;  /* 任务句柄 */
void info_Task(void *pvParameters);         /* 任务函数 */

/* TASK2--CMD 任务 配置 */
#define TASK2_PRIO      1                   /* 任务优先级 */
#define TASK2_STK_SIZE  128                 /* 任务堆栈大小 */
TaskHandle_t            Task2Task_Handler;  /* 任务句柄 */
void CMD_Task(void *pvParameters);          /* 任务函数 */

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
  xTaskCreate((TaskFunction_t )info_Task,
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
              
  vTaskDelete(StartTask_Handler); /* 删除开始任务 */
  taskEXIT_CRITICAL();            /* 退出临界区 */
}

N20_t n20[3];
int ccnt=0,ccnt1=0,ccnt2=0;
void info_Task(void *argument)
{  
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

  // while(1)
  // {
	// 	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
  //   osDelay(1000);
	// 	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
	// 	osDelay(1000);
  // }

  while(1) {
		ccnt = __HAL_TIM_GET_COUNTER(&htim3);
    ccnt1 = __HAL_TIM_GET_COUNTER(&htim4);
    ccnt2 = __HAL_TIM_GET_COUNTER(&htim5);
    updateN20(&n20[0]);
    updateN20(&n20[1]);
    updateN20(&n20[2]);
    n20[0].output = 0.5;
    n20[1].output = 0.5;
    n20[2].output = 0.5;
    setPWM(&n20[0]);
    setPWM(&n20[1]);
    setPWM(&n20[2]);
    osDelay(500);
  }

}

void CMD_Task(void *argument)
{
  // // DataScope test
  // double j=0;
  // while(1)	
	// {
	// 		j+=0.1;
	// 		if(j>3.14)  j=-3.14; 
	// 		DataScope_Get_Channel_Data(10*j, 1 );
	// 		DataScope_Get_Channel_Data(10*j, 2 );
	// 		DataScope_Get_Channel_Data(2*j, 3 );
	// 		DataScope_DMA_Send(3);
	// 		delay_ms(50); //20HZ 
	// } 

  while(1) {
    if(g_usart_rx_sta & 0x8000) {
      switch(g_usart_rx_buf[0]) {
        case 0x00:
          break;
        default:
          break;
      }
      g_usart_rx_sta = 0;
    }
  }

}
