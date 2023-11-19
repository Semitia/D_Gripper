#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "stdio.h"
#include "delay.h"
#include "stdlib.h"
#include <math.h>
#include "test.h"
#include "TIM.h"
#include "dma.h"
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
  usart_mutex = xSemaphoreCreateMutex();

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
  /* 创建任务3 */
  /*xTaskCreate((TaskFunction_t )FOC_Task,
              (const char*    )"FOCTask",
              (uint16_t       )TASK3_STK_SIZE,
              (void*          )NULL,
              (UBaseType_t    )TASK3_PRIO,
              (TaskHandle_t*  )&Task3Task_Handler);*/
              
  vTaskDelete(StartTask_Handler); /* 删除开始任务 */
  taskEXIT_CRITICAL();            /* 退出临界区 */
}
void info_Task(void *argument)
{
  //char pcWriteBuffer[1024];
  while(1)
  {
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
    osDelay(1000);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
		osDelay(1000);
    //printf("elec_angle: %.2f, velocity: %.2f\r\n", electricalAngle(&M0_encoder), getVelocity(&M0_encoder));
    // // 调用vTaskList()函数，获取所有任务的基本信息，并存入pcWriteBuffer数组
    //   vTaskList(pcWriteBuffer);

    //   // 打印pcWriteBuffer数组的内容
    //   printf("Name\tState\tPriority\tRemainStack\tSequence\r\n");
    //   printf("%s\r\n", pcWriteBuffer);

    //   // 调用uxTaskGetStackHighWaterMark()函数，获取任务1和任务2的最小剩余栈空间，并打印出来
    //   //printf("Task 1 minimum remaining stack: %d bytes\r\n", uxTaskGetStackHighWaterMark(xTask1Handle));
    //   //printf("Task 2 minimum remaining stack: %d bytes\r\n", uxTaskGetStackHighWaterMark(xTask2Handle));

    //   // 调用vTaskGetRunTimeStats()函数，获取所有任务的运行信息，并存入pcWriteBuffer数组
    //   vTaskGetRunTimeStats(pcWriteBuffer);

    //   // 打印pcWriteBuffer数组的内容
    //   printf("Name\tRunCounts\tUsingRate\r\n");
    //   printf("%s\r\n", pcWriteBuffer);
  }
}


void CMD_Task(void *argument)
{
  //DataScope test
  double j=0;
  while(1)	
	{
			j+=0.1;
			if(j>3.14)  j=-3.14; 
			DataScope_Get_Channel_Data(10*j, 1 );
			DataScope_Get_Channel_Data(10*j, 2 );
			DataScope_Get_Channel_Data(2*j, 3 );
			DataScope_DMA_Send(3);
			delay_ms(50); //20HZ 
	} 

}
