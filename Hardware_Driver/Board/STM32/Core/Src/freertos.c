#include "sys.h"
#include "usart.h"
#include "TIM.h"
#include "dma.h"
#include "N20.h"
#include "adc.h"
#include "DataScope_DP.h"

uint8_t USART1_BUF[] = "Hello FreeRTOS\r\n";
N20_t n20[3];

/* START_TASK ���� ���� */
#define START_TASK_PRIO 1                   /* �������ȼ� ����Խ�����ȼ�Խ��*/
#define START_STK_SIZE  128                 /* �����ջ��С */
TaskHandle_t            StartTask_Handler;  /* ������ */
void start_task(void *pvParameters);        /* ������ */

/* TASK1--INFO ���� ���� */
#define TASK1_PRIO      2                   /* �������ȼ� */
#define TASK1_STK_SIZE  128                 /* �����ջ��С */
TaskHandle_t            Task1Task_Handler;  /* ������ */
void N20_Task(void *pvParameters);         /* ������ */

/* TASK2--CMD ���� ���� */
#define TASK2_PRIO      1                   /* �������ȼ� */
#define TASK2_STK_SIZE  128                 /* �����ջ��С */
TaskHandle_t            Task2Task_Handler;  /* ������ */
void CMD_Task(void *pvParameters);          /* ������ */

/* TASK3--ADC �ɼ� ����*/
#define TASK3_PRIO      3                   
#define TASK3_STK_SIZE  128
TaskHandle_t            Task3Task_Handler;
void ADC_Task(void *pvParameters);

/**
  * @brief  FreeRTOS initialization
  */
void MX_FREERTOS_Init(void) {
	xTaskCreate((TaskFunction_t )start_task,          /* ������ */
						(const char*    )"start_task",          /* �������� */
						(uint16_t       )START_STK_SIZE,        /* �����ջ��С */
						(void*          )NULL,                  /* ������������Ĳ��� */
						(UBaseType_t    )START_TASK_PRIO,       /* �������ȼ� */
						(TaskHandle_t*  )&StartTask_Handler);   /* ������ */
}

/**
 * @brief       start_task
 * @param       pvParameters : �������(δ�õ�)
 * @retval      ��
 */
void start_task(void *pvParameters)
{
  taskENTER_CRITICAL();           /* �����ٽ��� */
  /* ��������1 */
  xTaskCreate((TaskFunction_t )N20_Task,
              (const char*    )"infoTask",
              (uint16_t       )TASK1_STK_SIZE,
              (void*          )NULL,
              (UBaseType_t    )TASK1_PRIO,
              (TaskHandle_t*  )&Task1Task_Handler);
  /* ��������2 */
  xTaskCreate((TaskFunction_t )CMD_Task,
              (const char*    )"CMDTask",
              (uint16_t       )TASK2_STK_SIZE,
              (void*          )NULL,
              (UBaseType_t    )TASK2_PRIO,
              (TaskHandle_t*  )&Task2Task_Handler);
  /* ��������3 */
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
	initN20(&n20[0], 0, REDUCTION_RATIO_WORM, 1);
	n20[1].htim_ENC = N1_ENC_TIM;
	n20[1].htim_PWM = N1_PWM_TIM;
	n20[1].channel[0] = N1_PWM_CHANNEL1;
	n20[1].channel[1] = N1_PWM_CHANNEL2;
	initN20(&n20[1], 1, REDUCTION_RATIO_GEAR, 1);
	n20[2].htim_ENC = N2_ENC_TIM;
	n20[2].htim_PWM = N2_PWM_TIM;
	n20[2].channel[0] = N2_PWM_CHANNEL1;
	n20[2].channel[1] = N2_PWM_CHANNEL2;
	initN20(&n20[2], 2, REDUCTION_RATIO_GEAR, 1);
							
  vTaskDelete(StartTask_Handler); /* ɾ����ʼ���� */
  taskEXIT_CRITICAL();            /* �˳��ٽ��� */
}

int ccnt=0,ccnt1=0,ccnt2=0;
void N20_Task(void *argument)
{  
  uint8_t i;
	osDelay(10);
  while(1) {
    for(i=0; i<3; i++) {
      updateN20(&n20[i]);
      switch(n20[i].mode) {
        case SPD_CTRL: {
          setSpd(&n20[i]);
          break;
        }
        case POS_CTRL: {
          setPos(&n20[i]);
          break;
        }
        default: {
          break;
        }
      }
      setPWM(&n20[i]);  
    }
    osDelay(5);
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
int16_t raw_spd, raw_pos;
float recv_spd, recv_pos;
void CMD_Task(void *argument)
{
  uint8_t id;
  while(1) {
		id=0;
    if(g_usart_rx_sta & 0x8000) {
      id = g_usart_rx_buf[0];
      switch(g_usart_rx_buf[1]) {
        case 0x01: {//�ٶȿ���ָ��
					raw_spd = (int16_t) (g_usart_rx_buf[2] << 8 | g_usart_rx_buf[3]);
          recv_spd = (float)raw_spd / SPD_SEND_SCALE;
          n20[id].spd_tar = recv_spd;
          n20[id].mode = SPD_CTRL;
          break;
        }
        case 0x02: {//λ�ÿ���ָ��
          raw_pos = (int16_t) (g_usart_rx_buf[2] << 24 | g_usart_rx_buf[3] << 16 
															|g_usart_rx_buf[4] <<  8 | g_usart_rx_buf[5] & 0xff);
          recv_pos = (float) raw_pos / POS_SEND_SCALE;
					n20[id].pos_tar = recv_pos;
          n20[id].mode = POS_CTRL;
          break;
        }
        case 0x03: {//����λ�������ٶ�
          int16_t send_spd;
          send_buf[0] = id;
          send_buf[1] = 0x03;
          send_spd = (int16_t)(n20[id].spd * SPD_SEND_SCALE);
          send_buf[2] = send_spd >> 8;
          send_buf[3] = send_spd & 0xff;
          sendMsg(send_buf, 4);
          break;
        }
        case 0x04: {//����λ������λ��
          int32_t send_pos;
          send_buf[0] = id;
          send_buf[1] = 0x04;
          send_pos = (int32_t)(n20[id].pos * POS_SEND_SCALE);
          send_buf[2] = (send_pos >> 24);
          send_buf[3] = (send_pos >> 16);
          send_buf[4] = (send_pos >>  8);
          send_buf[5] = send_pos & 0xff;
          sendMsg(send_buf, 6);
          break;
        }
        case 0x05:{ //ͣת
          int i;
          for(i=0; i<3; i++) {
            n20[i].mode = SPD_CTRL;
            n20[i].spd_tar = 0;
          }
          break;
        }
        case 0x07: {//����λ�����ͽǶȴ�����ֵ
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
