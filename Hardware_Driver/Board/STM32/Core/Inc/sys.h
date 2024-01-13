#ifndef __SYS_H__
#define __SYS_H__

/* std lib */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

/* STM32 lib */
#include "stm32f4xx_hal.h"
#include "core_cm4.h"

/* FreeRTOS lib */
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"

/* my lib */
#include "delay.h"

#define RTOS_FREC configTICK_RATE_HZ
#define FRE_1MS RTOS_FREC/1000
#define FRE_10MS RTOS_FREC/100

#ifndef PI 
#define PI 3.14159265358979323846f
#endif

//N20电机资源分配
#define N0_ENC_TIM &htim3
#define N0_PWM_TIM &htim2
#define N0_PWM_CHANNEL1 TIM_CHANNEL_3
#define N0_PWM_CHANNEL2 TIM_CHANNEL_4
#define N1_ENC_TIM &htim4
#define N1_PWM_TIM &htim2
#define N1_PWM_CHANNEL1 TIM_CHANNEL_1
#define N1_PWM_CHANNEL2 TIM_CHANNEL_2
#define N2_ENC_TIM &htim5
#define N2_PWM_TIM &htim1
#define N2_PWM_CHANNEL1 TIM_CHANNEL_1
#define N2_PWM_CHANNEL2 TIM_CHANNEL_4

void Error_Handler(void);

#endif // __SYS_H__

