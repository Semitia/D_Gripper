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

void Error_Handler(void);

#endif // __SYS_H__

