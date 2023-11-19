#ifndef __TEST_H
#define __TEST_H

#include "main.h"
#include "TIM.h"
#include "AS5600.h"
#include "usart.h"
#include "stdlib.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "FOC.h"
#include "dma.h"

void LED_test(void);
void PWM_test(void);
void AS5600_test(void);
void USART_RX_test(void);
void CMD_ctrl(void);

#endif

