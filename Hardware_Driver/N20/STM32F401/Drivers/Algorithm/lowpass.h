#ifndef __LOWPASS_H
#define __LOWPASS_H

#include "FreeRTOS.h"
#include "task.h"

typedef struct __lowpass_t{
    float TF;                      // 低通滤波器的时间常数  
    TickType_t ts;                 // 上次滤波的时间戳 
    float res;                     // 上次滤波结果
}lowpass_t;

void LP_init(lowpass_t *lp, float TF);
float LP(lowpass_t *lp, float x);

#endif

