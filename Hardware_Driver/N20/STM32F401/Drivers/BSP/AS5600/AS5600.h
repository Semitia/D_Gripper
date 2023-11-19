#ifndef __AS5600_H
#define __AS5600_H

#include "myiic.h"
#include "FreeRTOS.h"
#include "task.h"

#define MAX_TICKS 0xffffffff

typedef struct __AS5600_t{
    int _Mot_Num;
    float angle;                                 // 最近更新的角度
    float angle_prev;                            // 上次调用 getVelocity 时的角度
    int32_t rotations;                           // 最近更新的总圈数
    int32_t rotations_prev;                      // 上次调用 getVelocity 时的圈数计数
    TickType_t ts;                               // 最近更新的时间戳
    TickType_t ts_prev;                          // 上次调用 getVelocity 的时间戳

}AS5600_t;

float getAngle(AS5600_t *as);
float getVelocity(AS5600_t *as);
float getMechanicalAngle(AS5600_t *as);
void AS_update(AS5600_t *as);
void AS_init(AS5600_t *as, int _Mot_Num);
#endif

