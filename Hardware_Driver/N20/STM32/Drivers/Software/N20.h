#ifndef __N20_H__
#define __N20_H__

#include "sys.h"
#include "PID.h"
#include "filters.h"

#define LP_K 0.1


typedef struct __N20_t { 
    uint8_t id;
    int16_t encoder;
    TIM_HandleTypeDef *htim;
    float enc_spd_ratio; // 电机转速与编码器转速比
    float spd, spd_tar, spd_last;
    float pos, pos_tar;
    PID_t SpdPID;
    LowPass_t SpdLP;
}N20_t;

void initN20(N20_t *n20, uint8_t id, TIM_HandleTypeDef *htim);
void updateN20(N20_t *n20);

#endif // __N20_H__

