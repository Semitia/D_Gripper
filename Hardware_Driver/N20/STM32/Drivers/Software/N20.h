#ifndef __N20_H__
#define __N20_H__

#include "sys.h"
#include "PID.h"
#include "filters.h"

#define LP_K 0.1

#define ENCODER_PPR 7 // 编码器基础脉冲数
#define REDUCTION_RATIO 150 // 减速比

typedef struct __N20_t { 
    uint8_t id;
    int16_t encoder;
    TIM_HandleTypeDef *htim_ENC;
    TickType_t last_time;

    TIM_HandleTypeDef *htim_PWM;
    uint16_t arr;           //PWM TIM 自动重装值
    uint32_t channel[2];    //定时器通道
    float output;           //电机输出 0~1

    float enc_spd_ratio;    // 电机转速与编码器转速比
    float spd, spd_tar, spd_last;
    //
    float pos, pos_tar;
    PID_t SpdPID;
    LowPass_t SpdLP;
}N20_t;

void initN20(N20_t *n20, uint8_t id);
void updateN20(N20_t *n20);
void setPWM(N20_t *n20);
#endif // __N20_H__

