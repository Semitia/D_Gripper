/**
 * @file N20.c
 * @brief N20电机STM32驱动
 * @author Semitia
 * @date 2023-12-12
 * @version 0.1
*/
#include "N20.h"

void initN20(N20_t *n20, uint8_t id) {
    n20->id = id;
    n20->arr = __HAL_TIM_GET_AUTORELOAD(n20->htim_PWM);
    n20->enc_spd_ratio = (float)2 * PI/(ENCODER_PPR*REDUCTION_RATIO*4);
    n20->last_time = xTaskGetTickCount();
    lowPassInit(&n20->SpdLP, LP_K);
    n20->SpdPID.Kp = 0.1;
    n20->SpdPID.Ki = 0.0;
    n20->SpdPID.Kd = 0.0;
    n20->SpdPID.I_limit = 0.0;

    return;
}

float dx,dt;
TickType_t now_time;
void updateN20(N20_t *n20) {
    now_time = xTaskGetTickCount();
    int cnt = __HAL_TIM_GET_COUNTER(n20->htim_ENC);
    dx = (float)(cnt * n20->enc_spd_ratio);
    dt = (float)(now_time - n20->last_time) / RTOS_FREC;
    n20->encoder = (int16_t)cnt;
    n20->pos += dx;
    n20->spd_last = n20->spd;
    n20->spd = dx / dt;
    __HAL_TIM_SET_COUNTER(n20->htim_ENC, 0);       //reset count

    n20->last_time = now_time;
    return;
}

/**
 * @brief 根据output设置电机PWM
 * @param n20 N20电机结构体
*/
void setPWM(N20_t *n20) {
    uint16_t CCR;
    if(n20->output >0) {
        CCR = (uint16_t)(n20->arr * n20->output);
        __HAL_TIM_SET_COMPARE(n20->htim_PWM, n20->channel[0], CCR);
        __HAL_TIM_SET_COMPARE(n20->htim_PWM, n20->channel[1], 0);
    } else {
        CCR = (uint16_t)(n20->arr * (-n20->output));
        __HAL_TIM_SET_COMPARE(n20->htim_PWM, n20->channel[0], 0);
        __HAL_TIM_SET_COMPARE(n20->htim_PWM, n20->channel[1], CCR);
    }
    return;
}


