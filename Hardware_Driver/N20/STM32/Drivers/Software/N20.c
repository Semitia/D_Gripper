/**
 * @file N20.c
 * @brief N20电机STM32驱动
 * @author Semitia
 * @date 2023-12-12
 * @version 0.1
*/
#include "N20.h"

void initN20(N20_t *n20, uint8_t id, TIM_HandleTypeDef *htim) {
    n20->id = id;
    n20->htim = htim;
    n20->enc_spd_ratio = 1/(7*150*4);

    lowPassInit(&n20->SpdLP, LP_K);

    n20->SpdPID.Kp = 0.1;
    n20->SpdPID.Ki = 0.0;
    n20->SpdPID.Kd = 0.0;
    n20->SpdPID.I_limit = 0.0;


    return;
}

void updateN20(N20_t *n20) {
    static TickType_t last_time = 0;
    TickType_t now_time = xTaskGetTickCount();

    int cnt = __HAL_TIM_GET_COUNTER(n20->htim);
    //int dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(n20->htim);
    float dx = (float)(n20->encoder * n20->enc_spd_ratio);
    n20->encoder = (int16_t)cnt;
    n20->pos += dx;
    n20->spd_last = n20->spd;
    n20->spd = dx / (now_time - last_time) * RTOS_FREC;

    last_time = now_time;
    // DEBUG

    return;
}

