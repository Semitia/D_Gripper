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
    n20->mode = SPD_CTRL;
    n20->arr = __HAL_TIM_GET_AUTORELOAD(n20->htim_PWM);
    n20->enc_spd_ratio = (float)2 * PI/(ENCODER_PPR*REDUCTION_RATIO*4); //双相编码器，4倍频
    n20->last_time = xTaskGetTickCount();
    
    n20->spd = 0;
    n20->spd_tar = 0;
    n20->spd_last = 0;
    n20->pos = 0;
    n20->pos_tar = 0;

    lowPassInit(&n20->SpdLP, LP_K);
    n20->SpdPID.Kp = 0.8;
    n20->SpdPID.Ki = 0.0;
    n20->SpdPID.Kd = 0.0;
    n20->SpdPID.I_limit = 6.0;
    n20->SpdPID.res_max = 1.0;
    n20->SpdPID.res_min = -1.0;

    n20->PosPID.Kp = 0.8;
    n20->PosPID.Ki = 0.0;
    n20->PosPID.Kd = 0.0;
    n20->PosPID.I_limit = 6.0;
    n20->PosPID.res_max = 1.0;
    n20->PosPID.res_min = -1.0;
    return;
}

/**
 * @brief 更新电机状态
 * @param n20 N20电机结构体
*/
void updateN20(N20_t *n20) {
	float dx,dt;
	int16_t cnt = (int16_t)__HAL_TIM_GET_COUNTER(n20->htim_ENC);
	TickType_t now_time = xTaskGetTickCount();
	
    dx = (float)(cnt * n20->enc_spd_ratio);
    dt = (float)(now_time - n20->last_time) / RTOS_FREC;
    n20->encoder = cnt;
    n20->pos += dx;
    n20->spd_last = n20->spd;
    n20->spd = dx / dt;
    __HAL_TIM_SET_COUNTER(n20->htim_ENC, 0);       //reset count
    n20->last_time = now_time;
    return;
}

void setSpd(N20_t *n20) {
    n20->output = PID(&n20->SpdPID,n20->spd_tar - n20->spd);
    return;
}


void setPos(N20_t *n20) {
    n20->spd_tar = PID(&n20->PosPID,n20->pos_tar - n20->pos);
    n20->output  = PID(&n20->SpdPID,n20->spd_tar - n20->spd);
    return;
}

/**
 * @brief 根据output设置电机PWM
 * @param n20 N20电机结构体
*/
void setPWM(N20_t *n20) {
    uint16_t CCR;
    if(n20->output >= 0) {
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


