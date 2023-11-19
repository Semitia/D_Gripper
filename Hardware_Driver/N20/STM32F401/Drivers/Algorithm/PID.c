#include "PID.h"
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))//限幅函数

void PID_init(PID_t *pid, float Kp, float Ki, float Kd, float I_lim, float res_max, float res_min) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->I_limit = I_lim;
    pid->res_max = res_max;
    pid->res_min = res_min;
    pid->err_prev = 0;
    pid->err_sum = 0;
    pid->Ts = xTaskGetTickCount();
}

float PID(PID_t *pid, float err){
    TickType_t now_Ts = xTaskGetTickCount();
    float dt = (float)(now_Ts - pid->Ts) / configTICK_RATE_HZ;
    //P,I,D 计算
    float P = pid->Kp * err;
    pid->err_sum += err * dt;
    pid->err_sum = _constrain(pid->err_sum, -pid->I_limit, pid->I_limit);
    float I = pid->Ki * pid->err_sum ;
    float D = pid->Kd * (err - pid->err_prev) / dt;
    
    pid->res = P + I + D;
    pid->res = _constrain(pid->res, pid->res_min, pid->res_max);
    //update
    pid->err_prev = err;
    pid->Ts = now_Ts;
    return pid->res;
}

