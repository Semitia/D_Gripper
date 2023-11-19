#include "AS5600.h"

float abs_f(float x){
	if(x>0) return x;
	else return -x;
}

float getAngle(AS5600_t *as) {
    return (float) (as->rotations)*_2PI + (as->angle);
}

float getVelocity(AS5600_t *as) {
    float Ts = 0, vel = 0;
    // 计算采样时间
    if(as->ts > as->ts_prev) {
        Ts = (float)(as->ts - as->ts_prev) / configTICK_RATE_HZ;
    } else {
        Ts = (float)(as->ts + MAX_TICKS - as->ts_prev) / configTICK_RATE_HZ;
    }

    // 速度计算
    vel = ( (float)(as->rotations - as->rotations_prev)*_2PI + (as->angle - as->angle_prev) ) / Ts;    
    // 保存变量以待将来使用
    as->angle_prev = as->angle;
    as->rotations_prev = as->rotations;
    as->ts_prev = as->ts;
    return vel;
}

float getMechanicalAngle(AS5600_t *as) {
    return as->angle;
}

void AS_update(AS5600_t *as) {
    float d_angle = 0;
    float new_angle = AS5600_ReadAngle(AS5600_HI, AS5600_LO);
    as->ts = xTaskGetTickCount(); //应尽量靠近获取角度的时刻

    //圈数计算
    d_angle = new_angle - as->angle;
    if(abs_f(d_angle) > (0.8f*_2PI) ){
        as->rotations += ( d_angle > 0 ) ? -1 : 1; 
    } 
    as->angle = new_angle;
		
}

void AS_init(AS5600_t *as, int _Mot_Num) {
    as->_Mot_Num = _Mot_Num;
    as->angle = AS5600_ReadAngle(AS5600_HI, AS5600_LO);
    as->angle_prev = as->angle;
    as->rotations = 0;
    as->rotations_prev = 0;
    as->ts = xTaskGetTickCount();
    as->ts_prev = as->ts;
}

