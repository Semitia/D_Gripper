#include "lowpass.h"

void LP_init(lowpass_t *lp, float TF) {
    lp->TF = TF;
    lp->ts = xTaskGetTickCount();
    lp->res = 0;
}

float LP(lowpass_t *lp, float x) {
    float dt = 0;
    TickType_t new_ts = xTaskGetTickCount();
    //if... 溢出需要两个月，就不判断了，提高程序效率
    dt = (float)(new_ts - lp->ts) / configTICK_RATE_HZ;
    if(dt>0.3f) {
        lp->res = x;
        lp->ts = new_ts;
        return x;
    } 
    float alpha = lp->TF / (lp->TF + dt);
    lp->res = alpha*lp->res + (1.0f-alpha)*x;
    lp->ts = new_ts;
    return lp->res;
}

