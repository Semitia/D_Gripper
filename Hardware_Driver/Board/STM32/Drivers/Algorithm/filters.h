#ifndef __FILTERS_H
#define __FILTERS_H

/**
 * @brief 一阶低通滤波器
*/
typedef struct __LowPass_t {
    float K;            // 滤波系数
    float last;         // 上一次的输出
} LowPass_t;

void lowPassInit(LowPass_t *lp, float K);
float lowPass(LowPass_t *lp, float input);
#endif
