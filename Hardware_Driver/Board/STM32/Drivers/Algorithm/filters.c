#include "filters.h"

/**************************一阶低通滤波器**************************/
/**
 * @brief 一阶低通滤波器初始化（直接使用滤波系数）
 * @param lp 滤波器结构体
 * @param K  滤波系数
 * @retval None
 */
void lowPassInit(LowPass_t *lp, float K) {
    lp->K = K;
    lp->last = 0;
    return;
}

/**
 * @brief 一阶低通滤波器
 * @param lp 滤波器结构体
 * @param input 输入值
 * @retval 滤波后的值
*/
float lowPass(LowPass_t *lp, float input) {
    lp->last = lp->last + lp->K * (input - lp->last);
    return lp->last;
}

/**************************卡尔曼滤波器**************************/

