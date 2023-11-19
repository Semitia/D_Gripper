#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"



extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim1;
extern uint16_t arr_global;                // 自动重装载值


void MX_TIM2_Init(uint16_t arr, uint16_t psc);
void MX_TIM1_Init(uint16_t arr, uint16_t psc);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);



#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

