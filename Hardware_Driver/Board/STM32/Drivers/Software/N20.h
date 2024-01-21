#ifndef __N20_H__
#define __N20_H__

#include "sys.h"
#include "PID.h"
#include "filters.h"

#define LP_K 0.1
#define ENCODER_PPR 7 // 编码器基础脉冲数
#define REDUCTION_RATIO 150 // 减速比
#define REDUCTION_RATIO_WORM 236 // 蜗杆减速器减速比
#define REDUCTION_RATIO_GEAR 380 // 直齿轮箱
#define SPD_SEND_SCALE 1000 // 电机转速发送缩放比例
#define POS_SEND_SCALE 1000 // 电机位置发送缩放比例

enum ctrl_mode {
    SPD_CTRL = 0,
    POS_CTRL,
};

typedef struct __N20_t { 
    uint8_t id;
    enum ctrl_mode mode;

    // Encoder TIM
    int16_t encoder;
    TIM_HandleTypeDef *htim_ENC;
    TickType_t last_time;
    // PWM TIM
    TIM_HandleTypeDef *htim_PWM;
    uint16_t arr;           //PWM TIM 自动重装值
    uint32_t channel[2];    //定时器通道
    float output;           //电机输出 0~1
    short output_polar;

    float enc_spd_ratio;    // 电机转速与编码器转速比
    float spd, spd_tar, spd_last; // rad/s
    float pos, pos_tar, pos_nor;  // position, target, normalized  rad

    PID_t SpdPID, PosPID;
    LowPass_t SpdLP;
}N20_t;

void initN20(N20_t *n20, uint8_t id, float reduc_ratio, short polar);
void updateN20(N20_t *n20);
void setSpd(N20_t *n20);
void setPos(N20_t *n20);
void setPWM(N20_t *n20);
#endif // __N20_H__

