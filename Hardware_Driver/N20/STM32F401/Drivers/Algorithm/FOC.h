#ifndef __FOC_H
#define __FOC_H

#include "main.h"
#include "cmsis_os.h"
#include "TIM.h"
#include "AS5600.h"
#include "stdio.h"
#include "usart.h"
#include "math.h"
#include "lowpass.h"
#include "PID.h"

void FOC_init(float _voltage_power_supply, int _PP, int _DIR);
void setPWM(float Ua,float Ub,float Uc);
void setTorque(float Uq, float angle);
void setVelocity(float target_velocity);
void setPosition(float target_position);
void alignSensor(void);
float normalizeAngle(float angle);
float electricalAngle(AS5600_t *encoder);

typedef enum {
    TORQUE,
    VELOCITY,
    POSITION
} control_mode_t;

extern control_mode_t control_mode;
extern AS5600_t M0_encoder, M1_encoder;
extern float target_torque_0, target_torque_1;
extern float target_velocity_0, target_velocity_1;
extern float target_position_0, target_position_1;
extern PID_t M0_vel_PID, M1_vel_PID, M0_pos_PID, M1_pos_PID;

#endif

