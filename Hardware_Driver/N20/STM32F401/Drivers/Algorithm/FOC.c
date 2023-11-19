#include "FOC.h"

#define _3PI_2 4.71238898038f   //1.5PI
#define sqrt3 1.73205080757f
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

float voltage_power_supply=0;
float zero_electric_angle=0;
float target_position_0 = 0, target_position_1 = 0;
float target_velocity_0 = 0, target_velocity_1 = 0;
float target_torque_0 = 0, target_torque_1 = 0;
int PP=1,DIR=1;

control_mode_t control_mode = TORQUE;
lowpass_t M0_Vel_LPF, M1_Vel_LPF;
AS5600_t M0_encoder, M1_encoder;
PID_t M0_vel_PID, M1_vel_PID, M0_pos_PID, M1_pos_PID;

/**
 * @brief  将角度归一化到[0,2PI]
*/
float normalizeAngle(float angle){
    //printf("DEBUG: normalizeAngle input: %.2f, ", angle);
    while(angle >= _2PI) angle -= _2PI;
    while(angle < 0) angle += _2PI;
    //printf("output: %.2f\r\n", angle);
    return angle;
}

/**
 * @brief  FOC初始化
 * @param  _voltage_power_supply  电源电压
 * @param  _PP   点击极对数
 * @param  _DIR  电机转向
*/
void FOC_init(float _voltage_power_supply, int _PP, int _DIR){
    voltage_power_supply = _voltage_power_supply;
    PP = _PP;
    DIR = _DIR;

    // 初始化低通滤波器
    LP_init(&M0_Vel_LPF,0.01f);
    LP_init(&M1_Vel_LPF,0.01f);
    // 初始化编码器
    iic_init();							
    AS_init(&M0_encoder,0);
    AS_init(&M1_encoder,1);
    // 初始化PID控制器
    PID_init(&M0_vel_PID, 0.28f, 0.0f, 0.001f, 1, 1,  -1);
    PID_init(&M1_vel_PID, 0.3f, 7.0f, 1.0f, 10, 5,  -5);
    PID_init(&M0_pos_PID, 0.3f, 0,    1.0f,  0, 10, -10);
    PID_init(&M1_pos_PID, 0.3f, 0,    1.0f,  0, 10, -10);
    // 矫正电角度零点
    alignSensor();
		
	
}

void setPWM(float Ua,float Ub,float Uc){
    // 限幅
    Ua = _constrain(Ua, 0.0f, voltage_power_supply);
    Ub = _constrain(Ub, 0.0f, voltage_power_supply);
    Uc = _constrain(Uc, 0.0f, voltage_power_supply);
    // 转换到TIM的CCR值
    uint16_t CCR1_Val = (uint16_t)((Ua / voltage_power_supply) * arr_global);
    uint16_t CCR2_Val = (uint16_t)((Ub / voltage_power_supply) * arr_global);
    uint16_t CCR3_Val = (uint16_t)((Uc / voltage_power_supply) * arr_global);
    // 设置PWM
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, CCR1_Val);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, CCR2_Val);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, CCR3_Val);

    //printf("DEBUG-setPWM CCR1_Val: %d, CCR2_Val: %d, CCR3_Val: %d\r\n", CCR1_Val, CCR2_Val, CCR3_Val);
}

/**
 * @brief  设置转矩
 * @param  Uq  转矩
 * @param  angle  电角度
*/
void setTorque(float Uq, float angle){
    static float U_alpha=0,U_beta=0,Ua=0,Ub=0,Uc=0;

    // 更新角度值
    AS_update(&M0_encoder);
    //AS_update(&M1_encoder);

    Uq = _constrain(Uq, -(voltage_power_supply/2), voltage_power_supply/2);
    // Park逆变换
    U_alpha = -Uq*sin(angle);
    U_beta = Uq*cos(angle);
    // Clarke逆变换
    Ua = U_alpha + voltage_power_supply/2;
    Ub = (sqrt3*U_beta - U_alpha)/2 + voltage_power_supply/2;
    Uc = (-U_alpha-sqrt3*U_beta)/2 + voltage_power_supply/2;
    setPWM(Ua, Ub, Uc);
	
    printf("DEBUG-setTorque elec_angle: %.2f, Uq:%.2f, ", angle,Uq);
		printf("Ua: %.2f, Ub: %.2f, Uc: %.2f\r\n", Ua, Ub, Uc);
    return;
}

void setVelocity(float target_velocity){
		float err = target_velocity-getVelocity(&M0_encoder);
		float torque_tem = PID(&M0_vel_PID,err);
    setTorque(torque_tem, electricalAngle(&M0_encoder));
		printf("err %.2f, Uq:%.2f\r\n",err,torque_tem);
    return;
}

void setPosition(float target_position){
    setVelocity(PID(&M0_pos_PID, (target_position-getMechanicalAngle(&M0_encoder))));
    return;
}

float electricalAngle(AS5600_t *encoder){
    float res = normalizeAngle((float)(DIR*PP)*getMechanicalAngle(encoder) - zero_electric_angle);
	
    //printf("DEBUG-electricalAngle ele_Angel: %.2f\r\n", res);
    return res;
}

void alignSensor(void){
    setTorque(3,_3PI_2);    //起劲
    osDelay(1000);
    AS_update(&M0_encoder);
    //AS_update(&M1_encoder);
    zero_electric_angle = electricalAngle(&M0_encoder);
    setTorque(0,_3PI_2);    //松劲
    printf("DEBUG-alignSensor zero_electric_angle: %.2f\r\n", zero_electric_angle); 
}

