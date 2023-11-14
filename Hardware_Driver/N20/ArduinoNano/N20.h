/**
 * @file N20.h
 * @brief N20电机驱动头文件，配合DRV8833驱动芯片
 * @version 1.0
 * @date 2023-11-15
 * @author Semitia
*/

#ifndef __N20_H__
#define __N20_H__

#ifndef PI 
#define PI 3.1415926535897932384626433832795
#endif

/**
 * @brief 角度归一化函数
 * @param angle 待归一化角度
 * @return 归一化后角度，范围为[-PI, PI]
*/
double angleNormalize(double angle) {
    while(angle > PI) {
        angle -= 2*PI;
    }
    while(angle < -PI) {
        angle += 2*PI;
    }
    return angle;
}

class N20
{
    private:
        /* 引脚配置 */
        int IN1, IN2;       // 输入信号引脚
        int C1, C2;         // 编码器引脚
        int cnt;            // 编码器计数
        int last_cnt;       // 上一次编码器计数
        
        /* 电机参数 */
        double omg;            // 电机转速 rad/s
        double omg_target;     // 电机目标转速 rad/s
        double angle;          // 电机输出轴机械角度 rad
        double angle_accumu;   // 电机输出轴累计角度 rad
        double gear_ratio;     // 减速比
        double encoder_ratio;  // 编码器分辨率 一个脉冲对应角度

        /* 控制参数 */
        double T;             // 控制周期
        double ts;             // 更新时间戳
        double last_ts;        // 上一次更新时间戳
    
    public:
        N20(int IN1, int IN2, int C1, int C2);
        N20(int IN1, int IN2, int C1, int C2, double gear_ratio, double encoder_ratio);
        ~N20();
        void update(int d_cnt, double ts);
        void update(int vol1, int vol2, double ts);
        void setOmg(double omg);
        double getOmg(void);
        double getAngle(void);

};      

N20 ::N20(int IN1, int IN2, int C1, int C2)
{
    this->IN1 = IN1;
    this->IN2 = IN2;
    this->C1 = C1;
    this->C2 = C2;
    this->gear_ratio = 150;
    this->encoder_ratio = 2*PI/98;
}

N20 ::N20(int IN1, int IN2, int C1, int C2, double gear_ratio, double encoder_ratio)
{
    this->IN1 = IN1;
    this->IN2 = IN2;
    this->C1 = C1;
    this->C2 = C2;
    this->gear_ratio = gear_ratio;
    this->encoder_ratio = encoder_ratio;
}

N20 ::~N20()
{
}

/**
 * @brief 通过周期来更新电机状态，避免在中断中频繁更新
 * @param d_cnt 编码器计数差值
 * @param ts    更新的时间戳
*/
void N20::update(int d_cnt, double ts) {
    cnt += d_cnt;
    double dt = ts - last_ts;
    omg = d_cnt * encoder_ratio / dt;
    angle_accumu += d_cnt * encoder_ratio/ gear_ratio;
    angle = angleNormalize(angle_accumu);
    last_cnt = cnt;
    last_ts = ts;
    return;
}

/**
 * @brief 通过编码器电平更新电机状态，
 *        这个应当被放在输入引脚的中断函数中，但是显然不应该如此频繁的更新电机状态
 * @param vol1 编码器C1电平
 * @param vol2 编码器C2电平
 * @param ts   更新的时间戳
*/
void N20::update(int vol1, int vol2, double ts) {
    if(vol1 == vol2) {      // 电机正转
        cnt++;
    } else {
        cnt--;              // 电机反转
    }
    double dt = ts - last_ts;
    omg = encoder_ratio / dt;
    angle_accumu += encoder_ratio/ gear_ratio;
    angle = angleNormalize(angle_accumu);
    last_cnt = cnt;
    last_ts = ts;
    return;
}


#endif

