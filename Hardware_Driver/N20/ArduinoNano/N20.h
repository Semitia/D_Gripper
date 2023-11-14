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

class N20
{
    private:
        /* 引脚配置 */
        int IN1, IN2;       // 输入信号引脚
        int C1, C2;         // 编码器引脚

        /* 电机参数 */
        double omg;            // 电机转速 rad/s
        double angle;          // 电机角度 rad
        double gear_ratio;     // 减速比
        double encoder_ratio;  // 编码器分辨率 一个脉冲对应角度
    
    public:
        N20(int IN1, int IN2, int C1, int C2);
        N20(int IN1, int IN2, int C1, int C2, double gear_ratio, double encoder_ratio);
        ~N20();
        

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


#endif

