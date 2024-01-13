# 硬件驱动程序使用说明
## Huaner_Servo
幻尔舵机的python驱动，使用串口直接连接驱动板便可控制三个舵机

## Board
内含pc、Arduino、STM32文件夹
arduino与STM32二者任选其一即可
### pc
电脑上位机python驱动程序
### Arduino & STM32
使用Arduino或者STM32单片机完成对夹爪上三个电机与三个角度传感器的控制与数据读取
使用串口与电脑相连，使用python中的程序完成与下位机的对接。



## D-Gripper
夹爪控制驱动，