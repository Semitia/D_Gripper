"""

"""

from ServoCtrl import ServoCtrl
from STM32Ctrl import STM32Ctrl
import keyboard
import time


class DgripperCtrl:
    SERVO_NUM = 3

    def __init__(self, servo_port, servo_baud, nano_port, nano_baud):
        self.nano = STM32Ctrl(nano_port, nano_baud)
        self.Servo = ServoCtrl(servo_port, servo_baud, self.SERVO_NUM)
        print("DgripperCtrl Init Complete!")

    def run(self):
        while True:
            if keyboard.is_pressed('q'):                  # 夹爪0张开
                self.Servo.step_forward(0)
                print("q")

            elif keyboard.is_pressed('w'):                # 夹爪1张开
                self.Servo.step_forward(1)
                print("w")

            elif keyboard.is_pressed('e'):                # 夹爪2张开
                self.Servo.step_forward(2)
                print("e")

            elif keyboard.is_pressed('a'):                # 夹爪0闭合
                print("a")
                self.Servo.step_backward(0)

            elif keyboard.is_pressed('s'):                # 夹爪1闭合
                print("s")
                self.Servo.step_backward(1)

            elif keyboard.is_pressed('d'):                # 夹爪2闭合
                print("d")
                self.Servo.step_backward(2)

            elif keyboard.is_pressed('u'):               # 步进电机0正转
                print("u")
                self.nano.motor[0].set_speed(5, self.nano.ser)

            elif keyboard.is_pressed('i'):               # 步进电机1正转
                print("i")
                self.nano.motor[1].set_speed(5, self.nano.ser)

            elif keyboard.is_pressed('o'):               # 步进电机2正转
                print("o")
                self.nano.motor[2].set_speed(5, self.nano.ser)

            elif keyboard.is_pressed('j'):               # 步进电机0反转
                print("j")
                self.nano.motor[0].set_speed(-5, self.nano.ser)

            elif keyboard.is_pressed('k'):               # 步进电机1反转
                print("k")
                self.nano.motor[1].set_speed(-5, self.nano.ser)

            elif keyboard.is_pressed('l'):               # 步进电机2反转
                print("l")
                self.nano.motor[2].set_speed(-5, self.nano.ser)

            elif keyboard.is_pressed('1'):                # 读取角度传感器0,1,2角度
                sensor_id = int(input("Please input sensor id: "))
                self.nano.angle[sensor_id].read_data(self.nano.ser)

            elif keyboard.is_pressed('4'):                # 读取步进电机0,1,2速度
                motor_id = int(input("Please input motor id: "))
                self.nano.motor[motor_id].read_speed(self.nano.ser)

            elif keyboard.is_pressed('5'):                # 设置步进电机0,1,2速度
                motor_id = int(input("Please input motor id: "))
                target_speed = float(input("Please input target speed: "))
                self.nano.motor[motor_id].set_speed(target_speed, self.nano.ser)

            elif keyboard.is_pressed("7"):                # 读取步进电机0,1,2位置
                motor_id = int(input("Please input motor id: "))
                self.nano.motor[motor_id].read_position(self.nano.ser)

            elif keyboard.is_pressed('2'):                # 设置步进电机0,1,2位置
                motor_id = int(input("Please input motor id: "))
                target_position = int(input("Please input target position: "))
                self.nano.motor[motor_id].set_position(target_position, self.nano.ser)

            elif keyboard.is_pressed('v'):                # 手动输入舵机位置
                servo_id = int(input("Please input servo id: "))
                target_position = int(input("Please input target position: "))
                self.Servo.move(servo_id, target_position, 0)

            elif keyboard.is_pressed(' '):
                print("stop")
                for i in range(3):
                    self.nano.motor[i].stop(self.nano.ser)

            # self.Servo.update()
            time.sleep(0.2)


no1 = DgripperCtrl("COM7", 115200, "COM13", 115200)
no1.run()
