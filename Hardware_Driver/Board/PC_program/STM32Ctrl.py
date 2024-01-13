"""
This is a serial controller for Arduino nano board, which is equipped with 3 Stepper Motors and 3 Angle Sensors.
It works with the Stepperx3.ino.

author: Leonaruic
GitHub: github.com/semitia
date: 2023-09-04
version: 0.0.1
"""
from N20 import N20Ctrl
from AngleSensor import AngleSensor
import serial
import threading
import numpy as np


def add_tail(cmd):
    # 在指令后面添加0x0D 0x0A作为帧尾
    cmd.extend([0x0D, 0x0A])  # STM32的是\r\n
    return cmd


class STM32Ctrl:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud)
        self.motor = [None] * 3  # 初始化self.motor
        self.motor = [N20Ctrl(i) for i in range(3)]      # 3个电机
        self.angle = [None] * 3  # 初始化self.angle
        self.angle = [AngleSensor(i) for i in range(3)]      # 3个角度传感器
        self.rxbuf = bytearray()
        self.end_flag = 0
        # self.rxbuf_len = 0
        self.ReadPortThread = threading.Thread(target=self.read_port)
        self.ReadPortThread.start()
        print("Init STM32 board Complete with port:", port, "baud:", baud)

    def read_port(self):
        while True:
            if self.ser.in_waiting > 0:
                msg = self.ser.read(self.ser.in_waiting)
                # print(msg)
                for byte in msg:
                    self.process_byte(byte)

    def process_byte(self, byte):
        self.rxbuf.append(byte)
        # 结束标识符 0x41 0x49
        if self.end_flag == 0:
            if byte == 0x41:
                self.end_flag = 1
        elif self.end_flag == 1:
            if byte == 0x49:
                self.end_flag = 0
                # print("received ", self.rxbuf)
                self.process_frame()
            else:
                self.end_flag = 0
                self.rxbuf.clear()
                print("Error: end_flag = 1, byte != 0x49")
        return

    def process_frame(self):
        id_num = self.rxbuf[0]
        if self.rxbuf[1] == 0x03:
            # 读取速度
            speed = np.int16(self.rxbuf[2] << 8 | self.rxbuf[3])/1000  # 电机速度rad/s * 1000
            self.motor[id_num].speed_update(speed)
            print("motor", id_num, "'s speed", speed)
        elif self.rxbuf[1] == 0x04:
            # 读取位置
            pos = np.uint16(self.rxbuf[2] << 8 | self.rxbuf[3])/1000
            self.motor[id_num].pos_update(pos)
            print("motor", id_num, "'s pos", pos)

        elif self.rxbuf[1] == 0x07:
            # 读取角度
            angle = np.int16(self.rxbuf[2] << 8 | self.rxbuf[3])  # 角度(°) * 100
            angle /= 100
            self.angle[id_num].update(angle)
            print("angleSs", id_num, "'s angle", angle)
        self.rxbuf.clear()


