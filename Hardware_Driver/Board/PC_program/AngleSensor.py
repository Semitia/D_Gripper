"""
This is a library for an Angle sensor, whose feedback is an analog signal.
The range of the analog signal is 0~5V (if its voltage supply is 5V), corresponding to -160~160 degree.
We acquire the analog signal by Arduino nano's ADC, and then send it to the PC by serial.
"""
import serial
import time

class AngleSensor:
    def __init__(self, num_id):
        self.num_id = num_id
        self.angle = 0
        self.angle_offset = 0
        self.angle_raw = 0
        self.vol = 0
        self.update_flag = 0

    def set_angle_offset(self, angle_offset):
        self.angle_offset = angle_offset

    def update(self, voltage):
        self.vol = voltage
        self.angle_raw = (voltage - 2.5) * 160 / 2.5
        self.angle = self.angle_raw - self.angle_offset
        self.update_flag = 1
        return self.angle

    def read_data(self, ser):
        """
        发送读取角度指令，读取角度数据
        """
        cmd = bytearray([self.num_id, 0x07, 0x41, 0x49])
        ser.write(cmd)
        count = 50
        while not self.update_flag:
            count -= 1
            time.sleep(0.001)
            if count <= 0:
                print("read angle timeout")
                return
        self.update_flag = False
        print("received successfully")
        return self.angle

    def get_angle(self):
        return self.angle

