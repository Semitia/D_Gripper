"""
This is a library for Bus Servo Controller. (幻尔总线舵机)
The board uses Serial for communication.

Author: Leonaruic
GitHub: github.com/semitia
Date: 2023-08-08
Version: 0.0.1
"""
import time
import serial
import threading
# import keyboard

def range_limit(num, my_min, my_max):
    """
    限幅函数
    """
    if num > my_max:
        num = my_max
    elif num < my_min:
        num = my_min
    return num


def checksum(buf):
    # 左闭右开
    temp = sum(buf[2:buf[3] + 2])
    return ~temp & 0xff


class Servo:
    def __init__(self):
        self.temp = 25
        self.pos = 0
        self.vin = 0


class ServoCtrl:
    SERVO_RANGE_MX = 1000
    SERVO_RANGE_MN = 0
    # 串口消息类型
    SERVO_FRAME_HEADER = 0x55
    SERVO_MOVE_TIME_WRITE = 1
    SERVO_MOVE_TIME_READ = 2
    SERVO_MOVE_TIME_WAIT_WRITE = 7
    SERVO_MOVE_TIME_WAIT_READ = 8
    SERVO_MOVE_START = 11
    SERVO_MOVE_STOP = 12
    SERVO_ID_WRITE = 13
    SERVO_ID_READ = 14
    SERVO_ANGLE_OFFSET_ADJUST = 17
    SERVO_ANGLE_OFFSET_WRITE = 18
    SERVO_ANGLE_OFFSET_READ = 19
    SERVO_ANGLE_LIMIT_WRITE = 20
    SERVO_ANGLE_LIMIT_READ = 21
    SERVO_VIN_LIMIT_WRITE = 22
    SERVO_VIN_LIMIT_READ = 23
    SERVO_TEMP_MAX_LIMIT_WRITE = 24
    SERVO_TEMP_MAX_LIMIT_READ = 25
    SERVO_TEMP_READ = 26
    SERVO_VIN_READ = 27
    SERVO_POS_READ = 28
    SERVO_OR_MOTOR_MODE_WRITE = 29
    SERVO_OR_MOTOR_MODE_READ = 30
    SERVO_LOAD_OR_UNLOAD_WRITE = 31
    SERVO_LOAD_OR_UNLOAD_READ = 32
    SERVO_LED_CTRL_WRITE = 33
    SERVO_LED_CTRL_READ = 34
    SERVO_LED_ERROR_WRITE = 35
    SERVO_LED_ERROR_READ = 36

    def __init__(self, port, baud, servo_num):
        self.ser = serial.Serial(port, baud)
        self.servo_num = servo_num
        self.servo_list = [Servo() for _ in range(servo_num)]
        self.got_frame_header = False
        self.frame_header_count = 0
        self.data_count = 0
        self.data_length = 2
        self.com_mutex = threading.Lock()
        self.rx_completed = False
        self.rx_buf = bytearray(15)  # 0,1:header; 2:id; 3:length; 4:cmd; 5~:data; -1:checksum
        self.ReadPortThread = threading.Thread(target=self.read_port)
        self.update_pos_thread = threading.Thread(target=self.update_pos_thread)
        # self.update_pos_thread.start()
        self.ReadPortThread.start()
        self.update()
        # for j in range(3):
        #     for i in range(self.servo_num):
        #         self.read_position(i)
        #         time.sleep(0.2)

        print("ServoCtrl init successfully with COM port: ", port, " baud: ", baud)

    def update_pos_thread(self):
        while True:
            for i in range(self.servo_num):
                self.com_mutex.acquire()                    # 获取互斥锁
                self.read_position(i)
                time.sleep(0.1)
                self.com_mutex.release()                    # 释放互斥锁

    def read_port(self):
        """
        monitor the serial port and process the received data in a thread
        """
        while True:
            if self.ser.in_waiting > 0:
                msg = self.ser.read(self.ser.in_waiting)
                # print("serial read", msg)
                for byte in msg:
                    self.process_byte(byte)

    def process_byte(self, byte):
        """
        process the received data, save it to rx_buf
        :param byte:
        :return:
        """
        # print("data_count", self.data_count, "process_byte: ", byte)

        if not self.got_frame_header:
            if byte == 0x55:
                self.frame_header_count += 1
                if self.frame_header_count == 2:
                    self.frame_header_count = 0
                    self.got_frame_header = True
                    self.data_count = 2
                    # print("got frame header!")
            else:
                self.got_frame_header = False
                self.data_count = 0
                self.frame_header_count = 0
                print("frame header error!")
        else:
            self.rx_buf[self.data_count] = byte
            if self.data_count == 3:
                self.data_length = self.rx_buf[self.data_count]
                if self.data_length < 3 or self.data_length > 7:
                    print("data length error!")
                    self.data_length = 3
                    self.got_frame_header = False
            self.data_count += 1
            if self.data_count == self.data_length + 3:
                # print("msg received successfully!")
                self.rx_completed = True
                self.got_frame_header = False

    def set_id(self, old_id, new_id):
        """
        设置舵机Id
        """
        buf = bytearray(7)
        buf[0] = buf[1] = self.SERVO_FRAME_HEADER
        buf[2] = old_id
        buf[3] = 4
        buf[4] = self.SERVO_ID_WRITE
        buf[5] = new_id
        buf[6] = checksum(buf)
        self.ser.write(buf)

    def move(self, servo_id, position, time_use):
        """
        用一定时间移动到指定位置
        """
        if position < 0:
            position = 0
        if position > 1000:
            position = 1000
        buf = bytearray(10)
        buf[0] = buf[1] = 0x55
        buf[2] = servo_id
        buf[3] = 7
        buf[4] = self.SERVO_MOVE_TIME_WRITE
        buf[5] = position & 0xff
        buf[6] = (position >> 8) & 0xff
        buf[7] = time_use & 0xff
        buf[8] = (time_use >> 8) & 0xff
        buf[9] = checksum(buf)
        self.ser.write(buf)

    def unload(self, servo_id):
        """
        卸载舵机
        """
        buf = bytearray(7)
        buf[0] = buf[1] = self.SERVO_FRAME_HEADER
        buf[2] = servo_id
        buf[3] = 4
        buf[4] = self.SERVO_LOAD_OR_UNLOAD_WRITE
        buf[5] = 0
        buf[6] = checksum(buf)
        self.ser.write(buf)

    def load(self, servo_id):
        """
        装载舵机
        """
        buf = bytearray(7)
        buf[0] = buf[1] = self.SERVO_FRAME_HEADER
        buf[2] = servo_id
        buf[3] = 4
        buf[4] = self.SERVO_LOAD_OR_UNLOAD_WRITE
        buf[5] = 1
        buf[6] = checksum(buf)
        self.ser.write(buf)

    def read_response(self):
        """
        读取舵机返回的数据
        """
        count = 50
        while not self.rx_completed:
            count -= 1
            time.sleep(0.001)
            # print("waiting for response")
            if count < 0:
                print("waiting time out")
                return -2048
        # self.ser.in_waiting,属性表示串口接收缓冲区中的字节数
        self.rx_completed = False  # 这个很重要，之前一直有问题
        buf = self.rx_buf
        if checksum(buf) != buf[buf[3] + 2]:
            print("checksum error")
            return -2049
        cmd = buf[4]
        if cmd == self.SERVO_POS_READ:
            ret = buf[6] << 8 | buf[5]
            return ret
        # elif
        return 0

    def read_position(self, servo_id):
        """
        读取舵机位置
        :param servo_id:
        :return:
        """
        buf = bytearray(6)
        buf[0] = buf[1] = self.SERVO_FRAME_HEADER
        buf[2] = servo_id
        buf[3] = 3
        buf[4] = self.SERVO_POS_READ
        buf[5] = checksum(buf)
        self.ser.write(buf)
        # print("read_position cmd has been sent")
        ret = self.read_response()
        print("read", servo_id, "position result: ", ret)
        self.servo_list[servo_id].pos = ret
        return ret

    def update(self):
        """
        更新各个舵机状态, 舵机编号0，1，2
        """
        for i in range(self.servo_num):
            self.servo_list[i].pos = self.read_position(i)

    def step_forward(self, servo_id):
        """
        舵机向前转
        :param servo_id:
        :return:
        """
        # self.com_mutex.acquire()                                # 获取互斥锁
        tem_pos = self.servo_list[servo_id].pos
        time.sleep(0.01)
        # print(tem_pos)
        tem_pos = range_limit(tem_pos - 30, self.SERVO_RANGE_MN, self.SERVO_RANGE_MX)
        # print(tem_pos)
        self.move(servo_id, tem_pos, 0)
        # self.com_mutex.release()                                # 释放互斥锁

    def step_backward(self, servo_id):
        """
        舵机向后转
        :param servo_id:
        :return:
        """
        tem_pos = self.read_position(servo_id)
        time.sleep(0.01)
        tem_pos = range_limit(tem_pos + 30, self.SERVO_RANGE_MN, self.SERVO_RANGE_MX)
        # print(tem_pos)
        self.move(servo_id, tem_pos, 0)


# if __name__ == "__main__":
#     servo = ServoCtrl("COM6", 115200, 2)
#     time.sleep(1)
#     while True:
#         if keyboard.is_pressed('q'):
#             break

#         elif keyboard.is_pressed('w'):
#             servo.step_forward(0)
#             print("w")

#         elif keyboard.is_pressed('e'):
#             servo.step_forward(1)
#             print("e")

#         elif keyboard.is_pressed('s'):  # 夹爪0闭合
#             print("s")
#             servo.step_backward(0)

#         elif keyboard.is_pressed('d'):  # 夹爪1闭合
#             print("d")
#             servo.step_backward(1)

#         servo.update()
#         time.sleep(0.01)

#     servo.ser.close()
#     print("serial port closed")
#     print("program exit")
#     exit(0)

