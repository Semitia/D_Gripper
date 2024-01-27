"""

"""

import time
from STM32Ctrl import STM32Ctrl
import sys
import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
        
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

sys.path.append("..")
from scservo_sdk import *                      # Uses SCServo SDK library




class DgripperCtrl:
    SERVO_NUM = 3

    def __init__(self, servo_port, servo_baud, board_port, board_baud):
        # STM32
        self.board = STM32Ctrl(board_port, board_baud)
        # Feetech servo
        self.portHandler = PortHandler(servo_port)
        self.packetHandler = sms_sts(self.portHandler)

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()
        # Set port baudrate
        if self.portHandler.setBaudRate(servo_baud):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()
        print("DgripperCtrl Init Complete!")
        
        
    def write_servo(self, servo_id, position, speed, acc):
        scs_comm_result, scs_error = self.packetHandler.WritePosEx(servo_id, position, speed, acc)
        # if scs_comm_result != COMM_SUCCESS:
        #     print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        # elif scs_error != 0:
        #     print("%s" % self.packetHandler.getRxPacketError(scs_error))

    def read_servo(self, servo_id):
        scs_present_position, scs_present_speed, scs_comm_result, scs_error = self.packetHandler.ReadPosSpeed(servo_id)
        # if scs_comm_result != COMM_SUCCESS:
        #     print(self.packetHandler.getTxRxResult(scs_comm_result))
        # elif scs_error != 0:
        #     print(self.packetHandler.getRxPacketError(scs_error))
        moving, scs_comm_result, scs_error = self.packetHandler.ReadMoving(servo_id)
        if scs_comm_result != COMM_SUCCESS:
            print(self.packetHandler.getTxRxResult(scs_comm_result))
        else:

            print("[ID:%03d] PresPos:%d PresSpd:%d" % (servo_id, scs_present_position, scs_present_speed))
        if scs_error != 0:
            print(self.packetHandler.getRxPacketError(scs_error))

    def key_run(self):
        while True:
            print("Press any key to continue! (or press ESC to quit!)")
            key = getch()
            if key == chr(0x1b):
                break
            elif key == "q":          # servo 0 clockwise
                self.write_servo(0, 0, 1500, 50)
                print("servo 0 clockwise")
            elif key == "a":        # servo 0 counter-clockwise
                self.write_servo(0, 0, -800, 50)
                print("servo 0 counter-clockwise")
            elif key == "w":        # servo 1 clockwise
                self.write_servo(1, 0, 1500, 50) 
                print("servo 1 clockwise")
            elif key == "s":        # servo 1 counter-clockwise
                self.write_servo(1, 0, -800, 50)
                print("servo 1 counter-clockwise")
            elif key == "u":        # n20[0] clockwise
                self.board.motor[0].set_speed(5, self.board.ser)
                print("n20[0] clockwise")
            elif key == "j":        # n20[0] counter-clockwise
                self.board.motor[0].set_speed(-5, self.board.ser)
                print("n20[0] counter-clockwise")
            elif key == "i":        # n20[1] clockwise
                self.board.motor[1].set_speed(5, self.board.ser)
                print("n20[1] clockwise")
            elif key == "k":        # n20[1] counter-clockwise
                self.board.motor[1].set_speed(-5, self.board.ser)
                print("n20[1] counter-clockwise")
            elif key == "o":        # n20[2] clockwise
                self.board.motor[2].set_speed(8, self.board.ser)
                print("n20[2] clockwise")
            elif key == "l":        # n20[2] counter-clockwise
                self.board.motor[2].set_speed(-8, self.board.ser)
                print("n20[2] counter-clockwise")
            elif key == "1":        # set position of motor
                motor_id = int(input("motor id:"))
                position_tar = input("position:")
                self.board.motor[motor_id].set_position(position_tar, self.board.ser)
                print("set position of motor", motor_id, "to", position_tar)
            elif key == "2":        # read position of motor
                motor_id = int(input("motor id:"))
                self.board.motor[motor_id].read_position(self.board.ser)
            elif key == "3":        # read speed of motor
                motor_id = int(input("motor id:"))
                self.board.motor[motor_id].read_speed(self.board.ser)

            elif key == " ":        # stop all
                for i in range(3):
                    self.board.motor[i].stop(self.board.ser)
                self.write_servo(0, 0, 0, 50)
                self.write_servo(1, 0, 0, 50)
                print("stop all")

            time.sleep(0.05)

    def update(self):
        for i in range(3):
            self.board.motor[i].read_position(self.board.ser)
        self.read_servo(0)
        self.read_servo(1)

    def manipulate(self, pose, tar_pose):
        complete = False
        state = 0
        while not complete:
            
            self.update()
            time.sleep(0.05)
        

SERVO_BAUDRATE              = 1000000           
SERVO_PORTNAME              = '/dev/ttyUSB0'   
BOARD_BAUDRATE              = 115200          
BOARD_PORTNAME              = '/dev/ttyUSB1' 
if __name__ == '__main__':
    no1 = DgripperCtrl(SERVO_PORTNAME, SERVO_BAUDRATE, BOARD_PORTNAME, BOARD_BAUDRATE)
    no1.key_run()
