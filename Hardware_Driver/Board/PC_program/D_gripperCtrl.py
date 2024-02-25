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

PI = 3.1415926536
N20_RAD2DIS = 1.707553                         # translate angle(rad) of n20 to displacement(mm) of surface
SCREW_RAD2DIS = 1.227948                       # translate screw's angle(rad) to surface distance(mm) of 2 fingers
SERVO_VAL2ANG = 0.0015339807878                # translate servo's angle digital value(0~4095) to actual value(+-PI)
# range limitation of state
SCREW_MAX_DIS = 53
SCREW_MIN_DIS = -5
SERVO_MAX_ANG =  PI/2
SERVO_MIN_ANG = -PI/2
SCREW_INIT_DIS = 26.326634    # motor pos=0 <--> dis=168.326634
SERVO_INIT_VAL_0 = 2048
SERVO_INIT_VAL_1 = 2048

def limit(val, min_val, max_val):
    if val > max_val:
        val = max_val
    elif val < min_val:
        val = min_val
    return val

class DgripperCtrl:
    SERVO_NUM = 2
    def __init__(self, servo_port, servo_baud, board_port, board_baud):
        ## STM32
        self.board = STM32Ctrl(board_port, board_baud)
        
        ## Feetech servo
        self.servo_pos = [0] * self.SERVO_NUM
        self.servo_delta_pos = 60                       # the delta position after pressing one time
        self.portHandler = PortHandler(servo_port)
        self.packetHandler = sms_sts(self.portHandler)
        print("DgripperCtrl Init Complete!")
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
        
        self.servo_pos[0] = self.read_servo(0)
        self.servo_pos[1] = self.read_servo(1)
        # state: surface displacement(mm) of 2 sensors & distance(mm) of 2 fingers & 2 servo's target angle(rad); 
        self.state = [0, 0, 0, 0, 0]
        self.state_tar = [0, 0, 0, 0, 0]

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
        return scs_present_position

    def key_run(self):
        while True:
            print("Press any key to continue! (or press ESC to quit!)")
            key = getch()
            if key == chr(0x1b):
                break
            elif key == "q":          # servo 0 clockwise
                self.write_servo(0, self.servo_pos[0] + self.servo_delta_pos, 500, 50)
                print("servo 0 clockwise")
            elif key == "a":        # servo 0 counter-clockwise
                self.write_servo(0, self.servo_pos[0] - self.servo_delta_pos, 500, 50)
                print("servo 0 counter-clockwise")
            elif key == "w":        # servo 1 clockwise
                self.write_servo(1, self.servo_pos[1] + self.servo_delta_pos, 500, 50)
                print("servo 1 clockwise")
            elif key == "s":        # servo 1 counter-clockwise
                self.write_servo(1, self.servo_pos[1] - self.servo_delta_pos, 500, 50)
                print("servo 1 counter-clockwise")
            elif key == "u":        # n20[0] clockwise
                self.board.motor[0].set_speed(2, self.board.ser)
                print("n20[0] clockwise")
            elif key == "j":        # n20[0] counter-clockwise
                self.board.motor[0].set_speed(-2, self.board.ser)
                print("n20[0] counter-clockwise")
            elif key == "i":        # n20[1] clockwise
                self.board.motor[1].set_speed(2, self.board.ser)
                print("n20[1] clockwise")
            elif key == "k":        # n20[1] counter-clockwise
                self.board.motor[1].set_speed(-2, self.board.ser)
                print("n20[1] counter-clockwise")
            elif key == "o":        # n20[2] clockwise
                self.board.motor[2].set_speed(10, self.board.ser) # 
                print("n20[2] clockwise")
            elif key == "l":        # n20[2] counter-clockwise
                self.board.motor[2].set_speed(-10, self.board.ser)
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
                # self.write_servo(0, 0, 0, 50)
                # self.write_servo(1, 0, 0, 50)
                print("stop all")

            time.sleep(0.05)
            self.servo_pos[0] = self.read_servo(0)
            self.servo_pos[1] = self.read_servo(1)

    def update(self):
        for i in range(3):
            self.board.motor[i].read_position(self.board.ser)
        self.servo_pos[0] = self.read_servo(0)
        self.servo_pos[1] = self.read_servo(1)
        
        self.state[0] = self.board.motor[0].position * N20_RAD2DIS
        self.state[1] = self.board.motor[1].position * N20_RAD2DIS
        self.state[2] = self.board.motor[2].position * SCREW_RAD2DIS
        self.state[3] = self.servo_pos[0] * SERVO_VAL2ANG
        self.state[4] = self.servo_pos[1] * SERVO_VAL2ANG
        print("state:", self.state)

    def excute_motor(self):
        # limit range()
        self.state_tar[2] = limit(self.state_tar[2], SCREW_MIN_DIS, SCREW_MAX_DIS)
        self.state_tar[3] = limit(self.state_tar[3], SERVO_MIN_ANG, SERVO_MAX_ANG)
        self.state_tar[4] = limit(self.state_tar[4], SERVO_MIN_ANG, SERVO_MAX_ANG)
        # translate state to target position, excute
        tar_pos = [self.state_tar[0]/N20_RAD2DIS, self.state_tar[1]/N20_RAD2DIS, self.state_tar[2]/SCREW_RAD2DIS]
        for i in range(3):
            self.board.motor[i].set_position(tar_pos[i], self.board.ser)
        self.write_servo(0, self.state_tar[3]/SERVO_VAL2ANG, 500, 50)
        self.write_servo(1, self.state_tar[4]/SERVO_VAL2ANG, 500, 50)
        
    def reset_zero(self):
        '''
        reset all motors to zero position
        In order to ensure the accuracy of position control,
        gripper should be reset to zero position as long as it's powered off.
        '''
        self.state_tar = [0, 0, SCREW_INIT_DIS, SERVO_INIT_VAL_0, SERVO_INIT_VAL_1]
        self.excute_motor()
    
    def manipulate(self, pose, tar_pose):
        complete = False
        state = 0
        while not complete:
            
            self.update()
            time.sleep(0.05)
        
        
SERVO_BAUDRATE              = 1000000           
SERVO_PORTNAME              = '/dev/ttyUSB1'   
BOARD_BAUDRATE              = 115200          
BOARD_PORTNAME              = '/dev/ttyUSB0' 
if __name__ == '__main__':
    no1 = DgripperCtrl(SERVO_PORTNAME, SERVO_BAUDRATE, BOARD_PORTNAME, BOARD_BAUDRATE)
    no1.key_run()
