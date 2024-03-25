"""

"""

import time
from STM32Ctrl import STM32Ctrl
from HuanerServoCtrl import ServoCtrl
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

PI = 3.1415926536
N20_RAD2DIS = 1.707553                         # translate angle(rad) of n20 to displacement(mm) of surface
SCREW_RAD2DIS = 1.227948                       # translate screw's angle(rad) to surface distance(mm) of 2 fingers
SERVO_VAL2ANG = 0.00419                        # translate servo's angle digital value(0~1000) to actual value(+-PI)
# range limitation of state
ROLL_MAX_SPD = 4
SCREW_MAX_DIS = 53
SCREW_MAX_SPD = 10
SCREW_MIN_DIS = -20
SERVO_MAX_ANG =  PI/2
SERVO_MIN_ANG = -PI/2
SCREW_INIT_DIS = 32.8      # motor pos=0 <--> dis=122.8mm, assume that the thickness of sensor is 45mm, init_dis = 122.8-45x2=32.8mm  
SERVO_INIT_VAL_0 = 430
SERVO_INIT_VAL_1 = 420

def limit(val, min_val, max_val):
    if val > max_val:
        val = max_val
    elif val < min_val:
        val = min_val
    return val

class DgripperCtrl_H:
    SERVO_NUM = 2
    def __init__(self, servo_port, servo_baud, board_port, board_baud):
        ## STM32
        self.board = STM32Ctrl(board_port, board_baud)
        ## Huaner servo
        self.servo = ServoCtrl(servo_port, servo_baud, 2)
        self.servo.update()
        self.servo_pos = [0] * self.SERVO_NUM
        self.servo_pos[0] = self.servo.servo_list[0].pos
        self.servo_pos[1] = self.servo.servo_list[1].pos
        # state: surface displacement(mm) of 2 sensors , distance(mm) of 2 fingers , 2 servo's target angle(rad); 
        self.state = [0, 0, 0, 0, 0]
        #                 ..............  surface slide speed[0/1], screw speed
        self.state_tar = [0, 0, 50, 0, 0, 0, 0, 0]


    def key_run(self):
        while True:
            print("Press any key to continue! (or press ESC to quit!)")
            key = getch()
            if key == chr(0x1b):
                break
            elif key == "q":          # servo 0 clockwise
                self.servo.step_forward(0)
                print("servo 0 clockwise")
            elif key == "a":        # servo 0 counter-clockwise
                self.servo.step_backward(0)
                print("servo 0 counter-clockwise")
            elif key == "w":        # servo 1 clockwise
                self.servo.step_forward(1)
                print("servo 1 clockwise")
            elif key == "s":        # servo 1 counter-clockwise
                self.servo.step_backward(1)
                print("servo 1 counter-clockwise")
            elif key == "e":        # put in servo 's target value
                servo_id = int(input("servo id:"))
                target = int(input("target value:"))
                self.servo.move(servo_id, target, 0)

            elif key == "u":        # n20[0] clockwise
                self.board.motor[0].set_speed(3, self.board.ser)
                print("n20[0] clockwise")
            elif key == "j":        # n20[0] counter-clockwise
                self.board.motor[0].set_speed(-3, self.board.ser)
                print("n20[0] counter-clockwise")
            elif key == "i":        # n20[1] clockwise
                self.board.motor[1].set_speed(3, self.board.ser)
                print("n20[1] clockwise")
            elif key == "k":        # n20[1] counter-clockwise
                self.board.motor[1].set_speed(-3, self.board.ser)
                print("n20[1] counter-clockwise")
            elif key == "o":        # n20[2] clockwise
                self.board.motor[2].set_speed(10, self.board.ser) 
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
            elif key == "b":        # back to zero
                self.reset_zero()

            elif key == " ":        # stop all
                for i in range(3):
                    self.board.motor[i].stop(self.board.ser)
                print("stop all")

            time.sleep(0.05)
            self.servo.update()

    def update(self):
        for i in range(3):
            self.board.motor[i].read_position(self.board.ser)
        self.servo.update()
        self.servo_pos[0] = self.servo.servo_list[0].pos
        self.servo_pos[1] = self.servo.servo_list[1].pos
        
        self.state[0] = self.board.motor[0].position * N20_RAD2DIS
        self.state[1] = self.board.motor[1].position * N20_RAD2DIS
        self.state[2] = self.board.motor[2].position * SCREW_RAD2DIS + SCREW_INIT_DIS
        self.state[3] = self.servo_pos[0] * SERVO_VAL2ANG - PI
        self.state[4] = self.servo_pos[1] * SERVO_VAL2ANG - PI
        print("state:", self.state)

    def excute_motor(self):
        # limit range()
        self.state_tar[2] = limit(self.state_tar[2], SCREW_MIN_DIS, SCREW_MAX_DIS)
        self.state_tar[3] = limit(self.state_tar[3], SERVO_MIN_ANG, SERVO_MAX_ANG)
        self.state_tar[4] = limit(self.state_tar[4], SERVO_MIN_ANG, SERVO_MAX_ANG)
        self.state_tar[5] = limit(self.state_tar[5], -ROLL_MAX_SPD, ROLL_MAX_SPD)
        self.state_tar[6] = limit(self.state_tar[6], -ROLL_MAX_SPD, ROLL_MAX_SPD)
        self.state_tar[7] = limit(self.state_tar[7], -SCREW_MAX_SPD, SCREW_MAX_SPD)

        for i in range(3):
            self.board.motor[i].set_speed(self.state_tar[i+5], self.board.ser)
        
        servo_pos = [self.state_tar[3]/SERVO_VAL2ANG + SERVO_INIT_VAL_0, self.state_tar[4]/SERVO_VAL2ANG + SERVO_INIT_VAL_1]
        # self.write_servo(0, int(servo_pos[0]), 500, 50)
        # self.write_servo(1, int(servo_pos[1]), 500, 50)
        self.servo.move(0, int(servo_pos[0]), 0)
        self.servo.move(1, int(servo_pos[1]), 0)
        
    def reset_zero(self):
        '''
        reset all motors to zero position
        In order to ensure the accuracy of position control,
        gripper should be reset to zero position as long as it's powered off.
        '''
        self.state_tar = [0, 0, 50, 0, 0, 0, 0, 0]
        self.excute_motor()
    
    def manipulate(self, pose, tar_pose):
        complete = False
        state = 0
        while not complete:
            
            self.update()
            time.sleep(0.05)
    
        
SERVO_BAUDRATE              = 115200           
SERVO_PORTNAME              = '/dev/ttyUSB1'   
BOARD_BAUDRATE              = 115200          
BOARD_PORTNAME              = '/dev/ttyUSB2' 
if __name__ == '__main__':
    no1 = DgripperCtrl_H(SERVO_PORTNAME, SERVO_BAUDRATE, BOARD_PORTNAME, BOARD_BAUDRATE)
    no1.key_run()
