"""

"""
import rospy
import geometry_msgs.msg
import time
from D_gripperCtrl import DgripperCtrl
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
    
    
class DGripper_ros(DgripperCtrl):
    def __init__(self, servo_port, servo_baud, board_port, board_baud):
        super(DGripper_ros, self).__init__(servo_port, servo_baud, board_port, board_baud)
        rospy.init_node("motors")
        # self.pub = rospy.Publisher("motors", String, queue_size=10)
        self.image_sub = rospy.Subscriber("/raw_image", Image, self.image_callback)
        
    def image_callback(self, msg):
        print("ok")
    
    def run(self):
        while not rospy.is_shutdown():
            print("ok")
    


SERVO_BAUDRATE              = 1000000           
SERVO_PORTNAME              = '/dev/ttyUSB1'   
BOARD_BAUDRATE              = 115200          
BOARD_PORTNAME              = '/dev/ttyUSB0' 
if __name__ == '__main__':
    gripper = DGripper_ros(SERVO_PORTNAME, SERVO_BAUDRATE, BOARD_PORTNAME, BOARD_BAUDRATE)
    gripper.run()
