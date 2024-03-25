"""

"""
import rospy
import time
import os
import cv2 

from cv_bridge import CvBridge 
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from D_gripperCtrl_H import DgripperCtrl_H
from std_msgs.msg import Float32MultiArray

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
    
    
class DGripper_ros_H(DgripperCtrl_H):
    '''
    DGripper的ROS控制接口, 三种控制模式
    0 - 订阅5个目标电机位置直接来控制电机
    1 - 订阅位姿变换信息与接触点信息, 计算后控制电机
    2 - 订阅两个触觉图像与位姿变换信息, 处理后控制电机
    通过publisher不断发送电机位置信息
    '''
    def __init__(self, servo_port, servo_baud, board_port, board_baud):
        super(DGripper_ros_H, self).__init__(servo_port, servo_baud, board_port, board_baud)
        self.CTRL_MODE = 0
        # ROS
        rospy.init_node("DGripper_ros")
        # 五个电机位置信息 N20[0\1] 位移, N20[2]夹爪间距, servo[0\1] 角度 
        self.pub = rospy.Publisher("/motors_pos_real", Float32MultiArray, queue_size=10)
        # 左右两个摄像头的图像存储列表
        self.BUF_SIZE = 10
        self.image_left_list = []
        self.image_right_list = []
        self.image_sub_left  = rospy.Subscriber("/image_left", Image, self.image_left_callback)
        self.image_sub_right = rospy.Subscriber("/image_right", Image, self.image_right_callback)
        # 电机位置命令订阅  -  数据格式同pub_msg
        self.motor_sub = rospy.Subscriber("/motor_pos_tar", Float32MultiArray, self.motor_callback)
        # 位姿变换命令订阅 
        self.pose_sub = rospy.Subscriber("/pose_tar", Pose, self.pose_callback)
        
    def image_left_callback(self, msg):
        '''
        图像订阅回调函数
        '''
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("image", image)
        key = cv2.waitKey(1)
        if key == ord('y'):
            #保存最新数据
            self.save_latest_data()
        if key == ord('q'):
            quit()
        # 将图像数据添加到缓存列表中
        self.image_left_list.append(image)
        # 如果缓存列表超过了最大数目，则删除最旧的数据
        if len(self.image_left_list) > self.BUF_SIZE:
            self.image_left_list.pop(0)

    def image_right_callback(self, msg):
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("image", image)
        key = cv2.waitKey(1)
        if key == ord('y'):
            #保存最新数据
            self.save_latest_data()
        if key == ord('q'):
            quit()
        # 将图像数据添加到缓存列表中
        self.image_right_list.append(image)
        # 如果缓存列表超过了最大数目，则删除最旧的数据
        if len(self.image_right_list) > self.BUF_SIZE:
            self.image_right_list.pop(0)

    def motor_callback(self, msg):
        '''
        电机位置命令订阅回调函数
        '''
        self.CTRL_MODE = 0
        self.state_tar = list(msg.data)
        print("target: ", self.state_tar)
        
    def pose_callback(self, msg):
        '''
        位姿变换命令订阅回调函数
        '''
        self.CTRL_MODE = 1
        # self.pose_tar = msg
        print("pose callback: ", self.pose_tar)
    
    def run(self):
        # Create a rate object with a rate of 10Hz
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            # rospy.spinOnce()
            if self.CTRL_MODE == 0:
                self.excute_motor()
            # elif self.CTRL_MODE == 1:
            
            self.update()
            # 电机位置信息发布
            pub_msg = Float32MultiArray(data = self.state)
            self.pub.publish(pub_msg)
            rate.sleep()
    


SERVO_BAUDRATE              = 115200        
SERVO_PORTNAME              = '/dev/ttyUSB1'   
BOARD_BAUDRATE              = 115200          
BOARD_PORTNAME              = '/dev/ttyUSB2' 
if __name__ == '__main__':
    gripper = DGripper_ros_H(SERVO_PORTNAME, SERVO_BAUDRATE, BOARD_PORTNAME, BOARD_BAUDRATE)
    gripper.run()
