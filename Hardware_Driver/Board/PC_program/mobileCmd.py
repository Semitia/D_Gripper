import os
import time
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray

LEFT = 0
RIGHT = 1

mode = False # 决定摇杆是控制pitch旋转(True) 还是 传感器roll与间距(False)
# 五个电机位置信息 N20[0\1] 位移, N20[2]夹爪间距, servo[0\1] 角度 
state_real = [0, 0, 0, 0, 0]
state_tar  = [0, 0, 0, 0, 0]
# 记录两个摇杆状态的全局变量
class JoyState:
    def __init__(self):
        self.x = 0
        self.y = 0
stick = [JoyState(), JoyState()]


def motor_state_callback(msg):
    state_real = msg.data
    print("motor state:", state_real)

def button_callback(msg):
    if msg.data == True:
        print("button pressed")
        mode = not mode
        # 清空摇杆状态
        stick[LEFT].x = 0
        stick[LEFT].y = 0
        stick[RIGHT].x = 0
        stick[RIGHT].y = 0


def joy1_callback(msg):
    print("joy1:", msg)
    stick[LEFT].x = msg.linear.x
    stick[LEFT].y = msg.linear.y
    

def joy2_callback(msg):
    print("joy2:", msg)

if __name__ == "__main__":
    rospy.init_node("moblleTransmit")
    pub = rospy.Publisher("motor_pos_tar",Float32MultiArray, queue_size=10)
    state_sub = rospy.Subscriber("/motors_pos_real", Float32MultiArray, motor_state_callback)
    mobile_sub_btn = rospy.Subscriber("/mobile_btn", Bool, button_callback)
    mobile_sub_joy =   [rospy.Subscriber("/mobile_joy_left", Twist, joy1_callback), \
                        rospy.Subscriber("/mobile_joy_right", Twist, joy2_callback)]

    # 设置循环频率
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        pub.publish(state_tar)
        rate.sleep()
        
        