import os
import time
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray

LEFT = 0
RIGHT = 1
DISPLACEMENT_SCALE = 1
DISTANCE_SCALE = 1
ANGLE_SCALE = 0.2

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
    print("motor state:\r\n left roll: %f, left pitch: %f, right roll: %f, right pitch: %f, distance: %f" % (state_real[0], state_real[3], state_real[1], state_real[4], state_real[2]))

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
    stick[RIGHT].x = msg.linear.x
    stick[RIGHT].y = msg.linear.y

def get_tar_state():
    if mode == True: # 控制pitch舵机旋转
        for i in range(2):
            delta_angle = stick[i].y * ANGLE_SCALE
            state_tar[i + 3] = state_real[i + 3] + delta_angle
    else: # 控制n20电机, 夹爪间距和皮带周转
        for i in range(2):
            delta_x = stick[i].y * DISPLACEMENT_SCALE
            state_tar[i] = state_real[i] + delta_x
        delta_distance = (stick[RIGHT].x - stick[LEFT].x) * DISTANCE_SCALE 
        state_tar[2] = state_real[2] + delta_distance
    print("tar state:\r\n left roll: %f, left pitch: %f,\r\n right roll: %f, right pitch: %f, distance: %f" % (state_tar[0], state_tar[3], state_tar[1], state_tar[4], state_tar[2]))

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
        get_tar_state()
        pub_msg = Float32MultiArray(data = state_tar)
        # pub.publish(pub_msg)
        rate.sleep()
        
        