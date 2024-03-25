import os
import time
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray

LEFT = 0
RIGHT = 1
DISPLACEMENT_SCALE = 8
DISTANCE_SCALE = 3
ANGLE_SCALE = 0.02
PI = 3.1415926

mode = False # 决定摇杆是控制pitch旋转(True) 还是 传感器roll与间距(False)
# button flags
rst_flag = False
l_a90_flag = False
r_a90_flag = False
l_s90_flag = False
r_s90_flag = False

# 五个电机位置信息 N20[0\1] 位移, N20[2]夹爪间距, servo[0\1] 角度 
state_real = [0, 0, 0, 0, 0]
# add 3 motors' target speed behind
state_tar  = [0, 0, 50, 0, 0, 0, 0, 0]
# 记录两个摇杆状态的全局变量
class JoyState:
    def __init__(self):
        self.x = 0
        self.y = 0
stick = [JoyState(), JoyState()]

def motor_state_callback(msg):
    global state_real
    state_real = msg.data
    # print("motor state:\r\n left roll: %f, left pitch: %f, right roll: %f, right pitch: %f, distance: %f" % (state_real[0], state_real[3], state_real[1], state_real[4], state_real[2]))

def button_callback(msg):
    global mode, stick
    if msg.data == True:
        mode = not mode
        print("button pressed, now mode is ", mode)
        # 清空摇杆状态
        stick[LEFT].x = 0
        stick[LEFT].y = 0
        stick[RIGHT].x = 0
        stick[RIGHT].y = 0

def rst_button_callback(msg):
    global rst_flag
    if msg.data == True:
        rst_flag = True
    print("reset button pressed")
    
def left_a90_callback(msg):
    global l_a90_flag
    if msg.data == True:
        l_a90_flag = True
    print("left + 90")

def right_a90_callback(msg):
    global r_a90_flag
    if msg.data == True:
        r_a90_flag = True
    print("right + 90")

def left_s90_callback(msg):
    global l_s90_flag
    if msg.data == True:
        l_s90_flag = True
    print("left - 90")

def right_s90_callback(msg):
    global r_s90_flag
    if msg.data == True:
        r_s90_flag = True
    print("right - 90")

def joy1_callback(msg):
    # print("joy1:", msg)
    global stick
    stick[LEFT].x = msg.linear.x
    stick[LEFT].y = msg.linear.y
    
def joy2_callback(msg):
    # print("joy2:", msg)
    global stick
    stick[RIGHT].x = msg.linear.x
    stick[RIGHT].y = msg.linear.y

def get_tar_state():
    global state_tar, state_real, mode, stick, rst_flag, l_a90_flag, r_a90_flag, l_s90_flag, r_s90_flag
    if rst_flag == True:
        state_tar = [0, 0, 50, 0, 0, 0, 0, 0]
        rst_flag = False
        print("reset button pressed")
        return
    if l_a90_flag == True:
        state_tar[3] = state_real[3] - PI/2
        l_a90_flag = False
    if l_s90_flag == True:
        state_tar[3] = state_real[3] + PI/2
        l_s90_flag = False
    if r_a90_flag == True:
        state_tar[4] = state_real[4] + PI/2
        r_a90_flag = False
    if r_s90_flag == True:
        state_tar[4] = state_real[4] - PI/2
        r_s90_flag = False

    if mode == True: # 控制pitch舵机旋转
        delta_angle = stick[0].y * ANGLE_SCALE
        state_tar[3] = state_real[3] + delta_angle
        delta_angle = stick[1].y * ANGLE_SCALE
        state_tar[4] = state_real[4] - delta_angle
        # print("delta_angle: ", delta_angle)

        state_tar[5] = 0
        state_tar[6] = 0
        state_tar[7] = 0

    else: # 控制n20电机, 夹爪间距和皮带周转
        for i in range(2):
            delta_x = stick[i].y * DISPLACEMENT_SCALE
            state_tar[i] = state_real[i] + delta_x
        delta_distance = (stick[RIGHT].x - stick[LEFT].x) * DISTANCE_SCALE 
        state_tar[2] = state_real[2] + delta_distance
        # print("delta_distance: ", delta_distance)

        state_tar[5] = stick[0].y*3
        state_tar[6] = stick[1].y*3
        state_tar[7] = delta_distance*1.5

    print("tar state:\r\n left roll: %f, left pitch: %f,\r\n right roll: %f, right pitch: %f, distance: %f" % (state_tar[0], state_tar[3], state_tar[1], state_tar[4], state_tar[2]))
    print("tar speed: ", state_tar[5], state_tar[6], state_tar[7])


if __name__ == "__main__":
    rospy.init_node("moblleTransmit")
    pub = rospy.Publisher("motor_pos_tar",Float32MultiArray, queue_size=10)
    state_sub = rospy.Subscriber("/motors_pos_real", Float32MultiArray, motor_state_callback)
    mobile_sub_btn = rospy.Subscriber("/mobile_btn", Bool, button_callback)
    mobile_rst_btn = rospy.Subscriber("/reset_btn", Bool, rst_button_callback)
    mobile_sub_joy =   [rospy.Subscriber("/mobile_joy_left", Twist, joy1_callback), \
                        rospy.Subscriber("/mobile_joy_right", Twist, joy2_callback)]
    left_a_90 = rospy.Subscriber("/left_a_90", Bool, left_a90_callback)
    left_s_90 = rospy.Subscriber("/left_s_90", Bool, left_s90_callback)
    right_a_90 = rospy.Subscriber("/right_a_90", Bool, right_a90_callback)
    right_s_90 = rospy.Subscriber("/right_s_90", Bool, right_s90_callback)
    # 设置循环频率
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():

        sleep_flag = False
        if rst_flag or l_a90_flag or l_s90_flag or r_a90_flag or r_s90_flag: # button pressed, need delay to execute cmd
            sleep_flag = True
        get_tar_state()
        pub_msg = Float32MultiArray(data = state_tar)
        pub.publish(pub_msg)
        if sleep_flag == True:
            rospy.sleep(1)
            sleep_flag = False
        rate.sleep()
        
        