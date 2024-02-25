import rospy
from std_msgs.msg import Float32MultiArray

def callback(msg):
    rospy.loginfo("收到的数据:%s",msg.data)

if __name__ == '__main__':
    rospy.init_node('gripper_pub_test', anonymous=True)
    #3.实例化 发布者 对象
    pub = rospy.Publisher("/motor_tar", Float32MultiArray, queue_size=10)
    sub = rospy.Subscriber("/motors", Float32MultiArray, callback)
    #4.组织被发布的数据
    raw_msg = [0, 0, 50, 0, 0]
    msg = Float32MultiArray(data = raw_msg)
    # 设置循环频率
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()
        rospy.loginfo("发布的数据:%s",msg.data)
    
    