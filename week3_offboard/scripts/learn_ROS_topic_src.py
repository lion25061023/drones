import rospy
from mavros_msgs.msg import State

plane_z = 0

def state_cb(msg):
    global plane_z
    plane_z = msg.pose.position.z


my_node = rospy.init_node("learn_ROS_topic_src")
sub = rospy.Subscriber('mavros/state', State, state_cb)

def main():
    rospy.spin()  # 保持节点运行，等待回调函数处理消息