import rospy
from geometry_msgs.msg import PoseStamped     # import 换掉

plane_z = 0

def state_cb(msg):
    global plane_z
    plane_z = msg.pose.position.z
    print(plane_z)


rospy.init_node("learn_ROS_topic_src")  #node初始化没有返回值，不需要赋值
sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, state_cb)

def main():
    rospy.spin()  # 保持节点运行，等待回调函数处理消息
    #spin()函数会阻塞当前线程，直到节点关闭。它会让ROS进入循环，等待消息的到来，并调用相应的回调函数进行处理。在这个例子中，回调函数是state_cb，它会在接收到/mavros/local_position/pose话题的消息时被调用。
    #对应的rate = rospy.Rate(20)   # 20Hz > 2Hz 要求
    #rate是用来控制循环频率的对象，rate.sleep()会让循环以指定的频率运行。在这个例子中，rate被设置为20Hz，这意味着循环会以每秒20次的频率运行。


if __name__ == '__main__':
    main()