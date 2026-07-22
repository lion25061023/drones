#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Offboard 控制节点：起飞2m -> 悬停5s -> 正方形4点 -> 返回起点 -> 降落
异常处理：未连接不arm / 模式切换重试 / 位置误差过大悬停 / Ctrl+C安全退出
"""

# ---------- 导入 ----------
import math                                 # 标准数学库，用于三维距离计算 (math.sqrt)
import rospy                                # ROS Python 客户端库（节点、话题、服务、日志、时间）
from geometry_msgs.msg import PoseStamped   # 带时间戳的位姿消息：订阅当前位置 + 发布目标设定点
from mavros_msgs.msg import State           # 飞控状态消息：connected / armed / mode
from mavros_msgs.srv import CommandBool, SetMode  # CommandBool：解锁/上锁，SetMode：切换飞行模式

# ---------- 全局状态 ----------
# current用于存储当前飞控状态和位姿信息，供回调函数更新和主循环使用
#这里是创建了两个对象，前面已经导入了这两个消息类型
current_state = State()
current_pose = PoseStamped()

def state_cb(msg):
    global current_state
    current_state = msg

def pose_cb(msg):
    global current_pose
    current_pose = msg

def dist_to(target):
    """当前位置到目标点的三维距离"""
    p = current_pose.pose.position
    return math.sqrt((p.x-target[0])**2 + (p.y-target[1])**2 + (p.z-target[2])**2)

def main():
    #初始化节点，取名叫 'offboard_node'，并设置循环频率为20Hz
    rospy.init_node('offboard_node')
    #这里为什么要循环，是因为在offboard模式下，飞控要求持续接收设定点，否则会自动退出offboard模式。20Hz的频率可以保证设定点的连续性，同时也满足了飞控对频率的要求（大于2Hz）。
    rate = rospy.Rate(20)   # 20Hz > 2Hz 要求

    # ---------- 订阅/发布/服务 ----------
    #订阅飞控状态话题，获取当前飞控的连接状态、解锁状态和模式信息
    #变量
    rospy.Subscriber('/mavros/state', State, state_cb)
    #订阅飞控当前位置话题，获取当前无人机的位姿信息
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_cb)
    #发布目标设定点话题，发送无人机的目标位姿信息
    pose_pub = rospy.Publisher('/mavros/setpoint_position/local',
                               PoseStamped, queue_size=10)
    #创建服务客户端，用于发送解锁/上锁命令和切换飞行模式命令
    arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    #创建服务客户端，用于发送切换飞行模式命令
    set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

    # ---------- 异常处理1：等待FCU连接，未连接不arm ----------
    rospy.loginfo("等待 FCU 连接...")
    timeout = rospy.Time.now() + rospy.Duration(30)
    while not rospy.is_shutdown() and not current_state.connected:
        if rospy.Time.now() > timeout:
            rospy.logerr("连接超时，退出（未连接绝不arm）")
            return
        rate.sleep()
    rospy.loginfo("FCU 已连接")

    # ---------- 先连续发送 setpoint ----------
    target = PoseStamped()
    target.pose.position.x = 0
    target.pose.position.y = 0
    target.pose.position.z = 2     # 起飞目标高度 2m

    rospy.loginfo("预发 100 次 setpoint...")
    for _ in range(100):
        target.header.stamp = rospy.Time.now()
        pose_pub.publish(target)
        rate.sleep()

    # ---------- 异常处理2：切 OFFBOARD，失败重试 ----------
    offboard_ok = False
    for attempt in range(5):
        resp = set_mode_client(custom_mode="OFFBOARD")
        if resp.mode_sent:
            offboard_ok = True
            break
        rospy.logwarn(f"切 OFFBOARD 失败，第 {attempt+1}/5 次重试...")
        for _ in range(20):            # 重试期间保持发 setpoint
            pose_pub.publish(target)
            rate.sleep()
    if not offboard_ok:
        rospy.logerr("多次切换 OFFBOARD 失败，安全退出")
        return
    rospy.loginfo("已切入 OFFBOARD")

      # ---------- 切 OFFBOARD + arm（边发 setpoint 边切，失败自动重试） ----------
    last_request = rospy.Time.now()
    start_wait = rospy.Time.now()
    rospy.loginfo("等待切入 OFFBOARD 并解锁...")
    while not rospy.is_shutdown():
        # 关键：任何时刻都保持 setpoint 流不中断
        target.header.stamp = rospy.Time.now()
        pose_pub.publish(target)

        if current_state.mode != "OFFBOARD" and \
           (rospy.Time.now() - last_request) > rospy.Duration(5.0):
            if set_mode_client(custom_mode="OFFBOARD").mode_sent:
                rospy.loginfo("OFFBOARD 切换指令已发送")
            else:
                rospy.logwarn("切 OFFBOARD 被拒，5s 后重试")   # 异常处理：模式切换重试
            last_request = rospy.Time.now()

        elif not current_state.armed and \
             (rospy.Time.now() - last_request) > rospy.Duration(5.0):
            if arming_client(True).success:
                rospy.loginfo("解锁指令已接受")
            else:
                rospy.logwarn("arm 被拒，5s 后重试")
            last_request = rospy.Time.now()

        # 两个条件都满足，进入航线
        if current_state.mode == "OFFBOARD" and current_state.armed:
            rospy.loginfo("OFFBOARD + 解锁完成，开始任务")
            break

        # 异常处理：60s 还没成功就放弃，绝不瞎等
        if (rospy.Time.now() - start_wait) > rospy.Duration(60.0):
            rospy.logerr("60s 内未能进入 OFFBOARD+解锁，安全退出")
            return
        rate.sleep()

    # ---------- 航线：起飞 -> 悬停 -> 正方形 -> 返回 ----------
    waypoints = [
        ((0, 0, 2), "起飞到2m"),
        ((0, 0, 2), "悬停5s", 5.0),      # 同一点停留5秒
        ((2, 0, 2), "正方形 点1"),
        ((2, 2, 2), "正方形 点2"),
        ((0, 2, 2), "正方形 点3"),
        ((0, 0, 2), "返回起点"),
    ]

    for wp in waypoints:
        xyz, name = wp[0], wp[1]
        hover_time = wp[2] if len(wp) > 2 else 0.0

        target.pose.position.x, target.pose.position.y, target.pose.position.z = xyz
        rospy.loginfo(f">>> 前往 {name} {xyz}")

        arrive_time = None
        while not rospy.is_shutdown():
            target.header.stamp = rospy.Time.now()
            pose_pub.publish(target)

            err = dist_to(xyz)
            # 异常处理3：位置误差过大（>2m 且持续），报警并保持当前点
            if err > 2.0:
                rospy.logwarn_throttle(2, f"位置误差过大 {err:.2f}m，保持当前设定点等待收敛")
            elif err < 0.3:
                if arrive_time is None:
                    arrive_time = rospy.Time.now()
                    rospy.loginfo(f"到达 {name}")
                # 到达后需要悬停的话，掐表
                if (rospy.Time.now() - arrive_time).to_sec() >= hover_time:
                    break
            rate.sleep()

    # ---------- 降落 ----------
    rospy.loginfo("航线完成，切换 AUTO.LAND 降落")
    set_mode_client(custom_mode="AUTO.LAND")
    while not rospy.is_shutdown() and current_state.armed:
        pose_pub.publish(target)     # 降落期间维持话题活性
        rate.sleep()
    rospy.loginfo("已着陆，电机锁定，任务结束")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        # 异常处理4：Ctrl+C 时尽量安全降落
        rospy.logwarn("收到 Ctrl+C，尝试切 AUTO.LAND 安全退出")
        try:
            set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            set_mode_client(custom_mode="AUTO.LAND")
        except Exception:
            pass