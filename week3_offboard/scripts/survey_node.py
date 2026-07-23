#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Survey 巡视节点：起飞2m -> 悬停5s -> 绕三个地标巡视 -> 返回起点 -> 降落
异常处理：未连接不arm / 模式切换重试 / 位置误差过大悬停 / Ctrl+C安全退出
"""

# ---------- 导入 ----------
import math                                 # 标准数学库，用于三维距离计算 (math.sqrt)
import rospy                                # ROS Python 客户端库（节点、话题、服务、日志、时间）
from geometry_msgs.msg import PoseStamped   # 带时间戳的位姿消息：订阅当前位置 + 发布目标设定点
from mavros_msgs.msg import State           # 飞控状态消息：connected / armed / mode
from mavros_msgs.srv import CommandBool, SetMode  # CommandBool：解锁/上锁，SetMode：切换飞行模式


# ==============================================================================
# 本项目用到的 msg（消息）和 srv（服务）结构一览
# ==============================================================================
#
#  PoseStamped (geometry_msgs/msg)          State (mavros_msgs/msg)
#  ─────────────────────────────            ───────────────────────────
#  header                                   bool connected     ← FCU是否连接
#    stamp       ← 时间戳，rospy.Time.now()    bool armed        ← 是否解锁
#    frame_id      (坐标系，如 "map")           string mode       ← 当前飞行模式
#  pose                                           ↑
#    position.x, .y, .z  ← 三维坐标             三个核心字段，主循环中实时轮询
#    orientation.x,.y,.z,.w ← 四元数姿态
#
#  ┌ 发布方向 ───────────────────────────────────────────────────────┐
#  │ pose_pub -> /mavros/setpoint_position/local (PoseStamped)       │
#  │   ⇒ 告诉飞控"目标位置是 (x,y,z)"，OFFBOARD 模式下持续发出       │
#  └─────────────────────────────────────────────────────────────────┘
#
#  ┌ 订阅方向 ───────────────────────────────────────────────────────┐
#  │ /mavros/state             -> state_cb  -> current_state (State)   │
#  │ /mavros/local_position/pose -> pose_cb  -> current_pose  (PoseStamped) │
#  └─────────────────────────────────────────────────────────────────┘
#
#  srv（服务）：请求-应答，同步阻塞调用
#  ─────────────────────────────────────
#  CommandBool (mavros_msgs/srv)           SetMode (mavros_msgs/srv)
#  request:  bool value  (True=解锁)        request:  uint8 base_mode
#  response: bool success                             string custom_mode ← "OFFBOARD"/"AUTO.LAND"
#           uint8 result                      response: bool mode_sent
#
#  ┌ 服务调用方向 ──────────────────────────────────────────────────┐
#  │ arming_client(True)           -> /mavros/cmd/arming  (CommandBool) │
#  │ set_mode_client("OFFBOARD")   -> /mavros/set_mode    (SetMode)     │
#  └─────────────────────────────────────────────────────────────────┘


#时间戳的加入是因为PX4判断信息时会看时间戳，如果时间戳不对，飞控会认为信息过期而拒绝接收。















# ---------- 全局状态 ----------
# current用于存储当前飞控状态和位姿信息，供回调函数更新和主循环使用
#这里是创建了两个对象，前面已经导入了这两个消息类型
current_state = State()
current_pose = PoseStamped()

#这个回调函数就是保存获取到的飞控状态信息，保存到全局变量current_state中
def state_cb(msg):
    global current_state
    current_state = msg
#这个回调函数就是保存获取到的飞控位姿信息，保存到全局变量current_pose中
def pose_cb(msg):
    global current_pose
    current_pose = msg
#计算当前位置到目标点的三维距离
def dist_to(target):
    """当前位置到目标点的三维距离"""
    p = current_pose.pose.position
    return math.sqrt((p.x-target[0])**2 + (p.y-target[1])**2 + (p.z-target[2])**2)

def main():
    #初始化节点，取名叫 'offboard_node'，并设置循环频率为20Hz
    rospy.init_node('survey_node')  # 节点名改成 survey_node，避免和 offboard_node 重名冲突
    #这里为什么要循环，是因为在offboard模式下，飞控要求持续接收设定点，否则会自动退出offboard模式。20Hz的频率可以保证设定点的连续性，同时也满足了飞控对频率的要求（大于2Hz）。
    #rate就是一个实例化后的对象
    rate = rospy.Rate(20)   # 20Hz > 2Hz 要求

    # ---------- 订阅/发布/服务 ----------
    #订阅飞控状态话题，获取当前飞控的连接状态、解锁状态和模式信息
    #rospy.Subscriber(topic, message_type, callback_function)
    rospy.Subscriber('/mavros/state', State, state_cb)
    #订阅飞控当前位置话题，获取当前无人机的位姿信息
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_cb)
    #发布目标设定点话题，发送无人机的目标位姿信息
    pose_pub = rospy.Publisher('/mavros/setpoint_position/local',
                               PoseStamped, queue_size=10)
    #创建服务客户端，用于发送解锁/上锁命令
    arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    #创建服务客户端，用于发送切换飞行模式命令
    set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)


    # ---------- 异常处理1：等待FCU连接，未连接不arm ----------
    rospy.loginfo("等待 FCU（飞控） 连接...")
    #设置一个超时时间，等待飞控连接，如果超过30秒还未连接，则打印错误信息并退出程序
    #rospy.Time.now()获取当前时间，rospy.Duration(30)表示30秒的时间间隔，将两者相加得到超时时间
    timeout = rospy.Time.now() + rospy.Duration(30)
    # 节点未关闭，但是飞控未连接
    while not rospy.is_shutdown() and not current_state.connected:
        #如果当前时间超过了超时时间，则打印错误信息并退出程序
        if rospy.Time.now() > timeout:
            rospy.logerr("连接超时，退出（未连接绝不arm）")
            return
        #打印等待连接的日志信息，并以20Hz的频率循环等待
        rate.sleep()
    rospy.loginfo("FCU 已连接")

    # ---------- 先连续发送 setpoint ----------
    #初始化target对象
    target = PoseStamped()
    target.pose.position.x = 0
    target.pose.position.y = 0
    target.pose.position.z = 2     # 起飞目标高度 2m

    rospy.loginfo("预发 100 次 setpoint...")
    for _ in range(100):
        #目标位姿消息的时间戳设置为当前时间，确保消息的有效性
        target.header.stamp = rospy.Time.now()
        #调用发布器，将目标位姿消息发送到飞控，确保飞控接收到连续的设定点
        pose_pub.publish(target)
        #设定循环的频率，保证飞控的心跳包要求
        rate.sleep()

    # ---------- 异常处理2：切 OFFBOARD，失败重试 ----------
    offboard_ok = False
    for attempt in range(5):
        #客户端调用服务，发送切换飞行模式为OFFBOARD的请求
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
    ((0, 0, 2), "悬停5s", 5.0),
    ((3.5, 0, 2), "巡视点1-红柱外侧", 2.0),
    ((0, 3.5, 2), "巡视点2-绿箱外侧", 2.0),
    ((-3.5, 0, 2), "巡视点3-蓝塔外侧", 2.0),
    ((0, 0, 2), "返回起点"),
    ]  
    # 这里的 waypoints 是一个列表，包含了每个航点的坐标、名称和悬停时间（可选）。每个航点是一个元组，元组的第一个元素是一个三元组，表示 (x, y, z) 坐标；第二个元素是航点的名称；第三个元素是悬停时间（如果有的话）。
    for wp in waypoints:
        xyz, name = wp[0], wp[1]
        #悬停时间，如果元组长度大于2，则取第三个元素作为悬停时间，否则默认为0.0
        hover_time = wp[2] if len(wp) > 2 else 0.0

        target.pose.position.x, target.pose.position.y, target.pose.position.z = xyz
        rospy.loginfo(f">>> 前往 {name} {xyz}")

        arrive_time = None
        while not rospy.is_shutdown():
            target.header.stamp = rospy.Time.now()
            #发布目标位置
            pose_pub.publish(target)

            err = dist_to(xyz)
            # 异常处理3：位置误差过大（>2m 且持续），报警并保持当前点
            if err > 2.0:
                rospy.logwarn_throttle(2, f"位置误差过大 {err:.2f}m，保持当前设定点等待收敛")
            elif err < 0.3:
                if arrive_time is None:
                    arrive_time = rospy.Time.now()
                    rospy.loginfo(f"到达 {name}，实际误差 {err:.3f} m (< 0.5m 达标)")
                    # 0.3m 判定阈值比比赛规则的 0.5m 更严格，天然满足要求
                # 到达后需要悬停的话，掐表 ，这里就是卡着不让循环退出，直到悬停时间到达
                if (rospy.Time.now() - arrive_time).to_sec() >= hover_time:
                    break
            rate.sleep()

    # ---------- 降落 ----------
    rospy.loginfo("航线完成，切换 AUTO.LAND 降落")
    set_mode_client(custom_mode="AUTO.LAND")
    while not rospy.is_shutdown() and current_state.armed:
        # 降落期间维持话题活性
        pose_pub.publish(target)
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