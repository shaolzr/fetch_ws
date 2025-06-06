#!/usr/bin/env python

# 导入PoseStamped消息类型，用于表示位置和姿态
from geometry_msgs.msg import PoseStamped
# 导入robot_api模块，用于控制机器人
import robot_api
# 导入rospy，用于ROS节点操作
import rospy

def wait_for_time():
    """等待ROS时间初始化完成"""
    while rospy.Time().now().to_sec() == 0:
        pass

def print_usage():
    """打印使用说明"""
    print('Usage: rosrun applications check_cart_pose.py plan X Y Z')
    print('       rosrun applications check_cart_pose.py ik X Y Z')

def main():
    # 初始化ROS节点，节点名为'check_cart_pose'
    rospy.init_node('check_cart_pose')
    # 等待ROS时间初始化
    wait_for_time()
    # 获取命令行参数
    argv = rospy.myargv()
    # 检查参数数量是否正确
    if len(argv) < 5:
        print_usage()
        return

    # 解析命令行参数：命令类型和坐标值
    command, x, y, z = argv[1], float(argv[2]), float(argv[3]), float(argv[4])
    # 创建Arm对象用于控制机器人手臂
    arm = robot_api.Arm()

    # 创建PoseStamped消息对象
    ps = PoseStamped()
    # 设置参考坐标系为base_link（机器人基座）
    ps.header.frame_id = 'base_link'
    # 设置目标位置坐标
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.position.z = z
    # 设置目标姿态为单位四元数（表示正前方）
    ps.pose.orientation.w = 1.0  # 单位方向（正前方）

    # 根据命令类型执行不同的操作
    if command == 'plan':
        # 检查是否能够规划到目标位置
        error = arm.check_pose(ps, allowed_planning_time=1.0)
        if error is None:
            rospy.loginfo('Found plan!')  # 找到可行路径
        else:
            rospy.loginfo('No plan found.')  # 未找到可行路径
        # 取消所有正在执行的目标
        arm.cancel_all_goals()
    elif command == 'ik':
        # 检查是否能够计算逆运动学解
        if arm.compute_ik(ps):
            rospy.loginfo('Found IK!')  # 找到逆运动学解
        else:
            rospy.loginfo('No IK found.')  # 未找到逆运动学解
    else:
        # 如果命令类型不正确，打印使用说明
        print_usage()

# 当脚本直接运行时执行main函数
if __name__ == '__main__':
    main()
