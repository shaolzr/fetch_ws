#! /usr/bin/env python

import robot_api
import rospy
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
# 导入必要的库：
# robot_api：自定义的 robot api 包
# rospy：ROS 的 Python 库
# actionlib：ROS action client 库
# control_msgs：包含 GripperCommandAction/Goal 消息类型

def print_usage():
    print('Usage: rosrun applications gripper_demo.py open')
    print('       rosrun applications gripper_demo.py close 40')

#TODO
class Gripper:
    def __init__(self):
        """初始化 Gripper 类，创建 action client"""
        # 创建 action client，连接 Fetch 的 gripper action server
        self.client = actionlib.SimpleActionClient('/gripper_controller/gripper_action', GripperCommandAction)
        rospy.loginfo("Waiting for gripper_action server...") # 打印等待信息
        self.client.wait_for_server() # 阻塞等待 action server 可用
        rospy.loginfo("Gripper action server available.") # 打印连接成功信息
        self.MAX_EFFORT = 50.0 # 设置最大力度为 50N

    def open(self):
        """张开夹爪"""
        rospy.loginfo("Sending open command...") # 打印日志
        goal = GripperCommandGoal() # 创建目标消息
        # open：大约 0.10 米为开放状态
        goal.command.position = 0.10  # 设置张开位置（0.10米）
        goal.command.max_effort = 50.0   #设置力度为 50N
        self.client.send_goal(goal) # 发送目标
        self.client.wait_for_result()   # 阻塞，等待执行完毕
        result = self.client.get_result()   # 获取结果
        rospy.loginfo("Open result: %s", result)    # 打印执行结果
        return result

    def close(self, effort):
        """闭合夹爪，带指定力度"""
        rospy.loginfo("Sending close command with effort: %s", effort)  # 打印日志
        goal = GripperCommandGoal() # 创建目标消息
        # close：设置为 0.0 米为闭合状态
        goal.command.position = 0.0  # 设置闭合位置（0.0米）
        goal.command.max_effort = effort # 设置指定力度
        
        self.client.send_goal(goal) # 发送目标
        self.client.wait_for_result() # 阻塞等待
        result = self.client.get_result() # 获取结果
        rospy.loginfo("Close result: %s", result) # 打印结果
        return result

# 这里 **将 robot_api.Gripper 替换为上面定义的 Gripper 类**
# 相当于 monkey patch，把 demo 中 Gripper 实现绑定到 robot_api.Gripper
robot_api.Gripper = Gripper

def wait_for_time():
    """Wait for simulated time to begin.
    """
    """等待仿真时间开始（时间从 0 开始计时）"""
    while rospy.Time().now().to_sec() == 0:
        pass # 一直循环，直到 ROS 时间开始（避免用 real time 时时间=0）


def main():
    rospy.init_node('gripper_demo')  # 初始化 ROS 节点
    wait_for_time() # 确保仿真时间已启动
    
    argv = rospy.myargv() # 获取命令行参数
    if len(argv) < 2: # 如果没传足够参数
        print_usage() # 打印提示
        return # 退出程序
    
    command = argv[1] # 取出第一个参数作为命令（open 或 close）
    gripper = robot_api.Gripper() # 创建 Gripper 实例
    effort = gripper.MAX_EFFORT # 默认力度 = 最大力度

    # 如果命令是 close 且提供了力度参数
    if command == 'close' and len(argv) > 2:
        effort = float(argv[2]) # 解析输入的力度

    if command == 'open':
        gripper.open() # 调用 open 方法
    elif command == 'close':
        gripper.close(effort) # 调用 close 方法（带力度参数）
    else:
        print_usage() # 输入不合法时提示


if __name__ == '__main__':
    main()
