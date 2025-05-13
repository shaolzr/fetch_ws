#!/usr/bin/env python3
"""robot_api/gripper.py – 控制 Fetch 末端夹爪"""

import rospy
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal

# Fetch 仿真 / 真机默认 action 名
ACTION_NAME = '/gripper_controller/gripper_action'

CLOSED_POS  = 0.00   # 夹爪完全闭合位置（单位：米）
OPENED_POS  = 0.10   # 夹爪完全张开位置（单位：米）；0.086 也常用为极限值

class Gripper(object):
    """简单封装：open() / close() 同步等待执行完毕"""

    MIN_EFFORT = 35   # 夹爪最小夹持力（牛顿）
    MAX_EFFORT = 100  # 夹爪最大夹持力（牛顿）

    def __init__(self):
        # 创建 action client，用于连接 Fetch 的夹爪 action server
        self._ac = actionlib.SimpleActionClient(ACTION_NAME,
                                                GripperCommandAction)
        rospy.loginfo("Waiting for %s ..." % ACTION_NAME)
        self._ac.wait_for_server() # 等待 action server 启动并可用
        rospy.loginfo("Gripper action server connected.")

    # ---------------- 公共接口 ----------------
    def open(self, effort=MIN_EFFORT, wait=True):
        """张开夹爪
        
        effort: 使用的夹持力（默认为最小）
        wait: 是否等待动作执行完成
        """
        self._send_cmd(OPENED_POS, effort, wait)# 调用内部发送命令方法

    def close(self, effort=MAX_EFFORT, wait=True):
        """闭合夹爪
        
        effort: 使用的夹持力（默认为最大）
        wait: 是否等待动作执行完成
        """
        # 力度别低于 MIN_EFFORT，否则夹不紧
        effort = max(effort, Gripper.MIN_EFFORT)# 力度不能小于 MIN_EFFORT
        self._send_cmd(CLOSED_POS, effort, wait)# 调用内部发送命令方法

    # ---------------- 内部辅助 ----------------
    def _send_cmd(self, position, effort, wait):
        """内部方法：发送 GripperCommandGoal 命令"""
        goal = GripperCommandGoal()# 创建目标消息对象
        goal.command.position    = position   # 设置夹爪位置（单位：米）
        goal.command.max_effort  = float(effort)  # 设置夹爪夹持力（单位：牛顿）
        self._ac.send_goal(goal) # 向 action server 发送目标指令
        if wait:
            self._ac.wait_for_result() # 阻塞，等待动作完成
            result = self._ac.get_result() # 获取执行结果
            rospy.loginfo("Gripper result: %s", result) # 打印结果日志
