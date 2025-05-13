#! /usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

def print_usage():
    print ('Moves the torso to a certain height between [0.0, 0.4]')
    print ('Usage: rosrun applications torso_demo.py 0.4')

#TODO
class Torso:
    def __init__(self):
        """初始化 Torso 类：创建 action client 连接到 torso 控制器"""
        # 创建一个 action client，连接到控制躯干的 action server
        self.client = actionlib.SimpleActionClient('/torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for torso trajectory action server...")
        self.client.wait_for_server() # 阻塞等待，直到 server 可用
        rospy.loginfo("Torso action server available.")

    def set_height(self, height):
        """设置躯干目标高度"""
        # 创建一个 FollowJointTrajectoryGoal 目标消息
        goal = FollowJointTrajectoryGoal()
        # 设置要控制的关节名（Fetch 的 torso 关节叫 torso_lift_joint）
        goal.trajectory.joint_names = ['torso_lift_joint']
        
        # 创建一个轨迹点（JointTrajectoryPoint）
        point = JointTrajectoryPoint()
        point.positions = [height] # 设置目标高度（位置）
        point.time_from_start = rospy.Duration(2.0) # 规定移动需要 2 秒完成
        # 将轨迹点加入轨迹
        goal.trajectory.points.append(point)
        
        # 设置轨迹的开始时间：从当前时间延后 0.1 秒开始
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
        rospy.loginfo("Sending torso trajectory goal to move to height %f", height)
        # 发送 action 目标
        self.client.send_goal(goal)
        # 等待执行完成
        self.client.wait_for_result()
         # 获取执行结果
        result = self.client.get_result()
        rospy.loginfo("Torso movement result: %s", result)


def wait_for_time():
    """Wait for simulated time to begin.
    """
    """等待仿真时间启动（时间从0开始计时）"""
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('torso_demo') # 初始化 ROS 节点，节点名 torso_demo
    wait_for_time()
    argv = rospy.myargv() # 获取命令行参数（过滤掉 rosrun 的前缀）
    if len(argv) < 2: # 如果参数数量不足
        print_usage()
        return
    height = float(argv[1]) # 获取用户输入的目标高度（转为 float）

    #TODO
    # 实例化 Torso 对象，并调用 set_height 方法
    torso = Torso()
    torso.set_height(height)


if __name__ == '__main__':
    main()
