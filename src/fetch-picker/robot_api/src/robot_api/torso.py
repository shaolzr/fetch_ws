#!/usr/bin/env python

# TODO: import ?????????
# TODO: import ???????_msgs.msg
# TODO: import ??????????_msgs.msg
import rospy # 导入 ROS Python 库
import actionlib # 导入 actionlib 客户端库
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal # 导入 action 消息类型
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint # 导入轨迹消息类型


# TODO: ACTION_NAME = ???
# TODO: JOINT_NAME = ???
TIME_FROM_START = 5  # 设置躯干移动需要的时间（秒）
ACTION_NAME = '/torso_controller/follow_joint_trajectory' # action server 的名称
JOINT_NAME = 'torso_lift_joint' # 控制 torso 的关节名称


class Torso(object):
    """Torso controls the robot's torso height.
    """
    """Torso 控制机器人的躯干高度"""
    MIN_HEIGHT = 0.0
    MAX_HEIGHT = 0.4

    def __init__(self):
        """构造函数：创建 action client 并等待服务器"""
        # TODO: Create actionlib client
        # TODO: Wait for server
        # pass
         # 创建 action client，连接 torso controller 的 follow_joint_trajectory action
        self.client = actionlib.SimpleActionClient(ACTION_NAME, FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for torso action server...")
        # 阻塞直到 action server 准备好
        self.client.wait_for_server()
        rospy.loginfo("Torso action server is ready.")


    def set_height(self, height):
        """Sets the torso height.

        This will always take ~5 seconds to execute.

        Args:
            height: The height, in meters, to set the torso to. Values range
                from Torso.MIN_HEIGHT (0.0) to Torso.MAX_HEIGHT(0.4).
        """
        # TODO: Check that the height is between MIN_HEIGHT and MAX_HEIGHT.
        # TODO: Create a trajectory point
        # TODO: Set position of trajectory point
        # TODO: Set time of trajectory point

        # TODO: Create goal
        # TODO: Add joint name to list
        # TODO: Add the trajectory point created above to trajectory

        # TODO: Send goal
        # TODO: Wait for result
        # rospy.logerr('Not implemented.')
        
        # 检查高度是否在允许范围内
        if height < self.MIN_HEIGHT or height > self.MAX_HEIGHT:
            rospy.logerr("Height out of bounds: must be between %.2f and %.2f" % (self.MIN_HEIGHT, self.MAX_HEIGHT))
            return
        
        # 创建一个轨迹点
        point = JointTrajectoryPoint()
        point.positions = [height] # 设置轨迹点的位置为目标高度（列表，因为支持多个关节）
        point.time_from_start = rospy.Duration(TIME_FROM_START)# 设置运动时间，5 秒后完成
        
        # 创建 FollowJointTrajectory 的目标消息
        goal = FollowJointTrajectoryGoal() # 创建 FollowJointTrajectory 的目标消息
        goal.trajectory = JointTrajectory() # 初始化 trajectory 对象
        goal.trajectory.joint_names = [JOINT_NAME] # 指定需要控制的关节名
        goal.trajectory.points = [point] # 添加轨迹点到 trajectory

        # 设置 trajectory 的时间戳（当前时间）
        goal.trajectory.header.stamp = rospy.Time.now()

        # 发送 action 目标给 server
        self.client.send_goal(goal)
        self.client.wait_for_result()# 等待 action 执行结束
        rospy.loginfo("Torso motion complete.")