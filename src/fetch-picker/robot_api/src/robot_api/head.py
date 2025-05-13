#!/usr/bin/env python

# TODO: import ?????????
import actionlib #  1. 导入 actionlib（已写）
# TODO: import ???????_msgs.msg
import control_msgs.msg # 2. 导入 control_msgs 的 msg 模块# 提供 PointHeadAction、FollowJointTrajectoryAction 等
# TODO: import ??????????_msgs.msg
import trajectory_msgs.msg
import geometry_msgs.msg

import math
import rospy
# 让头看向目标点的 action 名称
LOOK_AT_ACTION_NAME = '/head_controller/point_head'  # TODO: Get the name of the look-at action
# 控制头部关节角度的 action 名称
PAN_TILT_ACTION_NAME = '/head_controller/follow_joint_trajectory'  # TODO: Get the name of the pan/tilt action
# 头部 pan 关节名字
PAN_JOINT = 'head_pan_joint'  # TODO: Get the name of the head pan joint
# 头部 tilt 关节名字
TILT_JOINT = 'head_tilt_joint'  # TODO: Get the name of the head tilt joint
# 移动头部需要的时间，单位秒
PAN_TILT_TIME = 2.5  # How many seconds it should take to move the head.

class Head(object):
    """Head controls the Fetch's head.

    It provides two interfaces:
        head.look_at(frame_id, x, y, z)
        head.pan_tilt(pan, tilt) # In radians

    For example:
        head = robot_api.Head()
        head.look_at('base_link', 1, 0, 0.3)
        head.pan_tilt(0, math.pi/4)
    """
    MIN_PAN = -1.57  # TODO: Minimum pan angle, in radians.最小 pan 角度（弧度）
    MAX_PAN = 1.57  # TODO: Maximum pan angle, in radians.最大 pan 角度（弧度）
    MIN_TILT = -0.76  # TODO: Minimum tilt angle, in radians.最小 tilt 角度（弧度）
    MAX_TILT = 1.45  # TODO: Maximum tilt angle, in radians.最大 tilt 角度（弧度）

    def __init__(self):
        #  创建 look_at 动作客户端（需要传入 Action 类型 PointHeadAction）
        self._lookat_ac = actionlib.SimpleActionClient(LOOK_AT_ACTION_NAME, control_msgs.msg.PointHeadAction)
        #  创建 pan_tilt 动作客户端（Action 类型 FollowJointTrajectoryAction）
        self._head_ac = actionlib.SimpleActionClient(PAN_TILT_ACTION_NAME, control_msgs.msg.FollowJointTrajectoryAction)
        # TODO: Wait for both servers
        #  等待两个 action server 启动
        rospy.loginfo("Waiting for head action servers...")
        self._lookat_ac.wait_for_server()
        self._head_ac.wait_for_server()
        rospy.loginfo("Head action servers connected.")
        

    def look_at(self, frame_id, x, y, z):
        """Moves the head to look at a point in space.

        Args:
            frame_id: The name of the frame in which x, y, and z are specified.
            x: The x value of the point to look at.
            y: The y value of the point to look at.
            z: The z value of the point to look at.
        """
        # TODO: Create goal
        #  创建 PointHeadGoal 对象
        goal = control_msgs.msg.PointHeadGoal()

        #  填充 goal 目标点
        # TODO: Fill out the goal (we recommend setting min_duration to 1 second)
        goal.target = geometry_msgs.msg.PointStamped()
        goal.target.header.stamp = rospy.Time.now()  # 当前时间戳
        goal.target.header.frame_id = frame_id  # 坐标系
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.pointing_frame = 'head_link'          # 指向 head_link 坐标系
        goal.pointing_axis.x = 1.0                 # 指向 +X 方向
        goal.min_duration = rospy.Duration(1.0)

        # 发送 goal
        self._lookat_ac.send_goal(goal)
        # 等待 action 完成，最多等待 5 秒
        self._lookat_ac.wait_for_result(rospy.Duration(5.0))

        # TODO: Send the goal
        self._lookat_ac.send_goal(goal)

        # TODO: Wait for result
        self._lookat_ac.wait_for_result()
        

    def pan_tilt(self, pan, tilt):
        """Moves the head by setting pan/tilt angles.

              Args:
            pan: The pan angle, in radians. A positive value is clockwise.
            tilt: The tilt angle, in radians. A positive value is downwards.
        """
        # TODO: Check that the pan/tilt angles are within joint limits
        #  检查角度是否在允许范围内
        if not (self.MIN_PAN <= pan <= self.MAX_PAN) or not (self.MIN_TILT <= tilt <= self.MAX_TILT):
            rospy.logerr("Pan or tilt angle out of range!")
            return

        # TODO: Create a trajectory point
        point = trajectory_msgs.msg.JointTrajectoryPoint()

        # TODO: Set positions of the two joints in the trajectory point
        #  设置运动持续时间 多久完成
        point.positions = [pan, tilt]

        # TODO: Set time of the trajectory point
        point.time_from_start = rospy.Duration(PAN_TILT_TIME)

        # TODO: Create goal 创建一个执行目标
        goal = control_msgs.msg.FollowJointTrajectoryGoal()

        # TODO: Add joint names to the list 这段轨迹是针对哪些关节的，是针对头和
        goal.trajectory.joint_names = [PAN_JOINT, TILT_JOINT]

        # TODO: Add trajectory point created above to trajectory 添加 trajectory point
        goal.trajectory.points.append(point)
        goal.trajectory.header.stamp = rospy.Time.now() #设置 header 时间戳
        # TODO: Send the goal  发送 goal
        self._head_ac.send_goal(goal)

        # TODO: Wait for result
        self._head_ac.wait_for_result()
      