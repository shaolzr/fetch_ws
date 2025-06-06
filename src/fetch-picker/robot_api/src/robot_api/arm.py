# TODO: import ?????????
import actionlib #导入 actionlib 库用于动作客户端
# TODO: import ???????_msgs.msg
import control_msgs.msg #导入控制消息 用于控制机械臂的 FollowJointTrajectory
# TODO: import ??????????_msgs.msg
import trajectory_msgs.msg #导入轨迹消息 用于发送轨迹点

import rospy

from .arm_joints import ArmJoints # 从当前包导入 ArmJoints 类
#Lab 19
from .moveit_goal_builder import MoveItGoalBuilder # 从当前包导入 MoveItGoalBuilder 类
from moveit_msgs.msg import MoveGroupAction, MoveItErrorCodes # 导入 MoveIt 消息类型
#Lab 20
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest # 导入逆运动学服务



class Arm(object): # 定义 Arm 类控制机器人手臂
    """Arm controls the robot's arm.

    Joint space control:
        joints = ArmJoints()
        # Fill out joint states
        arm = robot_api.Arm()
        arm.move_to_joints(joints)
    """

    def __init__(self): # 定义初始化函数
        # TODO: Create actionlib client
        # TODO: Wait for server
        # 创建动作客户端用于 arm_controller
        self._arm_ac = actionlib.SimpleActionClient(
            '/arm_controller/follow_joint_trajectory',
            control_msgs.msg.FollowJointTrajectoryAction)
        # 输出日志等待服务器
        rospy.loginfo("Waiting for arm_controller/follow_joint_trajectory action server...")
        # 等待服务器启动
        self._arm_ac.wait_for_server()
        # 输出日志服务器已连接
        rospy.loginfo("Arm action server connected.")
        # 创建 move_group 动作客户端
        self._move_group_client = actionlib.SimpleActionClient('move_group', MoveGroupAction)
        # 等待 move_group 服务器
        self._move_group_client.wait_for_server()
        # 创建 compute_ik 服务代理
        self._compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)



    def move_to_joints(self, arm_joints):
        """Moves the robot's arm to the given joints.

        Args:
            arm_joints: An ArmJoints object that specifies the joint values for
                the arm.
        """
        # TODO: Create a trajectory point
        point = trajectory_msgs.msg.JointTrajectoryPoint()# 创建轨迹点
        # TODO: Set position of trajectory point
        point.positions = arm_joints.values()# 设置轨迹点位置为 arm_joints 的值 获取各个角度的信息
        # TODO: Set time of trajectory point
        point.time_from_start = rospy.Duration(5.0)# 设置轨迹点执行时间为五秒

        # TODO: Create goal
        traj = trajectory_msgs.msg.JointTrajectory()# 创建轨迹消息 把角度信息设置进去

        # TODO: Add joint name to list
        traj.joint_names = ArmJoints.names()# 设置关节名称列表 告诉控制器"要控制哪些关节"
        # TODO: Add the trajectory point created above to trajectory
        traj.points.append(point)# 添加轨迹点到轨迹消息

        # TODO: Send goal
        goal = control_msgs.msg.FollowJointTrajectoryGoal(trajectory=traj)# 创建轨迹控制目标
        self._arm_ac.send_goal(goal)# 发送目标到动作服务器
        # TODO: Wait for result
        self._arm_ac.wait_for_result()  # 等待执行结果
        result = self._arm_ac.get_result()  # 获取执行结果
        rospy.loginfo("Arm movement result: %s", result)  # 输出执行结果日志
    
    # Lab19 Move to Pose
    # def move_to_pose(self, pose_stamped):
    #     """Moves the end-effector to a pose, using motion planning.

    #     Args:
    #        pose: geometry_msgs/PoseStamped. The goal pose for the gripper.

    #     Returns:
    #        string describing the error if an error occurred, else None.
    #     """
    #     # 创建一个MoveIt目标构建器，用于设置机器人的运动目标
    #     goal_builder = MoveItGoalBuilder()
    #     # 设置目标位姿，pose_stamped包含了目标位置和方向信息
    #     goal_builder.set_pose_goal(pose_stamped)
    #     # 构建最终的MoveIt运动目标
    #     goal = goal_builder.build()

    #     # 发送运动目标到MoveIt运动规划器
    #     self._move_group_client.send_goal(goal)
    #     # 等待运动执行完成，设置10秒超时时间
    #     finished = self._move_group_client.wait_for_result(rospy.Duration(10))  # Timeout version

    #     # 如果超时未完成，返回超时错误信息
    #     if not finished:
    #         return 'Timeout waiting for result'
        
    #     # 获取运动执行的结果
    #     result = self._move_group_client.get_result()
    #     # 检查运动是否成功完成
    #     if result.error_code.val != MoveItErrorCodes.SUCCESS:
    #         # 如果失败，返回具体的错误信息
    #         return Arm.moveit_error_string(result.error_code.val)
        
    #     # 如果一切正常，返回None表示成功
    #     return None

    #Lab 20的move to pose
    def move_to_pose(self,
                 pose_stamped,                    # 目标位姿，包含位置和方向信息
                 allowed_planning_time=10.0,      # 允许的规划时间，默认10秒
                 execution_timeout=15.0,          # 执行超时时间，默认15秒
                 group_name='arm',                # 要控制的机器人组名称，默认是'arm'
                 num_planning_attempts=1,         # 规划尝试次数，默认1次
                 plan_only=False,                 # 是否只规划不执行，默认False
                 replan=False,                    # 是否允许重新规划，默认False
                 replan_attempts=5,               # 重新规划的最大尝试次数，默认5次
                 tolerance=0.01,                  # 位置误差容忍度，默认0.01米
                 orientation_constraint=None):    # 方向约束，默认无约束
        """Moves the end-effector to a pose, using motion planning."""
        # 创建MoveIt目标构建器对象，用于设置运动目标
        goal_builder = MoveItGoalBuilder() # 设置目标位姿，包含位置和方向信息
        goal_builder.set_pose_goal(pose_stamped) # 设置规划时间限制，给规划器更多时间寻找解决方案
        goal_builder.allowed_planning_time = allowed_planning_time # 设置规划尝试次数，可以多次尝试规划
        goal_builder.num_planning_attempts = num_planning_attempts # 设置是否只规划不执行，用于检查路径是否可行
        goal_builder.plan_only = plan_only # 设置是否允许重新规划，提高规划成功率
        goal_builder.replan = replan # 设置重新规划的最大尝试次数
        goal_builder.replan_attempts = replan_attempts # 设置位置误差容忍度，允许一定的位置误差
        goal_builder.tolerance = tolerance # 设置要控制的机器人组名称，指定要控制的机器人部分
        goal_builder.group_name = group_name

        #Lab 22添加  修改方法内部，支持设置 orientation_constraint
        if orientation_constraint is not None:
            goal_builder.add_path_orientation_constraint(orientation_constraint)


        # 构建运动目标
        goal = goal_builder.build()

        # 发送运动目标到MoveIt
        self._move_group_client.send_goal(goal)
        # 等待执行结果，设置超时时间
        finished = self._move_group_client.wait_for_result(
                rospy.Duration(execution_timeout))

        # 如果超时未完成
        if not finished:
                rospy.logwarn("MoveIt timeout")
                return False                              # 返回规划失败

        # 获取执行结果
        result = self._move_group_client.get_result()
        # 检查执行结果是否成功
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return True                               # 返回规划成功
        else:
                # 输出错误信息
                rospy.logwarn("MoveIt failed: %s",
                            Arm.moveit_error_string(result.error_code.val))
                return False                              # 返回规划失败
    
    #Lab 20 逆运动
    def compute_ik(self, pose_stamped, timeout=rospy.Duration(5)):
        """计算给定位姿的逆运动学解"""
        # 创建逆运动学请求
        request = GetPositionIKRequest()
        # 设置目标位姿
        request.ik_request.pose_stamped = pose_stamped
        # 设置运动组名称
        request.ik_request.group_name = 'arm'
        # 设置超时时间
        request.ik_request.timeout = timeout
        # 发送请求并获取响应
        response = self._compute_ik(request)
        # 获取错误信息
        error_str = Arm.moveit_error_string(response.error_code.val)
        # 检查是否成功
        success = error_str == 'SUCCESS'
        if not success:
            return False
        # 获取关节状态
        joint_state = response.solution.joint_state
        # 输出每个关节的位置
        for name, position in zip(joint_state.name, joint_state.position):
            if name in ArmJoints.names():
                rospy.loginfo('{}: {}'.format(name, position))
        return True


    def check_pose(self, 
               pose_stamped,
               allowed_planning_time=10.0,
               group_name='arm',
               tolerance=0.01):
        # 调用move_to_pose进行规划检查
        return self.move_to_pose(
            pose_stamped,
            allowed_planning_time=allowed_planning_time,
            group_name=group_name,
            tolerance=tolerance,
            plan_only=True)


    
    #Lab 19 添加取消方法 cancel_all_goals
    def cancel_all_goals(self):
        self._arm_ac.cancel_all_goals() # 取消动作客户端的运动
        self._move_group_client.cancel_all_goals() # 取消MoveIt客户端的运动
    
    #Lab 19 添加工具函数 moveit_error_string
    @staticmethod
    def moveit_error_string(val):
        """返回与MoveItErrorCode关联的字符串
        
        参数:
            val: moveit_msgs/MoveItErrorCodes.msg中的val字段
            
        返回: 与错误值关联的字符串，如果值无效则返回'UNKNOWN_ERROR_CODE'
        """ 
        # 根据不同的错误码返回对应的错误信息
        if val == MoveItErrorCodes.SUCCESS:
            return 'SUCCESS'
        elif val == MoveItErrorCodes.FAILURE:
            return 'FAILURE'
        elif val == MoveItErrorCodes.PLANNING_FAILED:
            return 'PLANNING_FAILED'
        elif val == MoveItErrorCodes.INVALID_MOTION_PLAN:
            return 'INVALID_MOTION_PLAN'
        elif val == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
            return 'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE'
        elif val == MoveItErrorCodes.CONTROL_FAILED:
            return 'CONTROL_FAILED'
        elif val == MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA:
            return 'UNABLE_TO_AQUIRE_SENSOR_DATA'
        elif val == MoveItErrorCodes.TIMED_OUT:
            return 'TIMED_OUT'
        elif val == MoveItErrorCodes.PREEMPTED:
            return 'PREEMPTED'
        elif val == MoveItErrorCodes.START_STATE_IN_COLLISION:
            return 'START_STATE_IN_COLLISION'
        elif val == MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS:
            return 'START_STATE_VIOLATES_PATH_CONSTRAINTS'
        elif val == MoveItErrorCodes.GOAL_IN_COLLISION:
            return 'GOAL_IN_COLLISION'
        elif val == MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS:
            return 'GOAL_VIOLATES_PATH_CONSTRAINTS'
        elif val == MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED:
            return 'GOAL_CONSTRAINTS_VIOLATED'
        elif val == MoveItErrorCodes.INVALID_GROUP_NAME:
            return 'INVALID_GROUP_NAME'
        elif val == MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS:
            return 'INVALID_GOAL_CONSTRAINTS'
        elif val == MoveItErrorCodes.INVALID_ROBOT_STATE:
            return 'INVALID_ROBOT_STATE'
        elif val == MoveItErrorCodes.INVALID_LINK_NAME:
            return 'INVALID_LINK_NAME'                                      
        elif val == MoveItErrorCodes.INVALID_OBJECT_NAME:
            return 'INVALID_OBJECT_NAME'
        elif val == MoveItErrorCodes.FRAME_TRANSFORM_FAILURE:
            return 'FRAME_TRANSFORM_FAILURE'
        elif val == MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE:
            return 'COLLISION_CHECKING_UNAVAILABLE'
        elif val == MoveItErrorCodes.ROBOT_STATE_STALE:
            return 'ROBOT_STATE_STALE'
        elif val == MoveItErrorCodes.SENSOR_INFO_STALE:
            return 'SENSOR_INFO_STALE'
        elif val == MoveItErrorCodes.NO_IK_SOLUTION:
            return 'NO_IK_SOLUTION'
        else:
            return 'UNKNOWN_ERROR_CODE'
        
