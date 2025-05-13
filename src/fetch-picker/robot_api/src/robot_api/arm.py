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
        traj.joint_names = ArmJoints.names()# 设置关节名称列表 告诉控制器“要控制哪些关节”
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
    #     #lab 19提供的
    #     goal_builder = MoveItGoalBuilder()
    #     goal_builder.set_pose_goal(pose_stamped)
    #     goal = goal_builder.build()

    #     self._move_group_client.send_goal(goal)
    #     finished = self._move_group_client.wait_for_result(rospy.Duration(10))  # Timeout version

    #     if not finished:
    #         return 'Timeout waiting for result'
        
    #     result = self._move_group_client.get_result()
    #     if result.error_code.val != MoveItErrorCodes.SUCCESS:
    #         return Arm.moveit_error_string(result.error_code.val)
        
    #     return None

    #Lab 20的move to pose
    def move_to_pose(self,
                 pose_stamped,
                 allowed_planning_time=10.0,
                 execution_timeout=15.0,
                 group_name='arm',
                 num_planning_attempts=1,
                 plan_only=False,
                 replan=False,
                 replan_attempts=5,
                 tolerance=0.01,
                 orientation_constraint=None):
        """Moves the end-effector to a pose, using motion planning."""
        goal_builder = MoveItGoalBuilder()
        goal_builder.set_pose_goal(pose_stamped)
        goal_builder.allowed_planning_time = allowed_planning_time
        goal_builder.num_planning_attempts = num_planning_attempts
        goal_builder.plan_only = plan_only
        goal_builder.replan = replan
        goal_builder.replan_attempts = replan_attempts
        goal_builder.tolerance = tolerance
        goal_builder.group_name = group_name

        #Lab 22添加  修改方法内部，支持设置 orientation_constraint
        if orientation_constraint is not None:
            goal_builder.add_path_orientation_constraint(orientation_constraint)


        goal = goal_builder.build()
        # self._move_group_client.send_goal(goal)

        # finished = self._move_group_client.wait_for_result(rospy.Duration(execution_timeout))
        # if not finished:
        #     return 'Timeout waiting for result'

        # result = self._move_group_client.get_result()
        # if result.error_code.val != MoveItErrorCodes.SUCCESS:
        #     return Arm.moveit_error_string(result.error_code.val)
        # return None

        # Lab26
        self._move_group_client.send_goal(goal)
        finished = self._move_group_client.wait_for_result(
                rospy.Duration(execution_timeout))

        if not finished:
                rospy.logwarn("MoveIt timeout")
                return False                              # ⟵ 规划失败

        result = self._move_group_client.get_result()
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return True                               # ⟵ 规划成功
        else:
                rospy.logwarn("MoveIt failed: %s",
                            Arm.moveit_error_string(result.error_code.val))
                return False                              # ⟵ 规划失败
    
    #Lab 20 逆运动
    def compute_ik(self, pose_stamped, timeout=rospy.Duration(5)):
        """Computes inverse kinematics for the given pose."""
        request = GetPositionIKRequest()
        request.ik_request.pose_stamped = pose_stamped
        request.ik_request.group_name = 'arm'
        request.ik_request.timeout = timeout
        response = self._compute_ik(request)
        error_str = Arm.moveit_error_string(response.error_code.val)
        success = error_str == 'SUCCESS'
        if not success:
            return False
        joint_state = response.solution.joint_state
        for name, position in zip(joint_state.name, joint_state.position):
            if name in ArmJoints.names():  # 确保你有这个方法，或自己写个列表返回各关节名
                rospy.loginfo('{}: {}'.format(name, position))
        return True


    def check_pose(self, 
               pose_stamped,
               allowed_planning_time=10.0,
               group_name='arm',
               tolerance=0.01):
        return self.move_to_pose(
            pose_stamped,
            allowed_planning_time=allowed_planning_time,
            group_name=group_name,
            tolerance=tolerance,
            plan_only=True)


    
    #Lab 19 添加取消方法 cancel_all_goals
    def cancel_all_goals(self):
        self._arm_ac.cancel_all_goals() # Your action client from Lab 7 替代成lab7的client
        self._move_group_client.cancel_all_goals() # From this lab
    
    #Lab 19 添加工具函数 moveit_error_string
    @staticmethod
    def moveit_error_string(val):
        """Returns a string associated with a MoveItErrorCode.
            
        Args:
            val: The val field from moveit_msgs/MoveItErrorCodes.msg
            
        Returns: The string associated with the error value, 'UNKNOWN_ERROR_CODE'
            if the value is invalid.
        """ 
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
        
