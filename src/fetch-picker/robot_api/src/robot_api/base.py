#! /usr/bin/env python

# TODO: import ????????_msgs.msg
import rospy
from geometry_msgs.msg import Twist # /cmd_vel 速度消息
from nav_msgs.msg import Odometry # /odom 里程计消息
import tf.transformations # 四元数→欧拉角工具
import math
import copy


class Base(object):
    """Base controls the mobile base portion of the Fetch robot.

    Sample usage:
        base = robot_api.Base()
        while CONDITION:
            base.move(0.2, 0)
        base.stop()
    """

    def __init__(self):
        # TODO: Create publisher
        # 创建 /cmd_vel 发布者，消息类型 Twist，队列长度 10
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
         # 订阅 /odom，以获取实时里程计
        self._odom_sub = rospy.Subscriber('/odom', Odometry, self._odom_callback)
        self._latest_odom = None  # 保存最近一次的 odom
    
    def _odom_callback(self, msg):
        """里程计回调：更新最新姿态。"""
        self._latest_odom = msg

    def move(self, linear_speed, angular_speed):
        """Moves the base instantaneously at given linear and angular speeds.

        "Instantaneously" means that this method must be called continuously in
        a loop for the robot to move.

        Args:
            linear_speed: The forward/backward speed, in meters/second. A
                positive value means the robot should move forward.
            angular_speed: The rotation speed, in radians/second. A positive
                value means the robot should rotate clockwise.
        """
        # TODO: Create Twist msg
        msg = Twist()# 创建速度消息
        # TODO: Fill out msg
        msg.linear.x = linear_speed # 前后方向速度 (m/s)
        msg.angular.z = angular_speed # 旋转速度 (rad/s)
        # TODO: Publish msg
        self.pub.publish(msg) # 发布到 /cmd_vel
        ##rospy.logerr('Not implemented.')

    def stop(self):
        """Stops the mobile base from moving.
        """
        # TODO: Publish 0 velocity
        """停止运动：发送 0 速度。"""
        self.move(0, 0)
        ##rospy.logerr('Not implemented.')
    
    def go_forward(self, distance, speed=0.1):
        """Moves the robot a certain distance."""
        """依据里程计前进指定距离 (m)。"""
        rospy.sleep(1)# 等系统稳定
        while self._latest_odom is None: # 等首帧里程计
            rospy.sleep(0.1)

        # 记录起点坐标
        start = copy.deepcopy(self._latest_odom)
        start_x = start.pose.pose.position.x
        start_y = start.pose.pose.position.y
        rate = rospy.Rate(10) # 10 Hz 控制循环
        
        # 嵌套函数：计算已行驶距离
        def traveled():
            curr = self._latest_odom.pose.pose.position
            dx = curr.x - start_x
            dy = curr.y - start_y
            return math.sqrt(dx**2 + dy**2) #欧几里得距离

        direction = -1 if distance < 0 else 1 ## 负距离＝后退

        # 循环向目标迈进，并根据剩余距离自适应调速
        while traveled() < abs(distance):
            remaining = abs(distance) - traveled()
            linear_speed = max(0.05, min(0.3, remaining))
            self.move(direction * linear_speed, 0)
            rate.sleep()

        self.stop()

    def _get_yaw(self):
        """Returns the yaw angle from the latest odom reading."""
        orientation = self._latest_odom.pose.pose.orientation
        quat = [orientation.x, orientation.y, orientation.z, orientation.w]
        (_, _, yaw) = tf.transformations.euler_from_quaternion(quat)
        return yaw

    def turn(self, angular_distance, speed=0.5):
        """Rotates the robot a certain angle."""
        rospy.sleep(1)
        while self._latest_odom is None:
            rospy.sleep(0.1)

        start_yaw = self._get_yaw() % (2 * math.pi)# 初始角
        goal_yaw = (start_yaw + angular_distance) % (2 * math.pi)

        rate = rospy.Rate(10)

        # 嵌套函数：计算剩余角度（符号代表方向，绝对值代表大小）
        def remaining_angle():
            curr_yaw = self._get_yaw() % (2 * math.pi) 
            ccw_diff = (goal_yaw - curr_yaw) % (2 * math.pi) # 逆时针差
            cw_diff = (curr_yaw - goal_yaw) % (2 * math.pi) # 顺时针差
            return ccw_diff if ccw_diff < cw_diff else -cw_diff

        # 循环旋转，并根据剩余角度自适应调速
        while abs(remaining_angle()) > 0.01: # 误差 <0.01 rad (~0.6°) 即视为完成
            angle = abs(remaining_angle())
            direction = 1 if remaining_angle() > 0 else -1
            angular_speed = max(0.1, min(0.5, angle))
            self.move(0, direction * angular_speed)
            rate.sleep()

        self.stop()