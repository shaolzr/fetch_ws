#!/usr/bin/env python

import rospy  # 导入 ROS Python 客户端库
import math  # 导入数学库，用于计算距离
from nav_msgs.msg import Odometry  # 导入里程计消息类型
from visualization_msgs.msg import Marker  # 导入 RViz 可视化 Marker 消息类型
from geometry_msgs.msg import Point  # 导入几何点类型
from std_msgs.msg import ColorRGBA, Header  # 导入标准消息类型，用于颜色和消息头
from geometry_msgs.msg import Vector3, Pose, Quaternion  # 导入几何消息类型

class NavPath(object): # 定义一个类 NavPath，用于记录和发布路径轨迹
    def __init__(self):
        self._last_position = None # 上一次记录的位置，初始为空
        self._points = [] # 保存轨迹点的列表

        self._marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
         # 创建一个发布器，发布到 visualization_marker 话题，队列长度10
        rospy.sleep(0.5)  # 等待发布器和 RViz 连接稳定

    def callback(self, msg): # 回调函数，当收到 /odom 消息时调用
        pos = msg.pose.pose.position # 提取当前位置（x, y, z）
        current_point = Point(pos.x, pos.y, pos.z) # 创建 Point 对象

        # 如果是第一次收到数据，初始化 last_position 并存入 points，不画线
        if self._last_position is None:
            self._last_position = current_point
            self._points.append(current_point)
            return

        # 计算当前点与上一个记录点的欧式距离
        dist = math.sqrt(
            (current_point.x - self._last_position.x) ** 2 +
            (current_point.y - self._last_position.y) ** 2 +
            (current_point.z - self._last_position.z) ** 2
        )

        # 如果距离超过 0.1 米，才添加到轨迹中（避免太密集）
        if dist >= 0.1:
            self._points.append(current_point) # 加入轨迹点
            self._last_position = current_point # 更新上一次位置

            # 发布更新后的轨迹线到 RViz
            self.publish_marker()

    def publish_marker(self):
        # 如果少于两个点，不画线
        if len(self._points) < 2:
            return

        marker = Marker()
        marker.header = Header(frame_id="odom")   # 设置参考坐标系为 odom（或根据实际情况修改）
        marker.type = Marker.LINE_STRIP # 设置标记类型为线条串
        marker.action = Marker.ADD # 设置操作为添加
        marker.scale = Vector3(0.02, 0.0, 0.0) # 设置线条宽度（x 分量有效）
        marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0) # 设置颜色为红色，不透明
        marker.pose.orientation = Quaternion(0, 0, 0, 1) # 设置默认方向
        marker.points = self._points # 设置所有轨迹点
        marker.id = 0 # 设置标记 ID（唯一）
        marker.lifetime = rospy.Duration(0)  # 设置标记为永久存在

        self._marker_pub.publish(marker) # 发布 Marker 消息

def main():
    rospy.init_node('path_marker_node') # 初始化 ROS 节点，命名为 path_marker_node
    nav_path = NavPath() # 创建 NavPath 实例
    rospy.Subscriber('/odom', Odometry, nav_path.callback) # 订阅 /odom 话题，接收 Odometry 消息，并绑定回调函数
    rospy.spin() # 保持节点运行，进入循环等待回调

if __name__ == '__main__':
    main()
