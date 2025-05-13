#!/usr/bin/env python

import rospy # 导入 ROS Python 客户端库
from visualization_msgs.msg import Marker  # 导入 Marker 消息类型，用于在 RViz 中可视化标记
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3 # 导入几何消息类型，用于定义姿态、位置、缩放
from std_msgs.msg import Header, ColorRGBA # 导入标准消息类型，用于定义消息头和颜色

def wait_for_time():
    # 等待模拟时间启动（防止一开始获取到的时间是0）
    while rospy.Time().now().to_sec() == 0:
        pass

def show_text_in_rviz(marker_publisher, text):
    # 创建一个 Marker 消息，用于在 RViz 中显示文本
    marker = Marker(
        type=Marker.TEXT_VIEW_FACING, # 设置标记类型为“面向视角的文本”
        id=0, # 标记 ID，唯一标识
        lifetime=rospy.Duration(1.5), # 标记持续时间，这里是1.5秒后消失
        pose=Pose(Point(0.5, 0.5, 1.45), Quaternion(0, 0, 0, 1)), # 设置标记的位置和方向
        scale=Vector3(0.1, 0.1, 0.1), # 设置标记的缩放大小（注意文本只用 z 分量）
        header=Header(frame_id='base_link'), # 设置标记的坐标系，这里是 base_link
        color=ColorRGBA(0.0, 1.0, 0.0, 0.8), # 设置标记的颜色（绿色，透明度0.8）
        text=text # 要显示的文本内容
    )
    marker_publisher.publish(marker) # 发布这个标记到话题

def main():
    rospy.init_node('marker_demo') # 初始化 ROS 节点，命名为 marker_demo
    wait_for_time() # 等待模拟时间准备好
    marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)
    # 创建一个 Marker 类型的发布者，发布到 /visualization_marker 话题，队列长度5
    rospy.sleep(0.5)  # 等待 ROS master 建立连接和通知订阅者
    show_text_in_rviz(marker_publisher, 'Hello Fetch!')
    # 调用函数，在 RViz 中显示文本“Hello Fetch!”

if __name__ == '__main__':
    main() # 如果作为主程序运行，调用 main 函数
