#! /usr/bin/env python

# 导入数学计算库
import math
# 导入numpy，用于矩阵运算
import numpy as np
# 导入ROS几何消息类型
from geometry_msgs.msg import Point, Pose, PoseStamped
# 导入ROS颜色消息类型
from std_msgs.msg import ColorRGBA
# 导入ROS可视化消息类型
import visualization_msgs.msg
# 导入ROS Python客户端库
import rospy
# 导入tf变换工具
import tf.transformations as tft


def wait_for_time():
    """等待ROS时间初始化完成"""
    while rospy.Time().now().to_sec() == 0:
        pass


def cosd(degs):
    """将角度转换为弧度并计算余弦值"""
    return math.cos(degs * math.pi / 180)


def sind(degs):
    """将角度转换为弧度并计算正弦值"""
    return math.sin(degs * math.pi / 180)


def axis_marker(pose_stamped):
    """创建坐标轴标记"""
    # 创建标记对象
    marker = visualization_msgs.msg.Marker()
    # 设置标记的命名空间
    marker.ns = 'axes'
    # 设置标记的头部信息
    marker.header = pose_stamped.header
    # 设置标记的位姿
    marker.pose = pose_stamped.pose
    # 设置标记类型为线列表
    marker.type = visualization_msgs.msg.Marker.LINE_LIST
    # 设置线宽
    marker.scale.x = 0.1

    # 添加X轴（红色）
    marker.points.append(Point(0, 0, 0))
    marker.colors.append(ColorRGBA(1, 0, 0, 1))
    marker.points.append(Point(1, 0, 0))
    marker.colors.append(ColorRGBA(1, 0, 0, 1))

    # 添加Y轴（绿色）
    marker.points.append(Point(0, 0, 0))
    marker.colors.append(ColorRGBA(0, 1, 0, 1))
    marker.points.append(Point(0, 1, 0))
    marker.colors.append(ColorRGBA(0, 1, 0, 1))

    # 添加Z轴（蓝色）
    marker.points.append(Point(0, 0, 0))
    marker.colors.append(ColorRGBA(0, 0, 1, 1))
    marker.points.append(Point(0, 0, 1))
    marker.colors.append(ColorRGBA(0, 0, 1, 1))

    return marker


def transform_to_pose(matrix):
    """将4x4变换矩阵转换为Pose消息"""
    # 创建Pose对象
    pose = Pose()
    # 从矩阵中提取位置信息
    pose.position.x = matrix[0, 3]  # 提取x坐标
    pose.position.y = matrix[1, 3]  # 提取y坐标
    pose.position.z = matrix[2, 3]  # 提取z坐标

    # 从矩阵中提取旋转信息并转换为四元数
    qx, qy, qz, qw = tft.quaternion_from_matrix(matrix)
    pose.orientation.x = qx  # 设置四元数x分量
    pose.orientation.y = qy  # 设置四元数y分量
    pose.orientation.z = qz  # 设置四元数z分量
    pose.orientation.w = qw  # 设置四元数w分量

    return pose


def arrow_marker(point):
    """创建箭头标记"""
    # 创建标记对象
    marker = visualization_msgs.msg.Marker()
    # 设置标记的命名空间
    marker.ns = 'arrow'
    # 设置标记类型为箭头
    marker.type = visualization_msgs.msg.Marker.ARROW
    # 设置参考坐标系
    marker.header.frame_id = 'frame_a'
    # 设置箭头的起点和终点
    marker.points.append(Point(0, 0, 0))  # 起点
    marker.points.append(point)  # 终点
    # 设置箭头的尺寸
    marker.scale.x = 0.1  # 箭头长度
    marker.scale.y = 0.15  # 箭头宽度
    # 设置箭头颜色（白色）
    marker.color.r = 1
    marker.color.g = 1
    marker.color.a = 1
    return marker


def main():
    """主函数"""
    # 初始化ROS节点
    rospy.init_node('transformation_demo')
    # 等待时间初始化
    wait_for_time()
    # 创建可视化发布者
    viz_pub = rospy.Publisher(
        'visualization_marker', visualization_msgs.msg.Marker, queue_size=1)
    # 等待发布者初始化
    rospy.sleep(0.5)
    # 创建45度旋转的变换矩阵
    b_in_a = np.array([
        [cosd(45), -sind(45), 0, 0],  # 旋转矩阵部分
        [sind(45), cosd(45), 0, 0],   # 旋转矩阵部分
        [0, 0, 1, 0.5],               # 平移部分
        [0, 0, 0, 1]                  # 齐次坐标
    ])
    # 创建位姿消息
    ps = PoseStamped()
    # 设置参考坐标系
    ps.header.frame_id = 'frame_a'
    # 将变换矩阵转换为位姿
    ps.pose = transform_to_pose(b_in_a)
    # 发布坐标轴标记
    viz_pub.publish(axis_marker(ps))

    # 创建B坐标系中的点
    point_in_b = np.array([1, 0, 0, 1])
    # 将点从B坐标系转换到A坐标系
    point_in_a = np.dot(b_in_a, point_in_b)
    # 打印转换前后的点坐标
    rospy.loginfo(point_in_b)
    rospy.loginfo(point_in_a)
    # 创建Point消息
    point = Point(point_in_a[0], point_in_a[1], point_in_a[2])
    # 发布箭头标记
    viz_pub.publish(arrow_marker(point))

# 当脚本直接运行时执行main函数
if __name__ == '__main__':
    main()