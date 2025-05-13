#!/usr/bin/env python

import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer # 从 interactive_markers 包中导入交互式标记服务器类
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker, InteractiveMarkerFeedback
from geometry_msgs.msg import Point
# 导入 ROS 消息类型：
# InteractiveMarker：交互式标记本体
# InteractiveMarkerControl：定义控制方式（按钮、移动、旋转等）
# Marker：基本 RViz 可视化元素（立方体、球体等）
# InteractiveMarkerFeedback：定义来自 RViz 的反馈信息
# 导入 Point 消息类型，用于定义三维位置坐标

def make_interactive_marker():
    # 创建并配置一个交互式标记对象
    int_marker = InteractiveMarker() # 新建一个 InteractiveMarker 对象
    int_marker.header.frame_id = "base_link" # 设置它的参考坐标系为 base_link（通常是机器人底座）
    int_marker.name = "simple_marker" # 设置 marker 名字（用于唯一标识）
    int_marker.description = "Simple Click Control" # 在 RViz 中显示的简要描述
    int_marker.scale = 0.2 # 设置整体缩放比例
    int_marker.pose.position = Point(0.5, 0.0, 0.2) # 设置 marker 的初始位置，x=0.5, y=0.0, z=0.2

    # 创建一个可视化用的立方体 Marker
    box_marker = Marker()
    box_marker.type = Marker.CUBE # 设置 Marker 类型为立方体
    box_marker.pose.orientation.w = 1
    box_marker.scale.x = 0.45
    box_marker.scale.y = 0.45
    box_marker.scale.z = 0.45 # 设置立方体在 x, y, z 方向上的尺寸
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0 # 设置立方体颜色为青色 (0, 0.5, 0.5)，透明度为 1（完全不透明）

    # 创建一个交互式控制器
    button_control = InteractiveMarkerControl()
    button_control.interaction_mode = InteractiveMarkerControl.BUTTON # 设置交互模式为“按钮”，只响应点击
    button_control.always_visible = True # 即使没有鼠标悬停或选中，也始终显示
    button_control.markers.append(box_marker) # 把立方体 Marker 添加到控制器中
    

    # 把控制器添加到交互式标记中
    int_marker.controls.append(button_control)

    return int_marker # 返回配置好的交互式标记对象


# 定义处理 RViz 反馈的回调函数
def handle_viz_input(input):
    if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):# 如果事件类型是按钮点击
        rospy.loginfo(input.marker_name + ' was clicked.')# 打印 marker 名称和点击提示到终端
    else: # 如果是其他事件类型
        rospy.loginfo('Cannot handle this InteractiveMarker event') # 打印无法处理该事件的提示

def main():
    rospy.init_node("interactive_marker_demo", anonymous=True) # 初始化 ROS 节点，名称为 interactive_marker_demo

    server = InteractiveMarkerServer("interactive_marker_server") # 创建一个交互式标记服务器，名字叫 interactive_marker_server
    marker = make_interactive_marker() # 创建一个交互式标记对象

    # 使用你的 handle_viz_input 回调函数
    server.insert(marker, handle_viz_input)  # 将 marker 插入服务器，并绑定回调函数 handle_viz_input
    server.applyChanges() # 应用更改，使标记在 RViz 中生效

    rospy.spin()  # 保持节点运行，不断等待和处理 ROS 事件（直到 Ctrl+C 结束）

if __name__ == "__main__":
    main()
