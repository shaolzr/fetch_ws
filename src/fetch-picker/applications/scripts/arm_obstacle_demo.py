#!/usr/bin/env python  # 指定Python解释器路径
#Lab21  # 标记这是Lab21的代码
import rospy  # 导入ROS的Python客户端库
from geometry_msgs.msg import PoseStamped  # 导入位姿消息类型
from moveit_python import PlanningSceneInterface  # 导入MoveIt场景规划接口
import robot_api  # 导入机器人API模块
#Lab 22 添加 引入 OrientationConstraint  # 标记这是Lab22添加的代码
from moveit_msgs.msg import OrientationConstraint  # 导入方向约束消息类型


def main():  # 定义主函数
    # 初始化ROS节点，节点名为'arm_obstacle_demo'
    rospy.init_node('arm_obstacle_demo')  # 初始化ROS节点

    # 创建Arm对象用于控制机器人手臂
    arm = robot_api.Arm()  # 创建机器人手臂控制对象

    # 定义关闭函数，用于取消所有正在执行的目标
    def shutdown():  # 定义关闭函数
        arm.cancel_all_goals()  # 取消所有正在执行的目标
    # 注册关闭函数，当节点关闭时执行
    rospy.on_shutdown(shutdown)  # 注册关闭回调函数
    
    # 创建规划场景接口，用于管理障碍物
    planning_scene = PlanningSceneInterface('base_link')  # 创建规划场景接口，以base_link为参考系

    # 等待1秒，确保初始化完成
    rospy.sleep(1)  # 等待1秒，确保初始化完成

    # 清除之前的障碍物
    planning_scene.removeCollisionObject('table')  # 移除桌子碰撞物体
    planning_scene.removeCollisionObject('divider')  # 移除分隔板碰撞物体
    planning_scene.removeAttachedObject('tray')  # 移除托盘附着物体

    # 设置桌子的尺寸和位置参数
    table_size_x = 0.5  # 桌子长度
    table_size_y = 1.0  # 桌子宽度
    table_size_z = 0.03  # 桌子高度
    table_x = 0.8  # 桌子x坐标
    table_y = 0.0  # 桌子y坐标
    table_z = 0.6  # 桌子z坐标
    # 添加桌子到场景中
    planning_scene.addBox('table', table_size_x, table_size_y, table_size_z,
                          table_x, table_y, table_z)  # 添加桌子碰撞物体到场景

    # # 添加分隔板（divider）Lab 22注释掉  # 注释掉的分隔板代码
    # size_x = 0.3  # 分隔板长度
    # size_y = 0.01  # 分隔板宽度
    # size_z = 0.4  # 分隔板高度
    # x = table_x - (table_size_x / 2) + (size_x / 2)  # 分隔板x坐标
    # y = 0.0  # 分隔板y坐标
    # z = table_z + (table_size_z / 2) + (size_z / 2)  # 分隔板z坐标
    # planning_scene.addBox('divider', size_x, size_y, size_z, x, y, z)  # 添加分隔板到场景


    #设置两个姿态（绕开分隔器）
    pose1 = PoseStamped()  # 创建第一个位姿对象
    pose1.header.frame_id = 'base_link'  # 设置参考坐标系
    pose1.pose.position.x = 0.5  # 设置x坐标
    pose1.pose.position.y = -0.3  # 设置y坐标
    pose1.pose.position.z = 0.75  # 设置z坐标
    pose1.pose.orientation.w = 1.0  # 设置姿态（单位四元数）


    pose2 = PoseStamped()  # 创建第二个位姿对象
    pose2.header.frame_id = 'base_link'  # 设置参考坐标系
    pose2.pose.position.x = 0.5  # 设置x坐标
    pose2.pose.position.y = 0.3  # 设置y坐标
    pose2.pose.position.z = 0.75  # 设置z坐标
    pose2.pose.orientation.w = 1.0  # 设置姿态（单位四元数）
    
    # 创建方向约束对象
    oc = OrientationConstraint()  # 创建方向约束对象
    oc.header.frame_id = 'base_link'  # 设置参考坐标系
    oc.link_name = 'wrist_roll_link'  # 设置约束的连杆
    oc.orientation.w = 1  # 设置目标方向
    oc.absolute_x_axis_tolerance = 0.1  # x轴方向容差
    oc.absolute_y_axis_tolerance = 0.1  # y轴方向容差
    oc.absolute_z_axis_tolerance = 3.14  # z轴方向容差
    oc.weight = 1.0  # 约束权重


    #调用 move_to_pose 执行动作并避障
    kwargs = {  # 设置运动规划参数
        'allowed_planning_time': 15,  # 允许的规划时间
        'execution_timeout': 10,  # 执行超时时间
        'num_planning_attempts': 5,  # 规划尝试次数
        'replan': False  # 是否允许重新规划
    }
    
    
    error = arm.move_to_pose(pose1, **kwargs)  # 移动到第一个位置
    if error is not None:  # 如果发生错误
        rospy.logerr('Pose 1 failed: {}'.format(error))  # 记录错误信息
    else:  # 如果成功
        rospy.loginfo('Pose 1 succeeded')  # 记录成功信息
        #Lab 21-2添加 Lab 21 在 move_to_pose(pose1) 成功后添加：
        frame_attached_to = 'gripper_link'  # 设置托盘附着的连杆
        frames_okay_to_collide_with = [  # 设置允许碰撞的连杆列表
            'gripper_link', 'l_gripper_finger_link', 'r_gripper_finger_link'
        ]
        planning_scene.attachBox('tray', 0.3, 0.07, 0.01, 0.05, 0, 0,  # 添加托盘到场景
                                frame_attached_to, frames_okay_to_collide_with)
        planning_scene.setColor('tray', 1, 0, 1)  # 设置托盘颜色（紫色）
        planning_scene.sendColors()  # 发送颜色信息

    rospy.sleep(1)  # 等待1秒


    # Lab22 === 带方向约束去 pose2
    error = arm.move_to_pose(pose2, orientation_constraint=oc, **kwargs)  # 移动到第二个位置（带方向约束）
    
    #Lab22 注释掉
    #error = arm.move_to_pose(pose2, **kwargs)  # 注释掉的移动到第二个位置的代码
    if error is not None:  # 如果发生错误
        rospy.logerr('Pose 2 failed: {}'.format(error))  # 记录错误信息
    else:  # 如果成功
        rospy.loginfo('Pose 2 succeeded')  # 记录成功信息
    
    #移除障碍物（程序末尾清理）
    planning_scene.removeCollisionObject('table')  # 移除桌子
    planning_scene.removeCollisionObject('divider')  # 移除分隔板
    #Lab 21-2添加
    planning_scene.removeAttachedObject('tray')  # 移除托盘

if __name__ == '__main__':  # 当脚本直接运行时
    main()  # 执行主函数




