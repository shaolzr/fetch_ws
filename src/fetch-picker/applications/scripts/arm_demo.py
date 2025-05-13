#! /usr/bin/env python

import robot_api # 导入 robot_api 库用于控制机器人
import rospy # 导入 rospy 库用于 ROS 通信


def wait_for_time(): # 等待模拟时间开始
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0: # 如果时间为零
        pass


def main():
    rospy.init_node('arm_demo') # 初始化 ROS 节点
    wait_for_time()
    argv = rospy.myargv() # 获取命令行参数
    DISCO_POSES = [[1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0],
                   [0.8, 0.75, 0.0, -2.0, 0.0, 2.0, 0.0],
                   [-0.8, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0],
                   [-1.5, 1.1, -3.0, -0.5, -3.0, -1.0, -3.0],
                   [-0.8, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0],
                   [0.8, 0.75, 0.0, -2.0, 0.0, 2.0, 0.0],
                   [1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0]]

    torso = robot_api.Torso() # 创建机器人躯干对象
    torso.set_height(robot_api.Torso.MAX_HEIGHT) # 设置躯干高度为最大值

    arm = robot_api.Arm() # 创建机器人手臂对象
    for vals in DISCO_POSES: # 遍历每个目标姿态
        arm.move_to_joints(robot_api.ArmJoints.from_list(vals)) # 移动到目标关节位置


if __name__ == '__main__': # 如果是主程序执行
    main()
