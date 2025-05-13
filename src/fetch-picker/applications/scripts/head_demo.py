#!/usr/bin/env python

import rospy
from robot_api import Head  # 从 robot_api 模块导入 Head 类（需要 robot_api/__init__.py 里有 from .head import Head）

def print_usage():
    print ('Usage:')
    print ('    rosrun applications head_demo.py look_at FRAME_ID X Y Z')
    print ('    rosrun applications head_demo.py pan_tilt PAN_ANG TILT_ANG')
    print ('Examples:')
    print ('    rosrun applications head_demo.py look_at base_link 1 0 0.3')
    print ('    rosrun applications head_demo.py pan_tilt 0 0.707')

def wait_for_time():
    """Wait for simulated time to begin."""
    while rospy.Time().now().to_sec() == 0:
        pass

def main():
    rospy.init_node('head_demo') # 初始化 ROS 节点，名字是 head_demo
    wait_for_time() # 确保仿真时间启动再继续
    argv = rospy.myargv() # 获取命令行参数（必须用 rospy.myargv 而不是 sys.argv，因为 ROS 会插入自己的参数）
    if len(argv) < 2:  # 如果参数少于2个（至少要有命令名），打印用法
        print_usage()
        return

    command = argv[1]  # 获取第一个命令参数（look_at 或 pan_tilt）
    head = Head()  # 创建 Head 类对象，用于控制机器人头部

    if command == 'look_at':
        # 如果命令是 look_at，要求至少提供 frame_id、x、y、z（总共 6 个参数）
        if len(argv) < 6:
            print_usage() # 参数不够，打印用法
            return
        frame_id = argv[2] # 坐标系名称（例如 base_link）
        x, y, z = float(argv[3]), float(argv[4]), float(argv[5]) # 获取目标点坐标并转成 float
        head.look_at(frame_id, x, y, z) # 调用 Head 类的 look_at 方法，让头部转向目标点
    elif command == 'pan_tilt':
        if len(argv) < 4:
            print_usage() # 参数不够，打印用法
            return
        pan, tilt = float(argv[2]), float(argv[3]) # 获取 pan 和 tilt 角度并转成 float
        head.pan_tilt(pan, tilt) # 调用 Head 类的 pan_tilt 方法，让头部按角度转动
    else:
        # 命令既不是 look_at 也不是 pan_tilt，打印用法
        print_usage()

if __name__ == '__main__':
    main()
