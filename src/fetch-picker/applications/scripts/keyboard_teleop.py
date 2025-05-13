#!/usr/bin/env python

import robot_api
import rospy


import sys, select, termios, tty

msg = """
Control Your Fetch!
---------------------------
Moving around:
        w
   a    s    d

Space: force stop
i/k: increase/decrease only linear speed by 5 cm/s
u/j: increase/decrease only angular speed by 0.25 rads/s
anything else: stop smoothly

CTRL-C to quit
"""
## --- 按键到运动方向映射 ---
moveBindings = {'w': (1, 0), 'a': (0, 1), 'd': (0, -1), 's': (-1, 0)}

# --- 按键到速度增量映射 ---
speedBindings = {
    'i': (0.05, 0),
    'k': (-0.05, 0),
    'u': (0, 0.25),
    'j': (0, -0.25),
}


def getKey():
    """非阻塞读取单个按键；无按键时返回空字符。"""
    tty.setraw(sys.stdin.fileno())# 终端进入 raw 模式
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)# 最多等待 0.1 s
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)# 恢复终端模式
    return key


speed = 5  # 线速度基准 (m/s)；默认较大，实际会乘以 0/1
turn = 0.25# 角速度基准 (rad/s)


def vels(speed, turn):
    """返回当前基准速度字符串，用于打印。"""
    return "currently:\tspeed %s\tturn %s " % (speed, turn)


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin) # 备份终端属性，程序结束后恢复

    rospy.init_node('fetch_teleop_key')# 初始化节点
    base = robot_api.Base()# 创建底座控制对象
 # 状态变量初始化
    x = 0 # 前后方向系数：1 前，-1 后
    th = 0 # 左右转向系数：1 左转，-1 右转
    status = 0 # 用于定期打印提示
    count = 0 # 连续空键计数，实现松手后平滑减速
    target_speed = 0
    target_turn = 0
    control_speed = 0 # 当前实际发送的速度
    control_turn = 0
    try:
        print (msg)
        print (vels(speed, turn))
        while (1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                count = 0
            elif key in speedBindings.keys():
                speed += speedBindings[key][0]
                turn += speedBindings[key][1]
                count = 0

                print (vels(speed, turn))
                if (status == 14):
                    print (msg)
                status = (status + 1) % 15
            elif key == ' ':
                x = 0
                th = 0
                control_speed = 0
                control_turn = 0
            else:
                count = count + 1
                if count > 4: # 连续 4 次没按方向键 → 慢慢归零
                    x = 0
                    th = 0
                if (key == '\x03'):
                    break
            # --- 根据系数计算目标速度 ---
            target_speed = speed * x
            target_turn = turn * th
            # --- 一阶逼近：线速度平滑 ---
            if target_speed > control_speed:
                control_speed = min(target_speed, control_speed + 0.02)
            elif target_speed < control_speed:
                control_speed = max(target_speed, control_speed - 0.02)
            else:
                control_speed = target_speed
           
            # --- 一阶逼近：角速度平滑 ---
            if target_turn > control_turn:
                control_turn = min(target_turn, control_turn + 0.1)
            elif target_turn < control_turn:
                control_turn = max(target_turn, control_turn - 0.1)
            else:
                control_turn = target_turn
            
            # --- 发送到底座 ---
            base.move(control_speed, control_turn)
    except Exception as e:# 异常时打印 ROS 日志
        rospy.logerr('{}'.format(e))
    finally:
        base.stop() # 退出前确保停止
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
