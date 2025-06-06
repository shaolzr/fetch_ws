#!/usr/bin/env python

# 导入rospy，用于ROS节点操作
import rospy
# 导入tf，用于坐标变换
import tf

def main():
    # 初始化ROS节点，节点名为'ee_pose_demo'
    rospy.init_node('ee_pose_demo')
    # 创建tf监听器，用于获取坐标变换信息
    listener = tf.TransformListener()

    # 等待1秒，确保tf树已经建立并收到数据
    rospy.sleep(1.0)

    # 创建Rate对象，设置循环频率为1Hz（每秒一次）
    rate = rospy.Rate(1)  # 1Hz, 每秒一次
    # 主循环，当ROS节点没有关闭时持续运行
    while not rospy.is_shutdown():
        try:
            # 查询gripper_link（末端执行器）相对于base_link（机器人基座）的变换
            # trans: 位置变换（x,y,z）
            # rot: 姿态变换（四元数）
            (trans, rot) = listener.lookupTransform('base_link', 'gripper_link', rospy.Time(0))
            # 打印位置和姿态信息
            rospy.loginfo("Position: %s, Orientation: %s", trans, rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            # 如果发生异常（查找失败、连接异常、外推异常），打印警告信息
            rospy.logwarn(str(e))
        # 按照设定的频率休眠
        rate.sleep()

# 当脚本直接运行时执行main函数
if __name__ == '__main__':
    main()
#tf.TransformListener()：启动 TF 监听器

# lookupTransform('base_link', 'gripper_link', rospy.Time(0))：

# 查找 base_link ➔ gripper_link 的实时变换

# rospy.sleep(1.0)：留时间让 tf buffer 接收完初始数据

# rate = rospy.Rate(1)：以每秒 1次 的速度循环

# try/except：防止刚启动时找不到 frame 报错