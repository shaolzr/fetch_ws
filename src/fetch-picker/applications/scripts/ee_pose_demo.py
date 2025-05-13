#!/usr/bin/env python

import rospy
import tf

def main():
    rospy.init_node('ee_pose_demo')
    listener = tf.TransformListener()

    # 🔥 等 tf 收到数据，不要立刻 lookup
    rospy.sleep(1.0)

    rate = rospy.Rate(1)  # 1Hz, 每秒一次
    while not rospy.is_shutdown():
        try:
            # 🔥 查询 gripper_link 相对于 base_link 的变换
            (trans, rot) = listener.lookupTransform('base_link', 'gripper_link', rospy.Time(0))
            rospy.loginfo("Position: %s, Orientation: %s", trans, rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(str(e))
        rate.sleep()

if __name__ == '__main__':
    main()
#tf.TransformListener()：启动 TF 监听器

# lookupTransform('base_link', 'gripper_link', rospy.Time(0))：

# 查找 base_link ➔ gripper_link 的实时变换

# rospy.sleep(1.0)：留时间让 tf buffer 接收完初始数据

# rate = rospy.Rate(1)：以每秒 1次 的速度循环

# try/except：防止刚启动时找不到 frame 报错