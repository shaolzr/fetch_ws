#! /usr/bin/env python3
"""
Add floor & table to MoveIt PlanningScene
"""

import rospy
from moveit_python import PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from std_msgs.msg import ColorRGBA

# ------------ 桌子参数：按实际改 ------------
TABLE = dict(size=(0.50, 1.0, 0.72),   # x, y, z (m)
             pos =(1.00, 0.0, 0.72/2))  # center in base_link
# 地面
FLOOR = dict(size=(2.0, 2.0, 0.01),
             pos =(0.0, 0.0, 0.01/2))

def wait_sim():
    while rospy.Time().now().to_sec() == 0:
        rospy.sleep(0.1)

def main():
    rospy.init_node('lab26_obstacles')
    wait_sim()

    ps = PlanningSceneInterface('base_link')
    ps.clear()
    ps.removeCollisionObject('floor')
    ps.removeCollisionObject('table')

    # add floor & table
    ps.addBox('floor', *FLOOR['size'], *FLOOR['pos'])
    ps.addBox('table', *TABLE['size'], *TABLE['pos'])
    rospy.loginfo("Floor & table added.")

    # ---- 上色（灰色） -------------------------------------------------
    pub = rospy.Publisher('/planning_scene', PlanningScene, queue_size=1, latch=True)
    rospy.sleep(0.5)

    grey = ColorRGBA(0.5, 0.5, 0.5, 1.0)
    pc_msg = PlanningScene(is_diff=True,
                           object_colors=[
                               ObjectColor(id='floor', color=grey),
                               ObjectColor(id='table', color=grey)])
    pub.publish(pc_msg)
    rospy.loginfo("Colors applied to objects.")

if __name__ == '__main__':
    main()
