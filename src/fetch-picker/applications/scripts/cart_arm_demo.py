#!/usr/bin/env python
#Lab19
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from robot_api import Arm


def main():
    rospy.init_node('cart_arm_demo')
    arm = Arm()

    def shutdown():
        arm.cancel_all_goals()
    rospy.on_shutdown(shutdown)

    pose1 = PoseStamped()
    pose1.header.frame_id = 'base_link'
    pose1.pose = Pose(Point(0.042, 0.384, 1.826), Quaternion(0.173, -0.693, -0.242, 0.657))

    pose2 = PoseStamped()
    pose2.header.frame_id = 'base_link'
    pose2.pose = Pose(Point(0.047, 0.545, 1.822), Quaternion(-0.274, -0.701, 0.173, 0.635))

    poses = [pose1, pose2]

    rospy.sleep(1)  # Ensure connection ready

    while not rospy.is_shutdown():
        for pose in poses:
            error = arm.move_to_pose(pose)
            if error:
                rospy.logerr('Move failed: %s' % error)
            rospy.sleep(1)  # Wait for arm to settle

if __name__ == '__main__':
    main()
