#!/usr/bin/env python

import rospy
from joint_state_reader import JointStateReader

def wait_for_time():
    while rospy.Time().now().to_sec() == 0:
        pass

def main():
    rospy.init_node('joint_reader_demo')
    wait_for_time()
    reader = JointStateReader()
    rospy.sleep(0.5)

    # Fetch机器人手臂的关节列表
    names = [
        'shoulder_pan_joint', 'shoulder_lift_joint',
        'upperarm_roll_joint', 'elbow_flex_joint',
        'forearm_roll_joint', 'wrist_flex_joint',
        'wrist_roll_joint'
    ]

    joint_values = reader.get_joints(names)
    for name, value in zip(names, joint_values):
        print('{}:\t{}'.format(name, value))

if __name__ == '__main__':
    main()
