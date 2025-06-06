#!/usr/bin/env python
# Programming by Demonstration (PbD) - Save & Execute Poses Relative to Markers (ROS1 version)

import rospy
from geometry_msgs.msg import PoseStamped, Point
import tf
import pickle
import os
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander, roscpp_initialize, roscpp_shutdown
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
import actionlib

POSES_FILE = 'saved_program.p'
OPENED_POS = 0.10
CLOSED_POS = 0.00
MAX_EFFORT = 50.0

class PBDSystem:
    def __init__(self):
        rospy.init_node('pbd_system')

        self.tf_listener = tf.TransformListener()
        self.poses = []

        roscpp_initialize([])
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group = MoveGroupCommander("arm")
        self.group.set_planning_time(5.0)  # Allow longer planning time

        self._client = actionlib.SimpleActionClient('/gripper_controller/gripper_action', GripperCommandAction)
        self._client.wait_for_server()

        self.run_cli()

    def run_cli(self):
        while not rospy.is_shutdown():
            print("\nOptions:")
            print("1. Save current pose")
            print("2. Save program to file")
            print("3. Load and execute program")
            print("4. Exit")
            option = input("Choose: ").strip()

            if option == '1':
                self.save_current_pose()
            elif option == '2':
                self.save_to_file()
            elif option == '3':
                self.load_and_execute()
            elif option == '4':
                break

    def save_current_pose(self):
        marker_frame = input("Enter marker frame (e.g., ar_marker_1): ").strip()
        gripper_state = input("Gripper state [open/close]: ").strip()

        try:
            self.tf_listener.waitForTransform(marker_frame, 'wrist_roll_link', rospy.Time(0), rospy.Duration(2.0))
            (trans, rot) = self.tf_listener.lookupTransform(marker_frame, 'wrist_roll_link', rospy.Time(0))

            pose_dict = {
                'frame': marker_frame,
                'position': list(trans),
                'orientation': list(rot),
                'gripper': gripper_state
            }
            self.poses.append(pose_dict)
            print(f"Pose saved relative to {marker_frame}.")

        except Exception as e:
            print(f"Failed to lookup transform: {e}")

    def save_to_file(self):
        with open(POSES_FILE, 'wb') as f:
            pickle.dump(self.poses, f)
        print(f"Program saved to {POSES_FILE}.")

    def open_gripper(self):
        print("Opening gripper...")
        goal = GripperCommandGoal()
        goal.command.position = OPENED_POS
        goal.command.max_effort = -1.0
        self._client.send_goal(goal)
        self._client.wait_for_result()
        print("Gripper opened.")

    def close_gripper(self, max_effort=MAX_EFFORT):
        print("Closing gripper...")
        goal = GripperCommandGoal()
        goal.command.position = CLOSED_POS
        goal.command.max_effort = max_effort
        self._client.send_goal(goal)
        self._client.wait_for_result()
        print("Gripper closed.")

    def load_and_execute(self):
        if not os.path.exists(POSES_FILE):
            print("No saved file found.")
            return

        with open(POSES_FILE, 'rb') as f:
            self.poses = pickle.load(f)

        for i, pose in enumerate(self.poses):
            print(f"\n[Step {i}] Moving to pose relative to {pose['frame']}...")
            print(f"Position: {pose['position']}")
            print(f"Orientation: {pose['orientation']}")
            print(f"Gripper: {pose['gripper']}")

            try:
                self.tf_listener.waitForTransform('base_link', pose['frame'], rospy.Time(0), rospy.Duration(2.0))
                (trans, rot) = self.tf_listener.lookupTransform('base_link', pose['frame'], rospy.Time(0))

                target_pose = PoseStamped()
                target_pose.header.frame_id = 'base_link'
                target_pose.pose.position.x = pose['position'][0] + trans[0]
                target_pose.pose.position.y = pose['position'][1] + trans[1]
                target_pose.pose.position.z = pose['position'][2] + trans[2]
                target_pose.pose.orientation.x = pose['orientation'][0]
                target_pose.pose.orientation.y = pose['orientation'][1]
                target_pose.pose.orientation.z = pose['orientation'][2]
                target_pose.pose.orientation.w = pose['orientation'][3]

                print("Sending goal to MoveIt...")
                self.group.set_pose_target(target_pose)
                success = self.group.go(wait=True)
                self.group.stop()
                self.group.clear_pose_targets()
                print(f"MoveIt go() success: {success}")

                if not success:
                    print("⚠️ MoveIt failed to reach the pose. Skipping to next.")
                    continue

                if pose['gripper'] == 'open':
                    self.open_gripper()
                elif pose['gripper'] == 'close':
                    self.close_gripper()

            except Exception as e:
                print(f"Execution failed: {e}")


def main():
    PBDSystem()
    roscpp_shutdown()

if __name__ == '__main__':
    main()
