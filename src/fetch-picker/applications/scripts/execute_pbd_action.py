#!/usr/bin/env python

import rospy
import json
import os
from geometry_msgs.msg import PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
from robot_controllers_msgs.msg import QueryControllerStatesGoal, ControllerState, QueryControllerStatesAction
import actionlib
import robot_api
from robot_api import Arm, Gripper
import tf2_ros
import tf2_geometry_msgs
import rosbag
from sensor_msgs.msg import PointCloud2
import numpy as np

class PbDActionExecutor:
    def __init__(self):
        self.arm = Arm()
        self.gripper = Gripper()
        self.markers = []
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Subscribe to AR marker topic
        self.marker_sub = rospy.Subscriber(
            '/ar_pose_marker',
            AlvarMarkers,
            self.marker_callback
        )
        
        # Create controller state client
        self.controller_client = actionlib.SimpleActionClient(
            '/query_controller_states',
            QueryControllerStatesAction
        )
        self.controller_client.wait_for_server()

    def marker_callback(self, msg):
        """AR marker callback function"""
        self.markers = msg.markers

    def enable_arm(self):
        """Enable arm controller"""
        goal = QueryControllerStatesGoal()
        state = ControllerState()
        state.name = 'arm_controller/follow_joint_trajectory'
        state.state = ControllerState.RUNNING
        goal.updates.append(state)
        self.controller_client.send_goal(goal)
        self.controller_client.wait_for_result()

    def load_action(self, action_name):
        """Load action from bag file"""
        file_path = os.path.join('/fetch_ws/data', f"{action_name}.p")
        try:
            with rosbag.Bag(file_path, 'r') as bag:
                for topic, msg, t in bag.read_messages(topics=['/action_poses']):
                    # 将点云数据转换回姿态列表
                    poses = []
                    points = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 8)
                    for point in points:
                        x, y, z, intensity, qx, qy, qz, qw = point
                        pose_info = {
                            'pose': {
                                'position': {'x': x, 'y': y, 'z': z},
                                'orientation': {'x': qx, 'y': qy, 'z': qz, 'w': qw}
                            },
                            'reference_frame': 'base_link',
                            'gripper_open': intensity > 127.5  # 如果强度值大于127.5，认为夹爪是打开的
                        }
                        poses.append(pose_info)
                    return {'poses': poses}
        except Exception as e:
            rospy.logerr(f"Failed to load action file: {e}")
            return None

    def transform_pose(self, pose_info):
        """Transform pose to base frame"""
        pose = PoseStamped()
        pose.header.frame_id = pose_info['reference_frame']
        pose.header.stamp = rospy.Time.now()
        
        # Set position and orientation
        pose.pose.position.x = pose_info['pose']['position']['x']
        pose.pose.position.y = pose_info['pose']['position']['y']
        pose.pose.position.z = pose_info['pose']['position']['z']
        pose.pose.orientation.x = pose_info['pose']['orientation']['x']
        pose.pose.orientation.y = pose_info['pose']['orientation']['y']
        pose.pose.orientation.z = pose_info['pose']['orientation']['z']
        pose.pose.orientation.w = pose_info['pose']['orientation']['w']

        try:
            # Transform to base frame
            transformed_pose = self.tf_buffer.transform(pose, 'base_link')
            return transformed_pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Pose transformation failed: {e}")
            return None

    def execute_action(self, action_name):
        """Execute action sequence"""
        # Load action
        action_data = self.load_action(action_name)
        if not action_data:
            rospy.logerr("Failed to load action data")
            return False

        # Enable arm controller
        self.enable_arm()
        rospy.loginfo("Arm controller enabled")

        # Execute each pose
        for i, pose_info in enumerate(action_data['poses']):
            rospy.loginfo(f"Executing pose {i+1}/{len(action_data['poses'])}")
            
            # Transform pose
            target_pose = self.transform_pose(pose_info)
            if not target_pose:
                rospy.logerr("Pose transformation failed, stopping execution")
                return False

            # 打印目标位置信息
            rospy.loginfo(f"Target position: x={target_pose.pose.position.x:.3f}, y={target_pose.pose.position.y:.3f}, z={target_pose.pose.position.z:.3f}")
            rospy.loginfo(f"Target orientation: x={target_pose.pose.orientation.x:.3f}, y={target_pose.pose.orientation.y:.3f}, z={target_pose.pose.orientation.z:.3f}, w={target_pose.pose.orientation.w:.3f}")

            # Move arm
            try:
                error = self.arm.move_to_pose(target_pose)
                if error is not None:
                    rospy.logerr(f"Failed to move arm: {error}")
                    rospy.logerr("Please check if the target position is reachable")
                    return False
                rospy.loginfo("Arm movement completed successfully")
            except Exception as e:
                rospy.logerr(f"Exception during arm movement: {e}")
                return False

            # Control gripper
            try:
                if pose_info['gripper_open']:
                    self.gripper.open()
                    rospy.loginfo("Opening gripper")
                else:
                    self.gripper.close()
                    rospy.loginfo("Closing gripper")
            except Exception as e:
                rospy.logerr(f"Failed to control gripper: {e}")
                return False

            # Wait for gripper action to complete
            rospy.sleep(1.0)

        rospy.loginfo("Action execution completed")
        return True

def main():
    rospy.init_node('execute_pbd_action')
    
    executor = PbDActionExecutor()
    
    # 直接执行pick_boces.p
    if executor.execute_action('pick_boces'):
        print("Action executed successfully!")
    else:
        print("Action execution failed!")

if __name__ == '__main__':
    main() 