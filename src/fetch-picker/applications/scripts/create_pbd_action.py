#!/usr/bin/env python

import rospy
import json
import os
import numpy as np
from geometry_msgs.msg import PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
from robot_controllers_msgs.msg import QueryControllerStatesGoal, ControllerState, QueryControllerStatesAction
import actionlib
import robot_api
from robot_api import Arm, Gripper
import tf2_ros
import tf2_geometry_msgs
import rosbag
from sensor_msgs.msg import PointCloud2, PointField
import struct

# 设置TF日志级别为ERROR，忽略WARN级别的消息
rospy.set_param('/rosout/logger_level', 'ERROR')

class PbDActionCreator:
    def __init__(self):
        self.arm = Arm()
        self.gripper = Gripper()
        self.markers = []
        self.poses = []
        self.is_simulation = rospy.get_param('/use_sim_time', False)
        
        # Create TF listener with buffer size and cache time
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Create controller state client
        self.controller_client = actionlib.SimpleActionClient(
            '/query_controller_states',
            QueryControllerStatesAction
        )
        self.controller_client.wait_for_server()
        
        # Subscribe to AR marker topic
        self.marker_sub = rospy.Subscriber(
            '/ar_pose_marker',
            AlvarMarkers,
            self.marker_callback
        )

    def create_point_cloud(self, poses):
        """从姿态列表创建点云数据"""
        points = []
        for pose in poses:
            # 获取位置信息
            x = pose['pose']['position']['x']
            y = pose['pose']['position']['y']
            z = pose['pose']['position']['z']
            
            # 获取方向信息（四元数）
            qx = pose['pose']['orientation']['x']
            qy = pose['pose']['orientation']['y']
            qz = pose['pose']['orientation']['z']
            qw = pose['pose']['orientation']['w']
            
            # 将夹爪状态转换为强度值
            intensity = 255 if pose['gripper_open'] else 0
            
            # 添加点
            points.append([x, y, z, intensity, qx, qy, qz, qw])
        
        return np.array(points, dtype=np.float32)

    def create_pointcloud2_msg(self, points):
        """创建PointCloud2消息"""
        header = rospy.Header()
        header.frame_id = 'base_link'
        header.stamp = rospy.Time.now()

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.FLOAT32, 1),
            PointField('qx', 16, PointField.FLOAT32, 1),
            PointField('qy', 20, PointField.FLOAT32, 1),
            PointField('qz', 24, PointField.FLOAT32, 1),
            PointField('qw', 28, PointField.FLOAT32, 1)
        ]

        return PointCloud2(
            header=header,
            height=1,
            width=len(points),
            is_dense=True,
            is_bigendian=False,
            fields=fields,
            point_step=32,  # 8 fields * 4 bytes
            row_step=32 * len(points),
            data=points.tobytes()
        )

    def save_action(self, action_name):
        """Save action sequence to bag file"""
        if not self.poses:
            rospy.logerr("No poses to save!")
            return False
            
        # 创建点云数据
        points = self.create_point_cloud(self.poses)
        cloud_msg = self.create_pointcloud2_msg(points)
        
        # 使用绝对路径
        save_dir = '/fetch_ws/data'
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
            
        # 保存到bag文件
        bag_path = os.path.join(save_dir, 'pick_boces.p')
        with rosbag.Bag(bag_path, 'w') as bag:
            bag.write('/action_poses', cloud_msg)
            
        rospy.loginfo(f"Action saved to: {os.path.abspath(bag_path)}")
        return True

    def marker_callback(self, msg):
        """AR marker callback function"""
        self.markers = msg.markers

    def relax_arm(self):
        """Relax the arm"""
        if not self.is_simulation:
            goal = QueryControllerStatesGoal()
            state = ControllerState()
            state.name = 'arm_controller/follow_joint_trajectory'
            state.state = ControllerState.STOPPED
            goal.updates.append(state)
            self.controller_client.send_goal(goal)
            self.controller_client.wait_for_result()

    def enable_arm(self):
        """Enable arm controller"""
        goal = QueryControllerStatesGoal()
        state = ControllerState()
        state.name = 'arm_controller/follow_joint_trajectory'
        state.state = ControllerState.RUNNING
        goal.updates.append(state)
        self.controller_client.send_goal(goal)
        self.controller_client.wait_for_result()

    def get_current_pose(self):
        """Get current end-effector pose"""
        try:
            # Get current time
            current_time = rospy.Time.now()
            
            # Get transform from base_link to wrist_roll_link
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'wrist_roll_link',
                current_time,
                rospy.Duration(1.0)  # Wait up to 1 second for transform
            )
            
            # Create PoseStamped message
            pose = PoseStamped()
            pose.header.frame_id = 'base_link'
            pose.header.stamp = current_time
            
            # Set position
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            
            # Set orientation
            pose.pose.orientation = transform.transform.rotation
            
            return pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Failed to get current pose: {e}")
            return None

    def save_current_pose(self, reference_frame='base_link', marker_id=None):
        """Save current pose"""
        # Get current arm pose
        current_pose = self.get_current_pose()
        if current_pose is None:
            rospy.logerr("Failed to get current arm pose")
            return
        
        # Record gripper state - assume it's open if it's not fully closed
        gripper_state = True  # Default to open state
        
        # Save pose information
        pose_info = {
            'pose': {
                'position': {
                    'x': current_pose.pose.position.x,
                    'y': current_pose.pose.position.y,
                    'z': current_pose.pose.position.z
                },
                'orientation': {
                    'x': current_pose.pose.orientation.x,
                    'y': current_pose.pose.orientation.y,
                    'z': current_pose.pose.orientation.z,
                    'w': current_pose.pose.orientation.w
                }
            },
            'reference_frame': reference_frame,
            'marker_id': marker_id,
            'gripper_open': gripper_state
        }
        
        self.poses.append(pose_info)
        rospy.loginfo(f"Pose saved, reference frame: {reference_frame}, marker ID: {marker_id}")

def main():
    rospy.init_node('create_pbd_action')
    
    creator = PbDActionCreator()
    
    if not creator.is_simulation:
        creator.relax_arm()
        rospy.loginfo("Arm relaxed, you can now move it manually")
    
    while not rospy.is_shutdown():
        print("\n=== PbD Action Creation System ===")
        print("1. Save current pose")
        print("2. Open gripper")
        print("3. Close gripper")
        print("4. Save action")
        print("5. Exit")
        
        choice = input("Please select an option (1-5): ")
        
        if choice == '1':
            print("\nSelect reference frame:")
            print("1. Base frame (base_link)")
            print("2. AR marker")
            frame_choice = input("Please select (1-2): ")
            
            if frame_choice == '1':
                creator.save_current_pose()
            elif frame_choice == '2':
                if not creator.markers:
                    print("No AR markers detected!")
                    continue
                print("\nAvailable AR markers:")
                for marker in creator.markers:
                    print(f"ID: {marker.id}")
                marker_id = input("Please enter marker ID: ")
                creator.save_current_pose(reference_frame='ar_marker_' + marker_id, marker_id=int(marker_id))
                
        elif choice == '2':
            creator.gripper.open()
            print("Gripper opened")
            
        elif choice == '3':
            creator.gripper.close()
            print("Gripper closed")
            
        elif choice == '4':
            if creator.save_action('pick_boces'):
                print("Action saved as: pick_boces.p")
                creator.poses = []  # Clear saved poses
                
        elif choice == '5':
            break
            
        else:
            print("Invalid choice, please try again")
    
    if not creator.is_simulation:
        creator.enable_arm()
        rospy.loginfo("Arm controller re-enabled")

if __name__ == '__main__':
    main()
