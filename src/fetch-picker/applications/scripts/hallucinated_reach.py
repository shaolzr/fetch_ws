#!/usr/bin/env python

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped, Quaternion
import tf2_ros
import tf2_geometry_msgs
import robot_api
import tf_conversions

def wait_for_time():
    """Wait for simulated time to begin."""
    while rospy.Time().now().to_sec() == 0:
        pass


class ArTagReader(object):
    def __init__(self):
        self.markers = []
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.callback)

    def callback(self, msg):
        self.markers = msg.markers


def main():
    rospy.init_node('hallucinated_reach')
    wait_for_time()

    # Raise torso
    torso = robot_api.Torso()
    rospy.loginfo("Waiting for torso action server...")
    torso.set_height(0)  # full height (adjust based on robot model)
    rospy.loginfo("Torso motion complete.")

    # Move to start pose
    arm = robot_api.Arm()
    rospy.loginfo("Moving to start pose...")
    start = PoseStamped()
    start.header.frame_id = 'base_link'
    start.pose.position.x = 0.5
    start.pose.position.y = 0.0
    start.pose.position.z = 0.75
    start.pose.orientation = Quaternion(0, 0, 0, 1)  # identity orientation
    arm.move_to_pose(start)
    rospy.loginfo("Start pose reached.")

    # Initialize marker reader and TF buffer
    rospy.loginfo("Initializing AR marker detection...")
    reader = ArTagReader()
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rospy.sleep(1.0)  # wait for markers to appear
    rospy.loginfo(f"Number of markers detected: {len(reader.markers)}")

    if len(reader.markers) == 0:
        rospy.logerr("No AR markers detected! Please make sure:")
        rospy.logerr("1. AR marker is in camera view")
        rospy.logerr("2. Camera is properly calibrated")
        rospy.logerr("3. ar_track_alvar node is running")
        return

    for marker in reader.markers:
        rospy.loginfo(f"Found marker ID: {marker.id}")
        try:
            # Convert marker.pose.pose (usually in camera frame) to base_link
            marker_pose = PoseStamped()
            marker_pose.header = marker.header
            marker_pose.pose = marker.pose.pose

            # Transform pose to base_link
            rospy.loginfo("Transforming marker pose to base_link frame...")
            transformed = tf_buffer.transform(marker_pose, 'base_link', timeout=rospy.Duration(1.0))

            # Replace orientation with identity quaternion (gripper facing down)
            transformed.pose.orientation = Quaternion(0, 0, 0, 1)

            # Attempt to move
            rospy.loginfo(f"Attempting to move to marker {marker.id}...")
            error = arm.move_to_pose(transformed)
            if error is None:
                rospy.loginfo('Moved to marker {}'.format(marker.id))
                return
            else:
                rospy.logwarn('Failed to move to marker {}'.format(marker.id))
        except Exception as e:
            rospy.logwarn("TF Transform failed for marker {}: {}".format(marker.id, str(e)))

    rospy.logerr("Failed to move to any AR marker!")


if __name__ == '__main__':
    main()