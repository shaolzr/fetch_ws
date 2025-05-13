import rosbag
import rospy
from sensor_msgs.msg import PointCloud2

class MockCamera(object):
    """A MockCamera reads saved point clouds."""
    def __init__(self):
        pass

    def read_cloud(self, path):
        """Returns the sensor_msgs/PointCloud2 in the given bag file."""
        bag = rosbag.Bag(path)
        for topic, msg, t in bag.read_messages():
            bag.close()
            return msg
        bag.close()
        return None


if __name__ == '__main__':
    rospy.init_node('mock_camera_test')
    camera = MockCamera()
    cloud = camera.read_cloud('~/data/table.bag')
    if cloud is not None:
        rospy.loginfo("Successfully read point cloud from bag.")
    else:
        rospy.logwarn("No point cloud found in bag.")