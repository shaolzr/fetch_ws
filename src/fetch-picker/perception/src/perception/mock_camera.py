# 导入ROS bag操作库，用于读取bag文件
import rosbag
# 导入ROS Python客户端库
import rospy
# 导入点云消息类型
from sensor_msgs.msg import PointCloud2

class MockCamera(object):
    """模拟相机类，用于读取保存的点云数据"""
    def __init__(self):
        """初始化模拟相机对象"""
        pass

    def read_cloud(self, path):
        """从指定的bag文件中读取点云数据
        
        Args:
            path: bag文件的路径
            
        Returns:
            sensor_msgs/PointCloud2: 点云消息，如果文件为空则返回None
        """
        # 打开bag文件
        bag = rosbag.Bag(path)
        # 遍历bag文件中的所有消息
        for topic, msg, t in bag.read_messages():
            # 读取到消息后立即关闭文件
            bag.close()
            # 返回第一个点云消息
            return msg
        # 如果文件为空，关闭文件
        bag.close()
        # 返回None表示没有找到点云数据
        return None


# 当脚本直接运行时执行以下代码
if __name__ == '__main__':
    # 初始化ROS节点，节点名为'mock_camera_test'
    rospy.init_node('mock_camera_test')
    # 创建模拟相机对象
    camera = MockCamera()
    # 从指定路径读取点云数据
    cloud = camera.read_cloud('~/data/table.bag')
    # 检查是否成功读取点云
    if cloud is not None:
        # 如果成功读取，打印成功信息
        rospy.loginfo("Successfully read point cloud from bag.")
    else:
        # 如果读取失败，打印警告信息
        rospy.logwarn("No point cloud found in bag.")