#!/usr/bin/env python 

# 导入点云消息类型
from sensor_msgs.msg import PointCloud2
# 导入感知模块
import perception
# 导入ROS Python客户端库
import rospy


def wait_for_time(): 
    """等待ROS时间初始化完成"""
    while rospy.Time().now().to_sec() == 0:
        pass

    
def main():                                                                             
    # 初始化ROS节点，节点名为'publish_saved_cloud'
    rospy.init_node('publish_saved_cloud')
    # 等待时间初始化
    wait_for_time()                                                                     
    # 获取命令行参数
    argv = rospy.myargv()
    # 检查参数数量是否足够
    if len(argv) < 2:
        # 打印使用说明
        print ("Publishes a saved point cloud to a latched topic.")
        print ('Usage: rosrun applications publish_saved_cloud.py ~/cloud.bag')
        return
    # 获取bag文件路径
    path = argv[1]
    # 创建模拟相机对象
    camera = perception.MockCamera()
    # 从bag文件读取点云数据
    cloud = camera.read_cloud(path)
    
    # 检查是否成功读取点云
    if cloud is None:
        # 如果读取失败，打印错误信息并退出
        rospy.logerr('Could not load point cloud from {}'.format(path))
        return

    # 创建点云发布者，话题名为'mock_point_cloud'，队列大小为1
    pub = rospy.Publisher('mock_point_cloud', PointCloud2, queue_size=1)       
    # 创建发布频率对象，设置为2Hz
    rate = rospy.Rate(2)
    # 循环发布点云数据，直到节点被关闭
    while not rospy.is_shutdown():
        # 更新点云时间戳为当前时间
        cloud.header.stamp = rospy.Time.now()
        # 发布点云数据
        pub.publish(cloud)
        # 按照指定频率休眠
        rate.sleep()                                          
    
    
# 当脚本直接运行时执行main函数
if __name__ == '__main__':
    main()