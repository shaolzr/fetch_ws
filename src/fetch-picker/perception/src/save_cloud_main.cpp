// 包含ROS核心功能头文件，提供ROS基本功能支持
#include "ros/ros.h"
// 包含点云消息类型头文件，用于处理点云数据
#include "sensor_msgs/PointCloud2.h"
// 包含PCL点云变换功能头文件，用于点云坐标变换
#include "pcl_ros/transforms.h"
// 包含TF变换监听器头文件，用于获取坐标变换
#include "tf/transform_listener.h"
// 包含ROS bag操作头文件，用于保存数据
#include "rosbag/bag.h"
// 包含标准输入输出流头文件，用于控制台输出
#include <iostream>
// 包含字符串处理头文件，用于处理文件名
#include <string>

// 包含ROS核心功能头文件（重复包含，可以删除）
#include "ros/ros.h"

// 定义打印使用说明的函数
void print_usage() { 
  // 打印程序功能说明，说明程序将点云保存到bag文件
  std::cout << "Saves a point cloud on head_camera/depth_registered/points to "
               "NAME.bag in the current directory."
            << std::endl;
  // 打印使用示例，说明如何运行程序
  std::cout << "Usage: rosrun perception save_cloud NAME" << std::endl;
}

// 定义主函数，程序入口点
int main(int argc, char** argv) { 
  // 初始化ROS节点，设置节点名为"save_cloud_main"
  ros::init(argc, argv, "save_cloud_main");
  // 检查命令行参数数量是否小于2（程序名+文件名）
  if (argc < 2) { 
    // 如果参数不足，打印使用说明
    print_usage();
    // 返回错误码1表示程序异常退出
    return 1;
  } 
  // 获取命令行参数中的文件名
  std::string name(argv[1]);
  // 打印欢迎信息，显示文件名
  std::cout << "Hello, " << name << std::endl;

  // 等待并接收点云消息，设置超时时间
  sensor_msgs::PointCloud2ConstPtr cloud =
      ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
          "head_camera/depth_registered/points");
  // 检查是否成功接收到点云数据
  if (!cloud) {
    // 如果未收到点云，打印错误信息
    std::cerr << "未收到点云消息，程序退出。" << std::endl;
    // 返回错误码1表示程序异常退出
    return 1;
  }
  // 打印成功接收点云的信息
  std::cout << "已收到点云消息。" << std::endl;

  // 创建TF变换监听器对象
  tf::TransformListener tf_listener;
  // 等待从点云坐标系到base_link的变换可用，设置5秒超时
  tf_listener.waitForTransform("base_link", cloud->header.frame_id,
                               ros::Time(0), ros::Duration(5.0));
  // 创建变换对象，用于存储坐标变换
  tf::StampedTransform transform;
  try {
    // 获取从点云坐标系到base_link的变换
    tf_listener.lookupTransform("base_link", cloud->header.frame_id,
                                ros::Time(0), transform);
  } catch (tf::LookupException& e) {
    // 处理查找变换失败的情况
    std::cerr << e.what() << std::endl;
    // 返回错误码1表示程序异常退出
    return 1;
  } catch (tf::ExtrapolationException& e) {
    // 处理变换外推失败的情况
    std::cerr << e.what() << std::endl;
    // 返回错误码1表示程序异常退出
    return 1;
  }
  // 创建输出点云对象，用于存储变换后的点云
  sensor_msgs::PointCloud2 cloud_out;
  // 将点云从原始坐标系变换到base_link坐标系
  pcl_ros::transformPointCloud("base_link", transform, *cloud, cloud_out);
  // 打印变换成功信息
  std::cout << "点云已变换到 base_link 坐标系。" << std::endl;

  // 构建输出文件名，添加.bag后缀
  std::string filename(name + ".bag");
  // 创建ROS bag对象
  rosbag::Bag bag;
  // 以写入模式打开bag文件
  bag.open(filename, rosbag::bagmode::Write);
  // 将变换后的点云写入bag文件，使用当前时间戳
  bag.write("head_camera/depth_registered/points", ros::Time::now(), cloud_out);
  // 关闭bag文件
  bag.close();
  // 打印保存成功信息，显示文件名
  std::cout << "点云已保存到 " << filename << std::endl;

  // 返回0表示程序正常退出
  return 0;
}
