// 包含点云裁剪功能的头文件
#include "perception/crop.h"
// 包含点云降采样功能的头文件
#include "perception/downsample.h"
// 包含ROS核心功能的头文件
#include "ros/ros.h"
// 包含点云消息类型的头文件
#include "sensor_msgs/PointCloud2.h"

// 主函数
int main(int argc, char** argv) {
  // 初始化ROS节点，节点名为"point_cloud_demo"
  ros::init(argc, argv, "point_cloud_demo");
  // 创建节点句柄
  ros::NodeHandle nh;
  // 创建裁剪后点云的发布者，话题名为"cropped_cloud"，队列大小为1，使用latch模式
  ros::Publisher crop_pub = nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
  // 创建降采样后点云的发布者，话题名为"downsampled_cloud"，队列大小为1，使用latch模式
  ros::Publisher downsample_pub = nh.advertise<sensor_msgs::PointCloud2>("downsampled_cloud", 1, true);

  // 创建点云裁剪器对象
  perception::Cropper cropper(crop_pub);
  // 创建点云降采样器对象
  perception::Downsampler downsampler(downsample_pub);

  // 创建订阅者，订阅"cloud_in"话题，处理函数为cropper的Callback方法
  ros::Subscriber sub_crop = nh.subscribe("cloud_in", 1, &perception::Cropper::Callback, &cropper);
  // 创建订阅者，订阅"cropped_cloud"话题，处理函数为downsampler的Callback方法
  ros::Subscriber sub_downsample = nh.subscribe("cropped_cloud", 1, &perception::Downsampler::Callback, &downsampler);

  // 进入ROS消息循环
  ros::spin();
  // 程序正常退出
  return 0;
}