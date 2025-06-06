// 包含裁剪功能的头文件
#include "perception/crop.h"
// 包含PCL和ROS消息转换功能的头文件
#include <pcl_conversions/pcl_conversions.h>
// 包含PCL点云裁剪盒滤波器的头文件
#include <pcl/filters/crop_box.h>
// 包含PCL点类型定义的头文件
#include <pcl/point_types.h>
// 包含PCL点云数据结构的头文件
#include <pcl/point_cloud.h>
// 包含PCL通用功能的头文件
#include <pcl/common/common.h>
// 包含Eigen矩阵运算库的头文件
#include <Eigen/Dense>

// 定义perception命名空间
namespace perception {

// 定义带RGB颜色的点类型别名
typedef pcl::PointXYZRGB PointC;
// 定义带RGB颜色的点云类型别名
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

// 构造函数，初始化发布者
Cropper::Cropper(const ros::Publisher& pub) : pub_(pub) {}

// 点云回调函数，处理接收到的点云消息
void Cropper::Callback(const sensor_msgs::PointCloud2& msg) {
  // 创建新的点云指针
  PointCloudC::Ptr cloud(new PointCloudC());
  // 将ROS消息转换为PCL点云格式
  pcl::fromROSMsg(msg, *cloud);
  // 输出点云中的点数
  ROS_INFO("Got point cloud with %ld points", cloud->size());

  // 声明裁剪盒边界参数变量
  double min_x, min_y, min_z, max_x, max_y, max_z;
  // 从参数服务器读取裁剪盒x方向最小值，默认0.3
  ros::param::param("crop_min_x", min_x, 0.3);
  // 从参数服务器读取裁剪盒y方向最小值，默认-1.0
  ros::param::param("crop_min_y", min_y, -1.0);
  // 从参数服务器读取裁剪盒z方向最小值，默认0.5
  ros::param::param("crop_min_z", min_z, 0.5);
  // 从参数服务器读取裁剪盒x方向最大值，默认0.9
  ros::param::param("crop_max_x", max_x, 0.9);
  // 从参数服务器读取裁剪盒y方向最大值，默认1.0
  ros::param::param("crop_max_y", max_y, 1.0);
  // 从参数服务器读取裁剪盒z方向最大值，默认1.5
  ros::param::param("crop_max_z", max_z, 1.5);

  // 创建裁剪盒的最小点坐标向量
  Eigen::Vector4f min_pt(min_x, min_y, min_z, 1);
  // 创建裁剪盒的最大点坐标向量
  Eigen::Vector4f max_pt(max_x, max_y, max_z, 1);

  // 创建用于存储裁剪后点云的指针
  PointCloudC::Ptr cropped_cloud(new PointCloudC());
  // 创建裁剪盒滤波器对象
  pcl::CropBox<PointC> crop;
  // 设置输入点云
  crop.setInputCloud(cloud);
  // 设置裁剪盒的最小点
  crop.setMin(min_pt);
  // 设置裁剪盒的最大点
  crop.setMax(max_pt);
  // 执行裁剪操作，结果存储在cropped_cloud中
  crop.filter(*cropped_cloud);
  // 输出裁剪后的点数
  ROS_INFO("Cropped to %ld points", cropped_cloud->size());

  // 创建用于存储点云边界值的向量
  Eigen::Vector4f min_pt2, max_pt2;
  // 计算裁剪后点云在xyz三个方向上的最小和最大坐标
  pcl::getMinMax3D(*cropped_cloud, min_pt2, max_pt2);
  // 输出x方向的最小和最大坐标
  ROS_INFO("min: %f, max: %f", min_pt2.x(), max_pt2.x());

  // 创建用于发布的ROS点云消息
  sensor_msgs::PointCloud2 msg_out;
  // 将PCL点云转换为ROS消息格式
  pcl::toROSMsg(*cropped_cloud, msg_out);
  // 保持原始消息的时间戳和坐标系信息
  msg_out.header = msg.header;
  // 发布裁剪后的点云消息
  pub_.publish(msg_out);
}

}  // 结束perception命名空间