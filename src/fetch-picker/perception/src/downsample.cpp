#include "perception/downsample.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <Eigen/Dense>

namespace perception {
typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<PointC> PointCloudC;

Downsampler::Downsampler(const ros::Publisher& pub) : pub_(pub) {}

void Downsampler::Callback(const sensor_msgs::PointCloud2& msg) {
  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud);

  PointCloudC::Ptr downsampled_cloud(new PointCloudC());
  pcl::VoxelGrid<PointC> vox;
  vox.setInputCloud(cloud);
  double voxel_size;
  ros::param::param("voxel_size", voxel_size, 0.01);
  vox.setLeafSize(voxel_size, voxel_size, voxel_size);
  vox.filter(*downsampled_cloud);

  // 统计最小值、最大值和中心
  Eigen::Vector4f min_pt, max_pt;
  pcl::getMinMax3D(*downsampled_cloud, min_pt, max_pt);
  float center_x = (min_pt.x() + max_pt.x()) / 2.0;
  float center_y = (min_pt.y() + max_pt.y()) / 2.0;
  float center_z = (min_pt.z() + max_pt.z()) / 2.0;
  // ROS_INFO("min: (%f, %f, %f), max: (%f, %f, %f), center: (%f, %f, %f)",
  //          min_pt.x(), min_pt.y(), min_pt.z(),
  //          max_pt.x(), max_pt.y(), max_pt.z(),
  //          center_x, center_y, center_z);

  sensor_msgs::PointCloud2 msg_out;
  pcl::toROSMsg(*downsampled_cloud, msg_out);
  pub_.publish(msg_out);
}
}