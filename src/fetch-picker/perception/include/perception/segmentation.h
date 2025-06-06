#ifndef PERCEPTION_SEGMENTATION_H_
#define PERCEPTION_SEGMENTATION_H_

#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/segmentation/extract_clusters.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"

// 类型别名
typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception {

// 获取轴对齐边界框
void GetAxisAlignedBoundingBox(PointCloudC::Ptr cloud,
                             geometry_msgs::Pose* pose,
                             geometry_msgs::Vector3* dimensions);

// 分割箱子中的物体
void SegmentBinObjects(PointCloudC::Ptr cloud,
                      std::vector<pcl::PointIndices>* indices);

class Segmenter {
 public:
  Segmenter(const ros::Publisher& points_pub);
  void Callback(const sensor_msgs::PointCloud2& msg);

 private:
  ros::Publisher points_pub_;
};

}  // namespace perception

#endif  // PERCEPTION_SEGMENTATION_H_ 