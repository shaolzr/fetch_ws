#include "perception/segmentation.h"

#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/common/common.h"
#include "visualization_msgs/Marker.h"

namespace perception {

void GetAxisAlignedBoundingBox(PointCloudC::Ptr cloud,
                             geometry_msgs::Pose* pose,
                             geometry_msgs::Vector3* dimensions) {
  PointC min_pt, max_pt;
  pcl::getMinMax3D(*cloud, min_pt, max_pt);

  // 计算中心点
  pose->position.x = (min_pt.x + max_pt.x) / 2;
  pose->position.y = (min_pt.y + max_pt.y) / 2;
  pose->position.z = (min_pt.z + max_pt.z) / 2;
  pose->orientation.w = 1;

  // 计算尺寸
  dimensions->x = max_pt.x - min_pt.x;
  dimensions->y = max_pt.y - min_pt.y;
  dimensions->z = max_pt.z - min_pt.z;
}

void SegmentBinObjects(PointCloudC::Ptr cloud, std::vector<pcl::PointIndices>* indices) {
  double cluster_tolerance;
  int min_cluster_size, max_cluster_size;
  ros::param::param("ec_cluster_tolerance", cluster_tolerance, 0.01);
  ros::param::param("ec_min_cluster_size", min_cluster_size, 10);
  ros::param::param("ec_max_cluster_size", max_cluster_size, 10000);

  pcl::EuclideanClusterExtraction<PointC> euclid;
  euclid.setInputCloud(cloud);
  euclid.setClusterTolerance(cluster_tolerance);
  euclid.setMinClusterSize(min_cluster_size);
  euclid.setMaxClusterSize(max_cluster_size);
  euclid.extract(*indices);

  // 找到最小和最大簇的大小
  size_t min_size = std::numeric_limits<size_t>::max();
  size_t max_size = std::numeric_limits<size_t>::min();
  for (size_t i = 0; i < indices->size(); ++i) {
    size_t cluster_size = (*indices)[i].indices.size();
    min_size = std::min(min_size, cluster_size);
    max_size = std::max(max_size, cluster_size);
  }

  ROS_INFO("Found %ld objects, min size: %ld, max size: %ld",
           indices->size(), min_size, max_size);
}

Segmenter::Segmenter(const ros::Publisher& points_pub)
    : points_pub_(points_pub) {}

void Segmenter::Callback(const sensor_msgs::PointCloud2& msg) {
  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud);

  std::vector<pcl::PointIndices> object_indices;
  SegmentBinObjects(cloud, &object_indices);

  // 为每个物体创建边界框
  for (size_t i = 0; i < object_indices.size(); ++i) {
    // 从索引创建点云
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    *indices = object_indices[i];
    PointCloudC::Ptr object_cloud(new PointCloudC());
    
    // 提取物体点云
    pcl::ExtractIndices<PointC> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(indices);
    extract.filter(*object_cloud);

    // 发布边界框
    visualization_msgs::Marker object_marker;
    object_marker.ns = "objects";
    object_marker.id = i;
    object_marker.header.frame_id = "base_link";
    object_marker.type = visualization_msgs::Marker::CUBE;
    GetAxisAlignedBoundingBox(object_cloud, &object_marker.pose,
                            &object_marker.scale);
    object_marker.color.g = 1;
    object_marker.color.a = 0.3;
    points_pub_.publish(object_marker);
  }
}

}  // namespace perception 