#include "perception/segmentation.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_cloud_demo");
  ros::NodeHandle nh;
  ros::Publisher segment_pub =
      nh.advertise<sensor_msgs::PointCloud2>("segment_cloud", 1, true);
  perception::Segmenter segmenter(segment_pub);
  ros::Subscriber sub =
      nh.subscribe("cloud_in", 1, &perception::Segmenter::Callback, &segmenter);
  ros::spin();
  return 0;
} 