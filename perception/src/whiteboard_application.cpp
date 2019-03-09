#include "perception/crop.h"
#include "perception/downsample.h"
#include "perception/segmentation.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Pose.h"
#include <visualization_msgs/Marker.h>
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "whiteboard_application");
  ros::NodeHandle nh;
  std::string cloud_in = "/head_camera/depth_registered/points";
  //std::string cloud_in = "/mock_point_cloud";
  
  ros::Publisher whiteboard_pub =
      nh.advertise<sensor_msgs::PointCloud2>("whiteboard_cloud", 1, true);
  ros::Publisher marker_pub =
      nh.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
  ros::Publisher above_surface_pub =
      nh.advertise<sensor_msgs::PointCloud2>("above_surface", 1, true);
  ros::Publisher plane_pub =
      nh.advertise<geometry_msgs::Pose>("plane_pose", 1, true);
  perception::Segmenter segmenter(whiteboard_pub, marker_pub, above_surface_pub, plane_pub);
  ros::Subscriber sub =
      nh.subscribe(cloud_in, 1, &perception::Segmenter::VerticalCallback, &segmenter);

  ros::spin();
  return 0;
}
