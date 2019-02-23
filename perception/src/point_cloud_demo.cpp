#include "perception/crop.h"
#include "perception/downsample.h"
#include "perception/segmentation.h"
#include "sensor_msgs/PointCloud2.h"
#include "ros/ros.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_cloud_demo");
  ros::NodeHandle nh;
  std::string cloud_in = "mock_point_cloud";
  nh.getParam("cloud_in", cloud_in);

  ros::Publisher table_pub =
      nh.advertise<sensor_msgs::PointCloud2>("table_cloud", 1, true);
  perception::Segmenter segmenter(table_pub);
  ros::Subscriber sub =
      nh.subscribe(cloud_in, 1, &perception::Segmenter::Callback, &segmenter);

  ros::Publisher crop_pub =
      nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
  perception::Cropper cropper(crop_pub);
  ros::Subscriber sub_in =
      nh.subscribe(cloud_in, 1, &perception::Cropper::Callback, &cropper);

  ros::Publisher downsample_pub =
      nh.advertise<sensor_msgs::PointCloud2>("downsampled_cloud", 1, true);
  perception::Downsampler downsampler(downsample_pub);
  ros::Subscriber sub_cropped =
      nh.subscribe("cropped_cloud", 1, &perception::Downsampler::Callback, &downsampler);

  ros::spin();
  return 0;
}
