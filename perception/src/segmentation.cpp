#include "perception/segmentation.h"

#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"

#include "pcl/common/common.h"
#include "pcl/common/angles.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/segmentation/extract_clusters.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "simple_grasping/shape_extraction.h"
#include "shape_msgs/SolidPrimitive.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception {

void SegmentVerticalSurface(PointCloudC::Ptr cloud,
                            pcl::PointIndices::Ptr indices,
                            pcl::ModelCoefficients::Ptr coeff) {
  pcl::PointIndices indices_internal;
  pcl::SACSegmentation<PointC> seg;
  seg.setOptimizeCoefficients(true);
  // Search for a plane perpendicular to some axis (specified below).
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  // Set the distance to the plane for a point to be an inlier.
  double distance_thresh;
  ros::param::param("segment_vertical_distance_thresh", distance_thresh, 0.02);
  seg.setDistanceThreshold(distance_thresh);
  seg.setInputCloud(cloud);

  // Make sure that the plane is perpendicular to X-axis, 10 degree tolerance.
  Eigen::Vector3f axis;
  axis << 1, 0, 0;
  seg.setAxis(axis);
  seg.setEpsAngle(pcl::deg2rad(10.0));

  // coeff contains the coefficients of the plane:
  // ax + by + cz + d = 0
  seg.segment(indices_internal, *coeff);

  /*
  // Likely will want to do something like this to get a PC for things on the whiteboard.
  double distance_above_plane;
  ros::param::param("distance_above_plane", distance_above_plane, 0.005);

  // Build custom indices that ignores points above the plane.
  for (size_t i = 0; i < cloud->size(); ++i) {
    const PointC& pt = cloud->points[i];
    float val = coeff->values[0] * pt.x + coeff->values[1] * pt.y +
                coeff->values[2] * pt.z + coeff->values[3];
    if (val <= distance_above_plane) {
      indices->indices.push_back(i);
    }
  }
  */

  // Comment this out
  *indices = indices_internal;
  if (indices->indices.size() == 0) {
    ROS_ERROR("Unable to find surface.");
    return;
  }
}

void SegmentSurface(PointCloudC::Ptr cloud,
                    pcl::PointIndices::Ptr indices,
                    pcl::ModelCoefficients::Ptr coeff) {
  pcl::PointIndices indices_internal;
  pcl::SACSegmentation<PointC> seg;
  seg.setOptimizeCoefficients(true);
  // Search for a plane perpendicular to some axis (specified below).
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  // Set the distance to the plane for a point to be an inlier.
  seg.setDistanceThreshold(0.01);
  seg.setInputCloud(cloud);

  // Make sure that the plane is perpendicular to Z-axis, 10 degree tolerance.
  Eigen::Vector3f axis;
  axis << 0, 0, 1;
  seg.setAxis(axis);
  seg.setEpsAngle(pcl::deg2rad(10.0));

  // coeff contains the coefficients of the plane:
  // ax + by + cz + d = 0
  seg.segment(indices_internal, *coeff);

  double distance_above_plane;
  ros::param::param("distance_above_plane", distance_above_plane, 0.005);

  // Build custom indices that ignores points above the plane.
  for (size_t i = 0; i < cloud->size(); ++i) {
    const PointC& pt = cloud->points[i];
    float val = coeff->values[0] * pt.x + coeff->values[1] * pt.y +
                coeff->values[2] * pt.z + coeff->values[3];
    if (val <= distance_above_plane) {
      indices->indices.push_back(i);
    }
  }

  // Comment this out
  //*indices = indices_internal;
  if (indices->indices.size() == 0) {
    ROS_ERROR("Unable to find surface.");
    return;
  }
}

void GetAxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                               geometry_msgs::Pose* pose,
                               geometry_msgs::Vector3* dimensions) {
  PointC min_pcl;
  PointC max_pcl;
  pcl::getMinMax3D<PointC>(*cloud, min_pcl, max_pcl);
  float centerx = (max_pcl.x/2.0) + (min_pcl.x/2.0);
  float centery = (max_pcl.y/2.0) + (min_pcl.y/2.0);
  float centerz = (max_pcl.z/2.0) + (min_pcl.z/2.0);
  float dimx = max_pcl.x - min_pcl.x;
  float dimy = max_pcl.y - min_pcl.y;
  float dimz = max_pcl.z - min_pcl.z;

  pose->position.x = centerx;
  pose->position.y = centery;
  pose->position.z = centerz;
  dimensions->x = dimx;
  dimensions->y = dimy;
  dimensions->z = dimz;
}

void SegmentSurfaceObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           pcl::PointIndices::Ptr surface_indices,
                           std::vector<pcl::PointIndices>* object_indices) {
  pcl::ExtractIndices<PointC> extract;
  pcl::PointIndices::Ptr above_surface_indices(new pcl::PointIndices());
  extract.setInputCloud(cloud);
  extract.setIndices(surface_indices);
  extract.setNegative(true);
  extract.filter(above_surface_indices->indices);

  ROS_INFO("There are %ld points above the table", above_surface_indices->indices.size());

  double cluster_tolerance;
  int min_cluster_size, max_cluster_size;
  ros::param::param("ec_cluster_tolerance", cluster_tolerance, 0.01);
  ros::param::param("ec_min_cluster_size", min_cluster_size, 10);
  ros::param::param("ec_max_cluster_size", max_cluster_size, 10000);

  pcl::EuclideanClusterExtraction<PointC> euclid;
  euclid.setInputCloud(cloud);
  euclid.setIndices(above_surface_indices);
  euclid.setClusterTolerance(cluster_tolerance);
  euclid.setMinClusterSize(min_cluster_size); euclid.setMaxClusterSize(max_cluster_size);
  euclid.extract(*object_indices);

  // Find the size of the smallest and the largest object,
  // where size = number of points in the cluster
  size_t min_size = std::numeric_limits<size_t>::max();
  size_t max_size = std::numeric_limits<size_t>::min();
  for (size_t i = 0; i < object_indices->size(); ++i) {
    // TODO: implement this
    size_t cluster_size = (*object_indices)[i].indices.size();
    if (cluster_size < min_size) {
      min_size = cluster_size;
    }
    if (cluster_size > max_size) {
      max_size = cluster_size;
    }
  }

  ROS_INFO("Found %ld objects, min size: %ld, max size: %ld",
           object_indices->size(), min_size, max_size);
}

Segmenter::Segmenter(const ros::Publisher& surface_points_pub,
                     const ros::Publisher& marker_pub,
                     const ros::Publisher& above_surface_pub)
    : surface_points_pub_(surface_points_pub),
      marker_pub_(marker_pub),
      above_surface_pub_(above_surface_pub),
      pose_pub_(),
      tfl_() {}

Segmenter::Segmenter(const ros::Publisher& surface_points_pub,
                     const ros::Publisher& marker_pub,
                     const ros::Publisher& above_surface_pub,
                     const ros::Publisher& plane_pub)
    : surface_points_pub_(surface_points_pub),
      marker_pub_(marker_pub),
      above_surface_pub_(above_surface_pub),
      pose_pub_(plane_pub),
      tfl_() {}

void Segmenter::Callback(const sensor_msgs::PointCloud2& msg) {
  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud);

  pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices());
  pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients());
  SegmentSurface(cloud, table_inliers, coeffs);

  PointCloudC::Ptr subset_cloud(new PointCloudC);
  pcl::ExtractIndices<PointC> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(table_inliers);
  extract.filter(*subset_cloud);

  sensor_msgs::PointCloud2 msg_out;
  pcl::toROSMsg(*subset_cloud, msg_out);
  surface_points_pub_.publish(msg_out);

  visualization_msgs::Marker table_marker;
  table_marker.ns = "table";
  table_marker.header.frame_id = "base_link";
  table_marker.type = visualization_msgs::Marker::CUBE;

  PointCloudC::Ptr extract_out(new PointCloudC());
  shape_msgs::SolidPrimitive shape;
  geometry_msgs::Pose table_pose;
  simple_grasping::extractShape(*subset_cloud, coeffs, *extract_out, shape,
                                table_pose);

  if (shape.type == shape_msgs::SolidPrimitive::BOX) {
    table_marker.pose = table_pose;
    table_marker.scale.x = shape.dimensions[0];
    table_marker.scale.y = shape.dimensions[1];
    table_marker.scale.z = shape.dimensions[2];
    table_marker.pose.position.z -= table_marker.scale.z;
  }

  table_marker.color.r = 1;
  table_marker.color.a = 0.8;
  marker_pub_.publish(table_marker);

  std::vector<pcl::PointIndices> object_indices;
  SegmentSurfaceObjects(cloud, table_inliers, &object_indices);
  extract.setNegative(true);
  extract.filter(*subset_cloud);
  pcl::toROSMsg(*subset_cloud, msg_out);
  above_surface_pub_.publish(msg_out);

  for (size_t i = 0; i < object_indices.size(); ++i) {
    // Reify indices into a point cloud of the object.
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    *indices = object_indices[i];
    PointCloudC::Ptr object_cloud(new PointCloudC());
    extract.setNegative(false);
    extract.setIndices(indices);
    extract.filter(*object_cloud);

    // Publish a bounding box around it.
    visualization_msgs::Marker object_marker;
    object_marker.ns = "objects";
    object_marker.id = i;
    object_marker.header.frame_id = "base_link";
    object_marker.type = visualization_msgs::Marker::CUBE;
    GetAxisAlignedBoundingBox(object_cloud, &object_marker.pose,
                              &object_marker.scale);
    object_marker.color.g = 1;
    object_marker.color.a = 0.3;
    marker_pub_.publish(object_marker);
  }
}


void Segmenter::VerticalCallback(const sensor_msgs::PointCloud2& msg) {
  ROS_INFO("VerticalCallback");
  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud);

  tfl_.waitForTransform("base_link", cloud->header.frame_id,
          ros::Time(0), ros::Duration(0.5));

  tf::StampedTransform transform;                                                       
  try {                                                                                 
    tfl_.lookupTransform("base_link", cloud->header.frame_id,                    
                                ros::Time(0), transform);                               
  } catch (tf::LookupException& e) {                                                    
    std::cerr << e.what() << std::endl;                                                 
    return;                                                                           
  } catch (tf::ExtrapolationException& e) {                                             
    std::cerr << e.what() << std::endl;                                                 
    return;                                                                           
  }

  PointCloudC::Ptr cloud_out(new PointCloudC());                                                   
  pcl_ros::transformPointCloud<PointC>("base_link", *cloud, *cloud_out, tfl_);

  pcl::PointIndices::Ptr whiteboard_inliers(new pcl::PointIndices());
  pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients());
  SegmentVerticalSurface(cloud_out, whiteboard_inliers, coeffs);

  if (whiteboard_inliers->indices.size() == 0) {
    ROS_INFO("Oh no");
    return;
  }

  PointCloudC::Ptr subset_cloud(new PointCloudC);
  pcl::ExtractIndices<PointC> extract;
  extract.setInputCloud(cloud_out);
  extract.setIndices(whiteboard_inliers);
  extract.filter(*subset_cloud);

  sensor_msgs::PointCloud2 msg_out;
  pcl::toROSMsg(*subset_cloud, msg_out);
  surface_points_pub_.publish(msg_out);

  PointCloudC::Ptr extract_out(new PointCloudC());
  shape_msgs::SolidPrimitive shape;
  geometry_msgs::Pose table_pose;
  simple_grasping::extractShape(*subset_cloud, coeffs, *extract_out, shape,
                                table_pose);
  pose_pub_.publish(table_pose);
}

}  // namespace perception
