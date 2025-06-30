#ifndef LIDAR_BOUNDING_BOX_NODE_HPP_
#define LIDAR_BOUNDING_BOX_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "ultralytics_ros/msg/yolo_result.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_eigen/tf2_eigen.hpp"

#include <functional> // Required for std::function and std::bind
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class LidarBoundingBoxNode : public rclcpp::Node
{
public:
  explicit LidarBoundingBoxNode(const rclcpp::NodeOptions & options);

private:
  void synchronized_callback(
    const ultralytics_ros::msg::YoloResult::ConstSharedPtr& yolo_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& point_cloud_msg);

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bounding_box_publisher_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  message_filters::Subscriber<ultralytics_ros::msg::YoloResult> yolo_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> point_cloud_sub_;

  typedef message_filters::sync_policies::ApproximateTime<
    ultralytics_ros::msg::YoloResult,
    sensor_msgs::msg::CameraInfo,
    sensor_msgs::msg::PointCloud2
  > SyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;

  std::string input_yolo_topic_;
  std::string input_camera_info_topic_;
  std::string input_point_cloud_topic_;
  std::string output_bounding_box_topic_;
  std::string camera_frame_id_;
  std::string lidar_frame_id_;

  Eigen::Matrix3d camera_k_matrix_;
  double max_distance_threshold_; 
  double max_bbox_size_; 
  double sor_stddev_mul_thresh_; 
  int sor_mean_k_;
};

#endif  // LIDAR_BOUNDING_BOX_NODE_HPP_