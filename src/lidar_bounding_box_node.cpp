#include "lidar_bounding_box/lidar_bounding_box_node.hpp"

#include <chrono>
#include <Eigen/Dense>
#include <pcl/common/common.h>

using namespace std::chrono_literals;

LidarBoundingBoxNode::LidarBoundingBoxNode(const rclcpp::NodeOptions & options)
: Node("lidar_bounding_box_node", options)
{
  this->declare_parameter<std::string>("input_yolo_topic", "/yolo_result");
  this->declare_parameter<std::string>("input_camera_info_topic", "/camera_info");
  this->declare_parameter<std::string>("input_point_cloud_topic", "/point_cloud");
  this->declare_parameter<std::string>("output_bounding_box_topic", "/lidar_bounding_box");
  this->declare_parameter<std::string>("camera_frame_id", "camera_link");
  this->declare_parameter<std::string>("lidar_frame_id", "lidar_link");

  this->get_parameter("input_yolo_topic", input_yolo_topic_);
  this->get_parameter("input_camera_info_topic", input_camera_info_topic_);
  this->get_parameter("input_point_cloud_topic", input_point_cloud_topic_);
  this->get_parameter("output_bounding_box_topic", output_bounding_box_topic_);
  this->get_parameter("camera_frame_id", camera_frame_id_);
  this->get_parameter("lidar_frame_id", lidar_frame_id_);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.best_effort();

  // Initialize message_filters subscribers after parameters are loaded
  yolo_sub_.subscribe(this, input_yolo_topic_, qos.get_rmw_qos_profile());
  camera_info_sub_.subscribe(this, input_camera_info_topic_, qos.get_rmw_qos_profile());
  point_cloud_sub_.subscribe(this, input_point_cloud_topic_, qos.get_rmw_qos_profile());

  // Create ApproximateTimeSynchronizer
  synchronizer_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(10), yolo_sub_, camera_info_sub_, point_cloud_sub_);

  synchronizer_->registerCallback(
    std::bind(&LidarBoundingBoxNode::synchronized_callback, this,
              std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  bounding_box_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(output_bounding_box_topic_, qos);
}

void LidarBoundingBoxNode::synchronized_callback(
  const ultralytics_ros::msg::YoloResult::ConstSharedPtr& yolo_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_msg,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& point_cloud_msg)
{
  RCLCPP_INFO(this->get_logger(), "[Synchronized Callback] Received synchronized data.");

  // Populate camera_k_matrix_ from camera_info_msg
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      camera_k_matrix_(i, j) = camera_info_msg->k[i * 3 + j];
    }
  }
  RCLCPP_INFO(this->get_logger(), "[Synchronized Callback] Camera K matrix updated.");

  // Get transform from lidar frame to camera frame using the timestamp of the synchronized messages
  geometry_msgs::msg::TransformStamped transform_lidar_to_camera;
  try {
    transform_lidar_to_camera = tf_buffer_->lookupTransform(
      camera_frame_id_, lidar_frame_id_, yolo_msg->header.stamp); // Use YOLO msg timestamp for TF lookup
    RCLCPP_INFO(this->get_logger(), "[Synchronized Callback] Successfully looked up TF from %s to %s at timestamp %f.",
      lidar_frame_id_.c_str(), camera_frame_id_.c_str(), rclcpp::Time(yolo_msg->header.stamp).seconds());
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "[Synchronized Callback] Could not transform %s to %s: %s. Skipping processing.",
      lidar_frame_id_.c_str(), camera_frame_id_.c_str(), ex.what());
    return;
  }

  // Convert ROS PointCloud2 to PCL PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*point_cloud_msg, *pcl_cloud);
  RCLCPP_INFO(this->get_logger(), "[Synchronized Callback] Converted PointCloud2 to PCL PointCloud with %zu points.", pcl_cloud->points.size());

  visualization_msgs::msg::MarkerArray marker_array;
  int marker_id = 0;

  RCLCPP_INFO(this->get_logger(), "[Synchronized Callback] Processing %zu 2D detections.", yolo_msg->detections.detections.size());
  for (const auto & detection_2d : yolo_msg->detections.detections)
  {
    RCLCPP_INFO(this->get_logger(), "[Synchronized Callback] Processing detection ID: %d, BBox Center: (%.2f, %.2f), Size: (%.2f, %.2f)",
      marker_id, detection_2d.bbox.center.position.x, detection_2d.bbox.center.position.y, detection_2d.bbox.size_x, detection_2d.bbox.size_y);

    // Project 3D points to 2D image plane and check if they fall within the 2D bounding box
    pcl::PointCloud<pcl::PointXYZ>::Ptr points_in_bbox(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto & point : pcl_cloud->points)
    {
      // Transform point from lidar frame to camera frame
      Eigen::Vector4d point_lidar(point.x, point.y, point.z, 1.0);
      Eigen::Matrix4d transform_matrix = Eigen::Affine3d(tf2::transformToEigen(transform_lidar_to_camera)).matrix();
      Eigen::Vector4d point_camera = transform_matrix * point_lidar;

      // Project to 2D image plane
      Eigen::Vector3d point_camera_3d(point_camera.x(), point_camera.y(), point_camera.z());
      Eigen::Vector3d projected_point = camera_k_matrix_ * point_camera_3d;

      if (projected_point.z() > 0) // Ensure point is in front of camera
      {
        double u = projected_point.x() / projected_point.z();
        double v = projected_point.y() / projected_point.z();

        // Check if projected point is within 2D bounding box
        if (u >= detection_2d.bbox.center.position.x - detection_2d.bbox.size_x / 2.0 &&
            u <= detection_2d.bbox.center.position.x + detection_2d.bbox.size_x / 2.0 &&
            v >= detection_2d.bbox.center.position.y - detection_2d.bbox.size_y / 2.0 &&
            v <= detection_2d.bbox.center.position.y + detection_2d.bbox.size_y / 2.0)
        {
          points_in_bbox->push_back(point);
        }
      }
    }

    if (points_in_bbox->points.empty()) {
      RCLCPP_INFO(this->get_logger(), "[Synchronized Callback] No LiDAR points found within 2D bounding box for detection ID: %d. Skipping.", marker_id);
      continue; // No points found in this bounding box
    }
    RCLCPP_INFO(this->get_logger(), "[Synchronized Callback] Found %zu LiDAR points within 2D bounding box for detection ID: %d.", points_in_bbox->points.size(), marker_id);

    // Calculate 3D bounding box from points_in_bbox
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*points_in_bbox, min_pt, max_pt);
    RCLCPP_INFO(this->get_logger(), "[Synchronized Callback] Calculated 3D BBox for detection ID: %d. Min: (%.2f, %.2f, %.2f), Max: (%.2f, %.2f, %.2f)",
      marker_id, min_pt.x, min_pt.y, min_pt.z, max_pt.x, max_pt.y, max_pt.z);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = lidar_frame_id_; // Bounding box in lidar frame
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "lidar_bounding_box";
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = (min_pt.x + max_pt.x) / 2.0;
    marker.pose.position.y = (min_pt.y + max_pt.y) / 2.0;
    marker.pose.position.z = (min_pt.z + max_pt.z) / 2.0;
    marker.pose.orientation.w = 1.0; // No rotation for now, axis-aligned bounding box

    marker.scale.x = max_pt.x - min_pt.x;
    marker.scale.y = max_pt.y - min_pt.y;
    marker.scale.z = max_pt.z - min_pt.z;

    // Ensure minimum scale to avoid issues with empty or flat boxes
    if (marker.scale.x < 0.01) marker.scale.x = 0.01;
    if (marker.scale.y < 0.01) marker.scale.y = 0.01;
    if (marker.scale.z < 0.01) marker.scale.z = 0.01;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;

    marker.lifetime = rclcpp::Duration::from_seconds(0.1);

    marker_array.markers.push_back(marker);

    // Add text marker for class name
    if (!detection_2d.results.empty()) {
      visualization_msgs::msg::Marker text_marker;
      text_marker.header.frame_id = lidar_frame_id_;
      text_marker.header.stamp = this->get_clock()->now();
      text_marker.ns = "lidar_bounding_box_text";
      text_marker.id = marker_id++; // Use a new ID for the text marker
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.action = visualization_msgs::msg::Marker::ADD;

      text_marker.pose.position.x = marker.pose.position.x;
      text_marker.pose.position.y = marker.pose.position.y;
      text_marker.pose.position.z = max_pt.z + 0.2; // Position text slightly above the bounding box
      text_marker.pose.orientation.w = 1.0;

      text_marker.scale.z = 0.3; // Height of the text

      text_marker.color.r = 1.0;
      text_marker.color.g = 1.0;
      text_marker.color.b = 1.0;
      text_marker.color.a = 1.0; // Fully opaque

      text_marker.text = detection_2d.results[0].hypothesis.class_id; // Class name from YOLO detection
      text_marker.lifetime = rclcpp::Duration::from_seconds(0.1);

      marker_array.markers.push_back(text_marker);
    }
  }

  bounding_box_publisher_->publish(marker_array);
  RCLCPP_INFO(this->get_logger(), "[Synchronized Callback] Published MarkerArray with %zu markers.", marker_array.markers.size());
}
