#include "rclcpp/rclcpp.hpp"
#include "lidar_bounding_box/lidar_bounding_box_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<LidarBoundingBoxNode>(rclcpp::NodeOptions());
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
