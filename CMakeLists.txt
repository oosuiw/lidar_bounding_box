cmake_minimum_required(VERSION 3.8)
project(lidar_bounding_box)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ultralytics_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(${PROJECT_NAME}_node_lib src/lidar_bounding_box_node.cpp)
add_executable(${PROJECT_NAME}_main src/lidar_bounding_box_main.cpp)

target_include_directories(${PROJECT_NAME}_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(${PROJECT_NAME}_main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}_node_lib
  rclcpp
  rclcpp_components
  ultralytics_ros
  visualization_msgs
  sensor_msgs
  tf2_ros
  tf2_eigen
  pcl_conversions
)

ament_target_dependencies(${PROJECT_NAME}_main
  rclcpp
  rclcpp_components
  ultralytics_ros
  visualization_msgs
  sensor_msgs
  tf2_ros
  tf2_eigen
  pcl_conversions
)

target_link_libraries(${PROJECT_NAME}_node_lib
  ${PCL_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_main
  ${PROJECT_NAME}_node_lib
  ${PCL_LIBRARIES}
)

install(TARGETS
  ${PROJECT_NAME}_node_lib
  ${PROJECT_NAME}_main
  DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()