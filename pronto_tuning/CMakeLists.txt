cmake_minimum_required(VERSION 3.8)
project(pronto_tuning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pronto_msgs REQUIRED)
find_package(pi3hat_moteus_int_msgs REQUIRED)
find_package(mocap4r2_msgs REQUIRED)

add_executable(tuning_node src/tuning_node.cpp)
add_executable(command_node src/command_node.cpp)

ament_target_dependencies(tuning_node std_srvs rosbag2_cpp rclcpp geometry_msgs gazebo_msgs Eigen3 pronto_msgs pi3hat_moteus_int_msgs mocap4r2_msgs)
ament_target_dependencies(command_node std_srvs rosbag2_cpp rclcpp geometry_msgs gazebo_msgs Eigen3 pi3hat_moteus_int_msgs mocap4r2_msgs)
include_directories(include)
install(TARGETS
  tuning_node command_node
  DESTINATION lib/${PROJECT_NAME})
install(
    DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
  )

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

ament_package()
