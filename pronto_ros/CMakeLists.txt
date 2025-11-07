cmake_minimum_required(VERSION 3.8)
project(pronto_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wall)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pronto_msgs REQUIRED)
find_package(pronto_core REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2)
find_package(rclcpp_lifecycle)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(pi3hat_moteus_int_msgs REQUIRED)

add_library(pronto_ros  SHARED  src/ins_ros_handler.cpp
                                # src/index_meas_ros_handler.cpp
                                src/init_message_ros_handler.cpp
                                src/scan_matcher_ros_handler.cpp
                                src/ros_frontend.cpp
                                src/pose_msg_ros_handler.cpp
                                src/pronto_ros_conversions.cpp
                                # src/visual_odometry_ros_handler.cpp
                                # src/lidar_odometry_ros_handler.cpp
                                #src/vicon_ros_handler.cpp
                                # src/gps_ros_handler.cpp
)

target_compile_features(pronto_ros PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(pronto_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  pronto_ros
  "Eigen3"
  "pronto_msgs"
  "pronto_core"
  "sensor_msgs"
  "tf2_ros"
  "tf2_eigen"
  "geometry_msgs"
  "tf2_geometry_msgs"
  "rclcpp"
  "nav_msgs"
  "gazebo_msgs"
  "rclcpp_lifecycle"
  "pi3hat_moteus_int_msgs"
)

set(LIBS ${Eigen3_LIBRARIES} ${ament_cmake_LIBRARIES}
  ${sensor_msgs_LIBRARIES} ${rclcpp_LIBRARIES}
  ${tf2_LIBRARIES}
  ${pronto_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES} ${pronto_core_LIBRARIES}
  ${tf2_ros_LIBRARIES} ${nav_msgs_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES} ${tf2_eigen_LIBRARIES})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(pronto_ros PRIVATE "PRONTO_ROS_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS pronto_ros
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
"Eigen3"
"pronto_msgs"
"pronto_core"
"sensor_msgs"
"tf2_ros"
"tf2_eigen"
"geometry_msgs"
"tf2_geometry_msgs"
"rclcpp"
"nav_msgs"
"gazebo_msgs"
"rclcpp_lifecycle"
"pi3hat_moteus_int_msgs")

add_executable(${PROJECT_NAME}_node src/pronto_ros_node.cpp)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${LIBS})

add_executable(joint_state_with_acceleration_republisher src/joint_state_with_acceleration_republisher.cpp)
target_include_directories(joint_state_with_acceleration_republisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(joint_state_with_acceleration_republisher
  ${PROJECT_NAME} ${LIBS})

install(TARGETS joint_state_with_acceleration_republisher pronto_ros_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
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

ament_export_include_directories(
  include
)
ament_export_libraries(
  pronto_ros
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
