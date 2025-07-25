cmake_minimum_required(VERSION 3.5)
project(pronto_msgs)

# Add support for C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FilterState.msg" "msg/IndexedMeasurement.msg"
  "msg/QuadrupedStance.msg" "msg/JointStateWithAcceleration.msg"
  "msg/ControllerFootContact.msg"
  "msg/QuadrupedForceTorqueSensors.msg"
  "msg/VelocityWithSigmaBounds.msg"
  "msg/VisualOdometryUpdate.msg"
  "msg/GPSData.msg" "msg/LidarOdometryUpdate.msg" "msg/BipedForceTorqueSensors.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs ADD_LINTER_TESTS)

# set(INCLUDE_DIRS ${ament_cmake_INCLUDE_DIRS})
# include_directories(${INCLUDE_DIRS})

# set(LIBRARY_DIRS ${ament_cmake_LIBRARY_DIRS})

# link_directories(${LIBRARY_DIRS})

# set(LIBS ${ament_cmake_LIBRARIES})

ament_export_dependencies(rosidl_default_runtime)
# ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
