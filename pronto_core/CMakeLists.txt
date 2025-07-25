cmake_minimum_required(VERSION 3.8)
project(pronto_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wall)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON) # solution 3
# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(ament_cmake_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)

add_library(pronto_core SHARED  src/rbis.cpp
                                src/rotations.cpp
                                src/rigidbody.cpp
                                src/ins_module.cpp
                                src/scan_matcher_module.cpp
                                src/rbis_update_interface.cpp
                                src/state_est.cpp
                                src/update_history.cpp
                                src/indexed_meas_module.cpp
                                src/init_message_module.cpp
                                src/pose_meas_module.cpp
                                src/vicon_module.cpp
)

target_compile_features(pronto_core PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(pronto_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  pronto_core
  "Eigen3"
  "rclcpp"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(pronto_core PRIVATE "PRONTO_CORE_BUILDING_LIBRARY")

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies("Eigen3" "rclcpp")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS pronto_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
  pronto_core
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
