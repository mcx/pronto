cmake_minimum_required(VERSION 3.12)
project(pronto_solo12)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # add_compile_options(-Wall)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pronto_ros REQUIRED)
find_package(pronto_quadruped_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(urdf REQUIRED)

# add_executable(pinocchio_test_node  src/pinocchio_node.cpp)

add_library(${PROJECT_NAME}  SHARED src/feet_jacobians.cpp
                                    src/forward_kinematics.cpp
                                    src/dynamics.cpp
                                    src/feet_contact_forces.cpp
                                    src/pronto_mammal_utilities.cpp
                                    src/pinocchio_pronto_quadruped_model.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  ${PROJECT_NAME}
  "pronto_ros"
  "pronto_quadruped_ros"
  "rclcpp"
  "sensor_msgs"
  "pinocchio"
  "nav_msgs"
)
# ament_target_dependencies(
#   pinocchio_test_node
#   "pronto_ros"
#   "pronto_quadruped_ros"
#   "rclcpp"
#   "sensor_msgs"
#   "pinocchio"
#   "nav_msgs"
#   "urdf"
# )
# # ament_export_targets(my_libraryTargets HAS_LIBRARY_TARGET)
# # ament_export_dependencies(some_dependency)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS pronto_solo12
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# install(TARGETS
#   pinocchio_test_node
#   DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies("pronto_ros"
"pronto_quadruped_ros"
"rclcpp"
"sensor_msgs"
"pinocchio"
"nav_msgs")


# add_executable(pronto_estimator_node src/pronto_estimator.cpp)
# target_include_directories(pronto_estimator_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(pronto_estimator_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# target_link_libraries(pronto_estimator_node ${PROJECT_NAME})

# install(TARGETS pronto_estimator_node
#   DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ament_target_dependencies(
#   pronto_estimator_node
#   "pronto_ros"
#   "pronto_quadruped_ros"
#   "rclcpp"
#   "sensor_msgs"
# )

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
