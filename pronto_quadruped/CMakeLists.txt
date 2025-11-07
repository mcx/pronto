cmake_minimum_required(VERSION 3.8)
project(pronto_quadruped)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wall)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(ament_cmake_ros REQUIRED)
find_package(pronto_quadruped_commons REQUIRED)
find_package(pronto_utils REQUIRED)
find_package(pronto_core REQUIRED)

add_library(${PROJECT_NAME} SHARED  src/LegOdometer.cpp
                                    src/DynamicStanceEstimator.cpp
                                    src/ForceSensorStanceEstimator.cpp
                                    src/FootSensorStanceDetector.cpp
                                    src/StanceEstimator.cpp
                                    src/FlexEstimator.cpp
                                    src/DataLogger.cpp
                                    src/ImuBiasLock.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  ${PROJECT_NAME}
  "pronto_quadruped_commons"
  "pronto_utils"
  "pronto_core"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "PRONTO_QUADRUPED_BUILDING_LIBRARY")
# set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON) # solution 2

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies("pronto_quadruped_commons"
  "pronto_utils"
  "pronto_core")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
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
  pronto_quadruped
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
