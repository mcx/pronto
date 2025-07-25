cmake_minimum_required(VERSION 3.8)
project(pronto_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
  # add_compile_options(-Wall)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(ament_cmake_ros REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} SHARED  src/Filter.cpp
                                    src/HeavyLowPassFilter.cpp
                                    #src/SignalTap.cpp
                                    src/DataFileLogger.cpp
                                    src/BipolarSchmittTrigger.cpp
                                    src/MedianFilter.cpp
                                    src/RateChange.cpp
                                    src/TrapezoidalInt.cpp
                                    src/DistributedDiff.cpp
                                    src/NumericalDiff.cpp
                                    src/ExpireTimer.cpp
                                    src/SchmittTrigger.cpp
                                    src/BipolarSchmittTrigger.cpp
                                    src/Gaussian.cpp)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  ${PROJECT_NAME}
  "Eigen3"
)


# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "PRONTO_UTILS_BUILDING_LIBRARY")

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies("Eigen3")

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

add_library(kalman_filter SHARED  src/alpha_filter.cpp
                                  src/kalman_filter.cpp
                                  src/simple_kalman_filter.cpp)

target_compile_features(kalman_filter PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(kalman_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  kalman_filter
  "Eigen3"
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS kalman_filter
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(backlash_filter SHARED  src/backlash_filter.cpp
                                    src/torque_adjustment.cpp)

target_compile_features(backlash_filter PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(backlash_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(backlash_filter kalman_filter) # ament_target_dependencies doesn't work in this case

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS backlash_filter
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


target_compile_definitions(kalman_filter PRIVATE "PRONTO_UTILS_BUILDING_LIBRARY")
target_compile_definitions(backlash_filter PRIVATE "PRONTO_UTILS_BUILDING_LIBRARY")

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
  ${PROJECT_NAME} kalman_filter backlash_filter
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
