#ifndef PRONTO_QUADRUPED_ROS__VISIBILITY_CONTROL_H_
#define PRONTO_QUADRUPED_ROS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PRONTO_QUADRUPED_ROS_EXPORT __attribute__ ((dllexport))
    #define PRONTO_QUADRUPED_ROS_IMPORT __attribute__ ((dllimport))
  #else
    #define PRONTO_QUADRUPED_ROS_EXPORT __declspec(dllexport)
    #define PRONTO_QUADRUPED_ROS_IMPORT __declspec(dllimport)
  #endif
  #ifdef PRONTO_QUADRUPED_ROS_BUILDING_LIBRARY
    #define PRONTO_QUADRUPED_ROS_PUBLIC PRONTO_QUADRUPED_ROS_EXPORT
  #else
    #define PRONTO_QUADRUPED_ROS_PUBLIC PRONTO_QUADRUPED_ROS_IMPORT
  #endif
  #define PRONTO_QUADRUPED_ROS_PUBLIC_TYPE PRONTO_QUADRUPED_ROS_PUBLIC
  #define PRONTO_QUADRUPED_ROS_LOCAL
#else
  #define PRONTO_QUADRUPED_ROS_EXPORT __attribute__ ((visibility("default")))
  #define PRONTO_QUADRUPED_ROS_IMPORT
  #if __GNUC__ >= 4
    #define PRONTO_QUADRUPED_ROS_PUBLIC __attribute__ ((visibility("default")))
    #define PRONTO_QUADRUPED_ROS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PRONTO_QUADRUPED_ROS_PUBLIC
    #define PRONTO_QUADRUPED_ROS_LOCAL
  #endif
  #define PRONTO_QUADRUPED_ROS_PUBLIC_TYPE
#endif

#endif  // PRONTO_QUADRUPED_ROS__VISIBILITY_CONTROL_H_
