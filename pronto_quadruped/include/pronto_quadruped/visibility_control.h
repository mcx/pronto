#ifndef PRONTO_QUADRUPED__VISIBILITY_CONTROL_H_
#define PRONTO_QUADRUPED__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PRONTO_QUADRUPED_EXPORT __attribute__ ((dllexport))
    #define PRONTO_QUADRUPED_IMPORT __attribute__ ((dllimport))
  #else
    #define PRONTO_QUADRUPED_EXPORT __declspec(dllexport)
    #define PRONTO_QUADRUPED_IMPORT __declspec(dllimport)
  #endif
  #ifdef PRONTO_QUADRUPED_BUILDING_LIBRARY
    #define PRONTO_QUADRUPED_PUBLIC PRONTO_QUADRUPED_EXPORT
  #else
    #define PRONTO_QUADRUPED_PUBLIC PRONTO_QUADRUPED_IMPORT
  #endif
  #define PRONTO_QUADRUPED_PUBLIC_TYPE PRONTO_QUADRUPED_PUBLIC
  #define PRONTO_QUADRUPED_LOCAL
#else
  #define PRONTO_QUADRUPED_EXPORT __attribute__ ((visibility("default")))
  #define PRONTO_QUADRUPED_IMPORT
  #if __GNUC__ >= 4
    #define PRONTO_QUADRUPED_PUBLIC __attribute__ ((visibility("default")))
    #define PRONTO_QUADRUPED_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PRONTO_QUADRUPED_PUBLIC
    #define PRONTO_QUADRUPED_LOCAL
  #endif
  #define PRONTO_QUADRUPED_PUBLIC_TYPE
#endif

#endif  // PRONTO_QUADRUPED__VISIBILITY_CONTROL_H_
