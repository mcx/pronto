#ifndef PRONTO_UTILS__VISIBILITY_CONTROL_H_
#define PRONTO_UTILS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PRONTO_UTILS_EXPORT __attribute__ ((dllexport))
    #define PRONTO_UTILS_IMPORT __attribute__ ((dllimport))
  #else
    #define PRONTO_UTILS_EXPORT __declspec(dllexport)
    #define PRONTO_UTILS_IMPORT __declspec(dllimport)
  #endif
  #ifdef PRONTO_UTILS_BUILDING_LIBRARY
    #define PRONTO_UTILS_PUBLIC PRONTO_UTILS_EXPORT
  #else
    #define PRONTO_UTILS_PUBLIC PRONTO_UTILS_IMPORT
  #endif
  #define PRONTO_UTILS_PUBLIC_TYPE PRONTO_UTILS_PUBLIC
  #define PRONTO_UTILS_LOCAL
#else
  #define PRONTO_UTILS_EXPORT __attribute__ ((visibility("default")))
  #define PRONTO_UTILS_IMPORT
  #if __GNUC__ >= 4
    #define PRONTO_UTILS_PUBLIC __attribute__ ((visibility("default")))
    #define PRONTO_UTILS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PRONTO_UTILS_PUBLIC
    #define PRONTO_UTILS_LOCAL
  #endif
  #define PRONTO_UTILS_PUBLIC_TYPE
#endif

#endif  // PRONTO_UTILS__VISIBILITY_CONTROL_H_
