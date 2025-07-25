#ifndef PRONTO_CORE__VISIBILITY_CONTROL_H_
#define PRONTO_CORE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PRONTO_CORE_EXPORT __attribute__ ((dllexport))
    #define PRONTO_CORE_IMPORT __attribute__ ((dllimport))
  #else
    #define PRONTO_CORE_EXPORT __declspec(dllexport)
    #define PRONTO_CORE_IMPORT __declspec(dllimport)
  #endif
  #ifdef PRONTO_CORE_BUILDING_LIBRARY
    #define PRONTO_CORE_PUBLIC PRONTO_CORE_EXPORT
  #else
    #define PRONTO_CORE_PUBLIC PRONTO_CORE_IMPORT
  #endif
  #define PRONTO_CORE_PUBLIC_TYPE PRONTO_CORE_PUBLIC
  #define PRONTO_CORE_LOCAL
#else
  #define PRONTO_CORE_EXPORT __attribute__ ((visibility("default")))
  #define PRONTO_CORE_IMPORT
  #if __GNUC__ >= 4
    #define PRONTO_CORE_PUBLIC __attribute__ ((visibility("default")))
    #define PRONTO_CORE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PRONTO_CORE_PUBLIC
    #define PRONTO_CORE_LOCAL
  #endif
  #define PRONTO_CORE_PUBLIC_TYPE
#endif

#endif  // PRONTO_CORE__VISIBILITY_CONTROL_H_
