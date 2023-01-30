#ifndef CACC_CENTER__VISIBILITY_CONTROL_H_
#define CACC_CENTER__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CACC_CENTER_EXPORT __attribute__ ((dllexport))
    #define CACC_CENTER_IMPORT __attribute__ ((dllimport))
  #else
    #define CACC_CENTER_EXPORT __declspec(dllexport)
    #define CACC_CENTER_IMPORT __declspec(dllimport)
  #endif
  #ifdef CACC_CENTER_BUILDING_LIBRARY
    #define CACC_CENTER_PUBLIC CACC_CENTER_EXPORT
  #else
    #define CACC_CENTER_PUBLIC CACC_CENTER_IMPORT
  #endif
  #define CACC_CENTER_PUBLIC_TYPE CACC_CENTER_PUBLIC
  #define CACC_CENTER_LOCAL
#else
  #define CACC_CENTER_EXPORT __attribute__ ((visibility("default")))
  #define CACC_CENTER_IMPORT
  #if __GNUC__ >= 4
    #define CACC_CENTER_PUBLIC __attribute__ ((visibility("default")))
    #define CACC_CENTER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CACC_CENTER_PUBLIC
    #define CACC_CENTER_LOCAL
  #endif
  #define CACC_CENTER_PUBLIC_TYPE
#endif
#endif  // CACC_CENTER__VISIBILITY_CONTROL_H_
// Generated 30-Jan-2023 12:36:59
 