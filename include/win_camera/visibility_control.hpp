#ifndef WIN_CAMERA__VISIBILITY_CONTROL_H_
#define WIN_CAMERA__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define WIN_CAMERA_EXPORT __attribute__ ((dllexport))
    #define WIN_CAMERA_IMPORT __attribute__ ((dllimport))
  #else
    #define WIN_CAMERA_EXPORT __declspec(dllexport)
    #define WIN_CAMERA_IMPORT __declspec(dllimport)
  #endif
  #ifdef WIN_CAMERA_BUILDING_LIBRARY
    #define WIN_CAMERA_PUBLIC WIN_CAMERA_EXPORT
  #else
    #define WIN_CAMERA_PUBLIC WIN_CAMERA_IMPORT
  #endif
  #define WIN_CAMERA_PUBLIC_TYPE WIN_CAMERA_PUBLIC
  #define WIN_CAMERA_LOCAL
#else
  #define WIN_CAMERA_EXPORT __attribute__ ((visibility("default")))
  #define WIN_CAMERA_IMPORT
  #if __GNUC__ >= 4
    #define WIN_CAMERA_PUBLIC __attribute__ ((visibility("default")))
    #define WIN_CAMERA_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define WIN_CAMERA_PUBLIC
    #define WIN_CAMERA_LOCAL
  #endif
  #define WIN_CAMERA_PUBLIC_TYPE
#endif

#endif  // WIN_CAMERA__VISIBILITY_CONTROL_H_