#ifndef MTC_TUTORIAL__VISIBILITY_CONTROL_H_
#define MTC_TUTORIAL__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MTC_TUTORIAL_EXPORT __attribute__ ((dllexport))
    #define MTC_TUTORIAL_IMPORT __attribute__ ((dllimport))
  #else
    #define MTC_TUTORIAL_EXPORT __declspec(dllexport)
    #define MTC_TUTORIAL_IMPORT __declspec(dllimport)
  #endif
  #ifdef MTC_TUTORIAL_BUILDING_DLL
    #define MTC_TUTORIAL_PUBLIC MTC_TUTORIAL_EXPORT
  #else
    #define MTC_TUTORIAL_PUBLIC MTC_TUTORIAL_IMPORT
  #endif
  #define MTC_TUTORIAL_PUBLIC_TYPE MTC_TUTORIAL_PUBLIC
  #define MTC_TUTORIAL_LOCAL
#else
  #define MTC_TUTORIAL_EXPORT __attribute__ ((visibility("default")))
  #define MTC_TUTORIAL_IMPORT
  #if __GNUC__ >= 4
    #define MTC_TUTORIAL_PUBLIC __attribute__ ((visibility("default")))
    #define MTC_TUTORIAL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MTC_TUTORIAL_PUBLIC
    #define MTC_TUTORIAL_LOCAL
  #endif
  #define MTC_TUTORIAL_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // MTC_TUTORIAL__VISIBILITY_CONTROL_H_
