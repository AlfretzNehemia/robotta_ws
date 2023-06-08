#ifndef __ROBOTTA_HARDWARE__ROBOTTA_HARDWARE_COMPILER_H__
#define __ROBOTTA_HARDWARE__ROBOTTA_HARDWARE_COMPILER_H__

#if defined _WIN32 || defined __CYGWIN__
#   ifdef __GNUC__
#       define ROBOTTA_HARDWARE_EXPORT __attribute__((dllexport))
#       define ROBOTTA_HARDWARE_IMPORT __attribute__((dllimport))
#   else
#       define ROBOTTA_HARDWARE_EXPORT __declspec(dllexport)
#       define ROBOTTA_HARDWARE_IMPORT __declspec(dllimport)
#   endif
#   ifdef ROBOTTA_HARDWARE_BUILDING_DLL
#       define ROBOTTA_HARDWARE_PUBLIC ROBOTTA_HARDWARE_EXPORT
#   else
#       define ROBOTTA_HARDWARE_PUBLIC ROBOTTA_HARDWARE_IMPORT
#   endif
#   define ROBOTTA_HARDWARE_PUBLIC_TYPE ROBOTTA_HARDWARE_PUBLIC
#   define ROBOTTA_HARDWARE_LOCAL
#else
#   define ROBOTTA_HARDWARE_EXPORT __attribute__((visibility("default")))
#   define ROBOTTA_HARDWARE_IMPORT
#   if __GNUC__ >= 4
#       define ROBOTTA_HARDWARE_PUBLIC __attribute__((visibility("default")))
#       define ROBOTTA_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#   else
#       define ROBOTTA_HARDWARE_PUBLIC
#       define ROBOTTA_HARDWARE_LOCAL
#   endif
#   define ROBOTTA_HARDWARE_PUBLIC_TYPE
#endif

#endif // __ROBOTTA_HARDWARE__ROBOTTA_HARDWARE_COMPILER_H__