// generated from rosidl_generator_c/resource/rosidl_generator_c__visibility_control.h.in
// generated code does not contain a copyright notice

#ifndef CARLA_WAYPOINT_TYPES__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL_H_
#define CARLA_WAYPOINT_TYPES__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_GENERATOR_C_EXPORT_carla_waypoint_types __attribute__ ((dllexport))
    #define ROSIDL_GENERATOR_C_IMPORT_carla_waypoint_types __attribute__ ((dllimport))
  #else
    #define ROSIDL_GENERATOR_C_EXPORT_carla_waypoint_types __declspec(dllexport)
    #define ROSIDL_GENERATOR_C_IMPORT_carla_waypoint_types __declspec(dllimport)
  #endif
  #ifdef ROSIDL_GENERATOR_C_BUILDING_DLL_carla_waypoint_types
    #define ROSIDL_GENERATOR_C_PUBLIC_carla_waypoint_types ROSIDL_GENERATOR_C_EXPORT_carla_waypoint_types
  #else
    #define ROSIDL_GENERATOR_C_PUBLIC_carla_waypoint_types ROSIDL_GENERATOR_C_IMPORT_carla_waypoint_types
  #endif
#else
  #define ROSIDL_GENERATOR_C_EXPORT_carla_waypoint_types __attribute__ ((visibility("default")))
  #define ROSIDL_GENERATOR_C_IMPORT_carla_waypoint_types
  #if __GNUC__ >= 4
    #define ROSIDL_GENERATOR_C_PUBLIC_carla_waypoint_types __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_GENERATOR_C_PUBLIC_carla_waypoint_types
  #endif
#endif

#ifdef __cplusplus
}
#endif

#endif  // CARLA_WAYPOINT_TYPES__MSG__ROSIDL_GENERATOR_C__VISIBILITY_CONTROL_H_
