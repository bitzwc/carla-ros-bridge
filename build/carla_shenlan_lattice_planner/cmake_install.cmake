# Install script for directory: /home/zhangwencheng/Documents/autonomous_driving/projects/carla-ros-bridge/src/ros-bridge/carla_shenlan_projects/carla_shenlan_lattice_planner

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/zhangwencheng/Documents/autonomous_driving/projects/carla-ros-bridge/install/carla_shenlan_lattice_planner")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/carla_shenlan_lattice_planner/lattice_mpc_lateral_longitudinal" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/carla_shenlan_lattice_planner/lattice_mpc_lateral_longitudinal")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/carla_shenlan_lattice_planner/lattice_mpc_lateral_longitudinal"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/carla_shenlan_lattice_planner" TYPE EXECUTABLE FILES "/home/zhangwencheng/Documents/autonomous_driving/projects/carla-ros-bridge/build/carla_shenlan_lattice_planner/lattice_mpc_lateral_longitudinal")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/carla_shenlan_lattice_planner/lattice_mpc_lateral_longitudinal" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/carla_shenlan_lattice_planner/lattice_mpc_lateral_longitudinal")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/carla_shenlan_lattice_planner/lattice_mpc_lateral_longitudinal"
         OLD_RPATH "/home/zhangwencheng/Documents/autonomous_driving/projects/carla-ros-bridge/install/carla_msgs/lib:/opt/ros/foxy/lib:/opt/ros/foxy/lib/x86_64-linux-gnu:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/carla_shenlan_lattice_planner/lattice_mpc_lateral_longitudinal")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/carla_shenlan_lattice_planner/" TYPE DIRECTORY FILES
    "/home/zhangwencheng/Documents/autonomous_driving/projects/carla-ros-bridge/src/ros-bridge/carla_shenlan_projects/carla_shenlan_lattice_planner/launch"
    "/home/zhangwencheng/Documents/autonomous_driving/projects/carla-ros-bridge/src/ros-bridge/carla_shenlan_projects/carla_shenlan_lattice_planner/config"
    "/home/zhangwencheng/Documents/autonomous_driving/projects/carla-ros-bridge/src/ros-bridge/carla_shenlan_projects/carla_shenlan_lattice_planner/rviz"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/zhangwencheng/Documents/autonomous_driving/projects/carla-ros-bridge/build/carla_shenlan_lattice_planner/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/carla_shenlan_lattice_planner")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/zhangwencheng/Documents/autonomous_driving/projects/carla-ros-bridge/build/carla_shenlan_lattice_planner/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/carla_shenlan_lattice_planner")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/carla_shenlan_lattice_planner/environment" TYPE FILE FILES "/opt/ros/foxy/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/carla_shenlan_lattice_planner/environment" TYPE FILE FILES "/home/zhangwencheng/Documents/autonomous_driving/projects/carla-ros-bridge/build/carla_shenlan_lattice_planner/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/carla_shenlan_lattice_planner/environment" TYPE FILE FILES "/opt/ros/foxy/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/carla_shenlan_lattice_planner/environment" TYPE FILE FILES "/home/zhangwencheng/Documents/autonomous_driving/projects/carla-ros-bridge/build/carla_shenlan_lattice_planner/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/carla_shenlan_lattice_planner" TYPE FILE FILES "/home/zhangwencheng/Documents/autonomous_driving/projects/carla-ros-bridge/build/carla_shenlan_lattice_planner/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/carla_shenlan_lattice_planner" TYPE FILE FILES "/home/zhangwencheng/Documents/autonomous_driving/projects/carla-ros-bridge/build/carla_shenlan_lattice_planner/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/carla_shenlan_lattice_planner" TYPE FILE FILES "/home/zhangwencheng/Documents/autonomous_driving/projects/carla-ros-bridge/build/carla_shenlan_lattice_planner/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/carla_shenlan_lattice_planner" TYPE FILE FILES "/home/zhangwencheng/Documents/autonomous_driving/projects/carla-ros-bridge/build/carla_shenlan_lattice_planner/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/carla_shenlan_lattice_planner" TYPE FILE FILES "/home/zhangwencheng/Documents/autonomous_driving/projects/carla-ros-bridge/build/carla_shenlan_lattice_planner/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/zhangwencheng/Documents/autonomous_driving/projects/carla-ros-bridge/build/carla_shenlan_lattice_planner/ament_cmake_index/share/ament_index/resource_index/packages/carla_shenlan_lattice_planner")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/carla_shenlan_lattice_planner/cmake" TYPE FILE FILES
    "/home/zhangwencheng/Documents/autonomous_driving/projects/carla-ros-bridge/build/carla_shenlan_lattice_planner/ament_cmake_core/carla_shenlan_lattice_plannerConfig.cmake"
    "/home/zhangwencheng/Documents/autonomous_driving/projects/carla-ros-bridge/build/carla_shenlan_lattice_planner/ament_cmake_core/carla_shenlan_lattice_plannerConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/carla_shenlan_lattice_planner" TYPE FILE FILES "/home/zhangwencheng/Documents/autonomous_driving/projects/carla-ros-bridge/src/ros-bridge/carla_shenlan_projects/carla_shenlan_lattice_planner/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/zhangwencheng/Documents/autonomous_driving/projects/carla-ros-bridge/build/carla_shenlan_lattice_planner/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
