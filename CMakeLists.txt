# -*- cmake -*-

CMAKE_MINIMUM_REQUIRED(VERSION 3.14)

project(microblx-ros)

find_package(PkgConfig REQUIRED)

#find_package(orocos_kdl REQUIRED)
#include_directories(${orocos_kdl_INCLUDE_DIRS})
#link_directories(${orocos_kdl_LIBRARY_DIRS})

pkg_check_modules(ROSCPP REQUIRED roscpp)
pkg_check_modules(KDL REQUIRED orocos-kdl>=1.4.0)
pkg_check_modules(UBX REQUIRED ubxkdl)
pkg_check_modules(UBX REQUIRED ubx0>=0.8.0)
pkg_get_variable(UBX_MODDIR ubx0 UBX_MODDIR)

# ros_std
add_library(ros_std SHARED src/ros_std.hpp src/ros_std.cpp src/ros_std_pub.cpp src/ros_std_sub.cpp)

set_target_properties(ros_std PROPERTIES
  INSTALL_RPATH_USE_LINK_PATH TRUE
  PREFIX "")

target_compile_options(ros_std PUBLIC
  "${UBX_CFLAGS} ${ROS_CFLAGS} ${KDL_CFLAGS}"
  -Wall -Wextra -Werror
  -Wno-missing-field-initializers
  -fvisibility=hidden
  )

target_link_libraries(ros_std ${UBX_LDFLAGS} ${ROSCPP_LDFLAGS})

install(TARGETS ros_std DESTINATION ${UBX_MODDIR})


# ros_kdl
