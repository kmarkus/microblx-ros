# -*- cmake -*-

CMAKE_MINIMUM_REQUIRED(VERSION 3.14)

project(microblx-ros)

SET(UBXROS_VERSION 1.0.0)

find_package(PkgConfig REQUIRED)

#find_package(orocos_kdl REQUIRED)
#include_directories(${orocos_kdl_INCLUDE_DIRS})
#link_directories(${orocos_kdl_LIBRARY_DIRS})

pkg_check_modules(ROSCPP REQUIRED roscpp)
pkg_check_modules(KDL REQUIRED orocos-kdl>=1.4.0)
pkg_check_modules(UBX REQUIRED ubxkdl)
pkg_check_modules(UBX REQUIRED ubx0>=0.8.0)
pkg_get_variable(UBX_MODDIR ubx0 UBX_MODDIR)
find_program(TOCARR ubx-tocarr)

# generate type hexarrays
file(GLOB type_hdrs ${CMAKE_CURRENT_SOURCE_DIR}/src/types/*.h )
set(hexarr_outdir "${CMAKE_CURRENT_BINARY_DIR}/hexarr")
file(MAKE_DIRECTORY "${hexarr_outdir}/types")
set(hexarrays "")

foreach( type ${type_hdrs} )
  get_filename_component(type_name ${type} NAME)
  message("processing ${type} as ${type_name}")
  add_custom_command(
    OUTPUT "${hexarr_outdir}/types/${type_name}.hexarr"
    DEPENDS ${type}
    COMMAND ${TOCARR}
    ARGS -s ${type} -d "${hexarr_outdir}/types/${type_name}.hexarr")
  list(APPEND hexarrays "${hexarr_outdir}/types/${type_name}.hexarr")
endforeach(type)

# ros_std
add_library(ubxros SHARED src/ros_std.hpp src/ros_std.cpp src/ubxros.cpp ${hexarrays})

set_target_properties(ubxros PROPERTIES
  INSTALL_RPATH_USE_LINK_PATH TRUE
  PUBLIC_HEADER "${type_hdrs}"
  PREFIX "")

target_compile_options(ubxros PUBLIC
  ${UBX_CFLAGS} ${ROS_CFLAGS} ${KDL_CFLAGS}
  -I${hexarr_outdir}
  -Wall -Wextra -Werror
  -Wno-missing-field-initializers
  -fvisibility=hidden
  )

target_link_libraries(ubxros ${UBX_LDFLAGS} ${ROSCPP_LDFLAGS})

install(TARGETS ubxros DESTINATION ${UBX_MODDIR})

