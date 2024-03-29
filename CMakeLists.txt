cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(mavlink_sitl_gazebosim)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(Boost 1.58 REQUIRED COMPONENTS system thread filesystem)

find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

set(GAZEBO_VERSION 8)
find_package(gz-sim8 8.2.0 REQUIRED)
if(NOT ${gz-sim8_VERSION} EQUAL 8.2.0)
  message(FATAL_ERROR "gz-sim8 must be version 8.2.0. You have ${gz-sim8_VERSION}")
endif ()
find_package(gz-msgs10)
if(NOT ${gz-msgs10_VERSION} EQUAL 10.1.1)
  message(FATAL_ERROR "gz-msgs10 must be version 10.1.1. You have ${gz-msgs10_VERSION}")
endif ()
find_package(gz-sensors8 REQUIRED)
if(NOT ${gz-sensors8_VERSION} EQUAL 8.0.1)
  message(FATAL_ERROR "gz-sensors8 must be version 8.0.1. You have ${gz-sensors8_VERSION}")
endif ()
find_package(gz-transport13 13.1.0 REQUIRED)
if(NOT ${gz-transport13_VERSION} EQUAL 13.1.0)
  message(FATAL_ERROR "gz-transport13 must be version 13.1.0. You have ${gz-transport13_VERSION}")
endif ()

find_library(GZ_CUSTOM_MSGS NAMES gz_custom_messages PATHS /usr/local/bin)
if (NOT GZ_CUSTOM_MSGS)
  message(FATAL_ERROR "Please install messages library from https://github.com/tiiuae/TASFMessages")
endif ()

if(gz-msgs10_FOUND)
  set(GZ_MSG_VER 10)
  set(Protobuf_IMPORT_DIRS ${gz-msgs10_INCLUDE_DIRS})
else()
  set(GZ_MSG_VER 8)
  find_package(gz-msgs8 REQUIRED)
  set(Protobuf_IMPORT_DIRS ${gz-msgs8_INCLUDE_DIRS})
endif()

find_package(MAVLink)

find_package(Protobuf REQUIRED)

#Generate messages files for collision plugin
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS msgs/CollisionObject.proto)

#Generate messages files for groundtruth plugin
PROTOBUF_GENERATE_CPP(GT_PROTO_SRCS GT_PROTO_HDRS msgs/Groundtruth.proto)

include_directories(${CMAKE_BINARY_DIR})

include_directories(
  include
  ${Boost_INCLUDE_DIR}
#  ${MAVLINK_INCLUDE_DIRS}
)

add_library(mavlink_sitl_gazebosim SHARED src/gazebo_mavlink_interface.cpp src/mavlink_interface.cpp)
set_property(TARGET mavlink_sitl_gazebosim PROPERTY CXX_STANDARD 17)
target_link_libraries(mavlink_sitl_gazebosim
  PRIVATE ${Boost_LIBRARIES}
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GAZEBO_VERSION}::gz-sim${GAZEBO_VERSION}
  PRIVATE gz-sensors8::gz-sensors8
)

include(GNUInstallDirs)
install(TARGETS
  mavlink_sitl_gazebosim
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  )

set(CPACK_PACKAGE_CONTACT "Auterion")
include(CPack)

add_library(collision_plugin SHARED src/harmonic_collision.cpp)
set_property(TARGET collision_plugin PROPERTY CXX_STANDARD 17)
target_link_libraries(collision_plugin
        PRIVATE gz-sim${GAZEBO_VERSION}::gz-sim${GAZEBO_VERSION}
        PRIVATE ${GZ_CUSTOM_MSGS}
)

add_library(groundtruth_plugin SHARED src/groundtruth_plugin.cpp)
target_link_libraries(groundtruth_plugin
        PRIVATE gz-sim${GAZEBO_VERSION}::gz-sim${GAZEBO_VERSION}
        PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
        PRIVATE gz-msgs10::gz-msgs10
        PRIVATE ${GZ_CUSTOM_MSGS}
)
# ln -s ./libgroundtruth_plugin.so /usr/lib/x86_64-linux-gnu/gz-sim-8/plugins
# ln -s ./libcollision_plugin.so /usr/lib/x86_64-linux-gnu/gz-sim-8/plugins