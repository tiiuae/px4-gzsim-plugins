cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(mavlink_hitl_gazebosim)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(Boost 1.58 REQUIRED COMPONENTS system thread filesystem)

find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

set(GAZEBO_VERSION 8)
find_package(gz-sim8 REQUIRED)
find_package(gz-msgs10)
find_package(gz-sensors8 REQUIRED)
if(gz-msgs10_FOUND)
  set(GZ_MSG_VER 10)
  set(Protobuf_IMPORT_DIRS ${gz-msgs10_INCLUDE_DIRS})
else()
  set(GZ_MSG_VER 9)
  set(Protobuf_IMPORT_DIRS ${gz-msgs9_INCLUDE_DIRS})
endif()

find_package(MAVLink)

find_package(Protobuf REQUIRED)

include_directories(${CMAKE_BINARY_DIR})

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${MAVLINK_INCLUDE_DIRS}
)

add_library(mavlink_hitl_gazebosim SHARED src/gazebo_mavlink_interface.cpp src/mavlink_interface.cpp)
set_property(TARGET mavlink_hitl_gazebosim PROPERTY CXX_STANDARD 17)
target_link_libraries(mavlink_hitl_gazebosim
  PRIVATE ${Boost_LIBRARIES}
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GAZEBO_VERSION}::gz-sim${GAZEBO_VERSION}
  PRIVATE gz-sensors8::gz-sensors8
)

include(GNUInstallDirs)
install(TARGETS
  mavlink_hitl_gazebosim
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  )

set(CPACK_PACKAGE_CONTACT "Auterion")
include(CPack)
