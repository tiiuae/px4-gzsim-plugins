cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(mavlink_hitl_gazebosim)

set(DCMAKE_EXPORT_COMPILE_COMMANDS=ON)


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
#########################################################################
add_library(vibration_plugin SHARED src/gazebo_vibration_plugin.cpp)
add_library(parachute_plugin SHARED src/gazebo_parachute_plugin.cpp)
#########################################################################
set_property(TARGET mavlink_hitl_gazebosim PROPERTY CXX_STANDARD 17)
set_property(TARGET vibration_plugin PROPERTY CXX_STANDARD 17)
set_property(TARGET parachute_plugin PROPERTY CXX_STANDARD 17)


#########################################################################

# Replace the global include_directories usage with target-scoped include dirs:
target_include_directories(mavlink_hitl_gazebosim
  PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_BINARY_DIR}                # if generated headers live in build
    ${Boost_INCLUDE_DIR}               # Boost include variable
    ${MAVLINK_INCLUDE_DIRS}            # MAVLink include paths (from find_package)
)

target_include_directories(vibration_plugin
  PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${Boost_INCLUDE_DIR}
)

target_include_directories(parachute_plugin
  PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${Boost_INCLUDE_DIR}
)
#########################################################################


target_link_libraries(mavlink_hitl_gazebosim
  PRIVATE ${Boost_LIBRARIES}
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GAZEBO_VERSION}::gz-sim${GAZEBO_VERSION}
  PRIVATE gz-sensors8::gz-sensors8
)

target_link_libraries(vibration_plugin
  PRIVATE ${Boost_LIBRARIES}
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GAZEBO_VERSION}::gz-sim${GAZEBO_VERSION})

target_link_libraries(parachute_plugin
  PRIVATE ${Boost_LIBRARIES}
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GAZEBO_VERSION}::gz-sim${GAZEBO_VERSION})

include(GNUInstallDirs)
install(TARGETS
  mavlink_hitl_gazebosim
  vibration_plugin
  parachute_plugin
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  )

set(CPACK_PACKAGE_CONTACT "Auterion")
include(CPack)
