cmake_minimum_required(VERSION 3.8)
project(motion_capture_tracking)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_INSTALL_RPATH "$ORIGIN")
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

# To use NOKOV, place their SDK in
# motion_capture_tracking/deps/libmotioncapture/deps/nokov_sdk
# and set this variable to "ON"
set(LIBMOTIONCAPTURE_ENABLE_NOKOV OFF)

# To use Motion Analysis, place their SDK in
# motion_capture_tracking/deps/libmotioncapture/deps/cortex_sdk_linux
# and set this variable to "ON"
set(LIBMOTIONCAPTURE_ENABLE_MOTIONANALYSIS OFF)

add_subdirectory(deps/libmotioncapture EXCLUDE_FROM_ALL)

include_directories(
  deps/libmotioncapture/include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(motion_capture_tracking_node
  src/motion_capture_tracking_node.cpp
)
target_link_libraries(motion_capture_tracking_node
  libmotioncapture
)
ament_target_dependencies(motion_capture_tracking_node
  rclcpp
  tf2_ros
  geometry_msgs
)
target_include_directories(motion_capture_tracking_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(motion_capture_tracking_node PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++20

install(TARGETS motion_capture_tracking_node
  DESTINATION lib/${PROJECT_NAME})

# The precompiled libNatNet library is only available for x64 Linux
if ((CMAKE_SYSTEM_NAME STREQUAL "Linux") AND (CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64"))
  install(FILES 
    ${CMAKE_CURRENT_SOURCE_DIR}/deps/libmotioncapture/deps/NatNetSDKCrossplatform/lib/ubuntu/libNatNet.so
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files.
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install meshes.
install(DIRECTORY 
  meshes
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
