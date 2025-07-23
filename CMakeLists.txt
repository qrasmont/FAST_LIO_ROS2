cmake_minimum_required(VERSION 3.8)
project(fast_lio)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

ADD_COMPILE_OPTIONS(-std=c++17 -O3 -pthread -fexceptions)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")

  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

# ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(tf2_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
  pcl_ros
  pcl_conversions
  livox_ros_driver2
  rosbag2_cpp
  rosbag2_storage
  tf2_msgs
)

# Thirdparty libraries
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

message(Eigen: ${EIGEN3_INCLUDE_DIR})
message(STATUS "PCL: ${PCL_INCLUDE_DIRS}")

set(msg_files
  "msg/Pose6D.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)
ament_export_dependencies(rosidl_default_runtime)

# Define the core logic as a shared library
add_library(fast_lio_core SHARED
  src/FastLioCore.cpp
  src/preprocess.cpp
  src/IMU_Processing.cpp
  include/ikd-Tree/ikd_Tree.cpp
)

# Link headers for core
target_include_directories(fast_lio_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ikd-Tree> 
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)

# Link dependencies for core
target_link_libraries(fast_lio_core
  ${PCL_LIBRARIES}
  ${PYTHON_LIBRARIES}
  Eigen3::Eigen
)

# Link ROS dependencies for core
ament_target_dependencies(fast_lio_core ${dependencies})

# Link custom message typesupport
list(APPEND EOL_LIST "foxy" "galactic" "eloquent" "dashing" "crystal")
if($ENV{ROS_DISTRO} IN_LIST EOL_LIST)
  rosidl_target_interfaces(fast_lio_core
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
  rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(fast_lio_core ${cpp_typesupport_target})
endif()


# Define the fastlio_mapping ROS node executable
add_executable(fastlio_mapping src/laserMapping.cpp src/main.cpp)
target_link_libraries(fastlio_mapping fast_lio_core)


# ---------------- Install --------------- #
# Install the standalone node
install(TARGETS fastlio_mapping
  DESTINATION lib/${PROJECT_NAME}
)

set(export_targets_name "${PROJECT_NAME}_targets")

# Install the core library for use with other packages
install(TARGETS fast_lio_core
  EXPORT ${export_targets_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# Export the library and include directories
ament_export_targets(${export_targets_name}
  HAS_LIBRARY_TARGET
)

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
