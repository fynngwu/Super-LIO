cmake_minimum_required(VERSION 3.16)
project(super_lio_offline LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_definitions("-DPCL_NO_PRECOMPILE")
add_definitions(-DROOT=\"${CMAKE_CURRENT_SOURCE_DIR}/../super_lio/\")

# Performance optimization flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -fopenmp")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native -DNDEBUG -fopenmp -ffast-math -funroll-loops")
set(CMAKE_BUILD_TYPE Release)

# Find ROS2 packages (for message types and rosbag2)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

# Find other dependencies
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(glog REQUIRED)
find_package(gflags REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(TBB REQUIRED)

# Super-LIO paths
set(SUPER_LIO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../super_lio")
set(BASIC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../basic")

# Include directories
include_directories(
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${rosbag2_cpp_INCLUDE_DIRS}
    ${livox_ros_driver2_INCLUDE_DIRS}
    ${TBB_INCLUDE_DIRS}
    ${yaml-cpp_INCLUDE_DIRS}
    ${SUPER_LIO_ROOT}/include
    ${SUPER_LIO_ROOT}/3rdparty
    ${BASIC_ROOT}/include
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/wrapper
    ${CMAKE_CURRENT_SOURCE_DIR}/io
)

# Wrapper source files
set(WRAPPER_SOURCES
    wrapper/bag_io.cc
    wrapper/offline_wrapper.cpp
    io/yaml_io.cc
    io/file_io.cc
)

# Super-LIO core sources
set(LIO_SOURCES
    ${SUPER_LIO_ROOT}/src/lio/super_lio.cpp
    ${SUPER_LIO_ROOT}/src/lio/ESKF.cpp
    ${SUPER_LIO_ROOT}/src/lio/params.cpp
    ${BASIC_ROOT}/src/Manifold.cpp
)

# Add executable
add_executable(run_offline
    app/run_offline.cc
    ${WRAPPER_SOURCES}
    ${LIO_SOURCES}
)

# Link libraries
target_link_libraries(run_offline
    rclcpp::rclcpp
    rosbag2_cpp::rosbag2_cpp
    rosbag2_storage::rosbag2_storage
    sensor_msgs::sensor_msgs__rosidl_generator_cpp
    sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
    nav_msgs::nav_msgs__rosidl_generator_cpp
    nav_msgs::nav_msgs__rosidl_typesupport_cpp
    std_msgs::std_msgs__rosidl_generator_cpp
    std_msgs::std_msgs__rosidl_typesupport_cpp
    livox_ros_driver2::livox_interfaces2__rosidl_typesupport_cpp
    ${PCL_LIBRARIES}
    Eigen3::Eigen
    ${yaml-cpp_LIBRARIES}
    glog
    gflags
    pthread
    OpenMP::OpenMP_CXX
    TBB::tbb
)

# Install
install(TARGETS run_offline DESTINATION bin)