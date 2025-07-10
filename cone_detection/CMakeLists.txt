cmake_minimum_required(VERSION 3.10)
project(cone_detection)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


include_directories(${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${EIGEN3_INCLUDE_DIRS} ${PCL_LIBRARY_DIRS})

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/voxel_filtering.cpp
)

ament_target_dependencies(${PROJECT_NAME} 
    rclcpp
    sensor_msgs
    geometry_msgs
    pcl_conversions
    pcl_ros
    tf2
    tf2_ros
    tf2_geometry_msgs
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} 
  ${PCL_LIBRARIES} 
  Eigen3::Eigen
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "cone_detection::ConeDetection"
  EXECUTABLE ${PROJECT_NAME}_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
