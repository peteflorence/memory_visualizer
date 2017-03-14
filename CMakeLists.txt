cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
project(memory_visualizer)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  visualization_msgs
  nav_msgs
  roscpp
  std_msgs
  std_srvs
  tf
  tf2
  cv_bridge
  image_transport
  acl_fsw
  message_generation
  sensor_msgs
  mavros_msgs
  pcl_ros
  fla_msgs
  octomap_ros
)

add_message_files( DIRECTORY msg FILES Coeff.msg State.msg )
generate_messages( DEPENDENCIES std_msgs nav_msgs)

find_package( Eigen3 REQUIRED )
find_package(OpenCV 2.4.8 REQUIRED)
find_package(PCL REQUIRED)

include_directories ( src )
include_directories( ${EIGEN3_INCLUDE_DIR} )
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(orocos_kdl REQUIRED)
find_library(OROCOS_KDL orocos-kdl)
set(orocos_kdl_LIBRARIES ${OROCOS_KDL})

add_library( memory_visualizer_lib src/depth_pose.cpp src/fov_evaluator.cpp)

add_library( nanomap_lib src/nanomap.cpp)

add_executable( memory_visualizer_node src/memory_visualizer_node.cpp )
target_link_libraries( memory_visualizer_node memory_visualizer_lib ${catkin_LIBRARIES} orocos-kdl)

add_executable( depth_graph_visualizer_node src/depth_graph_visualizer_node.cpp)
add_executable( depth_history_latency_evaluator_node src/depth_history_latency_evaluator_node.cpp)

add_library( transform_utils src/transform_utils)
target_link_libraries( depth_graph_visualizer_node transform_utils memory_visualizer_lib ${catkin_LIBRARIES} orocos-kdl)
target_link_libraries( depth_history_latency_evaluator_node nanomap_lib transform_utils memory_visualizer_lib ${catkin_LIBRARIES} orocos-kdl)

