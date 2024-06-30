cmake_minimum_required(VERSION 3.0.2)
project(pcd_divider)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  pcl_conversions
  octomap_ros
)

find_package(PCL REQUIRED)
find_package(octomap REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

add_executable(pcd_divider_node src/pcd_divider_node.cpp)
target_link_libraries(pcd_divider_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})
add_dependencies(pcd_divider_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
