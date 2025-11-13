cmake_minimum_required(VERSION 3.0.2)
project(improved_dwa_local_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pluginlib
  nav_core
  base_local_planner
  costmap_2d
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

# Эта строка говорит другим пакетам, где искать наши заголовочные файлы
catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES improved_dwa_local_planner_lib
  CATKIN_DEPENDS roscpp pluginlib nav_core base_local_planner costmap_2d geometry_msgs tf2_ros tf2_geometry_msgs
)

# Эта строка говорит компилятору, где искать заголовочные файлы при сборке этого пакета
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(improved_dwa_local_planner_lib
  src/improved_dwa_local_planner.cpp
)

target_link_libraries(improved_dwa_local_planner_lib
  ${catkin_LIBRARIES}
)

install(TARGETS improved_dwa_local_planner_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  planner_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)