cmake_minimum_required(VERSION 3.0.2)
project(indooruav_waypoint)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_srvs
  std_msgs
  geometry_msgs
  nav_msgs
  indooruav_msgs
)

find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp roslib std_srvs std_msgs geometry_msgs nav_msgs indooruav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

# ----- waypoint_tracker -----
add_executable(waypoint_tracker_node
  node/waypoint_tracker_node.cpp
  src/waypoint_tracker.cpp
)
target_link_libraries(waypoint_tracker_node
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

# ----- waypoint_recorder -----
add_executable(waypoint_recorder_node
  node/waypoint_recorder_node.cpp
  src/waypoint_recorder.cpp
)
target_link_libraries(waypoint_recorder_node
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

install(TARGETS waypoint_tracker_node waypoint_recorder_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/waypoint_record_button.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
