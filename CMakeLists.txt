cmake_minimum_required(VERSION 3.8)
project(ros_controllers)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    tf2
    tf2_geometry_msgs
    nav_msgs
    geometry_msgs
    std_msgs
    std_srvs
    casadi
)


include_directories(include {catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ros_controllers
  CATKIN_DEPENDS roscpp rospy tf2 tf2_geometry_msgs nav_msgs geometry_msgs std_msgs std_srvs
)

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/
  ${catkin_INCLUDE_DIRS}
)

add_library(ros_controllers STATIC
            src/pid.cpp
            src/stanley.cpp
            src/pure_pursuit.cpp
            src/mpc.cpp
)

add_executable(ros_controllers_node src/main_node.cpp)

target_link_libraries(ros_controllers
                      ${catkin_LIBRARIES}
                      casadi
)     

target_link_libraries(ros_controllers_node 
                      ${catkin_LIBRARIES}
                      ros_controllers
                      casadi
)

add_dependencies(ros_controllers_node ${catkin_EXPORTED_TARGETS})


install(TARGETS
  ros_controllers_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

      