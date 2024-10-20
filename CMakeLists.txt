cmake_minimum_required(VERSION 3.8)
project(ros_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(casadi REQUIRED)

include_directories(include {catkin_INCLUDE_DIRS})

add_library(ros_controllers STATIC
            src/pid.cpp
            src/stanley.cpp
            src/pure_pursuit.cpp
            src/mpc.cpp
            src/mpc_controller.cpp
)

add_executable(ros_controllers_node src/main_node.cpp)

target_link_libraries(ros_controllers
                      ${catkin_LIBRARIES}
                      casadi

)     

ament_target_dependencies(ros_controllers rclcpp rclpy tf2 nav_msgs geometry_msgs std_msgs std_srvs)

target_link_libraries(ros_controllers_node 
                      ${catkin_LIBRARIES}
                      ros_controllers
                      casadi
)

install(TARGETS
  ros_controllers_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_target_dependencies(ros_controllers_node rclcpp rclpy tf2 nav_msgs geometry_msgs std_msgs std_srvs)

                      
ament_export_dependencies(rosidl_default_runtime)
ament_package()
