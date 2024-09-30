cmake_minimum_required(VERSION 3.8)
project(ros_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include {catkin_INCLUDE_DIRS})

add_library(ros_controllers STATIC
            src/stanley.cpp
)

add_executable(ros_controllers_node src/main_node.cpp)

target_link_libraries(ros_controllers
                      ${catkin_LIBRARIES}

)     

ament_target_dependencies(ros_controllers rclcpp rclpy nav_msgs)

target_link_libraries(ros_controllers_node 
                      ${catkin_LIBRARIES}
                      ros_controllers
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

ament_target_dependencies(ros_controllers_node rclcpp rclpy nav_msgs)

                      
ament_export_dependencies(rosidl_default_runtime)
ament_package()
