#include "ros_controllers/pid.hpp"


ROS2Controllers::PIDController::PIDController(const std::shared_ptr<rclcpp::Node> &node, double Kp, double Ki, 
                                              double Kd, double error_threshold, const nav_msgs::msg::Path path)
    : node_(node), Kp_(Kp), Ki_(Ki), Kd_(Kd), error_threshold_(error_threshold), path_(path) {
    // Subscribers

    // Publishers
    cmd_vel_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);    

    // Timers
}



ROS2Controllers::PIDController::~PIDController() {
    
}



void ROS2Controllers::PIDController::run() {
    RCLCPP_INFO_STREAM(node_->get_logger(), "PID run function.");
}
