#include "ros_controllers/pid.hpp"


ROS2Controllers::PIDController::PIDController(const std::shared_ptr<rclcpp::Node> &node, double Kp, double Ki, 
                                              double Kd, double error_threshold, const double signal_limit)
    : node_(node), Kp_(Kp), Ki_(Ki), Kd_(Kd), previous_error_(0.0), P_(0.0), I_(0.0), D_(0.0), 
      error_threshold_(error_threshold), signal_limit_(signal_limit){
    // Subscribers

    // Publishers

    // Timers

    // Parameters

}



ROS2Controllers::PIDController::~PIDController() {
    
}



double ROS2Controllers::PIDController::getPIDControllerSignal(double error, double dt) {
    P_ = Kp_ * error;
    I_ = I_ + Ki_ * error * dt;
    D_ = Kd_ * (error - previous_error_) / dt;

    previous_error_ = error;

    double signal = P_ + I_ + D_;

    if (signal > signal_limit_) {
        return signal_limit_;
    } else if (signal < -signal_limit_) {
        return -signal_limit_;
    }

    if (error <= error_threshold_) {
        return 0.0;
    } 

    return signal;
}



void ROS2Controllers::PIDController::run() {
    RCLCPP_INFO_STREAM(node_->get_logger(), "PID run function.");
}
