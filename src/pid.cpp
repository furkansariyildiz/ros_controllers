#include "ros_controllers/pid.hpp"


ROS2Controllers::PIDController::PIDController(double Kp, double Ki, double Kd,  
                                              double error_threshold, const double signal_limit)
    : Kp_(Kp), Ki_(Ki), Kd_(Kd), previous_error_(0.0), P_(0.0), I_(0.0), D_(0.0), 
      error_threshold_(error_threshold), signal_limit_(signal_limit){
    // Subscribers

    // Publishers

    // Timers

    // Parameters

}



ROS2Controllers::PIDController::~PIDController() {
    
}



std::pair<double, bool> ROS2Controllers::PIDController::getPIDControllerSignal(double error, double dt) {
    P_ = Kp_ * error;
    I_ = I_ + Ki_ * error * dt;
    D_ = Kd_ * (error - previous_error_) / dt;

    previous_error_ = error;

    double signal = P_ + I_ + D_;

    if (std::abs(error) <= error_threshold_) {
        return std::make_pair(signal, true);
    } 

    if (signal > signal_limit_) {
        return std::make_pair(signal_limit_, false);
    } else if (signal < -signal_limit_) {
        return std::make_pair(-signal_limit_, false);
    }

    return std::make_pair(signal, false);
}



void ROS2Controllers::PIDController::run() {
}
