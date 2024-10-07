#include "ros_controllers/pure_pursuit.hpp"



ROS2Controllers::PurePursuiteController::PurePursuiteController(double Kp, double Ki, double 
                                                                Kd, double error_threshold, double signal_limit)
    : Kp_(Kp), Ki_(Ki), Kd_(Kd), error_threshold_(error_threshold), signal_limit_(signal_limit) {
    // Subscribers

    // Publishers

    // Timers

}



ROS2Controllers::PurePursuiteController::~PurePursuiteController() {
    
}   



std::tuple<double, double, bool> ROS2Controllers::PurePursuiteController::getPurePursuiteSignal(double error, double dt) {
    return std::make_tuple(0.0, 0.0, false);
}