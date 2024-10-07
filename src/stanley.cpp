#include "ros_controllers/stanley.hpp"



ROS2Controllers::StanleyController::StanleyController(double V, double K, double 
                                                      error_threshold, double signal_limit)
    : V_(V), K_(K), error_threshold_(error_threshold), siganl_limit_(signal_limit) {
    // Subscribers

    // Publishers

    // Timers

}



ROS2Controllers::StanleyController::~StanleyController() {
    
}



std::tuple<double, double, bool> ROS2Controllers::StanleyController::getStanleyControllerSignal(double next_waypoint_x, double next_waypoint_y, 
                                                                      double previous_waypoint_x, double previous_waypoint_y,
                                                                      double vehicle_position_x, double vehicle_position_y,
                                                                      double vehicle_yaw) {
    double e_numerator = (next_waypoint_x - previous_waypoint_x) * (previous_waypoint_y - vehicle_position_y) -
                        (previous_waypoint_x - vehicle_position_x) * (next_waypoint_y - previous_waypoint_y);

    double e_denominator = std::sqrt(std::pow(next_waypoint_x - previous_waypoint_x, 2) + 
                                    std::pow(next_waypoint_y - previous_waypoint_y, 2));

    double error = e_numerator / e_denominator;

    double theta_track = atan2(next_waypoint_y - previous_waypoint_y, next_waypoint_x - previous_waypoint_x);
    
    double psi = theta_track - vehicle_yaw;

    psi = atan2(sin(psi), cos(psi));

    double track_steering_correction = atan2(K_ * error, V_);

    track_steering_correction = atan2(sin(track_steering_correction), cos(track_steering_correction));

    signal_ = psi + track_steering_correction;

    if (std::abs(signal_) > siganl_limit_) {
        if (signal_ > 0) {
            signal_ = siganl_limit_;
        } else {
            signal_ = -siganl_limit_;
        }
    }

    if (std::abs(error) <= error_threshold_) {
        return std::make_tuple(V_, 0.0, true);
    }

    return std::make_tuple(V_, signal_, false);
}


