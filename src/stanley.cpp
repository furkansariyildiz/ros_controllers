#include "ros_controllers/stanley.hpp"



ROS2Controllers::StanleyController::StanleyController(double V, double K)
    : V_(V), K_(K) {

}



ROS2Controllers::StanleyController::~StanleyController() {
    
}



void ROS2Controllers::StanleyController::findEquationOfLine(
    int first_position_x, int first_position_y, 
    int second_position_x, int second_position_y) {
    
    line_slope_ = (second_position_y - first_position_x) / (second_position_x - first_position_x);
    line_intercept_ = second_position_y - line_slope_ * second_position_x;
}



double ROS2Controllers::StanleyController::getStanleyControllerSignal(
    int first_position_x, int first_position_y, 
    int second_position_x, int second_position_y, 
    int vehicle_position_x, int vehicle_position_y, 
    int vehicle_yaw) {

    double error = (vehicle_position_y - line_slope_ * vehicle_position_x - line_intercept_) / (sqrt(pow(-line_slope_, 2) + pow(1, 2)));

    double cross_track_steering = atan2(V_, K_ * error);

    double heading_error = atan2(-(-line_slope_), 1) - vehicle_yaw; 

    double signal = heading_error + cross_track_steering;

    return signal;
}

