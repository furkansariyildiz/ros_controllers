#include "ros_controllers/mpc_controller.hpp"



ROS2Controllers::MPCController::MPCController(const int horizon, const double vehicle_base_width,
    const double error_threshold, const double signal_limit, const double dt)
    : horizon_(horizon), vehicle_base_width_(vehicle_base_width), error_threshold_(error_threshold), 
      signal_limit_(signal_limit), dt_(dt) {
    // Subscribers

    // Publishers

    // Timers

}



ROS2Controllers::MPCController::~MPCController() {
    
}



void ROS2Controllers::MPCController::setupOptimizationProblem() {
    SX x = SX::sym("x");
    SX y = SX::sym("y");
    SX theta = SX::sym("theta");

    // Xk = [x, y, theta]
    SX state = SX::vertcat({x, y, theta});

    SX v = SX::sym("v");
    SX w = SX::sym("w");
    
    // Uk = [v, w]
    SX control = SX::vertcat({v, w});

    // System dynamics
    SX rhs = SX::vertcat({
        v * SX::cos(theta),
        v * SX::sin(theta),
        w
    });

    // Discrete-time dynamics
    Function f = Function("f", {state, control}, {state + rhs * dt_});

    // Optimization variables
    SX U = SX::sym("U", 2, horizon_);              // Controls [v, w]
    SX X = SX::sym("X", 3, horizon_ + 1);          // States [x, y, theta]
    SX P = SX::sym("P", 3 + 3 * (horizon_ + 1));   // Parameters (initial state and references)

    // Objective function and constraints
    SX obj = 0;
    SX g;

    // Initial state constraint
    g = X(Slice(), 0) - P(Slice(0, 3));

    // Optimization loop
    for (int k=0; k<horizon_; k++) {
        
    }
}



std::tuple<double, double> ROS2Controllers::MPCController::getMPCControllerSignal(const double dt, const double vehicle_position_x,
    const double vehicle_position_y, const double vehicle_yaw, const std::vector<geometry_msgs::msg::PoseStamped> path) {
    return std::make_tuple(0.0, 0.0);
}