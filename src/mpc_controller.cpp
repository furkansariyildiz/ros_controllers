#include "ros_controllers/mpc_controller.hpp"



ROS2Controllers::MPCController::MPCController(const int horizon, const double vehicle_base_width,
    const double error_threshold, const double signal_limit, const double dt)
    : horizon_(horizon), vehicle_base_width_(vehicle_base_width), error_threshold_(error_threshold), 
      signal_limit_(signal_limit), dt_(dt), previous_solution_exists_(false) {
    // Subscribers

    // Publishers

    // Timers
    Q_ = casadi::SX::diag(casadi::SX({1.0, 1.0, 0.1}));   // x, y and theta
    R_ = casadi::SX::diag(casadi::SX({0.5, 0.5}));        // v, w 
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
    for (int k=0; k<horizon_; ++k) {
        // State references
        SX X_ref = P(Slice(3 + 3 * k, 3 + 3 * (k + 1)));

        // Goal function
        SX state_error = X(Slice(), k) - X_ref;
        SX control_k = U(Slice(), k);
        obj += SX::dot(state_error, Q_ * state_error) + SX::dot(control_k, R_ * control_k);

        // System dynamics (X_k+1 = f(X_k, U_k))
        std::vector<SX> args = {X(Slice(), k), U(Slice(), k)};
        SX X_next = f(args).at(0);

        // Accumulate constraints
        g = SX::vertcat({g, X_next - X(Slice(), k + 1)});
    }

    // NLP Problem Definition
    SX OPT_variables = SX::vertcat({SX::reshape(U, -1, 1), SX::reshape(X, -1, 1)});

    casadi::SXDict nlp = {
        {"x", OPT_variables},
        {"f", obj},
        {"g", g},
        {"p", P}
    };

    opts_["ipopt.print_level"] = 0; // 0: no output, 1: some output, 2: more output
    opts_["print_time"] = false; // Print timing information
    opts_["ipopt.max_iter"] = 100; // Maximum number of iterations
    opts_["ipopt.tol"] = 1e-6; // Tolerance

    // Solver Creation (ipopt solver gradient based)
    solver_ = casadi::nlpsol("solver", "ipopt", nlp, opts_);

}



casadi::DM ROS2Controllers::MPCController::shiftSolution(const casadi::DM &previous_solution, int n_controls, int n_states) {

    casadi::DM U_prev = previous_solution(casadi::Slice(0, n_controls));
    casadi::DM X_prev = previous_solution(casadi::Slice(n_controls, n_controls + n_states));

    int N = horizon_;

    casadi::DM U_prev_reshaped = SX::reshape(U_prev, n_controls, N);
    casadi::DM X_prev_reshaped = SX::reshape(X_prev, n_states, N + 1);

    casadi::DM U_shifted = casadi::DM::zeros(n_controls, N);
    U_shifted(casadi::Slice(), casadi::Slice(0, N - 1)) = U_prev_reshaped(casadi::Slice(), casadi::Slice(1, N));
    U_shifted(casadi::Slice(), N - 1) = U_prev_reshaped(casadi::Slice(), N - 1);

    casadi::DM X_shifted = casadi::DM::zeros(n_states, N + 1);
    X_shifted(casadi::Slice(), casadi::Slice(0, N)) = X_prev_reshaped(casadi::Slice(), casadi::Slice(1, N + 1));
    X_shifted(casadi::Slice(), N) = X_prev_reshaped(casadi::Slice(), N); 

    casadi::DM x0 = SX::vertcat({
        SX::reshape(U_shifted, -1, 1),
        SX::reshape(X_shifted, -1, 1)
    });

    return x0;
}



std::tuple<double, double> ROS2Controllers::MPCController::getMPCControllerSignal(const double vehicle_position_x,
    const double vehicle_position_y, const double vehicle_yaw, const std::vector<geometry_msgs::msg::PoseStamped> path) {
    
    // Prepare the initial state and references
    std::vector<double> p_data;
    p_data.reserve(3 + 3 * (horizon_ + 1));

    // Initial state
    p_data.push_back(vehicle_position_x);
    p_data.push_back(vehicle_position_y);
    p_data.push_back(vehicle_yaw);


    // Reference trajectory
    for (int k=1; k<horizon_ + 1; ++k) {
        const geometry_msgs::msg::PoseStamped &reference_previous_pose = path[k - 1];
        const geometry_msgs::msg::PoseStamped &reference_current_pose = path[k];
        const double reference_theta = atan2(reference_current_pose.pose.position.y - reference_previous_pose.pose.position.y, 
                                   reference_current_pose.pose.position.x - reference_previous_pose.pose.position.x);
        p_data.push_back(reference_current_pose.pose.position.x); // x_ref
        p_data.push_back(reference_current_pose.pose.position.y); // y_ref
        p_data.push_back(reference_theta); // theta_ref
    }

    casadi::DM p = casadi::DM(p_data);

    // Prediction of starting point
    int n_controls = 2 * horizon_;
    int n_states = 3 * (horizon_ + 1);

    casadi::DM x0;

    if (!previous_solution_exists_) {
        x0 = casadi::DM::zeros(n_states + n_controls);
        previous_solution_ = true;
    } else {
        // Update the initial guess
        x0 = shiftSolution(previous_solution_, n_controls, n_states);
    }
    
    // Constraints limits
    double v_min = 0.0;
    double v_max = 2.0;

    double w_min = -1.0;
    double w_max = 1.0;

    int n_controls_total = 2 * horizon_;
    int n_states_total = 3 * (horizon_ + 1);

    int nlp_g = 3 * (horizon_ + 1);
    int nlp_x = n_controls_total + n_states_total;
    casadi::DM lbg = casadi::DM::zeros(nlp_g); // Lower bound
    casadi::DM ubg = casadi::DM::zeros(nlp_g); // Upper bound
    casadi::DM lbx = casadi::DM::zeros(nlp_x); // Lower bound
    casadi::DM ubx = casadi::DM::zeros(nlp_x); // Upper bound

    for (int k=0; k<horizon_; ++k) {
        int idx_v = 2 * k;
        int idx_w = 2 * k + 1;

        // Linear velocity limit
        lbx(idx_v) = v_min;
        ubx(idx_v) = v_max;

        // Angular velocity limit
        lbx(idx_w) = w_min;
        ubx(idx_w) = w_max;
    }

    // Run the optimization
    std::map<std::string, casadi::DM> arg;

    arg["x0"] = x0;
    arg["lbg"] = lbg;
    arg["ubg"] = ubg;
    arg["p"] = p;

    arg["lbx"] = lbx;
    arg["ubx"] = ubx;

    auto res = solver_(arg);

    casadi::DM sol = res.at("x");
    casadi::DM u = sol(casadi::Slice(0, n_controls));

    double optimal_velocity = static_cast<double>(u(0));
    double optimal_angular_velocity = static_cast<double>(u(1));

    previous_solution_ = sol;
    
    
    return std::make_tuple(optimal_velocity, optimal_angular_velocity);
}