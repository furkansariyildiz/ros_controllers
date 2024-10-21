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

    setupOptimizationProblem();
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
    SX U = SX::sym("U", horizon_, 2);              // Controls [v, w]
    SX X = SX::sym("X", horizon_ + 1, 3);          // States [x, y, theta]
    SX P = SX::sym("P", 3 + 3 * (horizon_ + 1));   // Parameters (initial state and references)

    // Objective function and constraints
    SX obj = 0;
    SX g = X(0, Slice()).T() - P(Slice(0, 3));

    // Initial state constraint
    // g = X(Slice(), 0) - P(Slice(0, 3));

    // Optimization loop
    for (int k=0; k<horizon_; ++k) {
        // State references
        SX X_ref = P(Slice(3 + 3 * k, 3 + 3 * (k + 1)));

        // Goal function
        SX state_error = X(k, Slice()).T() - X_ref;
        SX control_k = U(k, Slice());
        // obj += SX::dot(state_error, Q_ * state_error) + SX::dot(control_k, R_ * control_k);

        SX cost_state = SX::mtimes(SX::mtimes(state_error.T(), Q_), state_error);
        SX cost_control = SX::mtimes(control_k, SX::mtimes(R_, control_k.T()));

        obj += cost_state + cost_control;

        // System dynamics (X_k+1 = f(X_k, U_k))
        std::vector<SX> args = {X(k, Slice()), U(k, Slice())};
        SX X_next = f(args).at(0);

        // Accumulate constraints
        g = SX::vertcat({g, X_next - X(k + 1, Slice()).T()});
    }

    // NLP Problem Definition
        SX OPT_variables = SX::vertcat({
        SX::reshape(U, -1, 1),
        SX::reshape(X, -1, 1)
    });

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

    int N = horizon_;
    int nu = 2;
    int nx = 3;

    casadi::DM U_prev = previous_solution(casadi::Slice(0, n_controls));
    casadi::DM X_prev = previous_solution(casadi::Slice(n_controls, n_controls + n_states));

    // U_prev ve X_prev matrislerini doğru boyutlarda yeniden şekillendirin
    casadi::DM U_prev_reshaped = SX::reshape(U_prev, N, nu);          // (N x 2)
    casadi::DM X_prev_reshaped = SX::reshape(X_prev, N + 1, nx);      // (N+1 x 3)

    // Boyutların uygunluğunu kontrol edin
    if (U_prev_reshaped.size1() != N || U_prev_reshaped.size2() != nu) {
        std::cout << "U_prev has incompatible size for reshape" << std::endl;
        throw std::runtime_error("U_prev has incompatible size for reshape");
    }

    if (X_prev_reshaped.size1() != N + 1 || X_prev_reshaped.size2() != nx) {
        std::cout << "X_prev has incompatible size for reshape" << std::endl;
        throw std::runtime_error("X_prev has incompatible size for reshape");
    }

    // Kontrol sinyallerini ve durum değişkenlerini kaydırın
    casadi::DM U_shifted = casadi::DM::zeros(N, nu);
    U_shifted(casadi::Slice(0, N - 1), casadi::Slice()) = U_prev_reshaped(casadi::Slice(1, N), casadi::Slice());
    U_shifted(N - 1, casadi::Slice()) = U_prev_reshaped(N - 1, casadi::Slice());

    casadi::DM X_shifted = casadi::DM::zeros(N + 1, nx);
    X_shifted(casadi::Slice(0, N), casadi::Slice()) = X_prev_reshaped(casadi::Slice(1, N + 1), casadi::Slice());
    X_shifted(N, casadi::Slice()) = X_prev_reshaped(N, casadi::Slice());

    // Karar değişkenlerini düzleştirerek birleştirin
    casadi::DM x0 = SX::vertcat({
        SX::reshape(U_shifted, N * nu, 1),
        SX::reshape(X_shifted, (N + 1) * nx, 1)
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
    for (int k=0; k<horizon_ + 1; ++k) {
        const geometry_msgs::msg::PoseStamped &reference_current_pose = path[k];
        double reference_theta;

        if (k<horizon_) {
            const geometry_msgs::msg::PoseStamped &reference_next_pose = path[k + 1];
            reference_theta = atan2(reference_next_pose.pose.position.y - reference_current_pose.pose.position.y, 
                                    reference_next_pose.pose.position.x - reference_current_pose.pose.position.x);
        } else {
            reference_theta = p_data[p_data.size() - 1];
        }

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
        previous_solution_exists_ = true;
    } else {
        // Update the initial guess
        x0 = shiftSolution(previous_solution_, n_controls, n_states);
        std::cout << "test";
    }
    
    // Constraints limits
    double v_min = 0.0;
    double v_max = 0.2;

    double w_min = -0.52;
    double w_max = 0.52;

    int n_controls_total = 2 * horizon_;
    int n_states_total = 3 * (horizon_ + 1);
    int nu = 2;
    
    int nlp_g = 3 * (horizon_ + 1);
    int nlp_x = n_controls_total + n_states_total;
    casadi::DM lbg = casadi::DM::zeros(nlp_g); 
    casadi::DM ubg = casadi::DM::zeros(nlp_g);
    casadi::DM lbx = -casadi::DM::inf(nlp_x);
    casadi::DM ubx = casadi::DM::inf(nlp_x);

    for (int k=0; k<horizon_; ++k) {
        int idx = k * nu;
        lbx(idx) = v_min;
        ubx(idx) = v_max;

        lbx(idx + 1) = w_min;
        ubx(idx + 1) = w_max;
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