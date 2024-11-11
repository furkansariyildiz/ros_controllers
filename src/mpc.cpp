#include "ros_controllers/mpc.hpp"



ROS2Controllers::MPCController::MPCController(double dt, int horizon, double L, std::vector<double> Q, std::vector<double> R, double signal_limit_linear_velocity, 
    double signal_limit_angular_velocity, double error_threshold)
    : dt_(dt), horizon_(horizon), L_(L), Q_vector_(Q), R_vector_(R), error_threshold_(error_threshold), 
      signal_limit_linear_velocity_(signal_limit_linear_velocity), signal_limit_angular_velocity_(signal_limit_angular_velocity), 
      discrete_linear_error_(0.0), continous_linear_error_(0.0) {

    // Define the weighting matrices
    Q_ = casadi::SX::diag(casadi::SX({Q_vector_[0], Q_vector_[1], Q_vector_[2]}));   // x, y and theta
    R_ = casadi::SX::diag(casadi::SX({R_vector_[0], R_vector_[1]}));        // v and w

    // Solver options
    opts_ = casadi::Dict();
    opts_["ipopt.print_level"] = 0;
    opts_["print_time"] = false;
    opts_["verbose"] = false;

    std::cout << "dt: " << dt_ << std::endl;
    std::cout << "horizon: " << horizon_ << std::endl;
    std::cout << "L: " << L_ << std::endl;

    // Setup the optimization problem
    setupOptimizationProblem();
}



ROS2Controllers::MPCController::~MPCController() {

}



double ROS2Controllers::MPCController::getDiscreteLinearError() {
    return discrete_linear_error_;
}



double ROS2Controllers::MPCController::getContinousLinearError() {
    return continous_linear_error_;
}



void ROS2Controllers::MPCController::setupOptimizationProblem() {
    using casadi::SX;
    using casadi::Slice;
    using casadi::Function;

    // State and control variables
    SX x = SX::sym("x");
    SX y = SX::sym("y");
    SX theta = SX::sym("theta"); 
    SX state = SX::vertcat({x, y, theta});

    SX v = SX::sym("v"); // Linear velocity
    SX delta = SX::sym("delta"); // Steering angle
    // SX w = SX::sym("w"); // Angular velocity
    SX control = SX::vertcat({v, delta}); // Linear velocity and steering angle
    // SX control = SX::vertcat({v, w}); // Linear velocity and angular velocity

    std::cout << "L_" << L_ << std::endl;

    // System dynamics
    SX rhs = SX::vertcat({
         v * SX::cos(theta),
         v * SX::sin(theta),
         (v / L_) * SX::tan(delta)
    });

    // SX rhs = SX::vertcat({
    //    v * SX::cos(theta),
    //    v * SX::sin(theta),
    //    w
    // });

    // Discrete-time dynamics
    Function f = Function("f", {state, control}, {state + rhs * dt_});

    // Optimization variables
    SX U = SX::sym("U", 2, horizon_);              // Controls
    SX X = SX::sym("X", 3, horizon_ + 1);          // States
    SX P = SX::sym("P", 3 + 3 * (horizon_ + 1));   // Parameters 

    // Goal function and constraints
    SX obj = 0;
    SX g = SX::zeros(3 * (horizon_ + 1), 1); 

    // Initial state constraint
    g(Slice(0, 3), 0) = X(Slice(), 0) - P(Slice(0, 3));

    // Optimization loop
    for (int k = 0; k < horizon_; ++k) {
        // Reference states
        SX X_ref = P(Slice(3 + 3 * k, 3 + 3 * (k + 1)));

        // Objective function
        SX state_error = X(Slice(), k) - X_ref;
        SX control_k = U(Slice(), k);
        obj += SX::mtimes(state_error.T(), SX::mtimes(Q_, state_error)) + SX::mtimes(control_k.T(), SX::mtimes(R_, control_k));

        // System dynamics (X_k+1 = f(X_k, U_k)) (X_k+1 = A * X_k + B * U_k)
        std::vector<SX> args = {X(Slice(), k), U(Slice(), k)};
        SX X_next = f(args).at(0);

        // State constraints
        g(Slice(3 * (k + 1), 3 * (k + 2)), 0) = X_next - X(Slice(), k + 1);
    }

    // For last step, calculating the state error and adding it to the objective function
    SX X_ref = P(Slice(3 + 3 * horizon_, 3 + 3 * (horizon_ + 1)));
    SX state_error = X(Slice(), horizon_) - X_ref;
    obj += SX::mtimes(state_error.T(), SX::mtimes(Q_, state_error));

    // Optimization variables
    SX OPT_variables = SX::vertcat({SX::reshape(U, 2 * horizon_, 1), SX::reshape(X, 3 * (horizon_ + 1), 1)});

    // Creating nlp 
    casadi::SXDict nlp = {
        {"x", OPT_variables},
        {"f", obj},
        {"g", g},
        {"p", P}
    };

    // Generating the solver
    solver_ = casadi::nlpsol("solver", "ipopt", nlp, opts_);
}



std::tuple<double, double, bool> ROS2Controllers::MPCController::computeControlSignal(
    const Eigen::VectorXd& state,
    const std::vector<Eigen::VectorXd>& reference_trajectory) {

    // Check the size of the state and reference trajectory
    if (state.size() < 3) {
        std::cout << "Error: State vector size is less than 3!" << std::endl;
        return std::make_tuple(0.0, 0.0, false);  
    } else if (reference_trajectory.size() < horizon_ + 1) {
        horizon_--;
        setupOptimizationProblem();
        std::cout << "Error: Reference trajectory size is less than expected!" << std::endl;
        // return std::make_tuple(0.0, 0.0, false);  
    }

    // Prepare the initial state and references
    std::vector<double> p_data;
    p_data.reserve(3 + 3 * (horizon_ + 1));

    // Initial state
    for (int i = 0; i < 3; ++i) {
        p_data.push_back(state(i));
    }
    
    // Reference trajectory
    for (int k = 0; k < horizon_ + 1; ++k) {
        const Eigen::VectorXd& ref = reference_trajectory[k];
        p_data.push_back(ref(0)); // x_ref
        p_data.push_back(ref(1)); // y_ref
        p_data.push_back(ref(2)); // theta_ref
    }

    // Converting the data (p_data) to casadi::DM
    casadi::DM p = casadi::DM(p_data);

    // Prediction of starting point
    casadi::DM U0 = casadi::DM::zeros(2, horizon_);
    std::vector<double> state_vector(state.data(), state.data() + state.size());
    casadi::DM state_dm = casadi::DM(state_vector);
    casadi::DM X0 = casadi::DM::repmat(state_dm, 1, horizon_ + 1);

    // Vertical concatenation of the optimization variables
    casadi::DM OPT_variables = casadi::DM::vertcat({
        casadi::DM::reshape(U0, 2 * horizon_, 1),
        casadi::DM::reshape(X0, 3 * (horizon_ + 1), 1)
    });

    // Constraints limits
    double v_min = 0.0; 
    double v_max = signal_limit_linear_velocity_;

    double w_min = -signal_limit_angular_velocity_;
    double w_max = signal_limit_angular_velocity_;
    
    int n_controls_total = 2 * horizon_;
    int n_states_total = 3 * (horizon_ + 1);
    int nu = 2;
    int nlp_x = n_controls_total + n_states_total;
    int nlp_g = 3 * (horizon_ + 1);

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

    // Running optimization
    std::map<std::string, casadi::DM> arg;

    arg["x0"] = OPT_variables;
    arg["lbg"] = lbg;
    arg["ubg"] = ubg;
    arg["lbx"] = lbx;
    arg["ubx"] = ubx;
    arg["p"] = p;

    auto res = solver_(arg);

    // Getting the optimal control signal
    casadi::DM sol = res.at("x");
    casadi::DM u = sol(casadi::Slice(0, 2 * horizon_));

    // Control inputs
    double optimal_velocity = static_cast<double>(u(0));
    double optimal_steering_angle = static_cast<double>(u(1));

    // Calculating distance between vehicle and first reference point
    continous_linear_error_ = std::sqrt(std::pow(reference_trajectory[0](0) - state(0), 2) + 
        std::pow(reference_trajectory[0](1) - state(1), 2));
    
    std::cout << "Distance to first reference point: " << continous_linear_error_ << std::endl;

    if (continous_linear_error_ < error_threshold_) {
        discrete_linear_error_ = continous_linear_error_;
        return std::make_tuple(optimal_velocity, optimal_steering_angle, true);
    }

    return std::make_tuple(optimal_velocity, optimal_steering_angle, false);
}

