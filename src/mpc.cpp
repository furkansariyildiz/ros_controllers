#include "ros_controllers/mpc.hpp"



ROS2Controllers::MPCController::MPCController(double dt, int horizon, double L, std::vector<double> Q_param, std::vector<double> R_param, double signal_limit_linear_velocity, 
    double signal_limit_angular_velocity, double error_threshold)
    : dt_(dt), N_(horizon), L_(L), Q_vector_(Q_param), R_vector_(R_param), error_threshold_(error_threshold), 
      signal_limit_linear_velocity_(signal_limit_linear_velocity), signal_limit_angular_velocity_(signal_limit_angular_velocity), 
      discrete_linear_error_(0.0), continous_linear_error_(0.0) {

    SX x = SX::sym("x");
    SX y = SX::sym("y");
    SX theta = SX::sym("theta");
    states_ = SX::vertcat({x, y, theta});
    n_states_ = states_.size1();
    
    SX v = SX::sym("v");
    SX omega = SX::sym("omega");
    controls_ = SX::vertcat({v, omega});
    n_controls_ = controls_.size1();

    // System dynamics (unicycle model)
    SX rhs = SX::vertcat({
        v * SX::cos(theta),
        v * SX::sin(theta),
        omega
    });

    // Convert the system dynamics to a function
    f_ = Function("f", {states_, controls_}, {rhs});

    // MPC variables
    U_ = SX::sym("U", n_controls_, N_);          
    X_ = SX::sym("X", n_states_, N_ + 1);
    P_ = SX::sym("P", n_states_ + N_ * n_states_);

    // Objective function and constraints
    SX obj = 0;
    std::vector<SX> g;

    // Initial state constraint
    SX st = X_(Slice(), 0);
    g.push_back(st - P_(Slice(0, n_states_)));

    // Cost function (state and control weights)
    SX Q = SX::diagcat(std::vector<SX>{SX(Q_vector_[0]), SX(Q_vector_[1]), SX(Q_vector_[2])});
    SX R = SX::diagcat(std::vector<SX>{SX(R_vector_[0]), SX(R_vector_[1])});

    // MPC loop
    for (int k = 0; k < N_; ++k) {
        st = X_(Slice(), k);
        SX con = U_(Slice(), k);
        SX st_next = X_(Slice(), k + 1);
        SX f_value = f_(std::vector<SX>{st, con})[0];
        SX st_next_euler = st + dt_ * f_value;
        g.push_back(st_next - st_next_euler);

        // Reference state
        SX ref = P_(Slice(n_states_ + k * n_states_, n_states_ + (k + 1) * n_states_));

        // Update the objective function
        obj = obj + mtimes((st - ref).T(), mtimes(Q, st - ref)) + mtimes(con.T(), mtimes(R, con));
    }

    // Create the concatenated constraint vector
    SX OPT_variables = SX::vertcat(std::vector<SX>{
        SX::reshape(U_, n_controls_ * N_, 1),
        SX::reshape(X_, n_states_ * (N_ + 1), 1)
    });

    SX G = SX::vertcat(g);

    // Create the NLP problem
    casadi::SXDict nlp_prob = {{"f", obj}, {"x", OPT_variables}, {"g", G}, {"p", P_}};

    // Options for the solver
    Dict opts;
    opts["ipopt.max_iter"] = 2000;
    opts["ipopt.print_level"] = 0;
    opts["print_time"] = 0;
    opts["ipopt.acceptable_tol"] = 1e-8;
    opts["ipopt.acceptable_obj_change_tol"] = 1e-6;

    // Create the solver function
    solver_ = nlpsol("solver", "ipopt", nlp_prob, opts);

    // Initialize control and states values for predictions
    u0_ = DM::zeros(N_, n_controls_);
    x_mpc_ = DM::zeros(N_ + 1, n_states_);

    // State and control constraints
    int n_vars = OPT_variables.size1();
    lbx_ = std::vector<double>(n_vars, -std::numeric_limits<double>::infinity());
    ubx_ = std::vector<double>(n_vars, std::numeric_limits<double>::infinity());

    double v_max = signal_limit_linear_velocity;
    double v_min = -signal_limit_linear_velocity;
    double omega_max = signal_limit_angular_velocity;
    double omega_min = -signal_limit_angular_velocity;

    for (int k = 0; k < N_; ++k) {
        lbx_[k * 2] = v_min;
        ubx_[k * 2] = v_max;

        lbx_[k * 2 + 1] = omega_min;
        ubx_[k * 2 + 1] = omega_max;
    }

    // Current state
    x0_ = DM::zeros(n_states_);

    ref_traj_ = DM::zeros(N_, n_states_);
}



ROS2Controllers::MPCController::~MPCController() {

}



void ROS2Controllers::MPCController::testFunction() {

}


double ROS2Controllers::MPCController::getDiscreteLinearError() {
    return discrete_linear_error_;
}



double ROS2Controllers::MPCController::getContinousLinearError() {
    return continous_linear_error_;
}



void ROS2Controllers::MPCController::setupOptimizationProblem() {

}



void ROS2Controllers::MPCController::updateReferenceTrajectory(const std::vector<Eigen::VectorXd>& reference_trajectory) {
    /*
    for (int k = 0; k < N_; ++k) {
        ref_traj_(k, 0) = reference_trajectory[k](0);
        ref_traj_(k, 1) = reference_trajectory[k](1);
        ref_traj_(k, 2) = reference_trajectory[k](2);
    }
    */
    double amplitude = 2.0;   
    double frequency = 0.2;    
    double v_ref = 0.1;         

    for (int k = 0; k < N_; ++k) {
        double t = (k + 1) * dt_;
        double x_ref = x0_(0).scalar() + v_ref * t;
        double y_ref = amplitude * sin(frequency * x_ref);

        double dy_dx = atan2(y_ref - x0_(1).scalar(), x_ref - x0_(0).scalar());
        double theta_ref = atan2(sin(dy_dx), cos(dy_dx));

        ref_traj_(k, 0) = x_ref;
        ref_traj_(k, 1) = y_ref;
        ref_traj_(k, 2) = theta_ref;

        if (x_ref >= 5.0) {
            saveData();
            rclcpp::shutdown();
        }

        reference_trajectory_.push_back({x_ref, y_ref, 0.0});
    }
}



void ROS2Controllers::MPCController::saveData() {
    std::ofstream vehicle_file("/home/furkan/controller_ws/src/ros_controllers/results/result_mpc.csv");
    vehicle_file << "vehicle_x,vehicle_y,vehicle_z\n";
    for (const auto& pos : vehicle_positions_)
    {
        vehicle_file << pos[0] << "," << pos[1] << "," << pos[2] << "\n";
    }
    vehicle_file.close();

    std::ofstream ref_file("/home/furkan/controller_ws/src/ros_controllers/results/desired_mpc.csv");
    ref_file << "desired_x,desired_y,desired_z\n";
    for (const auto& ref : reference_trajectory_)
    {
        ref_file << ref[0] << "," << ref[1] << "," << ref[2] << "\n";
    }
    ref_file.close();
}



std::tuple<double, double, bool> ROS2Controllers::MPCController::computeControlSignal(
    const Eigen::VectorXd& state,
    const std::vector<Eigen::VectorXd>& reference_trajectory) {

    // Get the current state
    x0_ = DM::vertcat({state(0), state(1), state(2)});
    vehicle_positions_.push_back({state(0), state(1), 0.0});

    // Update trajectory
    updateReferenceTrajectory(reference_trajectory);

    // Update parameters
    DM p = SX::vertcat({x0_, SX::reshape(ref_traj_.T(), N_ * n_states_, 1)});
    DM x0 = SX::vertcat({SX::reshape(u0_, N_ * n_controls_, 1), SX::reshape(x_mpc_, (N_ + 1) * n_states_, 1)});
    DM lbg = DM::zeros(G_.size1(), 1);
    DM ubg = DM::zeros(G_.size1(), 1);

    std::map<std::string, DM> args;
    args["p"] = p;
    args["x0"] = x0;
    args["lbg"] = lbg;
    args["ubg"] = ubg;
    args["lbx"] = lbx_;
    args["ubx"] = ubx_;

    // Solve the problem
    auto sol = solver_(args);
    DM sol_x = sol.at("x");
    DM sol_u = sol_x(Slice(0, N_ * n_controls_));
    DM sol_states = sol_x(Slice(N_ * n_controls_, sol_x.size1()));

    sol_u = SX::reshape(sol_u, n_controls_, N_).T();
    sol_states = SX::reshape(sol_states, n_states_, N_ + 1).T();

    // Apply the first control input
    auto u_applied = sol_u(Slice(0, 1), Slice());
    double linear_velocity = static_cast<double>(u_applied(0, 0));
    double angular_velocity = static_cast<double>(u_applied(0, 1));

    // Update control and state predictions
    u0_ = DM::vertcat(std::vector<DM>{
        sol_u(Slice(1, N_), Slice()), 
        sol_u(Slice(N_ - 1, N_), Slice())
    });

    x_mpc_ = DM::vertcat(std::vector<DM>{
        sol_states(Slice(1, N_ + 1), Slice()), 
        sol_states(Slice(N_, N_ + 1), Slice())
    });

    // Calculating distance between vehicle and first reference point
    continous_linear_error_ = std::sqrt(std::pow(reference_trajectory[0](0) - state(0), 2) + 
        std::pow(reference_trajectory[0](1) - state(1), 2));

    double angle_error = std::abs(reference_trajectory[0](2) - state(2));
    
    std::cout << "Distance to first reference point: " << continous_linear_error_ << std::endl;
    std::cout << "Angle error: " << angle_error << std::endl;

    if (continous_linear_error_ < error_threshold_) {
        discrete_linear_error_ = continous_linear_error_;
        return std::make_tuple(linear_velocity, angular_velocity, true);
    }

    return std::make_tuple(linear_velocity, angular_velocity, false);
}
