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

    // testFunction();

    // Setup the optimization problem
    setupOptimizationProblem();
}



ROS2Controllers::MPCController::~MPCController() {

}



void ROS2Controllers::MPCController::testFunction() {
    // Zaman adımı ve öngörü ufku
    double dt = 0.1;  // Zaman adımı
    int N = 30;       // Öngörü ufku

    // Durum ve kontrol değişkenleri
    SX x = SX::sym("x");
    SX y = SX::sym("y");
    SX theta = SX::sym("theta");
    SX states = SX::vertcat({x, y, theta});
    int n_states = states.size1();

    SX v = SX::sym("v");
    SX omega = SX::sym("omega");
    SX controls = SX::vertcat({v, omega});
    int n_controls = controls.size1();

    // Sistem dinamiği (unicycle modeli)
    SX rhs = SX::vertcat({
        v * SX::cos(theta),
        v * SX::sin(theta),
        omega
    });

    // Dinamiği fonksiyon haline getirelim
    Function f = Function("f", {states, controls}, {rhs});

    // MPC değişkenleri
    SX U = SX::sym("U", n_controls, N);          // Kontrol değişkenleri
    SX X = SX::sym("X", n_states, N + 1);        // Durum değişkenleri
    SX P = SX::sym("P", n_states + N * n_states);  // Parametre vektörü

    // Amaç fonksiyonu ve kısıtlar
    SX obj = 0;     // Amaç fonksiyonu
    std::vector<SX> g;      // Kısıtlar listesi

    // Başlangıç durumu kısıtı
    SX st = X(Slice(), 0);
    g.push_back(st - P(Slice(0, n_states)));  // x0 = x_initial

    // Maliyet fonksiyonu ağırlıkları
    SX Q = SX::diag(SX({20, 20, 0.05}));
    SX R = SX::diag(SX({0.5, 0.05}));

    // MPC döngüsü
    for (int k = 0; k < N; ++k) {
        st = X(Slice(), k);
        SX con = U(Slice(), k);
        SX st_next = X(Slice(), k + 1);
        std::vector<SX> args = {st, con};
        SX f_value = f(args)[0];
        SX st_next_euler = st + dt * f_value;
        g.push_back(st_next - st_next_euler);  // Sistem dinamiği kısıtı

        // Referans durumu
        SX ref = P(Slice(n_states + k * n_states, n_states + (k + 1) * n_states));

        // Amaç fonksiyonu güncelleme
        obj = obj + SX::mtimes((st - ref).T(), SX::mtimes(Q, (st - ref))) + SX::mtimes(con.T(), SX::mtimes(R, con));
    }

    // Değişkenleri tek bir vektörde birleştirelim
    SX OPT_variables = SX::vertcat({
        SX::reshape(U, N * n_controls, 1),
        SX::reshape(X, (N + 1) * n_states, 1)
    });

    // Kısıtları birleştirelim
    SX g_concat = vertcat(g);

    // NLP problemi tanımlayalım
    casadi::SXDict nlp_prob = {{"f", obj}, {"x", OPT_variables}, {"g", g_concat}, {"p", P}};

    // Çözümleyici ayarları
    Dict opts;
    opts["ipopt.max_iter"] = 2000;
    opts["ipopt.print_level"] = 0;
    opts["print_time"] = 0;
    opts["ipopt.acceptable_tol"] = 1e-8;
    opts["ipopt.acceptable_obj_change_tol"] = 1e-6;

    // Çözümleyiciyi oluştur
    Function solver = nlpsol("solver", "ipopt", nlp_prob, opts);

    // Başlangıç durumu
    std::vector<double> x0 = {0.0, 0.0, 0.0};  // x, y, theta

    // Başlangıç tahminleri
    std::vector<double> u0(N * n_controls, 0.0);          // Kontrol değişkenleri için başlangıç tahmini
    std::vector<double> x_mpc((N + 1) * n_states, 0.0);   // Durum değişkenleri için başlangıç tahmini
    for (int i = 0; i < n_states; ++i) {
        x_mpc[i] = x0[i];
    }

    // MPC çözümleme döngüsü
    double sim_tim = 20.0;  // Simülasyon süresi (saniye)
    double t = 0.0;
    int mpc_iter = 0;
    std::vector<std::vector<double>> xx;        // Gerçek yol
    std::vector<std::vector<double>> u_cl;      // Kontrol girdileri
    std::vector<std::vector<double>> ref_traj_full;  // Tam referans yolu

    double frequency = 0.5;  // Sinüs dalgasının frekansı
    double amplitude = 1.0;  // Sinüs dalgasının genliği

    while (mpc_iter * dt < sim_tim) {
        // Referans trajektoriyi güncelle
        std::vector<double> ref_traj(N * n_states, 0.0);
        for (int k = 0; k < N; ++k) {
            // x pozisyonu artıyor
            double x_ref = x0[0] + (k + 1) * dt * 1.0;
            ref_traj[k * n_states + 0] = x_ref;

            // y pozisyonu sinüs dalgası şeklinde
            double y_ref = amplitude * sin(frequency * x_ref);
            ref_traj[k * n_states + 1] = y_ref;

            // Theta açısı yolun eğimine göre ayarlanıyor
            double dx = 1.0;  // x artışı sabit
            double dy = amplitude * frequency * cos(frequency * x_ref);  // y'nin türevi
            double theta_ref = atan2(dy, dx);
            ref_traj[k * n_states + 2] = theta_ref;

            // Tam referans yolunu biriktir
            if (k == 0) {
                if (mpc_iter == 0) {
                    ref_traj_full.push_back({x_ref, y_ref});
                } else {
                    ref_traj_full.push_back({x_ref, y_ref});
                }
            }
        }

        // Parametreleri güncelle
        std::vector<double> p_vals;
        p_vals.insert(p_vals.end(), x0.begin(), x0.end());
        p_vals.insert(p_vals.end(), ref_traj.begin(), ref_traj.end());

        // Başlangıç değerleri
        std::vector<double> x_init;
        x_init.insert(x_init.end(), u0.begin(), u0.end());
        x_init.insert(x_init.end(), x_mpc.begin(), x_mpc.end());

        // Kısıt sınırları
        int nlp_g = g_concat.size().first;
        std::vector<double> lbg(nlp_g, 0.0);
        std::vector<double> ubg(nlp_g, 0.0);

        // Optimizasyon değişkenlerinin sınırları
        int nlp_x = OPT_variables.size().first;
        std::vector<double> lbx(nlp_x, -1e20);
        std::vector<double> ubx(nlp_x, 1e20);

        // Problemi çöz
        DMDict arg = {{"x0", x_init}, {"lbx", lbx}, {"ubx", ubx}, {"lbg", lbg}, {"ubg", ubg}, {"p", p_vals}};
        DMDict sol = solver(arg);

        // Çözümleri al
        std::vector<double> sol_vector = std::vector<double>(static_cast<std::vector<double>>(sol.at("x")));
        std::vector<double> sol_u(sol_vector.begin(), sol_vector.begin() + N * n_controls);
        std::vector<double> sol_x(sol_vector.begin() + N * n_controls, sol_vector.end());

        // İlk kontrol girdisini uygula
        std::vector<double> u_applied(sol_u.begin(), sol_u.begin() + n_controls);
        u_cl.push_back(u_applied);

        // Sistemi güncelle
        // Argümanları bir vektörde toplayın
        std::vector<DM> args = {x0, u_applied};
        DM f_value = f(args)[0];

        // Fonksiyonu çağırın
        std::vector<double> f_vec = std::vector<double>(f_value);

        // Sistemi güncelleyin
        for (int i = 0; i < n_states; ++i) {
            x0[i] += dt * f_vec[i];
        }
        t += dt;
        xx.push_back(x0);

        // Kontrol ve durum tahminlerini güncelle
        u0.erase(u0.begin(), u0.begin() + n_controls);
        u0.insert(u0.end(), sol_u.end() - n_controls, sol_u.end());

        x_mpc.erase(x_mpc.begin(), x_mpc.begin() + n_states);
        x_mpc.insert(x_mpc.end(), sol_x.end() - n_states, sol_x.end());

        mpc_iter += 1;
    }

    // Sonuçları dosyaya yazalım
    std::ofstream outfile("/home/furkan/controller_ws/src/ros_controllers/results/mpc_result.csv");
    outfile << "x,y,ref_x,ref_y\n";
    for (int i = 0; i < xx.size(); ++i) {
        outfile << xx[i][0] << "," << xx[i][1] << "," << ref_traj_full[i][0] << "," << ref_traj_full[i][1] << "\n";
    }
    outfile.close();

    std::cout << "MPC Simülasyonu Tamamlandı. Sonuçlar 'mpc_result.csv' dosyasına kaydedildi." << std::endl;

}


double ROS2Controllers::MPCController::getDiscreteLinearError() {
    return discrete_linear_error_;
}



double ROS2Controllers::MPCController::getContinousLinearError() {
    return continous_linear_error_;
}



void ROS2Controllers::MPCController::setupOptimizationProblem() {
    SX x = SX::sym("x");
    SX y = SX::sym("y");
    SX theta = SX::sym("theta");
    SX states = SX::vertcat({x, y, theta});
    n_states_ = states.size1();

    SX v = SX::sym("v");
    SX w = SX::sym("w");
    SX controls = SX::vertcat({v, w});
    n_controls_ = controls.size1();

    // System dynamics (unicycle model)
    SX rhs = SX::vertcat({
        v * SX::cos(theta),
        v * SX::sin(theta),
        w
    });

    // Convert the system dynamics to a function
    f_ = Function("f", {states, controls}, {rhs});

    // MPC variables
    SX U = SX::sym("U", n_controls_, horizon_);          // Control variables
    SX X = SX::sym("X", n_states_, horizon_ + 1);        // State variables
    SX P = SX::sym("P", n_states_ + 3 * (horizon_));  // Parameter vector

    // Objective function and constraints
    SX obj = 0;     // Objective function
    std::vector<SX> g;      // List of constraints

    // Initial state constraint
    SX st = X(Slice(), 0);
    g.push_back(st - P(Slice(0, n_states_)));  // x0 = x_initial

    // MPC loop
    for (int k = 0; k < horizon_; ++k) {
        st = X(Slice(), k);
        SX con = U(Slice(), k);
        SX st_next = X(Slice(), k + 1);

        std::vector<SX> args = {st, con};
        SX f_value = f_(args)[0];
        SX st_next_euler = st + dt_ * f_value;
        g.push_back(st_next - st_next_euler);  // Sistem dinamiği kısıtı

        // Reference state
        SX ref = P(Slice(n_states_ + k * n_states_, n_states_ + (k + 1) * n_states_));

        // Update the objective function
        obj = obj + SX::mtimes((st - ref).T(), SX::mtimes(Q_, (st - ref))) + SX::mtimes(con.T(), SX::mtimes(R_, con));
    }

    // Create the concatenated constraint vector
    OPT_variables_ = SX::vertcat({
        SX::reshape(U, horizon_ * n_controls_, 1),
        SX::reshape(X, (horizon_ + 1) * n_states_, 1)
    });

    g_concat_ = vertcat(g);

    // Create the NLP problem
    casadi::SXDict nlp_prob = {{"f", obj}, {"x", OPT_variables_}, {"g", g_concat_}, {"p", P}};

    // Create the solver
    casadi::Dict opts;
    opts["ipopt.max_iter"] = 2000;
    opts["ipopt.print_level"] = 0;
    opts["print_time"] = 0;
    opts["ipopt.acceptable_tol"] = 1e-8;
    opts["ipopt.acceptable_obj_change_tol"] = 1e-6;

    // Create the solver function
    solver_ = casadi::nlpsol("solver", "ipopt", nlp_prob, opts);
}



std::tuple<double, double, bool> ROS2Controllers::MPCController::computeControlSignal(
    const Eigen::VectorXd& state,
    const std::vector<Eigen::VectorXd>& reference_trajectory) {

    if (f_.is_null()) {
        setupOptimizationProblem();
    }

    std::vector<double> x0 = {state(0), state(1), state(2)};  // x, y, theta

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

    std::vector<double> u0(horizon_ * n_controls_, 0.0);
    std::vector<double> x_mpc((horizon_ + 1) * n_states_, 0.0);
    
    // State initialization
    for (int i=0; i<n_states_; ++i) {
        x_mpc[i] = x0[i];
    }

    std::vector<std::vector<double>> xx;
    std::vector<std::vector<double>> u_cl;
    std::vector<double> reference_trajectory_horizon(horizon_ * n_states_, 0.0); 

    // Reference trajectory initialization via horizon
    for (int k=0; k<horizon_; ++k) {
        reference_trajectory_horizon[k * n_states_ + 0] = reference_trajectory[k](0);

        reference_trajectory_horizon[k * n_states_ + 1] = reference_trajectory[k](1);

        reference_trajectory_horizon[k * n_states_ + 2] = reference_trajectory[k](2);
    }

    // Parameters vector
    std::vector<double> p_values;
    p_values.insert(p_values.end(), x0.begin(), x0.end());
    p_values.insert(p_values.end(), reference_trajectory_horizon.begin(), reference_trajectory_horizon.end());

    // Initial state
    std::vector<double> x_init;
    x_init.insert(x_init.end(), u0.begin(), u0.end());
    x_init.insert(x_init.end(), x_mpc.begin(), x_mpc.end());

    // Constraints limits
    int nlp_x = OPT_variables_.size().first;
    int nlp_g = g_concat_.size().first;
    casadi::DM lbg = casadi::DM::zeros(nlp_g);
    casadi::DM ubg = casadi::DM::zeros(nlp_g);
    casadi::DM lbx = -casadi::DM::inf(nlp_x);
    casadi::DM ubx = casadi::DM::inf(nlp_x);
    int nu = 2;

    for (int k=0; k<horizon_; ++k) {
        int idx = k * nu;
        lbx(idx) = -signal_limit_linear_velocity_;
        ubx(idx) = signal_limit_linear_velocity_;

        lbx(idx + 1) = -signal_limit_angular_velocity_;
        ubx(idx + 1) = signal_limit_angular_velocity_;
    }


    DMDict arg = {{"x0", x_init}, {"lbx", lbx}, {"ubx", ubx}, {"lbg", lbg}, {"ubg", ubg}, {"p", p_values}};
    DMDict sol = solver_(arg);

    // Get the solution
    casadi::DM sol_x = sol.at("x");

    // Extract the control inputs
    std::vector<double> sol_vector = static_cast<std::vector<double>>(sol_x);
    std::vector<double> sol_u(sol_vector.begin(), sol_vector.begin() + horizon_ * 2);

    // Apply the first control input
    double optimal_velocity = sol_u[0];
    double optimal_steering_angle = sol_u[1];

    std::vector<double> u_applied = {optimal_velocity, optimal_steering_angle};
    casadi::DM u_applied_dm = casadi::DM(u_applied);

    casadi::DM x0_dm = casadi::DM(x0);

    // Predict the system dynamics
    std::vector<casadi::DM> args = {x0_dm, u_applied_dm};
    casadi::DM f_value = f_(args)[0];

    // Calculating distance between vehicle and first reference point
    continous_linear_error_ = std::sqrt(std::pow(reference_trajectory[0](0) - state(0), 2) + 
        std::pow(reference_trajectory[0](1) - state(1), 2));

    double angle_error = std::abs(reference_trajectory[0](2) - state(2));
    
    std::cout << "Distance to first reference point: " << continous_linear_error_ << std::endl;
    std::cout << "Angle error: " << angle_error << std::endl;

    if (continous_linear_error_ < error_threshold_) {
        discrete_linear_error_ = continous_linear_error_;
        return std::make_tuple(optimal_velocity, optimal_steering_angle, true);
    }

    return std::make_tuple(optimal_velocity, optimal_steering_angle, false);
}



/*
std::tuple<double, double, bool> ROS2Controllers::MPCController::computeControlSignal(
    const Eigen::VectorXd& state,
    const std::vector<Eigen::VectorXd>& reference_trajectory) {

    // Durum ve referans trajektori boyutlarını kontrol et
    if (state.size() < 3) {
        std::cout << "Error: State vector size is less than 3!" << std::endl;
        return std::make_tuple(0.0, 0.0, false);
    } else if (reference_trajectory.size() < horizon_ + 1) {
        std::cout << "Error: Reference trajectory size is less than expected!" << std::endl;
        return std::make_tuple(0.0, 0.0, false);
    }

    // Parametreleri hazırlayın (başlangıç durumu ve referans trajektori)
    std::vector<double> p_data;
    p_data.reserve(3 + horizon_ * 3);

    // Başlangıç durumu
    for (int i = 0; i < 3; ++i) {
        p_data.push_back(state(i));
    }

    // Referans trajektori
    for (int k = 0; k < horizon_; ++k) {
        const Eigen::VectorXd& ref = reference_trajectory[k];
        p_data.push_back(ref(0)); // x_ref
        p_data.push_back(ref(1)); // y_ref
        p_data.push_back(ref(2)); // theta_ref
    }

    // Parametreleri CasADi DM tipine dönüştürün
    casadi::DM p = casadi::DM(p_data);

    // Başlangıç tahminleri (kontrol ve durum değişkenleri)
    // Eğer önceki kontrol girdilerini ve durum tahminlerini saklıyorsanız, bunları kullanabilirsiniz
    // Aksi halde sıfır matrisler kullanabilirsiniz
    std::vector<double> u0(horizon_ * 2, 0.0); // Kontrol girdileri için başlangıç tahmini
    std::vector<double> x0((horizon_ + 1) * 3, 0.0); // Durum değişkenleri için başlangıç tahmini

    // Başlangıç durumunu durum tahmininin ilk elemanına yerleştirin
    for (int i = 0; i < 3; ++i) {
        x0[i] = state(i);
    }

    // Optimizasyon değişkenlerini birleştirin
    std::vector<double> OPT_variables;
    OPT_variables.reserve(u0.size() + x0.size());
    OPT_variables.insert(OPT_variables.end(), u0.begin(), u0.end());
    OPT_variables.insert(OPT_variables.end(), x0.begin(), x0.end());

    // Kısıt sınırları
    int nlp_x = OPT_variables.size();
    int nlp_g = g_concat_.size().first;

    // Kontrol girdileri sınırları
    double v_min = 0.0;
    double v_max = signal_limit_linear_velocity_;
    double w_min = -signal_limit_angular_velocity_;
    double w_max = signal_limit_angular_velocity_;

    // Optimizasyon değişkenlerinin sınırları
    std::vector<double> lbx(nlp_x, -1e20);
    std::vector<double> ubx(nlp_x, 1e20);

    // Kontrol girdileri için sınırları ayarlayın
    for (int k = 0; k < horizon_; ++k) {
        int idx = k * 2;
        lbx[idx] = v_min;
        ubx[idx] = v_max;

        lbx[idx + 1] = w_min;
        ubx[idx + 1] = w_max;
    }

    // Kısıtlar için sınırlar
    std::vector<double> lbg(nlp_g, 0.0);
    std::vector<double> ubg(nlp_g, 0.0);

    // Optimizasyon problemini çözmek için argümanları hazırlayın
    std::map<std::string, casadi::DM> args;
    args["x0"] = OPT_variables;
    args["lbx"] = lbx;
    args["ubx"] = ubx;
    args["lbg"] = lbg;
    args["ubg"] = ubg;
    args["p"] = p;

    // Optimizasyon problemini çözün
    casadi::DMDict sol = solver_(args);

    // Çözümü alın
    casadi::DM sol_x = sol.at("x");

    // Kontrol girdilerini çıkarın
    std::vector<double> sol_vector = static_cast<std::vector<double>>(sol_x);
    std::vector<double> sol_u(sol_vector.begin(), sol_vector.begin() + horizon_ * 2);

    // İlk kontrol girdilerini alın
    double optimal_velocity = sol_u[0];
    double optimal_steering_angle = sol_u[1];

    // Aracın ilk referans noktası ile arasındaki mesafeyi hesaplayın
    continous_linear_error_ = std::sqrt(std::pow(reference_trajectory[0](0) - state(0), 2) +
                                        std::pow(reference_trajectory[0](1) - state(1), 2));

    std::cout << "Distance to first reference point: " << continous_linear_error_ << std::endl;

    bool goal_reached = continous_linear_error_ < error_threshold_;

    return std::make_tuple(optimal_velocity, optimal_steering_angle, goal_reached);
}
*/