#include "ros_controllers/mpc.hpp"

MPCController::MPCController(double dt, int horizon, double L)
    : dt_(dt), horizon_(horizon), L_(L) {

    // Ağırlık matrislerinin tanımlanması
    Q_ = casadi::SX::diag(casadi::SX({1.0, 1.0, 0.1}));   // x, y, theta için
    R_ = casadi::SX::diag(casadi::SX({0.5, 0.5}));        // v, delta için

    // opts_ sözlüğünün tanımlanması
    opts_ = casadi::Dict();
    opts_["ipopt.print_level"] = 0;
    opts_["print_time"] = false;
    opts_["verbose"] = false;

    // Optimizasyon probleminin kurulması
    setupOptimizationProblem();
}

MPCController::~MPCController() {}

void MPCController::setupOptimizationProblem() {
    using casadi::SX;
    using casadi::Slice;
    using casadi::Function;

    // Durum ve kontrol değişkenlerinin tanımlanması
    SX x = SX::sym("x");
    SX y = SX::sym("y");
    SX theta = SX::sym("theta");
    SX state = SX::vertcat({x, y, theta});

    SX v = SX::sym("v");
    SX delta = SX::sym("delta");
    SX control = SX::vertcat({v, delta});

    // Sistem dinamikleri
    SX rhs = SX::vertcat({
        v * SX::cos(theta),
        v * SX::sin(theta),
        (v / L_) * SX::tan(delta)
    });

    // Discrete-time dynamics
    Function f = Function("f", {state, control}, {state + rhs * dt_});

    // Optimizasyon değişkenleri
    SX U = SX::sym("U", 2, horizon_);              // Kontroller
    SX X = SX::sym("X", 3, horizon_ + 1);          // Durumlar
    SX P = SX::sym("P", 3 + 3 * (horizon_ + 1));   // Parametreler (başlangıç durumu ve referanslar)

    // Amaç fonksiyonu ve kısıtlar
    SX obj = 0;
    SX g = SX::zeros(3 * (horizon_ + 1), 1); // Kısıtlar (başlangıç durumu dahil)

    // Başlangıç durumu kısıtı
    g(Slice(0, 3), 0) = X(Slice(), 0) - P(Slice(0, 3));

    // Optimizasyon döngüsü
    for (int k = 0; k < horizon_; ++k) {
        // Durum referansları
        SX X_ref = P(Slice(3 + 3 * k, 3 + 3 * (k + 1)));

        // Amaç fonksiyonu
        SX state_error = X(Slice(), k) - X_ref;
        SX control_k = U(Slice(), k);
        obj += SX::mtimes(state_error.T(), SX::mtimes(Q_, state_error)) + SX::mtimes(control_k.T(), SX::mtimes(R_, control_k));

        // Sistem dinamikleri
        std::vector<SX> args = {X(Slice(), k), U(Slice(), k)};
        SX X_next = f(args).at(0);

        // Kısıtların biriktirilmesi
        g(Slice(3 * (k + 1), 3 * (k + 2)), 0) = X_next - X(Slice(), k + 1);
    }

    // Son adım için durum referansı
    SX X_ref = P(Slice(3 + 3 * horizon_, 3 + 3 * (horizon_ + 1)));
    SX state_error = X(Slice(), horizon_) - X_ref;
    obj += SX::mtimes(state_error.T(), SX::mtimes(Q_, state_error));

    // NLP Probleminin Tanımlanması
    SX OPT_variables = SX::vertcat({SX::reshape(U, 2 * horizon_, 1), SX::reshape(X, 3 * (horizon_ + 1), 1)});

    // NLP fonksiyonunun oluşturulması
    casadi::SXDict nlp = {
        {"x", OPT_variables},
        {"f", obj},
        {"g", g},
        {"p", P}
    };

    // Çözücünün Oluşturulması
    solver_ = casadi::nlpsol("solver", "ipopt", nlp, opts_);
}


std::tuple<double, double> MPCController::computeControlSignal(
    const Eigen::VectorXd& state,
    const std::vector<Eigen::VectorXd>& reference_trajectory) {

    // Başlangıç durumunun ve referansların hazırlanması
    std::vector<double> p_data;
    p_data.reserve(3 + 3 * (horizon_ + 1));

    // Başlangıç durumu
    for (int i = 0; i < 3; ++i) {
        p_data.push_back(state(i));
    }

    // Referans yörünge
    for (int k = 0; k < horizon_ + 1; ++k) {
        const Eigen::VectorXd& ref = reference_trajectory[k];
        p_data.push_back(ref(0)); // x_ref
        p_data.push_back(ref(1)); // y_ref
        p_data.push_back(ref(2)); // theta_ref
    }

    // DM tipine dönüştürme
    casadi::DM p = casadi::DM(p_data);

    // Başlangıç tahminleri
    casadi::DM U0 = casadi::DM::zeros(2, horizon_);
    std::vector<double> state_vector(state.data(), state.data() + state.size());
    casadi::DM state_dm = casadi::DM(state_vector);
    casadi::DM X0 = casadi::DM::repmat(state_dm, 1, horizon_ + 1);

    // Optimizasyon değişkenlerinin düzleştirilmesi
    casadi::DM OPT_variables = casadi::DM::vertcat({
        casadi::DM::reshape(U0, 2 * horizon_, 1),
        casadi::DM::reshape(X0, 3 * (horizon_ + 1), 1)
    });

    // Kısıt limitleri
    int nlp_g = 3 * (horizon_ + 1);
    casadi::DM lbg = casadi::DM::zeros(nlp_g);
    casadi::DM ubg = casadi::DM::zeros(nlp_g);

    // Optimizasyonun Çalıştırılması
    std::map<std::string, casadi::DM> arg;

    arg["x0"] = OPT_variables;
    arg["lbg"] = lbg;
    arg["ubg"] = ubg;
    arg["p"] = p;

    auto res = solver_(arg);

    // Çözülen kontrol sinyallerinin alınması
    casadi::DM sol = res.at("x");
    casadi::DM u = sol(casadi::Slice(0, 2 * horizon_));

    double optimal_velocity = static_cast<double>(u(0));
    double optimal_steering_angle = static_cast<double>(u(1));

    return std::make_tuple(optimal_velocity, optimal_steering_angle);
}

