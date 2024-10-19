#ifndef MPC_CONTROLLER_HPP
#define MPC_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <casadi/casadi.hpp>
#include <casadi/core/core.hpp>
#include <casadi/core/sparsity_interface.hpp>
#include <eigen3/Eigen/Dense>
#include <tuple>
#include <vector>
#include <iostream>

using namespace casadi;

class MPCController {
public:
    MPCController(double dt, int horizon, double L);
    ~MPCController();

    std::tuple<double, double> computeControlSignal(
        const Eigen::VectorXd& state,
        const std::vector<Eigen::VectorXd>& reference_trajectory);

private:
    double dt_;          // Zaman adımı
    int horizon_;        // Öngörü horizonu
    double L_;           // Dingil mesafesi

    // Optimizasyon parametreleri
    SX Q_;               // Durum hata ağırlıkları
    SX R_;               // Kontrol girdi ağırlıkları

    // Optimizasyon değişkenleri ve fonksiyonu
    Function solver_;
    Dict opts_;

    void setupOptimizationProblem();
};

#endif // MPC_CONTROLLER_HPP
