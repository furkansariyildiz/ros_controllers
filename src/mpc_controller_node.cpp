// mpc_controller_node.cpp

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <casadi/casadi.hpp>
#include <vector>
#include <fstream>

using namespace casadi;
using namespace std::chrono_literals;

class MPCController : public rclcpp::Node
{
public:
    MPCController() : Node("mpc_controller_node")
    {
        // MPC parametreleri
        dt_ = 0.1;       // Zaman adımı
        N_ = 20;         // Öngörü ufku
        frequency_ = 10; // Kontrol döngüsü frekansı (Hz)

        // Durum ve kontrol değişkenleri
        SX x = SX::sym("x");
        SX y = SX::sym("y");
        SX theta = SX::sym("theta");
        states_ = SX::vertcat({x, y, theta});
        n_states_ = states_.size1();

        SX v = SX::sym("v");
        SX omega = SX::sym("omega");
        controls_ = SX::vertcat({v, omega});
        n_controls_ = controls_.size1();

        // Sistem dinamiği (unicycle modeli)
        SX rhs = SX::vertcat({
            v * cos(theta),
            v * sin(theta),
            omega
        });

        // Dinamiği fonksiyon haline getirelim
        f_ = Function("f", {states_, controls_}, {rhs});

        // MPC değişkenleri
        U_ = SX::sym("U", n_controls_, N_);
        X_ = SX::sym("X", n_states_, N_ + 1);
        P_ = SX::sym("P", n_states_ + N_ * n_states_);

        // Amaç fonksiyonu ve kısıtlar
        SX obj = 0;
        std::vector<SX> g;

        // Başlangıç durumu kısıtı
        SX st = X_(Slice(), 0);
        g.push_back(st - P_(Slice(0, n_states_)));

        // Maliyet fonksiyonu (durum ve kontrol ağırlıkları)
        SX Q = SX::diagcat(std::vector<SX>{SX(20.0), SX(20.0), SX(1.0)});
        SX R = SX::diagcat(std::vector<SX>{SX(0.5), SX(0.05)});

        // MPC döngüsü
        for (int k = 0; k < N_; ++k)
        {
            st = X_(Slice(), k);
            SX con = U_(Slice(), k);
            SX st_next = X_(Slice(), k + 1);
            SX f_value = f_(std::vector<SX>{st, con})[0];
            SX st_next_euler = st + dt_ * f_value;
            g.push_back(st_next - st_next_euler);

            // Referans durumu
            SX ref = P_(Slice(n_states_ + k * n_states_, n_states_ + (k + 1) * n_states_));

            // Amaç fonksiyonu güncelleme
            obj = obj + mtimes((st - ref).T(), mtimes(Q, st - ref)) + mtimes(con.T(), mtimes(R, con));
        }

        // Değişkenleri tek bir vektörde birleştirelim
        SX OPT_variables = SX::vertcat(std::vector<SX>{
            SX::reshape(U_, n_controls_ * N_, 1),
            SX::reshape(X_, n_states_ * (N_ + 1), 1)
        });
        
        // Kısıtları birleştirelim
        SX G = SX::vertcat(g);

        // NLP problemi tanımlayalım
        casadi::SXDict nlp_prob = {{"f", obj}, {"x", OPT_variables}, {"g", G}, {"p", P_}};

        // Çözümleyici ayarları
        Dict opts;
        opts["ipopt.max_iter"] = 2000;
        opts["ipopt.print_level"] = 0;
        opts["print_time"] = 0;
        opts["ipopt.acceptable_tol"] = 1e-8;
        opts["ipopt.acceptable_obj_change_tol"] = 1e-6;

        // Çözümleyiciyi oluştur
        solver_ = nlpsol("solver", "ipopt", nlp_prob, opts);

        // Kontrol ve durum tahminleri için başlangıç değerleri
        u0_ = DM::zeros(N_, n_controls_);
        x_mpc_ = DM::zeros(N_ + 1, n_states_);

        // Durum ve kontrol sınırları
        int n_vars = OPT_variables.size1();
        lbx_ = std::vector<double>(n_vars, -std::numeric_limits<double>::infinity());
        ubx_ = std::vector<double>(n_vars, std::numeric_limits<double>::infinity());

        // Kontrol sınırları (TurtleBot3'ün fiziksel sınırlarına göre ayarlayın)
        double v_max = 0.22;  // m/s
        double v_min = -v_max;
        double omega_max = 2.84;  // rad/s
        double omega_min = -omega_max;

        for (int k = 0; k < N_; ++k)
        {
            // Hız sınırları
            lbx_[k * 2] = v_min;
            ubx_[k * 2] = v_max;
            // Açısal hız sınırları
            lbx_[k * 2 + 1] = omega_min;
            ubx_[k * 2 + 1] = omega_max;
        }

        // Mevcut durum
        x0_ = DM::zeros(n_states_);

        // Referans trajektori
        ref_traj_ = DM::zeros(N_, n_states_);

        // ROS2 ayarları
        // Odometri aboneliği
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&MPCController::odom_callback, this, std::placeholders::_1));

        // Kontrol komutu yayıncısı
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        position_ = nav_msgs::msg::Odometry();

        // Veri depolama
        vehicle_positions_.clear();
        reference_trajectory_.clear();

        // Kontrol döngüsünü başlat
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000 / frequency_)),
            std::bind(&MPCController::control_loop, this));
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Odometri verilerini al ve mevcut durumu güncelle
        position_ = *msg;
        auto position = msg->pose.pose.position;
        auto orientation = msg->pose.pose.orientation;

        tf2::Quaternion q(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        x0_(0) = position.x;
        x0_(1) = position.y;
        x0_(2) = yaw;

        vehicle_positions_.push_back({position.x, position.y, position.z});
    }

    void control_loop()
    {
        // Referans trajektoriyi güncelle
        update_reference();

        // Parametreleri güncelle
        DM p = SX::vertcat({x0_, reshape(ref_traj_.T(), N_ * n_states_, 1)});
        DM x0 = SX::vertcat({reshape(u0_, N_ * n_controls_, 1), reshape(x_mpc_, (N_ + 1) * n_states_, 1)});
        DM lbg = DM::zeros(G_.size1(), 1);
        DM ubg = DM::zeros(G_.size1(), 1);

        std::map<std::string, DM> args;
        args["p"] = p;
        args["x0"] = x0;
        args["lbg"] = lbg;
        args["ubg"] = ubg;
        args["lbx"] = lbx_;
        args["ubx"] = ubx_;

        // Problemi çöz
        auto sol = solver_(args);
        DM sol_x = sol.at("x");
        DM sol_u = sol_x(Slice(0, N_ * n_controls_));
        DM sol_states = sol_x(Slice(N_ * n_controls_, sol_x.size1()));

        sol_u = reshape(sol_u, n_controls_, N_).T();
        sol_states = reshape(sol_states, n_states_, N_ + 1).T();

        // İlk kontrol girdisini uygula
        auto u_applied = sol_u(Slice(0, 1), Slice());
        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = static_cast<double>(u_applied(0, 0));
        cmd.angular.z = static_cast<double>(u_applied(0, 1));
        cmd_pub_->publish(cmd);

        // Kontrol ve durum tahminlerini güncelle
        u0_ = DM::vertcat(std::vector<DM>{
            sol_u(Slice(1, N_), Slice()), 
            sol_u(Slice(N_ - 1, N_), Slice())
        });

        x_mpc_ = DM::vertcat(std::vector<DM>{
            sol_states(Slice(1, N_ + 1), Slice()), 
            sol_states(Slice(N_, N_ + 1), Slice())
        });
    }

    void update_reference()
    {
        // Sinüs dalga referans trajektorisi
        double amplitude = 1.0;     // Sinüs dalgasının genliği
        double frequency = 0.5;     // Sinüs dalgasının frekansı
        double v_ref = 0.1;         // x ekseninde referans hız

        if (position_.pose.pose.position.x >= 15.0)
        {
            RCLCPP_INFO(this->get_logger(), "Hedefe ulaşıldı!");
            save_data();
            rclcpp::shutdown();
            return;
        }

        for (int k = 0; k < N_; ++k)
        {
            double t = (k + 1) * dt_;
            double x_ref = x0_(0).scalar() + v_ref * t;
            double y_ref = amplitude * sin(frequency * x_ref);

            // y'nin x'e göre türevi (dy/dx)
            // double dy_dx = amplitude * frequency * cos(frequency * x_ref);
            //double theta_ref = atan2(dy_dx, 1.0);
            double dy_dx = atan2(y_ref - x0_(1).scalar(), x_ref - x0_(0).scalar());
            double theta_ref = atan2(sin(dy_dx), cos(dy_dx));

            ref_traj_(k, 0) = x_ref;
            ref_traj_(k, 1) = y_ref;
            ref_traj_(k, 2) = theta_ref;

            reference_trajectory_.push_back({x_ref, y_ref, 0.0});
        }
    }

    void save_data()
    {
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

    // MPC parametreleri
    double dt_;
    int N_;
    double frequency_;

    // Durum ve kontrol değişkenleri
    SX states_;
    SX controls_;
    int n_states_;
    int n_controls_;

    Function f_;

    SX U_;
    SX X_;
    SX P_;

    Function solver_;
    SX G_;
    std::vector<double> lbx_;
    std::vector<double> ubx_;

    DM u0_;
    DM x_mpc_;
    DM x0_;
    DM ref_traj_;

    // ROS2
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Odometry position_;

    // Veri depolama
    std::vector<std::array<double, 3>> vehicle_positions_;
    std::vector<std::array<double, 3>> reference_trajectory_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MPCController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
