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


namespace ROS2Controllers {
    class MPCController {
        private:
            /**
             * 
             */
            void setupOptimizationProblem();         
            
            /**
             * 
             */
            int horizon_;        
            
            /**
             * 
             */
            double L_;           

            std::vector<double> Q_vector_;

            std::vector<double> R_vector_;


            /**
             * 
             */
            SX Q_;              
            
            /**
             * 
             */
            SX R_;
            
            /**
             * 
             */
            Dict opts_;

            SX g_concat_;

            SX OPT_variables_;

            double signal_limit_linear_velocity_;

            double signal_limit_angular_velocity_;

            double error_threshold_;

            double discrete_error_;

            double continous_error_;

            double dt_;

            int N_;

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

            std::vector<std::array<double, 3>> vehicle_positions_;
            std::vector<std::array<double, 3>> reference_trajectory_;

        public:
            /**
             * 
             */
            MPCController(double dt, int horizon, double L, std::vector<double> Q, std::vector<double> R, double signal_limit_linear_velocity, 
                double signal_limit_angular_velocity, double error_threshold);
            
            /**
             * 
             */
            ~MPCController();

            double getDiscreteError();

            double getContinousError();
            
            void updateReferenceTrajectory(const std::vector<Eigen::VectorXd>& reference_trajectory);

            /**
             * 
             */
            std::tuple<double, double, bool> computeControlSignal(
                const Eigen::VectorXd& state,
                const std::vector<Eigen::VectorXd>& reference_trajectory);
    };
} // namespace ROS2Controllers


#endif // MPC_CONTROLLER_HPP
