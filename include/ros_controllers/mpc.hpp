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
            double dt_;          
            
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
            Function solver_;
            
            /**
             * 
             */
            Dict opts_;

            double signal_limit_linear_velocity_;

            double signal_limit_angular_velocity_;

            double error_threshold_;

            double discrete_linear_error_;

            double continous_linear_error_;

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

            double getDiscreteLinearError();

            double getContinousLinearError();
            
            /**
             * 
             */
            std::tuple<double, double, bool> computeControlSignal(
                const Eigen::VectorXd& state,
                const std::vector<Eigen::VectorXd>& reference_trajectory);
    };
} // namespace ROS2Controllers


#endif // MPC_CONTROLLER_HPP
