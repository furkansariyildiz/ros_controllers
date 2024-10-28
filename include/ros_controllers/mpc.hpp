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
        public:
            /**
             * 
             */
            MPCController(double dt, int horizon, double L);
            
            /**
             * 
             */
            ~MPCController();

            /**
             * 
             */
            std::tuple<double, double> computeControlSignal(
                const Eigen::VectorXd& state,
                const std::vector<Eigen::VectorXd>& reference_trajectory);
    };
} // namespace ROS2Controllers


#endif // MPC_CONTROLLER_HPP
