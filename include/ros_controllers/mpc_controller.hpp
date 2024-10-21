#ifndef MPC_CONTROLLER_HPP_
#define MPC_CONTROLLER_HPP_


#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <vector>
#include <iostream>
#include <tuple>
#include <eigen3/Eigen/Dense>
#include <casadi/casadi.hpp>

// Messages
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>


using namespace casadi;


namespace ROS2Controllers
{
    class MPCController
    {
        private:
            int horizon_;

            double vehicle_base_width_;

            double error_threshold_;

            double signal_limit_;

            double dt_;

            SX Q_;
            SX R_;

            Function solver_;
            Dict opts_;

            casadi::DM previous_solution_;

            bool previous_solution_exists_;
            
        public:
            MPCController(const int horizon, const double vehicle_base_width,
                const double error_threshold, const double signal_limit, const double dt);

            ~MPCController();

            void setupOptimizationProblem();

            casadi::DM shiftSolution(const casadi::DM &previous_solution, int n_controls, int n_states);

            std::tuple<double, double> getMPCControllerSignal(const double vehicle_position_x, const double vehicle_position_y, 
                const double vehicle_yaw, const std::vector<geometry_msgs::msg::PoseStamped> path);
    }; // namespace ROS2Controllers
}


#endif // MPC_CONTROLLER_HPP_