#ifndef PID_CONTROLLER_HPP_
#define PID_CONTROLLER_HPP_


#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <cmath>

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>



namespace ROS2Controllers
{
    class PIDController
    {
        private:
            std::shared_ptr<rclcpp::Node> node_;

            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

            double Kp_;

            double Ki_;

            double Kd_;

            double previous_error_;

            double P_, I_, D_;

            const double error_threshold_;

            const double signal_limit_;

            const nav_msgs::msg::Path path_;

        protected:

        public:
            PIDController(const std::shared_ptr<rclcpp::Node> &node, double Kp, double Ki, 
                          double Kd, double error_threshold, const double signal_limit, 
                          const nav_msgs::msg::Path path);
    
            ~PIDController();

            double getPIDControllerSignal(double error, double dt);

            void run();
    };
} // namespace ROS2Controllers


#endif // PID_CONTROLLER_HPP_