#ifndef STANLEY_CONTROLLER_HPP_
#define STANLEY_CONTROLLER_HPP_


#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <cmath>

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>


namespace ROS2Controllers
{
    class StanleyController
    {
        private: 
            std::shared_ptr<rclcpp::Node> node_;

            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

            double line_slope_;

            double line_intercept_;

            double V_;

            double K_;

            double error_threshold_;

            const nav_msgs::msg::Path &path_;

        protected:

        public:
            StanleyController(const std::shared_ptr<rclcpp::Node> &node, double V, double K, 
                              double error_threshold, const nav_msgs::msg::Path &path);

            ~StanleyController();

            void findEquationOfLine(int first_position_x, int first_position_y, 
                                    int second_position_x, int second_position_y);

            double getStanleyControllerSignal(int vehicle_position_x, int vehicle_position_y, 
                                              int vehicle_yaw);

            void run();
    };
} // namespace ROS2Controllers


#endif // STANLEY_CONTROLLER_HPP_