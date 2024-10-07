#ifndef STANLEY_CONTROLLER_HPP_
#define STANLEY_CONTROLLER_HPP_


#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <cmath>
#include <utility>
#include <tuple>

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>


namespace ROS2Controllers
{
    class StanleyController
    {
        private: 
            double line_slope_;

            double line_intercept_;

            double V_;

            double K_;

            double error_threshold_;

            double siganl_limit_;

            double signal_;

        protected:

        public:
            // https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points
            // https://medium.com/roboquest/understanding-geometric-path-tracking-algorithms-stanley-controller-25da17bcc219
            StanleyController(double V, double K, double error_threshold, double signal_limit);

            ~StanleyController();

            std::tuple<double, double, bool> getStanleyControllerSignal(double next_waypoint_x, double next_waypoint_y, 
                                              double previous_waypoint_x, double previous_waypoint_y,
                                              double vehicle_position_x, double vehicle_position_y,
                                              double vehicle_yaw);
    };
} // namespace ROS2Controllers


#endif // STANLEY_CONTROLLER_HPP_