#ifndef PID_CONTROLLER_HPP_
#define PID_CONTROLLER_HPP_


#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <cmath>
#include <utility>


namespace ROS2Controllers
{
    class PIDController
    {
        private:
            double Kp_;

            double Ki_;

            double Kd_;

            double previous_error_;

            double P_, I_, D_;

            const double error_threshold_;

            const double signal_limit_;

        protected:

        public:
            PIDController(double Kp, double Ki, double Kd, 
                        double error_threshold, const double signal_limit);
    
            ~PIDController();

            std::pair<double, bool> getPIDControllerSignal(double error, double dt);
    };
} // namespace ROS2Controllers


#endif // PID_CONTROLLER_HPP_