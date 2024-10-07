#ifndef PURE_PURSUITE_HPP_
#define PURE_PURSUITE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <cmath>
#include <tuple>


namespace ROS2Controllers
{
    class PurePursuiteController
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
            PurePursuiteController(double Kp, double Ki, double Kd, 
                        double error_threshold, const double signal_limit);
    
            ~PurePursuiteController();

            std::tuple<double, double, bool> getPurePursuiteSignal(double error, double dt);
    };
} // namespace ROS2Controllers



#endif // PURE_PURSUITE_HPP