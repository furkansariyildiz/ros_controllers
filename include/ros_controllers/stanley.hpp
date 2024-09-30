#ifndef STANLEY_CONTROLLER_HPP_
#define STANLEY_CONTROLLER_HPP_


#include <iostream>
#include <cmath>



namespace ROS2Controllers
{
    class StanleyController
    {
        private: 
            double line_slope_;

            double line_intercept_;

            double V_;

            double K_;

        protected:

        public:
            StanleyController(double V, double K);

            ~StanleyController();

            void findEquationOfLine(int first_position_x, int first_position_y, 
                                    int second_position_x, int second_position_y);

            double getStanleyControllerSignal(int first_position_x, int first_position_y, 
                                            int second_position_x, int second_position_y, 
                                            int vehicle_position_x, int vehicle_position_y, 
                                            int vehicle_yaw);

    };
} // namespace ROS2Controllers


#endif // STANLEY_CONTROLLER_HPP_