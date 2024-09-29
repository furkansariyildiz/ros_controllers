#ifndef STANLEY_CONTROLLER_HPP_
#define STANLEY_CONTROLLER_HPP_


#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>



namespace ROS2Controllers
{
    class StanleyController
    {
        private: 

        protected:

        public:
            StanleyController();

            ~StanleyController();

            void findEquationOfLine(int first_position_x, int first_position_y, 
                                    int second_position_x, int second_position_y);

            void getStanleyControllerSignal(int first_position_x, int first_position_y, 
                                            int second_position_x, int second_position_y, 
                                            int vehicle_position_x, int vehicle_position_y, 
                                            int robot_yaw);
    };
} // namespace ROS2Controllers


#endif // STANLEY_CONTROLLER_HPP_