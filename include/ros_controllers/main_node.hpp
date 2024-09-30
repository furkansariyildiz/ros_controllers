#ifndef MAIN_NODE_HPP_
#define MAIN_NODE_HPP_


#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

// ROS2 Controller Libraries
#include "ros_controllers/stanley.hpp"



class MainNode : public rclcpp::Node
{
    private:
        std::unique_ptr<ROS2Controllers::StanleyController> stanley_controller_;

    protected:

    public:
        MainNode();

        ~MainNode();

        void init();

        void controlManager();
};



#endif