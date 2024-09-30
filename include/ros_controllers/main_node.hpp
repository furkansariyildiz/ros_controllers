#ifndef MAIN_NODE_HPP_
#define MAIN_NODE_HPP_


#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

// ROS2 Controller Libraries
#include "ros_controllers/stanley.hpp"



class MainNode : public rclcpp::Node 
{
    private:

    protected:

    public:
        MainNode();

        ~MainNode();
};



#endif