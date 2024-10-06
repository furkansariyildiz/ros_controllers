#ifndef MAIN_NODE_HPP_
#define MAIN_NODE_HPP_


#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

// ROS2 Controller Libraries
#include "ros_controllers/stanley.hpp"
#include "ros_controllers/pid.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>



class MainNode : public rclcpp::Node
{
    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;

        std::unique_ptr<ROS2Controllers::StanleyController> stanley_controller_;
        std::unique_ptr<ROS2Controllers::PIDController> linear_velocity_pid_controller_;
        std::unique_ptr<ROS2Controllers::PIDController> angular_velocity_pid_controller_;
    
        nav_msgs::msg::Odometry odometry_message_;

        geometry_msgs::msg::Twist cmd_vel_message_;

        nav_msgs::msg::Path path_;

        double linear_velocity_error_;

        double angular_velocity_error_;

        double linear_velocity_signal_;

        double angular_velocity_signal_;

    protected:

    public:
        MainNode();

        ~MainNode();

        void init();

        void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr message);

        void controlManager();

        void stanley();

        void PID();
};



#endif