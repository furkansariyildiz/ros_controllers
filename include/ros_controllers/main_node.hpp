#ifndef MAIN_NODE_HPP_
#define MAIN_NODE_HPP_


#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>

// ROS2 Controller Libraries
#include "ros_controllers/stanley.hpp"
#include "ros_controllers/pid.hpp"

// Messages
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

// Services
#include <std_srvs/srv/empty.hpp>


#define USE_GAZEBO          true


class MainNode : public rclcpp::Node
{
    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;

        rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_simulation_client_;

        std::unique_ptr<ROS2Controllers::StanleyController> stanley_controller_;
        std::unique_ptr<ROS2Controllers::PIDController> linear_velocity_pid_controller_;
        std::unique_ptr<ROS2Controllers::PIDController> angular_velocity_pid_controller_;
    
        rclcpp::TimerBase::SharedPtr pid_timer_;

        nav_msgs::msg::Odometry odometry_message_;

        geometry_msgs::msg::Twist cmd_vel_message_;

        nav_msgs::msg::Path path_;

        double roll_;

        double pitch_;

        double yaw_;

        int sleep_time_;

        double dt_;

        double Kp_linear_velocity_;

        double Ki_linear_velocity_;

        double Kd_linear_velocity_;

        double error_threshold_linear_velocity_;

        double signal_limit_linear_velocity_;

        double Kp_angular_velocity_;

        double Ki_angular_velocity_;

        double Kd_angular_velocity_;

        double error_threshold_angular_velocity_;

        double signal_limit_angular_velocity_;

        double linear_velocity_error_;

        double angular_velocity_error_;

        double linear_velocity_signal_;

        double angular_velocity_signal_;

        int index_of_pose_;

        bool vehicle_position_is_reached_;

        bool vehicle_orientation_is_reached_;

    protected:

    public:
        MainNode();

        ~MainNode();

        void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr message);

        void prepareWaypoints();

        void resetSystem();

        void controlManager();

        void stanley();

        void PID();
};



#endif