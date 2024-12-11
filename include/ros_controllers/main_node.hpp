#ifndef MAIN_NODE_HPP_
#define MAIN_NODE_HPP_



#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>


// ROS2 Controller Libraries
#include "ros_controllers/mpc.hpp"
#include "ros_controllers/pid.hpp"
#include "ros_controllers/stanley.hpp"
#include "ros_controllers/pure_pursuit.hpp"

// Messages
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

// Services
#include <std_srvs/Empty.h>



// Constants
#define USE_GAZEBO          true



class MainNode 
{
    private:
        ros::NodeHandle &nh_;

        ros::Subscriber odometry_subscriber_;
        
        ros::Publisher cmd_vel_publisher_;

        ros::ServiceClient reset_simulation_client_;

        std::unique_ptr<ROS2Controllers::StanleyController> stanley_controller_;
        std::unique_ptr<ROS2Controllers::PIDController> linear_velocity_pid_controller_;
        std::unique_ptr<ROS2Controllers::PIDController> angular_velocity_pid_controller_;
        std::unique_ptr<ROS2Controllers::PurePursuitController> pure_pursuite_controller_;
        std::unique_ptr<ROS2Controllers::MPCController> mpc_controller_;

        ros::Timer pid_timer_;
        ros::Timer stanley_timer_;
        ros::Timer pure_pursuit_timer_;
        ros::Timer mpc_timer_;

        nav_msgs::Odometry odometry_message_;

        geometry_msgs::Twist cmd_vel_message_;

        nav_msgs::Path path_;

        std::vector<geometry_msgs::Pose> desired_poses_;
        std::vector<geometry_msgs::Pose> vehicle_poses_;
        std::vector<double> continuous_errors_;
        std::vector<double> discrete_errors_;

        double roll_;

        double pitch_;

        double yaw_;

        int sleep_time_;

        double dt_;

        // PID Controller Parameters
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

        // Stanley Controller Parameters
        double V_;

        double K_;

        double error_threshold_stanley_controller_;

        double signal_limit_stanley_controller_;

        int index_of_pose_;

        bool vehicle_position_is_reached_;

        bool vehicle_orientation_is_reached_;

        geometry_msgs::Pose previous_waypoint_;

        geometry_msgs::Pose next_waypoint_;

        // Pure-Pursuit Controller Parameters
        double lookahead_distance_pure_pursuit_controller_;

        double constant_velocity_pure_pursuit_controller_;

        double error_threshold_pure_pursuit_controller_;

        double signal_limit_pure_pursuit_controller_;

        double vehicle_base_width_;

        // MPC Controller Parameters
        int horizon_mpc_controller_;

        double error_threshold_mpc_controller_;

        double signal_limit_linear_velocity_mpc_controller_;

        double signal_limit_angular_velocity_mpc_controller_;

        std::vector<double> mpc_q_;

        std::vector<double> mpc_r_;

        bool vehicle_is_reached_;

        std::ofstream result_path_csv_file_;
        std::ofstream desired_path_csv_file_;
        std::ofstream continuous_error_csv_file_;
        std::ofstream discrete_error_csv_file_;

        std::string csv_folder_name_;

    protected:

    public:
        MainNode(ros::NodeHandle &node_handle);

        ~MainNode();

        void odometryCallback(const nav_msgs::Odometry::ConstPtr message);

        void prepareWaypoints();

        void resetSystem();

        void controlManager();

        void PID(const ros::TimerEvent &event);

        void stanley(const ros::TimerEvent &event);

        void purePursuit(const ros::TimerEvent &event);

        void mpc(const ros::TimerEvent &event);

        void writeAndPlotResults(const std::string test_name);
};



#endif