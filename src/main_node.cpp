#include "ros_controllers/main_node.hpp"



MainNode::MainNode() 
    : Node("ros_controllers_node") {

}



MainNode::~MainNode() {
    
}



void MainNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr message) {
    odometry_message_ = *message;
}



void MainNode::init() {
    nav_msgs::msg::Path example_path;

    stanley_controller_ = std::make_unique<ROS2Controllers::StanleyController>(
        shared_from_this(), 1.0, 0.5, 0.1, example_path);

    // Linear Velocity PID Controller
    linear_velocity_pid_controller_ = std::make_unique<ROS2Controllers::PIDController>(
        shared_from_this(), 1.0, 1.0, 1.0, 0.1, 1.0);
    
    // Angular Velocity PID Controller
    angular_velocity_pid_controller_ = std::make_unique<ROS2Controllers::PIDController>(
        shared_from_this(), 1.0, 1.0, 1.0, 0.1, 1.0);
}



void MainNode::controlManager() {
    stanley_controller_->run();
    PID();
}



void MainNode::stanley() {

}


void MainNode::PID() {
    int index_of_pose = 0;
    while (rclcpp::ok() && index_of_pose < path_.poses.size()) {
        geometry_msgs::msg::Pose desired_pose = path_.poses[index_of_pose].pose;
        geometry_msgs::msg::Pose vehicle_pose = odometry_message_.pose.pose;

        // Linear velocity error (sqrt((x2 - x1)^2 + (y2 - y1)^2)) meters
        linear_velocity_error_ = std::sqrt(std::pow(desired_pose.position.x - vehicle_pose.position.x, 2) + 
            std::pow(desired_pose.position.y - vehicle_pose.position.y, 2));

        // Angular velocity error atan((y2 - y1) / (x2 - x1)) radians
        angular_velocity_error_ = std::atan2((desired_pose.position.y - vehicle_pose.position.y), 
            (desired_pose.position.x - vehicle_pose.position.x));

        linear_velocity_signal_ = linear_velocity_pid_controller_->getPIDControllerSignal(linear_velocity_error_, 
            0.01);

        angular_velocity_signal_ = angular_velocity_pid_controller_->getPIDControllerSignal(angular_velocity_error_,
            0.01);

        if(linear_velocity_signal_ == 0.0 && angular_velocity_signal_ == 0.0) {
            index_of_pose++;
        }

        cmd_vel_message_.linear.x = linear_velocity_signal_;
        cmd_vel_message_.angular.z = angular_velocity_signal_;

        cmd_vel_publisher_->publish(cmd_vel_message_);
    }
}



int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto main_node = std::make_shared<MainNode>();
    main_node->init();
    main_node->controlManager();

    rclcpp::spin(main_node);
    rclcpp::shutdown();

    return 0;
}

/** 
 * 

    int index_of_pose = 0;
    while (rclcpp::ok() && index_of_pose < path_.poses.size()) {
        // Getting desired pose and vehicle pose
        geometry_msgs::msg::Pose desired_pose = path_.poses[index_of_pose].pose;
        geometry_msgs::msg::Pose vehicle_pose = odometry_message_.pose.pose;
        
        // Linear velocity error (sqrt((x2 - x1)^2 + (y2 - y1)^2)) meters
        linear_velocity_error_ = std::sqrt(std::pow(desired_pose.position.x - vehicle_pose.position.x, 2) + 
            std::pow(desired_pose.position.y - vehicle_pose.position.y, 2));
        
        // Angular velocity error atan((y2 - y1) / (x2 - x1)) radians
        angular_velocity_error_ = std::atan2((desired_pose.position.y - vehicle_pose.position.y), 
            (desired_pose.position.x - vehicle_pose.position.x));

        if (linear_velocity_error_ <= linear_error_threshold_) {
            linear_velocity_signal_ = 0.0;
            index_of_pose++;
        } else {
            // linear_velocity_signal_ = getPIDControllerSignal()
        }

        */