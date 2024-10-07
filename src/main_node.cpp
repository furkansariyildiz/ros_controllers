#include "ros_controllers/main_node.hpp"



MainNode::MainNode() 
    : Node("ros_controllers_node") {
    // Subscribers
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 1, 
        std::bind(&MainNode::odometryCallback, this, std::placeholders::_1));

    // Publishers
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1000);

    // Services
    reset_simulation_client_ = this->create_client<std_srvs::srv::Empty>("/reset_simulation");

    // Parameters
    declare_parameter("sleep_time", 100);
    sleep_time_ = this->get_parameter("sleep_time").as_int();

    // PID (linear velocity controller)
    declare_parameter("PID.linear_velocity.Kp", 0.5);
    declare_parameter("PID.linear_velocity.Ki", 0.0);
    declare_parameter("PID.linear_velocity.Kd", 0.0);
    declare_parameter("PID.linear_velocity.error_threshold", 0.1);
    declare_parameter("PID.linear_velocity.signal_limit", 0.5);
    
    Kp_linear_velocity_ = this->
        get_parameter("PID.linear_velocity.Kp").as_double();
    Ki_linear_velocity_ = this->
        get_parameter("PID.linear_velocity.Ki").as_double();
    Kd_linear_velocity_ = this->
        get_parameter("PID.linear_velocity.Kd").as_double();
    error_threshold_linear_velocity_ = this->
        get_parameter("PID.linear_velocity.error_threshold").as_double();
    signal_limit_linear_velocity_ = this->
        get_parameter("PID.linear_velocity.signal_limit").as_double();


    // PID (angular velocity controller)
    declare_parameter("PID.angular_velocity.Kp", 0.5);
    declare_parameter("PID.angular_velocity.Ki", 0.0);
    declare_parameter("PID.angular_velocity.Kd", 0.0);
    declare_parameter("PID.angular_velocity.error_threshold", 0.1);
    declare_parameter("PID.angular_velocity.signal_limit", 0.5);

    Kp_angular_velocity_ = this->
        get_parameter("PID.angular_velocity.Kp").as_double();
    Ki_angular_velocity_ = this->
        get_parameter("PID.angular_velocity.Ki").as_double();
    Kd_angular_velocity_ = this->
        get_parameter("PID.angular_velocity.Kd").as_double();
    error_threshold_angular_velocity_ = this->
        get_parameter("PID.angular_velocity.error_threshold").as_double();
    signal_limit_angular_velocity_ = this->
        get_parameter("PID.angular_velocity.signal_limit").as_double();

    // Stanley Controller Parameters
    declare_parameter("Stanley.V", 1.0);
    declare_parameter("Stanley.K", 0.5);
    declare_parameter("Stanley.error_threshold", 0.1);
    declare_parameter("Stanley.signal_limit", 0.5);

    V_ = this->get_parameter("Stanley.V").as_double();
    K_ = this->get_parameter("Stanley.K").as_double();
    error_threshold_stanley_controller_ = this->
        get_parameter("Stanley.error_threshold").as_double();
    signal_limit_stanley_controller_ = this->
        get_parameter("Stanley.signal_limit").as_double();
    
    RCLCPP_ERROR_STREAM(this->get_logger(), "Angular velocity limit: " << signal_limit_angular_velocity_);
    RCLCPP_ERROR_STREAM(this->get_logger(), "Linear velocity limit: " << signal_limit_linear_velocity_);


    // Linear Velocity PID Controller
    linear_velocity_pid_controller_ = std::make_unique<ROS2Controllers::PIDController>(Kp_linear_velocity_, 
        Ki_linear_velocity_, Kd_linear_velocity_, error_threshold_linear_velocity_, signal_limit_linear_velocity_);
    
    // Angular Velocity PID Controller
    angular_velocity_pid_controller_ = std::make_unique<ROS2Controllers::PIDController>(Kp_angular_velocity_, 
        Ki_angular_velocity_, Kd_angular_velocity_, error_threshold_angular_velocity_, signal_limit_angular_velocity_);

    // Stanley Controller
    stanley_controller_ = std::make_unique<ROS2Controllers::StanleyController>(V_, K_, error_threshold_stanley_controller_, 
        signal_limit_stanley_controller_);
    
    // Initialize class variables
    vehicle_position_is_reached_ = false;
    vehicle_orientation_is_reached_ = false;
    dt_ = sleep_time_ / 1000.0;

    // Timers
    pid_timer_ = this->create_wall_timer(std::chrono::milliseconds(sleep_time_), std::bind(&MainNode::PID, this));
    pid_timer_->cancel();

    stanley_timer_ = this->create_wall_timer(std::chrono::milliseconds(sleep_time_), std::bind(&MainNode::stanley, this));
    stanley_timer_->cancel();

    this->controlManager();
}



MainNode::~MainNode() {
    
}



void MainNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr message) {
    odometry_message_ = *message;

    tf2::Quaternion quaternion(
        message->pose.pose.orientation.x,
        message->pose.pose.orientation.y,
        message->pose.pose.orientation.z,
        message->pose.pose.orientation.w
    );

    tf2::Matrix3x3 matrix(quaternion);

    matrix.getRPY(roll_, pitch_, yaw_);
}



void MainNode::prepareWaypoints() {
    // Sine wave
    double ampliute = 2.0;
    double frequency = 0.2;
    int number_of_points = 100;

    path_.poses.clear();

    for (int i=0; i<number_of_points; i++) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = this->now();

        pose.pose.position.x = static_cast<double>(i) * 0.5;
        pose.pose.position.y = ampliute * std::sin(frequency * pose.pose.position.x);
        pose.pose.position.z = 0.0;

        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;

        path_.poses.push_back(pose);
    }

    index_of_pose_ = 0;

    previous_waypoint_ = path_.poses[index_of_pose_].pose;
    next_waypoint_ = path_.poses[index_of_pose_ + 1].pose;
}



void MainNode::resetSystem() {
    // Setting cmd vel message to 0.0
    cmd_vel_message_.linear.x = 0.0;
    cmd_vel_message_.angular.z = 0.0;
    cmd_vel_publisher_->publish(cmd_vel_message_);

    // Reset Gazebo Simulation
    if (USE_GAZEBO) {
        auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
        auto result = reset_simulation_client_->async_send_request(empty_request);
    } else {
        RCLCPP_INFO_STREAM(this->get_logger(), "Gazebo is not used.");
    }
} 



void MainNode::controlManager() {
    prepareWaypoints();
    //pid_timer_->reset();
    stanley_timer_->reset();
}



void MainNode::PID() {
    if (index_of_pose_ >= path_.poses.size()) {
        pid_timer_->cancel();
        stanley_timer_->reset();
        RCLCPP_INFO_STREAM(this->get_logger(), "PID controller is ended.");
        resetSystem();
    }

    // Desired pose
    geometry_msgs::msg::Pose desired_pose = path_.poses[index_of_pose_].pose;
    
    // Vehicle pose
    geometry_msgs::msg::Pose vehicle_pose = odometry_message_.pose.pose;

    // Linear velocity error (sqrt((x2 - x1)^2 + (y2 - y1)^2)) meters
    linear_velocity_error_ = std::sqrt(std::pow(desired_pose.position.x - vehicle_pose.position.x, 2) + 
        std::pow(desired_pose.position.y - vehicle_pose.position.y, 2));

    // Desired angle via atan((y2 - y1) / (x2 - x1)) radians
    double desired_angle = atan2((desired_pose.position.y - vehicle_pose.position.y), 
        (desired_pose.position.x - vehicle_pose.position.x));

    // Angular velocity error
    angular_velocity_error_ = desired_angle - yaw_;
    angular_velocity_error_ = atan2(sin(angular_velocity_error_), cos(angular_velocity_error_));

    // Getting linear velocity signal from PID Controller
    auto [linear_velocity_signal_, vehicle_position_is_reached_] = linear_velocity_pid_controller_->getPIDControllerSignal(
        linear_velocity_error_, dt_);

    // Getting angular velocity signal from PID Controller
    auto [angular_velocity_signal_, vehicle_orientation_is_reached_] = angular_velocity_pid_controller_->getPIDControllerSignal(
        angular_velocity_error_, dt_);

    RCLCPP_ERROR_STREAM(this->get_logger(), "Linear velocity: " << linear_velocity_signal_);
    RCLCPP_ERROR_STREAM(this->get_logger(), "Angular velocity: " << angular_velocity_signal_);

    // Checking vehicle is reached to position or not.
    if (vehicle_position_is_reached_  && vehicle_orientation_is_reached_) {
        index_of_pose_++;
        RCLCPP_INFO_STREAM(this->get_logger(), "Target is reached, index: " << index_of_pose_);
    }

    // Preparing cmd vel message 
    cmd_vel_message_.linear.x = linear_velocity_signal_;
    cmd_vel_message_.angular.z = angular_velocity_signal_;

    RCLCPP_ERROR_STREAM(this->get_logger(), "Linear error: " << linear_velocity_error_);
    RCLCPP_ERROR_STREAM(this->get_logger(), "Angular error: " << angular_velocity_error_ << "\n");
    RCLCPP_ERROR_STREAM(this->get_logger(), "Desired angle: " << desired_angle);
    RCLCPP_ERROR_STREAM(this->get_logger(), "Vehicle yaw: " << yaw_);

    RCLCPP_INFO_STREAM(this->get_logger(), "-----------------------------------------------");

    // Publishing cmd vel message
    cmd_vel_publisher_->publish(cmd_vel_message_);
}



void MainNode::stanley() {
    if (index_of_pose_ >= path_.poses.size()) {
        stanley_timer_->cancel();
        RCLCPP_INFO_STREAM(this->get_logger(), "Stanley controller is ended.");
        resetSystem();
    }

    // Vehicle pose
    geometry_msgs::msg::Pose vehicle_pose = odometry_message_.pose.pose;

    auto [linear_velocity_signal_, angular_velocity_signal_, vehicle_position_is_reached_] = stanley_controller_->
        getStanleyControllerSignal(next_waypoint_.position.x, next_waypoint_.position.y, previous_waypoint_.position.x, 
        previous_waypoint_.position.y, vehicle_pose.position.x, vehicle_pose.position.y, yaw_);

    if (vehicle_position_is_reached_) {
        previous_waypoint_ = path_.poses[index_of_pose_].pose;
        next_waypoint_ = path_.poses[index_of_pose_ + 1].pose;
        index_of_pose_++;
        RCLCPP_INFO_STREAM(this->get_logger(), "Target is reached, index: " << index_of_pose_);
    }

    // Preparing cmd vel message
    cmd_vel_message_.linear.x = linear_velocity_signal_;
    cmd_vel_message_.angular.z = angular_velocity_signal_;

    cmd_vel_publisher_->publish(cmd_vel_message_);    
}



int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto main_node = std::make_shared<MainNode>();
    rclcpp::spin(main_node);
    rclcpp::shutdown();

    return 0;
}
