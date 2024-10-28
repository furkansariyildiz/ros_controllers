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
    declare_parameter("csv_folder_name", "/home/furkan/Documents/csv_files/results/");
    declare_parameter("vehicle_base_width", 0.5);

    sleep_time_ = this->get_parameter("sleep_time").as_int();
    csv_folder_name_ = this->get_parameter("csv_folder_name").as_string();
    vehicle_base_width_ = this->get_parameter("vehicle_base_width").as_double();

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
    
    // Pure-Pursuit Controller Parameters
    declare_parameter("PurePursuit.lookahead_distance", 2.0);
    declare_parameter("PurePursuit.error_threshold", 0.1);
    declare_parameter("PurePursuit.signal_limit", 0.5);

    lookahead_distance_pure_pursuit_controller_ = this->
        get_parameter("PurePursuit.lookahead_distance").as_double();

    error_threshold_pure_pursuit_controller_ = this->
        get_parameter("PurePursuit.error_threshold").as_double();

    signal_limit_pure_pursuit_controller_ = this->
        get_parameter("PurePursuit.signal_limit").as_double();

    // MPC Controller Parameters
    declare_parameter("MPC.horizon", 10);
    declare_parameter("MPC.error_threshold", 0.1);
    declare_parameter("MPC.signal_limit", 0.5);
    declare_parameter("MPC.Q", std::vector<double>({1.0, 1.0, 0.1}));
    declare_parameter("MPC.R", std::vector<double>({0.01, 0.01}));

    horizon_mpc_controller_ = this->
        get_parameter("MPC.horizon").as_int();

    error_threshold_mpc_controller_ = this->
        get_parameter("MPC.error_threshold").as_double();

    signal_limit_mpc_controller_ = this->
        get_parameter("MPC.signal_limit").as_double();

    mpc_q_ = this->
        get_parameter("MPC.Q").as_double_array();

    mpc_r_ = this->
        get_parameter("MPC.R").as_double_array();


    for (const auto &q : mpc_q_) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Q: " << q);
    }

    for (const auto &r : mpc_r_) {
        RCLCPP_INFO_STREAM(this->get_logger(), "R: " << r);
    }
    
    // Initialize class variables
    vehicle_position_is_reached_ = false;
    vehicle_orientation_is_reached_ = false;
    dt_ = sleep_time_ / 1000.0;

    // Linear Velocity PID Controller
    linear_velocity_pid_controller_ = std::make_unique<ROS2Controllers::PIDController>(Kp_linear_velocity_, 
        Ki_linear_velocity_, Kd_linear_velocity_, error_threshold_linear_velocity_, signal_limit_linear_velocity_);
    
    // Angular Velocity PID Controller
    angular_velocity_pid_controller_ = std::make_unique<ROS2Controllers::PIDController>(Kp_angular_velocity_, 
        Ki_angular_velocity_, Kd_angular_velocity_, error_threshold_angular_velocity_, signal_limit_angular_velocity_);

    // Stanley Controller
    stanley_controller_ = std::make_unique<ROS2Controllers::StanleyController>(V_, K_, error_threshold_stanley_controller_, 
        signal_limit_stanley_controller_);

    // Pure-Pursuit Controller
    pure_pursuite_controller_ = std::make_unique<ROS2Controllers::PurePursuitController>(lookahead_distance_pure_pursuit_controller_, 
        vehicle_base_width_, error_threshold_pure_pursuit_controller_, signal_limit_pure_pursuit_controller_);

    // MPC Controller
    // mpc_controller_ = std::make_unique<ROS2Controllers::MPCController>(horizon_mpc_controller_, vehicle_base_width_, error_threshold_mpc_controller_, 
    //    signal_limit_mpc_controller_, 1.0);
    mpc_controller_ = std::make_unique<ROS2Controllers::MPCController>(dt_, horizon_mpc_controller_, vehicle_base_width_, mpc_q_, mpc_r_);


    // Timers
    pid_timer_ = this->create_wall_timer(std::chrono::milliseconds(sleep_time_), std::bind(&MainNode::PID, this));
    pid_timer_->cancel();

    stanley_timer_ = this->create_wall_timer(std::chrono::milliseconds(sleep_time_), std::bind(&MainNode::stanley, this));
    stanley_timer_->cancel();

    pure_pursuit_timer_ = this->create_wall_timer(std::chrono::milliseconds(sleep_time_), std::bind(&MainNode::purePursuit, this));
    pure_pursuit_timer_->cancel();

    mpc_timer_ = this->create_wall_timer(std::chrono::milliseconds(sleep_time_), std::bind(&MainNode::mpc, this));
    mpc_timer_->cancel();

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

    // Pushing desired poses for visualization
    for (int i=0; i<path_.poses.size(); i++) {
        desired_poses_.push_back(path_.poses[i].pose);
    }
}



void MainNode::resetSystem() {
    // Setting cmd vel message to 0.0
    cmd_vel_message_.linear.x = 0.0;
    cmd_vel_message_.angular.z = 0.0;
    cmd_vel_publisher_->publish(cmd_vel_message_);

    // Resetting vehicle poses 
    vehicle_poses_.clear();

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
    // pid_timer_->reset();
    // stanley_timer_->reset();
    // pure_pursuit_timer_->reset();
    mpc_timer_->reset();
}



void MainNode::PID() {
    if (index_of_pose_ >= path_.poses.size()) {
        pid_timer_->cancel();
        stanley_timer_->reset();
        RCLCPP_INFO_STREAM(this->get_logger(), "PID controller is ended.");
        writeAndPlotResults("pid");
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

        // Saving discrete errors for visualization
        discrete_errors_.push_back(linear_velocity_error_);
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

    // Saving vehicle poses for visualization
    vehicle_poses_.push_back(odometry_message_.pose.pose);

    // Saving continous errors for visualization
    continuous_errors_.push_back(linear_velocity_error_);
}



void MainNode::stanley() {
    if (index_of_pose_ >= path_.poses.size()) {
        stanley_timer_->cancel();
        RCLCPP_INFO_STREAM(this->get_logger(), "Stanley controller is ended.");
        writeAndPlotResults("stanley");
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

        // Saving discrete errors for visualization
        discrete_errors_.push_back(stanley_controller_->getLinearError());
        RCLCPP_INFO_STREAM(this->get_logger(), "Target is reached, index: " << index_of_pose_);
    }

    // Preparing cmd vel message
    cmd_vel_message_.linear.x = linear_velocity_signal_;
    cmd_vel_message_.angular.z = angular_velocity_signal_;

    cmd_vel_publisher_->publish(cmd_vel_message_);   

    // Saving vehicle poses for visualization
    vehicle_poses_.push_back(odometry_message_.pose.pose); 

    // Saving continous errors for visualization
    continuous_errors_.push_back(stanley_controller_->getLinearError());
}



void MainNode::purePursuit() {
    auto [angular_velocity_signal_, vehicle_is_reached_] = pure_pursuite_controller_->getPurePursuitSignal(
        odometry_message_.pose.pose.position.x, odometry_message_.pose.pose.position.y, yaw_, path_.poses);

    if (vehicle_is_reached_) {
        pure_pursuit_timer_->cancel();
        RCLCPP_INFO_STREAM(this->get_logger(), "Pure-Pursuit controller is ended.");
        writeAndPlotResults("pure_pursuit");
        resetSystem();
    }
    
    // Preparing cmd vel message
    /**
     * @todo Linear velocity will be calculated from pure-pursuit controller. 
     * For now, it is set to 0.5 m/s.
     */
    cmd_vel_message_.linear.x = 0.5;
    cmd_vel_message_.angular.z = angular_velocity_signal_;

    cmd_vel_publisher_->publish(cmd_vel_message_);

    // Saving vehicle poses for visualization
    vehicle_poses_.push_back(odometry_message_.pose.pose);

    // Saving continuous errors for visualization
    continuous_errors_.push_back(pure_pursuite_controller_->getContinousLinearError());
    
    // Saving discrete errors for visualization
    discrete_errors_.push_back(pure_pursuite_controller_->getDiscreteLinearError());
}



void MainNode::mpc() {
    Eigen::VectorXd state(3);
    state << odometry_message_.pose.pose.position.x, odometry_message_.pose.pose.position.y, yaw_;  // x, y, theta

    std::vector<Eigen::VectorXd> reference_trajectory;

    if (index_of_pose_ < path_.poses.size() - 1) {
        double distance = std::sqrt(std::pow(path_.poses[index_of_pose_].pose.position.x - odometry_message_.pose.pose.position.x, 2) + 
            std::pow(path_.poses[index_of_pose_].pose.position.y - odometry_message_.pose.pose.position.y, 2));
            RCLCPP_INFO_STREAM(this->get_logger(), "Distance: " << distance);
        if (distance < 0.4) {
            index_of_pose_++;
        }
    }

    for (int i=index_of_pose_; i<std::min(index_of_pose_ + horizon_mpc_controller_ + 1, (int)path_.poses.size()); i++) {
        Eigen::VectorXd ref_state(3);
        
        double x = path_.poses[i].pose.position.x;
        double y = path_.poses[i].pose.position.y;

        double reference_theta = atan2(y - state(1), x - state(0));

        // RCLCPP_INFO_STREAM(this->get_logger(), "Reference x: " << x);
        // RCLCPP_INFO_STREAM(this->get_logger(), "Reference y: " << y);
        // RCLCPP_INFO_STREAM(this->get_logger(), "Reference theta: " << reference_theta);

        ref_state << x, y, reference_theta;
        reference_trajectory.push_back(ref_state);
    }

    std::cout << "Reference trajectory size: " << reference_trajectory.size() << std::endl;

    auto [optimal_velocity, optimal_steering_angle] = mpc_controller_->computeControlSignal(state, reference_trajectory);

    RCLCPP_INFO_STREAM(this->get_logger(), "Optimal velocity: " << optimal_velocity);
    RCLCPP_INFO_STREAM(this->get_logger(), "Optimal steering angle: " << optimal_steering_angle);
    RCLCPP_INFO_STREAM(this->get_logger(), "Index of pose: " << index_of_pose_);
    RCLCPP_INFO_STREAM(this->get_logger(), "-----------------------------------------------" << "\n");

    // Preparing cmd vel message
    cmd_vel_message_.linear.x = optimal_velocity;
    cmd_vel_message_.angular.z = optimal_steering_angle;

    cmd_vel_publisher_->publish(cmd_vel_message_);
}



void MainNode::writeAndPlotResults(const std::string test_name) {
    // Writing vehicle poses and desired poses to csv file
    result_path_csv_file_.open(csv_folder_name_ + "result_" + test_name + ".csv");
    desired_path_csv_file_.open(csv_folder_name_ + "desired_" + test_name + ".csv");
    continuous_error_csv_file_.open(csv_folder_name_ + "continuous_error_" + test_name + ".csv");
    discrete_error_csv_file_.open(csv_folder_name_ + "discrete_error_" + test_name + ".csv");

    if (result_path_csv_file_.is_open() && desired_path_csv_file_.is_open() && continuous_error_csv_file_.is_open()) {
        result_path_csv_file_ << "vehicle_x,vehicle_y,vehicle_z" << std::endl;
        
        for (int i=0; i<vehicle_poses_.size(); i++) {
            result_path_csv_file_ << vehicle_poses_[i].position.x << "," << vehicle_poses_[i].position.y << "," << 
                vehicle_poses_[i].position.z << std::endl;
        }

        desired_path_csv_file_ << "desired_x,desired_y,desired_z" << std::endl;

        for (int i=0; i<desired_poses_.size(); i++) {
            desired_path_csv_file_ << desired_poses_[i].position.x << "," << desired_poses_[i].position.y << "," << 
                desired_poses_[i].position.z << std::endl;
        }

        continuous_error_csv_file_ << "error" << std::endl;

        for (int i=0; i<continuous_errors_.size(); i++) {
            continuous_error_csv_file_ << continuous_errors_[i] << std::endl;
        }

        discrete_error_csv_file_ << "error" << std::endl;

        for (int i=0; i<discrete_errors_.size(); i++) {
            discrete_error_csv_file_ << discrete_errors_[i] << std::endl;
        }

        result_path_csv_file_.close();
        desired_path_csv_file_.close();
        continuous_error_csv_file_.close();
        discrete_error_csv_file_.close();
    } else {
        RCLCPP_ERROR_STREAM(this->get_logger(), "File is not opened.");
    }
}



int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto main_node = std::make_shared<MainNode>();
    rclcpp::spin(main_node);
    rclcpp::shutdown();

    return 0;
}
