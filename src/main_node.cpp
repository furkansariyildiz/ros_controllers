#include "ros_controllers/main_node.hpp"



MainNode::MainNode(ros::NodeHandle &node_handle) 
    : nh_(node_handle) {
    // Subscribers
    odometry_subscriber_ = nh_.subscribe("/odom", 1, &MainNode::odometryCallback, this);

    // Publishers
    cmd_vel_publisher_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    
    // Services
    reset_simulation_client_ = nh_.serviceClient<std_srvs::Empty>("/reset_simulation");
    
    // Parameters
    nh_.getParam("/ros_controllers_node/sleep_time", sleep_time_);
    nh_.getParam("/ros_controllers_node/cs_folder_name", csv_folder_name_);
    nh_.getParam("/ros_controllers_node/vehicle_base_width", vehicle_base_width_);

    // PID (linear velocity controller)
    nh_.getParam("/ros_controllers_node/PID/linear_velocity/Kp", Kp_linear_velocity_);
    nh_.getParam("/ros_controllers_node/PID/linear_velocity/Ki", Ki_linear_velocity_);
    nh_.getParam("/ros_controllers_node/PID/linear_velocity/Kd", Kd_linear_velocity_);
    nh_.getParam("/ros_controllers_node/PID/linear_velocity/error_threshold", error_threshold_linear_velocity_);
    nh_.getParam("/ros_controllers_node/PID/linear_velocity/signal_limit", signal_limit_linear_velocity_);

    // PID (angular velocity controller)
    nh_.getParam("/ros_controllers_node/PID/angular_velocity/Kp", Kp_angular_velocity_);
    nh_.getParam("/ros_controllers_node/PID/angular_velocity/Ki", Ki_angular_velocity_);
    nh_.getParam("/ros_controllers_node/PID/angular_velocity/Kd", Kd_angular_velocity_);
    nh_.getParam("/ros_controllers_node/PID/angular_velocity/error_threshold", error_threshold_angular_velocity_);  
    nh_.getParam("/ros_controllers_node/PID/angular_velocity/signal_limit", signal_limit_angular_velocity_);

    // Stanley Controller Parameters
    nh_.getParam("/ros_controllers_node/Stanley/V", V_);
    nh_.getParam("/ros_controllers_nodre/Stanley/K", K_);
    nh_.getParam("/ros_controllers_node/Stanley/error_threshold", error_threshold_stanley_controller_);
    nh_.getParam("/ros_controllers_node/Stanley/signal_limit", signal_limit_stanley_controller_);

    // Pure-Pursuit Controller Parameters
    nh_.getParam("/ros_controllers_node/PurePursuit/lookahead_distance", lookahead_distance_pure_pursuit_controller_);
    nh_.getParam("/ros_controllers_node/PurePursuit/constant_velocity", constant_velocity_pure_pursuit_controller_);
    nh_.getParam("/ros_controllers_node/PurePursuit/error_threshold", error_threshold_pure_pursuit_controller_);
    nh_.getParam("/ros_controllers_node/PurePursuit/signal_limit", signal_limit_pure_pursuit_controller_);
    
    // MPC Controller Parameters
    nh_.getParam("/ros_controllers_node/MPC/horizon", horizon_mpc_controller_);
    nh_.getParam("/ros_controllers_node/MPC/error_threshold", error_threshold_mpc_controller_);
    nh_.getParam("/ros_controllers_node/MPC/linear_velocity/signal_limit", signal_limit_linear_velocity_mpc_controller_);
    nh_.getParam("/ros_controllers_node/MPC/angular_velocity/signal_limit", signal_limit_angular_velocity_mpc_controller_);
    nh_.getParam("/ros_controllers_node/MPC/Q", mpc_q_);
    nh_.getParam("/ros_controllers_node/MPC/R", mpc_r_);


    for (const auto &q : mpc_q_) {
        ROS_INFO_STREAM("Q: " << q);
    }

    for (const auto &r : mpc_r_) {
        ROS_INFO_STREAM("R: " << r);
    }

    ROS_INFO_STREAM("Signal limit linear velocity MPC: " << signal_limit_linear_velocity_mpc_controller_);
    ROS_INFO_STREAM("Signal limit angular velocity MPC: " << signal_limit_angular_velocity_mpc_controller_);
    
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
    mpc_controller_ = std::make_unique<ROS2Controllers::MPCController>(dt_, horizon_mpc_controller_, vehicle_base_width_, mpc_q_, mpc_r_, 
        signal_limit_linear_velocity_mpc_controller_, signal_limit_angular_velocity_mpc_controller_, error_threshold_mpc_controller_);

    // Timers
    pid_timer_ = nh_.createTimer(ros::Duration(dt_), &MainNode::PID, this);
    pid_timer_.stop();

    stanley_timer_ = nh_.createTimer(ros::Duration(dt_), &MainNode::stanley, this);
    stanley_timer_.stop();

    pure_pursuit_timer_ = nh_.createTimer(ros::Duration(dt_), &MainNode::purePursuit, this);
    pure_pursuit_timer_.stop();

    mpc_timer_ = nh_.createTimer(ros::Duration(dt_), &MainNode::mpc, this);
    mpc_timer_.stop();

    this->controlManager();
}   



MainNode::~MainNode() {
    
}



void MainNode::odometryCallback(const nav_msgs::Odometry::ConstPtr message) {
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

    for (int i=1; i<number_of_points + 1; i++) {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time::now();

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
    cmd_vel_publisher_.publish(cmd_vel_message_);

    // Resetting vehicle poses 
    vehicle_poses_.clear();

    // Reset Gazebo Simulation
    if (USE_GAZEBO) {
        std_srvs::Empty empty_request;
        auto result = reset_simulation_client_.call(empty_request);
        ROS_INFO_STREAM("Simulation is reset.");
    } else {
        ROS_INFO_STREAM("Gazebo is not used.");
    }
} 



void MainNode::controlManager() {
    prepareWaypoints();
    // pid_timer_->reset();
    // stanley_timer_->reset();
    // pure_pursuit_timer_->reset();
    mpc_timer_.start();
}



void MainNode::PID(const ros::TimerEvent &event) {
    if (index_of_pose_ >= path_.poses.size()) {
        pid_timer_.stop();
        ROS_INFO_STREAM("PID controller is ended.");
        writeAndPlotResults("pid");
        resetSystem();
        return;
    }

    // Desired pose
    geometry_msgs::Pose desired_pose = path_.poses[index_of_pose_].pose;
    
    // Vehicle pose
    geometry_msgs::Pose vehicle_pose = odometry_message_.pose.pose;

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

    ROS_ERROR_STREAM("Linear velocity: " << linear_velocity_signal_);
    ROS_ERROR_STREAM("Angular velocity: " << angular_velocity_signal_);

    // Checking vehicle is reached to position or not.
    if (vehicle_position_is_reached_  && vehicle_orientation_is_reached_) {
        index_of_pose_++;

        // Saving discrete errors for visualization
        discrete_errors_.push_back(linear_velocity_pid_controller_->getDiscreteError());
        ROS_INFO_STREAM("Target is reached, index: " << index_of_pose_);
    }

    // Preparing cmd vel message 
    cmd_vel_message_.linear.x = linear_velocity_signal_;
    cmd_vel_message_.angular.z = angular_velocity_signal_;

    ROS_ERROR_STREAM("Linear error: " << linear_velocity_error_);
    ROS_ERROR_STREAM("Angular error: " << angular_velocity_error_ << "\n");
    ROS_ERROR_STREAM("Desired angle: " << desired_angle);
    ROS_ERROR_STREAM("Vehicle yaw: " << yaw_);

    ROS_INFO_STREAM("-----------------------------------------------");

    // Publishing cmd vel message
    cmd_vel_publisher_.publish(cmd_vel_message_);

    // Saving vehicle poses for visualization
    vehicle_poses_.push_back(odometry_message_.pose.pose);

    // Saving continous errors for visualization
    continuous_errors_.push_back(linear_velocity_pid_controller_->getContinousError());
}



void MainNode::stanley(const ros::TimerEvent &event) {
    if (index_of_pose_ >= path_.poses.size()) {
        stanley_timer_.stop();
        ROS_INFO_STREAM("Stanley controller is ended.");
        writeAndPlotResults("stanley");
        resetSystem();
        return;
    }

    // Vehicle pose
    geometry_msgs::Pose vehicle_pose = odometry_message_.pose.pose;

    auto [linear_velocity_signal_, angular_velocity_signal_, vehicle_position_is_reached_] = stanley_controller_->
        getStanleyControllerSignal(next_waypoint_.position.x, next_waypoint_.position.y, previous_waypoint_.position.x, 
        previous_waypoint_.position.y, vehicle_pose.position.x, vehicle_pose.position.y, yaw_);

    if (vehicle_position_is_reached_) {
        previous_waypoint_ = path_.poses[index_of_pose_].pose;
        next_waypoint_ = path_.poses[index_of_pose_ + 1].pose;
        index_of_pose_++;

        ROS_INFO_STREAM("Target is reached, index: " << index_of_pose_);
    }

    // Preparing cmd vel message
    cmd_vel_message_.linear.x = linear_velocity_signal_;
    cmd_vel_message_.angular.z = angular_velocity_signal_;

    cmd_vel_publisher_.publish(cmd_vel_message_);   

    // Saving vehicle poses for visualization
    vehicle_poses_.push_back(odometry_message_.pose.pose); 

    // Saving continous errors for visualization
    continuous_errors_.push_back(stanley_controller_->getContinousError());
    
    // Saving discrete errors for visualization
    discrete_errors_.push_back(stanley_controller_->getDiscreteError());
}



void MainNode::purePursuit(const ros::TimerEvent &event) {
    auto [angular_velocity_signal_, vehicle_is_reached_] = pure_pursuite_controller_->getPurePursuitSignal(
        odometry_message_.pose.pose.position.x, odometry_message_.pose.pose.position.y, yaw_, path_.poses);

    if (vehicle_is_reached_) {
        pure_pursuit_timer_.stop();
        ROS_INFO_STREAM("Pure-Pursuit controller is ended.");
        writeAndPlotResults("pure_pursuit");
        resetSystem();
        return;
    }
    
    // Preparing cmd vel message
    /**
     * @todo Linear velocity will be calculated from pure-pursuit controller. 
     * For now, it is set to 0.5 m/s.
     */
    cmd_vel_message_.linear.x = constant_velocity_pure_pursuit_controller_;
    cmd_vel_message_.angular.z = angular_velocity_signal_;

    cmd_vel_publisher_.publish(cmd_vel_message_);

    // Saving vehicle poses for visualization
    vehicle_poses_.push_back(odometry_message_.pose.pose);

    // Saving continuous errors for visualization
    continuous_errors_.push_back(pure_pursuite_controller_->getContinousError());
    
    // Saving discrete errors for visualization
    discrete_errors_.push_back(pure_pursuite_controller_->getDiscreteError());
}



void MainNode::mpc(const ros::TimerEvent &event) {
    ROS_INFO_STREAM("MPC controller is started.");
    Eigen::VectorXd state(3);
    state << odometry_message_.pose.pose.position.x, odometry_message_.pose.pose.position.y, yaw_;  // x, y, theta

    std::vector<Eigen::VectorXd> reference_trajectory;

    if (index_of_pose_ >= path_.poses.size()) {
        mpc_timer_.stop();
        ROS_INFO_STREAM("MPC controller is ended.");
        writeAndPlotResults("mpc");
        resetSystem();
        return;
    }

    int end_pose = std::min(index_of_pose_ + horizon_mpc_controller_, (int)path_.poses.size() - 1);
    
    for (int i=index_of_pose_; i<(int)path_.poses.size(); i++) {
        Eigen::VectorXd ref_state(3);
        
        double x = path_.poses[i].pose.position.x;
        double y = path_.poses[i].pose.position.y;

        double x_next_waypoint = path_.poses[i + 1].pose.position.x;
        double y_next_waypoint = path_.poses[i + 1].pose.position.y;

        double x_current_waypoint = path_.poses[i].pose.position.x;
        double y_current_waypoint = path_.poses[i].pose.position.y;

        double reference_theta = atan2(y_next_waypoint - y_current_waypoint, x_next_waypoint - x_current_waypoint);
        
        ref_state << x_current_waypoint, y_current_waypoint, reference_theta;
        reference_trajectory.push_back(ref_state);
    }

    auto [optimal_velocity, optimal_steering_angle, vehicle_position_is_reached_] = mpc_controller_->computeControlSignal(state, reference_trajectory);

    if (vehicle_position_is_reached_) {
        index_of_pose_++;
        ROS_INFO_STREAM("Target is reached, index: " << index_of_pose_);
    }

    // Preparing cmd vel message
    cmd_vel_message_.linear.x = optimal_velocity;
    cmd_vel_message_.angular.z = optimal_steering_angle;

    cmd_vel_publisher_.publish(cmd_vel_message_);

    // Saving vehicle poses for visualization
    vehicle_poses_.push_back(odometry_message_.pose.pose);

    // Saving continuous errors for visualization
    continuous_errors_.push_back(mpc_controller_->getContinousError());

    // Saving discrete errors for visualization
    discrete_errors_.push_back(mpc_controller_->getDiscreteError());

    ROS_INFO_STREAM("Optimal steering angle: " << optimal_steering_angle);
    ROS_INFO_STREAM("Index of pose: " << index_of_pose_);
    ROS_INFO_STREAM("Discrete error: " << mpc_controller_->getDiscreteError());
    ROS_INFO_STREAM("Continuous error: " << mpc_controller_->getContinousError());
    ROS_INFO_STREAM("-----------------------------------------------" << "\n");
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
        ROS_ERROR_STREAM("File is not opened.");
    }
}



int main(int argc, char *argv[]) {
    ros::init(argc, argv, "ros_controllers_node");
    ros::NodeHandle node_handle;
    
    MainNode main_node(node_handle);

    ros::spin();
    ros::waitForShutdown();

    return 0;
}
