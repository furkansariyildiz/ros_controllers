#include "ros_controllers/main_node.hpp"



MainNode::MainNode() 
    : Node("ros_controllers_node") {

}



MainNode::~MainNode() {
    
}



void MainNode::init() {
    nav_msgs::msg::Path example_path;

    stanley_controller_ = std::make_unique<ROS2Controllers::StanleyController>(
        shared_from_this(), 1.0, 0.5, 0.1, example_path);

    pid_controller = std::make_unique<ROS2Controllers::PIDController>(
        shared_from_this(), 1.0, 1.0, 1.0, 0.1, example_path);
}



void MainNode::controlManager() {
    stanley_controller_->run();
    pid_controller->run();
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