#include "ros_controllers/main_node.hpp"



MainNode::MainNode() 
    : Node("ros_controllers_node") {

}


MainNode::~MainNode() {
    
}


void MainNode::init() {
        nav_msgs::msg::Path example_path;

        stanley_controller_ = std::make_unique<ROS2Controllers::StanleyController>(
            std::dynamic_pointer_cast<rclcpp::Node>(shared_from_this()), 1.0, 0.5, 0.1, example_path);
}



int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto main_node = std::make_shared<MainNode>();
    main_node->init();

    rclcpp::spin(main_node);
    rclcpp::shutdown();

    return 0;
}