#include "ros_controllers/main_node.hpp"



MainNode::MainNode() 
    : Node("ros_controllers_node") {

}



MainNode::~MainNode() {
    
}





int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MainNode>());
    return 0;
}