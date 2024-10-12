#include "ros_controllers/pure_pursuit.hpp"



ROS2Controllers::PurePursuitController::PurePursuitController(const double look_ahead_distance, const double vehicle_base_width,
    const double error_threshold, const double signal_limit)
    : look_ahead_distance_(look_ahead_distance), vehicle_base_width_(vehicle_base_width), error_threshold_(error_threshold), 
      signal_limit_(signal_limit), index_of_pose_(0) {
    // Subscribers

    // Publishers

    // Timers

}



ROS2Controllers::PurePursuitController::~PurePursuitController() {
    
}   



double ROS2Controllers::PurePursuitController::findDistance(const double x1, const double y1, const double x2, const double y2) {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}



void ROS2Controllers::PurePursuitController::findIndexOfNearestPoint() {
    double min_distance = std::numeric_limits<double>::max();
    int index = 0;

    for (int i=0; i<path_.size(); i++) {
        double distance = findDistance(vehicle_position_x_, vehicle_position_y_, path_[i].pose.position.x, path_[i].pose.position.y);

        if (distance < min_distance) {
            min_distance = distance;
            index = i;
        }
    }

    index_of_pose_ = index;
}



double ROS2Controllers::PurePursuitController::findDistanceViaIndex() {
    return findDistance(vehicle_position_x_, vehicle_position_y_, path_[index_of_pose_].pose.position.x, path_[index_of_pose_].pose.position.y);
}



void ROS2Controllers::PurePursuitController::findIndexOfClosestPointToLookAhead() {
    while (findDistanceViaIndex() < look_ahead_distance_) {
        index_of_pose_++;
    }
}



std::tuple<double, bool> ROS2Controllers::PurePursuitController::getPurePursuitSignal(const double vehicle_position_x, const double vehicle_position_y,
    const double vehicle_yaw, const std::vector<geometry_msgs::msg::PoseStamped> path) {
    
    path_ = path;
    vehicle_position_x_ = vehicle_position_x;
    vehicle_position_y_ = vehicle_position_y;

    findIndexOfNearestPoint();
    findIndexOfClosestPointToLookAhead();

    double target_x = path_[index_of_pose_].pose.position.x;
    double target_y = path_[index_of_pose_].pose.position.y;

    double alpha = atan2(target_y - vehicle_position_y_, target_x - vehicle_position_x_) - vehicle_yaw;

    if (alpha > M_PI / 2) {
        alpha = alpha - M_PI / 2;
    } else if (alpha < -M_PI / 2) {
        alpha = alpha +  M_PI / 2;
    }

    double ld = findDistance(target_x, target_y, vehicle_position_x_, vehicle_position_y_);

    double angular_velocity = atan2(2.0 * vehicle_base_width_ * sin(alpha), ld);

    if (std::abs(angular_velocity) > signal_limit_) {
        if (angular_velocity > 0) {
            angular_velocity = signal_limit_;
        } else {
            angular_velocity = -signal_limit_;
        }
    }

    if (index_of_pose_ >= path_.size()) {
        return std::make_tuple(angular_velocity, true);
    }

    return std::make_tuple(angular_velocity, false);
}