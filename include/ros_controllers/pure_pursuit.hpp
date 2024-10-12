#ifndef PURE_PURSUITE_HPP_
#define PURE_PURSUITE_HPP_

// ROS2 Libraries
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <cmath>
#include <tuple>

// Messages
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>



namespace ROS2Controllers
{
    class PurePursuitController
    {
        private:
            double lookahead_distance_;

            double vehicle_base_width_;

            double vehicle_position_x_;

            double vehicle_position_y_;

            std::vector<geometry_msgs::msg::PoseStamped> path_;

            const double error_threshold_;

            const double signal_limit_;

            int index_of_pose_;

        protected:

        public:
            PurePursuitController(const double lookahead_distance, const double vehicle_base_width, 
                const double error_threshold, const double signal_limit);
    
            ~PurePursuitController();

            double findDistance(const double x1, const double y1, const double x2, const double y2);

            void findIndexOfNearestPoint(void);
            
            double findDistanceViaIndex(void);
            
            void findIndexOfClosestPointToLookAhead(void);

            std::tuple<double, bool> getPurePursuitSignal(const double error, const double dt, 
                const double vehicle_yaw, const std::vector<geometry_msgs::msg::PoseStamped> path);
    };
} // namespace ROS2Controllers



#endif // PURE_PURSUITE_HPP