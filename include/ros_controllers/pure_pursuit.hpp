#ifndef PURE_PURSUITE_HPP_
#define PURE_PURSUITE_HPP_

// ROS2 Libraries
#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <tuple>

// Messages
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>



namespace ROS2Controllers
{
    class PurePursuitController
    {
        private:
            double lookahead_distance_;

            double vehicle_base_width_;

            double vehicle_position_x_;

            double vehicle_position_y_;

            std::vector<geometry_msgs::PoseStamped> path_;

            const double error_threshold_;

            const double signal_limit_;

            int index_of_pose_;

            int previous_index_of_pose_;

            bool previous_index_of_pose_initialized_;

            double continous_error_;

            double discrete_error_;

            bool vehicle_position_is_reached_;

        protected:

        public:
            PurePursuitController(const double lookahead_distance, const double vehicle_base_width, 
                const double error_threshold, const double signal_limit);
    
            ~PurePursuitController();

            double getContinousError(void) const;

            double getDiscreteError(void) const;

            double findDistance(const double x1, const double y1, const double x2, const double y2);

            double findDistanceViaIndex(void);

            void findIndexOfNearestPoint(void);
            
            void checkVehicleIsReached(void);
            
            void findIndexOfClosestPointToLookAhead(void);

            std::tuple<double, bool> getPurePursuitSignal(const double error, const double dt, 
                const double vehicle_yaw, const std::vector<geometry_msgs::PoseStamped> path);
    };
} // namespace ROS2Controllers



#endif // PURE_PURSUITE_HPP