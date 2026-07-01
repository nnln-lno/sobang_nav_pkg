#ifndef _RADAR_DEAD_RECKONING_HPP_
#define _RADAR_DEAD_RECKONING_HPP_

#include "rclcpp/rclcpp.hpp"
#include "vector"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>

#include "sobang_navigation/navTools.hpp"

namespace navigation
{
    class RadarEstimator
    {
    public:
        RadarEstimator();        

        bool egoVelocityEstimator();
        bool radarParser(const sensor_msgs::msg::PointCloud2::SharedPtr &radar_msg);

        MatXd getPointMatrix();    
        void setEgoVelocity(Vec3d velocity);
        Vec3d getEgoVelocity();

        void setCurrentPoints(MatXd points);
        void setPreviousPoints(MatXd points);

        MatXd getCurrentPoints();

        bool simpleRadar2DIcp(Mat3d R, Vec3d t);

        icpState getIcpPose() { return icp_pose; }

    private:            

        uint16_t point_size_;
        uint16_t zero_velocity_count_ = 0;

        Vec3d ego_velocity_;        

        MatXd radar_points_;
        VecXd radar_velocities_;

        // ICP map the current to previous => point w.r.t previous frame.
        MatXd current_points_; // target_points
        MatXd previous_points_; // source_points        

        icpState icp_pose;
                
    };
}  // namespace navigation

#endif  // _RADAR_DEAD_RECKONING_HPP_