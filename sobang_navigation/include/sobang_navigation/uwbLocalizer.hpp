#ifndef _UWB_LOCALIZER_HPP_
#define _UWB_LOCALIZER_HPP_

#include <algorithm>
#include <vector>
#include <numeric>
#include <cmath>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "sobang_navigation/navTools.hpp"
#include "sobang_navigation/msg/uwb_data.hpp"
#include "uwb_driver/msg/uwb_range.hpp"

struct uwbMeasurement
{
    int anchor_id;    
    double anchor_position_x_; // a prior information of anchor position w.r.t world frame
    double anchor_position_y_;
    double anchor_position_z_;
    double range;
};

namespace navigation
{
    class UWBLocalizer : public rclcpp::Node
    {
    public:
        UWBLocalizer();

        std::vector<Vec3d> anchor_positions_; // UWB Anchor positions in the world frame

        std::string uwb_topic_ = "/uwb/range";
        bool imu_ned_ = false;
        bool sim_sonar_ = false;
         // UWB Estimated position of the drone in the world frame

    private:

        std::vector<double> anchor_list_x_;
        std::vector<double> anchor_list_y_;
        std::vector<double> anchor_list_z_;

        std::vector<int64_t> anchor_id_lists_;

        bool visualization_flag_ = true;
        bool do_multilateration_ = false;
        uint num_anchors_;

        uwbMeasurement uwb_struct_[20];
        int id_matcher_[20] = {-1};

        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr uwb_marker_publisher_;

        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr uwb_text_publisher_;

        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr uwb_position_publisher_;

        rclcpp::Publisher<sobang_navigation::msg::UwbData>::SharedPtr uwb_range_publisher_;

        rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr sonar_publisher_;

        rclcpp::Subscription<uwb_driver::msg::UwbRange>::SharedPtr uwb_subscriber_;
        
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr drone_pose_subscriber_;

        void multilateration(const uwb_driver::msg::UwbRange::SharedPtr msg);
        
        void setUwbMarker(uwbMeasurement uwb_info);        

        void setCurrentPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        Vec3d getCurrentPosition();

        void timer_callback();

        rclcpp::TimerBase::SharedPtr mark_timer_;

        visualization_msgs::msg::Marker single_anchor_;
        visualization_msgs::msg::Marker text_marker_;

        visualization_msgs::msg::MarkerArray anchor_marker_array;
        visualization_msgs::msg::MarkerArray text_marker_array;

        bool valid = false;

        Vec3d drone_pos;
        Vec4d drone_att;
        
    };
}  // namespace navigation

#endif // _UWB_LOCALIZER_HPP_