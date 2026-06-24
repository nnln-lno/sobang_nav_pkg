#include "sobang_navigation/uwbLocalizer.hpp"
#include <functional>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace navigation
{
    UWBLocalizer::UWBLocalizer() : Node("uwb_localizer_node")
    {
        this->declare_parameter("anchor_id_lists", std::vector<int>{});
        this->declare_parameter("anchor_pos_x", std::vector<double>{0.0, 0.0, 0.0});
        this->declare_parameter("anchor_pos_y", std::vector<double>{0.0, 0.0, 0.0});
        this->declare_parameter("anchor_pos_z", std::vector<double>{0.0, 0.0, 0.0});
        this->declare_parameter("num_anchors", 0);
        this->declare_parameter("sim_sonar", false);
        this->declare_parameter("is_imu_ned", false);

        std::vector<uint64_t> tmp;

        this->get_parameter("anchor_pos_x", anchor_list_x_);
        this->get_parameter("anchor_pos_y", anchor_list_y_);
        this->get_parameter("anchor_pos_z", anchor_list_z_);
        this->get_parameter("uwb_topic", uwb_topic_);
        this->get_parameter("sim_sonar", sim_sonar_);
        this->get_parameter("is_imu_ned", imu_ned_);

        try
        {
            anchor_id_lists_ = this->get_parameter("anchor_id_lists").as_integer_array();
        }
        catch (const rclcpp::exceptions::ParameterNotDeclaredException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Anchor ID list parameter not declared. Check Config again.");
            throw e; // Re-throw the exception after logging the error
        }

        num_anchors_ = anchor_id_lists_.size();

        uwb_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/uwb/anchors", 10);
        uwb_text_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/uwb/anchors_texts", 10);

        for(uint i=0; i < num_anchors_; i++)
        {
            id_matcher_[i] = anchor_id_lists_[i];
            uwb_struct_[i].anchor_id = anchor_id_lists_[i];
            uwb_struct_[i].anchor_position_x_ = anchor_list_x_[i];
            uwb_struct_[i].anchor_position_y_ = -anchor_list_y_[i];
            uwb_struct_[i].anchor_position_z_ = anchor_list_z_[i];

            setUwbMarker(uwb_struct_[i]);

            anchor_marker_array.markers.push_back(single_anchor_);
            text_marker_array.markers.push_back(text_marker_);

            anchor_positions_.push_back(Vec3d(anchor_list_x_[i], -anchor_list_y_[i], anchor_list_z_[i]));

            // RCLCPP_INFO(this->get_logger(), "Anchor ID: %d, Position: [%.2f, %.2f, %.2f]", 
            //             uwb_struct_[i].anchor_id, 
            //             uwb_struct_[i].anchor_position_x_, 
            //             uwb_struct_[i].anchor_position_y_, 
            //             uwb_struct_[i].anchor_position_z_);
        }

        mark_timer_ = this->create_wall_timer(100ms, std::bind(&UWBLocalizer::timer_callback, this));

        uwb_position_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/uwb/position", 100);

        uwb_range_publisher_ = this->create_publisher<sobang_navigation::msg::UwbData>("/uwb/range_array", 100);

        sonar_publisher_ = this->create_publisher<sensor_msgs::msg::Range>("/sonar/range", 10);

        drone_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/nav/localState", 10, std::bind(&UWBLocalizer::setCurrentPose, this, _1));

        uwb_subscriber_ = this->create_subscription<uwb_driver::msg::UwbRange>(
            uwb_topic_, 10, std::bind(&UWBLocalizer::multilateration, this, _1));

        RCLCPP_INFO(this->get_logger(), "Success initializing UWB Localizer Node and %d Anchors Positions.", num_anchors_);
    }

    void UWBLocalizer::multilateration(const uwb_driver::msg::UwbRange::SharedPtr msg)
    {
        if (!do_multilateration_)
        {
            return;
        }

        Vec3d opt_pos = getCurrentPosition();
        
        // std::cout << "Current Position: [" << opt_pos.transpose() << "], Received UWB Range Data: [";
        // Vec3d opt_pos = Vec3d{0.0, 0.0, 0.0}; // For testing without DR

        // Multilateration part
        std::vector<float> dist = msg->dist; // UWB range measurements to anchors
        std::vector<int> anc_id = msg->anchor_ids;
        std::vector<float> temp_res;
        
        for (int i = 0; i < (int)msg->dist.size(); i++)
        {
            dist[i] = msg->dist[i] * 1e-2;
            anc_id[i] = msg->anchor_ids[i] - 1;
            temp_res.push_back(dist[i] - (anchor_positions_[anc_id[i]] - opt_pos).norm());            
        }        

        std::vector<int> idx(anc_id.size());
        std::iota(idx.begin(), idx.end(), 0); // Create an index vector

        std::sort(idx.begin(), idx.end(), [&dist](int i1, int i2)
                  { return dist[i1] < dist[i2]; }); // Sort indices based on distance

        int count = std::count_if(temp_res.begin(), temp_res.end(), [](float d) { return d < 2.0; });

        if ((count > 3) & (dist.size() >= 4))
        // if (dist.size() >= 4)
        {
            geometry_msgs::msg::PointStamped uwb_estimated_position_;

            double term_cond = 1e-4;
            double min_cond = 1e-1;

            double lamb = 0.5;
            int iter = 100;

            VecXd err_arr(iter);
            MatXd pos_arr(3, iter);

            for (int opt = 0; opt < iter; opt++)
            {
                MatXd J(4,3);
                Vec4d res;

                // Use most 4 closest anchors for multilateration
                for (int i = 0; i < 4; i++)
                {                    
                    J.row(i) = (anchor_positions_[anc_id[idx[i]]] - opt_pos).transpose() / (anchor_positions_[anc_id[idx[i]]] - opt_pos).norm(); 
                    res(i) = dist[idx[i]] - (anchor_positions_[anc_id[idx[i]]] - opt_pos).norm();
                }

                Vec3d d_pos = (J.transpose() * J + lamb * Mat3d::Identity()).inverse() * (J.transpose() * res);
                
                opt_pos = opt_pos - d_pos;

                err_arr(opt) = res.norm();
                pos_arr.col(opt) = opt_pos;

                if (opt > 0)
                {
                    if ((abs(err_arr(opt) - err_arr(opt-1)) < term_cond) || (err_arr(opt) < min_cond))
                    {
                        uwb_estimated_position_.header.frame_id = "map";
                        uwb_estimated_position_.header.stamp = this->get_clock()->now();

                        uwb_estimated_position_.point.x = opt_pos.x();
                        uwb_estimated_position_.point.y = opt_pos.y();
                        uwb_estimated_position_.point.z = opt_pos.z();

                        // std::cout << "estimated Position: [" << opt_pos.transpose() << "], Error: " << err_arr(opt) << std::endl;

                        uwb_position_publisher_->publish(uwb_estimated_position_);
                        // RCLCPP_INFO(this->get_logger(), "Success to publish ! Position : [%.2f, %.2f, %.2f], Error: %.4f", opt_pos.x(), opt_pos.y(), opt_pos.z(), err_arr(opt));
                        return;
                    }

                    if (err_arr(opt) < err_arr(opt - 1))
                    {
                        lamb = std::max(lamb / 9, 1e-7); // Decrease lambda if error is decreasing
                    }
                    else
                    {
                        lamb = std::min(lamb * 9, 1e+7); // Increase lambda if error is increasing
                    }
                }
            }

            int min_idx;
            double min_err = err_arr.minCoeff(&min_idx);

            if (min_err < 0.15)
            {
                uwb_estimated_position_.header.frame_id = "map";
                uwb_estimated_position_.header.stamp = this->get_clock()->now();

                uwb_estimated_position_.point.x = pos_arr(0, min_idx);
                uwb_estimated_position_.point.y = pos_arr(1, min_idx);
                uwb_estimated_position_.point.z = pos_arr(2, min_idx);

                uwb_position_publisher_->publish(uwb_estimated_position_);
                // RCLCPP_INFO(this->get_logger(), "Multilateration did not converge after full iteration with error %.4f", min_err);
            }
        }
        // else if ((count >= 1) && (count <= 3))
        // {
        //     sobang_navigation::msg::UwbData uwb_range_array;

        //     for (int i=0; i<count; i++)
        //     {
        //         uwb_range_array.ranges.push_back(dist[idx[i]]);
        //         uwb_range_array.anchor_ids.push_back(anc_id[idx[i]]);

        //         geometry_msgs::msg::Pose anc_pose;

        //         anc_pose.position.x = anchor_positions_[anc_id[idx[i]]].x();
        //         anc_pose.position.y = anchor_positions_[anc_id[idx[i]]].y();
        //         anc_pose.position.z = anchor_positions_[anc_id[idx[i]]].z();

        //         uwb_range_array.pose.poses.push_back(anc_pose);                
        //     }            
        //     // uwb_range_publisher_->publish(uwb_range_array);
        // }
    }

    void UWBLocalizer::setCurrentPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        do_multilateration_ = true;

        drone_pos.x() = msg->pose.position.x;
        drone_pos.y() = msg->pose.position.y;
        drone_pos.z() = msg->pose.position.z;

        drone_att.x() = msg->pose.orientation.x;
        drone_att.y() = msg->pose.orientation.y;
        drone_att.z() = msg->pose.orientation.z;
        drone_att.w() = msg->pose.orientation.w;
    }

    Vec3d UWBLocalizer::getCurrentPosition()
    {
        return drone_pos;
    }

    void UWBLocalizer::timer_callback()
    {
        // 마커 현재 퍼블리시 안되고있습니다. [TBD]
        uwb_marker_publisher_->publish(anchor_marker_array);
        uwb_text_publisher_->publish(text_marker_array);

        if (sim_sonar_)
        {
            sensor_msgs::msg::Range sonar_msg_;
            sonar_msg_.header.frame_id = "map";
            sonar_msg_.header.stamp = this->get_clock()->now();
        
            std::mt19937 rng{std::random_device{}()};
            double ht = 0.96 + 0.1* std::normal_distribution<double>{0.0, 0.5}(rng); 
            sonar_msg_.range = ht;

            sonar_publisher_->publish(sonar_msg_);
        }        
    }

    void UWBLocalizer::setUwbMarker(uwbMeasurement uwb_info)
    {   
        single_anchor_.header.frame_id = "map";
        single_anchor_.header.stamp = this->get_clock()->now();
        single_anchor_.ns = "uwb_anchors";
        single_anchor_.id = uwb_info.anchor_id;
        single_anchor_.type = visualization_msgs::msg::Marker::CUBE;
        single_anchor_.action = visualization_msgs::msg::Marker::ADD;

        single_anchor_.pose.position.x = uwb_info.anchor_position_x_;
        single_anchor_.pose.position.y = uwb_info.anchor_position_y_;
        single_anchor_.pose.position.z = uwb_info.anchor_position_z_;

        single_anchor_.scale.x = 0.2;
        single_anchor_.scale.y = 0.2;
        single_anchor_.scale.z = 0.2;
        single_anchor_.pose.orientation.w = 1.0;

        single_anchor_.color.a = 1.0; // Alpha
        single_anchor_.color.r = 1.0; // Red
        single_anchor_.color.g = 0.0; // Green
        single_anchor_.color.b = 0.0; // Blue        
        single_anchor_.lifetime = rclcpp::Duration(0, 0);

        text_marker_.header.frame_id = "map";
        text_marker_.header.stamp = this->now();
        text_marker_.ns = "uwb_labels";
        text_marker_.id = uwb_info.anchor_id; // Ensure unique ID for text marker
        text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker_.action = visualization_msgs::msg::Marker::ADD;
        text_marker_.text = "Anchor_" + std::to_string(uwb_info.anchor_id);

        text_marker_.pose.position.x = uwb_info.anchor_position_x_;
        text_marker_.pose.position.y = uwb_info.anchor_position_y_;
        text_marker_.pose.position.z = uwb_info.anchor_position_z_ + 0.4; // 구 위에 표시
        text_marker_.pose.orientation.w = 1.0;

        text_marker_.scale.z = 1; // 텍스트 크기
        text_marker_.color.r = 1.0;
        text_marker_.color.g = 1.0;
        text_marker_.color.b = 1.0;
        text_marker_.color.a = 1.0;        
        text_marker_.lifetime = rclcpp::Duration(0, 0);
    }
}  // namespace navigation