#include "sobang_navigation/uwbLocalizer.hpp"
#include <functional>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace navigation {
UWBLocalizer::UWBLocalizer() : Node("uwb_localizer_node") {
  this->declare_parameter("anchor_id_lists", std::vector<int>{});
  this->declare_parameter("anchor_pos_x", std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter("anchor_pos_y", std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter("anchor_pos_z", std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter("num_anchors", 0);
  this->declare_parameter("sim_sonar", false);
  this->declare_parameter("uwb_topic", "/uwb/range");
  this->declare_parameter("is_imu_ned", false);

  std::vector<uint64_t> tmp;

  this->get_parameter("anchor_pos_x", anchor_list_x_);
  this->get_parameter("anchor_pos_y", anchor_list_y_);
  this->get_parameter("anchor_pos_z", anchor_list_z_);
  this->get_parameter("uwb_topic", uwb_topic_);
  this->get_parameter("sim_sonar", sim_sonar_);
  this->get_parameter("is_imu_ned", imu_ned_);

  try {
    anchor_id_lists_ = this->get_parameter("anchor_id_lists").as_integer_array();
  } catch (const rclcpp::exceptions::ParameterNotDeclaredException &e) {
    RCLCPP_ERROR(this->get_logger(), "Anchor ID list parameter not declared. Check Config again.");
    throw e; // Re-throw the exception after logging the error
  }

  num_anchors_ = anchor_id_lists_.size();  

  for (uint i = 0; i < num_anchors_; i++) 
  {
    uint uidx = anchor_id_lists_[i] - 1;
    id_matcher_[uidx] = uidx + 1;
    uwb_struct_[uidx].anchor_id = uidx + 1;
    uwb_struct_[uidx].anchor_position_x_ = anchor_list_x_[uidx];
    uwb_struct_[uidx].anchor_position_y_ = anchor_list_y_[uidx];
    uwb_struct_[uidx].anchor_position_z_ = anchor_list_z_[uidx];

    anchor_positions_[uidx] =
        Vec3d(anchor_list_x_[uidx], anchor_list_y_[uidx], anchor_list_z_[uidx]);
  }  

  uwb_position_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/uwb/position", 100);

  uwb_range_publisher_ = this->create_publisher<sobang_navigation::msg::UwbData>("/uwb/range_array", 100);

  sonar_publisher_ = this->create_publisher<sensor_msgs::msg::Range>("/sonar/range", 10);

  drone_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/nav/localState", 10, std::bind(&UWBLocalizer::setCurrentPose, this, _1));

  uwb_subscriber_ = this->create_subscription<uwb_driver::msg::UwbRange>(
      uwb_topic_, 10, std::bind(&UWBLocalizer::multilateration, this, _1));

  RCLCPP_INFO(this->get_logger(), "Success initializing UWB Localizer Node and %d Anchors Positions.", num_anchors_);
}

void UWBLocalizer::multilateration(const uwb_driver::msg::UwbRange::SharedPtr msg) {

  if (!do_multilateration_) {
    return;
  }

  Vec3d opt_pos = getCurrentPosition();  

  // Multilateration part
  std::vector<float> dist = msg->dist; // UWB range measurements to anchors
  std::vector<int> anc_id = msg->anchor_ids;
  std::vector<float> temp_res;

  for (int i = 0; i < (int)msg->dist.size(); i++) {
    dist[i] = msg->dist[i] * 1e-2;
    anc_id[i] = msg->anchor_ids[i] - 1;
    temp_res.push_back(dist[i] - (anchor_positions_[anc_id[i]] - opt_pos).norm());
  }

  std::vector<int> idx(anc_id.size());
  std::iota(idx.begin(), idx.end(), 0); // Create an index vector

  std::sort(idx.begin(), idx.end(), [&dist](int i1, int i2) 
  {
    return dist[i1] < dist[i2];
  }); // Sort indices based on distance

  int count = std::count_if(temp_res.begin(), temp_res.end(), [](float d) { return d < 2.0; });

  if ((count > 3) & (dist.size() >= 4))  
  {
    geometry_msgs::msg::PointStamped uwb_estimated_position_;

    double term_cond = 1e-5;    
    double min_cond = 0.05;

    double lamb = 0.5;
    int iter = 200;

    VecXd err_arr(iter);
    MatXd pos_arr(3, iter);

    for (int opt = 0; opt < iter; opt++) {
      MatXd J(count, 3);
      VecXd res(count);

      // Use most 4 closest anchors for multilateration
      for (int i = 0; i < count; i++) {

        J.row(i) = (anchor_positions_[anc_id[idx[i]]] - opt_pos).transpose() / (anchor_positions_[anc_id[idx[i]]] - opt_pos).norm();
        res(i) = dist[idx[i]] - (anchor_positions_[anc_id[idx[i]]] - opt_pos).norm();
      }

      Vec3d d_pos = (J.transpose() * J + lamb * Mat3d::Identity()).inverse() * (J.transpose() * res);

      opt_pos = opt_pos - d_pos;

      err_arr(opt) = res.norm();
      pos_arr.col(opt) = opt_pos;

      if (opt > 0) {
        if ((abs(err_arr(opt) - err_arr(opt - 1)) < term_cond) || (err_arr(opt) < min_cond)) 
        {
          uwb_estimated_position_.header.frame_id = "map";
          uwb_estimated_position_.header.stamp = this->get_clock()->now();

          uwb_estimated_position_.point.x = opt_pos.x();
          uwb_estimated_position_.point.y = opt_pos.y();
          uwb_estimated_position_.point.z = opt_pos.z();

          uwb_position_publisher_->publish(uwb_estimated_position_);
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

    if (min_err < 0.1) {
      uwb_estimated_position_.header.frame_id = "map";
      uwb_estimated_position_.header.stamp = this->get_clock()->now();

      uwb_estimated_position_.point.x = pos_arr(0, min_idx);
      uwb_estimated_position_.point.y = pos_arr(1, min_idx);
      uwb_estimated_position_.point.z = pos_arr(2, min_idx);

      uwb_position_publisher_->publish(uwb_estimated_position_);
    }
  } 
  // 거리 측정이 1~3개인 경우에는 UWB range array를 퍼블리시합니다.
  else if ((count >= 1) && (count <= 3)) 
  {
    sobang_navigation::msg::UwbData uwb_range_array;

    for (int i = 0; i < count; i++) {
      uwb_range_array.ranges.push_back(dist[idx[i]]);
      uwb_range_array.anchor_ids.push_back(anc_id[idx[i]]);

      geometry_msgs::msg::Pose anc_pose;

      anc_pose.position.x = anchor_positions_[anc_id[idx[i]]].x();
      anc_pose.position.y = anchor_positions_[anc_id[idx[i]]].y();
      anc_pose.position.z = anchor_positions_[anc_id[idx[i]]].z();

      uwb_range_array.pose.poses.push_back(anc_pose);
    }
    uwb_range_publisher_->publish(uwb_range_array);
  }
}

void UWBLocalizer::setCurrentPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  do_multilateration_ = true;

  drone_pos.x() = msg->pose.position.x;
  drone_pos.y() = msg->pose.position.y;
  drone_pos.z() = msg->pose.position.z;

  drone_att.x() = msg->pose.orientation.x;
  drone_att.y() = msg->pose.orientation.y;
  drone_att.z() = msg->pose.orientation.z;
  drone_att.w() = msg->pose.orientation.w;
}

Vec3d UWBLocalizer::getCurrentPosition() { return drone_pos; }

} // namespace navigation
