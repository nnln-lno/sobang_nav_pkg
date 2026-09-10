#include "sobang_navigation/navigation.hpp"
#include <functional>
#include <limits>
#include <random>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace navigation {
Navigation::Navigation() : Node("sobang_navigation_node"), count_(0) {
  param_setting();

  auto sensor_qos = rclcpp::SensorDataQoS();
  rclcpp::QoS qos_profile(10);
  qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  
  state_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/nav/localState", 10);

  px4_state_publisher_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry", 10);

  radar_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(radar_topic_, sensor_qos, std::bind(&Navigation::radar_callback, this, _1));

  // IMU TOPIC CONDITION
  if ( imu_topic_ == "/imu_apps" )
  {
     imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic_, sensor_qos, std::bind(&Navigation::imu_callback, this, _1));
     RCLCPP_INFO(this->get_logger(), "VOXL2 IMU detected : topic name = /imu_apps");
  }
  else if ( imu_topic_ == "/fmu/out/sensor_combined" )
  {
    px4_imu_subscriber_ = this->create_subscription<px4_msgs::msg::SensorCombined>(imu_topic_, sensor_qos, std::bind(&Navigation::px4_imu_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "PX4 IMU detected : topic name = /fmu/out/sensor_combined");
  }
  // IMU TOPIC CONDITION

  uwb_position_subscriber_ =
  this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/uwb/position", 100, std::bind(&Navigation::uwbPositionCallback, this,
    _1));

  uwb_range_subscriber_ =
      this->create_subscription<sobang_navigation::msg::UwbData>(
          "/uwb/range_array", 100,
          std::bind(&Navigation::uwbRangeCallback, this, _1));

  // SONAR TOPIC CONDITION          
  if (sonar_topic_ == "/fmu/out/distance_sensor") 
  {
    px4_sonar_subscriber_ = this->create_subscription<px4_msgs::msg::DistanceSensor>(sonar_topic_, qos_profile, std::bind(&Navigation::px4_sonarCallback, this, _1));
    RCLCPP_INFO(this->get_logger(), "PX4 Sonar detected : topic name = /fmu/out/distance_sensor");
  } 
  else 
  {
    ros2_sonar_subscriber_ = this->create_subscription<sensor_msgs::msg::Range>(sonar_topic_, 10, std::bind(&Navigation::ros2_sonarCallback, this, _1));\
    RCLCPP_INFO(this->get_logger(), "MB1242 Sonar detected : topic name = %s", sonar_topic_.c_str());
  }
  // SONAR TOPIC CONDITION    

  if (view_path_) 
  {
    path_publisher_ =
        this->create_publisher<nav_msgs::msg::Path>("/nav/localPath", 10);
    RCLCPP_INFO(this->get_logger(), "Drone Path Publisher Enabled : topic name = /nav/localPath");
  }

  if (pub_egovel_) 
  {
    ego_vel_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/radar/egovel", 10);
    RCLCPP_INFO(this->get_logger(), "Ego Velocity Publisher Enabled : topic name = /radar/egovel");
  }

  if (pub_icp_) 
  {
    icp_state_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/icp/pose", 10);
    RCLCPP_INFO(this->get_logger(), "ICP State Publisher Enabled : topic name = /icp/pose");
  }

  setState(init_pos_, init_att_, init_gyro_bias_, Vec3d{1.0, 1.0, 1.0}); // for DR alignment

  timer_ = this->create_wall_timer(100ms, std::bind(&Navigation::timer_callback, this));

  px4_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / px4_fc_rate_), std::bind(&Navigation::px4_timer_callback, this));
}

// [IMU CALLBACK]
void Navigation::imu_callback(const sensor_msgs::msg::Imu::SharedPtr i_msg) {
  sensor_msgs::msg::Imu::SharedPtr msg =
      std::make_shared<sensor_msgs::msg::Imu>(*i_msg);

  setImuCurrentTime(msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);

  imu_cnt++;

  // init bias aligment IMU data.
  if (init_alignment_) {
    if (do_align_) 
    {
      RCLCPP_INFO_ONCE(this->get_logger(), "[WARN] DONT MOVE THE DRONE UNTIL INITIAL ALIGNMENT IS COMPLETE! (%.1f seconds)", align_time_);
    } else {
      RCLCPP_INFO_ONCE(this->get_logger(), "[INFO] INITIAL ALIGNMENT SKIPPED! BE CAREFUL WITH THE DRONE'S MOVEMENT AT THE BEGINNING.");
    }    
    initAlignment(msg);
    return; // Skip processing until initial alignment is complete
  }

  if (getImuCurrentTime() <= getImuPreviousTime()) {
    RCLCPP_WARN(this->get_logger(), "[WARN] Received IMU data with non-increasing timestamp. Skipping this measurement.");
    return;
  }

  if (getImuCurrentTime() <= getImuPreviousTime()) {
    RCLCPP_WARN(this->get_logger(), "[WARN] Received IMU data with non-increasing timestamp. Skipping this measurement.");
    return; // Skip processing if time is not moving forward
  }

  setImuTimeDelta();
  if (imu_cnt == 1)
  {
    imu_time_delta_ = 1.0 / imu_rate; // For the first IMU callback, set a default time delta based on the expected IMU rate
  }

  Vec3d w_b = Vec3d{msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z} - getState().gyro_bias;
  omega = w_b;

  if (!has_problems_) 
  {
    DeadReckoning(getState(), radar_estimator_.getEgoVelocity(), w_b, getImuTimeDelta());
    timeUpdate(getState(), radar_estimator_.getEgoVelocity(), w_b, getImuTimeDelta());
  } else 
  {
    DeadReckoning(getState(), Vec3d{0.0, 0.0, 0.0}, w_b, getImuTimeDelta());
    timeUpdate(getState(), Vec3d{0.0, 0.0, 0.0}, w_b, getImuTimeDelta());
  }

  Pk = (Fk * Pk * Fk.transpose()) + (Gk * Qk * Gk.transpose());

  if (has_clone_) { Pxc = (Fk * Pxc); }

  // ------------------------ Visualization Part ------------------------
  publishDronePath(getState().position, getState().quaternion);

  setImuPreviousTime(getImuCurrentTime());
}

void Navigation::px4_imu_callback(const px4_msgs::msg::SensorCombined::SharedPtr i_msg)
{
  px4_msgs::msg::SensorCombined::SharedPtr msg = std::make_shared<px4_msgs::msg::SensorCombined>(*i_msg);

  // auto t =  std::chrono::steady_clock::now();
  // auto us = std::chrono::duration_cast<std::chrono::microseconds>(t.time_since_epoch()).count();

  setImuCurrentTime(msg->timestamp * 1e-6);

  imu_cnt++;

  // init bias aligment IMU data.
  if (init_alignment_) {
    if (do_align_) 
    {
      RCLCPP_INFO_ONCE(this->get_logger(), "[WARN] DONT MOVE THE DRONE UNTIL INITIAL ALIGNMENT IS COMPLETE! (%.1f seconds)", align_time_);
    } else {
      RCLCPP_INFO_ONCE(this->get_logger(), "[INFO] INITIAL ALIGNMENT SKIPPED! BE CAREFUL WITH THE DRONE'S MOVEMENT AT THE BEGINNING.");
    }    
    px4_initAlignment(msg);
    return; // Skip processing until initial alignment is complete
  }

  if (getImuCurrentTime() <= getImuPreviousTime()) {
    RCLCPP_WARN(this->get_logger(), "[WARN] Received IMU data with non-increasing timestamp. Skipping this measurement.");
    return;
  }

  setImuTimeDelta();
  if (imu_cnt == 1)
  {
    imu_time_delta_ = 1.0 / imu_rate; // For the first IMU callback, set a default time delta based on the expected IMU rate
  }  

  Vec3d w_b = Vec3d{msg->gyro_rad[0], msg->gyro_rad[1],msg->gyro_rad[2]} - getState().gyro_bias;
  omega = w_b;

  if (!has_problems_) 
  {
    DeadReckoning(getState(), radar_estimator_.getEgoVelocity(), w_b, getImuTimeDelta());
    timeUpdate(getState(), radar_estimator_.getEgoVelocity(), w_b, getImuTimeDelta());
  } else 
  {
    // MODE 4 : If radar has problems, Zero Velocity Update conducted
    DeadReckoning(getState(), Vec3d{0.0, 0.0, 0.0}, w_b, getImuTimeDelta());
    timeUpdate(getState(), Vec3d{0.0, 0.0, 0.0}, w_b, getImuTimeDelta());
  }

  Pk = (Fk * Pk * Fk.transpose()) + (Gk * Qk * Gk.transpose());

  if (has_clone_) { Pxc = (Fk * Pxc); }

  // ------------------------ Visualization Part ------------------------
  publishDronePath(getState().position, getState().quaternion);

  setImuPreviousTime(getImuCurrentTime());

}

// [RADAR CALLBACK]
void Navigation::radar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  
  radar_valid = 0; // If radar callback is called, reset the radar_valid counter to 0

  radar_estimator_.radarParser(msg); // Here, parsing point cloud data.

  // ICP 처음 단계에서는, 해당 시점에서 이전 포인트 클라우드가 없으므로, 현재 포인트 클라우드를 이전 포인트 클라우드로 설정
  if (icp_cnt == 0) { radar_estimator_.setPreviousPoints(radar_estimator_.getPointMatrix()); }

  icp_cnt++;

  icp_current_state.position = getState().position;
  icp_current_state.quaternion = getState().quaternion;

  setRadarCurrentTime(msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);

  if (!init_alignment_) // Skip processing until initial alignment is complete
  {
    radar_estimator_.egoVelocityEstimator();
    
    if (pub_egovel_) 
    {
      Vec3d ego_vel_ = Cbi * Cir * radar_estimator_.getEgoVelocity();

      geometry_msgs::msg::TwistStamped ego_vel_msgs;

      ego_vel_msgs.header.frame_id = "map";
      ego_vel_msgs.header.stamp = this->get_clock()->now();

      ego_vel_msgs.twist.linear.x = ego_vel_(0);
      ego_vel_msgs.twist.linear.y = ego_vel_(1);
      ego_vel_msgs.twist.linear.z = ego_vel_(2);

      ego_vel_msgs.twist.angular.x = 0.0;
      ego_vel_msgs.twist.angular.y = 0.0;
      ego_vel_msgs.twist.angular.z = 0.0;

      ego_vel_publisher_->publish(ego_vel_msgs);
    }
    
    if (radar_estimator_.getEgoVelocity().norm() < 0.01)
    {
      radar_estimator_.setEgoVelocity(Vec3d{0.0, 0.0, 0.0});
    }
  } 
  else 
  {
    radar_estimator_.setEgoVelocity(Vec3d{0.0, 0.0, 0.0}); // During initial alignment, we assume the drone is stationary.
    return;
  }


  // 현재 자세와 이전 기준 자세간의 상대 변위 및 상대 회전 계산
  Mat3d rel_R = (quat2dcm(icp_prev_state.quaternion) * Cbi * Cir).transpose() * (quat2dcm(icp_current_state.quaternion) * Cbi * Cir);
  Vec3d rel_t = (quat2dcm(icp_prev_state.quaternion) * Cbi * Cir).transpose() * ((icp_current_state.position + quat2dcm(icp_current_state.quaternion) * tbr) 
                - (icp_prev_state.position + quat2dcm(icp_prev_state.quaternion) * tbr));

  // 10번째 스캔 마다 ICP 수행                
  if (icp_cnt % 10 == 0) 
  {

    // 현재 시점 점군 정보를 넣고 ICP 수행
    radar_estimator_.setCurrentPoints(radar_estimator_.getPointMatrix());

    if (radar_estimator_.simpleRadar2DIcp(rel_R, rel_t)) {

      icpState icp_result = radar_estimator_.getIcpPose();

      // ICP 기반의 자세 누적
      icp_att_sum += quat2euler(icp_result.quaternion)(2) * r2d;
      icp_pos_sum += Vec3d{icp_result.position(0), icp_result.position(1), quat2euler(icp_result.quaternion)(2)};

      if (pub_icp_) 
      {
        geometry_msgs::msg::PoseStamped icp_pose_msg;

        icp_pose_msg.header.frame_id = "map";
        icp_pose_msg.header.stamp = this->get_clock()->now();

        icp_pose_msg.pose.position.x = icp_pos_sum(0);
        icp_pose_msg.pose.position.y = icp_pos_sum(1);
        icp_pose_msg.pose.position.z = 0.0;

        Vec4d icp_quat = euler2quat(Vec3d{0.0, 0.0, icp_att_sum * d2r});

        icp_pose_msg.pose.orientation.x = icp_quat(1);
        icp_pose_msg.pose.orientation.y = icp_quat(2);
        icp_pose_msg.pose.orientation.z = icp_quat(3);
        icp_pose_msg.pose.orientation.w = icp_quat(0);

        icp_state_publisher_->publish(icp_pose_msg);
      }

      Vec3d z_t = icp_result.position - rel_t;
      Mat3d z_R = rel_R.transpose() * quat2dcm(icp_result.quaternion);

      double th_ = acos(std::clamp((z_R.trace() - 1.0) / 2.0, -1.0, 1.0));
      
      Vec3d z_att = Vec3d::Zero(3, 1);

      if (th_ < 1e-6)
      {
        z_att = 0.5 * Vec3d{z_R(2, 1) - z_R(1, 2), z_R(0, 2) - z_R(2, 0), z_R(1, 0) - z_R(0, 1)};
      } 
      else
      {
        z_att = (th_) / (2.0 * sin(th_)) * Vec3d{z_R(2, 1) - z_R(1, 2), z_R(0, 2) - z_R(2, 0), z_R(1, 0) - z_R(0, 1)};
      }      
      
      Vec3d d_vec = ( icp_current_state.position + quat2dcm(icp_current_state.quaternion) * tbr ) - ( icp_prev_state.position + quat2dcm(icp_prev_state.quaternion) * tbr );

      Mat3d A = quat2dcm(icp_prev_state.quaternion) * Cbi * Cir;      
      Mat3d B = quat2dcm(icp_current_state.quaternion) * Cbi * Cir;
      Mat3d ATB = A.transpose() * B;

      double ATB_th_ = acos(std::clamp((ATB.trace() - 1.0) / 2.0, -1.0, 1.0));
      Vec3d ATB_att = (ATB_th_) / (2.0 * sin(ATB_th_)) * Vec3d{ATB(2, 1) - ATB(1, 2), ATB(0, 2) - ATB(2, 0), ATB(1, 0) - ATB(0, 1)};
      Mat3d Jr_inv = Mat3d::Identity() + (0.5 * skew33(ATB_att));

      // STATE CLONING
      MatXd Full_Hk = MatXd::Zero(6, 18);

      Full_Hk.block<3,3>(0,0) = A.transpose();
      Full_Hk.block<3,3>(0,3) = -A.transpose() * skew33(quat2dcm(icp_current_state.quaternion) * tbr);
      Full_Hk.block<3,3>(0,12) = -A.transpose();
      Full_Hk.block<3,3>(0,15) = A.transpose() * skew33(quat2dcm(icp_prev_state.quaternion) * tbr + d_vec);

      Full_Hk.block<3,3>(3,3) = Jr_inv * B.transpose();
      Full_Hk.block<3,3>(3,15) = -Jr_inv * B.transpose();

      Vec6d R_icp_vec = icp_cov_;
      Mat6d R_icp = (R_icp_vec.cwiseProduct(R_icp_vec)).asDiagonal();

      measurementUpdate(getState(), Vec6d{z_t(0), z_t(1), z_t(2), z_att(0), z_att(1), z_att(2)}, Full_Hk, R_icp); // Only consider x, y translation and yaw rotation for 2D ICP

      // After the ICP update, reset the clone state and covariance
      has_clone_ = false;
      Pxc.setZero();
      Pcc.setZero();
    }
    icp_prev_state = icp_current_state;
    radar_estimator_.setPreviousPoints(radar_estimator_.getCurrentPoints());

    initializeCloneCovariance(); // Initialize clone covariance matrices after ICP update
  } 
  setRadarTimeDelta();
  setRadarPreviousTime(getRadarCurrentTime());
}

// [UWB POSITION CALLBACK]
void Navigation::uwbPositionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  
  Vec3d uwb_position(msg->point.x, msg->point.y, msg->point.z); // Vec3d residual = uwb_position - getState().position;
  
  Vec3d residual = uwb_position - (getState().position + quat2dcm(getState().quaternion) * (Cbi * tiu + tbi));

  MatXd Hk = MatXd::Zero(3, 12);
  Hk.block<3, 3>(0, 0) = Mat3d::Identity();
  Hk.block<3, 3>(0, 3) = -skew33(quat2dcm(getState().quaternion) * (Cbi * tiu + tbi));

  if (!init_alignment_) { measurementUpdate(getState(), residual, Hk, R_uwb); }
  
  // 레이더 센서에 문제가 생긴 경우
  if (has_problems_) 
  {

    // MODE 4 : If radar has problems, Replace position as UWB position
    Vec3d px4_cur_pos = uwb_position;
    Vec4d px4_cur_att = getState().quaternion;

    pose.header.frame_id = "map";
    pose.header.stamp = this->get_clock()->now();

    pose.pose.position.x = px4_cur_pos(0);
    pose.pose.position.y = px4_cur_pos(1);
    pose.pose.position.z = -prev_dist;

    pose.pose.orientation.x = px4_cur_att(1);
    pose.pose.orientation.y = px4_cur_att(2);
    pose.pose.orientation.z = px4_cur_att(3);
    pose.pose.orientation.w = px4_cur_att(0);

    state_publisher_->publish(pose);

    // ---------------------------------------------------------------------

    px4_pose.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    px4_pose.timestamp_sample = px4_pose.timestamp;
    px4_pose.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;

    px4_pose.position = {(float)px4_cur_pos(0), (float)px4_cur_pos(1), (float)px4_cur_pos(2)};
    px4_pose.q = {(float)px4_cur_att(0), (float)px4_cur_att(1), (float)px4_cur_att(2), (float)px4_cur_att(3)};

    px4_pose.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
    px4_pose.velocity.fill(std::numeric_limits<float>::quiet_NaN());

    px4_pose.angular_velocity = {(float)omega(0), (float)omega(1), (float)omega(2)};

    px4_pose.position_variance = {(float)px4_pos_cov_(0), (float)px4_pos_cov_(1), (float)px4_pos_cov_(2)};
    px4_pose.velocity_variance = {0.01, 0.01, 0.01};
    px4_pose.orientation_variance = {(float)px4_att_cov_(0), (float)px4_att_cov_(1), (float)px4_att_cov_(2)};

    px4_state_publisher_->publish(px4_pose);
  }
}

void Navigation::uwbRangeCallback(const sobang_navigation::msg::UwbData::SharedPtr msg) {
  std::vector<float> dist = msg->ranges;
  std::vector<int16_t> anc_id = msg->anchor_ids;

  std::vector<geometry_msgs::msg::Pose> anc_poses = msg->pose.poses;

  int count = dist.size();

  VecXd residual = VecXd::Zero(count);
  MatXd Hk = MatXd::Zero(count, 12);

  for (int i = 0; i < count; i++) 
  {
    Vec3d uwb_pos = getState().position + quat2dcm(getState().quaternion) * (tbi + Cbi * tiu);
    Vec3d anc_pos(anc_poses[i].position.x, anc_poses[i].position.y, anc_poses[i].position.z);

    residual(i) = dist[i] - (anc_pos - uwb_pos).norm();

    Hk.row(i).block<1, 3>(0, 0) = -(anc_pos - uwb_pos).transpose() / (anc_pos - uwb_pos).norm();
  }

  if (!init_alignment_) { measurementUpdate(getState(), residual, Hk, R_uwb_range * MatXd::Identity(count, count)); }
}

void Navigation::ros2_sonarCallback(const sensor_msgs::msg::Range::SharedPtr msg) {

  double sonar_range = msg->range;

  sonar_cnt++;

  // Sonar measurement filtering [To be developed]
  if (sonar_range >= 7.64999 ) {  
    return;
  } 
  else if ( ( std::abs(sonar_range - prev_dist) >= 1.5 ) && sonar_cnt < 10) 
  {
    return;
  }
  else if (sonar_range <= 0.209) 
  {
    sonar_range = 0.0;    
  }

  double residual = (sonar_range) - (-(getState().position.z() - tis.z())); // Assuming sonar measures height

  prev_dist = sonar_range;

  MatXd Hk = MatXd::Zero(1, 12);
  Hk(0, 2) = -1.0; // Derivative of measurement w.r.t z position

  if (!init_alignment_) { measurementUpdate(getState(), Vec1d{residual}, Hk, R_sonar); }// [HYPERPARAM] Sonar measurement noise covariance

  sonar_cnt = 0;
}

void Navigation::px4_sonarCallback(const px4_msgs::msg::DistanceSensor::SharedPtr msg) {

  double sonar_range = msg->current_distance;

  sonar_cnt++;

  // Sonar measurement filtering [To be developed]
  if (sonar_range >= 7.64999 ) {  
    return;
  } 
  else if ( ( std::abs(sonar_range - prev_dist) >= 1.5 ) && sonar_cnt < 10) 
  {
    return;
  }
  else if (sonar_range <= 0.209) 
  {
    sonar_range = 0.0;    
  }

  double residual = (sonar_range) - (-(getState().position.z() - tis.z())); // Assuming sonar measures height

  prev_dist = sonar_range; // Keep recently updated sonar distance for next callback

  MatXd Hk = MatXd::Zero(1, 12);
  Hk(0, 2) = -1.0; // Derivative of measurement w.r.t z position

  if (!init_alignment_) { measurementUpdate(getState(), Vec1d{residual}, Hk, R_sonar); } // [HYPERPARAM] Sonar measurement noise covariance

  sonar_cnt = 0;  
}

// Basic Navigation Functions
void Navigation::initAlignment(const sensor_msgs::msg::Imu::SharedPtr msg) {
  // Accumulate IMU data for initial alignment
  if (msg->linear_acceleration.z < 0) 
  {
    acc_accum += Vec3d({msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z}) + Vec3d({0.0, 0.0, 9.80665});
  } else 
  {
    acc_accum += Vec3d({msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z}) -Vec3d({0.0, 0.0, 9.80665});
  }
  
  gyro_accum += Vec3d{msg->angular_velocity.x, msg->angular_velocity.y,msg->angular_velocity.z};

  alignment_count_++;

  if (!do_align_) // SKIP ALIGNMENT PROCESS
  {
    init_alignment_ = false; // Skip alignment process and start navigation immediately

    setState(init_pos_, init_att_, init_gyro_bias_, Vec3d{1.0, 1.0, 1.0}); // For DR Alignment

    publishDronePath(getState().position, getState().quaternion);

    icp_prev_state.position = getState().position;
    icp_prev_state.quaternion = getState().quaternion;

    initializeCloneCovariance(); // Initialize clone covariance matrices after alignment

    printStateInfo(); // Print initial state information after skipping
                      // alignment
  } 
  else  // PERFORM ALIGNMENT PROCESS
  {
    if (alignment_count_ >= imu_rate * align_time_) // [HYPERPARAM] Align for the specified time duration
    {
      init_alignment_ = false; // Alignment complete

      Vec3d acc_mean = acc_accum / alignment_count_;
      Vec3d gyro_mean = gyro_accum / alignment_count_;

      std::cout << "BIAS ALIGNMENT RESULT : [ACC] = " << acc_mean.transpose() << ", [GYRO] = " << gyro_mean.transpose() << std::endl;

      init_acc_bias_ = acc_mean; 

      setState(init_pos_, init_att_, gyro_mean, Vec3d{1.0, 1.0, 1.0});

      publishDronePath(getState().position, getState().quaternion);

      icp_prev_state.position = getState().position;
      icp_prev_state.quaternion = getState().quaternion;

      initializeCloneCovariance(); // Initialize clone covariance matrices after alignment

      printStateInfo(); // Print initial state information after alignment
    }
  }
  setImuPreviousTime(getImuCurrentTime());
}

void Navigation::px4_initAlignment(const px4_msgs::msg::SensorCombined::SharedPtr msg) {
  // Accumulate IMU data for initial alignment
  if (msg->accelerometer_m_s2[2] < 0) 
  {
    acc_accum += Vec3d({msg->accelerometer_m_s2[0], msg->accelerometer_m_s2[1], msg->accelerometer_m_s2[2]}) + Vec3d({0.0, 0.0, 9.80665});
  } 
  else 
  {
    acc_accum += Vec3d({msg->accelerometer_m_s2[0], msg->accelerometer_m_s2[1], msg->accelerometer_m_s2[2]}) - Vec3d({0.0, 0.0, 9.80665});
  }
  
  gyro_accum += Vec3d{msg->gyro_rad[0], msg->gyro_rad[1], msg->gyro_rad[2]};

  alignment_count_++;

  if (!do_align_) 
  {    
    init_alignment_ = false; // Skip alignment process and start navigation immediately

    setState(init_pos_, init_att_, init_gyro_bias_, Vec3d{1.0, 1.0, 1.0}); // For DR Alignment
      
    publishDronePath(getState().position, getState().quaternion);

    icp_prev_state.position = getState().position;
    icp_prev_state.quaternion = getState().quaternion;

    initializeCloneCovariance(); // Initialize clone covariance matrices after alignment

    printStateInfo(); // Print initial state information after skipping
                      // alignment
  } 
  else 
  {
    if (alignment_count_ >= imu_rate * align_time_) // [HYPERPARAM] Align for the specified time duration
    {
      init_alignment_ = false; // Alignment complete

      Vec3d acc_mean = acc_accum / alignment_count_;
      Vec3d gyro_mean = gyro_accum / alignment_count_;

      std::cout << "BIAS ALIGNMENT RESULT : [ACC] = " << acc_mean.transpose() << ", [GYRO] = " << gyro_mean.transpose() << std::endl;

      init_acc_bias_  = acc_mean; 
      init_gyro_bias_ = gyro_mean;

      setState(init_pos_, init_att_, init_gyro_bias_, Vec3d{1.0, 1.0, 1.0});

      publishDronePath(getState().position, getState().quaternion);

      icp_prev_state.position = getState().position;
      icp_prev_state.quaternion = getState().quaternion;

      initializeCloneCovariance(); // Initialize clone covariance matrices after alignment

      printStateInfo(); // Print initial state information after alignment
    }
  }
  setImuPreviousTime(getImuCurrentTime());
}

void Navigation::DeadReckoning(const drState prev_state, Vec3d ego_velocity, Vec3d angular_rate, double dt) {
  
  Mat3d Cgb = quat2dcm(prev_state.quaternion);

  Mat3d w_skew = skew33(angular_rate);

  Vec3d new_position = prev_state.position + Cgb * Cbi * (Cir * ego_velocity - w_skew * tir) * dt;

  Vec4d new_quaternion = quatUpdate(prev_state.quaternion, Cbi * angular_rate, dt);

  setState(new_position, new_quaternion, prev_state.gyro_bias, prev_state.scale);
}

void Navigation::timeUpdate(const drState prev_state, Vec3d ego_velocity, Vec3d angular_rate, double dt) {
  
  Mat3d Cgb = quat2dcm(prev_state.quaternion);

  Vec3d inv_scale = prev_state.scale.array().inverse();
  Vec3d hat_vr = ego_velocity.cwiseProduct(inv_scale);

  Mat3d w_skew = skew33(angular_rate);
  Mat3d sk_br = skew33(tir);

  // First Row (3x11)
  Vec3d F12_vec = -(Cgb * Cbi * Cir * hat_vr - Cgb * Cbi * w_skew * tir);

  Mat3d F12 = skew33(F12_vec);
  Mat3d F13 = Cgb * Cbi * sk_br;
  Mat3d F14 = -Cgb * Cbi * Cir * hat_vr.asDiagonal();

  // Second Row
  Mat3d F23 = -Cgb * Cbi;

  // Third Row
  Mat3d F33 = (-1 / tau_bg) * Mat3d::Identity();
  Mat3d F44 = (-1 / tau_sr) * Mat3d::Identity();

  Mat12d F = Mat12d::Zero();
  F << Mat3d::Zero(),           F12,           F13,           F14,
       Mat3d::Zero(), Mat3d::Zero(),           F23, Mat3d::Zero(), 
       Mat3d::Zero(), Mat3d::Zero(),           F33, Mat3d::Zero(), 
       Mat3d::Zero(), Mat3d::Zero(), Mat3d::Zero(),           F44;

  Fk = Mat12d::Identity() + (F * dt) + (0.5 * F * F * dt * dt); // Discretized state transition matrix using second-order Taylor expansion                                

  Mat3d G11 = Cgb * Cbi * sk_br;
  Mat3d G13 = -Cgb * Cbi * Cir;

  Mat3d G21 = -Cgb * Cbi;

  Mat3d G32 = Mat3d::Identity();

  Mat3d G44 = Mat3d::Identity();

  Mat12d G = Mat12d::Zero();
  G <<           G11, Mat3d::Zero(),           G13, Mat3d::Zero(), 
                 G21, Mat3d::Zero(), Mat3d::Zero(), Mat3d::Zero(), 
       Mat3d::Zero(),           G32, Mat3d::Zero(), Mat3d::Zero(), 
       Mat3d::Zero(), Mat3d::Zero(), Mat3d::Zero(), G44;

  Gk = G;
}

void Navigation::measurementUpdate(const drState predicted_state, VecXd residual, MatXd Hk, MatXd Rk) {
  
  if (!init_alignment_) {

    const bool use_clone = has_clone_ && (Hk.cols() == 18);

    MatXd P_full;
    if (use_clone) 
    {
      P_full = MatXd::Zero(18, 18);
      P_full.block<12, 12>(0, 0) = Pk;
      P_full.block<12, 6>(0, 12) = Pxc;
      P_full.block<6, 12>(12, 0) = Pxc.transpose();
      P_full.block<6, 6>(12, 12) = Pcc;
    } 
    else 
    {
      P_full = Pk;
    }

    MatXd S = Hk * P_full * Hk.transpose() + Rk;
    MatXd K = P_full * Hk.transpose() * S.inverse();

    VecXd error_update = K * residual;

    VecXd temp_state_vec = VecXd::Zero(13);

    temp_state_vec.segment(0, 3) = predicted_state.position + error_update.segment(0, 3);    
    temp_state_vec.segment(3, 4) = quatProd(euler2quat(error_update.segment(3, 3)), predicted_state.quaternion);
    temp_state_vec.segment(7, 3) = predicted_state.gyro_bias + error_update.segment(6, 3);
    temp_state_vec.segment(10, 3) = predicted_state.scale + error_update.segment(9, 3);

    setState(temp_state_vec.segment(0, 3), temp_state_vec.segment(3, 4), temp_state_vec.segment(7, 3), temp_state_vec.segment(10, 3));

    // -- Covariance Update : Joseph form
    int n = P_full.rows();
    P_full = (MatXd::Identity(n,n) - K * Hk) * P_full * (MatXd::Identity(n,n) - K * Hk).transpose() + K * Rk * K.transpose();
         
    Mat12d GG = Mat12d::Zero();
    GG << Mat3d::Identity(),                                                Mat3d::Zero(),     Mat3d::Zero(),     Mat3d::Zero(),
              Mat3d::Zero(), Mat3d::Identity() + skew33(0.5 * error_update.segment(3, 3)),     Mat3d::Zero(),     Mat3d::Zero(), 
              Mat3d::Zero(),                                                Mat3d::Zero(), Mat3d::Identity(),     Mat3d::Zero(), 
              Mat3d::Zero(),                                                Mat3d::Zero(),     Mat3d::Zero(), Mat3d::Identity();

    if (use_clone) 
    {
      P_full.block<12, 12>(0, 0) = GG * P_full.block<12, 12>(0, 0) * GG.transpose();
      P_full.block<12, 6>(0, 12) = GG * P_full.block<12, 6>(0, 12);
      P_full.block<6, 12>(12, 0) = P_full.block<12,6>(0, 12).transpose();

      Pk = P_full.block<12, 12>(0, 0);
      Pxc = P_full.block<12, 6>(0, 12);
      Pcc = P_full.block<6, 6>(12, 12);
    } 
    else
    {
      MatXd Pxc_old = Pxc;
      Pk = GG * P_full * GG.transpose();

      if (has_clone_) {        
        Pxc = GG * ( Pxc_old - K * Hk * Pxc_old );
        Pcc = Pcc - Pxc_old.transpose() * Hk.transpose() * S.inverse() * Hk * Pxc_old;
      }
    }
  }
}

void Navigation::timer_callback() {

  count_++;
  radar_valid++;

  if (!init_alignment_ && radar_valid >= 50) {
    RCLCPP_INFO_ONCE(this->get_logger(), "[WARN] Radar sensor is aborted.. Position is replaced by uwb position info !");
    has_problems_ = true;
  }

  if (count_ % 50 == 0 && !init_alignment_) // Print state every 4 timer callbacks (2 seconds) after initial alignment is complete                        
  {
    if (view_state_) 
    {
      std::cout << std::fixed << std::setprecision(4)
                << "Current State - Position (m): [" << getState().position.transpose() 
                << "], Attitude (rad): [" << quat2euler(getState().quaternion).transpose()
                << "], Gyro Bias (rad/s): [" << getState().gyro_bias.transpose()
                << "], Scale: [" << getState().scale.transpose() << std::endl;
      std::cout << "], Position Variance: [" << Pk.block(0, 0, 3, 3).diagonal().transpose()
                << "], Attitude Variance: [" << Pk.block(3, 3, 3, 3).diagonal().transpose() << "]" << std::endl;
    }
  } 
  else if (count_ % 50 == 0 && imu_cnt == 0 && init_alignment_) // Print alignment status every 10 timer callbacks (5 seconds) when waiting for IMU data
  {
    RCLCPP_INFO(this->get_logger(), "[INFO] WAITING FOR IMU DATA ...");
  } 
  else if (count_ % 50 == 0 && imu_cnt > 0 && init_alignment_) // Print stop status every 10 timer callbacks (5 seconds) when waiting for alignment process 
  {
    RCLCPP_INFO(this->get_logger(), "[WARN] WAITING FOR ALIGNMENT PROCESS ...");
  }
}

void Navigation::px4_timer_callback() {

  if (!init_alignment_ && !has_problems_) 
  {
    px4_pose.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    px4_pose.timestamp_sample = px4_pose.timestamp;
    px4_pose.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;

    Vec3d px4_cur_pos = getState().position;
    Vec4d px4_cur_att = getState().quaternion;

    px4_pose.position = {(float)px4_cur_pos(0), (float)px4_cur_pos(1),
                         (float)px4_cur_pos(2)};
    px4_pose.q = {(float)px4_cur_att(0), (float)px4_cur_att(1),
                  (float)px4_cur_att(2), (float)px4_cur_att(3)};

    px4_pose.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
    px4_pose.velocity.fill(std::numeric_limits<float>::quiet_NaN());

    px4_pose.angular_velocity = {(float)omega(0), (float)omega(1),
                                 (float)omega(2)};

    px4_pose.position_variance = {(float)px4_pos_cov_(0), (float)px4_pos_cov_(1),
                                  (float)px4_pos_cov_(2)};
    px4_pose.velocity_variance = {0.01, 0.01, 0.01};
    px4_pose.orientation_variance = {(float)px4_att_cov_(0), (float)px4_att_cov_(1),
                                     (float)px4_att_cov_(2)};

    px4_state_publisher_->publish(px4_pose);
  }
}

void Navigation::setState(Vec3d position, Vec4d quaternion, Vec3d gyro_bias, Vec3d scale) {
  drone_state_.position = position; 
  drone_state_.quaternion = quaternion;  
  drone_state_.gyro_bias = gyro_bias;
  drone_state_.scale = scale;
}

void Navigation::publishDronePath(Vec3d position, Vec4d quaternion) {

  if (!has_problems_) {
    pose.header.frame_id = "map";
    pose.header.stamp = this->get_clock()->now();

    pose.pose.position.x = position(0);
    pose.pose.position.y = position(1);
    pose.pose.position.z = position(2);

    pose.pose.orientation.x = quaternion(1);
    pose.pose.orientation.y = quaternion(2);
    pose.pose.orientation.z = quaternion(3);
    pose.pose.orientation.w = quaternion(0);

    if (view_path_) {
      localPath.header.frame_id = "map";
      localPath.header.stamp = this->get_clock()->now();
      localPath.poses.push_back(pose);
      path_publisher_->publish(localPath);
    }

    state_publisher_->publish(pose);
  }
}

void Navigation::initializeCloneCovariance() {
  MatXd Pxc_new = MatXd::Zero(12, 6);
  Pxc_new.leftCols<3>() = Pk.block<12, 3>(0, 0);
  Pxc_new.rightCols<3>() = Pk.block<12, 3>(0, 3);

  Mat6d Pcc_new;
  Pcc_new.block<3,3>(0, 0) = Pk.block<3, 3>(0, 0);
  Pcc_new.block<3,3>(0, 3) = Pk.block<3, 3>(0, 3);
  Pcc_new.block<3,3>(3, 0) = Pk.block<3, 3>(3, 0);
  Pcc_new.block<3,3>(3, 3) = Pk.block<3, 3>(3, 3);
  
  Pxc = Pxc_new;
  Pcc = Pcc_new;
  has_clone_ = true;
}

drState Navigation::getState() { return drone_state_; }

Mat12d Navigation::getCovariance() { return Pk; }

Vec2d Navigation::ahrs(Vec3d acc_accum) {
  
  double phi = std::atan2(acc_accum(1), acc_accum(2)); // Roll
  double theta = std::atan2(-acc_accum(0), std::sqrt(acc_accum(1) * acc_accum(1) + acc_accum(2) * acc_accum(2))); // Pitch
  Vec2d ahrs_result = Vec2d{phi, theta};
  
  return ahrs_result;
}

// ----------------------- IMU Time Management -----------------------

void Navigation::setImuCurrentTime(double t) { imu_current_time_ = t; }

void Navigation::setImuPreviousTime(double t) { imu_previous_time_ = t; }

void Navigation::setImuTimeDelta() { imu_time_delta_ = imu_current_time_ - imu_previous_time_; }

double Navigation::getImuCurrentTime() { return imu_current_time_; }

double Navigation::getImuPreviousTime() { return imu_previous_time_; }

double Navigation::getImuTimeDelta() { return imu_time_delta_; }

// ----------------------- RADAR Time Management -----------------------

void Navigation::setRadarCurrentTime(double t) { radar_current_time_ = t; }

void Navigation::setRadarPreviousTime(double t) { radar_previous_time_ = t; }

void Navigation::setRadarTimeDelta() { radar_time_delta_ = radar_current_time_ - radar_previous_time_; }

double Navigation::getRadarCurrentTime() { return radar_current_time_; }

double Navigation::getRadarPreviousTime() { return radar_previous_time_; }

double Navigation::getRadarTimeDelta() { return radar_time_delta_; }

void Navigation::publish_radar_pointcloud(const MatXd &points) {
  sensor_msgs::msg::PointCloud2 radar_msgs;
  radar_msgs.header.frame_id = "map";
  radar_msgs.header.stamp = this->get_clock()->now();

  // Fill in the PointCloud2 message fields based on the Mat3d points
  // This is a simplified example and may need adjustments based on your
  // specific point cloud structure
  radar_msgs.height = 1;
  radar_msgs.width = points.cols();
  radar_msgs.point_step = 12; // Assuming each point has x, y, z (3 floats)
  radar_msgs.row_step = radar_msgs.point_step * radar_msgs.width;

  // std::cout << radar_msgs.point_step * radar_msgs.width << std::endl;

  radar_msgs.is_bigendian = false;
  radar_msgs.is_dense = true;

  radar_msgs.fields.resize(3);
  radar_msgs.data.resize(radar_msgs.row_step * radar_msgs.height);

  radar_msgs.fields[0].name = "x";
  radar_msgs.fields[0].offset = 0;
  radar_msgs.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  radar_msgs.fields[0].count = 1;

  radar_msgs.fields[1].name = "y";
  radar_msgs.fields[1].offset = 4;
  radar_msgs.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  radar_msgs.fields[1].count = 1;

  radar_msgs.fields[2].name = "z";
  radar_msgs.fields[2].offset = 8;
  radar_msgs.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  radar_msgs.fields[2].count = 1;

  for (int i = 0; i < points.cols(); ++i) {
    float my_x = points(0, i);
    float my_y = points(1, i);
    float my_z = points(2, i);

    uint8_t *ptr = &radar_msgs.data[i * radar_msgs.point_step];

    std::memcpy(ptr + 0, &my_x, sizeof(float));
    std::memcpy(ptr + 4, &my_y, sizeof(float));
    std::memcpy(ptr + 8, &my_z, sizeof(float));
  }
  // global_radar_publisher_->publish(radar_msgs);
}

void Navigation::printStateInfo() {
  std::cout << "------------------------------------------------------------------------" << std::endl;
  std::cout << std::endl;
  std::cout << "[INFO] CHECK DRONE STATE BY SUBSCRIBING '/fmu/in/Vehicle_Visual_Odometry' TOPIC !" << std::endl;
  std::cout << std::endl;
  std::cout << "-------------------- NAVIGATION INITIAL STATE INFO ---------------------" << std::endl;
  std::cout << std::endl;
  std::cout << "Position: [" << getState().position.transpose() << "]" << std::endl;
  std::cout << "Attitude (deg): ["  << quat2euler(getState().quaternion).transpose() * r2d << "]" << std::endl; 
  std::cout << "Gyro Bias: [" << getState().gyro_bias.transpose() << "]" << std::endl;
  std::cout << "Scale Factor : [" << getState().scale.transpose() << "]" << std::endl;
  std::cout << std::endl;
  std::cout << "------------------------------------------------------------------------" << std::endl;
}

void Navigation::param_setting() {

  // DECLARE PARAMETERS
  this->declare_parameter("imu_topic", "/vectornav/imu");
  this->declare_parameter("radar_topic", "/mmwave/radarScan");
  this->declare_parameter("sonar_topic", "/sonar/range");
  this->declare_parameter("align_time", 10.0);

  this->declare_parameter("imu_rate", 200);
  this->declare_parameter("radar_rate", 20);
  this->declare_parameter("init_pos", std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter("init_att", std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter("init_gyro_bias", std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter("init_acc_bias", std::vector<double>{0.0, 0.0, 0.0});  

  this->declare_parameter("do_align", true);

  this->declare_parameter("init_pos_cov", std::vector<double>{0.5, 0.5, 0.5});
  this->declare_parameter("init_att_cov", std::vector<double>{1.0 * d2r, 1.0 * d2r, 1.0 * d2r});
  this->declare_parameter("init_gyro_bias_cov", std::vector<double>{0.001 * d2r, 0.001 * d2r, 0.001 * d2r});
  this->declare_parameter("init_scale_cov", std::vector<double>{0.0001, 0.0001, 0.0001});
  
  this->declare_parameter("gyro_noise", std::vector<double>{0.01 * d2r, 0.01 * d2r, 0.01 * d2r});
  this->declare_parameter("gyro_bias_noise", std::vector<double>{0.0001 * d2r, 0.0001 * d2r, 0.0001 * d2r});
  this->declare_parameter("radar_vel_noise", std::vector<double>{0.005, 0.005, 0.005});
  this->declare_parameter("radar_scale_noise", std::vector<double>{0.0001, 0.0001, 0.0001});
  
  this->declare_parameter("px4_pos_cov", std::vector<double>{0.01, 0.01, 0.01});
  this->declare_parameter("px4_att_cov", std::vector<double>{0.01, 0.01, 0.01});

  this->declare_parameter("icp_cov", std::vector<double>{1.5, 1.5, 1e3, 1e3, 1e3, 5});                          
  
  // GET PARAMETERS

  this->get_parameter("imu_topic", imu_topic_);
  this->get_parameter("radar_topic", radar_topic_);
  this->get_parameter("sonar_topic", sonar_topic_);

  this->get_parameter("imu_rate", imu_rate);
  this->get_parameter("radar_rate", radar_rate);
  this->get_parameter("sonar_topic", sonar_topic_);
  this->get_parameter("align_time", align_time_);

  this->get_parameter("do_align", do_align_);

  std::vector<double> p_pos_vec = this->get_parameter("init_pos_cov").as_double_array();
  std::vector<double> p_att_vec = this->get_parameter("init_att_cov").as_double_array();
  std::vector<double> p_gyro_bias_vec = this->get_parameter("init_gyro_bias_cov").as_double_array();
  std::vector<double> p_scale_vec = this->get_parameter("init_scale_cov").as_double_array();

  Pk.diagonal() << p_pos_vec[0], p_pos_vec[1], p_pos_vec[2], p_att_vec[0] * d2r,
      p_att_vec[1] * d2r, p_att_vec[2] * d2r, p_gyro_bias_vec[0] * d2r,
      p_gyro_bias_vec[1] * d2r, p_gyro_bias_vec[2] * d2r, p_scale_vec[0],
      p_scale_vec[1], p_scale_vec[2];

  Pk = Pk.cwiseProduct(Pk);

  std::vector<double> w_gyro_vec = this->get_parameter("gyro_noise").as_double_array();
  std::vector<double> w_gyro_bias_vec = this->get_parameter("gyro_bias_noise").as_double_array();
  std::vector<double> w_radar_vel_vec = this->get_parameter("radar_vel_noise").as_double_array();
  std::vector<double> w_radar_scale_vec = this->get_parameter("radar_scale_noise").as_double_array();

  process_noise << w_gyro_vec[0] * d2r, w_gyro_vec[1] * d2r,
      w_gyro_vec[2] * d2r, w_gyro_bias_vec[0] * d2r, w_gyro_bias_vec[1] * d2r,
      w_gyro_bias_vec[2] * d2r, w_radar_vel_vec[0], w_radar_vel_vec[1],
      w_radar_vel_vec[2], w_radar_scale_vec[0], w_radar_scale_vec[1],
      w_radar_scale_vec[2];

  process_noise = process_noise.cwiseProduct(process_noise);

  Qk = process_noise.asDiagonal();

  std::vector<double> init_pos_vec = this->get_parameter("init_pos").as_double_array();

  init_pos_ = Vec3d{init_pos_vec[0], init_pos_vec[1], init_pos_vec[2]};

  std::vector<double> init_att_vec = this->get_parameter("init_att").as_double_array();

  init_att_ = euler2quat(Vec3d{init_att_vec[0] * d2r, init_att_vec[1] * d2r, init_att_vec[2] * d2r});

  std::vector<double> init_gyro_bias_vec = this->get_parameter("init_gyro_bias").as_double_array();

  init_gyro_bias_ = Vec3d{init_gyro_bias_vec[0], init_gyro_bias_vec[1], init_gyro_bias_vec[2]};

  std::vector<double> init_acc_bias_vec = this->get_parameter("init_acc_bias").as_double_array();

  init_acc_bias_ = Vec3d{init_acc_bias_vec[0], init_acc_bias_vec[1], init_acc_bias_vec[2]};

  std::vector<double> px4_pos_vec = this->get_parameter("px4_pos_cov").as_double_array();
  std::vector<double> px4_att_vec = this->get_parameter("px4_att_cov").as_double_array();

  px4_pos_cov_ = Vec3d{px4_pos_vec[0], px4_pos_vec[1], px4_pos_vec[2]};
  px4_att_cov_ = Vec3d{px4_att_vec[0], px4_att_vec[1], px4_att_vec[2]};

  std::vector<double> icp_cov_vec = this->get_parameter("icp_cov").as_double_array();

  icp_cov_ = Vec6d{icp_cov_vec[0], icp_cov_vec[1], icp_cov_vec[2],
                   icp_cov_vec[3] * d2r, icp_cov_vec[4] * d2r, icp_cov_vec[5] * d2r};

  // ---------------------------------------------------

  this->declare_parameter("uwb_cov", std::vector<double>{5.5, 5.5, 5.5});

  std::vector<double> uwb_cov_vec = this->get_parameter("uwb_cov").as_double_array();
  Vec3d R_uwb_vec = Vec3d{uwb_cov_vec[0], uwb_cov_vec[1], uwb_cov_vec[2]};
  
  R_uwb = (R_uwb_vec.cwiseProduct(R_uwb_vec)).asDiagonal();

  this->declare_parameter("uwb_range_cov", 5.5);
  
  double uwb_range_cov = this->get_parameter("uwb_range_cov").as_double();
  R_uwb_range = uwb_range_cov * uwb_range_cov;

  // ---------------------------------------------------

  this->declare_parameter("sonar_cov", 0.1);

  double sonar_cov = this->get_parameter("sonar_cov").as_double();
  R_sonar = Mat1d::Identity() * sonar_cov * sonar_cov;

  // ---------------------------------------------------

  this->declare_parameter("body_t_imu", std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter("body_R_imu", std::vector<double>{0.0, 0.0, 0.0});

  this->declare_parameter("imu_t_radar", std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter("imu_R_radar", std::vector<double>{0.0, 0.0, 0.0});

  this->declare_parameter("imu_t_sonar", std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter("imu_t_uwb", std::vector<double>{0.0, 0.0, 0.0});

  std::vector<double> body_t_imu_ = this->get_parameter("body_t_imu").as_double_array();
  std::vector<double> body_R_imu_ = this->get_parameter("body_R_imu").as_double_array();

  std::vector<double> imu_t_radar_ = this->get_parameter("imu_t_radar").as_double_array();
  std::vector<double> imu_R_radar_ = this->get_parameter("imu_R_radar").as_double_array();

  std::vector<double> imu_t_sonar_ = this->get_parameter("imu_t_sonar").as_double_array();
  std::vector<double> imu_t_uwb_ = this->get_parameter("imu_t_uwb").as_double_array();

  Cgb = euler2dcm(Vec3d{init_att_vec[0] * d2r, init_att_vec[1] * d2r,
                        init_att_vec[2] * d2r});
  tgb = Vec3d{init_pos_vec[0], init_pos_vec[1], init_pos_vec[2]};

  Cbi = euler2dcm(Vec3d(
      {body_R_imu_[0] * d2r, body_R_imu_[1] * d2r, body_R_imu_[2] * d2r}));
  tbi = Vec3d({body_t_imu_[0], body_t_imu_[1], body_t_imu_[2]});

  Cir = euler2dcm(Vec3d(
      {imu_R_radar_[0] * d2r, imu_R_radar_[1] * d2r, imu_R_radar_[2] * d2r}));
  tir = Vec3d({imu_t_radar_[0], imu_t_radar_[1], imu_t_radar_[2]});

  tis = Vec3d({imu_t_sonar_[0], imu_t_sonar_[1], imu_t_sonar_[2]});
  tiu = Vec3d({imu_t_uwb_[0], imu_t_uwb_[1], imu_t_uwb_[2]});

  Cbr = Cbi * Cir;
  tbr = tbi + Cbi * tir;

  tbu = tbi + Cbi * tiu;

  this->declare_parameter("view_path", false);
  this->get_parameter("view_path", view_path_);
  
  this->declare_parameter("view_state", false);
  this->get_parameter("view_state", view_state_);

  this->declare_parameter("pub_egovel", false);
  this->get_parameter("pub_egovel", pub_egovel_);

  this->declare_parameter("pub_icp_", false);
  this->get_parameter("pub_icp_", pub_icp_);

  this->declare_parameter("px4_fc_rate", 50.0);
  this->get_parameter("px4_fc_rate", px4_fc_rate_);
}

} // namespace navigation
