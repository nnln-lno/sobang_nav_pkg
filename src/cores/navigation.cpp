#include "sobang_navigation/navigation.hpp"
#include <functional>
#include <random>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace navigation
{
  Navigation::Navigation() : Node("sobang_navigation_node"), count_(0)
  {
    param_setting();

    this->get_parameter("imu_topic", imu_topic_);
    this->get_parameter("radar_topic", radar_topic_);
    this->get_parameter("sonar_topic", sonar_topic_);

    auto sensor_qos = rclcpp::SensorDataQoS();    

    // 항법해를 출력하기 위한 Publisher 생성
    state_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/nav/localState", 10);

    // 레이더 센서 데이터 취득을 위한 Subscriber 생성
    radar_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      radar_topic_, sensor_qos, std::bind(&Navigation::radar_callback, this, _1));

    // IMU 센서 데이터 취득을 위한 Subscriber 생성
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, sensor_qos, std::bind(&Navigation::imu_callback, this, _1));
    
    // UWB 거리 정보로 부터 계산되는 위치 정보를 취득하기 위한 Subscriber 생성
    uwb_position_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/uwb/position", 100, std::bind(&Navigation::uwbPositionCallback, this, _1));
      
    uwb_range_subscriber_ = this->create_subscription<sobang_navigation::msg::UwbData>(
      "/uwb/range_array", 100, std::bind(&Navigation::uwbRangeCallback, this, _1));

    sonar_subscriber_ = this->create_subscription<sensor_msgs::msg::Range>(
      sonar_topic_, 10, std::bind(&Navigation::sonarCallback, this, _1));
    
    if (view_path_)
    {
      // No need to make path in experiment, because of it accumulate all navigation solution data.
      path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/nav/localPath", 10);
    }

    // global_radar_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/nav/globalRadarPCL", 10);    

    setState(init_pos_, init_att_, init_gyro_bias_, Vec3d{1.0, 1.0, 1.0}); // for DR alignment

    timer_ = this->create_wall_timer(
      500ms, std::bind(&Navigation::timer_callback, this));
  }

  // Measurement Callback

  void Navigation::radar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    radar_estimator_.radarParser(msg); // Here, parsing point cloud data.

    if (icp_cnt == 0)
    {
      radar_estimator_.setPreviousPoints(radar_estimator_.getPointMatrix());
      prev_cnt = imu_cnt;
      icp_cnt++;
    }
    else if (icp_cnt == 1)
    {
      radar_estimator_.setCurrentPoints(radar_estimator_.getPointMatrix());            
    }

     // Update current state with the latest IMU data for ICP processing
    current_state.position = getState().position;
    current_state.quaternion = getState().quaternion;

    setRadarCurrentTime(msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);    

    if (!init_alignment_) // Skip processing until initial alignment is complete
    {
      radar_estimator_.egoVelocityEstimator();                  
      stop_check = false;
      if (radar_estimator_.getEgoVelocity().norm() < 0.01) // [HYPERPARAM] threshold for zero velocity after initial alignment
      {
          radar_estimator_.setEgoVelocity(Vec3d{0.0, 0.0, 0.0});          
          stop_check = true;
      }                
    }else{
      radar_estimator_.setEgoVelocity(Vec3d{0.0, 0.0, 0.0}); // During initial alignment, we assume the drone is stationary.
      stop_check = true;
      return;
    }

    Mat3d rel_R = quat2dcm(prev_state.quaternion).transpose() * quat2dcm(current_state.quaternion);
    Vec3d rel_t = quat2dcm(prev_state.quaternion).transpose() * (current_state.position - prev_state.position);    

    if ( ( (std::abs(dcm2euler(rel_R)(2)) > (1.0 * d2r)) || ( std::abs(rel_t(0)) > 0.3) ) && (icp_cnt > 1) )
    {  
      if (radar_estimator_.simpleRadar2DIcp(rel_R, rel_t))
      {
        icpState icp_result = radar_estimator_.getIcpPose();

        Vec3d z_t = icp_result.position - rel_t;
        Mat3d z_R =  rel_R.transpose() * quat2dcm(icp_result.quaternion);

        double th_ = acos( std::clamp( (z_R.trace() - 1.0) / 2.0, -1.0, 1.0) );
        Vec3d z_att = (th_)/(2.0 * sin(th_)) * Vec3d{z_R(2,1) - z_R(1,2), z_R(0,2) - z_R(2,0), z_R(1,0) - z_R(0,1)};

        Mat3d A = quat2dcm(prev_state.quaternion);
        Mat3d B = quat2dcm(current_state.quaternion);
        Mat3d ATB = A.transpose() * B;

        double ATB_th_ = acos( std::clamp( (ATB.trace() - 1.0) / 2.0, -1.0, 1.0 ) );
        Vec3d ATB_att = (ATB_th_)/(2.0 * sin(ATB_th_)) * Vec3d{ATB(2,1) - ATB(1,2), ATB(0,2) - ATB(2,0), ATB(1,0) - ATB(0,1)};
        Mat3d Jr = Mat3d::Identity() + (0.5 * skew33(ATB_att));
        
        MatXd Full_Hk = MatXd::Zero(6, 12);
        
        Full_Hk <<     A.transpose(),     -A.transpose(), MatXd::Zero(3, 6),
                   Mat3d::Zero(3, 3), Jr * B.transpose(), MatXd::Zero(3, 6);

        Vec6d R_icp_vec = Vec6d{2.0, 2.0, 2.0, 1e3* d2r, 1e3 * d2r, 3.0 * d2r};
        Mat6d R_icp = (R_icp_vec.cwiseProduct(R_icp_vec)).asDiagonal();
        measurementUpdate(getState(), Vec6d{z_t(0), z_t(1), z_t(2), z_att(0), z_att(1), z_att(2)}, Full_Hk, R_icp); // Only consider x, y translation and yaw rotation for 2D ICP
      }
      prev_state = current_state;

      radar_estimator_.setPreviousPoints(radar_estimator_.getCurrentPoints());
    }

    // Radar and IMU has same coordinate system, so rotation is eye(3) and translation is zero.
    MatXd global_points = quat2dcm(getState().quaternion) * radar_estimator_.getPointMatrix() + getState().position.replicate(1, radar_estimator_.getPointMatrix().cols());
    
    // publish_radar_pointcloud(global_points);

    setRadarTimeDelta();              

    setRadarPreviousTime(getRadarCurrentTime());              
  }

  void Navigation::imu_callback(const sensor_msgs::msg::Imu::SharedPtr i_msg)
  { 
    sensor_msgs::msg::Imu::SharedPtr msg = std::make_shared<sensor_msgs::msg::Imu>(*i_msg);

    if (imu_topic_ == "/imu_apps")
    {
      msg->header.stamp.sec = i_msg->header.stamp.sec;
      msg->header.stamp.nanosec = i_msg->header.stamp.nanosec;
      msg->linear_acceleration.x = -i_msg->linear_acceleration.x;
      msg->linear_acceleration.y = -i_msg->linear_acceleration.y;
      msg->linear_acceleration.z = i_msg->linear_acceleration.z;
      msg->angular_velocity.x = -i_msg->angular_velocity.x;
      msg->angular_velocity.y = -i_msg->angular_velocity.y;
      msg->angular_velocity.z = i_msg->angular_velocity.z;
    }

    setImuCurrentTime(msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);    

    imu_cnt++;

    // init bias aligment IMU data.
    if (init_alignment_) {
      if (do_align_)
      {
        RCLCPP_INFO_ONCE(this->get_logger(), "DONT MOVE THE DRONE UNTIL INITIAL ALIGNMENT IS COMPLETE! (%.1f seconds)", align_time_);
      }
      else{
        RCLCPP_INFO_ONCE(this->get_logger(), "Initial Alignment Skipped! Be careful with the drone's movement at the beginning.");
      }
      initAlignment(msg);
      return; // Skip processing until initial alignment is complete
    }    

    if (getImuCurrentTime() <= getImuPreviousTime())
    {
      RCLCPP_WARN(this->get_logger(), "Received IMU data with non-increasing timestamp. Skipping this measurement.");
      return; // Skip processing if time is not moving forward
    }

    setImuTimeDelta();    

    Vec3d w_b = Vec3d{msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z} - getState().gyro_bias;

    DeadReckoning(getState(), radar_estimator_.getEgoVelocity(), w_b, getImuTimeDelta());

    timeUpdate(getState(), radar_estimator_.getEgoVelocity(), w_b, getImuTimeDelta());

    Pk = (Fk * Pk * Fk.transpose()) + (Gk * Qk * Gk.transpose());      

    // ------------------------ Visualization Part ------------------------
    publishDronePath(getState().position, getState().quaternion);

    setImuPreviousTime(getImuCurrentTime());
  }

  void Navigation::uwbPositionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {    
    Vec3d uwb_position(msg->point.x, msg->point.y, msg->point.z); // Vec3d residual = uwb_position - getState().position;
    
    Vec3d residual = uwb_position - getState().position; // Assuming UWB measures height with some bias

    MatXd Hk = MatXd::Zero(3, 12);
    Hk.block<3, 3>(0, 0) = Mat3d::Identity();

    if (!init_alignment_ && !stop_check)
    {
      // RCLCPP_INFO(this->get_logger(), "Received UWB Position Measurement: [%.2f, %.2f, %.2f]", msg->point.x, msg->point.y, msg->point.z);
      measurementUpdate(getState(), residual, Hk, R_uwb);
    }
  }

  void Navigation::uwbRangeCallback(const sobang_navigation::msg::UwbData::SharedPtr msg)
  {
    std::vector<float> dist = msg->ranges;
    std::vector<int16_t> anc_id = msg->anchor_ids;

    std::vector<geometry_msgs::msg::Pose> anc_poses = msg->pose.poses;

    int count = dist.size();

    VecXd residual = VecXd::Zero(count);
    MatXd Hk = MatXd::Zero(count, 12);
    for (int i=0; i<count; i++)
    {
      Vec3d anc_pos(anc_poses[i].position.x, anc_poses[i].position.y, anc_poses[i].position.z);
      residual(i) = dist[i] - (anc_pos - getState().position).norm();

      Hk.row(i).block<1, 3>(0, 0) = -(anc_pos - getState().position).transpose() / (anc_pos - getState().position).norm();
    } 

    measurementUpdate(getState(), residual, Hk, R_uwb_range*MatXd::Identity(count, count)); // [HYPERPARAM] UWB range measurement noise covariance
  }

  void Navigation::sonarCallback(const sensor_msgs::msg::Range::SharedPtr msg)
  {
    double sonar_range = msg->range;

    double residual = (sonar_range - tis.z()) - (getState().position.z()); // Assuming sonar measures height

    // std::cout << "Sonar Range: " << sonar_range << ", Estimated Height: " << getState().position.z() << ", Residual: " << residual << std::endl;

    MatXd Hk = MatXd::Zero(1, 12);
    Hk(0, 2) = 1.0; // Derivative of measurement w.r.t z position

    if (!init_alignment_ && !stop_check)
    {
      // RCLCPP_INFO(this->get_logger(), "Received Sonar Range Measurement: %.2f m", sonar_range);
      measurementUpdate(getState(), Vec1d{ residual }, Hk, R_sonar); // [HYPERPARAM] Sonar measurement noise covariance
    }
  }

  // Basic Navigation Functions

  void Navigation::initAlignment(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // Accumulate IMU data for initial alignment
    if (msg->linear_acceleration.z < 0)
    {
      acc_accum += Vec3d({msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z}) + Vec3d({0.0, 0.0, 9.80665});    
    }
    else{
      acc_accum += Vec3d({msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z}) - Vec3d({0.0, 0.0, 9.80665});
    }    
    gyro_accum += Vec3d{msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};

    alignment_count_++;

    if (!do_align_)
    {
      // RCLCPP_INFO_ONCE(this->get_logger(), "Initial alignment skipped. Starting navigation with init parameters.");
      
      setState(init_pos_, init_att_, init_gyro_bias_, Vec3d{1.0, 1.0, 1.0}); // For DR Alignment

      publishDronePath(getState().position, getState().quaternion);

      init_alignment_ = false; // Skip alignment process and start navigation immediately

      prev_state.position = getState().position;
      prev_state.quaternion = getState().quaternion;          
      
      printStateInfo(); // Print initial state information after skipping alignment
    }
    else{
      if (alignment_count_ >= imu_rate * align_time_) // [HYPERPARAM] Align for the specified time duration
      {
        Vec3d acc_mean = acc_accum / alignment_count_;
        Vec3d gyro_mean = gyro_accum / alignment_count_;

        // double phi = std::atan2(acc_mean(1), acc_mean(2)); // Roll
        // double theta = std::atan2(-acc_mean(0), std::sqrt(acc_mean(1) * acc_mean(1) + acc_mean(2) * acc_mean(2))); // Pitch
        double psi = 0.0; 

        setState(Vec3d{0.0, 0.0, 0.0}, euler2quat(Vec3d{0, 0, psi}), gyro_mean, Vec3d{1.0, 1.0, 1.0});
        // setState(getState().position, getState().quaternion, getState().gyro_bias, Vec3d{1.0, 1.0, 1.0}); // For DR Alignment

        publishDronePath(getState().position, getState().quaternion);

        init_alignment_ = false; // Alignment complete    

        prev_state.position = getState().position;
        prev_state.quaternion = getState().quaternion;      

        // std::cout << "Alignment Complete! : " << std::endl;
        // std::cout << " Accel Bias: [" << acc_mean.x() << ", " << acc_mean.y() << ", " << acc_mean.z() << "]" << std::endl;
        // std::cout << " Gyro Bias: [" << gyro_mean.x() << ", " << gyro_mean.y() << ", " << gyro_mean.z() << "]" << std::endl;    

        printStateInfo(); // Print initial state information after alignment  
      }
    }

    setImuPreviousTime(getImuCurrentTime());
  }

  void Navigation::DeadReckoning(const drState prev_state, Vec3d ego_velocity, Vec3d angular_rate, double dt)
  {
    Mat3d Cnb = quat2dcm(prev_state.quaternion);

    Mat3d w_skew = skew33(angular_rate);

    Vec3d new_position       = prev_state.position + (Cnb*Cbr*ego_velocity - Cnb*w_skew*tbr)*dt;
    
    Vec4d new_quaternion     = quatUpdate(prev_state.quaternion, angular_rate, dt);
    
    // Vec3d new_velocity       = Cbr*ego_velocity;

    setState(new_position, new_quaternion, prev_state.gyro_bias, prev_state.scale);
  }

  void Navigation::timeUpdate(const drState prev_state, Vec3d ego_velocity, Vec3d angular_rate, double dt)
  {
    Mat3d Cnb = quat2dcm(prev_state.quaternion);

    Vec3d inv_scale = prev_state.scale.array().inverse();
    Vec3d hat_vr = ego_velocity.cwiseProduct(inv_scale);        

    Mat3d w_skew = skew33(angular_rate);
    Mat3d sk_br  = skew33(tbr);

    // First Row (3x11)
    Vec3d F12_vec = -(Cnb*Cbr*hat_vr - Cnb * w_skew * tbr);
    Mat3d F12     = skew33(F12_vec);
    Mat3d F13 = Cnb * sk_br;
    Mat3d F14 = -Cnb * Cbr * hat_vr.asDiagonal();
    
    // Second Row
    Mat3d F23 = -Cnb;
    
    // Third Row
    Mat3d F33 = (-1/tau_bg)*Mat3d::Identity();
    Mat3d F44 = (-1/tau_sr)*Mat3d::Identity();

    Mat12d F = Mat12d::Zero();
    F << Mat3d::Zero(),            F12,           F13,           F14,
         Mat3d::Zero(),  Mat3d::Zero(),           F23, Mat3d::Zero(),
         Mat3d::Zero(),  Mat3d::Zero(),           F33, Mat3d::Zero(),
         Mat3d::Zero(),  Mat3d::Zero(), Mat3d::Zero(),           F44;

    Fk = Mat12d::Identity() + (F * dt) + (0.5 * F * F * dt * dt); // Discretized state transition matrix using second-order Taylor expansion

    Mat3d G11 = Cnb * sk_br;
    Mat3d G13 = -Cnb * Cbr;
    
    Mat3d G21 = -Cnb;

    Mat3d G32 = Mat3d::Identity();

    Mat3d G44 = Mat3d::Identity();

    Mat12d G = Mat12d::Zero();
    G <<                        G11,  Mat3d::Zero(),           G13, Mat3d::Zero(),
                                G21,  Mat3d::Zero(), Mat3d::Zero(), Mat3d::Zero(),
                      Mat3d::Zero(),            G32, Mat3d::Zero(), Mat3d::Zero(),
                      Mat3d::Zero(),  Mat3d::Zero(), Mat3d::Zero(),           G44;

    Gk = G;
    
  }

  void Navigation::measurementUpdate(const drState predicted_state, VecXd residual, MatXd Hk, MatXd Rk)
  {
    if(!init_alignment_)
    {
      MatXd K = Pk * Hk.transpose() * (Hk * Pk * Hk.transpose() + Rk).inverse();

      VecXd error_update = K * residual;

      VecXd temp_state_vec = VecXd::Zero(13);

      temp_state_vec.segment(0, 3) = predicted_state.position + error_update.segment(0, 3);
      // getState().velocity += error_update.segment<3>(3);
      temp_state_vec.segment(3, 4) = quatProd(euler2quat(error_update.segment(3, 3)), predicted_state.quaternion);
      // getState().accel_bias += error_update.segment<3>(9);
      temp_state_vec.segment(7, 3) = predicted_state.gyro_bias + error_update.segment(6, 3);
      temp_state_vec.segment(10, 3) = predicted_state.scale + error_update.segment(9, 3);

      setState(temp_state_vec.segment(0, 3), temp_state_vec.segment(3, 4), temp_state_vec.segment(7, 3), temp_state_vec.segment(10, 3));

      publishDronePath(getState().position, getState().quaternion);

      Pk = (Mat12d::Identity() - K * Hk) * Pk * (Mat12d::Identity() - K * Hk).transpose() + K * Rk * K.transpose();

      Mat12d GG = Mat12d::Zero();
      GG << Mat3d::Identity(), Mat3d::Zero(), Mat3d::Zero(), Mat3d::Zero(),
          Mat3d::Zero(), Mat3d::Identity() + skew33(0.5 * error_update.segment(3, 3)), Mat3d::Zero(), Mat3d::Zero(),
          Mat3d::Zero(), Mat3d::Zero(), Mat3d::Identity(), Mat3d::Zero(),
          Mat3d::Zero(), Mat3d::Zero(), Mat3d::Zero(), Mat3d::Identity();

      Pk = GG * Pk * GG.transpose();
    }
  }

  // Timer callback for publishing drone path
  void Navigation::timer_callback()
  {
    count_++;
    // RCLCPP_INFO(this->get_logger(), "Publishing Vehicle Odometry: count %d", msg.count);
    
    if (count_ % 4 == 0 && !init_alignment_) // Print state every 4 timer callbacks (2 seconds) after initial alignment
    {
      if (view_state_)
      { 
        std::cout << std::fixed << std::setprecision(4) << "Current State - Position (m): [" << getState().position.transpose() << "], Attitude (rad): [" << quat2euler(getState().quaternion).transpose() << "], Position Variance: [" << Pk.block(0, 0, 3, 3).diagonal().transpose() << "], Attitude Variance: [" << Pk.block(3, 3, 3, 3).diagonal().transpose() << "]" << std::endl;
      }
    }    
    else if (count_ % 10 == 0 && imu_cnt == 0 && init_alignment_) // Print alignment status every 10 timer callbacks (5 seconds) during initial alignment
    {
      RCLCPP_INFO(this->get_logger(), "Waiting for IMU data ..."); 
    }
    else if (count_ % 10 == 0 && imu_cnt > 0 && init_alignment_) // Print stop status every 10 timer callbacks (5 seconds) when drone is stationary
    {
      RCLCPP_INFO(this->get_logger(), "Waiting for Alignment process ..."); 
    }
  }

  // void Navigation::setState(Vec3d position, Vec3d velocity, Vec4d quaternion, Vec3d, Vec3d gyro_bias, double scale)
  void Navigation::setState(Vec3d position, Vec4d quaternion, Vec3d gyro_bias, Vec3d scale)
  {
    drone_state_.position = position;
    // drone_state_.velocity = velocity;
    drone_state_.quaternion = quaternion;
    // drone_state_.accel_bias = accel_bias;
    drone_state_.gyro_bias = gyro_bias;
    drone_state_.scale = scale;
  }

  void Navigation::publishDronePath(Vec3d position, Vec4d quaternion)
  {
    pose.header.frame_id = "map";
    pose.header.stamp = this->get_clock()->now();

    if(ned_)
    {
      pose.pose.position.x = position(0);
      pose.pose.position.y = -position(1);
      pose.pose.position.z = -position(2);    

      Vec3d temp_eul = quat2euler(quaternion);
      Vec4d temp_quat = euler2quat(Vec3d{temp_eul.x(), -temp_eul.y(), -temp_eul.z()}); // Convert to NED by negating pitch and yaw

      pose.pose.orientation.x = temp_quat(1);
      pose.pose.orientation.y = temp_quat(2);
      pose.pose.orientation.z = temp_quat(3);
      pose.pose.orientation.w = temp_quat(0);
    }
    else{

    pose.pose.position.x = position(0);
    pose.pose.position.y = position(1);
    pose.pose.position.z = position(2);    

    pose.pose.orientation.x = quaternion(1);
    pose.pose.orientation.y = quaternion(2); 
    pose.pose.orientation.z = quaternion(3);
    pose.pose.orientation.w = quaternion(0);
    }

    if (view_path_)
    {
      localPath.header.frame_id = "map";
      localPath.header.stamp = this->get_clock()->now();
      localPath.poses.push_back(pose);
      path_publisher_->publish(localPath);
    }
    state_publisher_->publish(pose);
  }  

  drState Navigation::getState() { return drone_state_;}

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

  void Navigation::publish_radar_pointcloud(const MatXd &points)
  {
    sensor_msgs::msg::PointCloud2 radar_msgs;
    radar_msgs.header.frame_id = "map";
    radar_msgs.header.stamp = this->get_clock()->now();

    // Fill in the PointCloud2 message fields based on the Mat3d points
    // This is a simplified example and may need adjustments based on your specific point cloud structure
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
        float my_x = points(0,i);
        float my_y = points(1,i);
        float my_z = points(2,i);        
        
        // std::cout << "Publishing Point " << i << ": [" << my_x << ", " << my_y << ", " << my_z << "]" << std::endl;

        uint8_t *ptr = &radar_msgs.data[i * radar_msgs.point_step];      

        std::memcpy(ptr + 0, &my_x, sizeof(float));
        std::memcpy(ptr + 4, &my_y, sizeof(float));
        std::memcpy(ptr + 8, &my_z, sizeof(float));
    }

    // global_radar_publisher_->publish(radar_msgs);
  }
  
  void Navigation::printStateInfo()
  {
    drState printState = getState();

    std::cout << "-------------------- Navigation Node is Running ... --------------------" << std::endl;
    std::cout << std::endl;
    std::cout << "[INFO] CHECK DRONE STATE BY SUBSCRIBING '/nav/localState' TOPIC !" << std::endl; 
    std::cout << std::endl;
    std::cout << "------------------------------------------------------------------------" << std::endl;
  }

  void Navigation::param_setting()
  {
    this->declare_parameter("imu_rate", 200);
    this->declare_parameter("radar_rate", 20);
    this->declare_parameter("init_pos", std::vector<double>{0.0, 0.0, 0.0});
    this->declare_parameter("init_att", std::vector<double>{0.0, 0.0, 0.0});
    this->declare_parameter("init_gyro_bias", std::vector<double>{0.0, 0.0, 0.0});
    
    this->get_parameter("imu_rate", imu_rate);
    this->get_parameter("radar_rate", radar_rate);
    this->get_parameter("align_time", align_time_);    

    this->declare_parameter("do_align", true);
    this->declare_parameter("ned", false);

    this->get_parameter("do_align", do_align_);
    this->get_parameter("ned", ned_);    

    // Covariance and Noise Parameters

    this->declare_parameter("init_pos_cov", std::vector<double>{0.5, 0.5, 0.5});
    this->declare_parameter("init_att_cov", std::vector<double>{1.0 * d2r, 1.0 * d2r, 1.0 * d2r});
    this->declare_parameter("init_gyro_bias_cov", std::vector<double>{0.001 * d2r, 0.001 * d2r, 0.001 * d2r});
    this->declare_parameter("init_scale_cov", std::vector<double>{0.0001, 0.0001, 0.0001});

    this->declare_parameter("gyro_noise", std::vector<double>{0.01 * d2r, 0.01 * d2r, 0.01 * d2r});
    this->declare_parameter("gyro_bias_noise", std::vector<double>{0.0001 * d2r, 0.0001 * d2r, 0.0001 * d2r});
    this->declare_parameter("radar_vel_noise", std::vector<double>{0.005, 0.005, 0.005});
    this->declare_parameter("radar_scale_noise", std::vector<double>{0.0001, 0.0001, 0.0001});

    std::vector<double> p_pos_vec = this->get_parameter("init_pos_cov").as_double_array();
    std::vector<double> p_att_vec = this->get_parameter("init_att_cov").as_double_array();
    std::vector<double> p_gyro_bias_vec = this->get_parameter("init_gyro_bias_cov").as_double_array();
    std::vector<double> p_scale_vec = this->get_parameter("init_scale_cov").as_double_array();

    std::vector<double> w_gyro_vec = this->get_parameter("gyro_noise").as_double_array();
    std::vector<double> w_gyro_bias_vec = this->get_parameter("gyro_bias_noise").as_double_array();
    std::vector<double> w_radar_vel_vec = this->get_parameter("radar_vel_noise").as_double_array();
    std::vector<double> w_radar_scale_vec = this->get_parameter("radar_scale_noise").as_double_array();

    std::vector<double> init_pos_vec = this->get_parameter("init_pos").as_double_array();
    std::vector<double> init_att_vec = this->get_parameter("init_att").as_double_array();
    std::vector<double> init_gyro_bias_vec = this->get_parameter("init_gyro_bias").as_double_array();

    init_pos_ = Vec3d{init_pos_vec[0], init_pos_vec[1], init_pos_vec[2]};
    init_att_ = euler2quat(Vec3d{init_att_vec[0] * d2r, init_att_vec[1] * d2r, init_att_vec[2] * d2r});
    init_gyro_bias_ = Vec3d{init_gyro_bias_vec[0], init_gyro_bias_vec[1], init_gyro_bias_vec[2]};

    // std::cout << "Initial Position: [" << init_pos_.transpose() << "], Initial Attitude (Euler angles in degrees): [" 
    //           << init_att_vec[0] << ", " << init_att_vec[1] << ", " << init_att_vec[2] << "]" << std::endl;

    Pk.diagonal() << p_pos_vec[0], p_pos_vec[1], p_pos_vec[2],
                     p_att_vec[0]*d2r, p_att_vec[1]*d2r, p_att_vec[2]*d2r,
                     p_gyro_bias_vec[0]*d2r, p_gyro_bias_vec[1]*d2r, p_gyro_bias_vec[2]*d2r,
                     p_scale_vec[0], p_scale_vec[1], p_scale_vec[2];                   

    Pk = Pk.cwiseProduct(Pk);
    // RCLCPP_INFO(this->get_logger(), "Initial State Covariance = Position [%.2e, %.2e, %.2e], Attitude [%.2e, %.2e, %.2e], Gyro Bias [%.2e, %.2e, %.2e], Scale [%.2e, %.2e, %.2e]", 
    //             Pk.diagonal()(0), Pk.diagonal()(1), Pk.diagonal()(2),
    //             Pk.diagonal()(3), Pk.diagonal()(4), Pk.diagonal()(5),
    //             Pk.diagonal()(6), Pk.diagonal()(7), Pk.diagonal()(8),
    //             Pk.diagonal()(9), Pk.diagonal()(10), Pk.diagonal()(11));

    process_noise << w_gyro_vec[0]*d2r, w_gyro_vec[1]*d2r, w_gyro_vec[2]*d2r,
                     w_gyro_bias_vec[0]*d2r, w_gyro_bias_vec[1]*d2r, w_gyro_bias_vec[2]*d2r,
                     w_radar_vel_vec[0], w_radar_vel_vec[1], w_radar_vel_vec[2],
                     w_radar_scale_vec[0], w_radar_scale_vec[1], w_radar_scale_vec[2];                     

    process_noise = process_noise.cwiseProduct(process_noise);
    // RCLCPP_INFO(this->get_logger(), "Process Noise = Gyro Noise [%.2e, %.2e, %.2e], Gyro Noise Bias [%.2e, %.2e, %.2e], Radar Velocity Noise [%.2e, %.2e, %.2e], Radar Velocity Scale [%.2e, %.2e, %.2e]", 
    //             process_noise(0), process_noise(1), process_noise(2),
    //             process_noise(3), process_noise(4), process_noise(5),
    //             process_noise(6), process_noise(7), process_noise(8),
    //             process_noise(9), process_noise(10), process_noise(11));

    Qk = process_noise.asDiagonal();

    this->declare_parameter("uwb_cov", std::vector<double>{5.5, 5.5, 5.5});

    std::vector<double> uwb_cov_vec = this->get_parameter("uwb_cov").as_double_array();    
    Vec3d R_uwb_vec = Vec3d{uwb_cov_vec[0], uwb_cov_vec[1], uwb_cov_vec[2]};
    R_uwb = (R_uwb_vec.cwiseProduct(R_uwb_vec)).asDiagonal();// This function can be used to set parameters dynamically if needed  }

    this->declare_parameter("uwb_range_cov", 5.5);
    double uwb_range_cov = this->get_parameter("uwb_range_cov").as_double();
    R_uwb_range = uwb_range_cov * uwb_range_cov;

    this->declare_parameter("sonar_cov", 0.1);
    double sonar_cov = this->get_parameter("sonar_cov").as_double();
    R_sonar = Mat1d::Identity() * sonar_cov * sonar_cov;

    this->declare_parameter("imu_t_radar", std::vector<double>{0.0, 0.0, 0.0});
    this->declare_parameter("imu_R_radar", std::vector<double>{0.0, 0.0, 0.0});
    this->declare_parameter("imu_t_sonar", std::vector<double>{0.0, 0.0, 0.0});

    std::vector<double> imu_t_radar_ = this->get_parameter("imu_t_radar").as_double_array();
    std::vector<double> imu_R_radar_ = this->get_parameter("imu_R_radar").as_double_array();
    std::vector<double> imu_t_sonar_ = this->get_parameter("imu_t_sonar").as_double_array();
        
    Cbr = euler2dcm(Vec3d({imu_R_radar_[0] * d2r, imu_R_radar_[1] * d2r, imu_R_radar_[2] * d2r  }));
    tbr = Vec3d({imu_t_radar_[0], imu_t_radar_[1], imu_t_radar_[2]});

    tis = Vec3d({imu_t_sonar_[0], imu_t_sonar_[1], imu_t_sonar_[2]});
    
    this->declare_parameter("view_path", false);
    this->get_parameter("view_path", view_path_);
  }


}  // namespace navigation