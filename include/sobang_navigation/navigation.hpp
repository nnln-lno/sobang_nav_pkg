#ifndef NAVIGATION__NAVIGATION_HPP_
#define NAVIGATION__NAVIGATION_HPP_

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/range.hpp"

#include "px4_msgs/msg/distance_sensor.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"

#include "sobang_navigation/msg/local_state.hpp"
#include "sobang_navigation/msg/uwb_data.hpp"

#include "sobang_navigation/navTools.hpp"
#include "sobang_navigation/radarEstimator.hpp"
#include "sobang_navigation/uwbLocalizer.hpp"

namespace navigation {

class Navigation : public rclcpp::Node {
public:
  Navigation();

  RadarEstimator radar_estimator_;

  drState drone_state_;

  Vec3d acc_accum{0.0, 0.0, 0.0};
  Vec3d gyro_accum{0.0, 0.0, 0.0};

  Mat3d Cgb = Mat3d::Identity(); // Rotation from body frame to ref frame
  Vec3d tgb = Vec3d::Zero();     // Translation from body frame to ref frame

  Mat3d Cbi = Mat3d::Identity(); // Rotation from imu frame to body frame
  Vec3d tbi = Vec3d::Zero();     // Translation from imu frame to body frame

  Mat3d Cir = Mat3d::Identity(); // Rotation from radar frame to body frame
  Vec3d tir = Vec3d::Zero();     // Translation from radar frame to

  Vec3d tis = Vec3d::Zero(); // Translation from sonar frame to body frame

  Vec3d tiu = Vec3d::Zero(); // Translation from uwb tag to imu frame

  Mat3d Cbr = Mat3d::Identity();
  Vec3d tbr, tbu = Vec3d::Zero();

  Vec3d omega = Vec3d::Zero();
  Vec3d acc_stack_ = Vec3d::Zero();

  Vec3d init_pos_ = Vec3d::Zero();
  Vec4d init_att_ = Vec4d::Zero();
  Vec3d init_gyro_bias_ = Vec3d::Zero();
  Vec3d init_acc_bias_ = Vec3d::Zero();

  Vec3d px4_pos_cov_ = Vec3d::Zero();
  Vec3d px4_att_cov_ = Vec3d::Zero();

  Vec6d icp_cov_ = Vec6d::Zero();

  Vec12d process_noise = Vec12d::Zero();

  Mat12d Fk = Mat12d::Zero();
  Mat12d Gk = Mat12d::Zero();
  Mat12d Pk = Mat12d::Identity();
  Mat12d Qk = Mat12d::Zero();

  Mat6d  Pcc = Mat6d::Zero();
  MatXd  Pxc = MatXd::Zero(12, 6);

  Mat3d R_uwb = Mat3d::Identity() * 5.5; // Measurement noise covariance for UWB
  double R_uwb_range = 5.5; // Measurement noise covariance for UWB range
  Mat1d R_sonar =
      Mat1d::Identity() * 0.1; // Measurement noise covariance for Sonar

  uint16_t imu_rate = 200; // [HYPERPARAM] IMU data rate in Hz. Change this
  uint16_t radar_rate = 20;
  double px4_fc_rate_ = 10.0;

  double imu_previous_time_ =
      0.0; // For calculating time delta in state estimation
  double imu_current_time_ = 0.0;
  double imu_time_delta_ = 0.0;

  double radar_previous_time_ = 0.0; 
  double radar_current_time_ = 0.0;
  double radar_time_delta_ = 0.0;

  double tau_bg = 10000;
  double tau_sr = 10000;

  double align_time_ =
      10.0; // Time duration for initial alignment using IMU data

  uint16_t radar_valid = 0;  

  double prev_dist = 0.0;

  uint16_t sonar_cnt = 0;
  int32_t icp_cnt = 0;

  double icp_att_sum = 0.0;
  Vec3d icp_pos_sum = Vec3d::Zero(3, 1);

private:
  // Publisher - Publish Local State
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      state_publisher_;

  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr
      px4_state_publisher_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
      ego_vel_publisher_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      icp_state_publisher_;

  // Subscriber - Subscribe Radar Pointcloud Infornmation or other [TBD]
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      radar_subscriber_;

  // Subscriber - Subcribe IMU [TBD]
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;

  // Subscriber - Subscribe UWB based Localization Position
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
      uwb_position_subscriber_;

  // Subscriber - Subscribe UWB Distance Information
  rclcpp::Subscription<sobang_navigation::msg::UwbData>::SharedPtr
      uwb_range_subscriber_;

  // Subscriber - Subscribe Sonar Information
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr
      ros2_sonar_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::DistanceSensor>::SharedPtr
      px4_sonar_subscriber_;

  // Publisher - Publish path to Rviz2 [TBD]
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

  void param_setting();

  // Each sensors Subscriber callback
  void radar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

  void
  uwbPositionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

  void uwbRangeCallback(const sobang_navigation::msg::UwbData::SharedPtr msg);

  void ros2_sonarCallback(const sensor_msgs::msg::Range::SharedPtr msg);
  void px4_sonarCallback(const px4_msgs::msg::DistanceSensor::SharedPtr msg);

  // Time delta calculation for state estimation
  // IMU
  void setImuCurrentTime(double t);
  double getImuCurrentTime();

  void setImuPreviousTime(double t);
  double getImuPreviousTime();

  void setImuTimeDelta();
  double getImuTimeDelta();

  // RADAR
  void setRadarCurrentTime(double t);
  double getRadarCurrentTime();

  void setRadarPreviousTime(double t);
  double getRadarPreviousTime();

  void setRadarTimeDelta();
  double getRadarTimeDelta();

  void publish_radar_pointcloud(const MatXd &points);

  void publishDronePath(Vec3d position, Vec4d quaternion);

  void printStateInfo();

  // othet uitls functions
  void initAlignment(const sensor_msgs::msg::Imu::SharedPtr msg);

  void DeadReckoning(const drState prev_state, Vec3d ego_velocity,
                     Vec3d angular_rate, double dt);

  // Filter Function
  void timeUpdate(const drState prev_state, Vec3d ego_velocity,
                  Vec3d angular_rate, double dt);

  void measurementUpdate(const drState predicted_state, VecXd residual,
                         MatXd Hk, MatXd Rk);

  // void setState(Vec3d position, Vec3d velocity, Vec4d quaternion, Vec3d
  // accel_bias, Vec3d gyro_bias);
  void setState(Vec3d position, Vec4d quaternion, Vec3d gyro_bias, Vec3d scale);

  drState getState();

  void initializeCloneCovariance();

  Mat12d getCovariance();

  Vec2d ahrs(Vec3d acc_accum);

  uint32_t alignment_count_ = 0; // Counter for initial alignment using IMU data

  bool init_alignment_ = true; // Flag for initial alignment using IMU data
  bool radar_update = false;
  bool uwb_update = false;
  bool stop_check = false;
  bool do_align_ = true; // Flag to determine whether to perform initial
                         // alignment using IMU data
  bool ned_ = false;
  bool view_state_ = false;
  bool view_path_ = false;
  bool has_problems_ = false;
  bool has_clone_ = false;

  nav_msgs::msg::Path localPath;
  geometry_msgs::msg::PoseStamped pose;
  px4_msgs::msg::VehicleOdometry px4_pose{};

  std::string imu_topic_ = "/vectornav/imu";
  std::string radar_topic_ = "/mmwave/radarScan";
  std::string sonar_topic_ = "/sonar/range";

  icpState icp_prev_state;
  icpState icp_current_state;

  int imu_cnt = 0;
  int ahrs_cnt = 0;

  int prev_cnt = 0;
  int cur_cnt = 0;

  // Timer for publishing
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr px4_timer_;

  void timer_callback();

  void px4_timer_callback();

  int count_;
};

} // namespace navigation

#endif // NAVIGATION__NAVIGATION_HPP_
