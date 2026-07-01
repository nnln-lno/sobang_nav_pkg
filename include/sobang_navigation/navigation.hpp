#ifndef NAVIGATION__NAVIGATION_HPP_
#define NAVIGATION__NAVIGATION_HPP_

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/range.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "sobang_navigation/msg/local_state.hpp"
#include "sobang_navigation/msg/uwb_data.hpp"

#include "sobang_navigation/radarEstimator.hpp"
#include "sobang_navigation/uwbLocalizer.hpp"
#include "sobang_navigation/navTools.hpp"

namespace navigation
{

class Navigation : public rclcpp::Node
{
public:
  Navigation();

  RadarEstimator radar_estimator_;

  drState drone_state_;
 
  Vec3d acc_accum{0.0, 0.0, 0.0};
  Vec3d gyro_accum{0.0, 0.0, 0.0};

  Mat3d Cbr = Mat3d::Identity(); // Rotation from radar frame to body frame
  Vec3d tbr = Vec3d::Zero(); // Translation from radar frame to
  Vec3d tis = Vec3d::Zero(); // Translation from sonar frame to body frame

  Vec3d init_pos_ = Vec3d::Zero();
  Vec4d init_att_ = Vec4d::Zero();
  Vec3d init_gyro_bias_ = Vec3d::Zero();

  Vec12d process_noise = Vec12d::Zero();  

  Mat12d Fk = Mat12d::Zero(); 
  Mat12d Gk = Mat12d::Zero(); 
  Mat12d Pk = Mat12d::Identity();
  Mat12d Qk = Mat12d::Zero();

  Mat3d R_uwb = Mat3d::Identity() * 5.5; // Measurement noise covariance for UWB
  double R_uwb_range = 5.5; // Measurement noise covariance for UWB range
  Mat1d R_sonar = Mat1d::Identity() * 0.1; // Measurement noise covariance for Sonar

  uint16_t imu_rate = 200; // [HYPERPARAM] IMU data rate in Hz. Change this according to your IMU's actual data rate.
  uint16_t radar_rate = 20;

  double imu_previous_time_ = 0.0; // For calculating time delta in state estimation
  double imu_current_time_ = 0.0;
  double imu_time_delta_ = 0.0;

  double radar_previous_time_ = 0.0; // For calculating time delta in state estimation
  double radar_current_time_ = 0.0;
  double radar_time_delta_ = 0.0;

  double tau_bg = 10000;
  double tau_sr = 10000;

  double align_time_ = 10.0; // Time duration for initial alignment using IMU data

  uint icp_cnt = 0;

private:
  // Publisher - Publish Local State
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr state_publisher_;

  // Subscriber - Subscribe UWB Range Infornmation or other [TBD]
  // rclcpp::Subscription<uwb_driver::msg::UwbRange>::SharedPtr uwb_subscriber_;

  // Subscriber - Subscribe Radar Pointcloud Infornmation or other [TBD]
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr radar_subscriber_;

  // Subscriber - Subcribe IMU [TBD]
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;

  // Subscriber - Subscribe UWB based Localization Position
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr uwb_position_subscriber_;

  // Subscriber - Subscribe UWB Distance Information
  rclcpp::Subscription<sobang_navigation::msg::UwbData>::SharedPtr uwb_range_subscriber_;

  // Subscriber - Subscribe Sonar Information
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sonar_subscriber_;

  // Publisher - Publish path to Rviz2 [TBD]
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

  // Publisher - Publish pointcloud w.r.t world frame to Rviz2 [TBD]
  // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_radar_publisher_;

  void param_setting();
  
  // Each sensors Subscriber callback
  void radar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

  void uwbPositionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

  void uwbRangeCallback(const sobang_navigation::msg::UwbData::SharedPtr msg);

  void sonarCallback(const sensor_msgs::msg::Range::SharedPtr msg);

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

  // othet uitls functions
  void initAlignment(const sensor_msgs::msg::Imu::SharedPtr msg);

  void DeadReckoning(const drState prev_state, Vec3d ego_velocity, Vec3d angular_rate, double dt);

  // Filter Function

  void timeUpdate(const drState prev_state, Vec3d ego_velocity, Vec3d angular_rate, double dt);

  void measurementUpdate(const drState predicted_state, VecXd residual, MatXd Hk, MatXd Rk);

  // void setState(Vec3d position, Vec3d velocity, Vec4d quaternion, Vec3d accel_bias, Vec3d gyro_bias);
  void setState(Vec3d position, Vec4d quaternion, Vec3d gyro_bias, Vec3d scale);
  
  drState getState();

  uint32_t alignment_count_ = 0; // Counter for initial alignment using IMU data

  bool init_alignment_ = true; // Flag for initial alignment using IMU data  
  bool radar_update = false;
  bool uwb_update = false;
  bool stop_check = false;
  bool do_align_ = true; // Flag to determine whether to perform initial alignment using IMU data
  bool ned_ = false;
  bool view_state_ = false;
  bool view_path_ = false;

  nav_msgs::msg::Path localPath;
  geometry_msgs::msg::PoseStamped pose;

  std::string imu_topic_ = "/vectornav/imu";
  std::string radar_topic_ = "/mmwave/radarScan";
  std::string sonar_topic_ = "/sonar/range";

  icpState prev_state;
  icpState current_state;

  int imu_cnt = 0;

  int prev_cnt = 0;
  int cur_cnt = 0;  

  void publishDronePath(Vec3d position, Vec4d quaternion);

  void printStateInfo();

  // Timer for publishing
  rclcpp::TimerBase::SharedPtr timer_;
  void timer_callback();
  int count_;
};

}  // namespace navigation

#endif  // NAVIGATION__NAVIGATION_HPP_ 