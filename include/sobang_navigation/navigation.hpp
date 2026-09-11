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
#include "px4_msgs/msg/sensor_combined.hpp"

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

  /** @name Extrinsic Parameters
   * 센서간 좌표계 변환을 위한 회전행렬과 변환벡터
   */
  ///@{
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
  ///@}
  
  /** @name  IMU and State Variables
   * 가속도 및 각속도 변수와 상태변수 정의
  */
  ///@{
  Vec3d acc = Vec3d::Zero();
  Vec3d omega = Vec3d::Zero();
  
  Vec3d acc_accum{0.0, 0.0, 0.0};
  Vec3d acc_stack_ = Vec3d::Zero();
  
  Vec3d gyro_accum{0.0, 0.0, 0.0};
  
  drState drone_state_;
  
  Vec3d init_pos_ = Vec3d::Zero();
  Vec4d init_att_ = Vec4d::Zero();
  Vec3d init_gyro_bias_ = Vec3d::Zero();
  Vec3d init_acc_bias_ = Vec3d::Zero();

  double tau_bg = 10000;
  double tau_sr = 10000;

  icpState icp_prev_state;
  icpState icp_current_state;
  ///@}

  /** @name  Filter Matrices (System, Noise ...)
   * 상태 추정에 사용되는 공분산 행렬 정의 
   */
  ///@{
  Mat12d Fk = Mat12d::Zero();
  Mat12d Gk = Mat12d::Zero();
  Mat12d Pk = Mat12d::Identity();
  Mat12d Qk = Mat12d::Zero();

  Mat6d Pcc = Mat6d::Zero();
  MatXd Pxc = MatXd::Zero(12, 6);
  
  Vec12d process_noise = Vec12d::Zero();
  
  Vec3d px4_pos_cov_ = Vec3d::Zero();
  Vec3d px4_att_cov_ = Vec3d::Zero();
  
  /** @name Sensor Noise Covariance
   * 측정치에 대한 공분산 행렬 정의.
   */
  ///@{
  Vec6d icp_cov_ = Vec6d::Zero(); // Measurement noise covariance for ICP

  Mat1d R_sonar = Mat1d::Identity() * 0.1; // Measurement noise covariance for Sonar
  Mat3d R_uwb = Mat3d::Identity() * 5.5; // Measurement noise covariance for UWB
  double R_uwb_range = 5.5; // Measurement noise covariance for UWB range
  ///@}

  /** @name Sensor related parameters
   * 센서의 출력 주기나 dt들을 계산하기 위한 변수들
   */
  ///@{
  double imu_rate = 200;
  double radar_rate = 20;
  double px4_fc_rate_ = 10.0;

  double imu_previous_time_ = 0.0; 
  double imu_current_time_ = 0.0;
  double imu_time_delta_ = 0.0;

  double radar_previous_time_ = 0.0; 
  double radar_current_time_ = 0.0;
  double radar_time_delta_ = 0.0;
  
  double align_time_ = 10.0; // Time duration for initial alignment using IMU data
  ///@}

  /** @name nav_params
   * nav_params.yaml 파일에서 로드되는 파라미터들을 저장하기 위한 변수
   */
  ///@{
  std::string imu_topic_ = "/fmu/out/sensor_combined"; // IMU 센서 토픽 이름
  std::string radar_topic_ = "/mmwave/radarScan"; // 레이더 센서 토픽 이름
  std::string sonar_topic_ = "/fmu/out/distance_sensor"; // Sonar 센서 토픽 이름

  bool init_alignment_ = true; // 초기정렬 종료 여부  
  bool do_align_ = true; // 초기정렬 수행 여부

  bool view_state_ = false; // 터미널에 상태정보 출력 여부
  bool view_path_ = false; // Rviz2에 경로정보 출력 여부

  bool has_problems_ = false; // 레이더 센서의 문제 여부

  bool has_clone_ = false; // Cloning State 의 공분산 초기화 여부

  bool pub_egovel_ = false; // Ego Velocity 퍼블리시 여부

  bool pub_icp_ = false; // ICP 기반의 자세 및 위치 정보 퍼블
  ///@}

  // Timer for publishing
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr px4_timer_;

  /** @name Others..
   * 카운트 해야하는 변수들 및 기타 변수들 정의 (Description 참고)
   */
  ///@{
  int count_; // 타이머 콜백의 호출 횟수를 카운트하기 위한 변수 (터미널에 출력용)

  int imu_cnt = 0; // IMU 콜백 호출 횟수를 카운트하기 위한 변수 (초기정렬 수행용)  

  uint32_t alignment_count_ = 0; // IMU 초기 정렬을 위한 측정치 카운트용 변수

  uint16_t radar_valid = 0; // 레이더 측정 콜백을 카운트해서 끊김 여부 파악을 위해 사용

  double prev_dist = 0.0; // Sonar 데이터의 필터링을 위해 업데이트에 사용했던 가장 최근의 거리 정보 저장용

  uint16_t sonar_cnt = 0; // Sonar 데이터의 필터링을 위해 Sonar 콜백 횟수를 누적
  
  int32_t icp_cnt = 0; // 최초 ICP 가 수행되는 과정에서 처음엔 누적 데이터가 없으니 기준 시점을 만들어주는 변수

  double icp_att_sum = 0.0; // ICP 기반의 자세 정보를 업데이트하기위해 사용
  
  Vec3d icp_pos_sum = Vec3d::Zero(3, 1); // ICP 기반의 변위 정보를 업데이트하기위해 사용
  ///@}

  /** @name ROS Messages
   * ROS 메시지 정의
   */
  ///@{
  nav_msgs::msg::Path localPath; // Rviz2에 퍼블리시할 경로 메시지 정의
  
  geometry_msgs::msg::PoseStamped pose; // ROS 퍼블리시용 PoseStamped 메시지 정의
  
  px4_msgs::msg::VehicleOdometry px4_pose{}; // PX4 EKF2에 퍼블리시할 VehicleOdometry 메시지 정의
  ///@}

  private:
  /** @name Publisher Statement
   * ROS 퍼블리셔 정의
   */
  ///@{
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr state_publisher_; // 패키지 자체의 State Publisher [Optional]

  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_state_publisher_; // PX4 EKF2 측정치로 전달하기 위한 Publisher

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr ego_vel_publisher_; // Ego Velocity 분석을 위한 퍼블리셔 [Optional]

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr icp_state_publisher_; // ICP 기반의 자세 및 위치 정보를 퍼블리셔 [Optional]

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_; // 전체 궤적에 대한 메시지를 전달하기 위한 퍼블리셔 [Optional]
  ///@}

  /** @name Subscriber Statement
   * ROS 서브스크라이버 정의
   */
  ///@{
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr radar_subscriber_; // 레이더 포인트 클라우드 데이터를 수신하기 위한 서브스크라이버

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_; // IMU 데이터를 수신하기 위한 서브스크라이버 [Optional]

  rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr px4_imu_subscriber_; // PX4 IMU 데이터를 수신하기 위한 서브스크라이버

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr uwb_position_subscriber_; // UWB 위치 데이터를 수신하기 위한 서브스크라이버

  rclcpp::Subscription<sobang_navigation::msg::UwbData>::SharedPtr uwb_range_subscriber_; // UWB 거리 데이터를 수신하기 위한 서브스크라이버

  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr ros2_sonar_subscriber_; // ROS Sonar 패키지 데이터를 수신하기 위한 서브스크라이버 [Optional]

  rclcpp::Subscription<px4_msgs::msg::DistanceSensor>::SharedPtr px4_sonar_subscriber_; // PX4에 연결된 Sonar 데이터를 수신하기 위한 서브스크라이버
  ///@}

  /** @name Functions
   * 함수들 정의
   */
  ///@{
  void param_setting(); // 초기 params 파일에서 파라미터 설정 및 로드하는 함수

  /// @brief 센서 관련 콜백
  void radar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg); // 레이더 센서 콜백

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg); // IMU 센서 콜백

  void px4_imu_callback(const px4_msgs::msg::SensorCombined::SharedPtr msg); // PX4 IMU 센서 콜백

  void uwbPositionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg); // UWB 위치 데이터 콜백

  void uwbRangeCallback(const sobang_navigation::msg::UwbData::SharedPtr msg); // UWB 거리 데이터 콜백

  void ros2_sonarCallback(const sensor_msgs::msg::Range::SharedPtr msg); // ROS Sonar 패키지 데이터 콜백

  void px4_sonarCallback(const px4_msgs::msg::DistanceSensor::SharedPtr msg); // PX4 Sonar 데이터 콜백

  /// @brief 항법을 위한 유틸 함수들
  void setImuCurrentTime(double t); // 현재 측정된 IMU 시간 설정
  double getImuCurrentTime();       // 현재 측정된 IMU 시간 가져오기

  void setImuPreviousTime(double t); // 이전 IMU 시간 설정
  double getImuPreviousTime();       // 이전 IMU 시간 가져오기

  void setImuTimeDelta();   // IMU 시간 차이 설정
  double getImuTimeDelta(); // IMU 시간 차이 가져오기

  void setRadarCurrentTime(double t); // 현재 측정된 레이더 시간 설정
  double getRadarCurrentTime();       // 현재 측정된 레이더 시간 가져오기

  void setRadarPreviousTime(double t); // 이전 레이더 시간 설정
  double getRadarPreviousTime();       // 이전 레이더 시간 가져오기

  void setRadarTimeDelta();   // 레이더 시간 차이 설정
  double getRadarTimeDelta(); // 레이더 시간 차이 가져오기

  Vec2d ahrs(Vec3d acc_accum); // AHRS 계산을 위한 함수 (Roll, Pitch 계산) [TBD]

  void initAlignment(const sensor_msgs::msg::Imu::SharedPtr msg); // IMU 데이터를 이용한 초기 정렬 수행

  void px4_initAlignment(const px4_msgs::msg::SensorCombined::SharedPtr msg); // PX4 IMU 데이터를 이용한 초기 정렬 수행

  void DeadReckoning(const drState prev_state, Vec3d ego_velocity, Vec3d angular_rate, double dt); // 레이더 기반 추측항법 수행

  void timeUpdate(const drState prev_state, Vec3d ego_velocity, Vec3d angular_rate, double dt); // 상태 예측 업데이트 수행

  void measurementUpdate(const drState predicted_state, VecXd residual, MatXd Hk, MatXd Rk); // 상태 추정 업데이트 수행

  void setState(Vec3d position, Vec4d quaternion, Vec3d gyro_bias, Vec3d scale); // 상태 변수 설정

  drState getState(); // 상태 변수 가져오기

  void initializeCloneCovariance(); // Cloning State 의 공분산 초기화

  Mat12d getCovariance(); // 공분산 행렬 가져오기

  /// @brief 데이터 통신을 위한 퍼블리시 함수
  void publish_radar_pointcloud(const MatXd &points); // 레이더 포인트 클라우드 데이터를 퍼블리시하는 함수

  void publishDronePath(Vec3d position, Vec4d quaternion); // 드론의 위치 및 자세 정보를 퍼블리시하는 함수

  /// @brief 터미널에 정보를 출력하는 함수
  void printStateInfo(); // 상태 정보를 터미널에 출력하는 함수

  /// @brief 타이머 함수
  void timer_callback(); // 타이머 콜백 함수 (터미널에 출력용)

  void px4_timer_callback(); // 타이머 콜백 함수 (PX4 EKF2에 상태 정보 전달용)
  ///@}
};

} // namespace navigation
#endif // NAVIGATION__NAVIGATION_HPP_
