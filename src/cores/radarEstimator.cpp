#include "sobang_navigation/radarEstimator.hpp"

#include "algorithm"
#include "random"
#include "unsupported/Eigen/MatrixFunctions"

namespace navigation {
RadarEstimator::RadarEstimator() {
  std::cout << "Success initializing Radar Dead Reckoning Node." << std::endl;

  setEgoVelocity(Vec3d{0.0, 0.0, 0.0});
}

bool RadarEstimator::radarParser(
    const sensor_msgs::msg::PointCloud2::SharedPtr &radar_msg) {
  point_size_ = radar_msg->width * radar_msg->height;

  radar_points_ = MatXd(3, point_size_);
  radar_velocities_ = VecXd(point_size_);

  sensor_msgs::PointCloud2Iterator<float> iter_x(*radar_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*radar_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*radar_msg, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_vel(*radar_msg, "v_doppler");

  int point_cnt = 0;

  for (int i = 0; i < point_size_;
       i++, ++iter_x, ++iter_y, ++iter_z, ++iter_vel) {

    double rg = sqrt(*iter_x * *iter_x + *iter_y * *iter_y + *iter_z * *iter_z);
    if (!(rg < 0.5)) {
      radar_points_(0, point_cnt) = *iter_x;
      radar_points_(1, point_cnt) = *iter_y;
      radar_points_(2, point_cnt) = *iter_z;

      radar_velocities_(point_cnt) = *iter_vel;

      point_cnt++;
    }
    // std::cout << radar_velocities_(i);
    // Logic to parse radar pointcloud data and extract points and velocities
    // would go here
  }

  return true; // Dummy return value
}

bool RadarEstimator::egoVelocityEstimator() {
  // Change hyperparameter for velocity estmation using config file or dynamic
  // reconfigure [TBD]

  // This for checking whether the radar pointcloud data is parsed correctly.
  // Remove this after testing. std::cout << "Radar Points: " <<
  // radar_points_.cols() << std::endl;

  if (radar_velocities_.cwiseAbs().mean() <
      0.05) // [HYPERPARAM] threshold for zero velocity
  {
    // RCLCPP_WARN(this->get_logger(), "Zero Velocity Detected! (count : %d)",
    // ++zero_velocity_count_);
    setEgoVelocity(Vec3d{0.0, 0.0, 0.0});
    return false;
  }

  std::vector<int> valid_indices;

  for (int i = 0; i < point_size_; i++) {
    if (std::abs(radar_velocities_(i)) < 3.0 &&
        radar_points_.col(i).norm() > 0.5) // [HYPERPARAM] threshold for valid
                                           // radar points based on velocity
    {
      valid_indices.push_back(i);
    }
  }

  if (valid_indices.size() < 3) {
    std::cout << "Not enough valid radar points for velocity estimation! "
                 "(valid points : "
              << valid_indices.size() << ")" << std::endl;
    return false;
  }

  VecXd point_norm = radar_points_(Eigen::all, valid_indices).colwise().norm();

  MatXd H(valid_indices.size(), 3);
  VecXd y(valid_indices.size());

  for (size_t i = 0; i < valid_indices.size(); ++i) {
    H(i, 0) = radar_points_(0, valid_indices[i]) / point_norm(i);
    H(i, 1) = radar_points_(1, valid_indices[i]) / point_norm(i);
    H(i, 2) = radar_points_(2, valid_indices[i]) / point_norm(i);
    y(i) = radar_velocities_(valid_indices[i]);
  }

  std::random_device rd;
  std::mt19937 g(rd());

  uint8_t max_count_ = 0;

  std::vector<uint> inlier_indices_;

  for (int iter = 0; iter < 200;
       ++iter) // Example: Shuffle 100 times RANSAC iterations
  {
    // Shuffle and only take 3 front index data
    std::shuffle(valid_indices.begin(), valid_indices.end(), g);

    std::vector<int> test_indices_ = {valid_indices[0], valid_indices[1],
                                      valid_indices[2]};

    Mat3d H_test = H(test_indices_, Eigen::all);
    Mat3d HTH = H_test.transpose() * H_test;

    Vec3d y_test = y(test_indices_);

    VecXd S = HTH.jacobiSvd().singularValues();

    if ((S.maxCoeff() / S.minCoeff()) > 1e4) {
      continue; // Skip if the matrix is close to singular
    }

    Vec3d init_est_ = -HTH.inverse() * H_test.transpose() * y_test;

    VecXd residual_ = (y + H * init_est_).cwiseAbs();

    uint8_t inlier_count = 0;
    std::vector<uint> est_inlier_indices_;

    for (int i = 0; i < residual_.size(); i++) {
      if (residual_(i) <= 0.22) // [HYPERPARAM] threshold for inliers
      {
        inlier_count++;
        est_inlier_indices_.push_back(i);
      }
    }

    if (inlier_count > max_count_) {
      max_count_ = inlier_count;
      inlier_indices_ = est_inlier_indices_;
      est_inlier_indices_.clear();
    }
  }

  if (max_count_ < 3) {
    std::cout << "Not enough inliers for velocity estimation! (inliers : "
              << inlier_indices_.size() << ")" << std::endl;
    return false;
  }

  MatXd H_est = H(inlier_indices_, Eigen::all);
  VecXd y_est = y(inlier_indices_);

  ego_velocity_ =
      -(H_est.transpose() * H_est).inverse() * H_est.transpose() * y_est;

  // ego_velocity_ = Vec3d{ego_velocity_(0), 0.0, ego_velocity_(2)};

  return true; // Dummy return value
}

bool RadarEstimator::simpleRadar2DIcp(Mat3d R, Vec3d t) {

  int point_size = current_points_.cols();

  double init_yaw = dcm2euler(R)(2);
  double init_trs = Vec3d{t(0), t(1), 0.0}.norm();

  Vec3d final_est = Vec3d::Zero(3, 1);
  // Vec3d opt_est = Vec3d::Zero(3, 1);
  Vec3d opt_est = Vec3d{t(0), t(1), init_yaw};
  double optimal = 1e9;
  double lamb = 0.5;
  double cost_prev = 1e9;
  double delta = 0.3;

  for (uint i = 0; i < 100; i++) {
    MatXd save_prev = accum_points_;
    // MatXd save_prev = previous_points_;
    int match_cnt = 0;

    MatXd trf_points =
        euler2dcm(Vec3d{0.0, 0.0, opt_est(2)}) * current_points_ +
        Vec3d{opt_est(0), opt_est(1), 0.0}.replicate(1, current_points_.cols());

    MatXd corr_set = MatXd::Zero(2, point_size);

    MatXd trf_2d = trf_points.topRows(2);
    // MatXd prev_2d = accum_points_.topRows(2);
    MatXd prev_2d = save_prev.topRows(2);

    for (int j = 0; j < point_size; j++) {

      VecXd xy_err = (prev_2d - trf_2d.col(j).replicate(1, accum_points_.cols())).colwise().norm();
      
      // VecXd xy_err = (prev_2d - trf_2d.col(j).replicate(1, save_prev.cols())).colwise().norm();
      // double ecld_dist = xy_err.norm();

      Eigen::Index min_idx;
      double min_dist = xy_err.minCoeff(&min_idx);

      if (min_dist <= 2.0) {
        corr_set.col(match_cnt) = Vec2d(j, min_idx);
        match_cnt++;
      }
    }

    if (match_cnt < 4) {
      return false;
    }

    Mat3d H = Mat3d::Zero();
    Vec3d b = Vec3d::Zero();
    MatXd J = MatXd::Zero(2, 3);

    double cost = 0.0;

    for (int j = 0; j < match_cnt; j++) {
      int cur_id = corr_set(0, j);
      int prev_id = corr_set(1, j);

      Vec2d err_func = (prev_2d.col(prev_id) - trf_2d.col(cur_id));

      double r2 = err_func.squaredNorm();
      double weight;

      double r = std::sqrt(r2);
      if (r <= delta) {
        weight = 1.0;
      } else {
        weight = delta / r;
      }

      // cost += err_func.norm();
      cost += weight * r2;

      MatXd Jacob = MatXd::Zero(2, 3);

      Jacob << -1.0, 0.0,
          sin(opt_est(2)) * current_points_(0, cur_id) +
              cos(opt_est(2)) * current_points_(1, cur_id),
          0.0, -1.0,
          -cos(opt_est(2)) * current_points_(0, cur_id) +
              sin(opt_est(2)) * current_points_(1, cur_id);

      J += Jacob;
      // H += Jacob.transpose() * Jacob;
      // b += Jacob.transpose() * err_func;
      H += weight * Jacob.transpose() * Jacob;
      b += weight * Jacob.transpose() * err_func;
    }

    if (cost < optimal) {
      optimal = cost;
      final_est = opt_est;
    }

    Vec3d d_est = -(H + lamb * Mat3d::Identity()).inverse() * b;
    opt_est += d_est;

    if (i > 0) {
      if (cost < cost_prev) {
        lamb = std::max(lamb / 9.0, 1e-7);
      } else {
        lamb = std::min(lamb * 9.0, 1e7);
      }
    }

    cost_prev = cost;
  }

  if ( ( init_trs - Vec3d{final_est(0), final_est(1), 0.0}.norm() > 5.0) || ( std::abs(init_yaw - final_est(2)) > 20.0 * d2r ) ) {

    return false;

  }
  
  icp_pose.position = Vec3d{final_est(0), final_est(1), 0.0};
  icp_pose.quaternion = euler2quat(Vec3d{0.0, 0.0, final_est(2)});

  return true;
}

Vec3d RadarEstimator::getEgoVelocity() { return ego_velocity_; }

MatXd RadarEstimator::getPointMatrix() { return radar_points_; }

void RadarEstimator::setEgoVelocity(Vec3d velocity) {
  ego_velocity_ = velocity;
}

void RadarEstimator::setCurrentPoints(MatXd points) {
  current_points_ = points;
}

void RadarEstimator::setPreviousPoints(MatXd points) {
  previous_points_ = points;
  accum_points_ = points;
}

void RadarEstimator::pointAccumulation(MatXd points) {
  int old_cols = accum_points_.cols();
  int new_cols = points.cols();

  accum_points_.conservativeResize(Eigen::NoChange, old_cols + new_cols);
  accum_points_.rightCols(points.cols()) = points;
}

MatXd RadarEstimator::getCurrentPoints() { return current_points_; }

} // namespace navigation
