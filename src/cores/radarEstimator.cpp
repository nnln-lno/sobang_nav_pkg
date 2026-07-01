#include "sobang_navigation/radarEstimator.hpp"

#include "random"
#include "algorithm"
#include "unsupported/Eigen/MatrixFunctions"

namespace navigation
{
    RadarEstimator::RadarEstimator()
    {
        std::cout << "Success initializing Radar Dead Reckoning Node." << std::endl;

        setEgoVelocity(Vec3d{0.0, 0.0, 0.0});
    }
    
    bool RadarEstimator::radarParser(const sensor_msgs::msg::PointCloud2::SharedPtr &radar_msg)
    {
        point_size_ = radar_msg->width * radar_msg->height;

        radar_points_ = MatXd(3, point_size_);
        radar_velocities_ = VecXd(point_size_);

        sensor_msgs::PointCloud2Iterator<float> iter_x(*radar_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*radar_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*radar_msg, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_vel(*radar_msg, "v_doppler");            

        for(int i=0; i<point_size_; i++, ++iter_x, ++iter_y, ++iter_z, ++iter_vel)
        {        
            radar_points_(0,i) = *iter_x;
            radar_points_(1,i) = *iter_y;
            radar_points_(2,i) = *iter_z;

            radar_velocities_(i) = *iter_vel;
            // Logic to parse radar pointcloud data and extract points and velocities would go here
        }
        
        return true; // Dummy return value
    }    

    bool RadarEstimator::egoVelocityEstimator()
    {
        // Change hyperparameter for velocity estmation using config file or dynamic reconfigure [TBD]        

        // This for checking whether the radar pointcloud data is parsed correctly. Remove this after testing.
        // std::cout << "Radar Points: " << radar_points_.cols() << std::endl;                

        if ( radar_velocities_.cwiseAbs().mean() < 0.05 ) // [HYPERPARAM] threshold for zero velocity
        {
            // RCLCPP_WARN(this->get_logger(), "Zero Velocity Detected! (count : %d)", ++zero_velocity_count_);
            setEgoVelocity(Vec3d{0.0, 0.0, 0.0});
            return false;
        }

        std::vector<int> valid_indices;

        for(int i=0; i<point_size_; i++)
        {
            if (std::abs(radar_velocities_(i)) < 3.0 && radar_points_.col(i).norm() > 0.5) // [HYPERPARAM] threshold for valid radar points based on velocity
            {
                valid_indices.push_back(i);
            }
        }

        if (valid_indices.size() < 3)
        {            
            std::cout << "Not enough valid radar points for velocity estimation! (valid points : " << valid_indices.size() << ")" << std::endl;
            return false;
        }    
                
        VecXd point_norm = radar_points_(Eigen::all,valid_indices).colwise().norm();
        
        MatXd H(valid_indices.size(), 3);
        VecXd y(valid_indices.size());

        for (size_t i = 0; i < valid_indices.size(); ++i)
        {
            H(i, 0) = radar_points_(0, valid_indices[i]) / point_norm(i);
            H(i, 1) = radar_points_(1, valid_indices[i]) / point_norm(i);
            H(i, 2) = radar_points_(2, valid_indices[i]) / point_norm(i);
            y(i)    = radar_velocities_(valid_indices[i]);
        }

        std::random_device rd;
        std::mt19937 g(rd());

        uint8_t max_count_ = 0;

        std::vector<uint> inlier_indices_;

        for (int iter = 0; iter < 200; ++iter) // Example: Shuffle 100 times RANSAC iterations
        {
            // Shuffle and only take 3 front index data
            std::shuffle(valid_indices.begin(), valid_indices.end(), g);

            std::vector<int> test_indices_ = {valid_indices[0], valid_indices[1], valid_indices[2]};        

            Mat3d H_test = H(test_indices_, Eigen::all);
            Mat3d HTH = H_test.transpose() * H_test;

            Vec3d y_test = y(test_indices_);

            VecXd S = HTH.jacobiSvd().singularValues();

            if ( ( S.maxCoeff() / S.minCoeff() ) > 1e4 )
            {
                continue; // Skip if the matrix is close to singular
            }

            Vec3d init_est_ = -HTH.inverse() * H_test.transpose() * y_test;

            VecXd residual_ = (y + H * init_est_).cwiseAbs();

            uint8_t inlier_count = 0;
            std::vector<uint> est_inlier_indices_;
            
            for(int i=0; i<residual_.size(); i++)
            {
                if (residual_(i) <= 0.11) // [HYPERPARAM] threshold for inliers
                {
                    inlier_count++;                    
                    est_inlier_indices_.push_back(i);
                }
            }
            
            if (inlier_count > max_count_)
            {
                max_count_ = inlier_count;                
                inlier_indices_ = est_inlier_indices_;
                est_inlier_indices_.clear();
            }                    
                        
        }

        if (max_count_ < 3)
        {
            std::cout << "Not enough inliers for velocity estimation! (inliers : " << inlier_indices_.size() << ")" << std::endl;
            return false;
        }

        MatXd H_est = H(inlier_indices_, Eigen::all);
        VecXd y_est = y(inlier_indices_);

        ego_velocity_ = -(H_est.transpose() * H_est).inverse() * H_est.transpose() * y_est;

        return true; // Dummy return value
    }

    bool RadarEstimator::simpleRadar2DIcp(Mat3d R, Vec3d t)
    {
        icp_pose.position = t;
        icp_pose.quaternion = dcm2quat(R);

        uint iter = 200;
        double lamb = 0.5;        
        double delta = 0.1;

        double term_cond = 1e-7;
        double min_dt    = 1e-2; // m
        double min_dR    = 1e-2; // deg

        Mat3d R_est = euler2dcm(Vec3d{0.0, 0.0, dcm2euler(R)(2)}); // Only consider yaw rotation for 2D ICP
        Vec3d t_est = Vec3d{t(0), t(1), 0.0}; // Only consider x, y translation for 2D ICP
        int point_size = current_points_.cols();

        VecXd cost = VecXd::Ones(iter);
            
        for (uint i=0; i<iter; i++)
        {
            MatXd corr_set = MatXd::Zero(2, point_size);
            int match_cnt = 0;

            MatXd trf_points = R_est * current_points_ + t_est.replicate(1, current_points_.cols());            

            for (int j=0; j<point_size; j++)
            {
                VecXd dist = ( previous_points_ - trf_points.col(j).replicate(1, previous_points_.cols()) ).colwise().norm();

                int min_idx;
                double min_dist = dist.minCoeff(&min_idx);

                if (min_dist < 1.0)
                {
                    // Current point j corresponds to previous point min_idx
                    corr_set.col(match_cnt) = Vec2d(j, min_idx);
                    match_cnt++;
                }
            }

            if (match_cnt < 3)
            {            
                return false;
            }

            MatXd H = MatXd::Zero(3, 3);
            VecXd b = VecXd::Zero(3, 1);       
            double cost_sum = 0.0;     

            for (int j=0; j<match_cnt; j++)
            {
                Vec3d p = trf_points.col(corr_set(0, j));
                Vec3d q = previous_points_.col(corr_set(1, j));

                Vec3d res = q - p;                

                // Huber Loss for Outlier Rejection
                double denom = (res.norm() * res.norm()) + (delta * delta);
                cost_sum = cost_sum + ( (res.norm()) * (res.norm()) ) / denom;
                double weight = ( delta*delta ) / (denom * denom);
                
                MatXd Full_J = MatXd::Zero(3, 6);
                Full_J << -Mat3d::Identity(), skew33(R_est * current_points_.col(corr_set(0, j)));

                Mat3d J = Mat3d::Zero(); // Only consider x, y translation and yaw rotation for 2D ICP
                J << Full_J.block(0,0,3,2), Full_J.block(0,5,3,1);

                b += weight * J.transpose() * res;
                H += weight * J.transpose() * J;
            }

            cost(i) = cost_sum;
            Vec3d d_pose = -(H + lamb * Mat3d::Identity()).inverse() * b;

            Vec2d translation_update = d_pose.head(2);
            double rotation_update = d_pose(2);

            R_est = R_est * (skew33(Vec3d{0.0, 0.0, rotation_update})).exp(); 
            t_est = t_est + Vec3d{d_pose(0), d_pose(1), 0.0};

            if ( i > 0 )
            {
                if ( (std::abs(cost(i) - cost(i-1)) < term_cond) || ( (translation_update.norm() < min_dt) && ((Vec3d{0.0, 0.0, rotation_update*r2d}).norm() < min_dR)) )
                {
                    icp_pose.position = t_est;
                    icp_pose.quaternion = dcm2quat(R_est);

                    // std::cout << "ICP Converged at iteration " << i << " with cost " << cost(i) << std::endl;

                    return true; // Convergence check
                }

                if ( cost(i) < cost(i-1) )
                {
                    lamb = std::max(lamb / 9, 1e-7); // Decrease lambda if cost is decreasing
                }
                else
                {
                    lamb = std::min(lamb * 9, 1e+7); // Increase lambda if cost is increasing
                }
            }

            if ( (std::isnan(translation_update.norm())) || (translation_update.norm() == 0.0) || (std::abs(rotation_update) == 0.0) )
            {
                return false; // Return false if update is invalid
            }
        }

        return false; // Dummy return value
    }

    Vec3d RadarEstimator::getEgoVelocity()
    {
        return ego_velocity_;
    }

    MatXd RadarEstimator::getPointMatrix()
    {
        return radar_points_;
    }

    void RadarEstimator::setEgoVelocity(Vec3d velocity)
    {
        ego_velocity_ = velocity;
    }

    void RadarEstimator::setCurrentPoints(MatXd points)
    {
        current_points_ = points;
    }

    void RadarEstimator::setPreviousPoints(MatXd points)
    {
        previous_points_ = points;
    }

    MatXd RadarEstimator::getCurrentPoints()
    {
        return current_points_;
    }

}  // namespace navigation