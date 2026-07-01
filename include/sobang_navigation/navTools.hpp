#ifndef _NAV_TOOS_HPP_
#define _NAV_TOOS_HPP_

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <iostream>
#include <math.h>

// Variable Definition for Eigen Variable
namespace navigation
{
    inline double d2r = M_PI / 180.0; // degree to radian conversion factor
    inline double r2d = 180.0 / M_PI; // radian to degree conversion factor

    typedef Eigen::Matrix<double, 1, 1> Vec1d;
    typedef Eigen::Matrix<double, 2, 1> Vec2d;
    typedef Eigen::Matrix<double, 3, 1> Vec3d;
    typedef Eigen::Matrix<double, 4, 1> Vec4d;
    typedef Eigen::Matrix<double, 5, 1> Vec5d;
    typedef Eigen::Matrix<double, 6, 1> Vec6d;
    typedef Eigen::Matrix<double, 10, 1> Vec10d;
    typedef Eigen::Matrix<double, 12, 1> Vec12d;
    typedef Eigen::VectorXd VecXd;

    typedef Eigen::Matrix<double, 1, 1> Mat1d;
    typedef Eigen::Matrix<double, 2, 2> Mat2d;
    typedef Eigen::Matrix<double, 3, 3> Mat3d;
    typedef Eigen::Matrix<double, 4, 4> Mat4d;
    typedef Eigen::Matrix<double, 5, 5> Mat5d;
    typedef Eigen::Matrix<double, 6, 6> Mat6d;
    typedef Eigen::Matrix<double, 10, 10> Mat10d;
    typedef Eigen::Matrix<double, 12, 12> Mat12d;
    typedef Eigen::MatrixXd MatXd;

    typedef Eigen::Quaterniond Quat;
    
    // Define state variable struct later.. [TBD]
    typedef struct {
        Vec3d position; // x, y, z
        Vec3d velocity; // vx, vy, vz
        Vec4d quaternion; // roll, pitch, yaw
        Vec3d accel_bias; // ax_bias, ay_bias, az_bias
        Vec3d gyro_bias; // gx_bias, gy_bias, gz_bias            
    } insState;

    typedef struct {
        Vec3d position; // x, y, z        
        Vec4d quaternion; // roll, pitch, yaw
        Vec3d gyro_bias;
        Vec3d scale;
    } drState;

    typedef struct{
        Vec3d position; // x, y, z        
        Vec4d quaternion; // roll, pitch, yaw        
    } icpState;

    // To Quaternion
    Vec4d euler2quat(const Vec3d &euler);

    Vec4d dcm2quat(const Mat3d &dcm);
        
    // To DCM
    Mat3d euler2dcm(const Vec3d &euler);

    Mat3d quat2dcm(const Vec4d &quat);

    // To Euler
    Vec3d dcm2euler(const Mat3d &dcm);

    Vec3d quat2euler(const Vec4d &quat);

    // Quaternion Normalization
    Vec4d quatNorm(const Vec4d &quat);

    Vec4d quatUpdate(const Vec4d &quat, const Vec3d &gyro, double dt);

    Vec4d quatProd(const Vec4d &q1, const Vec4d &q2);

    // To Skew Symmetric Matrix
    Mat4d skew44(const Vec3d &vec);
    Mat3d skew33(const Vec3d &vec);

    Mat3d rightJacobianSO3(const Vec3d phi);

}

#endif  // _NAV_TOOS_HPP_