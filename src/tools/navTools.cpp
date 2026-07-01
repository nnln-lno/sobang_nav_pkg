#include "sobang_navigation/navTools.hpp"

namespace navigation
{
    Vec4d euler2quat(const Vec3d &euler)
    {
        double phi = euler(0);
        double theta = euler(1);
        double psi = euler(2);

        double a = cos(phi / 2.0)*cos(theta / 2.0)*cos(psi / 2.0) + sin(phi / 2.0)*sin(theta / 2.0)*sin(psi / 2.0);
        double b = sin(phi / 2.0)*cos(theta / 2.0)*cos(psi / 2.0) - cos(phi / 2.0)*sin(theta / 2.0)*sin(psi / 2.0);
        double c = cos(phi / 2.0)*sin(theta / 2.0)*cos(psi / 2.0) + sin(phi / 2.0)*cos(theta / 2.0)*sin(psi / 2.0);
        double d = cos(phi / 2.0)*cos(theta / 2.0)*sin(psi / 2.0) + sin(phi / 2.0)*sin(theta / 2.0)*cos(psi / 2.0);

        Vec4d quat = Vec4d(a, b, c, d);

        quat = quatNorm(quat); // Normalize the quaternion to ensure it's a unit quaternion
    
        return quat;
    }

    Vec4d dcm2quat(const Mat3d &dcm)
    {
        Vec4d quat = Vec4d::Zero();

        quat(0,0) = 0.5 * sqrt(1.0 + dcm(0,0) + dcm(1,1) + dcm(2,2));
        quat(1,0) = dcm(2,1) - dcm(1,2)/(4.0 * quat(0,0));
        quat(2,0) = dcm(0,2) - dcm(2,0)/(4.0 * quat(0,0));
        quat(3,0) = dcm(1,0) - dcm(0,1)/(4.0 * quat(0,0));
        // Placeholder for DCM to Quaternion conversion logic

        quat = quatNorm(quat); // Normalize the quaternion to ensure it's a unit quaternion

        return quat;
    }
        
    Mat3d euler2dcm(const Vec3d &euler)
    {
        double phi = euler(0);
        double theta = euler(1);
        double psi = euler(2);

        double cs = std::cos(psi);
        double ss = std::sin(psi);
        double ct = std::cos(theta);
        double st = std::sin(theta);
        double cp = std::cos(phi);
        double sp = std::sin(phi);

        Mat3d Cx, Cy, Cz;
        
        Cx << 1, 0, 0,
              0, cp, sp,
              0, -sp, cp;

        Cy << ct, 0, -st,
              0, 1, 0,
              st, 0, ct;

        Cz << cs, ss, 0,
              -ss, cs, 0,
              0, 0, 1;

        Mat3d DCM = Cz.transpose() * Cy.transpose() * Cx.transpose();
        // Placeholder for Euler to DCM conversion logic
        return DCM; // Dummy return value
    }

    Mat3d quat2dcm(const Vec4d &quat)
    {
        double a = quat(0);
        double b = quat(1);
        double c = quat(2);
        double d = quat(3);

        double c11, c12, c13;
        double c21, c22, c23; 
        double c31, c32, c33;

        c11 = a*a + b*b - c*c - d*d; c12 = 2*(b*c - a*d);         c13 = 2*(b*d + a*c);
        c21 = 2*(b*c + a*d);         c22 = a*a - b*b + c*c - d*d; c23 = 2*(c*d - a*b);
        c31 = 2*(b*d - a*c);         c32 = 2*(c*d + a*b);         c33 = a*a - b*b - c*c + d*d;

        Mat3d DCM;

        DCM.row(0) << c11, c12, c13;
        DCM.row(1) << c21, c22, c23;
        DCM.row(2) << c31, c32, c33;

        return DCM; // Dummy return value
    }

    Vec3d dcm2euler(const Mat3d &dcm)
    {
        double phi = atan2(dcm(2,1), dcm(2,2));
        double theta = asin(-dcm(2,0));
        double psi = atan2(dcm(1,0), dcm(0,0));
        
        Vec3d euler = Vec3d(phi, theta, psi);
        // Placeholder for DCM to Euler conversion logic
        return euler; // Dummy return value
    }

    Vec3d quat2euler(const Vec4d &quat)
    {
        double a = quat(0);
        double b = quat(1);        
        double c = quat(2);
        double d = quat(3);

        double phi = atan2(2*(c*d + a*b), 2*(a*a + d*d)-1);
        double theta = -asin(2*(b*d - a*c));
        double psi = atan2(2*(b*c + a*d), 2*(a*a + b*b)-1);

        Vec3d euler = Vec3d(phi, theta, psi);
        
        return euler;
    }

    Vec4d quatNorm(const Vec4d &quat)
    {
        double norm = quat.norm();
        if (norm > 0)
        {
            return quat / norm;
        }
        else
        {
            return Vec4d(1.0, 0.0, 0.0, 0.0); // Return identity quaternion if norm is zero
        }
    }

    Vec4d quatUpdate(const Vec4d &quat, const Vec3d &gyro, double dt)
    {
        Mat4d w_skew = skew44(gyro);

        Vec4d update_quat = (Eigen::MatrixXd::Identity(4,4) + 0.5 * w_skew * dt + 
                            0.125 * w_skew.cwiseProduct(w_skew) * dt * dt + 
                            1/6 * w_skew.cwiseProduct(w_skew.cwiseProduct(w_skew)) * dt * dt * dt 
                            + 1/24 * w_skew.cwiseProduct(w_skew.cwiseProduct(w_skew.cwiseProduct(w_skew))) * dt * dt * dt * dt ) * quat;   

        update_quat = quatNorm(update_quat);                                   
                        
        return update_quat;
    }

    Vec4d quatProd(const Vec4d &q1, const Vec4d &q2)
    {
        Vec4d quat_prod;

        quat_prod(0) = q1(0)*q2(0) - q1(1)*q2(1) - q1(2)*q2(2) - q1(3)*q2(3);
        quat_prod(1) = q1(0)*q2(1) + q1(1)*q2(0) + q1(2)*q2(3) - q1(3)*q2(2);
        quat_prod(2) = q1(0)*q2(2) - q1(1)*q2(3) + q1(2)*q2(0) + q1(3)*q2(1);
        quat_prod(3) = q1(0)*q2(3) + q1(1)*q2(2) - q1(2)*q2(1) + q1(3)*q2(0);

        quat_prod = quatNorm(quat_prod); // Normalize the resulting quaternion

        return quat_prod; // Dummy return value
    }

    Mat4d skew44(const Vec3d &vec)
    {
        Mat4d skew = Mat4d::Zero();

        double x = vec(0);
        double y = vec(1);
        double z = vec(2);

        skew.row(0) <<  0,  -x,  -y,  -z;
        skew.row(1) <<  x,   0,   z,  -y;
        skew.row(2) <<  y,  -z,   0,   x;
        skew.row(3) <<  z,   y,  -x,   0;

        // Placeholder for skew symmetric matrix generation logic (4x4)
        return skew; // Dummy return value
    }

    Mat3d skew33(const Vec3d &vec)
    {
        Mat3d skew = Mat3d::Zero();

        double x = vec(0);
        double y = vec(1);
        double z = vec(2);

        skew <<  0,  -z,   y,
                 z,   0,  -x,
                -y,   x,   0;

        // Placeholder for skew symmetric matrix generation logic (3x3)
        return skew; // Dummy return value
    }

    Mat3d rightJacobianSO3(const Vec3d phi)
    {
        double angle = phi.norm();
        Mat3d I = Mat3d::Identity();

        if (angle < 1e-6)
        {
            return I; // Return identity for small angles to avoid numerical issues
        }
        else
        {
            Vec3d axis = phi / angle;
            Mat3d skew_axis = skew33(axis);
            return I - (1 - cos(angle)) / (angle * angle) * skew_axis + (angle - sin(angle)) / (angle * angle * angle) * skew_axis * skew_axis;
        }
    }

}