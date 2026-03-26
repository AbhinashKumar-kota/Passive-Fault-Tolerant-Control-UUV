// Reference frame transformations between body frame and NED frame (ZYX Euler convention).

#ifndef FTC_CONTROL_UTILS_FRAMES_HPP_
#define FTC_CONTROL_UTILS_FRAMES_HPP_

#include <Eigen/Dense>
#include <cmath>

namespace ftc_control {
namespace utils {

/// @brief Rotation matrix R_nb from Euler angles (body to NED, ZYX convention)
inline Eigen::Matrix3d eulerToRotation(double phi, double theta, double psi) {
    double cphi = std::cos(phi);
    double sphi = std::sin(phi);
    double cth = std::cos(theta);
    double sth = std::sin(theta);
    double cpsi = std::cos(psi);
    double spsi = std::sin(psi);

    Eigen::Matrix3d R;
    R << cpsi*cth, cpsi*sth*sphi - spsi*cphi, cpsi*sth*cphi + spsi*sphi,
         spsi*cth, spsi*sth*sphi + cpsi*cphi, spsi*sth*cphi - cpsi*sphi,
         -sth,     cth*sphi,                  cth*cphi;

    return R;
}

/// @brief Euler rate transform T(phi, theta): eta_dot_rot = T * omega. Singular at theta = +/-90deg (gimbal lock).
inline Eigen::Matrix3d eulerRateTransform(double phi, double theta) {
    double cphi = std::cos(phi);
    double sphi = std::sin(phi);
    double cth = std::cos(theta);
    double tth = std::tan(theta);

    Eigen::Matrix3d T;
    T << 1, sphi*tth,  cphi*tth,
         0, cphi,      -sphi,
         0, sphi/cth,  cphi/cth;

    return T;
}

/// @brief Full 6-DOF Jacobian: eta_dot = J(eta) * nu. Block-diagonal [R_nb, T].
inline Eigen::Matrix<double, 6, 6> getJacobian(
    double phi, double theta, double psi
) {
    Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Zero();
    J.block<3, 3>(0, 0) = eulerToRotation(phi, theta, psi);
    J.block<3, 3>(3, 3) = eulerRateTransform(phi, theta);
    return J;
}

/// @brief Quaternion [x,y,z,w] to Euler angles [roll, pitch, yaw] (ZYX)
inline Eigen::Vector3d quaternionToEuler(double qx, double qy, double qz, double qw) {
    Eigen::Vector3d euler;

    // Roll
    double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    euler(0) = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (clamped for gimbal lock)
    double sinp = 2.0 * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1)
        euler(1) = std::copysign(M_PI / 2, sinp);
    else
        euler(1) = std::asin(sinp);

    // Yaw
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    euler(2) = std::atan2(siny_cosp, cosy_cosp);

    return euler;
}

/// @brief Euler angles [roll, pitch, yaw] to quaternion [x,y,z,w] (ZYX)
inline Eigen::Vector4d eulerToQuaternion(double phi, double theta, double psi) {
    double cy = std::cos(psi * 0.5);
    double sy = std::sin(psi * 0.5);
    double cp = std::cos(theta * 0.5);
    double sp = std::sin(theta * 0.5);
    double cr = std::cos(phi * 0.5);
    double sr = std::sin(phi * 0.5);

    Eigen::Vector4d q;
    q(3) = cr * cp * cy + sr * sp * sy;  // w
    q(0) = sr * cp * cy - cr * sp * sy;  // x
    q(1) = cr * sp * cy + sr * cp * sy;  // y
    q(2) = cr * cp * sy - sr * sp * cy;  // z

    return q;
}

}  // namespace utils
}  // namespace ftc_control

#endif  // FTC_CONTROL_UTILS_FRAMES_HPP_
