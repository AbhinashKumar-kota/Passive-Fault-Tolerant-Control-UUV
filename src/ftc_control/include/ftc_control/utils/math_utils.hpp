// Math utilities for AUV control: angle wrapping, saturation, skew matrix, pseudo-inverse, filtering.

#ifndef FTC_CONTROL_UTILS_MATH_UTILS_HPP_
#define FTC_CONTROL_UTILS_MATH_UTILS_HPP_

#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

namespace ftc_control {
namespace utils {

/// @brief Normalize angle to [-pi, pi]
inline double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

/// @brief Clamp value to [min_val, max_val]
inline double saturate(double x, double min_val, double max_val) {
    return std::max(min_val, std::min(max_val, x));
}

/// @brief Sign function with deadzone epsilon
inline double sign(double x, double epsilon = 1e-6) {
    if (x > epsilon) return 1.0;
    if (x < -epsilon) return -1.0;
    return 0.0;
}

/// @brief Smooth saturation function for sliding mode control (boundary layer thickness epsilon)
inline double sat(double x, double epsilon) {
    if (std::abs(x) > epsilon) {
        return sign(x);
    }
    return x / epsilon;
}

/// @brief Skew-symmetric matrix S(v) such that S(v)*u = v x u
inline Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d S;
    S <<  0,    -v(2),  v(1),
          v(2),  0,    -v(0),
         -v(1),  v(0),  0;
    return S;
}

/// @brief Moore-Penrose pseudo-inverse via SVD
template<typename MatrixType>
inline MatrixType pseudoInverse(const MatrixType& A, double tolerance = 1e-6) {
    Eigen::JacobiSVD<MatrixType> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    auto singular_values = svd.singularValues();
    MatrixType S_inv = MatrixType::Zero(A.cols(), A.rows());

    for (int i = 0; i < singular_values.size(); ++i) {
        if (singular_values(i) > tolerance) {
            S_inv(i, i) = 1.0 / singular_values(i);
        }
    }

    return svd.matrixV() * S_inv * svd.matrixU().transpose();
}

/// @brief Weighted pseudo-inverse: W_inv * A^T * (A * W_inv * A^T)^-1
template<int Rows, int Cols>
inline Eigen::Matrix<double, Cols, Rows> weightedPseudoInverse(
    const Eigen::Matrix<double, Rows, Cols>& A,
    const Eigen::Matrix<double, Cols, Cols>& W
) {
    Eigen::Matrix<double, Cols, Cols> W_inv = W.inverse();
    Eigen::Matrix<double, Rows, Rows> temp = A * W_inv * A.transpose();
    return W_inv * A.transpose() * temp.inverse();
}

/// @brief First-order discrete low-pass filter
class LowPassFilter {
public:
    LowPassFilter(double cutoff_freq, double sample_time)
        : alpha_(1.0 - std::exp(-sample_time * cutoff_freq * 2.0 * M_PI)),
          initialized_(false) {}

    double filter(double input) {
        if (!initialized_) {
            output_ = input;
            initialized_ = true;
        } else {
            output_ = alpha_ * input + (1.0 - alpha_) * output_;
        }
        return output_;
    }

    void reset() { initialized_ = false; }

private:
    double alpha_;
    double output_ = 0.0;
    bool initialized_;
};

}  // namespace utils
}  // namespace ftc_control

#endif  // FTC_CONTROL_UTILS_MATH_UTILS_HPP_
