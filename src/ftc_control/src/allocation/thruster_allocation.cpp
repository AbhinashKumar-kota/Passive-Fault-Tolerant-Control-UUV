#include "ftc_control/allocation/thruster_allocation.hpp"
#include <stdexcept>
#include <cmath>
#include <iostream>

namespace ftc_control {
namespace allocation {

ThrusterAllocator::ThrusterAllocator(int num_thrusters)
    : num_thrusters_(num_thrusters),
      fault_status_(num_thrusters) {

    B_.resize(DOF, num_thrusters_);
    B_.setZero();

    B_pinv_.resize(num_thrusters_, DOF);
    B_pinv_.setZero();
}

void ThrusterAllocator::initGirona500() {
    num_thrusters_ = 5;
    config_.clear();
    config_.resize(num_thrusters_);
    fault_status_ = FaultStatus(num_thrusters_);

    // T1: Surge port
    config_[0].name = "surge_port";
    config_[0].position = Eigen::Vector3d(-0.5, 0.22, 0.0);
    config_[0].direction = Eigen::Vector3d(1.0, 0.0, 0.0);

    // T2: Surge starboard
    config_[1].name = "surge_starboard";
    config_[1].position = Eigen::Vector3d(-0.5, -0.22, 0.0);
    config_[1].direction = Eigen::Vector3d(1.0, 0.0, 0.0);

    // T3: Sway
    config_[2].name = "sway";
    config_[2].position = Eigen::Vector3d(-0.5, 0.0, 0.0);
    config_[2].direction = Eigen::Vector3d(0.0, 1.0, 0.0);

    // T4: Heave fore
    config_[3].name = "heave_fore";
    config_[3].position = Eigen::Vector3d(0.3, 0.0, 0.0);
    config_[3].direction = Eigen::Vector3d(0.0, 0.0, 1.0);

    // T5: Heave aft
    config_[4].name = "heave_aft";
    config_[4].position = Eigen::Vector3d(-0.3, 0.0, 0.0);
    config_[4].direction = Eigen::Vector3d(0.0, 0.0, 1.0);

    B_.resize(DOF, num_thrusters_);
    B_.setZero();

    // Surge
    B_(0, 0) = 1.0;   // T1
    B_(0, 1) = 1.0;   // T2

    // Sway
    B_(1, 2) = 1.0;   // T3

    // Heave
    B_(2, 3) = 1.0;   // T4
    B_(2, 4) = 1.0;   // T5

    // Roll: not controllable on Girona500

    // Pitch: M = r_x × F_z
    B_(4, 3) = 0.3;   // T4 heave fore
    B_(4, 4) = -0.3;  // T5 heave aft

    // Yaw: N = r_y × F_x
    B_(5, 0) = 0.22;  // T1 surge port
    B_(5, 1) = -0.22; // T2 surge starboard

    updateAllocationMatrix();
}

void ThrusterAllocator::initBlueROV2() {
    num_thrusters_ = 8;
    config_.clear();
    config_.resize(num_thrusters_);
    fault_status_ = FaultStatus(num_thrusters_);

    // Horizontal thrusters (T1-T4) at 45deg angles for vectored surge/sway/yaw
    // Stonefish rpy yaw defines thrust axis; inverted flag negates setpoint internally.
    // Direction vectors pre-compensate for inversion:
    //   T1,T2 inverted=true; T3,T4 inverted=false

    double angle_45 = M_PI / 4.0;

    // T1: Front Right, yaw=-45deg, inverted=true
    config_[0].name = "front_right";
    config_[0].position = Eigen::Vector3d(0.1355, 0.1, 0.0725);
    config_[0].direction = Eigen::Vector3d(std::cos(3*angle_45), std::sin(3*angle_45), 0.0);

    // T2: Front Left, yaw=+45deg, inverted=true
    config_[1].name = "front_left";
    config_[1].position = Eigen::Vector3d(0.1355, -0.1, 0.0725);
    config_[1].direction = Eigen::Vector3d(std::cos(-3*angle_45), std::sin(-3*angle_45), 0.0);

    // T3: Back Right, yaw=-135deg, inverted=false
    config_[2].name = "back_right";
    config_[2].position = Eigen::Vector3d(-0.1475, 0.1, 0.0725);
    config_[2].direction = Eigen::Vector3d(std::cos(angle_45), std::sin(angle_45), 0.0);

    // T4: Back Left, yaw=+135deg, inverted=false
    config_[3].name = "back_left";
    config_[3].position = Eigen::Vector3d(-0.1475, -0.1, 0.0725);
    config_[3].direction = Eigen::Vector3d(std::cos(-angle_45), std::sin(-angle_45), 0.0);

    // Vertical thrusters (T5-T8) for heave/roll/pitch
    // Propeller axis points UP; +Z direction means positive force pushes robot down.
    // T5,T8 inverted=false; T6,T7 inverted=true

    // T5: Dive Front Right
    config_[4].name = "dive_front_right";
    config_[4].position = Eigen::Vector3d(0.12, 0.218, 0.0);
    config_[4].direction = Eigen::Vector3d(0.0, 0.0, 1.0);

    // T6: Dive Front Left
    config_[5].name = "dive_front_left";
    config_[5].position = Eigen::Vector3d(0.12, -0.218, 0.0);
    config_[5].direction = Eigen::Vector3d(0.0, 0.0, 1.0);

    // T7: Dive Back Right
    config_[6].name = "dive_back_right";
    config_[6].position = Eigen::Vector3d(-0.12, 0.218, 0.0);
    config_[6].direction = Eigen::Vector3d(0.0, 0.0, 1.0);

    // T8: Dive Back Left
    config_[7].name = "dive_back_left";
    config_[7].position = Eigen::Vector3d(-0.12, -0.218, 0.0);
    config_[7].direction = Eigen::Vector3d(0.0, 0.0, 1.0);

    // Build B matrix (6x8): force rows from direction, moment rows from r x direction
    B_.resize(DOF, num_thrusters_);
    B_.setZero();

    for (int i = 0; i < num_thrusters_; ++i) {
        B_.block<3, 1>(0, i) = config_[i].direction;
        Eigen::Vector3d moment = config_[i].position.cross(config_[i].direction);
        B_.block<3, 1>(3, i) = moment;
    }

    updateAllocationMatrix();

    // Debug output
    std::cout << "\n========== B Matrix (6x8) ==========\n";
    std::cout << "Row 0 (Fx): " << B_.row(0) << "\n";
    std::cout << "Row 1 (Fy): " << B_.row(1) << "\n";
    std::cout << "Row 2 (Fz): " << B_.row(2) << "\n";
    std::cout << "Row 3 (Mx): " << B_.row(3) << "\n";
    std::cout << "Row 4 (My): " << B_.row(4) << "\n";
    std::cout << "Row 5 (Mz): " << B_.row(5) << "\n";
    std::cout << "=====================================\n\n";
}

void ThrusterAllocator::setConfiguration(const std::vector<ThrusterConfig>& config) {
    num_thrusters_ = config.size();
    config_ = config;
    fault_status_ = FaultStatus(num_thrusters_);

    B_.resize(DOF, num_thrusters_);
    B_.setZero();

    for (int i = 0; i < num_thrusters_; ++i) {
        B_.block<3, 1>(0, i) = config_[i].direction;
        // Moment = r x F (torque about CoG)
        Eigen::Vector3d moment = config_[i].position.cross(config_[i].direction);
        B_.block<3, 1>(3, i) = moment;
    }

    updateAllocationMatrix();
}

void ThrusterAllocator::setFaultStatus(const FaultStatus& status) {
    fault_status_ = status;
    updateAllocationMatrix();
}

// u = B⁺ * τ (minimum-norm least-squares solution)
Eigen::VectorXd ThrusterAllocator::allocate(const Eigen::Matrix<double, DOF, 1>& tau) {
    return B_pinv_ * tau;
}

// Weighted pseudo-inverse: B_w⁺ = W⁻¹ * Bᵀ * (B * W⁻¹ * Bᵀ)⁻¹
Eigen::VectorXd ThrusterAllocator::allocateWeighted(
    const Eigen::Matrix<double, DOF, 1>& tau,
    const Eigen::MatrixXd& W) {

    Eigen::MatrixXd W_inv = W.inverse();
    Eigen::MatrixXd B_eff = getEffectiveB();
    Eigen::MatrixXd temp = B_eff * W_inv * B_eff.transpose();
    Eigen::MatrixXd B_w_pinv = W_inv * B_eff.transpose() * temp.inverse();

    return B_w_pinv * tau;
}

// B_eff[:,i] = B[:,i] * effectiveness[i]
Eigen::MatrixXd ThrusterAllocator::getEffectiveB() const {
    Eigen::MatrixXd B_eff = B_;

    for (int i = 0; i < num_thrusters_; ++i) {
        if (i < (int)fault_status_.effectiveness.size()) {
            B_eff.col(i) *= fault_status_.effectiveness[i];
        }
    }

    return B_eff;
}

bool ThrusterAllocator::isAchievable(const Eigen::Matrix<double, DOF, 1>& tau) const {
    Eigen::VectorXd u = B_pinv_ * tau;

    for (int i = 0; i < num_thrusters_; ++i) {
        if (i < (int)config_.size()) {
            if (u(i) > config_[i].max_thrust || u(i) < config_[i].min_thrust) {
                return false;
            }
        }
    }

    return true;
}

std::vector<std::string> ThrusterAllocator::getThrusterNames() const {
    std::vector<std::string> names;
    for (const auto& cfg : config_) {
        names.push_back(cfg.name);
    }
    return names;
}

// Pseudo-inverse via truncated SVD: B⁺ = V * Σ⁺ * Uᵀ
void ThrusterAllocator::updateAllocationMatrix() {
    Eigen::MatrixXd B_eff = getEffectiveB();
    B_pinv_.resize(num_thrusters_, DOF);

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(B_eff, Eigen::ComputeThinU | Eigen::ComputeThinV);

    // Threshold: singular values below this are treated as zero (regularization)
    double tolerance = 1e-6 * std::max(B_eff.cols(), B_eff.rows()) *
                       svd.singularValues().array().abs().maxCoeff();

    Eigen::VectorXd singularValuesInv(svd.singularValues().size());
    for (int i = 0; i < svd.singularValues().size(); ++i) {
        if (std::abs(svd.singularValues()(i)) > tolerance) {
            singularValuesInv(i) = 1.0 / svd.singularValues()(i);
        } else {
            singularValuesInv(i) = 0.0;
        }
    }

    // B⁺ = V * Σ⁺ * Uᵀ
    B_pinv_ = svd.matrixV() * singularValuesInv.asDiagonal() * svd.matrixU().transpose();
}

}  // namespace allocation
}  // namespace ftc_control
