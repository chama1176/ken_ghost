#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "ken_path_planner/eigen_dh.hpp"
#include "ken_path_planner/ken_fk.hpp"
#include "ken_path_planner/ken_ik.hpp"

using namespace Eigen;

KenIK::KenIK() {}

KenIK::~KenIK() {}

std::vector<double> KenIK::calcPositionIK(const Vector3d & target_pos)
{
  std::vector<double> joint_angle(5, 0.0);
  joint_angle[0] = 0.602854449639057;
  joint_angle[1] = 2.5356702423749646;
  joint_angle[2] = -2.495786741889938;
  joint_angle[3] = -1.5109710760673565;
  joint_angle[4] = -0.7516505860639642;

  for (size_t n = 0; n < 100; ++n) {
    KenFK fk(joint_angle);
    Matrix4d Tbe = fk.computeTbe();
    calc_Tbe_log_.push_back(Tbe);
    calc_joint_angle_log_.push_back(joint_angle);

    VectorXd e = target_pos - Tbe.block<3, 1>(0, 3);
    std::cout << "e" << e.transpose() << std::endl;
    if (std::abs(e(0)) <= 0.001 && std::abs(e(1)) <= 0.001 && std::abs(e(2)) <= 0.001) {
      std::cout << "IK solved" << std::endl;
      return joint_angle;
    }
    // TODO: 1e-6とかのオーダーでしか変化がない場合も抜け出すようにする

    MatrixXd Jv_pos = fk.computeJv().block<3, 5>(0, 0);
    MatrixXd W = MatrixXd::Identity(3, 3);
    MatrixXd E = (e.transpose() * W * e) / 2;

    MatrixXd Wn = 0.001 * MatrixXd::Identity(5, 5) + E(0, 0) * MatrixXd::Identity(5, 5);
    MatrixXd JWJ = Jv_pos.transpose() * W * Jv_pos + Wn;
    VectorXd dq = JWJ.inverse() * Jv_pos.transpose() * W * e;

    for (size_t jn = 0; jn < joint_angle.size(); ++jn) {
      joint_angle[jn] += dq(jn);
    }
    resetJointLimit(joint_angle);
  }

  std::cout << "Failed to solve IK" << std::endl;
  return joint_angle;
}

std::vector<Matrix4d> KenIK::getIKlogTbe(void) { return calc_Tbe_log_; }

void KenIK::resetJointLimit(std::vector<double> & joint_angle)
{
  for (size_t jn = 0; jn < joint_angle.size(); ++jn) {
    if (joint_angle[jn] < joint_limit_min_[jn]) joint_angle[jn] = joint_limit_min_[jn];
    if (joint_limit_max_[jn] < joint_angle[jn]) joint_angle[jn] = joint_limit_max_[jn];
  }
}
