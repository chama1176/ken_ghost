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

std::vector<double> KenIK::calcPositionIK()
{
  std::vector<double> joint_angle(5, 0.0);

  return joint_angle;
}
