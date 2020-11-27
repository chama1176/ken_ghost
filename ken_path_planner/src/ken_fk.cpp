#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "ken_path_planner/eigen_dh.hpp"
#include "ken_path_planner/ken_fk.hpp"

using namespace Eigen;

KenFK::KenFK()
{
  std::cout << "rotX" << std::endl;
  std::cout << rotX(M_PI / 4) << std::endl;
  std::cout << "transX" << std::endl;
  std::cout << transX(4.3) << std::endl;
  std::cout << "rotZ" << std::endl;
  std::cout << rotZ(M_PI / 4) << std::endl;
  std::cout << "transZ" << std::endl;
  std::cout << transZ(2.3) << std::endl;
  std::cout << "dhT" << std::endl;
  std::cout << dhT(2, 0.0, 3, M_PI / 4) << std::endl;
}

KenFK::~KenFK() {}

Matrix4d KenFK::getTbe(std::vector<double> joint_angle)
{
  Matrix4d Tbe;

  if (joint_angle.size() == 5) {
    Tbe = Tb0() * T01(joint_angle[0]) * T12(joint_angle[1]) * T23(joint_angle[2]) *
          T34(joint_angle[3]) * T45(joint_angle[4]) * T5e();
  }
  std::cout << Tbe << std::endl;

  return Tbe;
}

Matrix4d KenFK::getJv(std::vector<double> joint_angle)
{
  Matrix4d Jv;

  return Jv;
}
