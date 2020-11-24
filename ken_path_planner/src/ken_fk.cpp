#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

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

Eigen::Matrix4d KenFK::dhT(const double a, const double alfa, const double d, const double theta)
{
  Matrix4d M;

  M = transX(a) * rotX(alfa) * transZ(d) * rotZ(theta);

  return M;
}

Matrix4d KenFK::rotX(const double rad)
{
  Matrix3d R = AngleAxisd(rad, Vector3d::UnitX()).toRotationMatrix();
  Matrix4d M;
  M.setIdentity();
  M.block<3, 3>(0, 0) = R;
  return M;
}

Matrix4d KenFK::transX(const double m)
{
  Matrix4d M;
  M.setIdentity();
  M(0, 3) = m;
  return M;
}

Matrix4d KenFK::rotZ(const double rad)
{
  Matrix3d R = AngleAxisd(rad, Vector3d::UnitZ()).toRotationMatrix();
  Matrix4d M;
  M.setIdentity();
  M.block<3, 3>(0, 0) = R;
  return M;
}

Matrix4d KenFK::transZ(const double m)
{
  Matrix4d M;
  M.setIdentity();
  M(2, 3) = m;
  return M;
}
