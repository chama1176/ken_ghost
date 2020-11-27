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

  Tb0_ = Tb0();
  T01_.setIdentity();
  T12_.setIdentity();
  T23_.setIdentity();
  T34_.setIdentity();
  T45_.setIdentity();
  T5e_ = T5e();
}

KenFK::~KenFK() {}

Matrix4d KenFK::getTbe(std::vector<double> joint_angle)
{
  Matrix4d Tbe;
  setTransform(joint_angle);
  Tbe = Tb0_ * T01_ * T12_ * T23_ * T34_ * T45_ * T5e_;

  std::cout << Tbe << std::endl;

  return Tbe;
}

Matrix4d KenFK::getJv(std::vector<double> joint_angle)
{
  Matrix4d Jv;
  setTransform(joint_angle);

  return Jv;
}

void KenFK::setTransform(std::vector<double> joint_angle)
{
  if (joint_angle.size() == 5) {
    T01_ = T01(joint_angle[0]);
    T12_ = T12(joint_angle[1]);
    T23_ = T23(joint_angle[2]);
    T34_ = T34(joint_angle[3]);
    T45_ = T45(joint_angle[4]);
  }
}
