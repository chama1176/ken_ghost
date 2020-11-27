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

  auto ez = Vector3d(0, 0, 1);
  std::cout << "ez" << std::endl;
  std::cout << ez << std::endl;

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

MatrixXd KenFK::getJv(std::vector<double> joint_angle)
{
  setTransform(joint_angle);

  Matrix3d R01 = T01_.block<3, 3>(0, 0);
  Matrix3d R12 = T12_.block<3, 3>(0, 0);
  Matrix3d R23 = T23_.block<3, 3>(0, 0);
  Matrix3d R34 = T34_.block<3, 3>(0, 0);
  Matrix3d R45 = T45_.block<3, 3>(0, 0);

  Matrix3d R02 = R01 * R12;
  Matrix3d R03 = R02 * R23;
  Matrix3d R04 = R03 * R34;
  Matrix3d R05 = R04 * R45;

  auto ez = Vector3d(0, 0, 1);

  Vector3d z01 = R01 * ez;
  Vector3d z02 = R02 * ez;
  Vector3d z03 = R03 * ez;
  Vector3d z04 = R04 * ez;
  Vector3d z05 = R05 * ez;

  Vector3d p0e5 = R05 * T5e_.block<3, 1>(0, 3);
  Vector3d p0e4 = p0e5 + R04 * T45_.block<3, 1>(0, 3);
  Vector3d p0e3 = p0e4 + R03 * T34_.block<3, 1>(0, 3);
  Vector3d p0e2 = p0e3 + R02 * T23_.block<3, 1>(0, 3);
  Vector3d p0e1 = p0e2 + R01 * T12_.block<3, 1>(0, 3);

  Vector3d zp01 = z01.cross(p0e1);
  Vector3d zp02 = z02.cross(p0e2);
  Vector3d zp03 = z03.cross(p0e3);
  Vector3d zp04 = z04.cross(p0e4);
  Vector3d zp05 = z05.cross(p0e5);

  MatrixXd Jv(6, 5);
  Jv.block<3, 1>(0, 0) = zp01;
  Jv.block<3, 1>(0, 1) = zp02;
  Jv.block<3, 1>(0, 2) = zp03;
  Jv.block<3, 1>(0, 3) = zp04;
  Jv.block<3, 1>(0, 4) = zp05;
  Jv.block<3, 1>(3, 0) = z01;
  Jv.block<3, 1>(3, 1) = z02;
  Jv.block<3, 1>(3, 2) = z03;
  Jv.block<3, 1>(3, 3) = z04;
  Jv.block<3, 1>(3, 4) = z05;

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
