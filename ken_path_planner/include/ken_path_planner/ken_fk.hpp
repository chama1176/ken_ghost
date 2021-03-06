#ifndef KEN_FK_HPP_
#define KEN_FK_HPP_

#include <memory>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>

#include "ken_path_planner/eigen_dh.hpp"

class KenFK
{
public:
  KenFK(std::vector<double> joint_angle);
  ~KenFK();
  Matrix4d computeTbe();
  MatrixXd computeJv();

private:
  // a, alfa, d, theta
  inline Matrix4d Tb0() { return dhT(0.0, 0.0, 0.065, 0.0); }
  inline Matrix4d T01(const double & rad) { return dhT(0.0, 0.0, 0.0, rad); }
  inline Matrix4d T12(const double & rad) { return dhT(0.015, M_PI / 2, 0.02, rad); }
  inline Matrix4d T23(const double & rad) { return dhT(0.1, 0.0, 0.0, rad); }
  inline Matrix4d T34(const double & rad) { return dhT(0.125, 0.0, -0.014, rad - M_PI / 2); }
  inline Matrix4d T45(const double & rad) { return dhT(0.016, -M_PI / 2, 0.075, rad + M_PI); }
  inline Matrix4d T5e() { return dhT(0.31, 0.0, 0.0, 0.0); }

  void setTransform(std::vector<double> joint_angle);

  Matrix4d Tb0_;
  Matrix4d T01_;
  Matrix4d T12_;
  Matrix4d T23_;
  Matrix4d T34_;
  Matrix4d T45_;
  Matrix4d T5e_;
};

#endif  // KEN_FK_HPP_