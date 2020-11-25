#ifndef EIGEN_DH_HPP_
#define EIGEN_DH_HPP_

#include <memory>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>

using namespace Eigen;

inline Matrix4d rotX(const double & rad)
{
  Matrix3d R = AngleAxisd(rad, Vector3d::UnitX()).toRotationMatrix();
  Matrix4d M;
  M.setIdentity();
  M.block<3, 3>(0, 0) = R;
  return M;
}

inline Matrix4d transX(const double & m)
{
  Matrix4d M;
  M.setIdentity();
  M(0, 3) = m;
  return M;
}

inline Matrix4d rotZ(const double & rad)
{
  Matrix3d R = AngleAxisd(rad, Vector3d::UnitZ()).toRotationMatrix();
  Matrix4d M;
  M.setIdentity();
  M.block<3, 3>(0, 0) = R;
  return M;
}

inline Matrix4d transZ(const double & m)
{
  Matrix4d M;
  M.setIdentity();
  M(2, 3) = m;
  return M;
}

inline Eigen::Matrix4d dhT(
  const double & a, const double & alfa, const double & d, const double & theta)
{
  Matrix4d M;

  M = transX(a) * rotX(alfa) * transZ(d) * rotZ(theta);

  return M;
}

#endif  // EIGEN_DH_HPP_