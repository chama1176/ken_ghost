#ifndef KEN_IK_HPP_
#define KEN_IK_HPP_

#include <memory>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>

#include "ken_path_planner/eigen_dh.hpp"
#include "ken_path_planner/ken_fk.hpp"

class KenIK
{
public:
  KenIK();
  ~KenIK();
  std::vector<double> calcPositionIK(const Vector3d & target_pos);
  std::vector<Matrix4d> getIKlogTbe(void);

private:
  std::vector<std::vector<double>> calc_joint_angle_log_;
  std::vector<Matrix4d> calc_Tbe_log_;
  std::vector<double> joint_limit_min_ = {deg2rad(-180.0), deg2rad(-10.0), deg2rad(-150.0),
                                          deg2rad(-90.0), deg2rad(-180.0)};
  std::vector<double> joint_limit_max_ = {deg2rad(180.0), deg2rad(180.0), deg2rad(150.0),
                                          deg2rad(90.0), deg2rad(180.0)};

  void resetJointLimit(std::vector<double> & joint_angle);
  inline double deg2rad(double deg) { return deg * M_PI / 180; }
};

#endif  // KEN_IK_HPP_