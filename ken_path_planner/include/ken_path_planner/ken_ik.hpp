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
  std::vector<double> calcPositionIK();

private:
  KenFK fk_;
  std::vector<std::vector<double>> ik_calc_log_;
};

#endif  // KEN_IK_HPP_