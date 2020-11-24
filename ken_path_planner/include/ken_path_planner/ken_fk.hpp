#ifndef KEN_FK_HPP_
#define KEN_FK_HPP_

#include <memory>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>

class KenFK
{
public:
  KenFK();
  ~KenFK();

private:
  Eigen::Matrix4d dhT(const double a, const double alfa, const double d, const double theta);
  Eigen::Matrix4d rotX(const double rad);
  Eigen::Matrix4d transX(const double m);
  Eigen::Matrix4d rotZ(const double rad);
  Eigen::Matrix4d transZ(const double m);

  Eigen::Matrix4d T01_;
};

#endif  // KEN_FK_HPP_