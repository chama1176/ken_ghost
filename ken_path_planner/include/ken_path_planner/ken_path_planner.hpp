#ifndef KEN_PATH_PLANNER_HPP_
#define KEN_PATH_PLANNER_HPP_

#include <math.h>
#include <cinttypes>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>

#include <chrono>
#include <functional>

#include <eigen3/Eigen/Dense>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "ken_msgs/msg/mission_target_array.hpp"
#include "ken_msgs/msg/mission_trajectory.hpp"
#include "ken_path_planner/ken_fk.hpp"
#include "ken_path_planner/ken_ik.hpp"

using std::placeholders::_1;

class KenPathPlanner : public rclcpp::Node
{
public:
  /* 
  * コンストラクタ
  */
  KenPathPlanner();
  /*
  * デストラクタ
  */
  ~KenPathPlanner();

private:
  bool makeCurrentPosTrajectory(trajectory_msgs::msg::JointTrajectory & jtm);
  bool makeMoveHomeTrajectory(
    trajectory_msgs::msg::JointTrajectory & jtm, const std::vector<double> & current_pos);
  bool makeMoveKamaeTrajectory(
    trajectory_msgs::msg::JointTrajectory & jtm, const std::vector<double> & current_pos);
  bool makeMenTrajectory(
    trajectory_msgs::msg::JointTrajectory & jtm, const std::vector<double> & current_pos,
    const geometry_msgs::msg::Pose pose);
  bool makeRDouTrajectory(
    trajectory_msgs::msg::JointTrajectory & jtm, const std::vector<double> & current_pos,
    const geometry_msgs::msg::Pose pose);

  bool makeMenAfterTrajectory(trajectory_msgs::msg::JointTrajectory & jtm);

  void pushInterpolateTrajectoryPoints(
    trajectory_msgs::msg::JointTrajectory & jtm,
    const trajectory_msgs::msg::JointTrajectoryPoint start,
    const trajectory_msgs::msg::JointTrajectoryPoint end, int interpolate_num);

  void publishIKlog(const KenIK & ik);

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void mission_target_callback(const ken_msgs::msg::MissionTargetArray::SharedPtr msg);

  inline builtin_interfaces::msg::Duration second2duration(const double & second)
  {
    builtin_interfaces::msg::Duration duration;
    duration.sec = static_cast<int32_t>(second);
    duration.nanosec = static_cast<int32_t>((second - static_cast<double>(duration.sec)) * 1e9);

    return duration;
  }

  inline geometry_msgs::msg::Pose transform2pose(const Eigen::Matrix4d & Tbe)
  {
    geometry_msgs::msg::Pose p;
    Eigen::Quaterniond Tbe_q(Tbe.block<3, 3>(0, 0));
    p.position.x = Tbe(0, 3);
    p.position.y = Tbe(1, 3);
    p.position.z = Tbe(2, 3);
    p.orientation.x = Tbe_q.x();
    p.orientation.y = Tbe_q.y();
    p.orientation.z = Tbe_q.z();
    p.orientation.w = Tbe_q.w();
    return p;
  }

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr fk_debug_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr ik_debug_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr path_debug_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_debug_pub_;

  rclcpp::Publisher<ken_msgs::msg::MissionTrajectory>::SharedPtr mission_trajectory_pub_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  rclcpp::Subscription<ken_msgs::msg::MissionTargetArray>::SharedPtr mission_target_sub_;

  int64_t joint_num_;
  double move_time_;

  const std::string base_frame_id_;

  std::vector<std::string> name_vec_;
  std::vector<double> current_pos_;
  std::vector<double> kamae_pos_;
  const std::vector<double> rdou_base_pos_ = {0.0, M_PI_2, -M_PI_2, -M_PI_2, M_PI_4};
};

#endif  // KEN_PATH_PLANNER_HPP_