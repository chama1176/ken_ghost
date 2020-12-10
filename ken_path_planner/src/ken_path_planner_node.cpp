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
#include "ken_path_planner/ken_path_planner.hpp"

KenPathPlanner::KenPathPlanner() : Node("ken_path_planner"), base_frame_id_("base_link")
{
  this->declare_parameter("joint_num", -1);
  this->get_parameter("joint_num", joint_num_);

  this->declare_parameter("men_time", 0.0);
  this->get_parameter("men_time", men_time_);
  this->declare_parameter("dou_time", 0.0);
  this->get_parameter("dou_time", dou_time_);

  this->declare_parameter("move_time", 0.0);
  this->get_parameter("move_time", move_time_);

  this->declare_parameter("ik_test_pos_x", 0.0);
  this->declare_parameter("ik_test_pos_y", 0.0);
  this->declare_parameter("ik_test_pos_z", 0.0);

  if (joint_num_ > 0) {
    this->declare_parameter("name", std::vector<std::string>(joint_num_, ""));
    this->get_parameter("name", name_vec_);

    this->declare_parameter("kamae_pos", std::vector<double>(joint_num_, 0.0));
    this->get_parameter("kamae_pos", kamae_pos_);

    current_pos_.resize(joint_num_);

    for (size_t i = 0; i < (size_t)joint_num_; ++i) {
      RCLCPP_INFO(this->get_logger(), "Axis %d %s", i, name_vec_[i].c_str());
    }
  }

  RCLCPP_INFO(this->get_logger(), "Men time: %f", men_time_);
  RCLCPP_INFO(this->get_logger(), "Dou time: %f", dou_time_);

  RCLCPP_INFO(this->get_logger(), "Move time: %f", move_time_);
  RCLCPP_INFO(this->get_logger(), "Joint num: %d", joint_num_);

  cmd_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "ken_joint_trajectory_controller/joint_trajectory", 1);
  ik_debug_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("debug/ik_pose", 1);
  path_debug_pose_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseArray>("debug/path_pose", 1);
  path_debug_pub_ = this->create_publisher<nav_msgs::msg::Path>("debug/path", 1);

  mission_trajectory_pub_ =
    this->create_publisher<ken_msgs::msg::MissionTrajectory>("mission_trajectory", 1);

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 1, std::bind(&KenPathPlanner::joint_state_callback, this, _1));

  mission_target_sub_ = this->create_subscription<ken_msgs::msg::MissionTargetArray>(
    "mission_target", 1, std::bind(&KenPathPlanner::mission_target_callback, this, _1));

  std::cout << "Finish Initialize Path Planner" << std::endl;
}

KenPathPlanner::~KenPathPlanner() {}

void KenPathPlanner::pushInterpolateTrajectoryPoints(
  trajectory_msgs::msg::JointTrajectory & jtm,
  const trajectory_msgs::msg::JointTrajectoryPoint start,
  const trajectory_msgs::msg::JointTrajectoryPoint end, int interpolate_num)
{
  rclcpp::Duration duration_btwn_point = end.time_from_start;
  duration_btwn_point = duration_btwn_point - start.time_from_start;

  for (int i = 0; i < interpolate_num; ++i) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start =
      rclcpp::Duration(duration_btwn_point.nanoseconds() / interpolate_num * i) +
      start.time_from_start;
    for (size_t idx = 0; idx < start.positions.size(); ++idx) {
      point.positions.push_back(
        (end.positions[idx] - start.positions[idx]) / interpolate_num * i + start.positions[idx]);
    }
    jtm.points.push_back(point);
  }
  jtm.points.push_back(end);
}

bool KenPathPlanner::makeCurrentPosTrajectory(trajectory_msgs::msg::JointTrajectory & jtm)
{
  jtm.header.stamp = rclcpp::Time(0);
  jtm.joint_names = name_vec_;

  trajectory_msgs::msg::JointTrajectoryPoint start_point;
  start_point.time_from_start = second2duration(0.0);
  start_point.positions = current_pos_;

  jtm.points.push_back(start_point);
  return true;
}

bool KenPathPlanner::makeMoveHomeTrajectory(
  trajectory_msgs::msg::JointTrajectory & jtm, const std::vector<double> & current_pos)
{
  jtm.header.stamp = rclcpp::Time(0);
  jtm.joint_names = name_vec_;

  trajectory_msgs::msg::JointTrajectoryPoint start_point;
  start_point.time_from_start = second2duration(0.0);
  start_point.positions = current_pos;

  trajectory_msgs::msg::JointTrajectoryPoint end_point;
  end_point.time_from_start = second2duration(move_time_);
  end_point.positions = std::vector<double>(6, 0.0);

  pushInterpolateTrajectoryPoints(jtm, start_point, end_point, 100);

  return true;
}

bool KenPathPlanner::makeMoveKamaeTrajectory(
  trajectory_msgs::msg::JointTrajectory & jtm, const std::vector<double> & current_pos)
{
  jtm.header.stamp = rclcpp::Time(0);
  jtm.joint_names = name_vec_;

  trajectory_msgs::msg::JointTrajectoryPoint start_point;
  start_point.time_from_start = second2duration(0.0);
  start_point.positions = current_pos;

  trajectory_msgs::msg::JointTrajectoryPoint end_point;
  end_point.time_from_start = second2duration(move_time_);
  end_point.positions = kamae_pos_;

  pushInterpolateTrajectoryPoints(jtm, start_point, end_point, 100);

  return true;
}

bool KenPathPlanner::makeMenTrajectory(
  trajectory_msgs::msg::JointTrajectory & jtm, const std::vector<double> & current_pos,
  const geometry_msgs::msg::Pose pose)
{
  jtm.header.stamp = rclcpp::Time(0);
  jtm.joint_names = name_vec_;

  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> path_points(
    5, trajectory_msgs::msg::JointTrajectoryPoint());

  path_points[0].time_from_start = second2duration(0.0);
  path_points[1].time_from_start = second2duration(men_time_ * 0.3);
  path_points[2].time_from_start = second2duration(men_time_ * 0.6);
  path_points[3].time_from_start = second2duration(men_time_ * 0.7);
  path_points[4].time_from_start = second2duration(men_time_ * 1.0);

  path_points[0].positions = current_pos;

  Eigen::Vector3d target_pos = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);

  KenIK ik;
  std::vector<double> ik_ans;
  if (ik.calcPositionIK(target_pos, kamae_pos_, ik_ans)) {
    publishIKlog(ik);

    path_points[1].positions = ik_ans;
    path_points[1].positions[3] = 0.0;
    path_points[1].positions[4] = 0.0;

    path_points[2].positions = ik_ans;
    path_points[3].positions = ik_ans;

    path_points[4].positions = path_points[1].positions;

    for (size_t j = 0; j + 1 < path_points.size(); ++j) {
      pushInterpolateTrajectoryPoints(jtm, path_points[j], path_points[j + 1], 100);
    }

  } else {
    return false;
  }

  return true;
}

bool KenPathPlanner::makeRDouTrajectory(
  trajectory_msgs::msg::JointTrajectory & jtm, const std::vector<double> & current_pos,
  const geometry_msgs::msg::Pose pose)
{
  jtm.header.stamp = rclcpp::Time(0);
  jtm.joint_names = name_vec_;

  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> path_points(
    6, trajectory_msgs::msg::JointTrajectoryPoint());

  for (size_t i = 0; i < path_points.size(); ++i) {
    path_points[i].time_from_start =
      second2duration(dou_time_ * (double)i / (double)(path_points.size() - 1));
  }

  path_points[0].positions = current_pos;

  std::vector<Eigen::Vector3d> target_pos;
  target_pos.push_back(
    Eigen::Vector3d(pose.position.x, pose.position.y - 0.1, pose.position.z + 0.15));
  target_pos.push_back(Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z));

  KenIK ik;
  std::vector<std::vector<double>> ik_ans(target_pos.size(), std::vector<double>());
  bool is_ik_ok = true;

  for (size_t i = 0; i < target_pos.size(); ++i) {
    is_ik_ok = ik.calcPositionIK(target_pos[i], rdou_base_pos_, ik_ans[i]);
  }

  if (is_ik_ok) {
    publishIKlog(ik);

    path_points[1].positions = ik_ans[0];
    path_points[1].positions[3] = 0.0;
    path_points[1].positions[4] = rdou_base_pos_[4];

    path_points[2].positions = ik_ans[0];

    path_points[3].positions = ik_ans.back();

    path_points[4].positions = path_points[2].positions;

    path_points[5].positions = path_points[1].positions;

    for (size_t j = 0; j + 1 < path_points.size(); ++j) {
      pushInterpolateTrajectoryPoints(jtm, path_points[j], path_points[j + 1], 100);
    }

    // debug output
    nav_msgs::msg::Path debug_path;
    debug_path.header.frame_id = base_frame_id_;
    debug_path.header.stamp = rclcpp::Time(0);

    for (size_t i = 0; i < path_points.size(); ++i) {
      KenFK fk(path_points[i].positions);
      geometry_msgs::msg::PoseStamped pps;
      pps.header = debug_path.header;
      pps.pose = transform2pose(fk.computeTbe());
      debug_path.poses.push_back(pps);
    }

    path_debug_pub_->publish(debug_path);

  } else {
    return false;
  }

  return true;
}

void KenPathPlanner::publishIKlog(const KenIK & ik)
{
  // Publish IK calculation log
  std::vector<Eigen::Matrix4d> ik_Tbe_log = ik.getIKlogTbe();

  geometry_msgs::msg::PoseArray ik_debug_pose_array;
  ik_debug_pose_array.header.frame_id = base_frame_id_;
  ik_debug_pose_array.header.stamp = rclcpp::Time(0);
  for (size_t i = 0; i < ik_Tbe_log.size(); ++i) {
    geometry_msgs::msg::Pose Tbepose;
    Tbepose.position.x = ik_Tbe_log[i](0, 3);
    Tbepose.position.y = ik_Tbe_log[i](1, 3);
    Tbepose.position.z = ik_Tbe_log[i](2, 3);
    Eigen::Quaterniond Tbe_q(ik_Tbe_log[i].block<3, 3>(0, 0));
    Tbepose.orientation.x = Tbe_q.x();
    Tbepose.orientation.y = Tbe_q.y();
    Tbepose.orientation.z = Tbe_q.z();
    Tbepose.orientation.w = Tbe_q.w();

    ik_debug_pose_array.poses.push_back(Tbepose);
  }
  ik_debug_pose_pub_->publish(ik_debug_pose_array);
}

void KenPathPlanner::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->position.size() == (size_t)joint_num_) {
    current_pos_ = msg->position;
  }
}

void KenPathPlanner::mission_target_callback(const ken_msgs::msg::MissionTargetArray::SharedPtr msg)
{
  ken_msgs::msg::MissionTrajectory mtm;
  mtm.header.frame_id = base_frame_id_;
  mtm.header.stamp = rclcpp::Time(0);
  mtm.plan_result = true;
  mtm.type = msg->type;

  for (size_t i = 0; i < msg->type.size(); ++i) {
    trajectory_msgs::msg::JointTrajectory jt;
    std::string jt_name;
    bool is_planning_succeed_ = true;
    if (msg->type[i] == msg->HOLD) {
      is_planning_succeed_ &= makeCurrentPosTrajectory(jt);
      jt_name = "hold";
    } else if (msg->type[i] == msg->HOME) {
      is_planning_succeed_ &= makeMoveHomeTrajectory(jt, current_pos_);
      jt_name = "home";
    } else if (msg->type[i] == msg->KAMAE) {
      if (mtm.trajectories.empty())
        is_planning_succeed_ &= makeMoveKamaeTrajectory(jt, current_pos_);
      else
        is_planning_succeed_ &=
          makeMoveKamaeTrajectory(jt, mtm.trajectories.back().points.back().positions);
      jt_name = "kamae";
    } else if (msg->type[i] == msg->MEN) {
      if (mtm.trajectories.empty())
        is_planning_succeed_ &= makeMenTrajectory(jt, current_pos_, msg->poses[i]);
      else
        is_planning_succeed_ &=
          makeMenTrajectory(jt, mtm.trajectories.back().points.back().positions, msg->poses[i]);
      jt_name = "men";
    } else if (msg->type[i] == msg->RDOU) {
      if (mtm.trajectories.empty())
        is_planning_succeed_ &= makeRDouTrajectory(jt, current_pos_, msg->poses[i]);
      else
        is_planning_succeed_ &=
          makeRDouTrajectory(jt, mtm.trajectories.back().points.back().positions, msg->poses[i]);
      jt_name = "dou";
    } else {
      is_planning_succeed_ = false;
      break;
    }

    mtm.plan_result &= is_planning_succeed_;
    if (is_planning_succeed_) {
      mtm.trajectories.push_back(jt);
      mtm.type_names.push_back(jt_name);
    }
  }

  // Mission Manager側で後ろから処理していくので反転させておく
  std::reverse(mtm.trajectories.begin(), mtm.trajectories.end());
  std::reverse(mtm.type_names.begin(), mtm.type_names.end());

  mission_trajectory_pub_->publish(mtm);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KenPathPlanner>());
  rclcpp::shutdown();
  return 0;
}