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

bool KenPathPlanner::makeMenAfterTrajectory(trajectory_msgs::msg::JointTrajectory & jtm)
{
  jtm.header.stamp = rclcpp::Time(0);
  jtm.joint_names = name_vec_;

  trajectory_msgs::msg::JointTrajectoryPoint start_point;
  start_point.time_from_start = second2duration(0.0);
  start_point.positions = current_pos_;

  trajectory_msgs::msg::JointTrajectoryPoint end_point;
  end_point.time_from_start = second2duration(move_time_);
  end_point.positions = start_point.positions;
  end_point.positions[3] = 0.0;
  end_point.positions[4] = 0.0;

  pushInterpolateTrajectoryPoints(jtm, start_point, end_point, 100);

  return true;
}

bool KenPathPlanner::makeMenTrajectory(
  trajectory_msgs::msg::JointTrajectory & jtm, const std::vector<double> & current_pos,
  const geometry_msgs::msg::Pose pose)
{
  jtm.header.stamp = rclcpp::Time(0);
  jtm.joint_names = name_vec_;

  trajectory_msgs::msg::JointTrajectoryPoint start_point;
  start_point.time_from_start = second2duration(0.0);
  start_point.positions = current_pos;

  trajectory_msgs::msg::JointTrajectoryPoint via_point;
  via_point.time_from_start = second2duration(move_time_ * 1.0 / 3.0);

  trajectory_msgs::msg::JointTrajectoryPoint end_point;
  end_point.time_from_start = second2duration(move_time_ * 2.0 / 3.0);

  Eigen::Vector3d target_pos = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);

  KenIK ik;
  std::vector<double> ik_ans;
  if (ik.calcPositionIK(target_pos, kamae_pos_, ik_ans)) {
    publishIKlog(ik);
    for (size_t i = 0; i < (size_t)joint_num_; ++i) {
      end_point.positions.push_back(ik_ans[i]);
    }

    via_point.positions = end_point.positions;
    via_point.positions[3] = 0.0;
    via_point.positions[4] = 0.0;

    pushInterpolateTrajectoryPoints(jtm, start_point, via_point, 100);
    pushInterpolateTrajectoryPoints(jtm, via_point, end_point, 100);

    // TODO: ほんとは別のtrajectoryにしたほうが良い
    trajectory_msgs::msg::JointTrajectoryPoint back_point;
    back_point = end_point;
    back_point.positions = via_point.positions;
    back_point.time_from_start = second2duration(move_time_ * 3.0 / 3.0);
    pushInterpolateTrajectoryPoints(jtm, end_point, back_point, 100);

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

  trajectory_msgs::msg::JointTrajectoryPoint start_point;
  start_point.time_from_start = second2duration(0.0);
  start_point.positions = current_pos;

  trajectory_msgs::msg::JointTrajectoryPoint via_point;
  via_point.time_from_start = second2duration(move_time_ / 3.0);

  trajectory_msgs::msg::JointTrajectoryPoint end_point;
  end_point.time_from_start = second2duration(move_time_ * 2.0 / 3.0);

  std::vector<Eigen::Vector3d> target_pos;
  target_pos.push_back(
    Eigen::Vector3d(pose.position.x, pose.position.y - 0.1, pose.position.z + 0.05));
  target_pos.push_back(Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z));

  KenIK ik;
  std::vector<std::vector<double>> ik_ans(target_pos.size(), std::vector<double>());
  bool is_ik_ok = true;

  for (size_t i = 0; i < target_pos.size(); ++i) {
    is_ik_ok = ik.calcPositionIK(target_pos[i], rdou_base_pos_, ik_ans[i]);
  }

  if (is_ik_ok) {
    publishIKlog(ik);
    end_point.positions = ik_ans.back();

    via_point.positions = ik_ans.front();
    via_point.positions[3] = 0.0;
    via_point.positions[4] = rdou_base_pos_[4];

    trajectory_msgs::msg::JointTrajectoryPoint back_point;
    back_point.positions = via_point.positions;
    back_point.time_from_start = second2duration(move_time_ * 3 / 3);

    pushInterpolateTrajectoryPoints(jtm, start_point, via_point, 100);
    pushInterpolateTrajectoryPoints(jtm, via_point, end_point, 100);
    pushInterpolateTrajectoryPoints(jtm, end_point, back_point, 100);

    // debug output
    // TODO: Posearray is unused
    geometry_msgs::msg::PoseArray debug_path_pose;
    debug_path_pose.header.frame_id = base_frame_id_;
    debug_path_pose.header.stamp = rclcpp::Time(0);
    KenFK fk(start_point.positions);
    debug_path_pose.poses.push_back(transform2pose(fk.computeTbe()));
    fk = KenFK(via_point.positions);
    debug_path_pose.poses.push_back(transform2pose(fk.computeTbe()));
    fk = KenFK(end_point.positions);
    debug_path_pose.poses.push_back(transform2pose(fk.computeTbe()));

    nav_msgs::msg::Path debug_path;
    debug_path.header = debug_path_pose.header;
    for (size_t i = 0; i < debug_path_pose.poses.size(); ++i) {
      geometry_msgs::msg::PoseStamped pps;
      pps.header = debug_path_pose.header;
      pps.pose = debug_path_pose.poses[i];
      debug_path.poses.push_back(pps);
    }

    path_debug_pose_pub_->publish(debug_path_pose);
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