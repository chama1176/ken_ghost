#include <cinttypes>
#include <functional>
#include <memory>
#include <set>
#include <string>

#include <chrono>
#include <functional>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"

#include "ken_mission_manager/ken_mission_manager.hpp"
#include "ken_msgs/msg/mission_target_array.hpp"
#include "ken_msgs/msg/mission_trajectory.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

KenMissionManager::KenMissionManager()
: Node("ken_mission_manager"),
  clock_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)),
  tf2_buffer_(clock_),
  tf2_listener_(tf2_buffer_),
  last_auto_finish_time_(),
  is_plan_received_(false)
{
  this->declare_parameter("enable_button", -1);
  this->get_parameter("enable_button", enable_button_);

  this->declare_parameter("move_home_button", -1);
  this->get_parameter("move_home_button", move_home_button_);
  this->declare_parameter("move_kamae_button", -1);
  this->get_parameter("move_kamae_button", move_kamae_button_);

  this->declare_parameter("men_button", -1);
  this->get_parameter("men_button", men_button_);
  this->declare_parameter("dou_button", -1);
  this->get_parameter("dou_button", dou_button_);

  this->declare_parameter("auto_button", -1);
  this->get_parameter("auto_button", auto_button_);

  this->declare_parameter("cancel_button", -1);
  this->get_parameter("cancel_button", cancel_button_);

  this->declare_parameter("move_time", 0.0);
  this->get_parameter("move_time", move_time_);

  RCLCPP_INFO(this->get_logger(), "Teleop enable button: %d", enable_button_);
  RCLCPP_INFO(this->get_logger(), "Move home button: %d", move_home_button_);
  RCLCPP_INFO(this->get_logger(), "Move kamae button: %d", move_kamae_button_);

  RCLCPP_INFO(this->get_logger(), "Men button: %d", men_button_);
  RCLCPP_INFO(this->get_logger(), "Dou button: %d", dou_button_);
  RCLCPP_INFO(this->get_logger(), "Auto button: %d", auto_button_);
  RCLCPP_INFO(this->get_logger(), "Cancel button: %d", cancel_button_);

  RCLCPP_INFO(this->get_logger(), "Move time: %f", move_time_);

  cmd_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "ken_joint_trajectory_controller/joint_trajectory", 1);
  cmd_string_pub_ = this->create_publisher<std_msgs::msg::String>("ken_cmd_string", 1);
  mission_target_pub_ =
    this->create_publisher<ken_msgs::msg::MissionTargetArray>("mission_target", 1);

  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 1, std::bind(&KenMissionManager::joyCallback, this, _1));
  red_target_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    "red_target", 1, std::bind(&KenMissionManager::redTargetCallback, this, _1));
  blue_target_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    "blue_target", 1, std::bind(&KenMissionManager::blueTargetCallback, this, _1));
  yellow_target_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    "yellow_target", 1, std::bind(&KenMissionManager::yellowTargetCallback, this, _1));
  green_target_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    "green_target", 1, std::bind(&KenMissionManager::greenTargetCallback, this, _1));

  goal_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "goal_status", 1, std::bind(&KenMissionManager::goalStatusCallback, this, _1));

  mission_trajectory_sub_ = this->create_subscription<ken_msgs::msg::MissionTrajectory>(
    "mission_trajectory", 1, std::bind(&KenMissionManager::mission_trajectory_callback, this, _1));

  try {
    s2b_transform_ = tf2_buffer_.lookupTransform(
      "base_link", "camera_color_optical_frame", tf2::TimePoint(), tf2::durationFromSec(2.0));
    RCLCPP_INFO(this->get_logger(), "Got transform");
  } catch (const tf2::TransformException & e) {
    std::cerr << e.what() << '\n';
  }

  timer_ = this->create_wall_timer(100ms, std::bind(&KenMissionManager::timerCallback, this));

  setMissionState(MissionState::WAITING);
  publishHoldMissionTrajectory();

  std::cout << "Finish Initialize Mission Manager" << std::endl;
}

KenMissionManager::~KenMissionManager() {}

void KenMissionManager::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  int msg_buttons_size = static_cast<int>(msg->buttons.size());

  if (is_in_range(enable_button_, 0, msg_buttons_size))
    is_enable_button_pushed_ = msg->buttons[enable_button_];

  if (is_in_range(move_home_button_, 0, msg_buttons_size))
    is_move_home_button_pushed_ = msg->buttons[move_home_button_];

  if (is_in_range(move_kamae_button_, 0, msg_buttons_size))
    is_move_kamae_button_pushed_ = msg->buttons[move_kamae_button_];

  if (is_in_range(men_button_, 0, msg_buttons_size))
    is_men_button_pushed_ = msg->buttons[men_button_];

  if (is_in_range(dou_button_, 0, msg_buttons_size))
    is_dou_button_pushed_ = msg->buttons[dou_button_];

  if (is_in_range(auto_button_, 0, msg_buttons_size))
    is_auto_button_pushed_ = msg->buttons[auto_button_];

  if (is_in_range(cancel_button_, 0, msg_buttons_size))
    is_cancel_button_pushed_ = msg->buttons[cancel_button_];
}

void KenMissionManager::mission_trajectory_callback(
  const ken_msgs::msg::MissionTrajectory::SharedPtr msg)
{
  recieved_mission_trajectory_ = *msg;
  is_plan_received_ = true;
}

void KenMissionManager::timerCallback(void)
{
  updateStatus();
  executeMission();
}

void KenMissionManager::redTargetCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  red_target_ = *msg;
}
void KenMissionManager::blueTargetCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  blue_target_ = *msg;
}
void KenMissionManager::yellowTargetCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  yellow_target_ = *msg;
}
void KenMissionManager::greenTargetCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  green_target_ = *msg;
}

void KenMissionManager::goalStatusCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  is_goal_ = msg->data;
}

void KenMissionManager::setMissionState(const MissionState & st)
{
  current_state_ = st;
  RCLCPP_INFO(this->get_logger(), "%s ", enum_mission_state_map.at(st).c_str());
  std::cout << enum_mission_state_map.at(st).c_str() << std::endl;
}

void KenMissionManager::publishMissionTarget(ken_msgs::msg::MissionTargetArray & mta)
{
  mta.header.stamp = rclcpp::Time(0);
  mission_target_pub_->publish(mta);
  is_plan_received_ = false;
}

void KenMissionManager::updateStatus(void)
{
  switch (current_state_) {
    case MissionState::WAITING: {
      updateStatusWaiting();
    } break;

    case MissionState::DURING_EXECUTION: {
      if (is_cancel_button_pushed_) {
        setMissionState(MissionState::WAITING);
      } else if (
        is_goal_ && is_plan_received_ && recieved_mission_trajectory_.trajectories.empty()) {
        setMissionState(MissionState::WAITING);
        publishHoldMissionTrajectory();
      } else if (is_plan_received_ && recieved_mission_trajectory_.plan_result == false) {
        setMissionState(MissionState::WAITING);
      }
    } break;

    case MissionState::AUTO_WAITING: {
      rclcpp::Duration elasped = clock_->now() - last_auto_finish_time_;
      std::cout << elasped.seconds() << "." << elasped.nanoseconds() << std::endl;
      if (is_cancel_button_pushed_) {
        setMissionState(MissionState::WAITING);
      } else if (elasped.seconds() >= 2) {
        setMissionState(MissionState::AUTO_PLANNING);
      }
    } break;

    case MissionState::AUTO_PLANNING: {
      if (is_cancel_button_pushed_) {
        setMissionState(MissionState::WAITING);
      } else if (is_plan_received_ && recieved_mission_trajectory_.plan_result) {
        setMissionState(MissionState::AUTO_DURING_EXECUTION);
      }
    } break;

    case MissionState::AUTO_DURING_EXECUTION: {
      if (is_cancel_button_pushed_) {
        setMissionState(MissionState::WAITING);
      } else if (
        is_goal_ && is_plan_received_ && recieved_mission_trajectory_.trajectories.empty()) {
        setMissionState(MissionState::AUTO_WAITING);
        last_auto_finish_time_ = clock_->now();
        publishHoldMissionTrajectory();
      }
    } break;

    case MissionState::TORQUE_DISABLED: {
    } break;

    default:
      break;
  }
}
void KenMissionManager::publishHoldMissionTrajectory(void)
{
  ken_msgs::msg::MissionTargetArray mta;
  mta.header.stamp = rclcpp::Time(0);
  mta.type.push_back(mta.HOLD);
  mta.poses.push_back(geometry_msgs::msg::Pose());
  mission_target_pub_->publish(mta);
}

void KenMissionManager::updateStatusWaiting(void)
{
  if (is_move_home_button_pushed_) {
    // TODO: きれいにしたい
    ken_msgs::msg::MissionTargetArray mta;
    mta.type.push_back(mta.HOME);
    mta.poses.push_back(geometry_msgs::msg::Pose());
    publishMissionTarget(mta);
    setMissionState(MissionState::DURING_EXECUTION);
  } else if (is_move_kamae_button_pushed_) {
    ken_msgs::msg::MissionTargetArray mta;
    mta.type.push_back(mta.KAMAE);
    mta.poses.push_back(geometry_msgs::msg::Pose());
    publishMissionTarget(mta);
    setMissionState(MissionState::DURING_EXECUTION);
  } else if (is_men_button_pushed_) {
    ken_msgs::msg::MissionTargetArray mta;
    mta.type.push_back(mta.KAMAE);
    mta.poses.push_back(geometry_msgs::msg::Pose());
    if (!red_target_.poses.empty()) {
      geometry_msgs::msg::PoseStamped target_transformed;
      geometry_msgs::msg::PoseStamped target_pose;
      target_pose.pose = red_target_.poses.front();
      target_pose.header = red_target_.header;
      tf2::doTransform(target_pose, target_transformed, s2b_transform_);
      mta.type.push_back(mta.MEN);
      mta.poses.push_back(target_transformed.pose);
    }
    mta.type.push_back(mta.KAMAE);
    mta.poses.push_back(geometry_msgs::msg::Pose());
    publishMissionTarget(mta);
    setMissionState(MissionState::DURING_EXECUTION);
  } else if (is_dou_button_pushed_) {
    ken_msgs::msg::MissionTargetArray mta;
    mta.type.push_back(mta.KAMAE);
    mta.poses.push_back(geometry_msgs::msg::Pose());
    if (!blue_target_.poses.empty()) {
      geometry_msgs::msg::PoseStamped target_transformed;
      geometry_msgs::msg::PoseStamped target_pose;
      target_pose.pose = blue_target_.poses.front();
      target_pose.header = blue_target_.header;
      tf2::doTransform(target_pose, target_transformed, s2b_transform_);
      mta.type.push_back(mta.RDOU);
      mta.poses.push_back(target_transformed.pose);
    }
    mta.type.push_back(mta.KAMAE);
    mta.poses.push_back(geometry_msgs::msg::Pose());
    publishMissionTarget(mta);
    setMissionState(MissionState::DURING_EXECUTION);
  } else if (is_auto_button_pushed_) {
    setMissionState(MissionState::AUTO_WAITING);
  } else if (!green_target_.poses.empty()) {
    setMissionState(MissionState::AUTO_PLANNING);
  }
}

void KenMissionManager::executeMission(void)
{
  switch (current_state_) {
    case MissionState::WAITING: {
      // do nothing
      if (
        recieved_mission_trajectory_.plan_result &&
        !recieved_mission_trajectory_.trajectories.empty()) {
        auto cmd_string = std_msgs::msg::String();
        cmd_string.data = recieved_mission_trajectory_.type_names.back();
        cmd_string_pub_->publish(cmd_string);
        cmd_pub_->publish(recieved_mission_trajectory_.trajectories.back());
        recieved_mission_trajectory_.trajectories.pop_back();
        recieved_mission_trajectory_.type_names.pop_back();
      }
    } break;

    case MissionState::DURING_EXECUTION: {
      if (
        is_goal_ && is_plan_received_ && recieved_mission_trajectory_.plan_result &&
        !recieved_mission_trajectory_.trajectories.empty()) {
        auto cmd_string = std_msgs::msg::String();
        cmd_string.data = recieved_mission_trajectory_.type_names.back();
        cmd_string_pub_->publish(cmd_string);
        cmd_pub_->publish(recieved_mission_trajectory_.trajectories.back());
        recieved_mission_trajectory_.trajectories.pop_back();
        recieved_mission_trajectory_.type_names.pop_back();
      }
    } break;

    case MissionState::AUTO_WAITING: {
      // do nothing
    } break;

    case MissionState::AUTO_PLANNING: {
      execAutoPlanning();
    } break;

    case MissionState::AUTO_DURING_EXECUTION: {
      if (
        // TODO:should pub lish trajectory multiple time
        is_goal_ && recieved_mission_trajectory_.plan_result &&
        !recieved_mission_trajectory_.trajectories.empty()) {
        auto cmd_string = std_msgs::msg::String();
        cmd_string.data = recieved_mission_trajectory_.type_names.back();
        cmd_string_pub_->publish(cmd_string);
        cmd_pub_->publish(recieved_mission_trajectory_.trajectories.back());
        recieved_mission_trajectory_.trajectories.pop_back();
        recieved_mission_trajectory_.type_names.pop_back();
        is_goal_ = false;
      }
    } break;

    case MissionState::TORQUE_DISABLED: {
    } break;

    default:
      break;
  }
}

void KenMissionManager::execAutoPlanning(void)
{
  // TODO: Change push target
  // TODO: Check target range
  if (is_plan_received_) {
    ken_msgs::msg::MissionTargetArray mta;
    mta.type.push_back(mta.KAMAE);
    mta.poses.push_back(geometry_msgs::msg::Pose());
    if (!red_target_.poses.empty()) {
      geometry_msgs::msg::PoseStamped target_transformed;
      geometry_msgs::msg::PoseStamped target_pose;
      target_pose.pose = red_target_.poses.front();
      target_pose.header = red_target_.header;
      tf2::doTransform(target_pose, target_transformed, s2b_transform_);
      mta.type.push_back(mta.MEN);
      mta.poses.push_back(target_transformed.pose);
    }
    if (!blue_target_.poses.empty()) {
      geometry_msgs::msg::PoseStamped target_transformed;
      geometry_msgs::msg::PoseStamped target_pose;
      target_pose.pose = blue_target_.poses.front();
      target_pose.header = blue_target_.header;
      tf2::doTransform(target_pose, target_transformed, s2b_transform_);
      mta.type.push_back(mta.RDOU);
      mta.poses.push_back(target_transformed.pose);
    }
    if (!red_target_.poses.empty()) {
      geometry_msgs::msg::PoseStamped target_transformed;
      geometry_msgs::msg::PoseStamped target_pose;
      target_pose.pose = red_target_.poses.front();
      target_pose.header = red_target_.header;
      tf2::doTransform(target_pose, target_transformed, s2b_transform_);
      mta.type.push_back(mta.MEN);
      mta.poses.push_back(target_transformed.pose);
    }
    if (!blue_target_.poses.empty()) {
      geometry_msgs::msg::PoseStamped target_transformed;
      geometry_msgs::msg::PoseStamped target_pose;
      target_pose.pose = blue_target_.poses.front();
      target_pose.header = blue_target_.header;
      tf2::doTransform(target_pose, target_transformed, s2b_transform_);
      mta.type.push_back(mta.RDOU);
      mta.poses.push_back(target_transformed.pose);
    }

    mta.type.push_back(mta.KAMAE);
    mta.poses.push_back(geometry_msgs::msg::Pose());
    publishMissionTarget(mta);
  } else {
    // Wait for planning finished
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KenMissionManager>());
  rclcpp::shutdown();
  return 0;
}