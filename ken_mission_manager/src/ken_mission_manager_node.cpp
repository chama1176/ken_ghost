#include <cinttypes>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>

#include <chrono>
#include <functional>

#include <eigen3/Eigen/Dense>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"

#include "ken_msgs/msg/mission_target_array.hpp"
#include "ken_msgs/msg/mission_trajectory.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class KenMissionManager : public rclcpp::Node
{
public:
  /* 
  * コンストラクタ
  */
  KenMissionManager();
  /*
  * デストラクタ
  */
  ~KenMissionManager();

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void mission_trajectory_callback(const ken_msgs::msg::MissionTrajectory::SharedPtr msg);
  void timerCallback(void);
  void redTargetCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

  void updateStatus(void);
  void executeMission(void);

  inline bool is_in_range(const int & value, const int & min, const int & max)
  {
    if (value < min) return false;
    if (max < value) return false;
    return true;
  }

  rclcpp::Clock ros_clock_;
  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;
  geometry_msgs::msg::TransformStamped s2b_transform_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_string_pub_;

  rclcpp::Publisher<ken_msgs::msg::MissionTargetArray>::SharedPtr mission_target_pub_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr red_target_sub_;

  rclcpp::Subscription<ken_msgs::msg::MissionTrajectory>::SharedPtr mission_trajectory_sub_;

  enum MissionState {
    WAITING,
    DURING_EXECUTION,
    AUTO_WAITING,
    AUTO_PLANNING,
    AUTO_DURING_EXECUTION,
    TORQUE_DISABLED
  };

  MissionState current_state_;
  ken_msgs::msg::MissionTrajectory recieved_mission_trajectory_;

  geometry_msgs::msg::PoseArray red_target_;

  int64_t enable_button_;
  int64_t move_home_button_;
  int64_t move_kamae_button_;
  int64_t auto_button_;

  bool is_enable_button_pushed_;
  bool is_move_home_button_pushed_;
  bool is_move_kamae_button_pushed_;
  bool is_auto_button_pushed_;

  bool is_target_sent_;
  bool is_cmd_sent_;
  bool is_goal_;

  double move_time_;

  rclcpp::TimerBase::SharedPtr timer_;
};

KenMissionManager::KenMissionManager()
: Node("ken_mission_manager"),
  ros_clock_(RCL_ROS_TIME),
  tf2_buffer_(std::make_shared<rclcpp::Clock>(ros_clock_)),
  tf2_listener_(tf2_buffer_)
{
  this->declare_parameter("enable_button", -1);
  this->get_parameter("enable_button", enable_button_);

  this->declare_parameter("move_home_button", -1);
  this->get_parameter("move_home_button", move_home_button_);
  this->declare_parameter("move_kamae_button", -1);
  this->get_parameter("move_kamae_button", move_kamae_button_);
  this->declare_parameter("auto_button", -1);
  this->get_parameter("auto_button", auto_button_);

  this->declare_parameter("move_time", 0.0);
  this->get_parameter("move_time", move_time_);

  RCLCPP_INFO(this->get_logger(), "Teleop enable button: %d", enable_button_);
  RCLCPP_INFO(this->get_logger(), "Move home button: %d", move_home_button_);
  RCLCPP_INFO(this->get_logger(), "Move kamae button: %d", move_kamae_button_);

  RCLCPP_INFO(this->get_logger(), "Move time: %f", move_time_);

  cmd_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "ken_joint_trajectory_controller/joint_trajectory", 1);
  cmd_string_pub_ = this->create_publisher<std_msgs::msg::String>("ken_cmd_string", 1);
  mission_target_pub_ =
    this->create_publisher<ken_msgs::msg::MissionTargetArray>("mission_target", 1);

  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 1, std::bind(&KenMissionManager::joyCallback, this, _1));
  mission_trajectory_sub_ = this->create_subscription<ken_msgs::msg::MissionTrajectory>(
    "mission_trajectory", 1, std::bind(&KenMissionManager::mission_trajectory_callback, this, _1));
  red_target_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    "red_target", 1, std::bind(&KenMissionManager::redTargetCallback, this, _1));

  try {
    s2b_transform_ = tf2_buffer_.lookupTransform("base_link", "world", tf2::TimePoint(100ms));
  } catch (const tf2::TransformException & e) {
    std::cerr << e.what() << '\n';
  }

  timer_ = this->create_wall_timer(100ms, std::bind(&KenMissionManager::timerCallback, this));

  current_state_ = MissionState::WAITING;
  is_goal_ = true;

  std::cout << "Finish Initialization" << std::endl;
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

  if (is_in_range(auto_button_, 0, msg_buttons_size))
    is_auto_button_pushed_ = msg->buttons[auto_button_];
}

void KenMissionManager::mission_trajectory_callback(
  const ken_msgs::msg::MissionTrajectory::SharedPtr msg)
{
  recieved_mission_trajectory_ = *msg;
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

void KenMissionManager::updateStatus(void)
{
  switch (current_state_) {
    case MissionState::WAITING: {
      if (is_move_home_button_pushed_) {
        recieved_mission_trajectory_.plan_result = false;
        ken_msgs::msg::MissionTargetArray mta;
        mta.header.stamp = rclcpp::Time(0);
        mta.type.push_back(mta.HOME);
        mta.poses.push_back(geometry_msgs::msg::Pose());
        mission_target_pub_->publish(mta);

        current_state_ = MissionState::DURING_EXECUTION;
        RCLCPP_INFO(this->get_logger(), "During execution");
        is_goal_ = true;
      } else if (is_move_kamae_button_pushed_) {
        recieved_mission_trajectory_.plan_result = false;
        ken_msgs::msg::MissionTargetArray mta;
        mta.header.stamp = rclcpp::Time(0);
        mta.type.push_back(mta.KAMAE);
        mta.poses.push_back(geometry_msgs::msg::Pose());
        mission_target_pub_->publish(mta);

        current_state_ = MissionState::DURING_EXECUTION;
        RCLCPP_INFO(this->get_logger(), "During execution");
        is_goal_ = true;
      } else if (is_auto_button_pushed_) {
        recieved_mission_trajectory_.plan_result = false;
        current_state_ = MissionState::AUTO_PLANNING;
        RCLCPP_INFO(this->get_logger(), "Auto");
        is_goal_ = true;
      }

    } break;

    case MissionState::DURING_EXECUTION: {
      if (is_goal_ && recieved_mission_trajectory_.trajectories.empty()) {
        RCLCPP_INFO(this->get_logger(), "Waiting");
        current_state_ = MissionState::WAITING;
      }
    } break;

    case MissionState::AUTO_WAITING: {
    } break;

    case MissionState::AUTO_PLANNING: {
      if (recieved_mission_trajectory_.plan_result) {
        RCLCPP_INFO(this->get_logger(), "Auto exe");
        current_state_ = MissionState::AUTO_DURING_EXECUTION;
      }
    } break;

    case MissionState::AUTO_DURING_EXECUTION: {
      if (is_goal_ && recieved_mission_trajectory_.trajectories.empty()) {
        RCLCPP_INFO(this->get_logger(), "Waiting");
        current_state_ = MissionState::WAITING;
      }
    } break;

    case MissionState::TORQUE_DISABLED: {
    } break;

    default:
      break;
  }
}

void KenMissionManager::executeMission(void)
{
  switch (current_state_) {
    case MissionState::WAITING: {
      // do nothing
    } break;

    case MissionState::DURING_EXECUTION: {
      if (
        is_goal_ && recieved_mission_trajectory_.plan_result &&
        !recieved_mission_trajectory_.trajectories.empty()) {
        auto cmd_string = std_msgs::msg::String();
        cmd_string.data = recieved_mission_trajectory_.type_names.back();
        cmd_string_pub_->publish(cmd_string);
        cmd_pub_->publish(recieved_mission_trajectory_.trajectories.back());
        recieved_mission_trajectory_.trajectories.pop_back();
        recieved_mission_trajectory_.type_names.pop_back();
        is_goal_ = false;
        is_goal_ = true;
      }
    } break;

    case MissionState::AUTO_WAITING: {
    } break;

    case MissionState::AUTO_PLANNING: {
      // TODO: Change push target
      // TODO: Check target range
      recieved_mission_trajectory_.plan_result = false;
      ken_msgs::msg::MissionTargetArray mta;
      mta.header.stamp = rclcpp::Time(0);
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
      mission_target_pub_->publish(mta);
    } break;

    case MissionState::AUTO_DURING_EXECUTION: {
      if (
        is_goal_ && recieved_mission_trajectory_.plan_result &&
        !recieved_mission_trajectory_.trajectories.empty()) {
        auto cmd_string = std_msgs::msg::String();
        cmd_string.data = recieved_mission_trajectory_.type_names.back();
        cmd_string_pub_->publish(cmd_string);
        cmd_pub_->publish(recieved_mission_trajectory_.trajectories.back());
        recieved_mission_trajectory_.trajectories.pop_back();
        recieved_mission_trajectory_.type_names.pop_back();
        is_goal_ = false;
        is_goal_ = true;
      }
    } break;

    case MissionState::TORQUE_DISABLED: {
    } break;

    default:
      break;
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KenMissionManager>());
  rclcpp::shutdown();
  return 0;
}