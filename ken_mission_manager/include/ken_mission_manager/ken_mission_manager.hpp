#ifndef KEN_MISSION_MANAGER_HPP_
#define KEN_MISSION_MANAGER_HPP_

#include <cinttypes>
#include <functional>
#include <map>
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
  enum MissionState {
    WAITING,
    DURING_EXECUTION,
    AUTO_WAITING,
    AUTO_PLANNING,
    AUTO_DURING_EXECUTION,
    TORQUE_DISABLED
  };
  const std::map<MissionState, std::string> enum_mission_state_map{
    {WAITING, "WAITING"},
    {DURING_EXECUTION, "DURING_EXECUTION"},
    {AUTO_WAITING, "AUTO_WAITING"},
    {AUTO_PLANNING, "AUTO_PLANNING"},
    {AUTO_DURING_EXECUTION, "AUTO_DURING_EXECUTION"},
    {TORQUE_DISABLED, "TORQUE_DISABLED"}};

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void mission_trajectory_callback(const ken_msgs::msg::MissionTrajectory::SharedPtr msg);
  void timerCallback(void);
  void redTargetCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void blueTargetCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void yellowTargetCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void greenTargetCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

  void goalStatusCallback(const std_msgs::msg::Bool::SharedPtr msg);

  void setMissionState(const MissionState & st);
  void publishMissionTarget(ken_msgs::msg::MissionTargetArray & mta);

  void updateStatus(void);
  void executeMission(void);

  void publishHoldMissionTrajectory(void);
  void updateStatusWaiting(void);

  void execAutoPlanning(void);

  inline bool is_in_range(const int & value, const int & min, const int & max)
  {
    if (value < min) return false;
    if (max < value) return false;
    return true;
  }

  rclcpp::Clock::SharedPtr clock_;
  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;
  geometry_msgs::msg::TransformStamped s2b_transform_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_string_pub_;

  rclcpp::Publisher<ken_msgs::msg::MissionTargetArray>::SharedPtr mission_target_pub_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr red_target_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr blue_target_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr yellow_target_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr green_target_sub_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_status_sub_;
  rclcpp::Subscription<ken_msgs::msg::MissionTrajectory>::SharedPtr mission_trajectory_sub_;

  MissionState current_state_;
  ken_msgs::msg::MissionTrajectory recieved_mission_trajectory_;
  rclcpp::Time last_auto_finish_time_;

  geometry_msgs::msg::PoseArray red_target_;
  geometry_msgs::msg::PoseArray blue_target_;
  geometry_msgs::msg::PoseArray yellow_target_;
  geometry_msgs::msg::PoseArray green_target_;

  int64_t enable_button_;
  int64_t move_home_button_;
  int64_t move_kamae_button_;
  int64_t men_button_;
  int64_t dou_button_;
  int64_t auto_button_;
  int64_t cancel_button_;

  bool is_enable_button_pushed_;
  bool is_move_home_button_pushed_;
  bool is_move_kamae_button_pushed_;
  bool is_men_button_pushed_;
  bool is_dou_button_pushed_;
  bool is_auto_button_pushed_;
  bool is_cancel_button_pushed_;

  bool is_plan_received_;
  bool is_goal_;

  double move_time_;

  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // KEN_MISSION_MANAGER_HPP_