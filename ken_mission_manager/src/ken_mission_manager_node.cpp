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
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

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
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr cmd_pub_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  int64_t enable_button_;
  int64_t move_home_button_;
  int64_t move_kamae_button_;

  int64_t joint_num_;
  double move_time_;

  bool received_joint_state_msg_;
  bool sent_disable_msg_;
};

KenMissionManager::KenMissionManager() : Node("ken_mission_manager")
{
  this->declare_parameter("enable_button", -1);
  this->get_parameter("enable_button", enable_button_);

  this->declare_parameter("move_home_button", -1);
  this->get_parameter("move_home_button", move_home_button_);
  this->declare_parameter("move_kamae_button", -1);
  this->get_parameter("move_kamae_button", move_kamae_button_);

  this->declare_parameter("joint_num", -1);
  this->get_parameter("joint_num", joint_num_);

  this->declare_parameter("move_time", 0.0);
  this->get_parameter("move_time", move_time_);

  this->declare_parameter("ik_test_pos_x", 0.0);
  this->declare_parameter("ik_test_pos_y", 0.0);
  this->declare_parameter("ik_test_pos_z", 0.0);

  RCLCPP_INFO(this->get_logger(), "Teleop enable button: %d", enable_button_);
  RCLCPP_INFO(this->get_logger(), "Move home button: %d", move_home_button_);
  RCLCPP_INFO(this->get_logger(), "Move kamae button: %d", move_kamae_button_);

  RCLCPP_INFO(this->get_logger(), "Move time: %f", move_time_);
  RCLCPP_INFO(this->get_logger(), "Joint num: %d", joint_num_);

  sent_disable_msg_ = false;
  received_joint_state_msg_ = false;

  cmd_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "ken_joint_trajectory_controller/joint_trajectory", 1);
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 1, std::bind(&KenMissionManager::joy_callback, this, _1));

  std::cout << "Finish Initialization" << std::endl;
}

KenMissionManager::~KenMissionManager() {}

void KenMissionManager::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KenMissionManager>());
  rclcpp::shutdown();
  return 0;
}