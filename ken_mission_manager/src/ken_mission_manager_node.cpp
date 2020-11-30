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
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

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
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void timer_callback(void);

  inline bool is_in_range(const int & value, const int & min, const int & max)
  {
    if (value < min) return false;
    if (max < value) return false;
    return true;
  }

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_string_pub_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  int64_t enable_button_;
  int64_t move_home_button_;
  int64_t move_kamae_button_;

  bool enable_button_pushed_;
  bool move_home_button_pushed_;
  bool move_kamae_button_pushed_;

  double move_time_;

  rclcpp::TimerBase::SharedPtr timer_;
};

KenMissionManager::KenMissionManager() : Node("ken_mission_manager")
{
  this->declare_parameter("enable_button", -1);
  this->get_parameter("enable_button", enable_button_);

  this->declare_parameter("move_home_button", -1);
  this->get_parameter("move_home_button", move_home_button_);
  this->declare_parameter("move_kamae_button", -1);
  this->get_parameter("move_kamae_button", move_kamae_button_);

  this->declare_parameter("move_time", 0.0);
  this->get_parameter("move_time", move_time_);

  RCLCPP_INFO(this->get_logger(), "Teleop enable button: %d", enable_button_);
  RCLCPP_INFO(this->get_logger(), "Move home button: %d", move_home_button_);
  RCLCPP_INFO(this->get_logger(), "Move kamae button: %d", move_kamae_button_);

  RCLCPP_INFO(this->get_logger(), "Move time: %f", move_time_);

  cmd_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "ken_joint_trajectory_controller/joint_trajectory", 1);
  cmd_string_pub_ = this->create_publisher<std_msgs::msg::String>("ken_cmd_string", 1);

  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 1, std::bind(&KenMissionManager::joy_callback, this, _1));

  timer_ = this->create_wall_timer(100ms, std::bind(&KenMissionManager::timer_callback, this));

  std::cout << "Finish Initialization" << std::endl;
}

KenMissionManager::~KenMissionManager() {}

void KenMissionManager::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  int msg_buttons_size = static_cast<int>(msg->buttons.size());

  if (is_in_range(enable_button_, 0, msg_buttons_size))
    enable_button_pushed_ = msg->buttons[enable_button_];

  if (is_in_range(move_home_button_, 0, msg_buttons_size))
    move_home_button_pushed_ = msg->buttons[move_home_button_];

  if (is_in_range(move_kamae_button_, 0, msg_buttons_size))
    move_kamae_button_pushed_ = msg->buttons[move_kamae_button_];
}

void KenMissionManager::timer_callback(void)
{
  std_msgs::msg::String cmd_string;
  cmd_string.data = "men";
  cmd_string_pub_->publish(cmd_string);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KenMissionManager>());
  rclcpp::shutdown();
  return 0;
}