#include <cinttypes>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>

#include <chrono>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

using std::placeholders::_1;

class KenTeleopJoy : public rclcpp::Node
{
public:
  /* 
  * コンストラクタ
  */
  KenTeleopJoy();
  /*
  * デストラクタ
  */
  ~KenTeleopJoy();

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  int64_t enable_button_;
  int64_t joint_num_;
  std::vector<int64_t> axix_vec_;
  std::vector<double> scale_vec_;
};

KenTeleopJoy::KenTeleopJoy() : Node("ken_teleop_joy")
{
  cmd_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "ken_joint_trajectory_controller/joint_trajectory", 1);
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 1, std::bind(&KenTeleopJoy::joy_callback, this, _1));

  this->declare_parameter("enable_button", -1);
  this->get_parameter("enable_button", enable_button_);

  this->declare_parameter("joint_num", -1);
  this->get_parameter("joint_num", joint_num_);

  if (joint_num_ > 0) {
    this->declare_parameter("axis", std::vector<int64_t>(joint_num_, -1));
    this->get_parameter("axis", axix_vec_);

    this->declare_parameter("scale", std::vector<double>(joint_num_, 0.0));
    this->get_parameter("scale", scale_vec_);
  }

  RCLCPP_INFO(this->get_logger(), "Teleop enable button: %d", enable_button_);

  for (size_t i = 0; i < joint_num_; ++i) {
    RCLCPP_INFO(this->get_logger(), "Axis %d on %d at scale %f", i, axix_vec_[i], scale_vec_[i]);
  }

  std::cout << "Finish Initialization" << std::endl;
}

KenTeleopJoy::~KenTeleopJoy() {}

void KenTeleopJoy::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {}

void KenTeleopJoy::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KenTeleopJoy>());
  rclcpp::shutdown();
  return 0;
}