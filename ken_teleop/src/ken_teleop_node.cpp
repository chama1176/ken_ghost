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
  double getVal(const sensor_msgs::msg::Joy::SharedPtr joy_msg, const int64_t joint_index);

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  int64_t enable_button_;
  int64_t joint_num_;
  double move_time_;

  std::vector<int64_t> axix_vec_;
  std::vector<double> scale_vec_;
  std::vector<std::string> name_vec_;
  std::vector<double> current_pos_;

  bool received_joint_state_msg_;
  bool sent_disable_msg_;
};

KenTeleopJoy::KenTeleopJoy() : Node("ken_teleop_joy")
{
  cmd_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "ken_joint_trajectory_controller/joint_trajectory", 1);
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 1, std::bind(&KenTeleopJoy::joy_callback, this, _1));
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 1, std::bind(&KenTeleopJoy::joint_state_callback, this, _1));

  this->declare_parameter("enable_button", -1);
  this->get_parameter("enable_button", enable_button_);

  this->declare_parameter("joint_num", -1);
  this->get_parameter("joint_num", joint_num_);

  this->declare_parameter("move_time", 0.0);
  this->get_parameter("move_time", move_time_);

  if (joint_num_ > 0) {
    this->declare_parameter("axis", std::vector<int64_t>(joint_num_, -1));
    this->get_parameter("axis", axix_vec_);

    this->declare_parameter("scale", std::vector<double>(joint_num_, 0.0));
    this->get_parameter("scale", scale_vec_);

    this->declare_parameter("name", std::vector<std::string>(joint_num_, ""));
    this->get_parameter("name", name_vec_);
  }

  RCLCPP_INFO(this->get_logger(), "Teleop enable button: %d", enable_button_);
  RCLCPP_INFO(this->get_logger(), "Move time: %f", move_time_);

  for (size_t i = 0; i < (size_t)joint_num_; ++i) {
    RCLCPP_INFO(
      this->get_logger(), "Axis %d %s on axis %d at scale %f", i, name_vec_[i].c_str(),
      axix_vec_[i], scale_vec_[i]);
  }

  current_pos_.resize(joint_num_);
  sent_disable_msg_ = false;
  received_joint_state_msg_ = false;

  std::cout << "Finish Initialization" << std::endl;
}

KenTeleopJoy::~KenTeleopJoy() {}

double KenTeleopJoy::getVal(
  const sensor_msgs::msg::Joy::SharedPtr joy_msg, const int64_t joint_index)
{
  if (joy_msg->axes.size() <= (size_t)axix_vec_[joint_index] || axix_vec_[joint_index] < 0) {
    return current_pos_[joint_index];
  }

  return joy_msg->axes[axix_vec_[joint_index]] * scale_vec_[joint_index] +
         current_pos_[joint_index];
}

void KenTeleopJoy::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (
    received_joint_state_msg_ && enable_button_ >= 0 &&
    static_cast<int>(msg->buttons.size()) > enable_button_ && msg->buttons[enable_button_]) {
    auto message = trajectory_msgs::msg::JointTrajectory();
    message.header.stamp = rclcpp::Time(0);

    message.joint_names = name_vec_;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start.sec = (uint32_t)move_time_;
    point.time_from_start.nanosec = (uint32_t)(move_time_ * 1e9);
    for (size_t i = 0; i < (size_t)joint_num_; ++i) {
      point.positions.push_back(getVal(msg, i));
    }
    message.points.push_back(point);

    cmd_pub_->publish(message);
    sent_disable_msg_ = false;
  } else {
    if (!sent_disable_msg_) {
      auto message = trajectory_msgs::msg::JointTrajectory();
      message.joint_names = name_vec_;
      cmd_pub_->publish(message);
      sent_disable_msg_ = true;
    }
  }
}

void KenTeleopJoy::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->position.size() == (size_t)joint_num_) {
    for (size_t i = 0; i < (size_t)joint_num_; ++i) {
      current_pos_[i] = msg->position[i];
    }
    received_joint_state_msg_ = true;
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KenTeleopJoy>());
  rclcpp::shutdown();
  return 0;
}