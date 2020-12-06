#include <cinttypes>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>

#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

using std::placeholders::_1;

class KenGoalChecker : public rclcpp::Node
{
public:
  /* 
  * コンストラクタ
  */
  KenGoalChecker();
  /*
  * デストラクタ
  */
  ~KenGoalChecker();

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void jointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_state_pub_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_sub_;

  trajectory_msgs::msg::JointTrajectory last_joint_trajectory_;
  double goal_thes_;
};

KenGoalChecker::KenGoalChecker() : Node("ken_goal_checker_node")
{
  goal_state_pub_ = this->create_publisher<std_msgs::msg::Bool>("goal_status", 1);

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 1, std::bind(&KenGoalChecker::jointStateCallback, this, _1));
  joint_trajectory_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "ken_joint_trajectory_controller/joint_trajectory", 1,
    std::bind(&KenGoalChecker::jointTrajectoryCallback, this, _1));

  goal_thes_ = 10.0 * M_PI / 180.0;
  last_joint_trajectory_ = trajectory_msgs::msg::JointTrajectory();

  std::cout << "Finish Initialization" << std::endl;
}

KenGoalChecker::~KenGoalChecker() {}

void KenGoalChecker::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std_msgs::msg::Bool is_goal;
  is_goal.data = true;

  if (
    !last_joint_trajectory_.points.empty() && !msg->position.empty() &&
    last_joint_trajectory_.points.back().positions.size() == msg->position.size()) {
    for (size_t i = 0; i < msg->position.size(); ++i) {
      bool is_in_range =
        std::abs(msg->position[i] - last_joint_trajectory_.points.back().positions[i]) < goal_thes_;
      is_goal.data &= is_in_range;
    }

  } else {
    is_goal.data = false;
  }

  goal_state_pub_->publish(is_goal);
}

void KenGoalChecker::jointTrajectoryCallback(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  last_joint_trajectory_ = *msg;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KenGoalChecker>());
  rclcpp::shutdown();
  return 0;
}