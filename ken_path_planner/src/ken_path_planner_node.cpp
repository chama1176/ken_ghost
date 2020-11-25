#include <cinttypes>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>

#include <chrono>
#include <functional>

#include <eigen3/Eigen/Dense>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "ken_path_planner/ken_fk.hpp"

using std::placeholders::_1;

class KenPathPlanner : public rclcpp::Node
{
public:
  /* 
  * コンストラクタ
  */
  KenPathPlanner();
  /*
  * デストラクタ
  */
  ~KenPathPlanner();

private:
  void makeMoveHomeTrajectory(trajectory_msgs::msg::JointTrajectory & jtm);
  void makeMoveKamaeTrajectory(trajectory_msgs::msg::JointTrajectory & jtm);

  void pushInterpolateTrajectoryPoints(
    trajectory_msgs::msg::JointTrajectory & jtm,
    const trajectory_msgs::msg::JointTrajectoryPoint start,
    const trajectory_msgs::msg::JointTrajectoryPoint end, int interpolate_num);

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr fk_debug_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  int64_t enable_button_;
  int64_t move_home_button_;
  int64_t move_kamae_button_;

  int64_t joint_num_;
  double move_time_;

  std::vector<std::string> name_vec_;
  std::vector<double> current_pos_;
  std::vector<double> kamae_pos_;

  bool received_joint_state_msg_;
  bool sent_disable_msg_;

  KenFK fk_;
};

KenPathPlanner::KenPathPlanner() : Node("ken_path_planner")
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

  RCLCPP_INFO(this->get_logger(), "Teleop enable button: %d", enable_button_);
  RCLCPP_INFO(this->get_logger(), "Move home button: %d", move_home_button_);
  RCLCPP_INFO(this->get_logger(), "Move kamae button: %d", move_kamae_button_);

  RCLCPP_INFO(this->get_logger(), "Move time: %f", move_time_);
  RCLCPP_INFO(this->get_logger(), "Joint num: %d", joint_num_);

  sent_disable_msg_ = false;
  received_joint_state_msg_ = false;

  cmd_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "ken_joint_trajectory_controller/joint_trajectory", 1);
  fk_debug_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("debug/fk_pose", 1);

  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 1, std::bind(&KenPathPlanner::joy_callback, this, _1));
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 1, std::bind(&KenPathPlanner::joint_state_callback, this, _1));

  std::cout << "Finish Initialization" << std::endl;
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

void KenPathPlanner::makeMoveHomeTrajectory(trajectory_msgs::msg::JointTrajectory & jtm)
{
  jtm.header.stamp = rclcpp::Time(0);
  jtm.joint_names = name_vec_;

  trajectory_msgs::msg::JointTrajectoryPoint start_point;
  start_point.time_from_start.sec = 0;
  start_point.time_from_start.nanosec = 0;
  for (size_t i = 0; i < (size_t)joint_num_; ++i) {
    start_point.positions.push_back(current_pos_[i]);
  }
  trajectory_msgs::msg::JointTrajectoryPoint end_point;
  end_point.time_from_start.sec = (uint32_t)move_time_;
  end_point.time_from_start.nanosec =
    (uint32_t)((move_time_ - end_point.time_from_start.sec) * 1e9);
  for (size_t i = 0; i < (size_t)joint_num_; ++i) {
    end_point.positions.push_back(0.0);
  }

  pushInterpolateTrajectoryPoints(jtm, start_point, end_point, 100);
}

void KenPathPlanner::makeMoveKamaeTrajectory(trajectory_msgs::msg::JointTrajectory & jtm)
{
  jtm.header.stamp = rclcpp::Time(0);
  jtm.joint_names = name_vec_;

  trajectory_msgs::msg::JointTrajectoryPoint start_point;
  start_point.time_from_start.sec = 0;
  start_point.time_from_start.nanosec = 0;
  for (size_t i = 0; i < (size_t)joint_num_; ++i) {
    start_point.positions.push_back(current_pos_[i]);
  }
  trajectory_msgs::msg::JointTrajectoryPoint end_point;
  end_point.time_from_start.sec = (uint32_t)move_time_;
  end_point.time_from_start.nanosec =
    (uint32_t)((move_time_ - end_point.time_from_start.sec) * 1e9);
  for (size_t i = 0; i < (size_t)joint_num_; ++i) {
    end_point.positions.push_back(kamae_pos_[i]);
  }

  pushInterpolateTrajectoryPoints(jtm, start_point, end_point, 100);
}

void KenPathPlanner::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (
    received_joint_state_msg_ && enable_button_ >= 0 &&
    static_cast<int>(msg->buttons.size()) > enable_button_ && msg->buttons[enable_button_]) {
    if (
      move_home_button_ >= 0 && static_cast<int>(msg->buttons.size()) > move_home_button_ &&
      msg->buttons[move_home_button_]) {
      auto message = trajectory_msgs::msg::JointTrajectory();
      makeMoveHomeTrajectory(message);
      cmd_pub_->publish(message);
    }
    if (
      move_kamae_button_ >= 0 && static_cast<int>(msg->buttons.size()) > move_kamae_button_ &&
      msg->buttons[move_kamae_button_]) {
      auto message = trajectory_msgs::msg::JointTrajectory();
      makeMoveKamaeTrajectory(message);
      cmd_pub_->publish(message);
    }

    sent_disable_msg_ = false;
  } else {
    if (!sent_disable_msg_) {
      // auto message = trajectory_msgs::msg::JointTrajectory();
      // message.joint_names = name_vec_;
      // cmd_pub_->publish(message);
      sent_disable_msg_ = true;
    }
  }
}

void KenPathPlanner::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->position.size() == (size_t)joint_num_) {
    for (size_t i = 0; i < (size_t)joint_num_; ++i) {
      current_pos_[i] = msg->position[i];
    }
    received_joint_state_msg_ = true;
  }
  Eigen::Matrix4d Tbe = fk_.getTbe(current_pos_);

  Eigen::Quaterniond Tbe_q(Tbe.block<3, 3>(0, 0));
  geometry_msgs::msg::PoseStamped pose_pub;
  pose_pub.header.frame_id = "base_link";
  pose_pub.header.stamp = rclcpp::Time(0);
  pose_pub.pose.position.x = Tbe(0, 3);
  pose_pub.pose.position.y = Tbe(1, 3);
  pose_pub.pose.position.z = Tbe(2, 3);
  pose_pub.pose.orientation.x = Tbe_q.x();
  pose_pub.pose.orientation.y = Tbe_q.y();
  pose_pub.pose.orientation.z = Tbe_q.z();
  pose_pub.pose.orientation.w = Tbe_q.w();

  fk_debug_pub_->publish(pose_pub);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KenPathPlanner>());
  rclcpp::shutdown();
  return 0;
}