#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/rclcpp.hpp"

#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;

class JTMPublisher : public rclcpp::Node
{
public:
  /* 
  * コンストラクタ
  */
  JTMPublisher();
  /*
  * デストラクタ
  */
  ~JTMPublisher();

private:
  void timer_callback();
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

JTMPublisher::JTMPublisher() : Node("pub_test_subscriber")
{
  publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/ken_joint_trajectory_controller/joint_trajectory", 10);
  timer_ = this->create_wall_timer(500ms, std::bind(&JTMPublisher::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "Hello");
  std::cout << "Hello world" << std::endl;
}

JTMPublisher::~JTMPublisher() {}

void JTMPublisher::timer_callback()
{
  auto message = trajectory_msgs::msg::JointTrajectory();
  message.header.stamp = rclcpp::Time(0);

  message.joint_names.push_back("arm_base_to_arm_link1");
  message.joint_names.push_back("arm_link1_to_arm_link2");
  message.joint_names.push_back("arm_link2_to_arm_link3");
  message.joint_names.push_back("arm_link3_to_arm_link4");
  message.joint_names.push_back("arm_link4_to_shinai_link");

  for (size_t i = 0; i < 1; ++i) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start.sec = 1;
    point.time_from_start.nanosec = 1;
    point.positions.push_back(1.3);
    point.positions.push_back(2.3);
    point.positions.push_back(3.3);
    point.positions.push_back(4.3);
    point.positions.push_back(5.3);
    message.points.push_back(point);
  }
  for (size_t i = 0; i < 1; ++i) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start.sec = 2;
    point.time_from_start.nanosec = 2;
    point.positions.push_back(2.3);
    point.positions.push_back(2.3);
    point.positions.push_back(2.3);
    point.positions.push_back(2.3);
    point.positions.push_back(2.3);
    message.points.push_back(point);
  }
  publisher_->publish(message);
  RCLCPP_INFO(this->get_logger(), "publish");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JTMPublisher>());
  rclcpp::shutdown();
  return 0;
}