#include <cinttypes>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>

#include <chrono>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;

class KenMotionSound : public rclcpp::Node
{
public:
  /* 
  * コンストラクタ
  */
  KenMotionSound();
  /*
  * デストラクタ
  */
  ~KenMotionSound();

private:
  void stringCallback(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr string_sub_;
};

KenMotionSound::KenMotionSound() : Node("ken_motion_sound")
{
  string_sub_ = this->create_subscription<std_msgs::msg::String>(
    "ken_cmd_string", 1, std::bind(&KenMotionSound::stringCallback, this, _1));

  std::cout << "Finish Initialization" << std::endl;
}

KenMotionSound::~KenMotionSound() {}

void KenMotionSound::stringCallback(const std_msgs::msg::String::SharedPtr msg) {}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KenMotionSound>());
  rclcpp::shutdown();
  return 0;
}