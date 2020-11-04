#include <chrono>
#include <functional>
#include <memory>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

class MinimalSubscriber : public rclcpp::Node
{
public:
  /* 
  * コンストラクタ
  */
  MinimalSubscriber();
  /*
  * デストラクタ
  */
  ~MinimalSubscriber();

private:
  void topic_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & color_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg);

  message_filters::Subscriber<sensor_msgs::msg::Image> color_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;

  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>
    SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  std::shared_ptr<Sync> sync_;
};

MinimalSubscriber::MinimalSubscriber() : Node("minimal_subscriber")
{
  RCLCPP_INFO(this->get_logger(), "Hello");
  std::cout << "Hello world" << std::endl;

  color_sub_.subscribe(this, "/camera/color/image_raw");
  depth_sub_.subscribe(this, "/camera/aligned_depth_to_color/image_raw");
  info_sub_.subscribe(this, "/camera/color/camera_info");
  // message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo> sync(
  //   color_sub_, info_sub_, 10);
  sync_.reset(new Sync(SyncPolicy(10), color_sub_, info_sub_));
  sync_->registerCallback(std::bind(&MinimalSubscriber ::topic_callback, this, _1, _2));
}

MinimalSubscriber::~MinimalSubscriber() {}

void MinimalSubscriber::topic_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr & color_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
{
  RCLCPP_INFO(this->get_logger(), "I heard:");
  std::cout << "heard" << std::endl;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}