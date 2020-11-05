#include <memory>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <chrono>
#include <functional>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

static const std::string OPENCV_WINDOW = "Image window";

class VisionTargetDetector : public rclcpp::Node
{
public:
  /* 
  * コンストラクタ
  */
  VisionTargetDetector();
  /*
  * デストラクタ
  */
  ~VisionTargetDetector();

private:
  void topic_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & color_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg);

  bool process_image(
    const cv::Mat & src_color_img, const cv::Mat & src_depth_img, cv::Mat & dst_img,
    bool debug = false);

  void make_hsv_mask(
    const cv::Mat & src_color_img, cv::Mat & mask, const int h_min, const int h_max,
    const int s_min, const int s_max, const int v_min, const int v_max);

  void add_label(
    cv::Mat & src_color_img, const cv::Mat & mask, const int area_size_thres,
    const cv::Scalar label_color);

  message_filters::Subscriber<sensor_msgs::msg::Image> color_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;

  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::Image>
    SyncPolicy;

  typedef message_filters::Synchronizer<SyncPolicy> Sync;

  std::shared_ptr<Sync> sync_;
};

VisionTargetDetector::VisionTargetDetector() : Node("vision_target_detector")
{
  color_sub_.subscribe(this, "/camera/color/image_raw");
  depth_sub_.subscribe(this, "/camera/aligned_depth_to_color/image_raw");

  sync_.reset(new Sync(SyncPolicy(10), color_sub_, depth_sub_));
  sync_->registerCallback(std::bind(&VisionTargetDetector::topic_callback, this, _1, _2));

  this->declare_parameter("red_h_range_min", 0);
  this->declare_parameter("red_h_range_max", 0);
  this->declare_parameter("blue_h_range_min", 0);
  this->declare_parameter("blue_h_range_max", 0);
  this->declare_parameter("green_h_range_min", 0);
  this->declare_parameter("green_h_range_max", 0);
  this->declare_parameter("yellow_h_range_min", 0);
  this->declare_parameter("yellow_h_range_max", 0);
  this->declare_parameter("s_range_min", 0);
  this->declare_parameter("s_range_max", 0);
  this->declare_parameter("v_range_min", 0);
  this->declare_parameter("v_range_max", 0);
  this->declare_parameter("area_size_thres", 0);
  this->declare_parameter("depth_detection_area_thres", 0);

  std::cout << "Finish Initialization" << std::endl;
}

VisionTargetDetector::~VisionTargetDetector() { cv::destroyAllWindows(); }

void VisionTargetDetector::topic_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr & color_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg)
{
  RCLCPP_INFO(this->get_logger(), "I heard:");

  cv_bridge::CvImagePtr color_cv_ptr;
  try {
    color_cv_ptr = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  cv_bridge::CvImagePtr depth_cv_ptr;
  try {
    depth_cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat out_img;

  process_image(color_cv_ptr->image, depth_cv_ptr->image, out_img, true);
  cv::imshow(OPENCV_WINDOW, out_img);
  cv::waitKey(3);
}

bool VisionTargetDetector::process_image(
  const cv::Mat & src_color_img, const cv::Mat & src_depth_img, cv::Mat & dst_img, bool debug)
{
  // convert to HSV and make mask
  cv::Mat hsv_img;
  cv::cvtColor(src_color_img, hsv_img, CV_BGR2HSV_FULL);

  rclcpp::Parameter red_h_range_max;
  rclcpp::Parameter red_h_range_min;
  rclcpp::Parameter blue_h_range_max;
  rclcpp::Parameter blue_h_range_min;
  rclcpp::Parameter yellow_h_range_max;
  rclcpp::Parameter yellow_h_range_min;
  rclcpp::Parameter green_h_range_max;
  rclcpp::Parameter green_h_range_min;
  rclcpp::Parameter s_range_max;
  rclcpp::Parameter s_range_min;
  rclcpp::Parameter v_range_max;
  rclcpp::Parameter v_range_min;
  rclcpp::Parameter area_size_thres;
  rclcpp::Parameter depth_detection_area_thres;

  this->get_parameter("red_h_range_max", red_h_range_max);
  this->get_parameter("red_h_range_min", red_h_range_min);
  this->get_parameter("blue_h_range_max", blue_h_range_max);
  this->get_parameter("blue_h_range_min", blue_h_range_min);
  this->get_parameter("yellow_h_range_max", yellow_h_range_max);
  this->get_parameter("yellow_h_range_min", yellow_h_range_min);
  this->get_parameter("green_h_range_max", green_h_range_max);
  this->get_parameter("green_h_range_min", green_h_range_min);
  this->get_parameter("s_range_max", s_range_max);
  this->get_parameter("s_range_min", s_range_min);
  this->get_parameter("v_range_max", v_range_max);
  this->get_parameter("v_range_min", v_range_min);
  this->get_parameter("area_size_thres", area_size_thres);
  this->get_parameter("depth_detection_area_thres", depth_detection_area_thres);

  cv::Mat depth_mask;
  cv::inRange(src_depth_img, 0, depth_detection_area_thres.as_int(), depth_mask);

  cv::Mat red_mask;
  cv::Mat blue_mask;
  cv::Mat yellow_mask;
  cv::Mat green_mask;

  make_hsv_mask(
    hsv_img, red_mask, red_h_range_min.as_int(), red_h_range_max.as_int(), s_range_min.as_int(),
    s_range_max.as_int(), v_range_min.as_int(), v_range_max.as_int());
  make_hsv_mask(
    hsv_img, blue_mask, blue_h_range_min.as_int(), blue_h_range_max.as_int(), s_range_min.as_int(),
    s_range_max.as_int(), v_range_min.as_int(), v_range_max.as_int());
  make_hsv_mask(
    hsv_img, yellow_mask, yellow_h_range_min.as_int(), yellow_h_range_max.as_int(),
    s_range_min.as_int(), s_range_max.as_int(), v_range_min.as_int(), v_range_max.as_int());
  make_hsv_mask(
    hsv_img, green_mask, green_h_range_min.as_int(), green_h_range_max.as_int(),
    s_range_min.as_int(), s_range_max.as_int(), v_range_min.as_int(), v_range_max.as_int());

  if (debug) {
    cv::Mat depth_masked_color_img;
    cv::bitwise_and(src_color_img, src_color_img, depth_masked_color_img, depth_mask);
    cv::imshow("mask depth color", depth_masked_color_img);

    cv::Mat masked_red_img;
    cv::bitwise_and(src_color_img, src_color_img, masked_red_img, red_mask);
    cv::imshow("mask red", masked_red_img);
    cv::Mat masked_blue_img;
    cv::bitwise_and(src_color_img, src_color_img, masked_blue_img, blue_mask);
    cv::imshow("mask blue", masked_blue_img);
    cv::Mat masked_yellow_img;
    cv::bitwise_and(src_color_img, src_color_img, masked_yellow_img, yellow_mask);
    cv::imshow("mask yellow", masked_yellow_img);
    cv::Mat masked_green_img;
    cv::bitwise_and(src_color_img, src_color_img, masked_green_img, green_mask);
    cv::imshow("mask green", masked_green_img);

    cv::waitKey(3);
  }

  dst_img = src_color_img;

  add_label(dst_img, red_mask & depth_mask, area_size_thres.as_int(), cv::Scalar(0, 0, 255));
  add_label(dst_img, blue_mask & depth_mask, area_size_thres.as_int(), cv::Scalar(255, 0, 0));
  add_label(dst_img, yellow_mask & depth_mask, area_size_thres.as_int(), cv::Scalar(0, 255, 255));
  add_label(dst_img, green_mask & depth_mask, area_size_thres.as_int(), cv::Scalar(0, 255, 0));

  return true;
}

void VisionTargetDetector::make_hsv_mask(
  const cv::Mat & src_color_img, cv::Mat & mask, const int h_min, const int h_max, const int s_min,
  const int s_max, const int v_min, const int v_max)
{
  if (h_min <= h_max) {
    cv::Scalar mask_lower1 = cv::Scalar(h_min, s_min, v_min);
    cv::Scalar mask_upper1 = cv::Scalar(h_max, s_max, v_max);
    cv::inRange(src_color_img, mask_lower1, mask_upper1, mask);
  } else {
    cv::Scalar mask_lower1 = cv::Scalar(h_min, s_min, v_min);
    cv::Scalar mask_upper1 = cv::Scalar(255, s_max, v_max);
    cv::Mat mask1;
    cv::inRange(src_color_img, mask_lower1, mask_upper1, mask1);
    cv::Scalar mask_lower2 = cv::Scalar(0, s_min, v_min);
    cv::Scalar mask_upper2 = cv::Scalar(h_max, s_max, v_max);
    cv::Mat mask2;
    cv::inRange(src_color_img, mask_lower2, mask_upper2, mask2);
    mask = mask1 | mask2;
  }
}

void VisionTargetDetector::add_label(
  cv::Mat & src_color_img, const cv::Mat & mask, const int area_size_thres,
  const cv::Scalar label_color)
{
  // labeling
  // Check connected area size and position
  cv::Mat label_img;
  cv::Mat stats;
  cv::Mat centroids;

  int red_label = cv::connectedComponentsWithStats(mask, label_img, stats, centroids, 8, CV_16U);
  for (int i = 1; i < red_label; ++i) {
    if (stats.ptr<int>(i)[cv::ConnectedComponentsTypes::CC_STAT_AREA] > area_size_thres) {
      int x = static_cast<int>(centroids.ptr<double>(i)[0]);
      int y = static_cast<int>(centroids.ptr<double>(i)[1]);
      cv::circle(src_color_img, cv::Point(x, y), 5, label_color, -1);

      int left = stats.ptr<int>(i)[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
      int top = stats.ptr<int>(i)[cv::ConnectedComponentsTypes::CC_STAT_TOP];
      int width = stats.ptr<int>(i)[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];
      int height = stats.ptr<int>(i)[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
      cv::rectangle(src_color_img, cv::Rect(left, top, width, height), label_color, 2);
    }
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisionTargetDetector>());
  rclcpp::shutdown();
  return 0;
}