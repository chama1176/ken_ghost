#include <memory>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;
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
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  bool process_image(const cv::Mat & src_img, cv::Mat & dst_img, bool debug = false);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

VisionTargetDetector::VisionTargetDetector() : Node("vision_target_detector")
{
  subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/color/image_raw", 10, std::bind(&VisionTargetDetector ::topic_callback, this, _1));
  this->declare_parameter("red_h_range_min", 0);
  this->declare_parameter("red_h_range_max", 0);
}

VisionTargetDetector::~VisionTargetDetector() { cv::destroyAllWindows(); }

void VisionTargetDetector::topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat in_img;
  cv::Mat out_img;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  rclcpp::Parameter red_h_range_min;
  rclcpp::Parameter red_h_range_max;
  this->get_parameter("red_h_range_min", red_h_range_min);
  this->get_parameter("red_h_range_max", red_h_range_max);
  RCLCPP_INFO(
    this->get_logger(), "Hello %d %d", red_h_range_min.as_int(), red_h_range_max.as_int());

  process_image(cv_ptr->image, out_img, true);
  cv::imshow(OPENCV_WINDOW, out_img);
  cv::waitKey(3);

  RCLCPP_INFO(this->get_logger(), "I heard:");
}

bool VisionTargetDetector::process_image(const cv::Mat & src_img, cv::Mat & dst_img, bool debug)
{
  // convert to HSV and make mask
  cv::Mat hsv_img;
  cv::cvtColor(src_img, hsv_img, CV_BGR2HSV_FULL);

  cv::Scalar mask_lower1 = cv::Scalar(0, 0, 0);
  cv::Scalar mask_upper1 = cv::Scalar(30, 255, 255);
  cv::Mat mask1;
  cv::inRange(hsv_img, mask_lower1, mask_upper1, mask1);
  cv::Scalar mask_lower2 = cv::Scalar(150, 0, 0);
  cv::Scalar mask_upper2 = cv::Scalar(179, 255, 255);
  cv::Mat mask2;
  cv::inRange(hsv_img, mask_lower2, mask_upper2, mask2);
  cv::Scalar mask_lower3 = cv::Scalar(30, 0, 0);
  cv::Scalar mask_upper3 = cv::Scalar(90, 255, 255);
  cv::Mat mask3;
  cv::inRange(hsv_img, mask_lower3, mask_upper3, mask3);
  cv::Scalar mask_lower4 = cv::Scalar(90, 0, 0);
  cv::Scalar mask_upper4 = cv::Scalar(150, 255, 255);
  cv::Mat mask4;
  cv::inRange(hsv_img, mask_lower4, mask_upper4, mask4);

  if (debug) {
    cv::Mat masked_red_img;
    cv::bitwise_and(src_img, src_img, masked_red_img, mask1 | mask2);
    cv::imshow("mask red", masked_red_img);
    cv::waitKey(3);
    cv::Mat masked_green_img;
    cv::bitwise_and(src_img, src_img, masked_green_img, mask3);
    cv::imshow("mask green", masked_green_img);
    cv::waitKey(3);
    cv::Mat masked_blue_img;
    cv::bitwise_and(src_img, src_img, masked_blue_img, mask4);
    cv::imshow("mask blue", masked_blue_img);
    cv::waitKey(3);
  }

  dst_img = src_img;

  // labeling
  cv::Mat label_img;
  cv::Mat stats;
  cv::Mat centroids;

  int n_label = cv::connectedComponentsWithStats(mask1, label_img, stats, centroids, 8, CV_16U);

  for (int i = 1; i < n_label; ++i) {
    if (stats.ptr<int>(i)[cv::ConnectedComponentsTypes::CC_STAT_AREA] > 100) {
      int x = static_cast<int>(centroids.ptr<double>(i)[0]);
      int y = static_cast<int>(centroids.ptr<double>(i)[1]);
      cv::circle(dst_img, cv::Point(x, y), 10, cv::Scalar(0, 255, 0));
    }
  }

  // // make contour
  // std::vector<std::vector<cv::Point>> contours;
  // cv::findContours(mask, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

  // // make region
  // std::vector<std::vector<cv::Point>> hull(contours.size());
  // std::vector<cv::Rect> bound_rect(contours.size());
  // for(size_t i = 0; i < contours.size(); i++)
  // {
  //     cv::convexHull(contours[i], hull[i]);
  //     bound_rect[i] = cv::boundingRect(hull[i]);
  // }

  // // draw rect
  // dst_img = src_img;
  // for(size_t i = 0; i < bound_rect.size(); i++)
  // {
  //     cv::rectangle(dst_img, bound_rect[i], cv::Scalar(0, 255, 0));
  // }
  return true;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisionTargetDetector>());
  rclcpp::shutdown();
  return 0;
}