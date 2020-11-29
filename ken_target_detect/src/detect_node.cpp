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
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
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
    geometry_msgs::msg::PoseArray & red_target_pose,
    geometry_msgs::msg::PoseArray & blue_target_pose,
    geometry_msgs::msg::PoseArray & yellow_target_pose,
    geometry_msgs::msg::PoseArray & green_target_pose, bool debug = false);

  void make_hsv_mask(
    const cv::Mat & src_color_img, cv::Mat & mask, const int h_min, const int h_max,
    const int s_min, const int s_max, const int v_min, const int v_max);

  void add_label(
    cv::Mat & src_color_img, const cv::Mat & src_depth_img, const cv::Mat & mask,
    const int area_size_thres, const cv::Scalar label_color,
    geometry_msgs::msg::PoseArray & target_pose);

  void calc_3d_point(const cv::Point & point, const cv::Mat & src_depth, cv::Point3d & point_3d);

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr red_target_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr blue_target_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr yellow_target_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr green_target_publisher_;

  message_filters::Subscriber<sensor_msgs::msg::Image> color_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;

  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::Image>
    SyncPolicy;

  typedef message_filters::Synchronizer<SyncPolicy> Sync;

  std::shared_ptr<Sync> sync_;
  double fx_;
  double fy_;
  double cx_;
  double cy_;
};

VisionTargetDetector::VisionTargetDetector() : Node("vision_target_detector")
{
  red_target_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("red_target", 1);
  blue_target_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("blue_target", 1);
  yellow_target_publisher_ =
    this->create_publisher<geometry_msgs::msg::PoseArray>("yellow_target", 1);
  green_target_publisher_ =
    this->create_publisher<geometry_msgs::msg::PoseArray>("green_target", 1);

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
  this->declare_parameter("fx", 0.0);
  this->declare_parameter("fy", 0.0);
  this->declare_parameter("cx", 0.0);
  this->declare_parameter("cy", 0.0);

  rclcpp::Parameter fx_param;
  rclcpp::Parameter fy_param;
  rclcpp::Parameter cx_param;
  rclcpp::Parameter cy_param;
  this->get_parameter("fx", fx_param);
  this->get_parameter("fy", fy_param);
  this->get_parameter("cx", cx_param);
  this->get_parameter("cy", cy_param);
  fx_ = fx_param.as_double();
  fy_ = fy_param.as_double();
  cx_ = cx_param.as_double();
  cy_ = cy_param.as_double();

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
  geometry_msgs::msg::PoseArray red_target;
  geometry_msgs::msg::PoseArray blue_target;
  geometry_msgs::msg::PoseArray yellow_target;
  geometry_msgs::msg::PoseArray green_target;
  red_target.header = depth_msg->header;
  blue_target.header = depth_msg->header;
  yellow_target.header = depth_msg->header;
  green_target.header = depth_msg->header;

  process_image(
    color_cv_ptr->image, depth_cv_ptr->image, out_img, red_target, blue_target, yellow_target,
    green_target, false);
  cv::imshow(OPENCV_WINDOW, out_img);
  cv::waitKey(3);

  red_target_publisher_->publish(red_target);
  blue_target_publisher_->publish(blue_target);
  yellow_target_publisher_->publish(yellow_target);
  green_target_publisher_->publish(green_target);
}

bool VisionTargetDetector::process_image(
  const cv::Mat & src_color_img, const cv::Mat & src_depth_img, cv::Mat & dst_img,
  geometry_msgs::msg::PoseArray & red_target_pose, geometry_msgs::msg::PoseArray & blue_target_pose,
  geometry_msgs::msg::PoseArray & yellow_target_pose,
  geometry_msgs::msg::PoseArray & green_target_pose, bool debug)
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

  add_label(
    dst_img, src_depth_img, red_mask & depth_mask, area_size_thres.as_int(), cv::Scalar(0, 0, 255),
    red_target_pose);
  add_label(
    dst_img, src_depth_img, blue_mask & depth_mask, area_size_thres.as_int(), cv::Scalar(255, 0, 0),
    blue_target_pose);
  add_label(
    dst_img, src_depth_img, yellow_mask & depth_mask, area_size_thres.as_int(),
    cv::Scalar(0, 255, 255), yellow_target_pose);
  add_label(
    dst_img, src_depth_img, green_mask & depth_mask, area_size_thres.as_int(),
    cv::Scalar(0, 255, 0), green_target_pose);

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
  cv::Mat & src_color_img, const cv::Mat & src_depth_img, const cv::Mat & mask,
  const int area_size_thres, const cv::Scalar label_color,
  geometry_msgs::msg::PoseArray & target_pose)
{
  // labeling
  // Check connected area size and position
  cv::Mat label_img;
  cv::Mat stats;
  cv::Mat centroids;

  int label = cv::connectedComponentsWithStats(mask, label_img, stats, centroids, 8, CV_16U);
  for (int i = 1; i < label; ++i) {
    if (stats.ptr<int>(i)[cv::ConnectedComponentsTypes::CC_STAT_AREA] > area_size_thres) {
      int x = static_cast<int>(centroids.ptr<double>(i)[0]);
      int y = static_cast<int>(centroids.ptr<double>(i)[1]);
      cv::circle(src_color_img, cv::Point(x, y), 5, label_color, -1);

      int left = stats.ptr<int>(i)[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
      int top = stats.ptr<int>(i)[cv::ConnectedComponentsTypes::CC_STAT_TOP];
      int width = stats.ptr<int>(i)[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];
      int height = stats.ptr<int>(i)[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
      cv::rectangle(src_color_img, cv::Rect(left, top, width, height), label_color, 2);
      cv::Point3d target_point_tmp;
      calc_3d_point(cv::Point(x, y), src_depth_img, target_point_tmp);
      geometry_msgs::msg::Pose target_pose_tmp;
      target_pose_tmp.position.x = target_point_tmp.x;
      target_pose_tmp.position.y = target_point_tmp.y;
      target_pose_tmp.position.z = target_point_tmp.z;
      target_pose_tmp.orientation.w = 1.0;
      target_pose.poses.push_back(target_pose_tmp);
    }
  }
}

void VisionTargetDetector::calc_3d_point(
  const cv::Point & point, const cv::Mat & src_depth, cv::Point3d & point_3d)
{
  double x_over_z = ((double)point.x - cx_) / fx_;
  double y_over_z = ((double)point.y - cy_) / fy_;

  std::cout << " cy: " << cy_ << " cx: " << cx_ << std::endl;

  double d = (double)src_depth.at<ushort>(point.y, point.x) / 1000.0;
  //  point_3d.z = d / std::sqrt(1 + x_over_z * x_over_z + y_over_z * y_over_z);
  point_3d.z = d;

  std::cout << " px: " << point.x << " py: " << point.y << " d: " << d << std::endl;

  point_3d.x = x_over_z * point_3d.z;
  point_3d.y = y_over_z * point_3d.z;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisionTargetDetector>());
  rclcpp::shutdown();
  return 0;
}