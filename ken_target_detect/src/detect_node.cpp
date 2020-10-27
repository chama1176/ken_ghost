#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
using std::placeholders::_1;
static const std::string OPENCV_WINDOW = "Image window";

class VisionTargetDetector : public rclcpp::Node
{
    public:
        /*
        * コンストラクタ
        */
        VisionTargetDetector ();
        /*
        * デストラクタ
        */
        ~VisionTargetDetector();

    private:

        void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg);

        bool process_image(const cv::Mat& src_img, cv::Mat& dst_img, bool debug = false);

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

VisionTargetDetector::VisionTargetDetector ()
: Node("vision_target_detector")
{
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/color/image_raw", 10, std::bind(&VisionTargetDetector ::topic_callback, this, _1));
}

VisionTargetDetector::~VisionTargetDetector()
{
    cv::destroyAllWindows();
}

void VisionTargetDetector::topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat in_img;
    cv::Mat out_img;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    }
    catch(cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    process_image(cv_ptr->image, out_img, true);
    cv::imshow(OPENCV_WINDOW, out_img);
    cv::waitKey(3);

    RCLCPP_INFO(this->get_logger(), "I heard:");
}

bool VisionTargetDetector::process_image(const cv::Mat& src_img, cv::Mat& dst_img, bool debug)
{
    // convert to HSV and make mask
    cv::Mat hsv_img, mask;
    cv::Scalar mask_lower, mask_upper;
    mask_lower = cv::Scalar(0, 100, 50);
    mask_upper = cv::Scalar(30, 200, 200);
    cv::cvtColor(src_img, hsv_img, CV_BGR2HSV_FULL);
    cv::inRange(hsv_img, mask_lower, mask_upper, mask);
    if(debug)
    {
        cv::Mat masked_img;
        cv::bitwise_and(src_img, src_img, masked_img, mask);
        cv::imshow("mask", masked_img);
        cv::waitKey(3);
    }

    dst_img = src_img;

    // labeling
    cv::Mat label_img;
    cv::Mat stats;
    cv::Mat centroids;

    int n_label = cv::connectedComponentsWithStats(mask, label_img, stats, centroids, 8, CV_16U);

    for(int i = 1; i < n_label; ++i)
    {
        if(stats.ptr<int>(i)[cv::ConnectedComponentsTypes::CC_STAT_AREA] > 100)
        {
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
    rclcpp::spin(std::make_shared<VisionTargetDetector >());
    rclcpp::shutdown();
    return 0;
}