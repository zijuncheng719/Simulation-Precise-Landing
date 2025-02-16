#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

class ImageListener : public rclcpp::Node
{
public:
  ImageListener() : Node("image_listener")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10, std::bind(&ImageListener::imageCallback, this, std::placeholders::_1));
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) const
  {
    try
    {
      // 修改这里，使用CvImageConstPtr
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
      cv::Mat image = cv_ptr->image;

      // 使用OpenCV处理图像
      cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
      cv::imshow("Image window", image);
      cv::waitKey(3); // 等待3毫秒，以便更新窗口
    }
    catch (const cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageListener>());
  rclcpp::shutdown();
  return 0;
}