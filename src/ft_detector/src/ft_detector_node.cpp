#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <ft_detector/DetectorConfig.h>
#include <ft_msgs/DetectionArray.h>
#include <opencv2/opencv.hpp>
#include <boost/bind.hpp>
#include <memory>

class FtDetector {
 public:
  FtDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    pnh.param<std::string>("image_topic", image_topic_, "/hk_camera/image_raw/compressed");
    pnh.param<std::string>("camera_info_topic", info_topic_, "/hk_camera/camera_info");
    pnh.param<std::string>("detection_topic", det_topic_, "/ft/detection");
    pnh.param<std::string>("debug_topic", debug_topic_, "/ft/debug/image");
    pnh.param<int>("queue_size", queue_size_, 2);

    image_sub_ = nh.subscribe(image_topic_, queue_size_, &FtDetector::imageCallback, this);
    info_sub_ = nh.subscribe(info_topic_, 1, &FtDetector::infoCallback, this);
    debug_pub_ = nh.advertise<sensor_msgs::Image>(debug_topic_, 1);
    det_pub_ = nh.advertise<ft_msgs::DetectionArray>(det_topic_, 10);

    dr_server_.reset(new dynamic_reconfigure::Server<ft_detector::DetectorConfig>(pnh));
    dr_server_->setCallback(boost::bind(&FtDetector::reconfigureCallback, this, _1, _2));
    ROS_INFO_STREAM("ft_detector: image=" << image_topic_ << ", info=" << info_topic_);
  }

 private:
  void infoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
    camera_info_ = *msg;
    has_info_ = true;
  }

  void reconfigureCallback(ft_detector::DetectorConfig& cfg, uint32_t level) {
    (void)level;
    cfg_ = cfg;
  }

  void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (const cv_bridge::Exception& e) {
      ROS_ERROR_STREAM_THROTTLE(1.0, "cv_bridge: " << e.what());
      return;
    }

    std_msgs::Header header;
    header.stamp = msg->header.stamp;  // 必须用图像时间戳
    header.frame_id = !msg->header.frame_id.empty()
                          ? msg->header.frame_id
                          : (has_info_ ? camera_info_.header.frame_id : "camera");

    cv::Mat debug = cv_ptr->image.clone();
    cv::putText(debug, "ft_detector skeleton", cv::Point(20, 40),
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
    if (!has_info_) {
      cv::putText(debug, "no camera_info", cv::Point(20, 80),
                  cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
    }

    sensor_msgs::Image debug_msg;
    cv_bridge::CvImage(header, "bgr8", debug).toImageMsg(debug_msg);
    debug_pub_.publish(debug_msg);

    ft_msgs::DetectionArray detections;
    detections.header = header;
    det_pub_.publish(detections);
  }

  std::string image_topic_, info_topic_, det_topic_, debug_topic_;
  int queue_size_ = 2;
  bool has_info_ = false;
  sensor_msgs::CameraInfo camera_info_;
  ft_detector::DetectorConfig cfg_;
  ros::Subscriber image_sub_, info_sub_;
  ros::Publisher debug_pub_, det_pub_;
  std::unique_ptr<dynamic_reconfigure::Server<ft_detector::DetectorConfig>> dr_server_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ft_detector");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  FtDetector detector(nh, pnh);
  ros::spin();
  return 0;
}
