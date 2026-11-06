#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/TransformStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <ft_detector/DetectorConfig.h>
#include <ft_msgs/Detection.h>
#include <ft_msgs/DetectionArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <opencv2/opencv.hpp>
#include <boost/bind.hpp>
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace {
struct LightBar {
  cv::RotatedRect rect;
  cv::Point2f center;
  float width = 0.f;
  float height = 0.f;
  float angle = 0.f;
};

cv::Point2f rotatePoint(const cv::Point2f& p, float angle_deg) {
  const float a = angle_deg * static_cast<float>(CV_PI) / 180.f;
  return cv::Point2f(p.x * std::cos(a) - p.y * std::sin(a),
                     p.x * std::sin(a) + p.y * std::cos(a));
}
}  // namespace

class FtDetector {
 public:
  FtDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    pnh.param<std::string>("image_topic", image_topic_, "/hk_camera/image_raw/compressed");
    pnh.param<std::string>("camera_info_topic", info_topic_, "/hk_camera/camera_info");
    pnh.param<std::string>("detection_topic", det_topic_, "/ft/detection");
    pnh.param<std::string>("debug_topic", debug_topic_, "/ft/debug/image");
    pnh.param<int>("queue_size", queue_size_, 2);
    pnh.param<double>("large_armor_width", large_w_, 0.235);
    pnh.param<double>("large_armor_height", large_h_, 0.127);
    pnh.param<double>("small_armor_width", small_w_, 0.120);
    pnh.param<double>("small_armor_height", small_h_, 0.125);
    pnh.param<double>("large_gap_ratio", large_gap_ratio_, 1.4);
    pnh.param<double>("max_gap_ratio", max_gap_ratio_, 3.0);
    pnh.param<double>("merge_distance", merge_distance_, 0.25);
    pnh.param<double>("min_aspect", min_aspect_, 1.5);
    pnh.param<double>("max_aspect", max_aspect_, 12.0);
    pnh.param<double>("min_fill", min_fill_, 0.35);
    pnh.param<std::string>("fallback_frame", fallback_frame_, "camera");
    pnh.getParam("fallback_camera_matrix", fallback_K_);
    pnh.getParam("fallback_distortion", fallback_D_);

    image_sub_ = nh.subscribe(image_topic_, queue_size_, &FtDetector::imageCallback, this);
    info_sub_ = nh.subscribe(info_topic_, 1, &FtDetector::infoCallback, this);
    debug_pub_ = nh.advertise<sensor_msgs::Image>(debug_topic_, 1);
    det_pub_ = nh.advertise<ft_msgs::DetectionArray>(det_topic_, 10);
    tf_broadcaster_.reset(new tf2_ros::TransformBroadcaster());

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

  void buildMasks(const cv::Mat& hsv, cv::Mat& red_mask, cv::Mat& blue_mask) {
    cv::Mat r1, r2;
    cv::inRange(hsv, cv::Scalar(cfg_.red_h_min, cfg_.s_min, cfg_.v_min),
                cv::Scalar(cfg_.red_h_max, 255, 255), r1);
    cv::inRange(hsv, cv::Scalar(cfg_.red_h_min2, cfg_.s_min, cfg_.v_min),
                cv::Scalar(cfg_.red_h_max2, 255, 255), r2);
    cv::bitwise_or(r1, r2, red_mask);
    cv::inRange(hsv, cv::Scalar(cfg_.blue_h_min, cfg_.s_min, cfg_.v_min),
                cv::Scalar(cfg_.blue_h_max, 255, 255), blue_mask);
  }

  void detectBars(const cv::Mat& mask, std::vector<LightBar>& bars) {
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    // 竖向连接被反光/遮挡切断的灯条，横向不连接，避免跨灯条粘连。
    cv::Mat vertical = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 15));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, vertical);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (const auto& c : contours) {
      const double area = cv::contourArea(c);
      if (area < cfg_.min_area) continue;
      cv::RotatedRect rect = cv::minAreaRect(c);
      const float rw = rect.size.width;
      const float rh = rect.size.height;
      if (rw < 1.f || rh < 1.f) continue;

      // 灯条的长轴必须接近竖直；横向的顶部状态灯要排除。
      const bool width_is_long = rw >= rh;
      const float long_axis_angle = std::fabs(rect.angle);
      const bool long_is_vertical = width_is_long ? (long_axis_angle > 45.f)
                                                  : (long_axis_angle < 45.f);
      if (!long_is_vertical) continue;

      const float w = std::min(rw, rh);
      const float h = std::max(rw, rh);
      const float aspect = h / w;
      if (aspect < min_aspect_ || aspect > max_aspect_) continue;
      const float fill = static_cast<float>(area / (rw * rh + 1e-6));
      if (fill < min_fill_) continue;

      LightBar bar;
      bar.rect = rect;
      bar.center = rect.center;
      bar.width = w;
      bar.height = h;
      bar.angle = rect.angle;
      while (bar.angle < -45.f) bar.angle += 90.f;
      while (bar.angle >= 45.f) bar.angle -= 90.f;
      bars.push_back(bar);
    }
  }

  bool buildPair(const LightBar& a, const LightBar& b, bool& is_large) {
    const float avg_h = 0.5f * (a.height + b.height);
    const float avg_w = 0.5f * (a.width + b.width);
    const float height_diff = std::fabs(a.height - b.height) / (avg_h + 1e-6f);
    const float angle_diff = std::fabs(a.angle - b.angle);
    const float center_dy = std::fabs(a.center.y - b.center.y) / (avg_h + 1e-6f);
    const float gap = std::fabs(b.center.x - a.center.x);

    if (height_diff > cfg_.max_height_diff) return false;
    if (angle_diff > cfg_.max_angle_diff) return false;
    if (center_dy > 0.5f) return false;
    if (gap < 0.7f * avg_w || gap > cfg_.max_gap) return false;
    const float gap_ratio = gap / (avg_h + 1e-6f);
    if (gap_ratio < 0.4f || gap_ratio > max_gap_ratio_) return false;

    is_large = gap_ratio > large_gap_ratio_;
    return true;
  }

  void pairBars(const std::vector<LightBar>& bars, std::vector<std::pair<int, int>>& pairs,
                std::vector<bool>& is_large) {
    std::vector<bool> used(bars.size(), false);
    for (size_t i = 0; i < bars.size(); ++i) {
      if (used[i]) continue;
      int best = -1;
      float best_score = 1e9f;
      bool best_large = false;
      for (size_t j = i + 1; j < bars.size(); ++j) {
        if (used[j]) continue;
        bool large = false;
        if (!buildPair(bars[i], bars[j], large)) continue;
        const float score = std::fabs(bars[i].center.y - bars[j].center.y) +
                            std::fabs(bars[i].angle - bars[j].angle);
        if (score < best_score) { best_score = score; best = static_cast<int>(j); best_large = large; }
      }
      if (best >= 0) {
        used[i] = true;
        used[best] = true;
        pairs.emplace_back(static_cast<int>(i), best);
        is_large.push_back(best_large);
      }
    }
  }

  static void barCorners(const LightBar& bar, cv::Point2f& left_top, cv::Point2f& left_bottom,
                         cv::Point2f& right_top, cv::Point2f& right_bottom, bool take_left) {
    cv::Point2f pts[4];
    bar.rect.points(pts);
    std::vector<cv::Point2f> p(pts, pts + 4);
    std::sort(p.begin(), p.end(), [](const cv::Point2f& u, const cv::Point2f& v) {
      return u.x < v.x;
    });
    cv::Point2f a = p[0], b = p[1];
    if (take_left) {
      left_top = a.y < b.y ? a : b;
      left_bottom = a.y < b.y ? b : a;
    } else {
      cv::Point2f c = p[2], d = p[3];
      right_top = c.y < d.y ? c : d;
      right_bottom = c.y < d.y ? d : c;
    }
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
    header.stamp = msg->header.stamp;
    if (has_info_ && !camera_info_.header.frame_id.empty()) {
      header.frame_id = camera_info_.header.frame_id;
    } else if (!msg->header.frame_id.empty()) {
      header.frame_id = msg->header.frame_id;
    } else {
      header.frame_id = fallback_frame_;
    }

    cv::Mat hsv;
    cv::cvtColor(cv_ptr->image, hsv, cv::COLOR_BGR2HSV);
    cv::Mat red_mask, blue_mask;
    buildMasks(hsv, red_mask, blue_mask);

    std::vector<LightBar> red_bars, blue_bars;
    detectBars(red_mask, red_bars);
    detectBars(blue_mask, blue_bars);

    cv::Mat debug = cv_ptr->image.clone();
    ft_msgs::DetectionArray out;
    out.header = header;

    cv::Mat K, D;
    if (has_info_) {
      K = cv::Mat(3, 3, CV_64F, const_cast<double*>(camera_info_.K.data())).clone();
      D = camera_info_.D.empty()
              ? cv::Mat()
              : cv::Mat(1, static_cast<int>(camera_info_.D.size()), CV_64F,
                        const_cast<double*>(camera_info_.D.data())).clone();
    } else if (fallback_K_.size() == 9) {
      K = cv::Mat(3, 3, CV_64F, fallback_K_.data()).clone();
      if (!fallback_D_.empty()) {
        D = cv::Mat(1, static_cast<int>(fallback_D_.size()), CV_64F, fallback_D_.data()).clone();
      }
    }

    int id = 0;
    for (int color = 0; color < 2; ++color) {
      const auto& bars = (color == 0) ? red_bars : blue_bars;
      std::vector<std::pair<int, int>> pairs;
      std::vector<bool> large_flags;
      pairBars(bars, pairs, large_flags);
      for (size_t k = 0; k < pairs.size(); ++k) {
        const LightBar& left = bars[pairs[k].first];
        const LightBar& right = bars[pairs[k].second];
        const LightBar& b1 = left.center.x < right.center.x ? left : right;
        const LightBar& b2 = left.center.x < right.center.x ? right : left;

        cv::Point2f lt, lb, rt, rb;
        barCorners(b1, lt, lb, rt, rb, true);
        cv::Point2f lt2, lb2, rt2, rb2;
        barCorners(b2, lt2, lb2, rt2, rb2, false);

        std::vector<cv::Point2f> img_pts = {lt, rt2, rb2, lb};
        const bool is_large = large_flags[k];
        const double W = is_large ? large_w_ : small_w_;
        const double H = is_large ? large_h_ : small_h_;
        std::vector<cv::Point3f> obj_pts = {
            {0.f, 0.f, 0.f}, {static_cast<float>(W), 0.f, 0.f},
            {static_cast<float>(W), static_cast<float>(H), 0.f},
            {0.f, static_cast<float>(H), 0.f}};

        cv::Scalar color_draw = (color == 0) ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
        cv::line(debug, lt, rt2, color_draw, 2);
        cv::line(debug, rt2, rb2, color_draw, 2);
        cv::line(debug, rb2, lb, color_draw, 2);
        cv::line(debug, lb, lt, color_draw, 2);
        cv::Point2f pts[4];
        left.rect.points(pts);
        for (int i = 0; i < 4; ++i) cv::line(debug, pts[i], pts[(i + 1) % 4], cv::Scalar(0, 255, 0), 1);

        ft_msgs::Detection det;
        det.id = static_cast<uint32_t>(id);
        det.is_red = (color == 0);
        det.is_large_armor = is_large;

        if (has_info_ && K.rows == 3) {
          cv::Vec3d rvec, tvec;
          try {
            cv::solvePnP(obj_pts, img_pts, K, D, rvec, tvec, false, cv::SOLVEPNP_IPPE);
          } catch (const cv::Exception& e1) {
            try {
              cv::solvePnP(obj_pts, img_pts, K, D, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
            } catch (const cv::Exception& e2) {
              ROS_WARN_STREAM_THROTTLE(1.0, "solvePnP failed: " << e1.what() << " / " << e2.what());
              ++id;
              continue;
            }
          }
          det.pose.position.x = tvec[0];
          det.pose.position.y = tvec[1];
          det.pose.position.z = tvec[2];

          cv::Mat R;
          cv::Rodrigues(rvec, R);
          tf2::Matrix3x3 m(R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
                           R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
                           R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2));
          tf2::Quaternion q;
          m.getRotation(q);
          q.normalize();
          det.pose.orientation.x = q.x();
          det.pose.orientation.y = q.y();
          det.pose.orientation.z = q.z();
          det.pose.orientation.w = q.w();

          std::vector<cv::Point3f> axes = {{0.f, 0.f, 0.f}, {0.03f, 0.f, 0.f},
                                           {0.f, 0.03f, 0.f}, {0.f, 0.f, 0.03f}};
          std::vector<cv::Point2f> proj;
          cv::projectPoints(axes, rvec, tvec, K, D, proj);
          cv::line(debug, proj[0], proj[1], cv::Scalar(0, 0, 255), 2);
          cv::line(debug, proj[0], proj[2], cv::Scalar(0, 255, 0), 2);
          cv::line(debug, proj[0], proj[3], cv::Scalar(255, 0, 0), 2);

          geometry_msgs::TransformStamped tf;
          tf.header = header;
          tf.child_frame_id = "armor_" + std::to_string(id);
          tf.transform.translation.x = tvec[0];
          tf.transform.translation.y = tvec[1];
          tf.transform.translation.z = tvec[2];
          tf.transform.rotation = det.pose.orientation;
          tf_broadcaster_->sendTransform(tf);
        }

        cv::putText(debug, (is_large ? "L" : "S") + std::string(color == 0 ? "R" : "B") +
                               " " + std::to_string(id),
                    cv::Point(static_cast<int>(lt.x), static_cast<int>(lt.y) - 6),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, color_draw, 1);
        bool duplicate = false;
        if (has_info_) {
          for (const auto& existing : out.detections) {
            if (existing.is_red != det.is_red) continue;
            const double dx = existing.pose.position.x - det.pose.position.x;
            const double dy = existing.pose.position.y - det.pose.position.y;
            const double dz = existing.pose.position.z - det.pose.position.z;
            if (std::sqrt(dx * dx + dy * dy + dz * dz) < merge_distance_) {
              duplicate = true;
              break;
            }
          }
        }
        if (duplicate) { ++id; continue; }
        out.detections.push_back(det);
        ++id;
      }
    }

    if (!has_info_) {
      cv::putText(debug, "no camera_info: PnP skipped", cv::Point(20, 30),
                  cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    }

    sensor_msgs::Image debug_msg;
    cv_bridge::CvImage(header, "bgr8", debug).toImageMsg(debug_msg);
    debug_pub_.publish(debug_msg);
    det_pub_.publish(out);
  }

  std::string image_topic_, info_topic_, det_topic_, debug_topic_;
  int queue_size_ = 2;
  bool has_info_ = false;
  double large_w_ = 0.235, large_h_ = 0.127;
  double small_w_ = 0.120, small_h_ = 0.125;
  double large_gap_ratio_ = 1.4;
  double max_gap_ratio_ = 3.0;
  double merge_distance_ = 0.25;
  double min_aspect_ = 1.5, max_aspect_ = 12.0, min_fill_ = 0.35;
  std::string fallback_frame_ = "camera";
  std::vector<double> fallback_K_, fallback_D_;
  sensor_msgs::CameraInfo camera_info_;
  ft_detector::DetectorConfig cfg_;
  ros::Subscriber image_sub_, info_sub_;
  ros::Publisher debug_pub_, det_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<dynamic_reconfigure::Server<ft_detector::DetectorConfig>> dr_server_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ft_detector");
  ros::NodeHandle nh, pnh("~");
  FtDetector detector(nh, pnh);
  ros::spin();
  return 0;
}
