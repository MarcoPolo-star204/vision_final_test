#include <ros/ros.h>
#include <ft_msgs/DetectionArray.h>
#include <ft_msgs/Track.h>
#include <visualization_msgs/Marker.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>

// ft_tracker_node.cpp
// ------------------------------------------------------------------
// 职责：订阅 /ft/detection，做单目标 Kalman 跟踪，输出 /ft/track。
//
// 状态： [x, y, z, vx, vy, vz]（相机系，米/米每秒）
// 观测： [x, y, z]
// 预测： position_predict = position + velocity * T（T 为提前量）
//
// 关键点：
// - dt 用两帧图像时间戳相减，不用 now()；
// - 丢检时继续 predict，但 tracking=false；
// - 连续丢检超过 max_lost 或外推超过 2 m，就 reset，不能无限外推。
class FtTracker {
 public:
  FtTracker(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    pnh.param<std::string>("detection_topic", det_topic_, "/ft/detection");
    pnh.param<std::string>("track_topic", track_topic_, "/ft/track");
    pnh.param<std::string>("marker_topic", marker_topic_, "/ft/debug/marker");
    // 跟踪参数：T 提前量；gate 关联门限；Q/R 过程/观测噪声
    pnh.param<double>("T", T_, 0.1);
    pnh.param<double>("gate", gate_, 0.4);
    pnh.param<double>("q_pos", q_pos_, 0.01);
    pnh.param<double>("q_vel", q_vel_, 0.1);
    pnh.param<double>("r_meas", r_meas_, 0.02);
    pnh.param<double>("max_lost", max_lost_, 0.5);

    // 6 维状态、3 维观测、0 维控制量。
    kf_.init(6, 3, 0);
    cv::setIdentity(kf_.transitionMatrix);
    cv::setIdentity(kf_.measurementMatrix);
    kf_.processNoiseCov = cv::Mat::zeros(6, 6, CV_32F);
    for (int i = 0; i < 3; ++i) {
      kf_.processNoiseCov.at<float>(i, i) = static_cast<float>(q_pos_);
      kf_.processNoiseCov.at<float>(i + 3, i + 3) = static_cast<float>(q_vel_);
    }
    kf_.measurementNoiseCov = cv::Mat::eye(3, 3, CV_32F) * static_cast<float>(r_meas_);
    cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(1.0));

    sub_ = nh.subscribe(det_topic_, 10, &FtTracker::callback, this);
    track_pub_ = nh.advertise<ft_msgs::Track>(track_topic_, 10);
    marker_pub_ = nh.advertise<visualization_msgs::Marker>(marker_topic_, 1);
  }

 private:
  // 清空轨迹：下一次有检测时重新初始化。
  void reset() {
    kf_.statePost = cv::Mat::zeros(6, 1, CV_32F);
    kf_.statePre = cv::Mat::zeros(6, 1, CV_32F);
    initialized_ = false;
    lost_time_ = 0.0;
  }

  // 用第一帧检测初始化位置，速度先设为 0（没有历史，猜不了速度）。
  void initFrom(const geometry_msgs::Pose& pose) {
    kf_.statePost.at<float>(0) = static_cast<float>(pose.position.x);
    kf_.statePost.at<float>(1) = static_cast<float>(pose.position.y);
    kf_.statePost.at<float>(2) = static_cast<float>(pose.position.z);
    kf_.statePost.at<float>(3) = 0.0f;
    kf_.statePost.at<float>(4) = 0.0f;
    kf_.statePost.at<float>(5) = 0.0f;
    initialized_ = true;
    lost_time_ = 0.0;
  }

  // 每帧检测进来：算 dt -> 预测 -> 关联 -> 修正 -> 发布。
  void callback(const ft_msgs::DetectionArrayConstPtr& msg) {
    double dt = 0.05;
    if (!last_stamp_.isZero()) {
      if (msg->header.stamp < last_stamp_) {
        // bag 循环或时间戳回退：重置计时，避免 dt 过小导致永远不丢轨迹
        last_stamp_ = msg->header.stamp;
        lost_time_ = 0.0;
        dt = 0.05;
      } else {
        dt = (msg->header.stamp - last_stamp_).toSec();
      }
    }
    dt = std::max(0.001, std::min(dt, 0.2));
    last_stamp_ = msg->header.stamp;

    // 还没初始化：拿这一帧第一个检测作为轨迹起点。
    if (!initialized_) {
      if (msg->detections.empty()) return;
      initFrom(msg->detections.front().pose);
    }

    // 匀速模型：x += vx*dt，y += vy*dt，z += vz*dt。
    // 每次 dt 变了都要重新写 F 矩阵，不能只在构造时写一次。
    kf_.transitionMatrix.at<float>(0, 3) = static_cast<float>(dt);
    kf_.transitionMatrix.at<float>(1, 4) = static_cast<float>(dt);
    kf_.transitionMatrix.at<float>(2, 5) = static_cast<float>(dt);
    kf_.predict();

    // 数据关联：在预测位置附近找最近的检测；超过 gate 就不认。
    int best = -1;
    double best_dist = 1e9;
    for (size_t i = 0; i < msg->detections.size(); ++i) {
      const auto& p = msg->detections[i].pose.position;
      const double dx = p.x - kf_.statePre.at<float>(0);
      const double dy = p.y - kf_.statePre.at<float>(1);
      const double dz = p.z - kf_.statePre.at<float>(2);
      const double d = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (d < best_dist) { best_dist = d; best = static_cast<int>(i); }
    }

    // 有命中 -> correct，tracking=true；没命中 -> 只 predict，tracking=false。
    bool tracking = false;
    if (best >= 0 && best_dist <= gate_) {
      cv::Mat meas(3, 1, CV_32F);
      meas.at<float>(0) = static_cast<float>(msg->detections[best].pose.position.x);
      meas.at<float>(1) = static_cast<float>(msg->detections[best].pose.position.y);
      meas.at<float>(2) = static_cast<float>(msg->detections[best].pose.position.z);
      // 用观测修正状态；同时记录最后一次真实观测位置，防止外推太远。
      kf_.correct(meas);
      tracking = true;
      lost_time_ = 0.0;
      miss_count_ = 0;
      last_observed_.x = kf_.statePost.at<float>(0);
      last_observed_.y = kf_.statePost.at<float>(1);
      last_observed_.z = kf_.statePost.at<float>(2);
    } else {
      lost_time_ += dt;
      ++miss_count_;
      const double dx = kf_.statePost.at<float>(0) - last_observed_.x;
      const double dy = kf_.statePost.at<float>(1) - last_observed_.y;
      const double dz = kf_.statePost.at<float>(2) - last_observed_.z;
      // 连续 10 帧没关联，或预测离最后观测超过 2 m：认为跟丢了，重置。
      if (miss_count_ > 10 || std::sqrt(dx*dx + dy*dy + dz*dz) > 2.0) {
        reset();
        return;
      }
    }

    if (lost_time_ > max_lost_) { reset(); return; }

    // 发布跟踪结果：position/velocity 是滤波后的值，
    // position_predict 是往前推 T 秒的提前量。
    ft_msgs::Track out;
    out.header = msg->header;
    out.id = 1;
    out.tracking = tracking;
    out.position.x = kf_.statePost.at<float>(0);
    out.position.y = kf_.statePost.at<float>(1);
    out.position.z = kf_.statePost.at<float>(2);
    out.velocity.x = kf_.statePost.at<float>(3);
    out.velocity.y = kf_.statePost.at<float>(4);
    out.velocity.z = kf_.statePost.at<float>(5);
    out.position_predict.x = out.position.x + out.velocity.x * T_;
    out.position_predict.y = out.position.y + out.velocity.y * T_;
    out.position_predict.z = out.position.z + out.velocity.z * T_;
    track_pub_.publish(out);

    // RViz marker：画在预测位置，方便肉眼检查“有没有往前一截”。
    visualization_msgs::Marker marker;
    marker.header = out.header;
    marker.ns = "ft_track";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = out.position_predict.x;
    marker.pose.position.y = out.position_predict.y;
    marker.pose.position.z = out.position_predict.z;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.06;
    marker.color.r = 1.0f; marker.color.g = 0.3f; marker.color.b = 0.3f;
    marker.color.a = 0.8f;
    marker.lifetime = ros::Duration(0.2);
    marker_pub_.publish(marker);
  }

  std::string det_topic_, track_topic_, marker_topic_;
  // T：提前量；gate：关联门限；q_pos/q_vel/r_meas：Kalman 噪声；max_lost：丢检删除时间
  double T_ = 0.1, gate_ = 0.4, q_pos_ = 0.01, q_vel_ = 0.1, r_meas_ = 0.02, max_lost_ = 0.5;
  cv::KalmanFilter kf_;
  bool initialized_ = false;
  double lost_time_ = 0.0;
  int miss_count_ = 0;
  geometry_msgs::Point last_observed_;
  ros::Time last_stamp_;
  ros::Subscriber sub_;
  ros::Publisher track_pub_, marker_pub_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ft_tracker");
  ros::NodeHandle nh, pnh("~");
  FtTracker tracker(nh, pnh);
  ros::spin();
  return 0;
}
