// ============================================================
// ft_tracker_node.cpp 逐行注释版
// 说明：只用于学习，不参与编译；生产代码仍是 src/ 原文件。
// ============================================================

// 引入 ROS 总头文件
#include <ros/ros.h>

// 引入 DetectionArray 消息，跟踪节点的输入
#include <ft_msgs/DetectionArray.h>

// 引入 Track 消息，跟踪节点的输出
#include <ft_msgs/Track.h>

// 引入 Marker 消息，用于 RViz 可视化
#include <visualization_msgs/Marker.h>

// 引入 OpenCV，使用 KalmanFilter
#include <opencv2/opencv.hpp>

// 引入 std::max / std::min
#include <algorithm>

// 引入 std::sqrt
#include <cmath>

// 跟踪节点类
class FtTracker {

// public 成员
 public:

// 构造函数
  FtTracker(ros::NodeHandle& nh, ros::NodeHandle& pnh) {

// 读检测话题
    pnh.param<std::string>("detection_topic", det_topic_, "/ft/detection");

// 读跟踪输出话题
    pnh.param<std::string>("track_topic", track_topic_, "/ft/track");

// 读 marker 话题
    pnh.param<std::string>("marker_topic", marker_topic_, "/ft/debug/marker");

// 读提前量 T
    pnh.param<double>("T", T_, 0.1);

// 读关联门限 gate
    pnh.param<double>("gate", gate_, 0.4);

// 读位置过程噪声 q_pos
    pnh.param<double>("q_pos", q_pos_, 0.01);

// 读速度过程噪声 q_vel
    pnh.param<double>("q_vel", q_vel_, 0.1);

// 读观测噪声 r_meas
    pnh.param<double>("r_meas", r_meas_, 0.02);

// 读丢检删除时间 max_lost
    pnh.param<double>("max_lost", max_lost_, 0.5);

// 初始化 Kalman：6 维状态、3 维观测、0 维控制
    kf_.init(6, 3, 0);

// F 设为单位阵
    cv::setIdentity(kf_.transitionMatrix);

// H 设为单位阵
    cv::setIdentity(kf_.measurementMatrix);

// Q 先清零
    kf_.processNoiseCov = cv::Mat::zeros(6, 6, CV_32F);

// 遍历 0/1/2 三个位置
    for (int i = 0; i < 3; ++i) {

// 位置过程噪声
      kf_.processNoiseCov.at<float>(i, i) = static_cast<float>(q_pos_);

// 速度过程噪声
      kf_.processNoiseCov.at<float>(i + 3, i + 3) = static_cast<float>(q_vel_);

// 循环结束
    }

// R 设为单位阵乘 r_meas
    kf_.measurementNoiseCov = cv::Mat::eye(3, 3, CV_32F) * static_cast<float>(r_meas_);

// P 设为单位阵
    cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(1.0));

// 订阅检测数组
    sub_ = nh.subscribe(det_topic_, 10, &FtTracker::callback, this);

// 发布 Track
    track_pub_ = nh.advertise<ft_msgs::Track>(track_topic_, 10);

// 发布 Marker
    marker_pub_ = nh.advertise<visualization_msgs::Marker>(marker_topic_, 1);

// 构造函数结束
  }

// private 成员
 private:

// 清空轨迹
  void reset() {

// 后验状态清零
    kf_.statePost = cv::Mat::zeros(6, 1, CV_32F);

// 先验状态清零
    kf_.statePre = cv::Mat::zeros(6, 1, CV_32F);

// 标记未初始化
    initialized_ = false;

// 丢检时间清零
    lost_time_ = 0.0;

// reset 结束
  }

// 用第一帧检测初始化
  void initFrom(const geometry_msgs::Pose& pose) {

// 位置 x
    kf_.statePost.at<float>(0) = static_cast<float>(pose.position.x);

// 位置 y
    kf_.statePost.at<float>(1) = static_cast<float>(pose.position.y);

// 位置 z
    kf_.statePost.at<float>(2) = static_cast<float>(pose.position.z);

// 速度 x 先设 0
    kf_.statePost.at<float>(3) = 0.0f;

// 速度 y 先设 0
    kf_.statePost.at<float>(4) = 0.0f;

// 速度 z 先设 0
    kf_.statePost.at<float>(5) = 0.0f;

// 标记已初始化
    initialized_ = true;

// 丢检时间清零
    lost_time_ = 0.0;

// initFrom 结束
  }

// 每帧检测回调
  void callback(const ft_msgs::DetectionArrayConstPtr& msg) {

// 默认 dt 0.05 秒
    double dt = 0.05;

// 如果不是第一帧
    if (!last_stamp_.isZero()) {

// 时间戳回退（bag 循环）
      if (msg->header.stamp < last_stamp_) {

// 记录新时间戳
        last_stamp_ = msg->header.stamp;

// 丢检时间清零
        lost_time_ = 0.0;

// dt 用默认值
        dt = 0.05;

// 时间正常
      } else {

// 两帧时间戳相减
        dt = (msg->header.stamp - last_stamp_).toSec();

// if 结束
      }

// if 结束
    }

// dt 限制在 1ms~200ms
    dt = std::max(0.001, std::min(dt, 0.2));

// 更新上一帧时间
    last_stamp_ = msg->header.stamp;

// 还没初始化
    if (!initialized_) {

// 没有检测就返回
      if (msg->detections.empty()) return;

// 用第一个检测初始化
      initFrom(msg->detections.front().pose);

// if 结束
    }

// F(0,3)=dt：x += vx*dt
    kf_.transitionMatrix.at<float>(0, 3) = static_cast<float>(dt);

// F(1,4)=dt：y += vy*dt
    kf_.transitionMatrix.at<float>(1, 4) = static_cast<float>(dt);

// F(2,5)=dt：z += vz*dt
    kf_.transitionMatrix.at<float>(2, 5) = static_cast<float>(dt);

// 预测
    kf_.predict();

// 最佳检测下标
    int best = -1;

// 最小距离
    double best_dist = 1e9;

// 遍历检测
    for (size_t i = 0; i < msg->detections.size(); ++i) {

// 取第 i 个检测的位置
      const auto& p = msg->detections[i].pose.position;

// x 差
      const double dx = p.x - kf_.statePre.at<float>(0);

// y 差
      const double dy = p.y - kf_.statePre.at<float>(1);

// z 差
      const double dz = p.z - kf_.statePre.at<float>(2);

// 三维距离
      const double d = std::sqrt(dx * dx + dy * dy + dz * dz);

// 更近就更新 best
      if (d < best_dist) { best_dist = d; best = static_cast<int>(i); }

// 循环结束
    }

// 这一帧是否跟住
    bool tracking = false;

// 有检测且距离在门限内
    if (best >= 0 && best_dist <= gate_) {

// 观测向量 3x1
      cv::Mat meas(3, 1, CV_32F);

// 观测 x
      meas.at<float>(0) = static_cast<float>(msg->detections[best].pose.position.x);

// 观测 y
      meas.at<float>(1) = static_cast<float>(msg->detections[best].pose.position.y);

// 观测 z
      meas.at<float>(2) = static_cast<float>(msg->detections[best].pose.position.z);

// 用观测修正
      kf_.correct(meas);

// 标记跟住
      tracking = true;

// 丢检时间清零
      lost_time_ = 0.0;

// 丢检计数清零
      miss_count_ = 0;

// 记录最后观测 x
      last_observed_.x = kf_.statePost.at<float>(0);

// 记录最后观测 y
      last_observed_.y = kf_.statePost.at<float>(1);

// 记录最后观测 z
      last_observed_.z = kf_.statePost.at<float>(2);

// 没命中
    } else {

// 丢检时间累加
      lost_time_ += dt;

// 丢检计数加一
      ++miss_count_;

// 预测与最后观测的 x 差
      const double dx = kf_.statePost.at<float>(0) - last_observed_.x;

// y 差
      const double dy = kf_.statePost.at<float>(1) - last_observed_.y;

// z 差
      const double dz = kf_.statePost.at<float>(2) - last_observed_.z;

// 连续 10 帧没关联，或外推超过 2 米
      if (miss_count_ > 10 || std::sqrt(dx*dx + dy*dy + dz*dz) > 2.0) {

// 重置
        reset();

// 返回
        return;

// if 结束
      }

// else 结束
    }

// 丢检超过 max_lost 也重置
    if (lost_time_ > max_lost_) { reset(); return; }

// 构造 Track
    ft_msgs::Track out;

// header 沿用输入图像
    out.header = msg->header;

// 基础版 id 固定 1
    out.id = 1;

// 跟踪状态
    out.tracking = tracking;

// 位置 x
    out.position.x = kf_.statePost.at<float>(0);

// 位置 y
    out.position.y = kf_.statePost.at<float>(1);

// 位置 z
    out.position.z = kf_.statePost.at<float>(2);

// 速度 x
    out.velocity.x = kf_.statePost.at<float>(3);

// 速度 y
    out.velocity.y = kf_.statePost.at<float>(4);

// 速度 z
    out.velocity.z = kf_.statePost.at<float>(5);

// 提前量 x
    out.position_predict.x = out.position.x + out.velocity.x * T_;

// 提前量 y
    out.position_predict.y = out.position.y + out.velocity.y * T_;

// 提前量 z
    out.position_predict.z = out.position.z + out.velocity.z * T_;

// 发布 Track
    track_pub_.publish(out);

// 构造 Marker
    visualization_msgs::Marker marker;

// header
    marker.header = out.header;

// 命名空间
    marker.ns = "ft_track";

// id
    marker.id = 0;

// 类型：球
    marker.type = visualization_msgs::Marker::SPHERE;

// 动作：添加
    marker.action = visualization_msgs::Marker::ADD;

// 位置 x
    marker.pose.position.x = out.position_predict.x;

// 位置 y
    marker.pose.position.y = out.position_predict.y;

// 位置 z
    marker.pose.position.z = out.position_predict.z;

// 朝向
    marker.pose.orientation.w = 1.0;

// 大小 6cm
    marker.scale.x = marker.scale.y = marker.scale.z = 0.06;

// 颜色：红
    marker.color.r = 1.0f; marker.color.g = 0.3f; marker.color.b = 0.3f;

// 透明度
    marker.color.a = 0.8f;

// 生命周期 0.2s
    marker.lifetime = ros::Duration(0.2);

// 发布 marker
    marker_pub_.publish(marker);

// 回调结束
  }

// 话题名
  std::string det_topic_, track_topic_, marker_topic_;

// T/gate/q/r/max_lost
  double T_ = 0.1, gate_ = 0.4, q_pos_ = 0.01, q_vel_ = 0.1, r_meas_ = 0.02, max_lost_ = 0.5;

// Kalman 滤波器
  cv::KalmanFilter kf_;

// 是否初始化
  bool initialized_ = false;

// 丢检时间
  double lost_time_ = 0.0;

// 连续丢检帧数
  int miss_count_ = 0;

// 最后观测位置
  geometry_msgs::Point last_observed_;

// 上一帧时间
  ros::Time last_stamp_;

// 订阅者
  ros::Subscriber sub_;

// 发布者
  ros::Publisher track_pub_, marker_pub_;

// 类结束
};

// 主函数
int main(int argc, char** argv) {

// 初始化 ROS
  ros::init(argc, argv, "ft_tracker");

// 句柄
  ros::NodeHandle nh, pnh("~");

// 创建跟踪对象
  FtTracker tracker(nh, pnh);

// 进入回调循环
  ros::spin();

// 正常退出
  return 0;

// main 结束
}
