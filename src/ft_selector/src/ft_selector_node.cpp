#include <ros/ros.h>
#include <ft_msgs/Track.h>
#include <ft_msgs/TargetSelect.h>

/*
 * ft_selector_node.cpp
 * ------------------------------------------------------------------
 * 基础版目标选择：把 /ft/track 转成 /ft/target。
 * 当前只做“跟踪有效就有效”，不做多车选择/旋转模型（那是进阶）。
 */
class FtSelector {
 public:
  FtSelector(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    pnh.param<std::string>("track_topic", track_topic_, "/ft/track");
    pnh.param<std::string>("target_topic", target_topic_, "/ft/target");
    sub_ = nh.subscribe(track_topic_, 10, &FtSelector::callback, this);
    pub_ = nh.advertise<ft_msgs::TargetSelect>(target_topic_, 10);
  }
 private:
  // valid 直接跟 tracking 走：纯预测时 valid=false，调用方不能用。
  void callback(const ft_msgs::TrackConstPtr& msg) {
    ft_msgs::TargetSelect out;
    out.header = msg->header;
    out.valid = msg->tracking;
    out.target_id = msg->id;
    out.position = msg->position;
    out.position_predict = msg->position_predict;
    pub_.publish(out);
  }
  std::string track_topic_, target_topic_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ft_selector");
  ros::NodeHandle nh, pnh("~");
  FtSelector selector(nh, pnh);
  ros::spin();
  return 0;
}
