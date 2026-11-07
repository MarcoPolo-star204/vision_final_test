// ============================================================
// ft_selector_node.cpp 逐行注释版（每一行代码单独一行注释）
// 说明：本文件只用于学习，不参与 catkin_make；生产代码仍是 src/ 下的原文件。
// ============================================================

// 引入 ROS 的 C++ 头文件，提供 ros::NodeHandle、ros::init、ros::spin 等
#include <ros/ros.h>

// 引入 Track 消息，跟踪节点的输出类型
#include <ft_msgs/Track.h>

// 引入 TargetSelect 消息，目标选择节点的输出类型
#include <ft_msgs/TargetSelect.h>

// 定义一个类，把订阅、发布、回调装在一起
class FtSelector {

// public 表示下面的成员外部可以访问
 public:

// 构造函数：创建对象时自动调用
  FtSelector(ros::NodeHandle& nh, ros::NodeHandle& pnh) {

// 从私有命名空间读参数 track_topic，默认 /ft/track
    pnh.param<std::string>("track_topic", track_topic_, "/ft/track");

// 从私有命名空间读参数 target_topic，默认 /ft/target
    pnh.param<std::string>("target_topic", target_topic_, "/ft/target");

// 订阅 track_topic，队列 10，消息到了调用 callback，this 表示当前对象
    sub_ = nh.subscribe(track_topic_, 10, &FtSelector::callback, this);

// 发布 target_topic，消息类型是 ft_msgs::TargetSelect，队列 10
    pub_ = nh.advertise<ft_msgs::TargetSelect>(target_topic_, 10);

// 构造函数结束
  }

// private 表示下面的成员只有类内部能访问
 private:

// 回调函数：收到 Track 消息时由 ROS 自动调用
  void callback(const ft_msgs::TrackConstPtr& msg) {

// 创建一个空的 TargetSelect 消息
    ft_msgs::TargetSelect out;

// 把输入 Track 的 header 原样复制给输出，时间戳/frame 保持一致
    out.header = msg->header;

// valid 直接等于 tracking：只有真正跟住时才有效
    out.valid = msg->tracking;

// 目标 ID 沿用 Track 的 id
    out.target_id = msg->id;

// 当前位置直接复制
    out.position = msg->position;

// 提前量预测位置直接复制
    out.position_predict = msg->position_predict;

// 发布 TargetSelect
    pub_.publish(out);

// 回调结束
  }

// 话题名成员变量
  std::string track_topic_, target_topic_;

// 订阅者对象
  ros::Subscriber sub_;

// 发布者对象
  ros::Publisher pub_;

// 类定义结束，必须有分号
};

// 程序入口
int main(int argc, char** argv) {

// 初始化 ROS 节点，节点名 ft_selector
  ros::init(argc, argv, "ft_selector");

// 创建全局句柄 nh 和私有句柄 pnh
  ros::NodeHandle nh, pnh("~");

// 创建 FtSelector 对象，构造函数会订阅和发布
  FtSelector selector(nh, pnh);

// 进入事件循环，阻塞等待回调
  ros::spin();

// 正常退出
  return 0;

// main 结束
}
