// ============================================================
// ft_detector_node.cpp 逐行注释版 · 第 1 批
// 覆盖：头文件、结构体、类、构造函数、回调注册、buildMasks、
//       detectBars、buildPair、pairBars、barCorners。
// 说明：这是生产代码，参与 catkin_make；每一行代码上方都有一行注释。
// ============================================================

// 引入 ROS 总头文件：NodeHandle、Publisher、Subscriber、spin 等
#include <ros/ros.h>

// 引入压缩图像消息类型，对应 /hk_camera/image_raw/compressed
#include <sensor_msgs/CompressedImage.h>

// 引入相机信息消息类型，里面有 K、D、frame_id
#include <sensor_msgs/CameraInfo.h>

// 引入普通图像消息类型，用来发布 /ft/debug/image
#include <sensor_msgs/Image.h>

// 引入 Header 类型，消息的时间戳和坐标系都放在这里
#include <std_msgs/Header.h>

// 引入 TF 变换消息类型，用来发布 camera -> armor
#include <geometry_msgs/TransformStamped.h>

// 引入 cv_bridge，把 ROS 图像和 cv::Mat 互相转换
#include <cv_bridge/cv_bridge.h>

// 引入动态参数服务器，支持 rqt_reconfigure 在线调参
#include <dynamic_reconfigure/server.h>

// 引入自动生成的动态参数配置类型
#include <ft_detector/DetectorConfig.h>

// 引入单条检测消息
#include <ft_msgs/Detection.h>

// 引入检测数组消息，发布到 /ft/detection
#include <ft_msgs/DetectionArray.h>

// 引入 TF 广播器，用来发 camera -> armor_<id>
#include <tf2_ros/transform_broadcaster.h>

// 引入 3x3 矩阵，用来把旋转矩阵转四元数
#include <tf2/LinearMath/Matrix3x3.h>

// 引入四元数类型
#include <tf2/LinearMath/Quaternion.h>

// 引入 OpenCV 总头文件
#include <opencv2/opencv.hpp>

// 引入 boost::bind，用来绑定动态参数回调
#include <boost/bind.hpp>

// 引入 std::sort、std::min、std::max
#include <algorithm>

// 引入 std::sqrt、std::fabs、std::cos、std::sin
#include <cmath>

// 引入 std::string
#include <string>

// 引入 std::vector
#include <vector>

// 匿名命名空间：里面的东西只在本文件可见
namespace {

// 定义“一根灯条”的数据结构
struct LightBar {

// 灯条的最小外接旋转矩形
  cv::RotatedRect rect;

// 灯条中心点
  cv::Point2f center;

// 灯条短边（宽度）
  float width;

// 灯条长边（高度）
  float height;

// 灯条角度，后面会归一到 [-45,45)
  float angle;

// 结构体定义结束，必须有分号
};

// 定义“一对灯条”的数据结构
struct BarPair {

// 左灯条在 bars 数组里的下标
  int left;

// 右灯条在 bars 数组里的下标
  int right;

// 这一对是不是大装甲板
  bool is_large;

// 结构体定义结束
};

// 按 x 从小到大排序的比较函数，替代 lambda
bool comparePointX(const cv::Point2f& a, const cv::Point2f& b) {

// 如果 a.x 小于 b.x，a 排在 b 前面
  return a.x < b.x;

// 函数结束
}

// 把一个点绕原点旋转 angle_deg 度，返回旋转后的点
cv::Point2f rotatePoint(const cv::Point2f& p, float angle_deg) {

// 角度转弧度
  const float a = angle_deg * static_cast<float>(CV_PI) / 180.f;

// 标准二维旋转公式
  return cv::Point2f(p.x * std::cos(a) - p.y * std::sin(a),

// 第二行：y 分量
                     p.x * std::sin(a) + p.y * std::cos(a));

// 函数结束
}

// 匿名命名空间结束
}  // namespace

// 检测节点类：把参数、订阅、发布、回调装在一起
class FtDetector {

// public 成员外部可以访问
 public:

// 构造函数：创建对象时自动调用
  FtDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh) {

// 读图像话题名，默认压缩图话题
    pnh.param<std::string>("image_topic", image_topic_, "/hk_camera/image_raw/compressed");

// 读 camera_info 话题名
    pnh.param<std::string>("camera_info_topic", info_topic_, "/hk_camera/camera_info");

// 读检测结果话题名
    pnh.param<std::string>("detection_topic", det_topic_, "/ft/detection");

// 读调试图话题名
    pnh.param<std::string>("debug_topic", debug_topic_, "/ft/debug/image");

// 读图像队列长度，默认 2
    pnh.param<int>("queue_size", queue_size_, 2);

// 读大装甲板宽度，单位米
    pnh.param<double>("large_armor_width", large_w_, 0.235);

// 读大装甲板高度
    pnh.param<double>("large_armor_height", large_h_, 0.127);

// 读小装甲板宽度
    pnh.param<double>("small_armor_width", small_w_, 0.120);

// 读小装甲板高度
    pnh.param<double>("small_armor_height", small_h_, 0.125);

// 读大小装甲板判定比例
    pnh.param<double>("large_gap_ratio", large_gap_ratio_, 1.4);

// 读配对最大间距比例，防止跨板配对
    pnh.param<double>("max_gap_ratio", max_gap_ratio_, 3.0);

// 读重复检测合并距离
    pnh.param<double>("merge_distance", merge_distance_, 0.25);

// 读检测最远距离
    pnh.param<double>("max_distance", max_distance_, 8.0);

// 读检测最近距离
    pnh.param<double>("min_distance", min_distance_, 1.0);

// 读重投影误差上限
    pnh.param<double>("max_reproj_error", max_reproj_error_, 30.0);

// 读反光过滤亮度上限
    pnh.param<double>("max_body_v", max_body_v_, 90.0);

// 读灯条最小长宽比
    pnh.param<double>("min_aspect", min_aspect_, 1.5);

// 读灯条最大长宽比
    pnh.param<double>("max_aspect", max_aspect_, 12.0);

// 读灯条最小填充率
    pnh.param<double>("min_fill", min_fill_, 0.35);

// 读没有 camera_info 时的回退 frame
    pnh.param<std::string>("fallback_frame", fallback_frame_, "camera");

// 读回退相机内参矩阵
    pnh.getParam("fallback_camera_matrix", fallback_K_);

// 读回退畸变系数
    pnh.getParam("fallback_distortion", fallback_D_);

// 订阅压缩图像，队列 queue_size_，回调 imageCallback
    image_sub_ = nh.subscribe(image_topic_, queue_size_, &FtDetector::imageCallback, this);

// 订阅 camera_info，队列 1，回调 infoCallback
    info_sub_ = nh.subscribe(info_topic_, 1, &FtDetector::infoCallback, this);

// 发布调试图，队列 1
    debug_pub_ = nh.advertise<sensor_msgs::Image>(debug_topic_, 1);

// 发布检测数组，队列 10
    det_pub_ = nh.advertise<ft_msgs::DetectionArray>(det_topic_, 10);

// 在堆上创建 TF 广播器
    tf_broadcaster_ = new tf2_ros::TransformBroadcaster();

// 在堆上创建动态参数服务器
    dr_server_ = new dynamic_reconfigure::Server<ft_detector::DetectorConfig>(pnh);

// 绑定动态参数回调，_1/_2 是占位符
    dr_server_->setCallback(boost::bind(&FtDetector::reconfigureCallback, this, _1, _2));

// 打印日志，确认话题名
    ROS_INFO_STREAM("ft_detector: image=" << image_topic_ << ", info=" << info_topic_);

// 构造函数结束
  }

// 析构函数：对象销毁时释放 new 出来的东西
  ~FtDetector() {

// 释放 TF 广播器
    delete tf_broadcaster_;

// 释放动态参数服务器
    delete dr_server_;

// 析构函数结束
  }

// private 成员只有类内部能访问
 private:

// camera_info 回调：缓存最新一份内参
  void infoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {

// 把消息内容拷贝到成员变量
    camera_info_ = *msg;

// 标记已经收到过 camera_info
    has_info_ = true;

// 回调结束
  }

// 动态参数回调：rqt 拖动滑块时自动调用
  void reconfigureCallback(ft_detector::DetectorConfig& cfg, uint32_t level) {

// level 用不到，显式 void 掉，避免编译警告
    (void)level;

// 把新参数保存到成员变量
    cfg_ = cfg;

// 回调结束
  }

// buildMasks：HSV -> 红/蓝两张 mask
  void buildMasks(const cv::Mat& hsv, cv::Mat& red_mask, cv::Mat& blue_mask) {  // 输入 HSV 图，输出红/蓝两张黑白 mask

// 定义两张红色临时 mask
    cv::Mat r1, r2;  // r1=红色第一段，r2=红色第二段（红色跨 0 度）

// 红色第一段：H 在 0 附近
    cv::inRange(hsv, cv::Scalar(cfg_.red_h_min, cfg_.s_min, cfg_.v_min),  // 红色第一段：H 在下限附近

// S/V 上限固定 255，只卡下限
                cv::Scalar(cfg_.red_h_max, 255, 255), r1);  // S/V 上限固定 255，只卡下限

// 红色第二段：H 在 170~180
    cv::inRange(hsv, cv::Scalar(cfg_.red_h_min2, cfg_.s_min, cfg_.v_min),  // 红色第二段：H 在 170~180

// 第二段同样只卡 S/V 下限
                cv::Scalar(cfg_.red_h_max2, 255, 255), r2);  // 同样只卡 S/V 下限

// 两段红色按位或，合并成一张 red_mask
    cv::bitwise_or(r1, r2, red_mask);  // 两段红色合并成一张 red_mask

// 蓝色只有一段：H 100~130
    cv::inRange(hsv, cv::Scalar(cfg_.blue_h_min, cfg_.s_min, cfg_.v_min),  // 蓝色只有一段：H 100~130

// 输出蓝 mask
                cv::Scalar(cfg_.blue_h_max, 255, 255), blue_mask);  // 输出蓝 mask

// buildMasks 结束
  }

// detectBars：从颜色 mask 里提取竖直灯条
  void detectBars(const cv::Mat& mask, std::vector<LightBar>& bars) {

// 3x3 矩形结构元素
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

// 开运算：先腐蚀后膨胀，去掉小噪点
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

// 闭运算：先膨胀后腐蚀，补小洞
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

// 竖向结构元素：宽 2、高 15
    cv::Mat vertical = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 15));

// 竖向闭运算：把断成几截的灯条连起来
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, vertical);

// 存放所有轮廓
    std::vector<std::vector<cv::Point>> contours;

// 只找外轮廓，只存近似点
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

// 遍历每个轮廓
    for (size_t ci = 0; ci < contours.size(); ++ci) {

// 取第 ci 个轮廓的引用，不拷贝
      const std::vector<cv::Point>& c = contours[ci];

// 算轮廓面积
      const double area = cv::contourArea(c);

// 面积太小，跳过
      if (area < cfg_.min_area) continue;

// 求最小外接旋转矩形
      cv::RotatedRect rect = cv::minAreaRect(c);

// 取矩形宽
      const float rw = rect.size.width;

// 取矩形高
      const float rh = rect.size.height;

// 宽或高太小，跳过
      if (rw < 1.f || rh < 1.f) continue;

// 判断宽是不是长边
      const bool width_is_long = rw >= rh;

// 取角度绝对值
      const float long_axis_angle = std::fabs(rect.angle);

// 长边必须接近竖直：宽是长边时角度要大于 45 度，否则小于 45 度
      const bool long_is_vertical = width_is_long ? (long_axis_angle > 45.f)

// 第二行：高是长边的情况
                                                  : (long_axis_angle < 45.f);

// 不是竖直灯条，跳过（排除横向状态灯）
      if (!long_is_vertical) continue;

// 短边作为宽度
      const float w = std::min(rw, rh);

// 长边作为高度
      const float h = std::max(rw, rh);

// 长宽比
      const float aspect = h / w;

// 长宽比不在范围内，跳过
      if (aspect < min_aspect_ || aspect > max_aspect_) continue;

// 填充率 = 轮廓面积 / 外接矩形面积
      const float fill = static_cast<float>(area / (rw * rh + 1e-6));

// 填充率太低，跳过
      if (fill < min_fill_) continue;

// 创建一根灯条
      LightBar bar;

// 保存旋转矩形
      bar.rect = rect;

// 保存中心
      bar.center = rect.center;

// 保存短边
      bar.width = w;

// 保存长边
      bar.height = h;

// 保存角度
      bar.angle = rect.angle;

// 角度小于 -45 度，加 90 度
      while (bar.angle < -45.f) bar.angle += 90.f;

// 角度大于等于 45 度，减 90 度
      while (bar.angle >= 45.f) bar.angle -= 90.f;

// 把这根灯条放进结果数组
      bars.push_back(bar);

// 轮廓循环结束
    }

// detectBars 结束
  }

// buildPair：判断两根灯条能不能组成一块装甲板
  bool buildPair(const LightBar& a, const LightBar& b, bool& is_large) {

// 两根灯条平均高度
    const float avg_h = 0.5f * (a.height + b.height);

// 两根灯条平均宽度
    const float avg_w = 0.5f * (a.width + b.width);

// 高度差比例
    const float height_diff = std::fabs(a.height - b.height) / (avg_h + 1e-6f);

// 角度差
    const float angle_diff = std::fabs(a.angle - b.angle);

// 中心高度差比例
    const float center_dy = std::fabs(a.center.y - b.center.y) / (avg_h + 1e-6f);

// 水平间距
    const float gap = std::fabs(b.center.x - a.center.x);

// 高度差太大，不配
    if (height_diff > cfg_.max_height_diff) return false;

// 角度差太大，不配
    if (angle_diff > cfg_.max_angle_diff) return false;

// 中心一高一低，不配
    if (center_dy > 0.5f) return false;

// 间距太小或太大，不配
    if (gap < 0.7f * avg_w || gap > cfg_.max_gap) return false;

// 间距 / 高度
    const float gap_ratio = gap / (avg_h + 1e-6f);

// gap_ratio 太小是同一根灯条，太大是跨板，都不配
    if (gap_ratio < 0.4f || gap_ratio > max_gap_ratio_) return false;

// 大于阈值判为大装甲板
    is_large = gap_ratio > large_gap_ratio_;

// 能配
    return true;

// buildPair 结束
  }

// pairBars：一帧多根灯条的贪心配对
  void pairBars(const std::vector<LightBar>& bars, std::vector<BarPair>& pairs) {

// used 标记每根灯条有没有被用掉
    std::vector<char> used(bars.size(), 0);

// 遍历每根灯条 i
    for (size_t i = 0; i < bars.size(); ++i) {

// i 已经用掉，跳过
      if (used[i]) continue;

// 最佳 j，初始 -1 表示没找到
      int best = -1;

// 最佳得分，初始很大
      float best_score = 1e9f;

// 最佳 j 对应的大小板标记
      bool best_large = false;

// 从 i 后面找 j
      for (size_t j = i + 1; j < bars.size(); ++j) {

// j 已经用掉，跳过
        if (used[j]) continue;

// 这一对是不是大板
        bool large = false;

// 不能配，跳过
        if (!buildPair(bars[i], bars[j], large)) continue;

// 得分 = 中心高度差 + 角度差
        const float score = std::fabs(bars[i].center.y - bars[j].center.y) +

// 第二行：角度差
                            std::fabs(bars[i].angle - bars[j].angle);

// 得分更小，更新最佳
        if (score < best_score) { best_score = score; best = static_cast<int>(j); best_large = large; }

// j 循环结束
      }

// 找到可配的 j
      if (best >= 0) {

// 标记 i 已用
        used[i] = true;

// 标记 best 已用
        used[best] = true;

// 创建一对
        BarPair p;

// 左下标
        p.left = static_cast<int>(i);

// 右下标
        p.right = best;

// 大小板标记
        p.is_large = best_large;

// 放进结果
        pairs.push_back(p);

// if 结束
      }

// i 循环结束
    }

// pairBars 结束
  }

// barCorners：取灯条外侧两个角点
  static void barCorners(const LightBar& bar, cv::Point2f& left_top, cv::Point2f& left_bottom,

// 参数第二行：右侧两个角点 + 取左/右标志
                         cv::Point2f& right_top, cv::Point2f& right_bottom, bool take_left) {

// 四个角点的 C 数组
    cv::Point2f pts[4];

// 把旋转矩形四个角写进数组
    bar.rect.points(pts);

// 用数组初始化 vector
    std::vector<cv::Point2f> p(pts, pts + 4);

// 按 x 排序
    std::sort(p.begin(), p.end(), comparePointX);

// 取最左两个点
    cv::Point2f a = p[0], b = p[1];

// 取左灯条
    if (take_left) {

// y 小的当左上
      left_top = a.y < b.y ? a : b;

// y 大的当左下
      left_bottom = a.y < b.y ? b : a;

// 否则取右灯条
    } else {

// 取最右两个点
      cv::Point2f c = p[2], d = p[3];

// y 小的当右上
      right_top = c.y < d.y ? c : d;

// y 大的当右下
      right_bottom = c.y < d.y ? d : c;

// else 结束
    }

// barCorners 结束
  }


// ============================================================
// 第 2 批：imageCallback 1–8 步 + 成员变量 + main
// ============================================================

// 主回调：每来一帧压缩图，走完整条检测链路
  void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg) {

// 定义一个 cv_bridge 图像指针，用来接解码结果
    cv_bridge::CvImagePtr cv_ptr;

// try 保护解码，失败不要让节点崩
    try {

// 把压缩图解码成 bgr8 的 cv::Mat
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

// 捕获 cv_bridge 异常
    } catch (const cv_bridge::Exception& e) {

// 打印错误，1 秒最多一次，避免刷屏
      ROS_ERROR_STREAM_THROTTLE(1.0, "cv_bridge: " << e.what());

// 解码失败，直接返回
      return;

// catch 结束
    }

// 组装消息头
    std_msgs::Header header;

// 时间戳必须用图像自带的，不能用 now
    header.stamp = msg->header.stamp;

// 如果有 camera_info 且 frame 不为空
    if (has_info_ && !camera_info_.header.frame_id.empty()) {

// 优先用 camera_info 的 frame
      header.frame_id = camera_info_.header.frame_id;

// 否则如果图像自己有 frame
    } else if (!msg->header.frame_id.empty()) {

// 用图像的 frame
      header.frame_id = msg->header.frame_id;

// 都没有
    } else {

// 用 YAML 里的回退 frame
      header.frame_id = fallback_frame_;

// if 结束
    }

// 定义 HSV 图
    cv::Mat hsv;

// BGR 转 HSV
    cv::cvtColor(cv_ptr->image, hsv, cv::COLOR_BGR2HSV);

// 定义红/蓝 mask
    cv::Mat red_mask, blue_mask;

// 调 buildMasks 得到两张 mask
    buildMasks(hsv, red_mask, blue_mask);

// 定义红/蓝灯条数组
    std::vector<LightBar> red_bars, blue_bars;

// 从红 mask 找灯条
    detectBars(red_mask, red_bars);

// 从蓝 mask 找灯条
    detectBars(blue_mask, blue_bars);

// debug 图要 clone，不能画到原图
    cv::Mat debug = cv_ptr->image.clone();

// 创建检测数组
    ft_msgs::DetectionArray out;

// 检测数组的 header 与图像一致
    out.header = header;

// 定义内参矩阵 K 和畸变 D
    cv::Mat K, D;

// 有 camera_info 就用它
    if (has_info_) {

// 把 camera_info 的 K 拷贝成 3x3 double 矩阵
      K = cv::Mat(3, 3, CV_64F, const_cast<double*>(camera_info_.K.data())).clone();

// 如果有畸变系数
      D = camera_info_.D.empty()

// 没有畸变就传空 Mat
              ? cv::Mat()

// 有畸变就拷贝成 1xN
              : cv::Mat(1, static_cast<int>(camera_info_.D.size()), CV_64F,
                        const_cast<double*>(camera_info_.D.data())).clone();

// 没有 camera_info，但 YAML 有 9 个内参
    } else if (fallback_K_.size() == 9) {

// 用 YAML 的 K
      K = cv::Mat(3, 3, CV_64F, fallback_K_.data()).clone();

// 如果 YAML 有畸变
      if (!fallback_D_.empty()) {

// 用 YAML 的 D
        D = cv::Mat(1, static_cast<int>(fallback_D_.size()), CV_64F, fallback_D_.data()).clone();

// if 结束
      }

// if 结束
    }

// 当前帧检测编号，从 0 开始
    int id = 0;

// color=0 红色，color=1 蓝色
    for (int color = 0; color < 2; ++color) {

// 根据 color 选红或蓝灯条数组
      const std::vector<LightBar>& bars = (color == 0) ? red_bars : blue_bars;

// 存放这一种颜色的配对
      std::vector<BarPair> pairs;

// 调 pairBars 做配对
      pairBars(bars, pairs);

// 遍历每一对
      for (size_t k = 0; k < pairs.size(); ++k) {

// 取左灯条
        const LightBar& left = bars[pairs[k].left];

// 取右灯条
        const LightBar& right = bars[pairs[k].right];

// 保证 b1 在左边
        const LightBar& b1 = left.center.x < right.center.x ? left : right;

// 保证 b2 在右边
        const LightBar& b2 = left.center.x < right.center.x ? right : left;

// 定义四个角点
        cv::Point2f lt, lb, rt, rb;

// 从左灯条取左侧两个角
        barCorners(b1, lt, lb, rt, rb, true);

// 定义右灯条四个临时角点
        cv::Point2f lt2, lb2, rt2, rb2;

// 从右灯条取右侧两个角
        barCorners(b2, lt2, lb2, rt2, rb2, false);

// 按左上、右上、右下、左下排好图像点
        std::vector<cv::Point2f> img_pts = {lt, rt2, rb2, lb};

// 这一对是不是大装甲板
        const bool is_large = pairs[k].is_large;

// 大板用大宽度，小板用小宽度
        const double W = is_large ? large_w_ : small_w_;

// 大板用大高度，小板用小高度
        const double H = is_large ? large_h_ : small_h_;

// 物体点：左上、右上、右下、左下，单位米
        std::vector<cv::Point3f> obj_pts = {

// 左上
            {0.f, 0.f, 0.f}, {static_cast<float>(W), 0.f, 0.f},

// 右上、右下
            {static_cast<float>(W), static_cast<float>(H), 0.f},

// 左下
            {0.f, static_cast<float>(H), 0.f}};

// 求四个角点的包围框
        cv::Rect pair_box = cv::boundingRect(img_pts);

// 横向往里缩四分之一
        const int mx = std::max(1, pair_box.width / 4);

// 纵向往里缩四分之一
        const int my = std::max(1, pair_box.height / 4);

// 左边往里缩
        pair_box.x += mx; pair_box.y += my;

// 宽高各缩 2*mx / 2*my
        pair_box.width = std::max(1, pair_box.width - 2 * mx);

// 高度也缩
        pair_box.height = std::max(1, pair_box.height - 2 * my);

// 检查缩放后的框还在图像内
        if (pair_box.x >= 0 && pair_box.y >= 0 &&

// 右边界不越界
            pair_box.x + pair_box.width <= hsv.cols &&

// 下边界不越界
            pair_box.y + pair_box.height <= hsv.rows) {

// 算框内平均 V（亮度）
          const double body_v = cv::mean(hsv(pair_box))[2];

// 太亮认为是反光，跳过
          if (body_v > max_body_v_) { ++id; continue; }

// if 结束
        }

// 红色画红框，蓝色画蓝框
        cv::Scalar color_draw = (color == 0) ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);

// 画上边
        cv::line(debug, lt, rt2, color_draw, 2);

// 画右边
        cv::line(debug, rt2, rb2, color_draw, 2);

// 画下边
        cv::line(debug, rb2, lb, color_draw, 2);

// 画左边
        cv::line(debug, lb, lt, color_draw, 2);

// 定义灯条四点数组
        cv::Point2f pts[4];

// 取左灯条四个角
        left.rect.points(pts);

// 画左灯条绿色框
        for (int i = 0; i < 4; ++i) cv::line(debug, pts[i], pts[(i + 1) % 4], cv::Scalar(0, 255, 0), 1);

// 创建一条检测
        ft_msgs::Detection det;

// 帧内 id
        det.id = static_cast<uint32_t>(id);

// 颜色
        det.is_red = (color == 0);

// 大小
        det.is_large_armor = is_large;

// 有内参才做 PnP
        if (has_info_ && K.rows == 3) {

// 旋转向量和平移向量
          cv::Vec3d rvec, tvec;

// 优先用 IPPE
          try {

// 平面目标 PnP
            cv::solvePnP(obj_pts, img_pts, K, D, rvec, tvec, false, cv::SOLVEPNP_IPPE);

// IPPE 失败
          } catch (const cv::Exception& e1) {

// 退回 ITERATIVE
            try {

// 迭代法 PnP
              cv::solvePnP(obj_pts, img_pts, K, D, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);

// 两种都失败
            } catch (const cv::Exception& e2) {

// 打印警告
              ROS_WARN_STREAM_THROTTLE(1.0, "solvePnP failed: " << e1.what() << " / " << e2.what());

// 跳过这一对
              ++id;
              continue;

// catch 结束
            }

// catch 结束
          }

// 距离太近/太远跳过
          if (tvec[2] < min_distance_ || tvec[2] > max_distance_) { ++id; continue; }

// 存放重投影点
          std::vector<cv::Point2f> reproj;

// 把物体点投回图像
          cv::projectPoints(obj_pts, rvec, tvec, K, D, reproj);

// 误差累加器
          double err = 0.0;

// 遍历四个点
          for (size_t pi = 0; pi < reproj.size(); ++pi) {

// 累加重投影误差
            err += cv::norm(reproj[pi] - img_pts[pi]);

// 循环结束
          }

// 取平均误差
          err /= static_cast<double>(reproj.size());

// 误差太大跳过
          if (err > max_reproj_error_) { ++id; continue; }

// 填位置 x
          det.pose.position.x = tvec[0];

// 填位置 y
          det.pose.position.y = tvec[1];

// 填位置 z
          det.pose.position.z = tvec[2];

// 旋转矩阵
          cv::Mat R;

// 旋转向量转旋转矩阵
          cv::Rodrigues(rvec, R);

// 用旋转矩阵构造 tf2 矩阵
          tf2::Matrix3x3 m(R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),

// 第二行
                           R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),

// 第三行
                           R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2));

// 四元数
          tf2::Quaternion q;

// 从矩阵取四元数
          m.getRotation(q);

// 归一化
          q.normalize();

// 填四元数 x
          det.pose.orientation.x = q.x();

// 填四元数 y
          det.pose.orientation.y = q.y();

// 填四元数 z
          det.pose.orientation.z = q.z();

// 填四元数 w
          det.pose.orientation.w = q.w();

// 坐标轴三个端点
          std::vector<cv::Point3f> axes = {{0.f, 0.f, 0.f}, {0.03f, 0.f, 0.f},

// 第二行
                                           {0.f, 0.03f, 0.f}, {0.f, 0.f, 0.03f}};

// 投影后的坐标轴
          std::vector<cv::Point2f> proj;

// 把坐标轴投回图像
          cv::projectPoints(axes, rvec, tvec, K, D, proj);

// 画 x 轴红色
          cv::line(debug, proj[0], proj[1], cv::Scalar(0, 0, 255), 2);

// 画 y 轴绿色
          cv::line(debug, proj[0], proj[2], cv::Scalar(0, 255, 0), 2);

// 画 z 轴蓝色
          cv::line(debug, proj[0], proj[3], cv::Scalar(255, 0, 0), 2);

// TF 变换
          geometry_msgs::TransformStamped tf;

// TF 头
          tf.header = header;

// 子坐标系 armor_id
          tf.child_frame_id = "armor_" + std::to_string(id);

// 平移 x
          tf.transform.translation.x = tvec[0];

// 平移 y
          tf.transform.translation.y = tvec[1];

// 平移 z
          tf.transform.translation.z = tvec[2];

// 旋转
          tf.transform.rotation = det.pose.orientation;

// 发 TF
          tf_broadcaster_->sendTransform(tf);

// if 结束
        }

// 在调试图上写 L/S + R/B + id
        cv::putText(debug, (is_large ? "L" : "S") + std::string(color == 0 ? "R" : "B") +

// 第二行
                               " " + std::to_string(id),

// 文字位置
                    cv::Point(static_cast<int>(lt.x), static_cast<int>(lt.y) - 6),

// 字体
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, color_draw, 1);

// 重复标记
        bool duplicate = false;

// 有内参才做 3D 去重
        if (has_info_) {

// 遍历已有检测
          for (size_t ei = 0; ei < out.detections.size(); ++ei) {

// 取已有检测
            const ft_msgs::Detection& existing = out.detections[ei];

// 颜色不同不算重复
            if (existing.is_red != det.is_red) continue;

// x 差
            const double dx = existing.pose.position.x - det.pose.position.x;

// y 差
            const double dy = existing.pose.position.y - det.pose.position.y;

// z 差
            const double dz = existing.pose.position.z - det.pose.position.z;

// 距离小于阈值算重复
            if (std::sqrt(dx * dx + dy * dy + dz * dz) < merge_distance_) {

// 标记重复
              duplicate = true;

// 跳出
              break;

// if 结束
            }

// 循环结束
          }

// if 结束
        }

// 重复就跳过
        if (duplicate) { ++id; continue; }

// 放进检测数组
        out.detections.push_back(det);

// id 自增
        ++id;

// 配对循环结束
      }

// 颜色循环结束
    }

// 没有 camera_info
    if (!has_info_) {

// 在调试图上提示
      cv::putText(debug, "no camera_info: PnP skipped", cv::Point(20, 30),

// 字体
                  cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);

// if 结束
    }

// 调试图消息
    sensor_msgs::Image debug_msg;

// cv::Mat 转 ROS Image
    cv_bridge::CvImage(header, "bgr8", debug).toImageMsg(debug_msg);

// 发布调试图
    debug_pub_.publish(debug_msg);

// 发布检测数组
    det_pub_.publish(out);

// 回调结束
  }

// 话题名
  std::string image_topic_, info_topic_, det_topic_, debug_topic_;

// 图像队列
  int queue_size_ = 2;

// 是否收到 camera_info
  bool has_info_ = false;

// 大板宽高
  double large_w_ = 0.235, large_h_ = 0.127;

// 小板宽高
  double small_w_ = 0.120, small_h_ = 0.125;

// 大小板比例
  double large_gap_ratio_ = 1.4;

// 配对最大比例
  double max_gap_ratio_ = 3.0;

// 去重距离
  double merge_distance_ = 0.25;

// 最远距离
  double max_distance_ = 8.0;

// 最近距离
  double min_distance_ = 1.0;

// 重投影阈值
  double max_reproj_error_ = 30.0;

// 反光亮度
  double max_body_v_ = 90.0;

// 长宽比/填充率
  double min_aspect_ = 1.5, max_aspect_ = 12.0, min_fill_ = 0.35;

// 回退 frame
  std::string fallback_frame_ = "camera";

// 回退内参/畸变
  std::vector<double> fallback_K_, fallback_D_;

// 缓存 camera_info
  sensor_msgs::CameraInfo camera_info_;

// 动态参数
  ft_detector::DetectorConfig cfg_;

// 订阅者
  ros::Subscriber image_sub_, info_sub_;

// 发布者
  ros::Publisher debug_pub_, det_pub_;

// TF 广播器
  tf2_ros::TransformBroadcaster* tf_broadcaster_;

// 动态参数服务器
  dynamic_reconfigure::Server<ft_detector::DetectorConfig>* dr_server_;

// 类结束
};

// 主函数
int main(int argc, char** argv) {

// 初始化 ROS
  ros::init(argc, argv, "ft_detector");

// 全局/私有句柄
  ros::NodeHandle nh, pnh("~");

// 创建检测对象
  FtDetector detector(nh, pnh);

// 进入回调循环
  ros::spin();

// 正常退出
  return 0;

// main 结束
}
