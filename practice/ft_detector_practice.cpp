// ============================================================
// ft_detector_node.cpp 挖空练习（不参与编译）
// 把 ______ 换成正确代码；答案在《视觉考核-挖空练习答案.pptx》
// 行号对应 src/ft_detector/src/ft_detector_node.cpp 的逐行注释版
// ============================================================

// BLANK 01 · 第 620 行附近
// BGR 图转 HSV，颜色阈值必须在 HSV 上做
cv::cvtColor(cv_ptr->image, hsv, ______);

// BLANK 02 · 第 288 行附近
// 红色跨 0°，这是第二段的下限
cv::inRange(hsv, cv::Scalar(cfg_.______, cfg_.s_min, cfg_.v_min),
            cv::Scalar(cfg_.red_h_max2, 255, 255), r2);

// BLANK 03 · 第 293 行附近
// 两段红色 mask 合并
cv::______(r1, r2, red_mask);

// BLANK 04 · 第 321 行附近
// 竖向核闭运算：把断成几截的灯条连起来
cv::morphologyEx(mask, mask, cv::______, vertical);

// BLANK 05 · 第 327 行附近
// 只找外轮廓，只存近似点
cv::______(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

// BLANK 06 · 第 342 行附近
// 求轮廓的最小外接旋转矩形
cv::RotatedRect rect = cv::______(c);

// BLANK 07 · 第 439 行附近
// 间距 / 平均高度：分大小板，也防跨板
const float gap_ratio = gap / (______ + 1e-6f);

// BLANK 08 · 第 864 行附近
// 4 个物体点 + 4 个图像点 → rvec/tvec
cv::______(obj_pts, img_pts, K, D, rvec, tvec, false, cv::SOLVEPNP_IPPE);

// BLANK 09 · 第 931 行附近
// 旋转向量 → 旋转矩阵
cv::______(rvec, R);

// BLANK 10 · 第 946 行附近
// 旋转矩阵 → 四元数
m.______(q);
q.normalize();

// BLANK 11 · 第 1006 行附近
// 把 camera → armor 的变换发出去
tf_broadcaster_->______(tf);

// BLANK 12 · 第 898 行附近
// 把物体点再投回图像，算重投影误差
cv::______(obj_pts, rvec, tvec, K, D, reproj);

// BLANK 13 · 第 916 行附近
// 重投影误差超过阈值就丢弃
if (err > ______) { ++id; continue; }

// ============================================================
// 附加：订阅入口（第 216 行附近）
// ============================================================
// BLANK 14
// 消息到了调用 imageCallback，this 表示当前对象
image_sub_ = nh.______(image_topic_, queue_size_, &FtDetector::imageCallback, this);
