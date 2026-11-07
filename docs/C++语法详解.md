# C++ 语法详解（结合本项目）

> 对应代码：`ft_detector_node.cpp` / `ft_tracker_node.cpp` / `ft_selector_node.cpp`
> 原则：只讲本项目真实出现的语法；每个语法都给代码原句、逐部分解释、常见错误。

## 0. 怎么读

- 先看“代码原句”，知道它长什么样；
- 再看“逐部分”，知道每个符号什么意思；
- 最后看“常见错误”，这是答辩最容易被问的地方。

---

## 1. `#include`：把头文件拿进来

```cpp
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "ft_detector/DetectorConfig.h"
```

- `#include` 是预处理指令，在编译前把目标文件内容复制进来；
- `<...>` 表示系统/第三方头文件，编译器去 include 路径找；
- `"..."` 表示项目内头文件，先找当前目录；
- `ros/ros.h` 提供 `ros::NodeHandle`、`ros::Publisher` 等；
- `opencv2/opencv.hpp` 是 OpenCV 的总头文件。

**常见错误**：VS Code 报 `cannot open source file "ros/ros.h"`，不是代码错，是编辑器没找到 `/opt/ros/noetic/include`；用 Dev Container 打开。

**一句话**：include 是“把别人的工具箱搬进来”。

---

## 2. `namespace {}`：匿名命名空间

```cpp
namespace {
struct LightBar { ... };
bool comparePointX(...) { ... }
}  // namespace
```

- `namespace` 用来避免名字冲突；
- 不写名字的 `namespace {}` 叫匿名命名空间，里面的东西只在本文件可见；
- 相当于 C 里的 `static`，但更 C++。

**为什么**：`LightBar`、`comparePointX` 只给本文件用，不暴露给别的文件。

**常见错误**：把函数定义在匿名命名空间里，却在别的文件声明它，会链接不到。

**一句话**：匿名命名空间 = 本文件私有。

---

## 3. `struct`：把几个字段打包

```cpp
struct LightBar {
  cv::RotatedRect rect;
  cv::Point2f center;
  float width;
  float height;
  float angle;
};
```

- `struct` 定义一种新类型；
- 里面的 `rect/center/width/...` 是成员变量；
- `LightBar bar;` 创建一个对象，用 `bar.width` 访问；
- C++ 的 `struct` 和 `class` 几乎一样，区别只是默认访问权限：struct 默认 public，class 默认 private。

**为什么**：一根灯条有 5 个属性，用 5 个平行数组容易错位；打包成一个结构体，`bars[i].width` 一眼看懂。

**常见错误**：结构体定义末尾忘了分号 `;`，报 `expected ';'`。

**一句话**：struct 是“数据包裹”。

---

## 4. `class` / `public` / `private`

```cpp
class FtDetector {
 public:
  FtDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh);
 private:
  void imageCallback(...);
  ros::Subscriber image_sub_;
};
```

- `class` 定义类；
- `public:` 后面的成员外部能访问；
- `private:` 后面的成员只有类内部能访问；
- 构造函数、`imageCallback`、订阅者、发布者都装在类里。

**为什么**：检测节点有状态（参数、camera_info、发布者），用类把这些状态和函数绑在一起；多个对象也互不干扰。

**常见错误**：在类外直接访问 `private` 成员，编译报 `is private within this context`。

**一句话**：class 把“数据 + 操作数据的函数”装进一个盒子。

---

## 5. 构造函数

```cpp
FtDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
  pnh.param<std::string>("image_topic", image_topic_, "...");
  image_sub_ = nh.subscribe(...);
}
```

- 函数名和类名相同、没有返回值，就是构造函数；
- 创建对象时自动调用：`FtDetector detector(nh, pnh);`
- 这里做参数读取、订阅、发布、创建 TF/动态参数服务器。

**为什么**：对象一出生就要处于“可用状态”，所以初始化和订阅放构造函数。

**常见错误**：在构造函数里写 `while(1)`，程序永远走不到 `ros::spin()`，回调不会执行。

**一句话**：构造函数 = 对象出生时做的第一件事。

---

## 6. 析构函数 / `new` / `delete`

```cpp
~FtDetector() {
  delete tf_broadcaster_;
  delete dr_server_;
}
tf_broadcaster_ = new tf2_ros::TransformBroadcaster();
```

- `~类名()` 是析构函数，对象销毁时自动调用；
- `new` 在堆上分配对象，返回指针；
- `delete` 释放 `new` 出来的对象；
- 这里用裸指针是为了避免 `std::unique_ptr` 这类初学者不熟的语法。

**为什么**：`new` 出来的东西不会自动释放，必须在析构里 `delete`，否则内存泄漏。

**常见错误**：`new[]` 要用 `delete[]`；同一个指针 `delete` 两次会崩。

**一句话**：new 借内存，delete 还内存。

---

## 7. 函数定义、参数、返回值

```cpp
void buildMasks(const cv::Mat& hsv, cv::Mat& red_mask, cv::Mat& blue_mask)
bool buildPair(const LightBar& a, const LightBar& b, bool& is_large)
static void barCorners(...)
```

- `void` 表示不返回值；`bool` 返回真/假；
- 参数列表里每个参数都有类型和名字；
- `static` 成员函数不属于某个对象，可以直接 `FtDetector::barCorners(...)` 调用；
- `bool& is_large` 是引用参数，函数里改它，外面的变量也会变。

**为什么**：`buildPair` 需要返回“能不能配”，还要顺带返回“是不是大装甲板”；返回值只能一个，另一个用引用带出来。

**常见错误**：函数声明和定义的参数类型/const 不一致，链接时报 `undefined reference`。

**一句话**：返回值带一个结果，引用参数带多个结果。

---

## 8. `const` 和 `&`：只读引用

```cpp
void detectBars(const cv::Mat& mask, std::vector<LightBar>& bars)
const LightBar& left = bars[pairs[k].left];
```

- `&` 是引用，不是拷贝；函数里操作的就是原对象；
- `const` 表示“只读”，函数里不能改它；
- `const cv::Mat&` = 不拷贝整张图，也不允许改图；
- `std::vector<LightBar>&` = 不拷贝数组，允许往里面写。

**为什么**：图像和数组很大，拷贝很慢；只读的加 `const`，编译器帮你检查。

**常见错误**：对 `const` 引用调用非 const 函数会编译错；返回局部变量的引用会悬空。

**一句话**：`&` 不拷贝，`const` 不许改。

---

## 9. 指针 `*`、`->`、`this`

```cpp
tf2_ros::TransformBroadcaster* tf_broadcaster_;
tf_broadcaster_->sendTransform(tf);
image_sub_ = nh.subscribe(..., &FtDetector::imageCallback, this);
```

- `*` 表示指针，存的是地址；
- `p->f()` 等价于 `(*p).f()`；
- `this` 是当前对象的地址；
- `&FtDetector::imageCallback` 是成员函数指针。

**为什么**：ROS 的 subscribe 需要知道“消息来了调哪个对象的哪个函数”，所以传函数指针 + `this`。

**常见错误**：指针没初始化就 `->`，会段错误；`this` 只能在成员函数里用。

**一句话**：指针存地址，`->` 顺着地址找成员。

---

## 10. `std::string`

```cpp
std::string image_topic_;
pnh.param<std::string>("image_topic", image_topic_, "/hk_camera/...");
std::string child = "armor_" + std::to_string(id);
```

- `std::string` 是 C++ 字符串，能拼接、能比较、能取长度；
- 比 C 的 `char*` 安全，不用手动管理内存；
- `std::to_string(id)` 把整数转成字符串。

**常见错误**：`"armor_" + id` 不能直接把字符串和整数相加，必须先 `to_string`。

**一句话**：C++ 字符串用 `std::string`，不要用 `char*` 拼。

---

## 11. `std::vector`：会变长的数组

```cpp
std::vector<LightBar> red_bars, blue_bars;
bars.push_back(bar);
for (size_t i = 0; i < bars.size(); ++i) ...
```

- `vector<T>` 是动态数组，元素类型是 T；
- `push_back` 往末尾加一个；
- `size()` 返回元素个数；
- `bars[i]` 访问第 i 个，下标从 0 开始。

**为什么**：一帧里灯条数量不固定，用 vector 不用提前定大小。

**常见错误**：`bars[bars.size()]` 越界；`push_back` 可能让迭代器/引用失效。

**一句话**：vector 是会自动变长的数组。

---

## 12. `char` / `bool` / `size_t`

```cpp
std::vector<char> used(bars.size(), 0);
for (size_t i = 0; i < bars.size(); ++i)
```

- `char` 是 1 字节整数，这里当“小布尔”用；
- `bool` 只有 true/false；
- `size_t` 是无符号整数，专门表示大小/下标；
- `vector<char> used(n, 0)` 创建 n 个 0。

**为什么**：`std::vector<bool>` 是位压缩的特化，初学者容易踩坑；用 `char` 更直观。

**常见错误**：`size_t` 和 `int` 混用，出现负数比较警告；下标别用有符号数。

**一句话**：下标用 `size_t`，标志位用 `char` 或 `bool`。

---

## 13. 整数和小数类型

```cpp
int id = 0;
float width, height;
double W = 0.235;
uint32_t id;
```

- `int` 整数；`float` 单精度；`double` 双精度；
- `uint32_t` 无符号 32 位整数，消息字段常用；
- 小数默认用 `double`，`0.235` 是 double；
- `0.f` / `0.0f` 的 `f` 表示 float。

**常见错误**：整数除法 `1/2` 得 0；要 `1.0/2`；float 和 double 混用会有精度/警告。

**一句话**：像素/下标用整数，距离/角度用 double。

---

## 14. `if / else if / else`

```cpp
if (has_info_) {
  ...
} else if (!msg->header.frame_id.empty()) {
  ...
} else {
  ...
}
```

- `if (条件)` 条件为真执行；
- `else if` 前面的不成立才判断；
- `else` 前面都不成立才执行；
- `!` 是逻辑非，`empty()` 返回真表示空。

**常见错误**：条件里用 `=` 赋值而不是 `==` 比较；大括号漏了导致悬空 else。

**一句话**：if 是岔路口，else 是“否则”。

---

## 15. `for` 循环

```cpp
for (size_t i = 0; i < bars.size(); ++i) { ... }
for (int color = 0; color < 2; ++color) { ... }
```

- 初始化 → 条件 → 循环体 → `++i` → 再判断；
- `++i` 是前置自增，效率略好；
- 循环变量只在循环内有效。

**常见错误**：`i <= size()` 越界；循环里修改 `size()` 导致条件变化。

**一句话**：for 适合“次数已知/下标遍历”。

---

## 16. `while` 循环

```cpp
while (bar.angle < -45.f) bar.angle += 90.f;
while (bar.angle >= 45.f) bar.angle -= 90.f;
```

- 条件为真就一直执行；
- 这里把角度归一到 `[-45, 45)`；
- 循环体只有一句时可以省略大括号。

**常见错误**：循环变量不改变，死循环；浮点数比较要用容差。

**一句话**：while 适合“不知道几次，但知道什么时候停”。

---

## 17. `break` / `continue` / `return`

```cpp
if (duplicate) { ++id; continue; }
if (err > max_reproj_error_) { ++id; continue; }
return;
```

- `continue` 跳过本次循环剩下的语句，进入下一次；
- `break` 直接跳出整个循环；
- `return` 结束当前函数，可带返回值。

**为什么**：检测里“这一对不合法”就用 `continue` 跳过，不打断整帧。

**常见错误**：在 switch 里 `break` 只跳出 switch；在嵌套循环里 `break` 只跳一层。

**一句话**：continue 跳本次，break 跳整个循环，return 跳函数。

---

## 18. 三元运算符 `? :`

```cpp
const double W = is_large ? large_w_ : small_w_;
const LightBar& b1 = left.center.x < right.center.x ? left : right;
```

- `条件 ? A : B`，条件真取 A，假取 B；
- 适合简单二选一，不适合写复杂逻辑。

**常见错误**：嵌套太多三元，可读性差；A/B 类型不一致会有隐式转换。

**一句话**：三元是“二选一”的简写。

---

## 19. 逻辑、比较、算术、复合赋值

```cpp
if (has_info_ && K.rows == 3)
if (a < b || c > d)
++id;  id += 1;  angle -= 90.f;
```

- `&&` 与，`||` 或，`!` 非；
- `==` 等于，`!=` 不等，`< <= > >=` 比较；
- `+ - * / %` 算术；`%` 取余；
- `++` 自增，`--` 自减；
- `+= -= *= /=` 复合赋值。

**常见错误**：`=` 和 `==` 混；`&&` 有短路，右边可能不执行。

**一句话**：逻辑判断用 `&& || !`，比较用 `==`。

---

## 20. `std::max` / `std::min`

```cpp
const int mx = std::max(1, pair_box.width / 4);
const float w = std::min(rw, rh);
```

- 返回两个值里较大/较小的；
- 两个参数类型要一致，否则模板推导失败。

**常见错误**：`std::max(1, 2.5)` 类型不一致，编译错。

**一句话**：max/min 取大小，类型要一致。

---

## 21. `<cmath>`：sqrt / fabs / cos / sin

```cpp
std::sqrt(dx*dx + dy*dy + dz*dz);
std::fabs(a.height - b.height);
std::cos(a); std::sin(a);
```

- `sqrt` 开方，算距离；
- `fabs` 浮点绝对值；
- `cos/sin` 三角函数，角度要转弧度。

**常见错误**：角度直接传给 cos/sin，要先乘 `CV_PI/180`。

**一句话**：距离用 sqrt，角度先转弧度。

---

## 22. `std::sort` + 比较函数

```cpp
std::sort(p.begin(), p.end(), comparePointX);
bool comparePointX(const cv::Point2f& a, const cv::Point2f& b) {
  return a.x < b.x;
}
```

- `sort(begin, end, cmp)` 排序；
- 比较函数返回 true 表示 a 应排在 b 前面；
- 这里按 x 从小到大排。

**为什么不用 lambda**：lambda 对初学者不直观；普通函数更好读。

**常见错误**：比较函数写成 `<=` 可能违反严格弱序，排序崩。

**一句话**：sort 的第三个参数是“谁在前”的规则。

---

## 23. `static_cast` / `const_cast`

```cpp
static_cast<int>(camera_info_.D.size())
static_cast<float>(W)
const_cast<double*>(camera_info_.K.data())
```

- `static_cast<T>(x)` 是编译期类型转换，比 C 风格 `(T)x` 更安全、更明显；
- `const_cast` 去掉/加上 const，这里因为 OpenCV 的 Mat 构造函数要非 const 指针；
- 不要用 `const_cast` 去改真正 const 的对象。

**常见错误**：`const_cast` 后写入只读内存，未定义行为。

**一句话**：转换要写明 `static_cast`，别用万能 C 强转。

---

## 24. 大括号初始化

```cpp
std::vector<cv::Point3f> obj_pts = {
  {0.f,0.f,0.f}, {W,0.f,0.f}, {W,H,0.f}, {0.f,H,0.f}
};
std::vector<cv::Point2f> img_pts = {lt, rt2, rb2, lb};
```

- `{...}` 是初始化列表；
- vector 可以用列表直接初始化；
- 结构体也可以 `LightBar bar{...}`（本项目用逐个赋值，更直观）。

**常见错误**：列表元素类型/数量不对，编译错；嵌套大括号要数清楚。

**一句话**：大括号初始化，比一个个 push_back 简洁。

---

## 25. `try / catch`

```cpp
try {
  cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
} catch (const cv_bridge::Exception& e) {
  ROS_ERROR_STREAM_THROTTLE(1.0, "cv_bridge: " << e.what());
  return;
}
```

- `try` 里放可能抛异常的代码；
- `catch (类型& e)` 捕获异常；
- `e.what()` 返回错误信息；
- 捕获后可以选择返回、重试、降级。

**为什么**：压缩图可能解码失败，不能让它把整个节点搞崩。

**常见错误**：catch 里再抛不处理；捕获类型不对，漏掉异常。

**一句话**：try 保护可能出错的代码，catch 处理错误。

---

## 26. `boost::bind` + `_1` / `_2`

```cpp
dr_server_->setCallback(
  boost::bind(&FtDetector::reconfigureCallback, this, _1, _2));
```

- `boost::bind` 把成员函数、对象、占位符绑成一个可调用对象；
- `_1`、`_2` 是占位符，表示调用时传入的第 1、2 个参数；
- 这里把 `reconfigureCallback(cfg, level)` 绑成回调。

**为什么**：C++ 成员函数不能直接当普通函数指针用，需要绑定 `this`。

**常见错误**：参数个数/顺序不对，编译报 `no matching function`。

**一句话**：bind 把“对象 + 成员函数 + 参数”打包成回调。

---

## 27. ROS 日志宏

```cpp
ROS_INFO_STREAM("ft_detector: image=" << image_topic_);
ROS_ERROR_STREAM_THROTTLE(1.0, "cv_bridge: " << e.what());
ROS_WARN_STREAM_THROTTLE(1.0, "solvePnP failed: " << ...);
```

- `ROS_INFO_STREAM` 像 `std::cout`，用 `<<` 拼接；
- `THROTTLE(1.0, ...)` 表示 1 秒内只打一次，避免刷屏；
- 日志输出到终端和 rosout。

**常见错误**：在回调里每帧都打日志，导致 CPU 高/日志爆炸；用 THROTTLE。

**一句话**：日志用 ROS_*_STREAM，频繁打印加 THROTTLE。

---

## 28. `pnh.param<T>` / `getParam`

```cpp
pnh.param<std::string>("image_topic", image_topic_, "/hk_camera/...");
pnh.param<double>("max_body_v", max_body_v_, 90.0);
pnh.getParam("fallback_camera_matrix", fallback_K_);
```

- `param<T>(键, 变量, 默认值)`：有值就覆盖，没有就用默认；
- `getParam(键, 变量)`：返回 bool 表示有没有取到；
- 键是私有命名空间下的名字，对应 YAML 的 `ft_detector:`。

**常见错误**：YAML 层级/键名写错，取不到但用了默认值，行为“看起来没生效”。

**一句话**：param 带默认值，getParam 要知道有没有取到。

---

## 29. `subscribe` / `advertise` / 回调

```cpp
image_sub_ = nh.subscribe(image_topic_, queue_size_, &FtDetector::imageCallback, this);
det_pub_ = nh.advertise<ft_msgs::DetectionArray>(det_topic_, 10);
```

- `subscribe(话题, 队列, 回调, this)` 订阅；
- `advertise<类型>(话题, 队列)` 发布；
- 回调函数由 ROS 在消息到达时自动调用；
- 队列大小是“最多缓存几条”，不是越大越好。

**常见错误**：话题名/消息类型不一致，收不到；队列太大积压旧帧。

**一句话**：subscribe 收货，advertise 发货，回调自动触发。

---

## 30. `cv::Mat` / `Point` / `Scalar`

```cpp
cv::Mat hsv;
cv::cvtColor(cv_ptr->image, hsv, cv::COLOR_BGR2HSV);
cv::Point2f center;
cv::Scalar(0, 0, 255);
```

- `cv::Mat` 是图像矩阵，含 data/rows/cols/type；
- `cv::Point2f` 是二维点，float；
- `cv::Scalar` 是四通道值，画图时常用 BGR+alpha；
- OpenCV 默认 BGR，不是 RGB。

**常见错误**：`Mat` 赋值共享内存，改一个影响另一个；需要独立副本用 `clone()`。

**一句话**：Mat 是图，Point 是点，Scalar 是颜色。

---

## 31. `cv::RotatedRect` / `.points`

```cpp
cv::RotatedRect rect = cv::minAreaRect(c);
cv::Point2f pts[4];
rect.points(pts);
```

- `RotatedRect` 有 center、size、angle；
- `points(pts)` 把四个角点写进数组；
- 数组 `cv::Point2f pts[4]` 是 C 风格数组，大小固定。

**常见错误**：数组开太小越界；角点顺序不固定，后面要自己排序。

**一句话**：RotatedRect 给中心/宽高/角度，points 给四个角。

---

## 32. `solvePnP` / `Vec3d`

```cpp
cv::Vec3d rvec, tvec;
cv::solvePnP(obj_pts, img_pts, K, D, rvec, tvec, false, cv::SOLVEPNP_IPPE);
```

- `Vec3d` 是 3 个 double 的向量；
- `rvec` 旋转向量，`tvec` 平移向量；
- `solvePnP` 用 3D-2D 对应点求位姿；
- `false` 表示不用初始猜测。

**常见错误**：点顺序不一致；物体点单位混用；平面目标用 IPPE 更稳。

**一句话**：solvePnP 输入对应点，输出旋转和平移。

---

## 33. `tf2::Quaternion` / `Matrix3x3`

```cpp
tf2::Matrix3x3 m(R00, R01, R02, R10, R11, R12, R20, R21, R22);
tf2::Quaternion q;
m.getRotation(q);
q.normalize();
```

- `Matrix3x3` 是 3×3 旋转矩阵；
- `Quaternion` 是四元数，避免万向锁；
- `getRotation` 从矩阵取四元数；
- `normalize` 归一化，防止数值误差。

**常见错误**：矩阵元素顺序写错；四元数不归一化，TF 报错。

**一句话**：旋转矩阵转四元数，再归一化。

---

## 34. 模板 `Template<T>`

```cpp
dynamic_reconfigure::Server<ft_detector::DetectorConfig> dr_server_;
std::vector<LightBar> bars;
```

- `模板<类型>` 是“把类型当参数”；
- `Server<Config>` 表示这个服务器处理 Config 类型；
- `vector<LightBar>` 表示元素是 LightBar 的数组；
- 编译器会为每个类型生成一份代码。

**常见错误**：模板参数写错类型，报一长串 `template argument deduction`。

**一句话**：尖括号里是“类型参数”。

---

## 35. 本项目**没有用**的语法（避免你被误导）

- `std::unique_ptr`：已改成裸指针 + 析构 `delete`；
- `auto`：已改成写清楚类型；
- lambda：已改成普通函数 `comparePointX`；
- `std::pair`：已改成 `struct BarPair`；
- 范围 for：已改成下标 for。

这些不是“不能用”，而是为了让你一眼看懂。

---

## 36. 一张表记住

| 语法 | 一句话 |
| --- | --- |
| `#include` | 搬工具箱 |
| `namespace {}` | 本文件私有 |
| `struct` | 数据包裹 |
| `class` | 数据 + 函数 |
| 构造函数 | 对象出生时运行 |
| 析构函数 | 对象销毁时运行 |
| `&` | 引用，不拷贝 |
| `const &` | 只读引用 |
| `*` / `->` | 指针 / 顺着指针找成员 |
| `vector` | 会变长的数组 |
| `for` | 次数已知的循环 |
| `while` | 不知道几次的循环 |
| `continue` | 跳过本次 |
| `break` | 跳出整个循环 |
| `return` | 结束函数 |
| `? :` | 二选一 |
| `static_cast` | 明确的类型转换 |
| `try/catch` | 保护可能出错的代码 |
| `boost::bind` | 打包成员函数回调 |
| `param<T>` | 读参数，带默认值 |
| `subscribe` | 订阅话题 |
| `advertise` | 发布话题 |
| `Mat` | 图像矩阵 |
| `solvePnP` | 3D-2D 求位姿 |
| `Quaternion` | 四元数旋转 |

> 答辩时先讲“这句话在干什么”，再讲“为什么这么写”，最后讲“写错会怎样”。
