// nn_infer.cpp
// ------------------------------------------------------------------
// 进阶：ONNX Runtime 装甲板推理骨架（Day 18–20 填充）
//
// 模型协议（nn_model/README.md）：
//   输入  images  float32 [1,3,480,640]
//   输出  output0 float32 [1,21,6300]
//   预处理：BGR 等比缩放 -> 640x480 居中补边(124) -> RGB -> /255
//   输出：0–2 颜色(蓝/红/灭)，4–12 编号，13–20 四角点(x,y)
//
// 说明：这里只放骨架和注释，不参与当前 catkin_make；
//       Day 18 再接 ONNX Runtime C++ API，Day 20 再解码/NMS。
// ------------------------------------------------------------------
#include <string>
#include <vector>

struct ArmorNNResult {
  bool is_red = false;
  int number = -1;
  float confidence = 0.0f;
  float corners[8] = {0};  // 左上、左下、右下、右上（模型顺序）
};

class ArmorNN {
 public:
  bool load(const std::string& model_path);
  bool infer(const unsigned char* bgr, int width, int height,
             std::vector<ArmorNNResult>& results);

 private:
  // TODO(Day 18): Ort::Session session_;
  // TODO(Day 19): letterbox 参数 scale/pad_x/pad_y
  // TODO(Day 20): decode + NMS
};
