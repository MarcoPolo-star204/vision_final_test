# 进阶 NN 计划（Day 18–20）

当前 `src/ft_detector/src/nn_infer.cpp` 只是骨架，不参与编译。

- Day 18：接 ONNX Runtime，加载 `armor.onnx`，打印 `[1,21,6300]`
- Day 19：letterbox 640×480、补边 124、BGR→RGB、/255
- Day 20：解码颜色/编号/四角点，映射回原图，NMS
- Day 21：接入 `/ft/detection`，传统/NN 模式切换

模型是 FLOAT16，OpenCV 4.2 DNN 不支持，必须用 ONNX Runtime。
