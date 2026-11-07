# 单目标跟踪结果
std_msgs/Header header      # 时间戳沿用输入图像
uint32 id                   # 轨迹 ID（基础版固定 1）
bool tracking               # 当前帧有观测命中=true；纯预测=false
geometry_msgs/Vector3 position          # 滤波后的装甲板中心（米）
geometry_msgs/Vector3 velocity          # 滤波后的速度（m/s）
geometry_msgs/Vector3 position_predict  # 往前推 T 秒的预测位置（米）
