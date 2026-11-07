# 最终目标（基础版从 /ft/track 直接转）
std_msgs/Header header
bool valid                      # 当前目标能不能用
uint32 target_id                # 选中的轨迹 ID
geometry_msgs/Vector3 position         # 当前选中装甲板位置
geometry_msgs/Vector3 position_predict # T 秒后预测位置
