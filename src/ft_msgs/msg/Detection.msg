# 单条装甲板检测（一帧里可能有多条）
uint32 id                 # 当前帧内唯一；不要求跨帧跟踪 ID
bool is_red               # true=红，false=蓝
bool is_large_armor       # true=大装甲板 235x127 mm，false=小 120x125 mm
geometry_msgs/Pose pose   # position=tvec（相机系，米）；orientation=rvec 转四元数
