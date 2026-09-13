# 单次巡检视觉结果（话题 /inspect/result 与服务响应共用语义）
string point_name          # 巡检点名称，如 F1_P1
uint8 floor                # 楼层号
string task                # gauge / indicator / extinguisher
string status              # NORMAL / ABNORMAL / UNKNOWN
string message             # 可读说明
string image_path          # 落盘图片路径（可为空）
float32 confidence         # 0~1
builtin_interfaces/Time stamp
