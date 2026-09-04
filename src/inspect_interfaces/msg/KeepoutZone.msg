# 禁区管理消息：静态 YAML 加载与运行期话题动态增删共用
#
# 应用场景：玻璃墙体激光穿透产生栅格空洞，KeepOutZone 层用多边形禁区
# 人工补齐虚拟障碍。巡检路线调整时通过话题动态增删，无需改配置重启。
#
# zone_id:      禁区唯一标识，同 id 重复 ADD 等于更新顶点
# action:      0=ADD 添加/更新, 1=REMOVE 删除
# polygon:     多边形顶点（map 坐标系，米）；REMOVE 时只需 zone_id
# lethal:      true=致命禁区（不可通行），false=高代价区（可通行但被规划器回避）
geometry_msgs/Point[] polygon
uint8 action
uint8 ADD=0
uint8 REMOVE=1
string zone_id
bool lethal
