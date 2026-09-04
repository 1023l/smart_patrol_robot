#include "nav2_keepout_layer/keepout_layer.hpp"

#include <algorithm>
#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_keepout_layer
{

void Zone::computeAABB()
{
  if (polygon.empty()) {
    min_x = min_y = max_x = max_y = 0.0;
    return;
  }
  min_x = max_x = polygon[0].x;
  min_y = max_y = polygon[0].y;
  for (const auto & p : polygon) {
    min_x = std::min(min_x, p.x);
    max_x = std::max(max_x, p.x);
    min_y = std::min(min_y, p.y);
    max_y = std::max(max_y, p.y);
  }
}

void KeepoutLayer::onInitialize()
{
  // 声明并读取参数
  declareParameter("topic_name", rclcpp::ParameterValue(topic_name_));
  declareParameter("high_cost_value", rclcpp::ParameterValue(high_cost_value_));
  declareParameter("enabled", rclcpp::ParameterValue(true));

  node_->get_parameter(name_ + "." + "topic_name", topic_name_);
  node_->get_parameter(name_ + "." + "high_cost_value", high_cost_value_);
  node_->get_parameter(name_ + "." + "enabled", enabled_);

  // YAML 静态禁区加载
  loadStaticZones();

  // 动态增删话题（KeepoutZone 消息，QoS 保留 latched 语义：新订阅者能拿到最后状态）
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.transient_local();   // 与地图类静态发布一致，方便调试工具晚接入也能收到
  qos.reliable();
  zone_sub_ = node_->create_subscription<inspect_interfaces::msg::KeepoutZone>(
    topic_name_, qos,
    std::bind(&KeepoutLayer::onZoneMsg, this, std::placeholders::_1));

  RCLCPP_INFO(
    node_->get_logger(),
    "KeepOutZone 禁区层已初始化: 静态禁区 %zu 个，动态话题 %s",
    zones_.size(), topic_name_.c_str());
}

void KeepoutLayer::loadStaticZones()
{
  // 参数结构（nav2_params.yaml）：
  //   keepout_layer:
  //     ros__parameters:
  //       zones:
  //         glass_wall_1:              # 禁区 id
  //           lethal: true
  //           points: [x0,y0, x1,y1, x2,y2, ...]   # 多边形顶点展平数组
  //         elevator_shaft:
  //           lethal: true
  //           points: [...]
  std::vector<std::string> zone_ids;
  if (!node_->has_parameter(name_ + ".zones")) {
    return;  // 无静态禁区配置，仅依赖话题动态管理
  }
  node_->get_parameter(name_ + ".zones", zone_ids);

  for (const auto & zid : zone_ids) {
    const std::string prefix = name_ + ".zones." + zid;
    Zone z;
    z.id = zid;

    declareParameterIfNotDeclared(prefix + ".lethal", rclcpp::ParameterValue(true));
    declareParameterIfNotDeclared(prefix + ".points", rclcpp::ParameterValue(std::vector<double>{}));

    node_->get_parameter(prefix + ".lethal", z.lethal);
    std::vector<double> pts;
    node_->get_parameter(prefix + ".points", pts);

    // 顶点展平数组 → Point 序列（奇偶坐标交替）
    if (pts.size() < 6 || pts.size() % 2 != 0) {
      RCLCPP_WARN(
        node_->get_logger(),
        "静态禁区 %s 顶点数非法（需>=3个顶点且坐标成对），跳过", zid.c_str());
      continue;
    }
    for (size_t i = 0; i + 1 < pts.size(); i += 2) {
      geometry_msgs::msg::Point p;
      p.x = pts[i];
      p.y = pts[i + 1];
      p.z = 0.0;
      z.polygon.push_back(p);
    }
    z.computeAABB();
    zones_[zid] = z;
    RCLCPP_INFO(
      node_->get_logger(), "静态禁区已加载: %s（%zu 顶点，%s）",
      zid.c_str(), z.polygon.size(), z.lethal ? "致命" : "高代价");
  }
}

void KeepoutLayer::onZoneMsg(const inspect_interfaces::msg::KeepoutZone::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(zones_mutex_);
  if (msg->action == inspect_interfaces::msg::KeepoutZone::REMOVE) {
    if (zones_.erase(msg->zone_id) > 0) {
      RCLCPP_INFO(node_->get_logger(), "禁区已移除: %s", msg->zone_id.c_str());
      requestRemap();
    }
    return;
  }

  // ADD：同 id 重复添加等于更新顶点（话题端不用先 REMOVE 再 ADD）
  if (msg->polygon.size() < 3) {
    RCLCPP_WARN(
      node_->get_logger(), "禁区 %s 顶点数<3，忽略", msg->zone_id.c_str());
    return;
  }
  Zone z;
  z.id = msg->zone_id;
  z.lethal = msg->lethal;
  z.polygon = msg->polygon;
  z.computeAABB();
  zones_[msg->zone_id] = z;
  RCLCPP_INFO(
    node_->get_logger(), "禁区已添加/更新: %s（%zu 顶点，%s）",
    msg->zone_id.c_str(), z.polygon.size(), z.lethal ? "致命" : "高代价");
  requestRemap();
}

void KeepoutLayer::requestRemap()
{
  // 禁区集合变化后，通知 layered_costmap 需要重算 bounds 并重绘本层。
  // Nav2 Layer 机制：current_ 置 false 会触发 layer 重算，间接达到 remap 效果。
  current_ = false;
}

void KeepoutLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  // 用全部禁区的 AABB 并集圈定受影响区域：
  // 相比逐栅格全图扫描，把代价计算范围收敛到禁区外接矩形内。
  std::lock_guard<std::mutex> lock(zones_mutex_);
  if (zones_.empty()) {
    return;
  }
  double lo_x = std::numeric_limits<double>::infinity();
  double lo_y = std::numeric_limits<double>::infinity();
  double hi_x = -std::numeric_limits<double>::infinity();
  double hi_y = -std::numeric_limits<double>::infinity();
  for (const auto & kv : zones_) {
    lo_x = std::min(lo_x, kv.second.min_x);
    lo_y = std::min(lo_y, kv.second.min_y);
    hi_x = std::max(hi_x, kv.second.max_x);
    hi_y = std::max(hi_y, kv.second.max_y);
  }
  *min_x = std::min(*min_x, lo_x);
  *min_y = std::min(*min_y, lo_y);
  *max_x = std::max(*max_x, hi_x);
  *max_y = std::max(*max_y, hi_y);
}

void KeepoutLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(zones_mutex_);
  if (zones_.empty()) {
    return;
  }

  // 代价地图坐标系：栅格中心世界坐标 = origin + (mx+0.5)*resolution
  const double res = master_grid.getResolution();
  const double ox = master_grid.getOriginX();
  const double oy = master_grid.getOriginY();
  const unsigned int size_x = master_grid.getSizeInCellsX();
  const unsigned int size_y = master_grid.getSizeInCellsY();

  // updateBounds 给出的栅格窗口与地图边界求交（安全裁剪）
  unsigned int i0 = std::max(0, min_i);
  unsigned int j0 = std::max(0, min_j);
  unsigned int i1 = std::min(static_cast<int>(size_x), max_i);
  unsigned int j1 = std::min(static_cast<int>(size_y), max_j);

  unsigned char * costmap = master_grid.getCharMap();

  for (unsigned int j = j0; j < j1; ++j) {
    const double wy = oy + (j + 0.5) * res;
    for (unsigned int i = i0; i < i1; ++i) {
      const double wx = ox + (i + 0.5) * res;

      // 对每个栅格中心点，逐禁区做"先 AABB 粗筛，再射线法精判"两级检测
      for (const auto & kv : zones_) {
        const Zone & z = kv.second;
        // AABB 粗筛：O(1) 排除绝大多数无关栅格
        if (wx < z.min_x || wx > z.max_x || wy < z.min_y || wy > z.max_y) {
          continue;
        }
        // 射线法精判：O(V)
        if (!pointInPolygon(wx, wy, z)) {
          continue;
        }

        const unsigned int idx = j * size_x + i;
        if (z.lethal) {
          // 致命禁区直接赋 LETHAL_OBSTACLE：规划器视为硬障碍
          costmap[idx] = nav2_costmap_2d::LETHAL_OBSTACLE;
        } else {
          // 高代价区：取现有值与高代价值的较大者，不覆盖更严的已有障碍
          costmap[idx] = std::max(costmap[idx],
              static_cast<unsigned char>(high_cost_value_));
        }
        break;  // 该栅格已命中一个禁区，无需再测其他禁区
      }
    }
  }
}

bool KeepoutLayer::pointInPolygon(double x, double y, const Zone & zone)
{
  // 经典射线法（ray casting，奇偶规则 / crossing number）：
  // 从待测点 (x,y) 向 +x 方向引一条水平射线，逐边统计与多边形边的交点数，
  // 交点数为奇 → 点在多边形内；偶 → 外部。
  //
  // 边 (x1,y1)-(x2,y2) 与水平线 y 相交的条件（严格半开区间处理端点）：
  //   (y1 > y) != (y2 > y)，避免顶点处重复计数
  // 再由相似三角形求交点横坐标 cx，与 x 比较。
  //
  // 时间复杂度 O(V)，对凸/凹多边形均正确；边界点按外部处理（巡检场景
  // 宁可少杀一个边界栅格，由膨胀层兜底，避免打点在禁区边缘时误锁死）。
  const auto & poly = zone.polygon;
  bool inside = false;
  const size_t n = poly.size();
  for (size_t a = 0, b = n - 1; a < n; b = a++) {
    const double xi = poly[a].x, yi = poly[a].y;
    const double xj = poly[b].x, yj = poly[b].y;

    const bool straddle = (yi > y) != (yj > y);
    if (!straddle) {
      continue;  // 边完全在射线上方或下方
    }
    // 边与水平线 y 的交点横坐标（线性插值）
    const double cx = xj + (y - yj) * (xi - xj) / (yi - yj);
    if (x < cx) {
      inside = !inside;  // 交点在待测点右侧，计入一次穿越
    }
  }
  return inside;
}

}  // namespace nav2_keepout_layer

// pluginlib 导出
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_keepout_layer::KeepoutLayer, nav2_costmap_2d::Layer)
