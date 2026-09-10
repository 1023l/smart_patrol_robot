#ifndef NAV2_KEEPOUT_LAYER__KEEPOUT_LAYER_HPP_
#define NAV2_KEEPOUT_LAYER__KEEPOUT_LAYER_HPP_

#include <cmath>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "inspect_interfaces/msg/keepout_zone.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_keepout_layer
{

/**
 * @brief 多边形禁区结构
 */
struct Zone
{
  std::string id;                     // 唯一标识（话题动态增删的 key）
  std::vector<geometry_msgs::msg::Point> polygon;  // 世界系顶点（map 坐标系）
  bool lethal{true};                  // true=致命禁区，false=高代价区

  // 世界系外接矩形（updateBounds 用，避免逐栅格全图扫描）
  double min_x{0.0}, min_y{0.0}, max_x{0.0}, max_y{0.0};
  void computeAABB();
};

/**
 * @brief KeepOutZone 禁区代价地图层
 *
 * 解决的问题：玻璃墙体激光雷达穿透 → 栅格空洞 → 虚拟障碍缺失 →
 * 规划器把玻璃当自由空间，机器人撞玻璃。
 *
 * 实现要点（面试核心）：
 * 1. 继承 nav2_costmap_2d::Layer，重写 onInitialize / updateBounds /
 *    updateCosts 三个标准接口；
 * 2. YAML 静态多边形配置 + /keepout_zones 话题动态增删（同一消息类型
 *    KeepoutZone，action=ADD/REMOVE），运行期调整巡检路线不需要重启导航；
 * 3. updateBounds 用 AABB 外接矩形快速圈定受影响栅格范围，避免全图扫描；
 * 4. updateCosts 内层用射线法（ray casting，奇偶规则）判断栅格中心点
 *    是否落在多边形内：从待测点向 +x 方向引射线，统计与多边形各边的
 *    交点数，奇数在内部、偶数在外部。时间复杂度 O(V)，V 为顶点数；
 * 5. 线程安全：话题回调与 updateCosts 并发访问 zones_，用互斥锁保护。
 */
class KeepoutLayer : public nav2_costmap_2d::Layer
{
public:
  KeepoutLayer() = default;
  ~KeepoutLayer() override = default;

  // Layer 标准接口
  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;

  // 本层是否需要合并进主图（含动态变化时重新请求合并）
  bool isClearable() override {return false;}

private:
  /** @brief 解析 YAML 静态禁区配置（zones 参数） */
  void loadStaticZones();

  /** @brief 话题回调：动态增删禁区 */
  void onZoneMsg(const inspect_interfaces::msg::KeepoutZone::SharedPtr msg);

  /** @brief 通知 layered_costmap 重算 bounds（禁区变化时调用） */
  void requestRemap();

  /**
   * @brief 射线法点-多边形包含测试（ray casting，奇偶规则）
   * @param x,y 待测点世界坐标
   * @param zone 目标多边形
   * @return true=点在多边形内部（边界按外部处理，避免边界栅格误杀）
   */
  static bool pointInPolygon(double x, double y, const Zone & zone);

  // ---- 成员 ----
  std::unordered_map<std::string, Zone> zones_;
  mutable std::mutex zones_mutex_;          // 话题回调 vs updateCosts 并发保护
  rclcpp::Subscription<inspect_interfaces::msg::KeepoutZone>::SharedPtr zone_sub_;
  double high_cost_value_{252};             // 非致命禁区的高代价值（接近内切膨胀）
  std::string topic_name_{"keepout_zones"};
  // 层本身没有内部状态需要重置；重置外部地图与本层无关
  bool need_remap_{false};
};

}  // namespace nav2_keepout_layer

#endif  // NAV2_KEEPOUT_LAYER__KEEPOUT_LAYER_HPP_
