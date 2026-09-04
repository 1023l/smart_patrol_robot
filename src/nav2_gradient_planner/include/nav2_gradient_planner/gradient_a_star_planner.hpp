#ifndef NAV2_GRADIENT_PLANNER__GRADIENT_A_STAR_PLANNER_HPP_
#define NAV2_GRADIENT_PLANNER__GRADIENT_A_STAR_PLANNER_HPP_

#include <cmath>
#include <cstdint>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_gradient_planner
{

/**
 * @brief 梯度感知 A* 全局规划器
 *
 * 与传统 NavFn / 二值 A* 的核心区别：
 * 1. 连续梯度代价：节点扩展时的通行代价不是"能走 / 不能走"二值，
 *    而是把膨胀层代价值按 cost_weight 权重连续叠加到移动代价上：
 *        traversal(n -> n') = dist * (1 + cost_weight * cost(n') / 253)
 *    代价值越高的栅格（越贴近障碍物 / 膨胀内圈）移动代价越大，
 *    搜索会主动选择远离高危区域的可通行走廊。
 * 2. 8 邻域扩展 + octile 启发式，保证启发式可采纳（不高估）。
 * 3. 目标点落在膨胀区 / 障碍内时按 tolerance 在目标邻域搜索最近可
 *    达点，避免整条任务因目标贴墙直接失败。
 */
class GradientAStarPlanner : public nav2_core::GlobalPlanner
{
public:
  GradientAStarPlanner() = default;
  ~GradientAStarPlanner() override = default;

  // 插件生命周期：配置 / 清理 / 激活 / 停用
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  // 规划入口：给定起始位姿与目标位姿，返回全局路径
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  /**
   * @brief A* 主搜索
   * @param start_mx,start_my 起点栅格坐标
   * @param goal_mx,goal_my 终点栅格坐标
   * @return 搜索成功返回从起点到终点的栅格索引序列，失败返回空
   */
  std::vector<unsigned int> search(
    unsigned int start_mx, unsigned int start_my,
    unsigned int goal_mx, unsigned int goal_my);

  /**
   * @brief 判断栅格是否可通行
   * 连续代价意义上的"不可通行"仍保留二值底线：
   * LETHAL_OBSTACLE(254) / INSCRIBED_INFLATED_OBSTACLE(253) 一律禁止，
   * NO_INFORMATION(255) 由 allow_unknown_ 决定。
   */
  inline bool traversable(unsigned char cost) const
  {
    if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
      cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      return false;
    }
    if (cost == nav2_costmap_2d::NO_INFORMATION) {
      return allow_unknown_;
    }
    return true;
  }

  /**
   * @brief 计算从当前栅格移动到相邻栅格的连续梯度通行代价
   * @param dist 欧氏移动距离（邻域为 1 或 sqrt(2) 个栅格）
   * @param target_cost 目标栅格的膨胀层代价值
   * @return 连续代价：dist * (1 + cost_weight * cost / 253)
   */
  inline double traversalCost(double dist, unsigned char target_cost) const
  {
    return dist * (1.0 + cost_weight_ * static_cast<double>(target_cost) /
             static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE));
  }

  /**
   * @brief 目标点不可达（落在障碍或膨胀内圈）时，在 tolerance 半径内
   *        螺旋搜索最近的可通行替代目标
   */
  bool findNearestFreeGoal(
    unsigned int goal_mx, unsigned int goal_my,
    unsigned int & out_mx, unsigned int & out_my);

  // 把栅格索引序列转成世界系 Path，姿态朝向取路径切线方向
  nav_msgs::msg::Path indicesToPath(const std::vector<unsigned int> & indices,
                                    const geometry_msgs::msg::PoseStamped & goal);

  // ---- ROS2 插件常规成员 ----
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_util::LifecycleNode::SharedPtr node_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::string global_frame_;
  std::string name_;

  // ---- 规划参数 ----
  double cost_weight_{2.0};         // 梯度代价权重：越大路径离障碍越远
  double tolerance_{0.25};          // 目标容差（米），目标不可达时搜索半径
  bool allow_unknown_{true};        // 是否允许穿越未知区域
  double interpolation_resolution_{0.05}; // 输出路径相邻点最小间距（米）

  // ---- 搜索中间量（每次 createPlan 重建）----
  // open list 节点：f 值升序的小顶堆
  struct Node
  {
    double f;
    double g;
    unsigned int index;
  };
  struct NodeCompare
  {
    bool operator()(const Node & a, const Node & b) const { return a.f > b.f; }
  };
  // 8 邻域偏移（dx, dy, 距离）——仅用于搜索循环，避免运行时构造
  static constexpr int8_t NB_DX[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static constexpr int8_t NB_DY[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  static constexpr double NB_DIST[8] = {1.0, 1.0, 1.0, 1.0,
    M_SQRT2, M_SQRT2, M_SQRT2, M_SQRT2};
};

}  // namespace nav2_gradient_planner

#endif  // NAV2_GRADIENT_PLANNER__GRADIENT_A_STAR_PLANNER_HPP_
