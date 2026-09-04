#include "nav2_gradient_planner/gradient_a_star_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_set>
#include <vector>

#include "nav2_util/node_utils.hpp"

namespace nav2_gradient_planner
{

// 静态成员初始化：8 邻域偏移与对应距离
constexpr int8_t GradientAStarPlanner::NB_DX[8];
constexpr int8_t GradientAStarPlanner::NB_DY[8];
constexpr double GradientAStarPlanner::NB_DIST[8];

void GradientAStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  tf_ = tf;
  node_ = parent.lock();
  name_ = name;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // ---- 参数声明与读取（均有默认值，nav2_params.yaml 可覆盖）----
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".cost_weight", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".allow_unknown", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.05));

  node_->get_parameter(name_ + ".cost_weight", cost_weight_);
  node_->get_parameter(name_ + ".tolerance", tolerance_);
  node_->get_parameter(name_ + ".allow_unknown", allow_unknown_);
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);

  RCLCPP_INFO(
    node_->get_logger(),
    "梯度感知 A* 规划器已配置: cost_weight=%.2f tolerance=%.2f allow_unknown=%s",
    cost_weight_, tolerance_, allow_unknown_ ? "true" : "false");
}

void GradientAStarPlanner::cleanup()
{
  RCLCPP_INFO(node_->get_logger(), "正在清理规划器插件 %s", name_.c_str());
}

void GradientAStarPlanner::activate()
{
  RCLCPP_INFO(node_->get_logger(), "正在激活规划器插件 %s", name_.c_str());
}

void GradientAStarPlanner::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "正在停用规划器插件 %s", name_.c_str());
}

nav_msgs::msg::Path GradientAStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  auto t_begin = std::chrono::steady_clock::now();

  nav_msgs::msg::Path global_path;
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;

  // 1. 坐标系校验
  if (start.header.frame_id != global_frame_) {
    throw nav2_core::PlannerException(
            "起点坐标系不匹配，规划器仅接受 " + global_frame_ + " 系的起点");
  }
  if (goal.header.frame_id != global_frame_) {
    throw nav2_core::PlannerException(
            "目标坐标系不匹配，规划器仅接受 " + global_frame_ + " 系的目标");
  }

  // 2. 世界坐标转栅格坐标
  unsigned int start_mx, start_my, goal_mx, goal_my;
  if (!costmap_->worldToMap(
      start.pose.position.x, start.pose.position.y, start_mx, start_my))
  {
    throw nav2_core::PlannerException("起点在代价地图范围之外");
  }
  if (!costmap_->worldToMap(
      goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my))
  {
    throw nav2_core::PlannerException("目标在代价地图范围之外");
  }

  // 3. 起点必须可通行（机器人本体所在栅格，若被膨胀覆盖直接报错而不是空转搜索）
  unsigned char start_cost = costmap_->getCost(start_mx, start_my);
  if (!traversable(start_cost)) {
    throw nav2_core::PlannerException("起点处于障碍物或内切膨胀区，无法规划");
  }

  // 4. 目标不可达时按容差搜索最近可通行替代点（贴墙目标兜底）
  unsigned char goal_cost = costmap_->getCost(goal_mx, goal_my);
  if (!traversable(goal_cost)) {
    unsigned int alt_mx = goal_mx, alt_my = goal_my;
    if (!findNearestFreeGoal(goal_mx, goal_my, alt_mx, alt_my)) {
      throw nav2_core::PlannerException(
              std::string("目标处于障碍物且 ").append(
                std::to_string(tolerance_)).append("m 容差内无可达点"));
    }
    goal_mx = alt_mx;
    goal_my = alt_my;
  }

  // 5. A* 主搜索
  std::vector<unsigned int> indices = search(start_mx, start_my, goal_mx, goal_my);
  if (indices.empty()) {
    throw nav2_core::PlannerException("A* 搜索失败：目标不可达");
  }

  // 6. 栅格序列转世界系路径
  global_path = indicesToPath(indices, goal);

  auto t_end = std::chrono::steady_clock::now();
  double ms = std::chrono::duration<double, std::milli>(t_end - t_begin).count();
  RCLCPP_DEBUG(
    node_->get_logger(), "规划完成: %zu 个路径点, 耗时 %.2f ms",
    global_path.poses.size(), ms);

  return global_path;
}

std::vector<unsigned int> GradientAStarPlanner::search(
  unsigned int start_mx, unsigned int start_my,
  unsigned int goal_mx, unsigned int goal_my)
{
  const unsigned int size_x = costmap_->getSizeInCellsX();
  const unsigned int size_y = costmap_->getSizeInCellsY();
  const unsigned int total = size_x * size_y;
  const unsigned int start_idx = start_my * size_x + start_mx;
  const unsigned int goal_idx = goal_my * size_x + goal_mx;

  if (start_idx == goal_idx) {
    return {start_idx};
  }

  // ---- octile 启发式：8 邻域运动下的可采纳下界 ----
  auto heuristic = [size_x](unsigned int idx, unsigned int goal) {
      int dx = std::abs(static_cast<int>(idx % size_x) - static_cast<int>(goal % size_x));
      int dy = std::abs(static_cast<int>(idx / size_x) - static_cast<int>(goal / size_x));
      double dmin = std::min(dx, dy);
      double dmax = std::max(dx, dy);
      return dmin * M_SQRT2 + (dmax - dmin);
    };

  // ---- 搜索数据结构 ----
  // g 值表：到起点的最小连续梯度代价
  std::vector<double> g_cost(total, std::numeric_limits<double>::infinity());
  // 前驱表：回溯路径用，UINT32_MAX 表示未访问
  std::vector<unsigned int> came_from(total, std::numeric_limits<unsigned int>::max());
  // open list：按 f 值升序的小顶堆（std::priority_queue 默认大顶堆，比较器取反）
  std::priority_queue<Node, std::vector<Node>, NodeCompare> open;
  // closed 标记：出堆时结算
  std::vector<bool> closed(total, false);

  g_cost[start_idx] = 0.0;
  open.push({heuristic(start_idx, goal_idx), 0.0, start_idx});

  // 8 邻域：行列偏移与边界检查
  while (!open.empty()) {
    Node cur = open.top();
    open.pop();

    // 惰性删除：堆里可能存在已被更优路径覆盖的过期节点
    if (closed[cur.index]) {
      continue;
    }
    closed[cur.index] = true;

    if (cur.index == goal_idx) {
      break;  // 目标出堆即最优，无需扩展
    }

    const int cur_mx = static_cast<int>(cur.index % size_x);
    const int cur_my = static_cast<int>(cur.index / size_x);

    for (int k = 0; k < 8; ++k) {
      const int nb_mx = cur_mx + NB_DX[k];
      const int nb_my = cur_my + NB_DY[k];

      // 地图边界检查
      if (nb_mx < 0 || nb_mx >= static_cast<int>(size_x) ||
        nb_my < 0 || nb_my >= static_cast<int>(size_y))
      {
        continue;
      }

      const unsigned int nb_idx = static_cast<unsigned int>(nb_my) * size_x +
        static_cast<unsigned int>(nb_mx);
      if (closed[nb_idx]) {
        continue;
      }

      const unsigned char nb_cost = costmap_->getCost(
        static_cast<unsigned int>(nb_mx), static_cast<unsigned int>(nb_my));
      if (!traversable(nb_cost)) {
        continue;
      }

      // ---- 核心：连续梯度代价扩展 ----
      // 传统 A* 的 g 增量为 dist（二值代价）；这里把膨胀层代价值连续
      // 叠加进移动代价，越高代价栅格越"贵"，路径被推离高危区域。
      const double step_cost = traversalCost(NB_DIST[k], nb_cost);
      const double new_g = g_cost[cur.index] + step_cost;

      if (new_g < g_cost[nb_idx]) {
        g_cost[nb_idx] = new_g;
        came_from[nb_idx] = cur.index;
        open.push({new_g + heuristic(nb_idx, goal_idx), new_g, nb_idx});
      }
    }
  }

  // 目标从未被访问，搜索失败
  if (!closed[goal_idx]) {
    return {};
  }

  // ---- 回溯：从目标沿前驱回到起点，再反转 ----
  std::vector<unsigned int> indices;
  unsigned int cur = goal_idx;
  while (cur != std::numeric_limits<unsigned int>::max()) {
    indices.push_back(cur);
    if (cur == start_idx) {
      break;
    }
    cur = came_from[cur];
  }
  std::reverse(indices.begin(), indices.end());
  return indices;
}

bool GradientAStarPlanner::findNearestFreeGoal(
  unsigned int goal_mx, unsigned int goal_my,
  unsigned int & out_mx, unsigned int & out_my)
{
  // 把容差（米）换算成栅格半径，螺旋扩张搜索
  const double resolution = costmap_->getResolution();
  const int radius_cells = std::max(
    1, static_cast<int>(std::ceil(tolerance_ / resolution)));

  const unsigned int size_x = costmap_->getSizeInCellsX();
  const unsigned int size_y = costmap_->getSizeInCellsY();

  double best_dist2 = std::numeric_limits<double>::infinity();
  bool found = false;

  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const int mx = static_cast<int>(goal_mx) + dx;
      const int my = static_cast<int>(goal_my) + dy;
      if (mx < 0 || mx >= static_cast<int>(size_x) ||
        my < 0 || my >= static_cast<int>(size_y))
      {
        continue;
      }
      const unsigned char cost = costmap_->getCost(
        static_cast<unsigned int>(mx), static_cast<unsigned int>(my));
      if (!traversable(cost)) {
        continue;
      }
      const double dist2 = dx * dx + dy * dy;
      if (dist2 < best_dist2) {
        best_dist2 = dist2;
        out_mx = static_cast<unsigned int>(mx);
        out_my = static_cast<unsigned int>(my);
        found = true;
      }
    }
  }
  return found;
}

nav_msgs::msg::Path GradientAStarPlanner::indicesToPath(
  const std::vector<unsigned int> & indices,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.stamp = node_->now();
  path.header.frame_id = global_frame_;

  const unsigned int size_x = costmap_->getSizeInCellsX();
  const double resolution = costmap_->getResolution();
  double origin_x, origin_y;
  costmap_->getMapCoordinates(
    static_cast<unsigned int>(0), static_cast<unsigned int>(0), origin_x, origin_y);

  // 按相邻点最小间距降采样，控制路径点规模
  const double min_gap2 = interpolation_resolution_ * interpolation_resolution_;

  double wx_prev = 0.0, wy_prev = 0.0;
  auto push_world = [&](double wx, double wy, double yaw) {
      geometry_msgs::msg::PoseStamped p;
      p.header.stamp = node_->now();
      p.header.frame_id = global_frame_;
      p.pose.position.x = wx;
      p.pose.position.y = wy;
      p.pose.position.z = 0.0;
      p.pose.orientation.z = std::sin(yaw / 2.0);
      p.pose.orientation.w = std::cos(yaw / 2.0);
      path.poses.push_back(p);
    };

  for (size_t i = 0; i < indices.size(); ++i) {
    const unsigned int mx = indices[i] % size_x;
    const unsigned int my = indices[i] / size_x;
    double wx = origin_x + (mx + 0.5) * resolution;
    double wy = origin_y + (my + 0.5) * resolution;

    if (i > 0 && i + 1 < indices.size()) {
      const double d2 = (wx - wx_prev) * (wx - wx_prev) + (wy - wy_prev) * (wy - wy_prev);
      if (d2 < min_gap2) {
        continue;  // 距上一保留点太近，降采样跳过
      }
    }

    // 姿态朝向取路径切线方向（与下一个保留点连线）
    double yaw = 0.0;
    if (i + 1 < indices.size()) {
      const unsigned int nmx = indices[i + 1] % size_x;
      const unsigned int nmy = indices[i + 1] / size_x;
      double nwx = origin_x + (nmx + 0.5) * resolution;
      double nwy = origin_y + (nmy + 0.5) * resolution;
      yaw = std::atan2(nwy - wy, nwx - wx);
    }

    push_world(wx, wy, yaw);
    wx_prev = wx;
    wy_prev = wy;
  }

  // 末点用原始目标位姿（保留用户指定的目标朝向），避免降采样丢目标
  geometry_msgs::msg::PoseStamped goal_pose = goal;
  goal_pose.header.stamp = node_->now();
  goal_pose.header.frame_id = global_frame_;
  path.poses.push_back(goal_pose);

  return path;
}

}  // namespace nav2_gradient_planner

// pluginlib 导出：nav2_params.yaml 中 planner plugin 字段引用的类名
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  nav2_gradient_planner::GradientAStarPlanner,
  nav2_core::GlobalPlanner)
