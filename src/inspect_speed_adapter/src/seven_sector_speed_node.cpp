/**
 * @file seven_sector_speed_node.cpp
 * @brief 基于 360° 激光雷达七扇区分区感知的自适应调速节点
 *
 * 架构定位（面试要点）：
 *   不替换 TEB 控制器，而是挂载在 Nav2 标准 speed_limit 接口上。
 *   controller_server 订阅 "speed_limit" 话题（nav2_msgs/SpeedLimit），
 *   TEB 的 setSpeedLimit 会按百分比同步缩放 max_vel_x 与 max_vel_theta，
 *   正好实现"根据环境自动动态调整最大线速度、角速度"，且不侵入控制器。
 *
 * 七扇区布局（机器人坐标系，角度左正右负）：
 *                     后左     后右
 *                   左     ↑     右
 *                 前左     |     前右
 *                   \      |      /
 *                    \  前方  /
 *   每扇区约 51.4°（π/7），前方扇区以正前方为中心对称。
 *
 * 环境分类与限速策略：
 *   OPEN    开阔区域       前方与两侧均宽敞           → 100%（不干预）
 *   CORRIDOR 窄通道        左右两侧同近距离墙壁       → 55%（限线速度+角速度防蹭墙）
 *   CORNER  拐角           前方近距受阻               → 40%（保转向能力慢行）
 *   DENSE   密集/人流      前方+侧向多扇区近距离受阻   → 25%（为 RL 接管预留余量）
 *
 * 工程细节（防抖动，面试可讲）：
 *   1. 状态确认机制：环境分类需连续 N 帧扫描一致才切换，避免边界处来回跳变；
 *   2. 限速斜率控制：目标百分比按最大变化率逼近，避免速度限值阶跃引起颠簸；
 *   3. 扫描超时看门狗：雷达数据断流时回退安全限速，而不是维持旧值。
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

#include "nav2_msgs/msg/speed_limit.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

namespace inspect_speed_adapter
{

// 扇区索引定义
enum SectorId : uint8_t
{
  FRONT = 0, FRONT_LEFT, FRONT_RIGHT, LEFT, RIGHT, REAR_LEFT, REAR_RIGHT, SECTOR_NUM
};

// 环境状态
enum class EnvState : uint8_t { OPEN, CORRIDOR, CORNER, DENSE, UNKNOWN };

// 状态名（用于日志与诊断）
inline const char * stateName(EnvState s)
{
  switch (s) {
    case EnvState::OPEN: return "OPEN";
    case EnvState::CORRIDOR: return "CORRIDOR";
    case EnvState::CORNER: return "CORNER";
    case EnvState::DENSE: return "DENSE";
    case EnvState::UNKNOWN: return "UNKNOWN";
  }
  return "?";
}

class SevenSectorSpeedNode : public rclcpp::Node
{
public:
  explicit SevenSectorSpeedNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("seven_sector_speed", options)
  {
    // ---- 参数声明（默认值对应 0.5m/s 巡检机器人）----
    declare_parameter("corridor_threshold", 1.2);   // 左右同近时判定窄通道的门槛（米）
    declare_parameter("corner_threshold", 1.0);     // 前方受阻判定拐角的门槛（米）
    declare_parameter("danger_threshold", 0.45);    // 密集/人流判定门槛（米）
    declare_parameter("effective_max_range", 6.0);  // 有效感知距离，超过按此值封顶（米）
    declare_parameter("confirm_scans", 3);          // 状态切换所需连续一致帧数（防抖）
    declare_parameter("publish_period", 0.1);        // 限速发布周期（秒）
    declare_parameter("max_step_percent", 20.0);    // 限速最大变化率（%/s），平滑逼近
    // 各状态目标限速（百分比，作用于 TEB max_vel_x / max_vel_theta）
    declare_parameter("limit_open", 100.0);
    declare_parameter("limit_corridor", 55.0);
    declare_parameter("limit_corner", 40.0);
    declare_parameter("limit_dense", 25.0);
    declare_parameter("limit_unknown", 60.0);       // 感知异常时的安全回退限速

    corridor_th_ = get_parameter("corridor_threshold").as_double();
    corner_th_ = get_parameter("corner_threshold").as_double();
    danger_th_ = get_parameter("danger_threshold").as_double();
    eff_max_range_ = get_parameter("effective_max_range").as_double();
    confirm_scans_ = static_cast<int>(get_parameter("confirm_scans").as_int());
    const double period = get_parameter("publish_period").as_double();
    max_step_percent_ = get_parameter("max_step_percent").as_double();
    limit_[static_cast<uint8_t>(EnvState::OPEN)] = get_parameter("limit_open").as_double();
    limit_[static_cast<uint8_t>(EnvState::CORRIDOR)] = get_parameter("limit_corridor").as_double();
    limit_[static_cast<uint8_t>(EnvState::CORNER)] = get_parameter("limit_corner").as_double();
    limit_[static_cast<uint8_t>(EnvState::DENSE)] = get_parameter("limit_dense").as_double();
    limit_[static_cast<uint8_t>(EnvState::UNKNOWN)] = get_parameter("limit_unknown").as_double();

    // ---- 订阅与发布 ----
    // 激光雷达：10 档 QoS 传感器数据（best effort）
    auto scan_qos = rclcpp::SensorDataQoS();
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", scan_qos,
      std::bind(&SevenSectorSpeedNode::onScan, this, std::placeholders::_1));

    // Nav2 标准限速话题：controller_server 订阅后调用 TEB 的 setSpeedLimit
    speed_limit_pub_ = create_publisher<nav2_msgs::msg::SpeedLimit>("speed_limit", 1);

    // 诊断信息：各扇区最近距离 + 当前环境状态（供 RViz / rostopic 观察）
    diag_pub_ = create_publisher<std_msgs::msg::String>("~/diagnostics", 1);

    // 周期发布定时器：限速斜率控制也在这里做
    timer_ = create_wall_timer(
      std::chrono::duration<double>(period),
      std::bind(&SevenSectorSpeedNode::onTimer, this));

    current_pct_ = limit_[static_cast<uint8_t>(EnvState::UNKNOWN)];
    has_published_ = false;

    RCLCPP_INFO(
      get_logger(),
      "七扇区调速节点已启动: corridor=%.2fm corner=%.2fm danger=%.2fm 确认帧数=%d",
      corridor_th_, corner_th_, danger_th_, confirm_scans_);
  }

private:
  /**
   * @brief 激光回调：重采样到七扇区，取每扇区最近有效距离
   * 任意分辨率的 LaserScan 都能处理：逐点按角度归扇区，取最小值。
   */
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    last_scan_time_ = now();
    sector_min_.fill(std::numeric_limits<double>::infinity());

    const double r_min = msg->range_min;
    const double r_max = std::min(static_cast<double>(msg->range_max), eff_max_range_);

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const double r = msg->ranges[i];
      if (!std::isfinite(r) || r < r_min || r > r_max) {
        continue;  // 无效点 / inf 点（未探测到障碍）不计入
      }
      // 归一化角度到 [-pi, pi]
      double a = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
      while (a > M_PI) {a -= 2.0 * M_PI;}
      while (a < -M_PI) {a += 2.0 * M_PI;}
      const uint8_t s = sectorOf(a);
      sector_min_[s] = std::min(sector_min_[s], r);
    }
    scan_received_ = true;

    // 环境分类（带确认机制的候选状态）
    updateStateWithHysteresis(classify());
  }

  /** @brief 角度 → 扇区号（前方扇区以正前为中心对称分布） */
  static uint8_t sectorOf(double a)
  {
    const double b = std::fabs(a);
    const double s1 = M_PI / 7.0;    // 25.7°：前方扇区半宽
    const double s2 = 3.0 * M_PI / 7.0;   // 77.1°
    const double s3 = 5.0 * M_PI / 7.0;  // 128.6°
    if (b <= s1) {return FRONT;}
    if (b <= s2) {return a > 0.0 ? FRONT_LEFT : FRONT_RIGHT;}
    if (b <= s3) {return a > 0.0 ? LEFT : RIGHT;}
    return a > 0.0 ? REAR_LEFT : REAR_RIGHT;
  }

  /** @brief 当前扇区最近距离 → 环境候选状态（优先级：密集 > 拐角 > 窄通道 > 开阔） */
  EnvState classify() const
  {
    const double d_f = sector_min_[FRONT];
    const double d_fl = sector_min_[FRONT_LEFT];
    const double d_fr = sector_min_[FRONT_RIGHT];
    const double d_l = sector_min_[LEFT];
    const double d_r = sector_min_[RIGHT];

    // 密集/人流：前方受阻且侧向同时存在近距障碍（人群连片特征）
    if (d_f < danger_th_ &&
      (d_fl < danger_th_ || d_fr < danger_th_ || d_l < danger_th_ || d_r < danger_th_))
    {
      return EnvState::DENSE;
    }
    // 拐角：前方近距受阻但未构成密集
    if (d_f < corner_th_) {
      return EnvState::CORNER;
    }
    // 窄通道：左右两侧同时贴近墙壁
    if (d_l < corridor_th_ && d_r < corridor_th_) {
      return EnvState::CORRIDOR;
    }
    return EnvState::OPEN;
  }

  /**
   * @brief 状态切换防抖：候选状态需连续 confirm_scans_ 帧一致才提交
   * 场景：通道口/拐角边缘距离在门槛附近抖动时，避免限速来回跳变。
   */
  void updateStateWithHysteresis(EnvState candidate)
  {
    if (candidate == state_) {
      candidate_streak_ = 0;  // 已是当前状态，重置候选计数
      return;
    }
    if (candidate != last_candidate_) {
      last_candidate_ = candidate;
      candidate_streak_ = 1;
      return;
    }
    if (++candidate_streak_ >= confirm_scans_) {
      RCLCPP_INFO(
        get_logger(), "环境状态切换: %s -> %s",
        stateName(state_), stateName(candidate));
      state_ = candidate;
      candidate_streak_ = 0;
    }
  }

  /** @brief 周期任务：看门狗检查 + 限速斜率逼近 + 发布 */
  void onTimer()
  {
    // 看门狗：扫描断流超过 3 个发布周期 → 感知异常，进入 UNKNOWN 安全限速
    if (!scan_received_ ||
      (now() - last_scan_time_).seconds() > 3.0 * publish_period_guess_)
    {
      state_ = EnvState::UNKNOWN;
    }

    // 目标限速
    const double target = limit_[static_cast<uint8_t>(state_)];

    // 斜率控制：按 max_step_percent（%/s）向目标逼近，避免限值阶跃
    const double max_step = max_step_percent_ * publish_period_guess_;
    double diff = target - current_pct_;
    diff = std::clamp(diff, -max_step, max_step);
    current_pct_ += diff;

    // 发布条件：逼近中（仍有差距）或首次发布
    if (std::fabs(target - current_pct_) > 1e-6 || !has_published_) {
      nav2_msgs::msg::SpeedLimit sl;
      sl.speed_limit = current_pct_;
      sl.percentage = true;  // true 表示百分比缩放（false 为绝对值 m/s）
      speed_limit_pub_->publish(sl);
      has_published_ = true;
    }

    // 诊断信息（每周期一条，量小可接受）
    std_msgs::msg::String diag;
    std::ostringstream oss;
    oss << "state=" << stateName(state_)
        << " limit=" << current_pct_ << "%"
        << " sectors=[F:" << fmt(sector_min_[FRONT])
        << " FL:" << fmt(sector_min_[FRONT_LEFT])
        << " FR:" << fmt(sector_min_[FRONT_RIGHT])
        << " L:" << fmt(sector_min_[LEFT])
        << " R:" << fmt(sector_min_[RIGHT]) << "]";
    diag.data = oss.str();
    diag_pub_->publish(diag);
  }

  static std::string fmt(double v)
  {
    if (!std::isfinite(v)) {return "inf";}
    std::ostringstream o;
    o << std::fixed << std::setprecision(2) << v;
    return o.str();
  }

  // ---- 参数 ----
  double corridor_th_{1.2};
  double corner_th_{1.0};
  double danger_th_{0.45};
  double eff_max_range_{6.0};
  int confirm_scans_{3};
  double publish_period_guess_{0.1};
  double max_step_percent_{20.0};
  std::array<double, 5> limit_{100.0, 55.0, 40.0, 25.0, 60.0};  // 索引同 EnvState

  // ---- 状态 ----
  std::array<double, SECTOR_NUM> sector_min_{
    std::numeric_limits<double>::infinity()};
  EnvState state_{EnvState::UNKNOWN};
  EnvState last_candidate_{EnvState::UNKNOWN};
  int candidate_streak_{0};
  double current_pct_{60.0};
  bool has_published_{false};
  bool scan_received_{false};
  rclcpp::Time last_scan_time_;

  // ---- ROS 接口 ----
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace inspect_speed_adapter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<inspect_speed_adapter::SevenSectorSpeedNode>());
  rclcpp::shutdown();
  return 0;
}
