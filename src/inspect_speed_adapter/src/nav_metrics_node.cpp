/**
 * @file nav_metrics_node.cpp
 * @brief 导航指标观测节点：规划时延 / 任务完成率 / 路径平滑度 / 卡死检测
 *
 * 作为系统迭代依据（简历原文），采集四类指标：
 *   1. 规划时延 plan_latency_ms：路径消息时间戳与接收时刻的差值，
 *      反映"规划完成 → 消费端收到"的端到端延迟；
 *   2. 任务完成率 task_success_rate：订阅 NavigateToPose 动作状态数组，
 *      按 goal_id 去重统计 SUCCEEDED / ABORTED / CANCELED 终态；
 *   3. 路径平滑度 path_smoothness：相邻路径点航向角变化的绝对值均值 / 路径长度
 *      （rad/m），数值越小说明路径越平滑，用于评估规划器与平滑器效果；
 *   4. 规划冻结 freeze_max_s：目标执行期间零速连续最大时长，
 *      是 RL 避障模块接管时机设计的实测依据。
 *
 * 指标通过 ~/metrics（Float64MultiArray，字段顺序见下方 kMetricNames）
 * 周期发布，并同步打印一行摘要日志；提供 ~/reset_metrics 服务清零计数器。
 */

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include <unordered_set>
#include <vector>

#include "action_msgs/msg/goal_status_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace inspect_speed_adapter
{

// 指标字段顺序（发布数组与此一致，便于消费端解析）
static const std::array<const char *, 8> kMetricNames = {
  "plan_latency_ms",      // 0 规划端到端时延（均值，ms）
  "path_smoothness",      // 1 路径平滑度（rad/m，越小越平滑）
  "path_length_m",        // 2 最近一条路径长度（米）
  "task_success_rate",    // 3 任务完成率 [0,1]
  "task_total",           // 4 已结束任务总数
  "task_succeeded",       // 5 成功任务数
  "freeze_max_s",         // 6 最大零速连续时长（秒，冻结/卡死检测）
  "cmd_vel_mean_v"        // 7 平均指令线速度（m/s）
};

class NavMetricsNode : public rclcpp::Node
{
public:
  explicit NavMetricsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("nav_metrics", options)
  {
    declare_parameter("metrics_period", 1.0);          // 指标发布周期（秒）
    declare_parameter("freeze_vel_threshold", 0.03);  // 零速判定阈值 |v|、|w|（m/s, rad/s）
    declare_parameter("freeze_active_only", true);    // 仅统计任务执行中的冻结

    const double period = get_parameter("metrics_period").as_double();
    freeze_th_ = get_parameter("freeze_vel_threshold").as_double();
    freeze_active_only_ = get_parameter("freeze_active_only").as_bool();

    // 路径：规划器输出（nav2 planner_server 默认发布话题）
    plan_sub_ = create_subscription<nav_msgs::msg::Path>(
      "plan", rclcpp::QoS(1),
      std::bind(&NavMetricsNode::onPlan, this, std::placeholders::_1));

    // 控制器输出指令：速度统计与冻结检测
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(10),
      std::bind(&NavMetricsNode::onCmdVel, this, std::placeholders::_1));

    // NavigateToPose 动作状态数组：任务完成率统计
    status_sub_ = create_subscription<action_msgs::msg::GoalStatusArray>(
      "navigate_to_pose_status", rclcpp::QoS(10),
      std::bind(&NavMetricsNode::onStatus, this, std::placeholders::_1));

    // 指标发布与摘要日志
    metrics_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/metrics", 10);

    // 指标清零服务（每轮巡检开始前调用，轮次间隔离）
    reset_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/reset_metrics",
      [this](std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr res) {
        resetMetrics();
        res->success = true;
        res->message = "指标计数器已清零";
      });

    timer_ = create_wall_timer(
      std::chrono::duration<double>(period),
      std::bind(&NavMetricsNode::onTimer, this));

    RCLCPP_INFO(get_logger(), "导航指标观测节点已启动，监听 /plan /cmd_vel /动作状态");
  }

private:
  /** @brief 路径回调：规划时延 / 路径长度 / 平滑度 */
  void onPlan(const nav_msgs::msg::Path::SharedPtr msg)
  {
    // 规划时延：接收时刻 - 消息时间戳（规划器在生成路径时打的时间戳）
    const double latency_ms = (now() - rclcpp::Time(msg->header.stamp)).seconds() * 1000.0;
    latency_sum_ms_ += latency_ms;
    latency_cnt_++;

    // 路径长度 + 平滑度：相邻点航向角变化均值 / 长度
    double length = 0.0;
    double heading_change_sum = 0.0;
    double prev_h = std::numeric_limits<double>::quiet_NaN();
    for (size_t i = 0; i + 1 < msg->poses.size(); ++i) {
      const auto & p0 = msg->poses[i].pose.position;
      const auto & p1 = msg->poses[i + 1].pose.position;
      const double dx = p1.x - p0.x;
      const double dy = p1.y - p0.y;
      const double seg = std::hypot(dx, dy);
      if (seg < 1e-6) {continue;}
      const double h = std::atan2(dy, dx);
      if (std::isfinite(prev_h)) {
        double dh = std::fabs(h - prev_h);
        if (dh > M_PI) {dh = 2.0 * M_PI - dh;}  // 航向差归一到 [0, pi]
        heading_change_sum += dh;
      }
      prev_h = h;
      length += seg;
    }
    if (msg->poses.size() >= 3 && length > 1e-6) {
      last_path_length_ = length;
      last_smoothness_ = heading_change_sum / length;  // rad/m
    }
    plan_cnt_++;
  }

  /** @brief 速度指令回调：均值统计 + 冻结检测 */
  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const bool zero_vel =
      std::fabs(msg->linear.x) < freeze_th_ && std::fabs(msg->angular.z) < freeze_th_;

    const bool counting = freeze_active_only_ ? goal_active_ : true;
    if (zero_vel && counting) {
      // 持续零速：累计冻结时长
      const rclcpp::Time t = now();
      if (freeze_start_.nanoseconds() == 0) {
        freeze_start_ = t;
      }
      const double dur = (t - freeze_start_).seconds();
      freeze_max_s_ = std::max(freeze_max_s_, dur);
    } else {
      freeze_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);  // 任意非零速度即解除冻结计时
    }
    v_sum_ += std::fabs(msg->linear.x);
    cmd_cnt_++;
  }

  /** @brief 动作状态回调：按 goal_id 去重统计终态（状态数组是全量重发的） */
  void onStatus(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
  {
    goal_active_ = false;
    for (const auto & st : msg->status_list) {
      const std::string gid = uuidToHex(st.goal_info.goal_id.uuid);
      // 有任务处于非终态 → 当前有活动目标（供冻结检测使用）
      if (st.status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
        st.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
        st.status == action_msgs::msg::GoalStatus::STATUS_CANCELING)
      {
        goal_active_ = true;
      }
      // 终态只统计一次
      if (st.status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED ||
        st.status == action_msgs::msg::GoalStatus::STATUS_ABORTED ||
        st.status == action_msgs::msg::GoalStatus::STATUS_CANCELED)
      {
        if (seen_goal_ids_.insert(gid).second) {  // 新出现的 goal_id
          task_total_++;
          if (st.status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
            task_succeeded_++;
          }
        }
      }
    }
  }

  /** @brief 周期发布：Float64MultiArray（字段顺序同 kMetricNames）+ 摘要日志 */
  void onTimer()
  {
    std_msgs::msg::Float64MultiArray out;
    out.data.resize(kMetricNames.size());

    out.data[0] = latency_cnt_ > 0 ? latency_sum_ms_ / latency_cnt_ : 0.0;
    out.data[1] = std::isfinite(last_smoothness_) ? last_smoothness_ : 0.0;
    out.data[2] = last_path_length_;
    out.data[3] = task_total_ > 0
      ? static_cast<double>(task_succeeded_) / static_cast<double>(task_total_)
      : 1.0;  // 无任务时按 1.0 而不是 NaN，便于下游直接聚合
    out.data[4] = static_cast<double>(task_total_);
    out.data[5] = static_cast<double>(task_succeeded_);
    out.data[6] = freeze_max_s_;
    out.data[7] = cmd_cnt_ > 0 ? v_sum_ / cmd_cnt_ : 0.0;

    metrics_pub_->publish(out);

    // 一行摘要：巡检现场看日志就能拿到核心指标
    RCLCPP_INFO(
      get_logger(),
      "[指标] 规划时延=%.1fms 平滑度=%.3frad/m 任务=%zu/%zu(%.0f%%) 冻结max=%.1fs 均速=%.2fm/s",
      out.data[0], out.data[1],
      task_succeeded_, task_total_, out.data[3] * 100.0,
      out.data[6], out.data[7]);
  }

  void resetMetrics()
  {
    latency_sum_ms_ = 0.0;
    latency_cnt_ = 0;
    last_smoothness_ = std::numeric_limits<double>::quiet_NaN();
    last_path_length_ = 0.0;
    plan_cnt_ = 0;
    task_total_ = 0;
    task_succeeded_ = 0;
    freeze_max_s_ = 0.0;
    v_sum_ = 0.0;
    cmd_cnt_ = 0;
    freeze_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    seen_goal_ids_.clear();
  }

  /** @brief UUID(16字节) 转十六进制字符串，用于动作目标去重 */
  static std::string uuidToHex(const std::array<uint8_t, 16> & u)
  {
    static const char * hex = "0123456789abcdef";
    std::string s;
    s.reserve(32);
    for (uint8_t b : u) {
      s.push_back(hex[b >> 4]);
      s.push_back(hex[b & 0x0F]);
    }
    return s;
  }

  // ---- 参数 ----
  double freeze_th_{0.03};
  bool freeze_active_only_{true};

  // ---- 统计量 ----
  double latency_sum_ms_{0.0};
  uint64_t latency_cnt_{0};
  double last_smoothness_{std::numeric_limits<double>::quiet_NaN()};
  double last_path_length_{0.0};
  uint64_t plan_cnt_{0};
  size_t task_total_{0};
  size_t task_succeeded_{0};
  double freeze_max_s_{0.0};
  rclcpp::Time freeze_start_{0, 0, RCL_ROS_TIME};
  bool goal_active_{false};
  double v_sum_{0.0};
  uint64_t cmd_cnt_{0};
  std::unordered_set<std::string> seen_goal_ids_;

  // ---- ROS 接口 ----
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr metrics_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace inspect_speed_adapter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<inspect_speed_adapter::NavMetricsNode>());
  rclcpp::shutdown();
  return 0;
}
