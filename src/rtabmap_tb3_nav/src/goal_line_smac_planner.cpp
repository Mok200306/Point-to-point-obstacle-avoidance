#include "rtabmap_tb3_nav/goal_line_smac_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <stdexcept>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace rtabmap_tb3_nav
{

void GoalLineSmacPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  nav2_smac_planner::SmacPlanner2D::configure(parent, name, tf, costmap_ros);

  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("GoalLineSmacPlanner parent node is unavailable");
  }

  node->declare_parameter(name + ".line_bias_enabled", line_bias_enabled_);
  node->declare_parameter(name + ".line_bias_max_cost", line_bias_max_cost_);
  node->declare_parameter(
    name + ".line_bias_distance_scale", line_bias_distance_scale_);
  node->declare_parameter(name + ".line_bias_exponent", line_bias_exponent_);
  node->declare_parameter(name + ".side_bias_enabled", side_bias_enabled_);
  node->declare_parameter(
    name + ".side_bias_preferred_y_sign", side_bias_preferred_y_sign_);
  node->declare_parameter(name + ".side_bias_max_cost", side_bias_max_cost_);
  node->declare_parameter(
    name + ".side_bias_distance_scale", side_bias_distance_scale_);
  node->declare_parameter(name + ".side_bias_exponent", side_bias_exponent_);
  node->declare_parameter(
    name + ".side_bias_world_x_min", side_bias_world_x_min_);
  node->declare_parameter(
    name + ".side_bias_world_x_max", side_bias_world_x_max_);
  node->declare_parameter(
    name + ".side_bias_apply_to_unknown", side_bias_apply_to_unknown_);
  node->declare_parameter(
    name + ".side_bias_unknown_base_cost", side_bias_unknown_base_cost_);
  node->declare_parameter(
    name + ".side_bias_target_world_y_enabled", side_bias_target_world_y_enabled_);
  node->declare_parameter(
    name + ".side_bias_reference_world_y", side_bias_reference_world_y_);
  node->declare_parameter(
    name + ".side_bias_target_world_y", side_bias_target_world_y_);
  node->declare_parameter(
    name + ".side_bias_target_offset", side_bias_target_offset_);
  node->declare_parameter(
    name + ".side_bias_target_max_cost", side_bias_target_max_cost_);
  node->declare_parameter(
    name + ".side_bias_target_distance_scale", side_bias_target_distance_scale_);
  node->declare_parameter(
    name + ".side_bias_target_exponent", side_bias_target_exponent_);
  node->get_parameter(name + ".line_bias_enabled", line_bias_enabled_);
  node->get_parameter(name + ".line_bias_max_cost", line_bias_max_cost_);
  node->get_parameter(
    name + ".line_bias_distance_scale", line_bias_distance_scale_);
  node->get_parameter(name + ".line_bias_exponent", line_bias_exponent_);
  node->get_parameter(name + ".side_bias_enabled", side_bias_enabled_);
  node->get_parameter(
    name + ".side_bias_preferred_y_sign", side_bias_preferred_y_sign_);
  node->get_parameter(name + ".side_bias_max_cost", side_bias_max_cost_);
  node->get_parameter(
    name + ".side_bias_distance_scale", side_bias_distance_scale_);
  node->get_parameter(name + ".side_bias_exponent", side_bias_exponent_);
  node->get_parameter(
    name + ".side_bias_world_x_min", side_bias_world_x_min_);
  node->get_parameter(
    name + ".side_bias_world_x_max", side_bias_world_x_max_);
  node->get_parameter(
    name + ".side_bias_apply_to_unknown", side_bias_apply_to_unknown_);
  node->get_parameter(
    name + ".side_bias_unknown_base_cost", side_bias_unknown_base_cost_);
  node->get_parameter(
    name + ".side_bias_target_world_y_enabled", side_bias_target_world_y_enabled_);
  node->get_parameter(
    name + ".side_bias_reference_world_y", side_bias_reference_world_y_);
  node->get_parameter(
    name + ".side_bias_target_world_y", side_bias_target_world_y_);
  node->get_parameter(
    name + ".side_bias_target_offset", side_bias_target_offset_);
  node->get_parameter(
    name + ".side_bias_target_max_cost", side_bias_target_max_cost_);
  node->get_parameter(
    name + ".side_bias_target_distance_scale", side_bias_target_distance_scale_);
  node->get_parameter(
    name + ".side_bias_target_exponent", side_bias_target_exponent_);

  line_bias_max_cost_ = std::clamp(line_bias_max_cost_, 0.0, 252.0);
  line_bias_distance_scale_ = std::max(line_bias_distance_scale_, 0.05);
  line_bias_exponent_ = std::max(line_bias_exponent_, 1.0);
  side_bias_preferred_y_sign_ = side_bias_preferred_y_sign_ >= 0 ? 1 : -1;
  side_bias_max_cost_ = std::clamp(side_bias_max_cost_, 0.0, 252.0);
  side_bias_distance_scale_ = std::max(side_bias_distance_scale_, 0.05);
  side_bias_exponent_ = std::max(side_bias_exponent_, 1.0);
  side_bias_unknown_base_cost_ = std::clamp(
    side_bias_unknown_base_cost_, 1.0, 200.0);
  side_bias_target_world_y_ = std::clamp(
    side_bias_target_world_y_, -100.0, 100.0);
  side_bias_reference_world_y_ = std::clamp(
    side_bias_reference_world_y_, -100.0, 100.0);
  side_bias_target_offset_ = std::max(side_bias_target_offset_, 0.0);
  side_bias_target_max_cost_ = std::clamp(side_bias_target_max_cost_, 0.0, 252.0);
  side_bias_target_distance_scale_ = std::max(side_bias_target_distance_scale_, 0.05);
  side_bias_target_exponent_ = std::max(side_bias_target_exponent_, 1.0);
  if (side_bias_world_x_min_ > side_bias_world_x_max_) {
    side_bias_enabled_ = false;
  }

  RCLCPP_INFO(
    logger_,
    "Goal-line bias: enabled=%s max_cost=%.1f distance_scale=%.2f m exponent=%.2f; "
    "side bias: enabled=%s preferred_y_sign=%d max_cost=%.1f x=[%.2f, %.2f] "
    "apply_to_unknown=%s unknown_base_cost=%.1f target_world_y=%s ref_y=%.2f "
    "target_y=%.2f target_offset=%.2f target_cost=%.1f",
    line_bias_enabled_ ? "true" : "false", line_bias_max_cost_,
    line_bias_distance_scale_, line_bias_exponent_,
    side_bias_enabled_ ? "true" : "false", side_bias_preferred_y_sign_,
    side_bias_max_cost_, side_bias_world_x_min_, side_bias_world_x_max_,
    side_bias_apply_to_unknown_ ? "true" : "false",
    side_bias_unknown_base_cost_,
    side_bias_target_world_y_enabled_ ? "true" : "false",
    side_bias_reference_world_y_, side_bias_target_world_y_,
    side_bias_target_offset_,
    side_bias_target_max_cost_);
}

double GoalLineSmacPlanner::distanceToSegment(
  double x, double y, double start_x, double start_y,
  double goal_x, double goal_y) const
{
  const double dx = goal_x - start_x;
  const double dy = goal_y - start_y;
  const double length_squared = dx * dx + dy * dy;
  if (length_squared <= std::numeric_limits<double>::epsilon()) {
    return std::hypot(x - start_x, y - start_y);
  }

  const double projection =
    ((x - start_x) * dx + (y - start_y) * dy) / length_squared;
  const double clamped_projection = std::clamp(projection, 0.0, 1.0);
  const double closest_x = start_x + clamped_projection * dx;
  const double closest_y = start_y + clamped_projection * dy;
  return std::hypot(x - closest_x, y - closest_y);
}

nav_msgs::msg::Path GoalLineSmacPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  if ((!line_bias_enabled_ || line_bias_max_cost_ <= 0.0) &&
    (!side_bias_enabled_ || side_bias_max_cost_ <= 0.0))
  {
    return nav2_smac_planner::SmacPlanner2D::createPlan(start, goal);
  }

  if (_costmap == nullptr) {
    return nav2_smac_planner::SmacPlanner2D::createPlan(start, goal);
  }

  auto * costmap = _costmap;
  // Smac reads the same char map while planning. Hold the costmap lock for
  // both the temporary bias and the inherited planner call so an update can
  // never observe a partially modified grid.
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(
    *costmap->getMutex());
  auto * char_map = costmap->getCharMap();
  const unsigned int size_x = costmap->getSizeInCellsX();
  const unsigned int size_y = costmap->getSizeInCellsY();
  std::vector<std::pair<unsigned int, unsigned char>> changed_cells;
  changed_cells.reserve(static_cast<size_t>(size_x) * size_y / 8U);

  const double start_x = start.pose.position.x;
  const double start_y = start.pose.position.y;
  const double goal_x = goal.pose.position.x;
  const double goal_y = goal.pose.position.y;

  for (unsigned int my = 0; my < size_y; ++my) {
    for (unsigned int mx = 0; mx < size_x; ++mx) {
      const unsigned int index = costmap->getIndex(mx, my);
      const unsigned char original_cost = costmap->getCost(index);

      const bool unknown = original_cost == nav2_costmap_2d::NO_INFORMATION;
      // Keep inflated/lethal cells unchanged. Unknown is normally left alone
      // too; an explicit benchmark option below can turn only the bounded
      // side-hint window into a soft traversability preference while
      // allow_unknown remains true in Smac.
      if (original_cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE &&
        !unknown)
      {
        continue;
      }

      double world_x;
      double world_y;
      costmap->mapToWorld(mx, my, world_x, world_y);
      unsigned int soft_cost = unknown ?
        static_cast<unsigned int>(std::lround(side_bias_unknown_base_cost_)) :
        static_cast<unsigned int>(original_cost);
      bool unknown_was_adjusted = false;

      if (!unknown && line_bias_enabled_ && line_bias_max_cost_ > 0.0) {
        const double distance = distanceToSegment(
          world_x, world_y, start_x, start_y, goal_x, goal_y);
        const double ratio = distance / line_bias_distance_scale_;
        const auto line_cost = static_cast<unsigned int>(std::lround(
          std::min(
            line_bias_max_cost_ * std::pow(ratio, line_bias_exponent_),
            252.0)));
        soft_cost = unknown ? soft_cost + line_cost : std::max(soft_cost, line_cost);
      }

      if (side_bias_enabled_ && side_bias_max_cost_ > 0.0 &&
        world_x >= side_bias_world_x_min_ && world_x <= side_bias_world_x_max_)
      {
        const double dx = goal_x - start_x;
        const double dy = goal_y - start_y;
        const double line_length = std::hypot(dx, dy);
        if (line_length > std::numeric_limits<double>::epsilon()) {
          // In the fixed-world-Y benchmark mode, use a map-fixed side
          // preference. Recomputing the cross-track side from the current
          // replanning start would move the preferred corridor every time
          // the robot made progress, which can cause runaway detours.
          const double signed_distance = side_bias_target_world_y_enabled_ ?
            static_cast<double>(side_bias_preferred_y_sign_) *
            (world_y - side_bias_reference_world_y_) :
            (dx * (world_y - start_y) - dy * (world_x - start_x)) / line_length;
          const double opposite_distance = std::max(
            0.0,
            -static_cast<double>(side_bias_preferred_y_sign_) * signed_distance);
          const double ratio = opposite_distance / side_bias_distance_scale_;
          const auto side_cost = static_cast<unsigned int>(std::lround(
            std::min(
              side_bias_max_cost_ * std::pow(ratio, side_bias_exponent_),
              252.0)));
          soft_cost = unknown ? soft_cost + side_cost : std::max(soft_cost, side_cost);

          if (side_bias_target_offset_ > 0.0 && side_bias_target_max_cost_ > 0.0) {
            const double target_distance = side_bias_target_world_y_enabled_ ?
              std::abs(world_y - side_bias_target_world_y_) :
              std::abs(
                signed_distance -
                static_cast<double>(side_bias_preferred_y_sign_) * side_bias_target_offset_);
            const double target_ratio = target_distance / side_bias_target_distance_scale_;
            const auto target_cost = static_cast<unsigned int>(std::lround(
              std::min(
                side_bias_target_max_cost_ *
                std::pow(target_ratio, side_bias_target_exponent_),
                252.0)));
            soft_cost = unknown ? soft_cost + target_cost : std::max(soft_cost, target_cost);
          }

          unknown_was_adjusted = unknown && side_bias_apply_to_unknown_;
        }
      }

      soft_cost = std::min(soft_cost, 252U);

      if ((!unknown && soft_cost <= static_cast<unsigned int>(original_cost)) ||
        (unknown && !unknown_was_adjusted))
      {
        continue;
      }

      changed_cells.emplace_back(index, original_cost);
      char_map[index] = static_cast<unsigned char>(soft_cost);
    }
  }

  try {
    auto plan = nav2_smac_planner::SmacPlanner2D::createPlan(start, goal);
    for (const auto & changed : changed_cells) {
      char_map[changed.first] = changed.second;
    }
    return plan;
  } catch (...) {
    for (const auto & changed : changed_cells) {
      char_map[changed.first] = changed.second;
    }
    throw;
  }
}

}  // namespace rtabmap_tb3_nav

PLUGINLIB_EXPORT_CLASS(
  rtabmap_tb3_nav::GoalLineSmacPlanner, nav2_core::GlobalPlanner)
