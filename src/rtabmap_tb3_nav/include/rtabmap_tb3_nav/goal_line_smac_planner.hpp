#ifndef RTABMAP_TB3_NAV__GOAL_LINE_SMAC_PLANNER_HPP_
#define RTABMAP_TB3_NAV__GOAL_LINE_SMAC_PLANNER_HPP_

#include <string>
#include <utility>
#include <vector>

#include "nav2_smac_planner/smac_planner_2d.hpp"

namespace rtabmap_tb3_nav
{

/**
 * SmacPlanner2D with soft preferences for the current start-to-goal line and
 * an optional, bounded route-side hint.
 *
 * The preference is applied only for the duration of one planner call and
 * only to non-lethal cells. Obstacle, inflation and footprint costs therefore
 * remain the hard source of feasibility; the added costs only break detour
 * ties and encourage a return to the goal line after an obstacle. The side
 * hint is deliberately bounded in world X so it can make a benchmark route
 * deterministic without forcing a global left/right rule on every map.
 */
class GoalLineSmacPlanner : public nav2_smac_planner::SmacPlanner2D
{
public:
  GoalLineSmacPlanner() = default;
  ~GoalLineSmacPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  double distanceToSegment(
    double x, double y, double start_x, double start_y,
    double goal_x, double goal_y) const;

  bool line_bias_enabled_{true};
  double line_bias_max_cost_{60.0};
  double line_bias_distance_scale_{1.5};
  double line_bias_exponent_{2.0};
  bool side_bias_enabled_{false};
  int side_bias_preferred_y_sign_{1};
  double side_bias_max_cost_{45.0};
  double side_bias_distance_scale_{0.9};
  double side_bias_exponent_{2.0};
  double side_bias_world_x_min_{0.0};
  double side_bias_world_x_max_{0.0};
  bool side_bias_apply_to_unknown_{false};
  double side_bias_unknown_base_cost_{60.0};
  bool side_bias_target_world_y_enabled_{false};
  double side_bias_reference_world_y_{0.0};
  double side_bias_target_world_y_{0.0};
  double side_bias_target_offset_{0.0};
  double side_bias_target_max_cost_{30.0};
  double side_bias_target_distance_scale_{0.75};
  double side_bias_target_exponent_{2.0};
  bool side_bias_target_schedule_enabled_{false};
  std::vector<double> side_bias_target_schedule_x_;
  std::vector<double> side_bias_target_schedule_y_;
  rclcpp::Logger logger_{rclcpp::get_logger("GoalLineSmacPlanner")};
};

}  // namespace rtabmap_tb3_nav

#endif  // RTABMAP_TB3_NAV__GOAL_LINE_SMAC_PLANNER_HPP_
