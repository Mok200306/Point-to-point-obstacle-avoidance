#ifndef RTABMAP_TB3_NAV__GOAL_LINE_SMAC_PLANNER_HPP_
#define RTABMAP_TB3_NAV__GOAL_LINE_SMAC_PLANNER_HPP_

#include <string>
#include <utility>
#include <vector>

#include "nav2_smac_planner/smac_planner_2d.hpp"

namespace rtabmap_tb3_nav
{

/**
 * SmacPlanner2D with a soft preference for the current start-to-goal line.
 *
 * The preference is applied only for the duration of one planner call and
 * only to non-lethal cells. Obstacle, inflation and footprint costs therefore
 * remain the hard source of feasibility; the added cost only breaks detour
 * ties and encourages a return to the goal line after an obstacle.
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
  rclcpp::Logger logger_{rclcpp::get_logger("GoalLineSmacPlanner")};
};

}  // namespace rtabmap_tb3_nav

#endif  // RTABMAP_TB3_NAV__GOAL_LINE_SMAC_PLANNER_HPP_
