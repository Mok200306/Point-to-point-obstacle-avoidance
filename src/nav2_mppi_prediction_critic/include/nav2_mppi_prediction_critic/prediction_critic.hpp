#ifndef NAV2_MPPI_PREDICTION_CRITIC__PREDICTION_CRITIC_HPP_
#define NAV2_MPPI_PREDICTION_CRITIC__PREDICTION_CRITIC_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_prediction_critic/footprint_sampling.hpp"
#include "nav2_mppi_prediction_critic/prediction_grid_sampler.hpp"
#include "oracle_dynamic_nav_msgs/msg/predicted_occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mppi::critics
{

class PredictionCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(CriticData & data) override;

private:
  struct GridSnapshot
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    std::string frame_id;
    float resolution{0.0F};
    unsigned int width{0};
    unsigned int height{0};
    float origin_x{0.0F};
    float origin_y{0.0F};
    float dt{0.0F};
    unsigned int steps{0};
    std::vector<float> data;
    std::size_t occupied_cells{0};
    std::size_t occupied_values{0};
    float message_max_risk{0.0F};
    bool has_occupied_extent{false};
    float occupied_min_x{0.0F};
    float occupied_max_x{0.0F};
    float occupied_min_y{0.0F};
    float occupied_max_y{0.0F};
  };

  using Message = oracle_dynamic_nav_msgs::msg::PredictedOccupancyGrid;

  void messageCallback(const Message::SharedPtr message);
  std::shared_ptr<const GridSnapshot> latestSnapshot() const;
  void logStatus(
    const char * status, double age_s, float max_risk, float max_cost,
    std::size_t valid_samples, std::size_t out_of_horizon,
    std::size_t out_of_bounds, std::size_t risk_hits) const;
  void logGeometry(
    const GridSnapshot & grid, const CriticData & data,
    std::size_t batch_size, std::size_t trajectory_steps,
    float candidate_min_x, float candidate_max_x,
    float candidate_min_y, float candidate_max_y,
    std::size_t valid_samples, std::size_t risk_hits,
    std::size_t out_of_bounds, std::size_t out_of_horizon,
    double risk_hit_min_tau_s, double risk_hit_max_tau_s) const;

  rclcpp::Subscription<Message>::SharedPtr subscription_;
  rclcpp::Clock::SharedPtr clock_;

  mutable std::mutex snapshot_mutex_;
  std::shared_ptr<const GridSnapshot> snapshot_;

  std::string topic_;
  std::string expected_frame_;
  std::string interpolation_;
  float weight_{50.0F};
  unsigned int power_{1};
  double stale_threshold_s_{0.30};
  double clock_skew_tolerance_s_{0.05};
  bool enabled_for_prediction_{true};
  bool use_footprint_{true};
  unsigned int footprint_edge_samples_{2U};
  std::vector<FootprintSample> footprint_samples_;
  bool ignore_out_of_bounds_{true};
  bool ignore_out_of_horizon_{true};
  PredictionGridSampler sampler_;

  mutable std::atomic<std::uint64_t> received_count_{0};
  mutable std::atomic<std::uint64_t> accepted_count_{0};
  mutable std::atomic<std::uint64_t> rejected_count_{0};
  mutable std::atomic<std::uint64_t> stale_count_{0};
  mutable std::atomic<std::uint64_t> out_of_bounds_count_{0};
  mutable std::atomic<std::uint64_t> out_of_horizon_count_{0};
  // Geometry summaries are diagnostic only.  Limit the full-grid scan in the
  // subscription callback so diagnostics cannot consume the 10 Hz control
  // budget.  The latest summary is copied to snapshots between scans.
  mutable std::atomic<std::int64_t> last_summary_stamp_ns_{-1};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_PREDICTION_CRITIC__PREDICTION_CRITIC_HPP_
