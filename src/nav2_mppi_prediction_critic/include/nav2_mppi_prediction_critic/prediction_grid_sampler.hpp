#ifndef NAV2_MPPI_PREDICTION_CRITIC__PREDICTION_GRID_SAMPLER_HPP_
#define NAV2_MPPI_PREDICTION_CRITIC__PREDICTION_GRID_SAMPLER_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace mppi::critics
{

struct PredictionGridView
{
  float resolution{0.0F};
  std::uint32_t width{0};
  std::uint32_t height{0};
  float origin_x{0.0F};
  float origin_y{0.0F};
  float dt{0.0F};
  std::uint32_t steps{0};
  const std::vector<float> * data{nullptr};
};

struct PredictionSample
{
  bool valid{false};
  bool out_of_bounds{false};
  bool out_of_horizon{false};
  float risk{0.0F};
};

/**
 * @brief Pure sampler for a time-indexed occupancy grid.
 *
 * This class deliberately has no ROS dependency.  It is shared by the
 * runtime critic and the Gate 4 unit test so that the tested time/space
 * indexing is exactly the indexing used in MPPI scoring.
 */
class PredictionGridSampler
{
public:
  PredictionGridSampler() = default;

  PredictionGridSampler(
    std::string temporal_interpolation, double clock_skew_tolerance_s,
    bool ignore_out_of_bounds, bool ignore_out_of_horizon)
  : temporal_interpolation_(std::move(temporal_interpolation)),
    clock_skew_tolerance_s_(clock_skew_tolerance_s),
    ignore_out_of_bounds_(ignore_out_of_bounds),
    ignore_out_of_horizon_(ignore_out_of_horizon)
  {
  }

  PredictionSample sample(
    const PredictionGridView & grid, float x, float y, double tau_s) const
  {
    PredictionSample result;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(tau_s) ||
      grid.resolution <= 0.0F || grid.width == 0U || grid.height == 0U ||
      grid.steps == 0U || grid.dt <= 0.0F || grid.data == nullptr)
    {
      result.out_of_bounds = true;
      return result;
    }

    const auto expected_size = static_cast<std::size_t>(grid.width) * grid.height * grid.steps;
    if (grid.data->size() != expected_size) {
      result.out_of_bounds = true;
      return result;
    }

    const double max_tau = static_cast<double>(grid.steps - 1U) * grid.dt;
    if (tau_s < -clock_skew_tolerance_s_ || tau_s > max_tau + clock_skew_tolerance_s_) {
      result.out_of_horizon = true;
      if (ignore_out_of_horizon_) {
        return result;
      }
      tau_s = std::max(0.0, std::min(tau_s, max_tau));
    } else {
      tau_s = std::max(0.0, std::min(tau_s, max_tau));
    }

    const auto column = static_cast<int>(std::floor(
      (static_cast<double>(x) - grid.origin_x) / grid.resolution));
    const auto row = static_cast<int>(std::floor(
      (static_cast<double>(y) - grid.origin_y) / grid.resolution));
    if (column < 0 || row < 0 || column >= static_cast<int>(grid.width) ||
      row >= static_cast<int>(grid.height))
    {
      result.out_of_bounds = true;
      if (ignore_out_of_bounds_) {
        return result;
      }
      result.valid = true;
      result.risk = 1.0F;
      return result;
    }

    const auto cell = static_cast<std::size_t>(row) * grid.width + column;
    const double layer_position = tau_s / grid.dt;
    std::size_t lower = 0U;
    std::size_t upper = 0U;
    double alpha = 0.0;
    if (temporal_interpolation_ == "nearest") {
      lower = static_cast<std::size_t>(std::llround(layer_position));
      lower = std::min(lower, static_cast<std::size_t>(grid.steps - 1U));
      upper = lower;
    } else {
      lower = static_cast<std::size_t>(std::floor(layer_position));
      lower = std::min(lower, static_cast<std::size_t>(grid.steps - 1U));
      upper = std::min(lower + 1U, static_cast<std::size_t>(grid.steps - 1U));
      alpha = layer_position - static_cast<double>(lower);
    }

    const auto layer_size = static_cast<std::size_t>(grid.width) * grid.height;
    const auto lower_index = lower * layer_size + cell;
    const auto upper_index = upper * layer_size + cell;
    if (lower_index >= grid.data->size() || upper_index >= grid.data->size()) {
      result.out_of_bounds = true;
      return result;
    }

    const float lower_risk = std::clamp((*grid.data)[lower_index], 0.0F, 1.0F);
    const float upper_risk = std::clamp((*grid.data)[upper_index], 0.0F, 1.0F);
    result.valid = true;
    result.risk = static_cast<float>((1.0 - alpha) * lower_risk + alpha * upper_risk);
    return result;
  }

private:
  std::string temporal_interpolation_{"linear"};
  double clock_skew_tolerance_s_{0.05};
  bool ignore_out_of_bounds_{true};
  bool ignore_out_of_horizon_{true};
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_PREDICTION_CRITIC__PREDICTION_GRID_SAMPLER_HPP_
