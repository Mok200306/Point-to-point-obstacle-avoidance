#ifndef NAV2_MPPI_PREDICTION_CRITIC__FOOTPRINT_SAMPLING_HPP_
#define NAV2_MPPI_PREDICTION_CRITIC__FOOTPRINT_SAMPLING_HPP_

#include <cmath>
#include <cstddef>
#include <vector>

#include "geometry_msgs/msg/point.hpp"

namespace mppi::critics
{

struct FootprintSample
{
  float x{0.0F};
  float y{0.0F};
};

/**
 * @brief Create a small set of points representing a padded robot footprint.
 *
 * It includes the center, all vertices and uniformly spaced points on each
 * edge.  The set is intentionally bounded because this helper is used in the
 * MPPI critic's hot path.
 */
inline std::vector<FootprintSample> makeFootprintSamples(
  const std::vector<geometry_msgs::msg::Point> & footprint,
  unsigned int edge_samples)
{
  std::vector<FootprintSample> samples;
  samples.reserve(1U + footprint.size() * (edge_samples + 1U));
  samples.push_back({0.0F, 0.0F});
  if (footprint.empty()) {
    return samples;
  }

  for (const auto & point : footprint) {
    if (std::isfinite(point.x) && std::isfinite(point.y)) {
      samples.push_back({static_cast<float>(point.x), static_cast<float>(point.y)});
    }
  }

  const std::size_t count = footprint.size();
  for (std::size_t index = 0; index < count; ++index) {
    const auto & first = footprint[index];
    const auto & second = footprint[(index + 1U) % count];
    for (unsigned int sample = 1U; sample <= edge_samples; ++sample) {
      const double ratio = static_cast<double>(sample) /
        static_cast<double>(edge_samples + 1U);
      const double x = first.x + ratio * (second.x - first.x);
      const double y = first.y + ratio * (second.y - first.y);
      if (std::isfinite(x) && std::isfinite(y)) {
        samples.push_back({static_cast<float>(x), static_cast<float>(y)});
      }
    }
  }
  return samples;
}

inline FootprintSample transformFootprintSample(
  float pose_x, float pose_y, float pose_yaw, const FootprintSample & local)
{
  const float cosine = std::cos(pose_yaw);
  const float sine = std::sin(pose_yaw);
  return {
    pose_x + cosine * local.x - sine * local.y,
    pose_y + sine * local.x + cosine * local.y,
  };
}

}  // namespace mppi::critics

#endif  // NAV2_MPPI_PREDICTION_CRITIC__FOOTPRINT_SAMPLING_HPP_
