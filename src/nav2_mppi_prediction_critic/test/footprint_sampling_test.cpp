#include <algorithm>
#include <cmath>
#include <vector>

#include "gtest/gtest.h"
#include "geometry_msgs/msg/point.hpp"
#include "nav2_mppi_prediction_critic/footprint_sampling.hpp"

namespace
{

geometry_msgs::msg::Point point(double x, double y)
{
  geometry_msgs::msg::Point result;
  result.x = x;
  result.y = y;
  return result;
}

TEST(FootprintSampling, IncludesCenterVerticesAndEdges)
{
  const std::vector<geometry_msgs::msg::Point> rectangle{
    point(0.30, 0.24), point(0.30, -0.24),
    point(-0.30, -0.24), point(-0.30, 0.24)};
  const auto samples = mppi::critics::makeFootprintSamples(rectangle, 2U);

  // center + four vertices + two points on each of four edges
  ASSERT_EQ(samples.size(), 13U);
  EXPECT_FLOAT_EQ(samples.front().x, 0.0F);
  EXPECT_FLOAT_EQ(samples.front().y, 0.0F);
  EXPECT_TRUE(std::any_of(
    samples.begin(), samples.end(), [](const auto & sample) {
      return std::abs(sample.x - 0.30F) < 1.0e-6F &&
             std::abs(sample.y - 0.24F) < 1.0e-6F;
    }));
}

TEST(FootprintSampling, RotatesLocalPointIntoWorld)
{
  constexpr float quarter_turn = 1.5707963267948966F;
  const auto world = mppi::critics::transformFootprintSample(
    1.0F, 2.0F, quarter_turn, {0.30F, 0.0F});
  EXPECT_NEAR(world.x, 1.0F, 1.0e-5F);
  EXPECT_NEAR(world.y, 2.30F, 1.0e-5F);
}

TEST(FootprintSampling, EmptyFootprintFallsBackToCenter)
{
  const auto samples = mppi::critics::makeFootprintSamples(
    std::vector<geometry_msgs::msg::Point>{}, 2U);
  ASSERT_EQ(samples.size(), 1U);
  EXPECT_FLOAT_EQ(samples.front().x, 0.0F);
  EXPECT_FLOAT_EQ(samples.front().y, 0.0F);
}

}  // namespace
