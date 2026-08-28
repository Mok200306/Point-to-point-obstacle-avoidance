#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_mppi_prediction_critic/prediction_grid_sampler.hpp"

namespace
{

mppi::critics::PredictionGridView makeGrid(
  std::vector<float> & data, unsigned int steps = 4U)
{
  data.assign(4U * 4U * steps, 0.0F);
  return {
    1.0F, 4U, 4U, 0.0F, 0.0F, 1.0F, steps, &data};
}

std::size_t index(unsigned int layer, unsigned int row, unsigned int column, unsigned int width = 4U)
{
  return static_cast<std::size_t>(layer) * width * width + row * width + column;
}

}  // namespace

TEST(PredictionGridSampler, T1StaticZeroRisk)
{
  std::vector<float> data;
  const auto grid = makeGrid(data);
  const mppi::critics::PredictionGridSampler sampler("linear", 0.05, true, true);
  const auto sample = sampler.sample(grid, 1.2F, 2.2F, 1.0);
  EXPECT_TRUE(sample.valid);
  EXPECT_FLOAT_EQ(sample.risk, 0.0F);
  EXPECT_FALSE(sample.out_of_bounds);
  EXPECT_FALSE(sample.out_of_horizon);
}

TEST(PredictionGridSampler, T2SameSpaceDifferentTime)
{
  std::vector<float> data;
  const auto grid = makeGrid(data);
  data[index(1U, 2U, 1U)] = 1.0F;
  const mppi::critics::PredictionGridSampler sampler("linear", 0.05, true, true);

  const auto before = sampler.sample(grid, 1.2F, 2.2F, 0.2);
  const auto at_conflict = sampler.sample(grid, 1.2F, 2.2F, 1.0);
  const auto away_from_conflict = sampler.sample(grid, 1.2F, 2.2F, 0.5);
  EXPECT_TRUE(before.valid);
  EXPECT_TRUE(at_conflict.valid);
  EXPECT_TRUE(away_from_conflict.valid);
  EXPECT_NEAR(before.risk, 0.2F, 1.0e-6F);
  EXPECT_FLOAT_EQ(at_conflict.risk, 1.0F);
  EXPECT_NEAR(away_from_conflict.risk, 0.5F, 1.0e-6F);
}

TEST(PredictionGridSampler, T3MessageAgeIsAddedBeforeSampling)
{
  std::vector<float> data;
  const auto grid = makeGrid(data);
  data[index(1U, 2U, 1U)] = 1.0F;
  const mppi::critics::PredictionGridSampler sampler("linear", 0.05, true, true);

  // t_eval - t_msg = 0.2 s and k * model_dt = 0.8 s, so tau_k = 1.0 s.
  const double age_s = 0.2;
  const double model_dt = 0.2;
  const std::size_t k = 4U;
  const auto sample = sampler.sample(
    grid, 1.2F, 2.2F, age_s + static_cast<double>(k) * model_dt);
  EXPECT_TRUE(sample.valid);
  EXPECT_FLOAT_EQ(sample.risk, 1.0F);
}

TEST(PredictionGridSampler, T4StaleHorizonFallsBackWithoutCrash)
{
  std::vector<float> data;
  const auto grid = makeGrid(data, 2U);
  const mppi::critics::PredictionGridSampler sampler("linear", 0.05, true, true);
  const auto sample = sampler.sample(grid, 1.2F, 2.2F, 2.0);
  EXPECT_FALSE(sample.valid);
  EXPECT_TRUE(sample.out_of_horizon);
  EXPECT_FALSE(sample.out_of_bounds);
}

TEST(PredictionGridSampler, T5BoundsAndFiniteOutputs)
{
  std::vector<float> data;
  const auto grid = makeGrid(data);
  const mppi::critics::PredictionGridSampler sampler("linear", 0.05, true, true);
  const auto outside = sampler.sample(grid, 9.0F, 2.0F, 0.0);
  const auto nonfinite = sampler.sample(
    grid, std::numeric_limits<float>::quiet_NaN(), 2.0F, 0.0);
  EXPECT_FALSE(outside.valid);
  EXPECT_TRUE(outside.out_of_bounds);
  EXPECT_FALSE(nonfinite.valid);
  EXPECT_TRUE(nonfinite.out_of_bounds);
  EXPECT_TRUE(std::isfinite(outside.risk));
  EXPECT_TRUE(std::isfinite(nonfinite.risk));
}
