#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "nav2_mppi_prediction_critic/prediction_grid_sampler.hpp"

namespace
{

using mppi::critics::PredictionGridSampler;
using mppi::critics::PredictionGridView;

struct Row
{
  std::string test;
  double tau_s;
  float risk;
  bool valid;
  bool out_of_bounds;
  bool out_of_horizon;
  double cost;
  bool passed;
  std::string note;
};

std::size_t index(
  unsigned int layer, unsigned int row, unsigned int column,
  unsigned int width, unsigned int height)
{
  return static_cast<std::size_t>(layer) * width * height + row * width + column;
}

bool finite(float value)
{
  return std::isfinite(static_cast<double>(value));
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string output = "experiments/oracle_mppi/gate4/critic_debug.csv";
  if (argc == 3 && std::string(argv[1]) == "--output") {
    output = argv[2];
  } else if (argc != 1) {
    std::cerr << "Usage: prediction_critic_offline_test [--output PATH]\n";
    return 2;
  }

  constexpr unsigned int width = 4U;
  constexpr unsigned int height = 4U;
  constexpr unsigned int steps = 4U;
  constexpr float dt = 1.0F;
  constexpr float weight = 50.0F;
  std::vector<float> data(width * height * steps, 0.0F);
  data[index(1U, 2U, 1U, width, height)] = 1.0F;
  const PredictionGridView grid{
    1.0F, width, height, 0.0F, 0.0F, dt, steps, &data};
  const PredictionGridSampler sampler("linear", 0.05, true, true);

  std::vector<Row> rows;
  const auto add = [&rows](
    const std::string & test, double tau_s, const auto & sample,
    double cost, bool passed, const std::string & note) {
      rows.push_back({
        test, tau_s, sample.risk, sample.valid, sample.out_of_bounds,
        sample.out_of_horizon, cost, passed, note});
    };

  // T1: no future occupancy produces zero critic cost.
  std::fill(data.begin(), data.end(), 0.0F);
  auto t1 = sampler.sample(grid, 1.2F, 2.2F, 1.0);
  add("T1_zero_risk", 1.0, t1, weight * t1.risk, t1.valid && t1.risk == 0.0F,
      "all Oracle layers zero");

  // Restore a single conflict layer for T2-T5.
  data[index(1U, 2U, 1U, width, height)] = 1.0F;
  auto t2_before = sampler.sample(grid, 1.2F, 2.2F, 0.2);
  auto t2_conflict = sampler.sample(grid, 1.2F, 2.2F, 1.0);
  auto t2_interpolated = sampler.sample(grid, 1.2F, 2.2F, 0.5);
  add("T2_same_space_before", 0.2, t2_before, weight * t2_before.risk,
      t2_before.valid && t2_before.risk < t2_conflict.risk,
      "same x,y before conflict time");
  add("T2_same_space_conflict", 1.0, t2_conflict, weight * t2_conflict.risk,
      t2_conflict.valid && t2_conflict.risk == 1.0F,
      "same x,y at conflict time");
  add("T2_linear_interpolation", 0.5, t2_interpolated,
      weight * t2_interpolated.risk,
      t2_interpolated.valid && std::abs(t2_interpolated.risk - 0.5F) < 1.0e-6F,
      "linear risk between layers");

  // T3: tau_k = age + k * model_dt.
  constexpr double age_s = 0.2;
  constexpr std::size_t k = 4U;
  constexpr double model_dt = 0.2;
  const double tau_k = age_s + static_cast<double>(k) * model_dt;
  auto t3 = sampler.sample(grid, 1.2F, 2.2F, tau_k);
  add("T3_message_age_alignment", tau_k, t3, weight * t3.risk,
      t3.valid && t3.risk == 1.0F,
      "tau_k=(t_eval-t_msg)+k*model_dt");

  // T4: a candidate point beyond the future horizon falls back without cost.
  auto t4 = sampler.sample(grid, 1.2F, 2.2F, 4.0);
  add("T4_out_of_horizon", 4.0, t4, 0.0,
      !t4.valid && t4.out_of_horizon && finite(t4.risk),
      "ignore_out_of_horizon=true");

  // T5: a point outside the local grid falls back without NaN/exception.
  auto t5 = sampler.sample(
    grid, std::numeric_limits<float>::quiet_NaN(), 2.0F, 0.0);
  add("T5_bounds_finite", 0.0, t5, 0.0,
      !t5.valid && t5.out_of_bounds && finite(t5.risk),
      "non-finite sample rejected safely");

  // Required Gate 4 behavioral proof: the conflict candidate has higher cost
  // than the identical-space candidate evaluated before the conflict time.
  const bool separation = t2_conflict.valid && t2_before.valid &&
    (weight * t2_conflict.risk > weight * t2_before.risk);
  rows.push_back({
    "cost_separation", 1.0, t2_conflict.risk, t2_conflict.valid,
    t2_conflict.out_of_bounds, t2_conflict.out_of_horizon,
    weight * t2_conflict.risk, separation,
    "conflict trajectory cost > pre-conflict trajectory cost"});

  std::ofstream stream(output);
  if (!stream) {
    std::cerr << "Cannot open output: " << output << "\n";
    return 3;
  }
  stream << "test,tau_s,risk,valid,out_of_bounds,out_of_horizon,cost,passed,note\n";
  stream << std::setprecision(9);
  bool all_pass = true;
  for (const auto & row : rows) {
    stream << row.test << ',' << row.tau_s << ',' << row.risk << ','
           << (row.valid ? "true" : "false") << ','
           << (row.out_of_bounds ? "true" : "false") << ','
           << (row.out_of_horizon ? "true" : "false") << ','
           << row.cost << ',' << (row.passed ? "true" : "false") << ','
           << row.note << '\n';
    all_pass = all_pass && row.passed;
  }
  std::cout << "wrote " << rows.size() << " rows to " << output << "\n";
  std::cout << "all_pass=" << (all_pass ? "true" : "false") << "\n";
  return all_pass ? 0 : 1;
}
