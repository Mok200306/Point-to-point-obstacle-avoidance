#include "nav2_mppi_prediction_critic/prediction_critic.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>

#include "pluginlib/class_list_macros.hpp"

namespace mppi::critics
{

void PredictionCritic::initialize()
{
  auto node = parent_.lock();
  if (!node) {
    throw std::runtime_error("PredictionCritic parent node is unavailable");
  }
  clock_ = node->get_clock();

  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(weight_, "cost_weight", 50.0F);
  getParam(power_, "cost_power", 1);
  getParam(topic_, "oracle_topic", std::string("/oracle/predicted_occupancy"));
  getParam(
    expected_frame_, "expected_frame",
    costmap_ros_ ? costmap_ros_->getGlobalFrameID() : std::string("odom"));
  getParam(interpolation_, "temporal_interpolation", std::string("linear"));
  getParam(stale_threshold_s_, "stale_threshold_s", 0.30);
  getParam(clock_skew_tolerance_s_, "clock_skew_tolerance_s", 0.05);
  getParam(use_footprint_, "use_footprint", true);
  int footprint_edge_samples = 2;
  getParam(footprint_edge_samples, "footprint_edge_samples", 2);
  getParam(ignore_out_of_bounds_, "ignore_out_of_bounds", true);
  getParam(ignore_out_of_horizon_, "ignore_out_of_horizon", true);

  if (weight_ < 0.0F || power_ == 0U) {
    throw std::runtime_error("PredictionCritic cost_weight must be >= 0 and cost_power > 0");
  }
  if (stale_threshold_s_ < 0.0 || clock_skew_tolerance_s_ < 0.0) {
    throw std::runtime_error("PredictionCritic stale and clock tolerances must be >= 0");
  }
  if (footprint_edge_samples < 0) {
    throw std::runtime_error("PredictionCritic footprint_edge_samples must be >= 0");
  }
  footprint_edge_samples_ = static_cast<unsigned int>(footprint_edge_samples);
  if (interpolation_ != "linear" && interpolation_ != "nearest") {
    throw std::runtime_error(
            "PredictionCritic temporal_interpolation must be 'linear' or 'nearest'");
  }

  if (footprint_edge_samples_ > 16U) {
    throw std::runtime_error("PredictionCritic footprint_edge_samples must be <= 16");
  }
  footprint_samples_.clear();
  if (use_footprint_ && costmap_ros_) {
    footprint_samples_ = makeFootprintSamples(
      costmap_ros_->getRobotFootprint(), footprint_edge_samples_);
  }
  if (footprint_samples_.empty()) {
    footprint_samples_.push_back({0.0F, 0.0F});
  }

  sampler_ = PredictionGridSampler(
    interpolation_, clock_skew_tolerance_s_, ignore_out_of_bounds_, ignore_out_of_horizon_);

  if (!enabled_) {
    RCLCPP_INFO(logger_, "PredictionCritic disabled; no Oracle subscription created");
    return;
  }

  subscription_ = node->create_subscription<Message>(
    topic_, rclcpp::QoS(10),
    std::bind(&PredictionCritic::messageCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    logger_,
    "PredictionCritic enabled: topic=%s expected_frame=%s weight=%.3f power=%u "
    "stale_threshold=%.3fs interpolation=%s footprint=%s footprint_samples=%zu "
    "out_of_bounds=%s out_of_horizon=%s",
    topic_.c_str(), expected_frame_.c_str(), weight_, power_, stale_threshold_s_,
    interpolation_.c_str(), use_footprint_ ? "enabled" : "center_only",
    footprint_samples_.size(), ignore_out_of_bounds_ ? "ignore" : "penalize",
    ignore_out_of_horizon_ ? "ignore" : "last_layer");
}

void PredictionCritic::messageCallback(const Message::SharedPtr message)
{
  received_count_.fetch_add(1, std::memory_order_relaxed);
  if (!message) {
    rejected_count_.fetch_add(1, std::memory_order_relaxed);
    return;
  }

  const auto frame_id = message->header.frame_id;
  const auto cell_count = static_cast<std::size_t>(message->width) * message->height;
  const auto expected_data_size = cell_count * message->steps;
  const auto origin_q = message->origin.orientation;
  const double origin_yaw = std::atan2(
    2.0 * (origin_q.w * origin_q.z + origin_q.x * origin_q.y),
    1.0 - 2.0 * (origin_q.y * origin_q.y + origin_q.z * origin_q.z));

  if (frame_id != expected_frame_ || message->resolution <= 0.0F ||
    message->width == 0U || message->height == 0U || message->steps == 0U ||
    message->dt <= 0.0F || message->data.size() != expected_data_size ||
    !std::isfinite(origin_yaw) || std::abs(origin_yaw) > 1.0e-3)
  {
    rejected_count_.fetch_add(1, std::memory_order_relaxed);
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000,
      "PredictionCritic rejected Oracle message: frame=%s expected=%s "
      "grid=%ux%u steps=%u dt=%.3f data=%zu expected=%zu origin_yaw=%.6f",
      frame_id.c_str(), expected_frame_.c_str(), message->width, message->height,
      message->steps, message->dt, message->data.size(), expected_data_size, origin_yaw);
    return;
  }

  auto next = std::make_shared<GridSnapshot>();
  next->stamp = rclcpp::Time(message->header.stamp, RCL_ROS_TIME);
  next->frame_id = frame_id;
  next->resolution = message->resolution;
  next->width = message->width;
  next->height = message->height;
  next->origin_x = static_cast<float>(message->origin.position.x);
  next->origin_y = static_cast<float>(message->origin.position.y);
  next->dt = message->dt;
  next->steps = message->steps;
  next->data = message->data;

  // Keep a compact summary of the received Oracle raster for throttled runtime
  // diagnostics.  This does not participate in scoring.  A full message may
  // contain hundreds of thousands of cells; scanning it on every 10 Hz
  // callback needlessly competes with MPPI's control loop.  Scan at most once
  // per five seconds and carry the previous summary between scans.
  constexpr std::int64_t summary_period_ns = 5'000'000'000LL;
  const auto message_stamp_ns =
    static_cast<std::int64_t>(message->header.stamp.sec) * 1'000'000'000LL +
    static_cast<std::int64_t>(message->header.stamp.nanosec);
  auto previous_summary_stamp_ns = last_summary_stamp_ns_.load(
    std::memory_order_relaxed);
  bool summarize_message = previous_summary_stamp_ns < 0;
  if (!summarize_message && message_stamp_ns > previous_summary_stamp_ns) {
    summarize_message = message_stamp_ns - previous_summary_stamp_ns >= summary_period_ns;
  }
  if (summarize_message &&
    !last_summary_stamp_ns_.compare_exchange_strong(
      previous_summary_stamp_ns, message_stamp_ns, std::memory_order_relaxed))
  {
    summarize_message = false;
  }

  if (summarize_message) {
    const auto layer_size = cell_count;
    std::vector<unsigned char> occupied_flags(cell_count, 0U);
    for (std::size_t layer = 0; layer < next->steps; ++layer) {
      const auto layer_offset = layer * layer_size;
      for (std::size_t cell = 0; cell < layer_size; ++cell) {
        const auto raw_risk = next->data[layer_offset + cell];
        if (!std::isfinite(raw_risk)) {
          continue;
        }
        const auto risk = std::clamp(raw_risk, 0.0F, 1.0F);
        next->message_max_risk = std::max(next->message_max_risk, risk);
        if (risk <= 1.0e-4F) {
          continue;
        }
        ++next->occupied_values;
        if (occupied_flags[cell] != 0U) {
          continue;
        }
        occupied_flags[cell] = 1U;
        ++next->occupied_cells;
        const auto column = cell % static_cast<std::size_t>(next->width);
        const auto row = cell / static_cast<std::size_t>(next->width);
        const auto x = next->origin_x +
          (static_cast<float>(column) + 0.5F) * next->resolution;
        const auto y = next->origin_y +
          (static_cast<float>(row) + 0.5F) * next->resolution;
        if (!next->has_occupied_extent) {
          next->has_occupied_extent = true;
          next->occupied_min_x = x;
          next->occupied_max_x = x;
          next->occupied_min_y = y;
          next->occupied_max_y = y;
        } else {
          next->occupied_min_x = std::min(next->occupied_min_x, x);
          next->occupied_max_x = std::max(next->occupied_max_x, x);
          next->occupied_min_y = std::min(next->occupied_min_y, y);
          next->occupied_max_y = std::max(next->occupied_max_y, y);
        }
      }
    }
  } else {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    if (snapshot_) {
      next->occupied_cells = snapshot_->occupied_cells;
      next->occupied_values = snapshot_->occupied_values;
      next->message_max_risk = snapshot_->message_max_risk;
      next->has_occupied_extent = snapshot_->has_occupied_extent;
      next->occupied_min_x = snapshot_->occupied_min_x;
      next->occupied_max_x = snapshot_->occupied_max_x;
      next->occupied_min_y = snapshot_->occupied_min_y;
      next->occupied_max_y = snapshot_->occupied_max_y;
    }
  }

  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    snapshot_ = std::move(next);
  }
  accepted_count_.fetch_add(1, std::memory_order_relaxed);
}

std::shared_ptr<const PredictionCritic::GridSnapshot> PredictionCritic::latestSnapshot() const
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  return snapshot_;
}

void PredictionCritic::score(CriticData & data)
{
  if (!enabled_ || !enabled_for_prediction_ || !clock_) {
    return;
  }

  const auto grid = latestSnapshot();
  if (!grid) {
    stale_count_.fetch_add(1, std::memory_order_relaxed);
    logStatus(
      "no_message", std::numeric_limits<double>::infinity(), 0.0F, 0.0F,
      0, 0, 0, 0);
    return;
  }

  const auto pose_stamp = rclcpp::Time(data.state.pose.header.stamp, RCL_ROS_TIME);
  const auto eval_stamp = pose_stamp.nanoseconds() == 0 ? clock_->now() : pose_stamp;
  const double age_s = (eval_stamp - grid->stamp).seconds();
  if (!std::isfinite(age_s) || age_s > stale_threshold_s_ ||
    age_s < -clock_skew_tolerance_s_)
  {
    stale_count_.fetch_add(1, std::memory_order_relaxed);
    logStatus("stale", age_s, 0.0F, 0.0F, 0, 0, 0, 0);
    return;
  }

  const double effective_age_s = std::max(0.0, age_s);
  PredictionGridView grid_view;
  grid_view.resolution = grid->resolution;
  grid_view.width = grid->width;
  grid_view.height = grid->height;
  grid_view.origin_x = grid->origin_x;
  grid_view.origin_y = grid->origin_y;
  grid_view.dt = grid->dt;
  grid_view.steps = grid->steps;
  grid_view.data = &grid->data;
  float global_max_risk = 0.0F;
  float global_max_cost = 0.0F;
  std::size_t valid_samples = 0;
  std::size_t out_of_horizon = 0;
  std::size_t out_of_bounds = 0;
  std::size_t risk_hits = 0;
  double risk_hit_min_tau_s = std::numeric_limits<double>::infinity();
  double risk_hit_max_tau_s = -std::numeric_limits<double>::infinity();
  float candidate_min_x = std::numeric_limits<float>::infinity();
  float candidate_max_x = -std::numeric_limits<float>::infinity();
  float candidate_min_y = std::numeric_limits<float>::infinity();
  float candidate_max_y = -std::numeric_limits<float>::infinity();

  const auto batch_size = data.costs.shape(0);
  const auto trajectory_steps = data.trajectories.x.shape(1);
  for (std::size_t trajectory = 0; trajectory < batch_size; ++trajectory) {
    float risk_sum = 0.0F;
    for (std::size_t k = 0; k < trajectory_steps; ++k) {
      float pose_max_risk = 0.0F;
      const auto pose_x = data.trajectories.x(trajectory, k);
      const auto pose_y = data.trajectories.y(trajectory, k);
      const auto pose_yaw = data.trajectories.yaws(trajectory, k);
      candidate_min_x = std::min(candidate_min_x, pose_x);
      candidate_max_x = std::max(candidate_max_x, pose_x);
      candidate_min_y = std::min(candidate_min_y, pose_y);
      candidate_max_y = std::max(candidate_max_y, pose_y);
      const double tau_s = effective_age_s + static_cast<double>(k) * data.model_dt;
      for (const auto & local_sample : footprint_samples_) {
        const auto sample_pose = transformFootprintSample(
          pose_x, pose_y, pose_yaw, local_sample);
        const auto sample = sampler_.sample(
          grid_view, sample_pose.x, sample_pose.y, tau_s);
        if (sample.out_of_bounds) {
          ++out_of_bounds;
        }
        if (sample.out_of_horizon) {
          ++out_of_horizon;
        }
        if (sample.valid) {
          pose_max_risk = std::max(pose_max_risk, sample.risk);
          global_max_risk = std::max(global_max_risk, sample.risk);
          ++valid_samples;
          if (sample.risk > 1.0e-4F) {
            ++risk_hits;
            risk_hit_min_tau_s = std::min(risk_hit_min_tau_s, tau_s);
            risk_hit_max_tau_s = std::max(risk_hit_max_tau_s, tau_s);
          }
        }
      }
      risk_sum += pose_max_risk;
    }

    const float critic_cost = std::pow(weight_ * risk_sum, static_cast<float>(power_));
    data.costs(trajectory) += critic_cost;
    global_max_cost = std::max(global_max_cost, critic_cost);
  }

  if (out_of_bounds > 0U) {
    out_of_bounds_count_.fetch_add(out_of_bounds, std::memory_order_relaxed);
  }
  if (out_of_horizon > 0U) {
    out_of_horizon_count_.fetch_add(out_of_horizon, std::memory_order_relaxed);
  }
  logStatus(
    "active", age_s, global_max_risk, global_max_cost, valid_samples,
    out_of_horizon, out_of_bounds, risk_hits);
  logGeometry(
    *grid, data, batch_size, trajectory_steps, candidate_min_x, candidate_max_x,
    candidate_min_y, candidate_max_y, valid_samples, risk_hits, out_of_bounds,
    out_of_horizon, risk_hit_min_tau_s, risk_hit_max_tau_s);
}

void PredictionCritic::logStatus(
  const char * status, double age_s, float max_risk, float max_cost,
  std::size_t valid_samples, std::size_t out_of_horizon,
  std::size_t out_of_bounds, std::size_t risk_hits) const
{
  RCLCPP_INFO_THROTTLE(
    logger_, *clock_, 5000,
    "PredictionCritic status=%s age_s=%.4f max_risk=%.3f max_cost=%.3f "
    "valid_samples=%zu risk_hits=%zu out_of_bounds=%zu out_of_horizon=%zu "
    "accepted=%llu rejected=%llu stale=%llu",
    status, age_s, max_risk, max_cost, valid_samples, risk_hits, out_of_bounds,
    out_of_horizon,
    static_cast<unsigned long long>(accepted_count_.load(std::memory_order_relaxed)),
    static_cast<unsigned long long>(rejected_count_.load(std::memory_order_relaxed)),
    static_cast<unsigned long long>(stale_count_.load(std::memory_order_relaxed)));
}

void PredictionCritic::logGeometry(
  const GridSnapshot & grid, const CriticData & data,
  std::size_t batch_size, std::size_t trajectory_steps,
  float candidate_min_x, float candidate_max_x,
  float candidate_min_y, float candidate_max_y,
  std::size_t valid_samples, std::size_t risk_hits,
  std::size_t out_of_bounds, std::size_t out_of_horizon,
  double risk_hit_min_tau_s, double risk_hit_max_tau_s) const
{
  const auto grid_max_x = grid.origin_x +
    static_cast<float>(grid.width) * grid.resolution;
  const auto grid_max_y = grid.origin_y +
    static_cast<float>(grid.height) * grid.resolution;
  const auto & pose = data.state.pose;
  const auto pose_stamp_s = pose.header.stamp.sec +
    static_cast<double>(pose.header.stamp.nanosec) / 1.0e9;
  const auto grid_stamp_s = grid.stamp.nanoseconds() / 1.0e9;
  const auto pose_frame = pose.header.frame_id.empty() ? "<empty>" : pose.header.frame_id.c_str();
  const auto candidate_bounds_valid =
    std::isfinite(candidate_min_x) && std::isfinite(candidate_max_x) &&
    std::isfinite(candidate_min_y) && std::isfinite(candidate_max_y);
  const auto occupied_bounds_valid = grid.has_occupied_extent;

  RCLCPP_INFO_THROTTLE(
    logger_, *clock_, 5000,
    "PredictionCritic geometry pose_frame=%s pose_stamp_s=%.3f grid_stamp_s=%.3f "
    "pose=(%.3f,%.3f) "
    "grid_frame=%s grid_origin=(%.3f,%.3f) grid_bounds=[%.3f,%.3f]x[%.3f,%.3f] "
    "grid_occ_cells=%zu grid_occ_values=%zu grid_max_risk=%.3f "
    "occ_bounds_valid=%s occ_bounds=[%.3f,%.3f]x[%.3f,%.3f] "
    "candidates=%zux%zu candidate_bounds_valid=%s candidate_bounds=[%.3f,%.3f]x[%.3f,%.3f] "
    "valid_samples=%zu risk_hits=%zu risk_hit_tau=[%.3f,%.3f] "
    "out_of_bounds=%zu out_of_horizon=%zu model_dt=%.3f",
    pose_frame, pose_stamp_s, grid_stamp_s, pose.pose.position.x, pose.pose.position.y,
    grid.frame_id.c_str(),
    grid.origin_x, grid.origin_y, grid.origin_x, grid_max_x, grid.origin_y, grid_max_y,
    grid.occupied_cells, grid.occupied_values, grid.message_max_risk,
    occupied_bounds_valid ? "true" : "false",
    grid.occupied_min_x, grid.occupied_max_x, grid.occupied_min_y, grid.occupied_max_y,
    batch_size, trajectory_steps, candidate_bounds_valid ? "true" : "false",
    candidate_min_x, candidate_max_x, candidate_min_y, candidate_max_y,
    valid_samples, risk_hits, risk_hit_min_tau_s, risk_hit_max_tau_s, out_of_bounds,
    out_of_horizon, data.model_dt);
}

}  // namespace mppi::critics

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PredictionCritic,
  mppi::critics::CriticFunction)
