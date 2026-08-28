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
  getParam(ignore_out_of_bounds_, "ignore_out_of_bounds", true);
  getParam(ignore_out_of_horizon_, "ignore_out_of_horizon", true);

  if (weight_ < 0.0F || power_ == 0U) {
    throw std::runtime_error("PredictionCritic cost_weight must be >= 0 and cost_power > 0");
  }
  if (stale_threshold_s_ < 0.0 || clock_skew_tolerance_s_ < 0.0) {
    throw std::runtime_error("PredictionCritic stale and clock tolerances must be >= 0");
  }
  if (interpolation_ != "linear" && interpolation_ != "nearest") {
    throw std::runtime_error(
            "PredictionCritic temporal_interpolation must be 'linear' or 'nearest'");
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
    "stale_threshold=%.3fs interpolation=%s out_of_bounds=%s out_of_horizon=%s",
    topic_.c_str(), expected_frame_.c_str(), weight_, power_, stale_threshold_s_,
    interpolation_.c_str(), ignore_out_of_bounds_ ? "ignore" : "penalize",
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
    logStatus("no_message", std::numeric_limits<double>::infinity(), 0.0F, 0.0F, 0, 0);
    return;
  }

  const auto pose_stamp = rclcpp::Time(data.state.pose.header.stamp, RCL_ROS_TIME);
  const auto eval_stamp = pose_stamp.nanoseconds() == 0 ? clock_->now() : pose_stamp;
  const double age_s = (eval_stamp - grid->stamp).seconds();
  if (!std::isfinite(age_s) || age_s > stale_threshold_s_ ||
    age_s < -clock_skew_tolerance_s_)
  {
    stale_count_.fetch_add(1, std::memory_order_relaxed);
    logStatus("stale", age_s, 0.0F, 0.0F, 0, 0);
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

  const auto batch_size = data.costs.shape(0);
  const auto trajectory_steps = data.trajectories.x.shape(1);
  for (std::size_t trajectory = 0; trajectory < batch_size; ++trajectory) {
    float risk_sum = 0.0F;
    for (std::size_t k = 0; k < trajectory_steps; ++k) {
      float risk = 0.0F;
      const double tau_s = effective_age_s + static_cast<double>(k) * data.model_dt;
      const auto sample = sampler_.sample(
        grid_view, data.trajectories.x(trajectory, k), data.trajectories.y(trajectory, k), tau_s);
      if (sample.out_of_bounds) {
        ++out_of_bounds;
      }
      if (sample.out_of_horizon) {
        ++out_of_horizon;
      }
      if (sample.valid) {
        risk = sample.risk;
        risk_sum += risk;
        global_max_risk = std::max(global_max_risk, risk);
        ++valid_samples;
      }
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
    "active", age_s, global_max_risk, global_max_cost, valid_samples, out_of_horizon);
}

void PredictionCritic::logStatus(
  const char * status, double age_s, float max_risk, float max_cost,
  std::size_t valid_samples, std::size_t out_of_horizon) const
{
  RCLCPP_INFO_THROTTLE(
    logger_, *clock_, 5000,
    "PredictionCritic status=%s age_s=%.4f max_risk=%.3f max_cost=%.3f "
    "valid_samples=%zu out_of_horizon=%zu accepted=%llu rejected=%llu stale=%llu",
    status, age_s, max_risk, max_cost, valid_samples, out_of_horizon,
    static_cast<unsigned long long>(accepted_count_.load(std::memory_order_relaxed)),
    static_cast<unsigned long long>(rejected_count_.load(std::memory_order_relaxed)),
    static_cast<unsigned long long>(stale_count_.load(std::memory_order_relaxed)));
}

}  // namespace mppi::critics

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PredictionCritic,
  mppi::critics::CriticFunction)
