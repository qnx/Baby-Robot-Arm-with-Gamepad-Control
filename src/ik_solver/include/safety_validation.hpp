/**
 * Copyright (c) 2026, BlackBerry Limited. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef IK_SOLVER_SAFETY_VALIDATION_HPP
#define IK_SOLVER_SAFETY_VALIDATION_HPP

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace ik_solver::safety
{

enum class FixedStepStatus
{
  ready,
  too_soon,
  stale_or_invalid
};

enum class SynchronizationStatus
{
  ready,
  malformed,
  invalid_epoch,
  stale_epoch,
  invalid_payload
};

// Float64MultiArray is retained for wire compatibility. This is the largest
// positive epoch for which adjacent integer values remain exact in binary64.
inline constexpr std::uint64_t kMaxSessionEpoch = 9007199254740991ULL;

inline bool finite(double value)
{
  return std::isfinite(value);
}

inline bool exact_session_epoch(double value, std::uint64_t & epoch)
{
  if (!finite(value) || value < 1.0 || value > static_cast<double>(kMaxSessionEpoch) ||
    std::floor(value) != value)
  {
    return false;
  }
  const auto candidate = static_cast<std::uint64_t>(value);
  if (static_cast<double>(candidate) != value) {
    return false;
  }
  epoch = candidate;
  return true;
}

class SessionEpochGate
{
public:
  // A strictly newer synchronization revokes the previous command stream
  // before the caller validates and commits its accompanying joint state.
  bool begin_synchronization(std::uint64_t candidate)
  {
    if (candidate <= highest_seen_epoch_) {
      return false;
    }
    // Burn the epoch before validating its payload. If a corrupted newer sync
    // revokes epoch N, a delayed N+1 must never reactivate after N+2 fails.
    highest_seen_epoch_ = candidate;
    pending_epoch_ = candidate;
    active_epoch_ = 0U;
    return true;
  }

  bool commit_synchronization(std::uint64_t candidate)
  {
    if (candidate == 0U || candidate != highest_seen_epoch_ || candidate != pending_epoch_) {
      return false;
    }
    active_epoch_ = candidate;
    pending_epoch_ = 0U;
    return true;
  }

  bool command_matches(std::uint64_t candidate) const
  {
    return active_epoch_ != 0U && candidate == active_epoch_;
  }

  bool has_active_epoch() const
  {
    return active_epoch_ != 0U;
  }

  std::uint64_t active_epoch() const
  {
    return active_epoch_;
  }

  std::uint64_t highest_seen_epoch() const
  {
    return highest_seen_epoch_;
  }

private:
  std::uint64_t active_epoch_ = 0U;
  std::uint64_t highest_seen_epoch_ = 0U;
  std::uint64_t pending_epoch_ = 0U;
};

inline SynchronizationStatus begin_synchronization_message(
  const std::vector<double> & values, std::size_t payload_size,
  SessionEpochGate & gate, std::uint64_t & epoch)
{
  if (values.size() != payload_size + 1U) {
    return SynchronizationStatus::malformed;
  }
  if (!exact_session_epoch(values.back(), epoch)) {
    return SynchronizationStatus::invalid_epoch;
  }
  // A well-framed newer authority token revokes the prior stream before its
  // state payload is trusted. Corrupt state must leave the new session
  // inhibited, never let the former session continue producing commands.
  if (!gate.begin_synchronization(epoch)) {
    return SynchronizationStatus::stale_epoch;
  }
  for (std::size_t index = 0U; index < payload_size; ++index) {
    if (!finite(values[index])) {
      return SynchronizationStatus::invalid_payload;
    }
  }
  return SynchronizationStatus::ready;
}

inline bool valid_limit_pair(double lower, double upper)
{
  return finite(lower) && finite(upper) && lower < upper;
}

inline bool finite_vector(const std::vector<double> & values, std::size_t expected_size)
{
  if (values.size() != expected_size) {
    return false;
  }

  for (double value : values) {
    if (!finite(value)) {
      return false;
    }
  }
  return true;
}

inline bool within_closed_range(double value, double lower, double upper)
{
  return finite(value) && valid_limit_pair(lower, upper) && value >= lower && value <= upper;
}

inline bool within_step(double previous, double next, double max_step)
{
  return finite(previous) && finite(next) && finite(max_step) && max_step > 0.0 &&
         std::abs(next - previous) <= max_step;
}

inline bool cartesian_position_within_bounds(
  const std::array<double, 3> & position,
  const std::array<double, 3> & lower,
  const std::array<double, 3> & upper)
{
  for (std::size_t axis = 0U; axis < position.size(); ++axis) {
    if (!within_closed_range(position[axis], lower[axis], upper[axis])) {
      return false;
    }
  }
  return true;
}

inline FixedStepStatus classify_elapsed(double elapsed, double period, double max_interval)
{
  if (!finite(elapsed) || !finite(period) || !finite(max_interval) || period <= 0.0 ||
      max_interval < period || elapsed <= 0.0 || elapsed > max_interval) {
    return FixedStepStatus::stale_or_invalid;
  }
  if (elapsed < period) {
    return FixedStepStatus::too_soon;
  }
  return FixedStepStatus::ready;
}

}  // namespace ik_solver::safety

#endif  // IK_SOLVER_SAFETY_VALIDATION_HPP
