/**
 * Copyright (c) 2026, BlackBerry Limited. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "safety_validation.hpp"

#include <limits>
#include <vector>

int main()
{
  using ik_solver::safety::FixedStepStatus;

  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  int failures = 0;
  const auto check = [&failures](bool condition) {
    if (!condition) {
      ++failures;
    }
  };

  check(ik_solver::safety::finite_vector({0.0, 1.0, -1.0}, 3));
  check(!ik_solver::safety::finite_vector({0.0, nan, 1.0}, 3));
  check(!ik_solver::safety::finite_vector({0.0, infinity, 1.0}, 3));
  check(!ik_solver::safety::finite_vector({0.0, 1.0}, 3));

  std::uint64_t epoch = 0U;
  check(ik_solver::safety::exact_session_epoch(1.0, epoch) && epoch == 1U);
  check(ik_solver::safety::exact_session_epoch(
    static_cast<double>(ik_solver::safety::kMaxSessionEpoch), epoch) &&
    epoch == ik_solver::safety::kMaxSessionEpoch);
  check(!ik_solver::safety::exact_session_epoch(0.0, epoch));
  check(!ik_solver::safety::exact_session_epoch(-1.0, epoch));
  check(!ik_solver::safety::exact_session_epoch(1.5, epoch));
  check(!ik_solver::safety::exact_session_epoch(9007199254740992.0, epoch));
  check(!ik_solver::safety::exact_session_epoch(nan, epoch));
  check(!ik_solver::safety::exact_session_epoch(infinity, epoch));

  // Exercise both cross-topic orders. Commands for a future epoch are rejected
  // before synchronization; stale commands are rejected after it.
  ik_solver::safety::SessionEpochGate epoch_gate;
  check(!epoch_gate.command_matches(1U));
  check(epoch_gate.begin_synchronization(1U));
  check(epoch_gate.commit_synchronization(1U));
  check(!epoch_gate.commit_synchronization(1U));
  check(epoch_gate.command_matches(1U));
  check(!epoch_gate.command_matches(2U));
  check(epoch_gate.begin_synchronization(2U));
  check(!epoch_gate.command_matches(1U));
  check(epoch_gate.commit_synchronization(2U));
  check(epoch_gate.command_matches(2U));
  check(!epoch_gate.command_matches(1U));
  check(!epoch_gate.begin_synchronization(2U));
  check(!epoch_gate.begin_synchronization(1U));
  check(epoch_gate.command_matches(2U));

  // A malformed newer synchronization burns its epoch and revokes the active
  // stream. Neither an intermediate nor the former session may reactivate.
  const std::vector<double> corrupt_newer_sync{0.0, nan, 0.0, 0.0, 0.0, 4.0};
  check(ik_solver::safety::begin_synchronization_message(
    corrupt_newer_sync, 5U, epoch_gate, epoch) ==
    ik_solver::safety::SynchronizationStatus::invalid_payload);
  check(epoch == 4U);
  check(!epoch_gate.has_active_epoch());
  check(!epoch_gate.begin_synchronization(3U));
  check(!epoch_gate.commit_synchronization(3U));
  check(!epoch_gate.begin_synchronization(2U));
  check(!epoch_gate.commit_synchronization(2U));
  check(!epoch_gate.has_active_epoch());

  // An unparseable authority envelope cannot revoke a valid session because
  // it does not identify a trustworthy epoch.
  ik_solver::safety::SessionEpochGate envelope_gate;
  check(envelope_gate.begin_synchronization(1U));
  check(envelope_gate.commit_synchronization(1U));
  check(ik_solver::safety::begin_synchronization_message(
    {0.0, 0.0, 0.0, 0.0, nan}, 5U, envelope_gate, epoch) ==
    ik_solver::safety::SynchronizationStatus::malformed);
  check(envelope_gate.command_matches(1U));
  check(ik_solver::safety::begin_synchronization_message(
    {0.0, 0.0, 0.0, 0.0, 0.0, 1.5}, 5U, envelope_gate, epoch) ==
    ik_solver::safety::SynchronizationStatus::invalid_epoch);
  check(envelope_gate.command_matches(1U));

  check(ik_solver::safety::valid_limit_pair(-1.0, 1.0));
  check(!ik_solver::safety::valid_limit_pair(1.0, 1.0));
  check(!ik_solver::safety::valid_limit_pair(nan, 1.0));
  check(ik_solver::safety::within_closed_range(0.0, -1.0, 1.0));
  check(!ik_solver::safety::within_closed_range(2.0, -1.0, 1.0));
  check(!ik_solver::safety::within_closed_range(nan, -1.0, 1.0));

  check(ik_solver::safety::within_step(0.0, 0.1, 0.1));
  check(!ik_solver::safety::within_step(0.0, 0.1001, 0.1));
  check(!ik_solver::safety::within_step(0.0, nan, 0.1));

  const std::array<double, 3> lower{-0.25, -0.25, 0.05};
  const std::array<double, 3> upper{0.25, 0.25, 0.35};
  check(ik_solver::safety::cartesian_position_within_bounds(
    {0.0, -0.25, 0.35}, lower, upper));
  check(!ik_solver::safety::cartesian_position_within_bounds(
    {0.0, -0.251, 0.35}, lower, upper));
  check(!ik_solver::safety::cartesian_position_within_bounds(
    {0.0, nan, 0.20}, lower, upper));
  check(!ik_solver::safety::cartesian_position_within_bounds(
    {0.0, 0.0, 0.20}, {0.25, -0.25, 0.05}, upper));

  check(
    ik_solver::safety::classify_elapsed(0.02, 0.02, 0.25) == FixedStepStatus::ready);
  check(
    ik_solver::safety::classify_elapsed(0.01, 0.02, 0.25) ==
    FixedStepStatus::too_soon);
  check(
    ik_solver::safety::classify_elapsed(0.30, 0.02, 0.25) ==
    FixedStepStatus::stale_or_invalid);
  check(
    ik_solver::safety::classify_elapsed(-0.01, 0.02, 0.25) ==
    FixedStepStatus::stale_or_invalid);
  check(
    ik_solver::safety::classify_elapsed(nan, 0.02, 0.25) ==
    FixedStepStatus::stale_or_invalid);

  return failures == 0 ? 0 : 1;
}
