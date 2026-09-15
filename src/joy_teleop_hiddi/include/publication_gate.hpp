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

#ifndef JOY_TELEOP_PUBLICATION_GATE_HPP_
#define JOY_TELEOP_PUBLICATION_GATE_HPP_

namespace joy_teleop
{

enum class PendingPublication
{
  none,
  neutral,
  fresh
};

/**
 * Orders safety-neutral publications relative to subsequently queued samples.
 *
 * The HID callback and ROS timer exchange only the latest fresh report, but a
 * startup neutral is an authority-revocation event and must never be coalesced
 * away. A runtime input fault is stronger: the gate becomes terminal so a
 * depth-one DDS writer can never replace the final neutral with a later report.
 * Callers provide their own lock around this small state machine.
 */
class PublicationGate
{
public:
  void require_neutral() noexcept
  {
    if (terminal_) {
      return;
    }
    fresh_pending_ = false;
    neutral_pending_ = true;
  }

  void latch_terminal_neutral() noexcept
  {
    if (terminal_) {
      return;
    }
    // Terminal means no later fresh sample may leave this source. Ignoring all
    // later reports keeps neutral as the final KEEP_LAST(1) command state;
    // orderly shutdown may safely publish another neutral.
    fresh_pending_ = false;
    neutral_pending_ = true;
    terminal_ = true;
  }

  void reset(bool publish_neutral) noexcept
  {
    if (terminal_) {
      return;
    }
    fresh_pending_ = false;
    neutral_pending_ = publish_neutral;
  }

  void queue_fresh() noexcept
  {
    if (terminal_) {
      return;
    }
    // A later valid report may be retained, but it cannot cancel or overtake a
    // pending authority-revocation neutral.
    fresh_pending_ = true;
  }

  bool terminal() const noexcept
  {
    return terminal_;
  }

  PendingPublication take_next() noexcept
  {
    if (neutral_pending_) {
      neutral_pending_ = false;
      return PendingPublication::neutral;
    }
    if (fresh_pending_) {
      fresh_pending_ = false;
      return PendingPublication::fresh;
    }
    return PendingPublication::none;
  }

private:
  bool neutral_pending_{false};
  bool fresh_pending_{false};
  bool terminal_{false};
};

}  // namespace joy_teleop

#endif  // JOY_TELEOP_PUBLICATION_GATE_HPP_
