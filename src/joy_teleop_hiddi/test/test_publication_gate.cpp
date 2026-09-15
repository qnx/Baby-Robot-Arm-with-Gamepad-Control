#include "publication_gate.hpp"

#include <cstdio>

namespace
{

int failures = 0;

void expect_event(
  const char * label,
  joy_teleop::PendingPublication expected,
  joy_teleop::PendingPublication actual)
{
  if (expected != actual) {
    std::fprintf(stderr, "%s: publication order mismatch\n", label);
    ++failures;
  }
}

void test_startup_neutral_is_sticky_and_first()
{
  joy_teleop::PublicationGate gate;

  // Reproduce the safety-critical ordering: an old fresh sample is revoked,
  // then a neutral HID frame and a held-dead-man frame arrive before the timer.
  gate.queue_fresh();
  gate.require_neutral();
  gate.queue_fresh();
  gate.queue_fresh();

  expect_event(
    "startup neutral precedes later dead-man sample",
    joy_teleop::PendingPublication::neutral,
    gate.take_next());
  expect_event(
    "latest fresh sample follows revocation",
    joy_teleop::PendingPublication::fresh,
    gate.take_next());
  expect_event(
    "queue is drained",
    joy_teleop::PendingPublication::none,
    gate.take_next());
}

void test_terminal_fault_prevents_fresh_after_neutral()
{
  joy_teleop::PublicationGate gate;

  gate.queue_fresh();
  gate.latch_terminal_neutral();

  // Model reports received before and after the timer publishes the final
  // neutral. Neither may become a later KEEP_LAST(1) writer sample.
  gate.queue_fresh();
  expect_event(
    "terminal fault publishes neutral",
    joy_teleop::PendingPublication::neutral,
    gate.take_next());
  gate.queue_fresh();
  gate.require_neutral();
  gate.reset(false);

  expect_event(
    "terminal neutral has no fresh successor",
    joy_teleop::PendingPublication::none,
    gate.take_next());
  if (!gate.terminal()) {
    std::fprintf(stderr, "terminal fault latch was unexpectedly cleared\n");
    ++failures;
  }
}

void test_reset_and_normal_coalescing()
{
  joy_teleop::PublicationGate gate;
  gate.queue_fresh();
  gate.reset(true);
  expect_event(
    "neutral reset discards old sample",
    joy_teleop::PendingPublication::neutral,
    gate.take_next());
  expect_event(
    "neutral reset has no fresh tail",
    joy_teleop::PendingPublication::none,
    gate.take_next());

  gate.queue_fresh();
  gate.queue_fresh();
  expect_event(
    "ordinary reports coalesce to latest",
    joy_teleop::PendingPublication::fresh,
    gate.take_next());
}

}  // namespace

int main()
{
  test_startup_neutral_is_sticky_and_first();
  test_terminal_fault_prevents_fresh_after_neutral();
  test_reset_and_normal_coalescing();
  if (failures != 0) {
    return 1;
  }
  std::puts("publication gate tests passed");
  return 0;
}
