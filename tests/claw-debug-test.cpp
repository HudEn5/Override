#include "mechanisms.h"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <limits>
#include <type_traits>
#include <utility>

template <class T> struct accepts_open_without_timeout {
  template <class U> static auto test(int)
      -> decltype(std::declval<U&>().ClawOpen(200), std::true_type());
  template <class> static std::false_type test(...);
  static constexpr bool value = decltype(test<T>(0))::value;
};
template <class T> struct accepts_close_without_timeout {
  template <class U> static auto test(int)
      -> decltype(std::declval<U&>().ClawClose(200), std::true_type());
  template <class> static std::false_type test(...);
  static constexpr bool value = decltype(test<T>(0))::value;
};
static_assert(!accepts_open_without_timeout<mechanisms::claw_controller>::value,
              "ClawOpen requires an explicit timeout");
static_assert(!accepts_close_without_timeout<mechanisms::claw_controller>::value,
              "ClawClose requires an explicit timeout");

struct fixture {
  mechanisms::claw_controller claw;
  double now = 0;
  fixture() { claw.observe(true, true, now); }
  void Tick(double step_ms = 20, bool online = true, bool allowed = true) {
    now += step_ms; claw.observe(online, allowed, now);
  }
};

void CheckError(const mechanisms::claw_controller& claw, const char* reason) {
  assert(claw.info().result == mechanisms::state::error);
  assert(!claw.busy() && claw.info().millivolts == 0);
  assert(!std::strcmp(claw.info().status, reason));
}

void CheckPanel() {
  using mechanisms::claw_debug_action;
  mechanisms::claw_debug_panel panel;
  assert(panel.Update(true, 0, 176, true) == claw_debug_action::none);
  assert(panel.Update(true, 160, 176, true) == claw_debug_action::none);
  panel.Update(false, 0, 176, true);
  const int xs[] = {-1, 0, 159, 160, 319, 320, 479, 480};
  const int ys[] = {175, 176, 207, 208};
  for (int y : ys) for (int x : xs) {
    claw_debug_action expected = claw_debug_action::none;
    if (x >= 0 && x < 480 && y >= 176 && y < 208)
      expected = x < 160 ? claw_debug_action::open : x < 320 ? claw_debug_action::close : claw_debug_action::cancel;
    assert(panel.Update(true, x, y, true) == expected);
    assert(panel.Update(true, x, y, true) == claw_debug_action::none);
    panel.Update(false, x, y, true);
  }
  assert(panel.Update(true, 500, 176, true) == claw_debug_action::none);
  assert(panel.Update(true, 0, 176, true) == claw_debug_action::none);
  panel.Update(false, 0, 176, true);
  assert(panel.Update(true, 160, 176, false) == claw_debug_action::none);
  assert(panel.Update(true, 160, 176, true) == claw_debug_action::none);
  panel.Update(false, 160, 176, false);
  assert(panel.Update(true, 320, 207, true) == claw_debug_action::cancel);
  assert(panel.Update(true, 320, 207, false) == claw_debug_action::none);
  assert(panel.Update(true, 320, 207, true) == claw_debug_action::none);
}

void CheckNumericEntries() {
  fixture open;
  assert(mechanisms::StartClawOpenDebug(open.claw));
  assert(open.claw.busy() && open.claw.info().duration_ms == 200);
  assert(open.claw.info().millivolts == -6000);
  assert(!std::strcmp(open.claw.info().status, "OPENING"));
  open.Tick(100);
  assert(open.claw.busy() && open.claw.info().elapsed_ms == 100);
  open.Tick(100);
  assert(open.claw.info().result == mechanisms::state::done);
  assert(open.claw.info().millivolts == 0 && open.claw.info().elapsed_ms == 200);

  fixture close;
  assert(mechanisms::StartClawCloseDebug(close.claw));
  assert(close.claw.busy() && close.claw.info().duration_ms == 200);
  assert(close.claw.info().millivolts == 6000);
  assert(!std::strcmp(close.claw.info().status, "CLOSING"));
  const mechanisms::claw_info original = close.claw.info();
  assert(!close.claw.ClawOpen(1, 1));
  assert(!close.claw.ClawClose(1, 1));
  assert(close.claw.info().millivolts == original.millivolts);
  assert(close.claw.info().duration_ms == original.duration_ms);
  assert(close.claw.info().status == original.status);
  close.Tick(100); close.Tick(100);
  assert(close.claw.info().result == mechanisms::state::done && close.claw.info().millivolts == 0);

  // 用显式放宽的观察间隔检查真实入口的 1000 ms timeout，而非另写一个测试动作。
  typedef bool (*debug_start)(mechanisms::claw_controller&);
  const debug_start starts[] = {mechanisms::StartClawOpenDebug, mechanisms::StartClawCloseDebug};
  for (debug_start start : starts) {
    mechanisms::claw_config config; config.maximum_gap_ms = 1000;
    fixture before;
    before.claw.configure(config);
    assert(start(before.claw));
    before.Tick(999);
    assert(before.claw.info().result == mechanisms::state::done);
    fixture at;
    at.claw.configure(config);
    assert(start(at.claw));
    at.Tick(1000);
    CheckError(at.claw, "TIMEOUT");
  }

  fixture zero;
  assert(zero.claw.ClawOpen(0, 1));
  assert(zero.claw.info().result == mechanisms::state::done && zero.claw.info().millivolts == 0);
  assert(zero.claw.ClawClose(0, 1));
  assert(zero.claw.info().result == mechanisms::state::done && zero.claw.info().millivolts == 0);

  fixture same_limit;
  assert(same_limit.claw.ClawClose(200, 200));
  same_limit.Tick(100); same_limit.Tick(100);
  CheckError(same_limit.claw, "TIMEOUT");
  assert(same_limit.claw.info().elapsed_ms == 200);

  fixture timeout;
  assert(timeout.claw.ClawOpen(200, 100));
  timeout.Tick(99);
  assert(timeout.claw.busy());
  timeout.Tick(1);
  CheckError(timeout.claw, "TIMEOUT");
  timeout.Tick();
  CheckError(timeout.claw, "TIMEOUT");

  fixture maximum;
  assert(maximum.claw.ClawOpen(5000, 6000));
  for (int i = 0; i < 50; ++i) maximum.Tick(100);
  assert(maximum.claw.info().result == mechanisms::state::done && maximum.claw.info().millivolts == 0);
}

void CheckInvalidArguments() {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const double bad_durations[] = {-1, 5000.01, nan, inf, -inf};
  for (double duration : bad_durations) {
    fixture f;
    assert(!f.claw.ClawOpen(duration, 1000));
    CheckError(f.claw, "DURATION CONFIG");
    assert(!f.claw.ClawClose(duration, 1000));
    CheckError(f.claw, "DURATION CONFIG");
  }
  const double bad_timeouts[] = {0, -1, nan, inf, -inf};
  for (double timeout : bad_timeouts) {
    fixture f;
    assert(!f.claw.ClawOpen(200, timeout));
    CheckError(f.claw, "TIMEOUT CONFIG");
    assert(!f.claw.ClawClose(200, timeout));
    CheckError(f.claw, "TIMEOUT CONFIG");
  }
  const int bad_voltages[] = {-1, 0, 12001};
  for (int voltage : bad_voltages) {
    fixture f;
    mechanisms::claw_config config; config.voltage_mV = voltage;
    f.claw.configure(config);
    assert(!f.claw.ClawOpen(200, 1000));
    CheckError(f.claw, "CONFIG");
  }
  const double bad_config_times[] = {0, -1, nan, inf, -inf};
  for (double invalid : bad_config_times) {
    fixture duration;
    mechanisms::claw_config config; config.maximum_duration_ms = invalid;
    duration.claw.configure(config);
    assert(!duration.claw.ClawClose(200, 1000));
    CheckError(duration.claw, "CONFIG");
    fixture gap;
    config = mechanisms::claw_config(); config.maximum_gap_ms = invalid;
    gap.claw.configure(config);
    assert(!gap.claw.ClawClose(200, 1000));
    CheckError(gap.claw, "CONFIG");
  }
  const int good_voltages[] = {1, 12000};
  for (int voltage : good_voltages) {
    fixture f;
    mechanisms::claw_config config; config.voltage_mV = voltage;
    f.claw.configure(config);
    assert(f.claw.ClawClose(1, 1000));
    assert(f.claw.info().millivolts == voltage);
  }
}

void CheckReadinessAndStops() {
  mechanisms::claw_controller unseen;
  assert(!unseen.ClawOpen(200, 1000));
  CheckError(unseen, "DISABLED/FEEDBACK");
  unseen.observe(false, true, 0);
  assert(!unseen.ClawOpen(200, 1000));
  unseen.observe(true, false, 20);
  assert(!unseen.ClawOpen(200, 1000));

  fixture cancelled;
  assert(cancelled.claw.ClawOpen(200, 1000));
  cancelled.claw.cancel();
  assert(cancelled.claw.info().result == mechanisms::state::cancelled && cancelled.claw.info().millivolts == 0);
  cancelled.Tick();
  assert(cancelled.claw.info().result == mechanisms::state::cancelled && !cancelled.claw.busy());

  fixture disabled;
  assert(disabled.claw.ClawClose(200, 1000));
  disabled.Tick(20, true, false);
  assert(disabled.claw.info().result == mechanisms::state::cancelled && disabled.claw.info().millivolts == 0);
  disabled.Tick();
  assert(disabled.claw.info().result == mechanisms::state::cancelled && !disabled.claw.busy());

  fixture offline;
  assert(offline.claw.ClawClose(200, 1000));
  offline.Tick(20, false);
  CheckError(offline.claw, "MOTOR OFF");
  offline.Tick();
  CheckError(offline.claw, "MOTOR OFF");

  fixture reconfigured;
  assert(reconfigured.claw.ClawClose(200, 1000));
  reconfigured.claw.configure(mechanisms::claw_config());
  assert(reconfigured.claw.info().result == mechanisms::state::cancelled && reconfigured.claw.info().millivolts == 0);
}

void CheckClockFailures() {
  const double bad_initial[] = {-1, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()};
  for (double time : bad_initial) {
    mechanisms::claw_controller claw;
    claw.observe(true, true, time);
    assert(!claw.ClawOpen(200, 1000));
    CheckError(claw, "DISABLED/FEEDBACK");
  }
  const double bad_running[] = {-1, 101, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()};
  for (double time : bad_running) {
    fixture f;
    assert(f.claw.ClawClose(200, 1000));
    f.claw.observe(true, true, time);
    CheckError(f.claw, "CLOCK/STALE");
    f.claw.observe(true, true, 200);
    f.claw.observe(true, true, 220);
    CheckError(f.claw, "CLOCK/STALE");
  }
  fixture reversed;
  reversed.Tick(20);
  assert(reversed.claw.ClawOpen(200, 1000));
  reversed.claw.observe(true, true, 19);
  CheckError(reversed.claw, "CLOCK/STALE");
  reversed.claw.observe(true, true, 40);
  CheckError(reversed.claw, "CLOCK/STALE");
}

int main() {
  CheckPanel();
  CheckNumericEntries();
  CheckInvalidArguments();
  CheckReadinessAndStops();
  CheckClockFailures();
  std::puts("Claw debug checks: explicit timeout, numeric entries, timing priority, validation, output stop and touch edges.");
}
