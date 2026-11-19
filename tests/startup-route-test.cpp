#include "autonomous.h"
#include "startup-mode.h"
#include "performance-config.h"
#include <cassert>
#include <cstdio>

void autonomous_program::RunRoutine() {
  using namespace route;
  RunRoute({DriveStraight(60, 100, 100000), Wait(120, 100000), Turn(90, 100, 100000)});
}
struct fixture {
  runtime::startup_mode Selection;
  autonomous_program Program;
  manual::driver_control Driver{false};
  manual::input input;
  manual::output output;
  drive::straight_feedback frame;
  bool field = false, enabled = false, driver = false, automatic = false, controller = true,
       field_owner = false;
  runtime::competition_stage stage = runtime::competition_stage::none;
  double left = 0, right = 0;
  int now = 0;
  fixture() {
    drive::straight_config s;
    s.motion.wheel_diameter_mm = 69.85;
    s.motion.motor_revs_per_wheel_rev = 1.2;
    s.left_forward.kS = 400;
    s.left_forward.kV = 16;
    s.left_forward.calibrated = true;
    s.right_forward = s.left_reverse = s.right_reverse = s.left_forward;
    drive::turn_config t;
    t.kP = 1.6;
    t.acceleration_rpm_s = 240;
    t.correction_fraction = 3;
    t.pid.kP = t.pid.kI = 20;
    t.pid.max_correction_mV = 2000;
    t.pid.max_integral_mV = 1200;
    t.left_forward = t.right_forward = t.left_reverse = t.right_reverse = s.left_forward;
    Program.Configure(config::HighSpeedStraightControl(s), t);
    frame.motors_online = frame.inertial_ready = frame.position.valid = frame.velocity.valid = true;
  }
  void Step() {
    const auto motor = [](int mv) {
      return std::copysign(std::max(0.0, std::fabs(static_cast<double>(mv)) - 400) / 16, mv);
    };
    left += (motor(output.left_mV) - left) * .25;
    right += (motor(output.right_mV) - right) * .25;
    frame.position.left_deg += left * .12;
    frame.position.right_deg += right * .12;
    frame.heading_deg +=
        (right - left) * (69.85 * 3.141592653589793 / 72) / 325 * 180 / 3.141592653589793 * .02;
    now += 20;
    frame.velocity.sequence++;
    frame.velocity.time_ms = now;
    frame.velocity.left_rpm = left;
    frame.velocity.right_rpm = right;
    frame.stopped = std::fabs(left) < 5 && std::fabs(right) < 5;
    const bool sticks = std::abs(input.y_axis) > 5 || std::abs(input.angular_axis) > 5;
    Selection.Update(input, controller, field, sticks, frame.stopped, frame.stopped, Program.Info().finished,
                     now);
    const auto access =
        runtime::SessionPermissions(Selection, true, field, enabled, driver, automatic, controller, stage);
    output = Driver.Update(input, access.manual, 5, 12000, 12000, 4000, 6000);
    if (!field && !access.autonomous && Program.Info().active)
      Program.CancelRoutine();
    if (field != field_owner) {
      Program.Update(false, frame, now);
      field_owner = field;
    }
    Program.Update(access.autonomous, frame, now);
    if (access.autonomous) {
      output.left_mV = Program.Info().left_mV;
      output.right_mV = Program.Info().right_mV;
    }
    assert(!(access.autonomous && access.manual));
  }
  void Start() {
    input = manual::input();
    Step();
    input.test_start = true;
    Step();
    for (int i = 0; i < 5; ++i)
      Step();
    assert(Program.Info().run == 0);
    input.test_start = false;
    Step();
    assert(Program.Info().active);
  }
};
int main() {
  fixture completed;
  completed.Start();
  int finished_at = -1, manual_at = -1;
  for (int i = 0; i < 1500 && completed.Selection.Mode() != runtime::local_mode::manual; ++i) {
    completed.Step();
    if (completed.Program.Info().finished && finished_at < 0)
      finished_at = completed.now;
    if (completed.Program.Info().finished)
      assert(completed.output.left_mV == 0 && completed.output.right_mV == 0);
    if (completed.Selection.Mode() == runtime::local_mode::manual)
      manual_at = completed.now;
  }
  assert(manual_at - finished_at >= 3000 && manual_at - finished_at <= 3020);
  assert(completed.Program.Info().result == route_result::done && completed.Program.Info().step == 3);
  assert(completed.Selection.Mode() == runtime::local_mode::manual);
  for (int i = 0; i < 30; ++i)
    completed.Step();
  assert(completed.Program.Info().run == 1 && completed.output.left_mV == 0 &&
         completed.output.right_mV == 0);
  fixture cancel;
  cancel.Start();
  for (int i = 0; i < 500 && !cancel.Program.Info().waiting; ++i)
    cancel.Step();
  assert(cancel.Program.Info().waiting);
  cancel.input.test_cancel = true;
  cancel.Step();
  assert(cancel.Program.Info().result == route_result::cancelled && cancel.Program.Info().step == 1);
  assert(cancel.output.left_mV == 0 && cancel.output.right_mV == 0);
  fixture takeover;
  takeover.Start();
  takeover.input.y_axis = 50;
  takeover.Step();
  assert(takeover.Program.Info().result == route_result::cancelled && takeover.output.left_mV > 0);
  fixture handoff;
  handoff.Start();
  handoff.Step();
  handoff.field = handoff.enabled = handoff.automatic = true;
  handoff.controller = false;
  handoff.stage = runtime::competition_stage::autonomous;
  handoff.Step();
  assert(handoff.Program.Info().run == 2 &&
         handoff.Program.Info().active);  // 场控启动新路线，手柄离线仍允许。
  handoff.enabled = false;
  handoff.Step();
  assert(!handoff.Program.Info().active && handoff.output.left_mV == 0 && handoff.output.right_mV == 0);
  handoff.field = false;
  handoff.controller = true;
  handoff.Step();
  assert(handoff.Selection.Mode() == runtime::local_mode::select && !handoff.Program.Info().active);
  fixture match;
  match.field = match.enabled = match.automatic = true;
  match.controller = false;
  match.stage = runtime::competition_stage::autonomous;
  for (int i = 0; i < 1500 && !match.Program.Info().finished; ++i)
    match.Step();
  assert(match.Program.Info().result == route_result::done && match.Program.Info().run == 1);
  for (int i = 0; i < 200; ++i) {
    match.Step();
    assert(!match.Selection.ReturningToManual() && match.Selection.Mode() == runtime::local_mode::select);
    assert(match.Program.Info().run == 1 && match.output.left_mV == 0 && match.output.right_mV == 0);
  }
  std::puts(
      "Startup/route integration checks passed: one selected run, completion-to-manual, wait cancellation, "
      "stick takeover and fresh field-owned run without controller.");
}
