#include "autonomous.h"
#include "performance-config.h"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <limits>

int scenario = 0;
void autonomous_program::RunRoutine() {
  using namespace route;
  switch (scenario) {
    case 0:
      RunRoute({DriveStraight(300, route::DEFAULT_DRIVE_RPM, 100000), Wait(120, 100000),
                Turn(90, route::DEFAULT_TURN_RPM, 100000), Wait(0, 100000),
                DriveStraight(-120, route::DEFAULT_DRIVE_RPM, 100000),
                TurnToHeading(0, route::DEFAULT_TURN_RPM, 100000),
                DriveCorner(100, 90, 100, route::DEFAULT_DRIVE_RPM, 100000)});
      break;
    case 1:
      RunRoute({Wait(std::numeric_limits<double>::quiet_NaN(), 100000)});
      break;
    case 2:
      RunRoute({Wait(-1, 100000)});
      break;
    case 3:
      RunRoute({DriveStraight(std::numeric_limits<double>::quiet_NaN(), route::DEFAULT_DRIVE_RPM, 100000)});
      break;
    case 4:
      RunRoute({Wait(500, 100000)});
      break;
    case 5:
      RunRoute({Turn(90, 601, 100000)});
      break;
    case 6:
      RunRoute({command(static_cast<action>(99), 1000)});
      break;
    case 7:
      RunRoute({Wait(0, 100000), DriveStraight(0, route::DEFAULT_DRIVE_RPM, 100000), Wait(0, 100000)});
      break;
    case 8:
      RunRoute({});
      break;
    case 9:
      RunRoute({Wait(std::numeric_limits<double>::infinity(), 100000)});
      break;
  }
}
struct fixture {
  autonomous_program Program;
  drive::straight_feedback frame;
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
    t.kD = .2;
    t.acceleration_rpm_s = 240;
    t.minimum_settle_rpm = 18;
    t.minimum_finish_rpm = 12;
    t.minimum_rate_deg_s = 16;
    t.pid.kP = t.pid.kI = 20;
    t.pid.max_correction_mV = 2000;
    t.pid.max_integral_mV = 1200;
    t.correction_fraction = 3;
    t.left_forward = t.right_forward = t.left_reverse = t.right_reverse = s.left_forward;
    drive::corner_config c;
    c.wheel_diameter_mm = 69.85;
    c.motor_revs_per_wheel_rev = 1.2;
    c.track_width_mm = 325;
    c.angle_tolerance_deg = c.exit_angle_tolerance_deg = 1.5;
    c.exit_rate_tolerance_deg_s = 4;
    c.pid = t.pid;
    c.finish_kP = .6;
    c.finish_max_rpm = 30;
    c.left_forward = c.right_forward = c.left_reverse = c.right_reverse = s.left_forward;
    Program.Configure(config::HighSpeedStraightControl(s), t, config::HighSpeedCornerControl(c));
    frame.motors_online = frame.inertial_ready = frame.position.valid = frame.velocity.valid = frame.stopped =
        true;
  }
  void Step(bool permitted = true) {
    const auto& old = Program.Info();
    const auto motor = [](int mv) {
      return std::copysign(std::max(0.0, std::fabs(static_cast<double>(mv)) - 400) / 16, mv);
    };
    left += (motor(old.left_mV) - left) * .25;
    right += (motor(old.right_mV) - right) * .25;
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
    Program.Update(permitted, frame, now);
    assert(std::abs(Program.Info().left_mV) <= 12000 && std::abs(Program.Info().right_mV) <= 12000);
  }
};
int main() {
  fixture f;
  int wait_begin = -1, wait_end = -1;
  for (int i = 0; i < 3000 && !f.Program.Info().finished; ++i) {
    f.Step();
    const auto& a = f.Program.Info();
    if (a.waiting) {
      assert(a.motion == autonomous_motion::none && a.left_mV == 0 && a.right_mV == 0);
      if (wait_begin < 0)
        wait_begin = f.now;
    } else if (wait_begin >= 0 && wait_end < 0)
      wait_end = f.now;
  }
  if (!(f.Program.Info().finished && f.Program.Info().result == route_result::done &&
        f.Program.Info().step == 7)) {
    const auto& a = f.Program.Info();
    std::printf(
        "Route failed: step %u result %d status %s turn %s error %.3f corner %s error %.3f distance %.3f\n",
        a.step, static_cast<int>(a.result), a.status, a.turn.status, a.turn.error_deg, a.corner.status,
        a.corner.error_deg, a.drive.motion.remaining_mm);
    std::fflush(stdout);
  }
  assert(f.Program.Info().finished && f.Program.Info().result == route_result::done &&
         f.Program.Info().step == 7);
  assert(std::fabs(f.frame.heading_deg - 90) <= 1.5 && wait_end - wait_begin == 120);
  const auto run = f.Program.Info().run;
  for (int i = 0; i < 20; ++i)
    f.Step();
  assert(f.Program.Info().run == run && f.Program.Info().left_mV == 0 && f.Program.Info().right_mV == 0);
  for (int bad : {1, 2, 3, 5, 6, 9}) {
    scenario = bad;
    fixture invalid;
    invalid.Step();
    assert(invalid.Program.Info().finished && invalid.Program.Info().result == route_result::error);
    assert(invalid.Program.Info().left_mV == 0 && invalid.Program.Info().right_mV == 0);
  }
  for (int empty : {7, 8}) {
    scenario = empty;
    fixture instant;
    instant.Step();
    assert(instant.Program.Info().finished && instant.Program.Info().result == route_result::done);
    assert(instant.Program.Info().left_mV == 0 && instant.Program.Info().right_mV == 0);
  }
  scenario = 4;
  fixture waiting;
  waiting.Step();
  waiting.Step();
  waiting.Step(false);
  assert(waiting.Program.Info().finished && waiting.Program.Info().result == route_result::cancelled &&
         !waiting.Program.Info().waiting);
  waiting.Step();
  const auto new_run = waiting.Program.Info().run;
  for (int i = 0; i < 24; ++i)
    waiting.Step();
  assert(waiting.Program.Info().active && waiting.Program.Info().run == new_run);
  waiting.Step();
  assert(waiting.Program.Info().finished && waiting.Program.Info().result == route_result::done);
  fixture cancelled;
  cancelled.Step();
  cancelled.Program.CancelRoutine();
  cancelled.Step(false);
  assert(cancelled.Program.Info().result == route_result::cancelled &&
         !std::strcmp(cancelled.Program.Info().status, "CANCEL"));
  for (double clock :
       {-1.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
    fixture broken;
    broken.Step();
    broken.Program.Update(true, broken.frame, clock);
    assert(broken.Program.Info().finished && broken.Program.Info().result == route_result::error);
    assert(!std::strcmp(broken.Program.Info().status, "CLOCK ERR"));
  }
  fixture backwards;
  backwards.Step();
  backwards.Program.Update(true, backwards.frame, 10);
  assert(backwards.Program.Info().result == route_result::error);
  scenario = 0;
  fixture sensor;
  sensor.Step();
  sensor.frame.inertial_ready = false;
  sensor.Step();
  assert(sensor.Program.Info().result == route_result::error && sensor.Program.Info().step == 0 &&
         sensor.Program.Info().left_mV == 0);
  std::puts(
      "Route sequence checks passed: high-speed mixed commands, exact nonblocking Wait, immediate start "
      "errors, clock guards, cancel/restart and terminal latch.");
}
