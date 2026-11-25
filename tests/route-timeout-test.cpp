#include "autonomous.h"
#include <cassert>
#include <cstring>
#include <limits>
#include <cstdio>

// 所有路线工厂漏写最后的timeout都必须编译失败。
#ifdef MISSING_ROUTE_TIMEOUT
void MissingTimeout() {
  using namespace route;
#if MISSING_ROUTE_TIMEOUT == 0
  DriveStraight(100, 100);
#elif MISSING_ROUTE_TIMEOUT == 1
  Turn(90, 100);
#elif MISSING_ROUTE_TIMEOUT == 2
  TurnToHeading(90, 100);
#elif MISSING_ROUTE_TIMEOUT == 3
  DriveCorner(100, 90, 100, 100);
#elif MISSING_ROUTE_TIMEOUT == 4
  MoveToPoint(100, 100, 100, 100);
#elif MISSING_ROUTE_TIMEOUT == 5
  Swing(90, drive::pivot_side::left, 100);
#elif MISSING_ROUTE_TIMEOUT == 6
  Wait(100);
#elif MISSING_ROUTE_TIMEOUT == 7
  PushFor(100, 4);
#elif MISSING_ROUTE_TIMEOUT == 8
  DriveTurnDrive(100, 90, 100, turn_mode::rolling, 150, 100);
#elif MISSING_ROUTE_TIMEOUT == 9
  StartMechanism(mechanism::channel::arm, 1);
#elif MISSING_ROUTE_TIMEOUT == 10
  WaitMechanism(mechanism::channel::arm);
#elif MISSING_ROUTE_TIMEOUT == 11
  DriveStraightWithTrigger(100, 100, 50, mechanism::channel::arm, 1, 1000);
#elif MISSING_ROUTE_TIMEOUT == 12
  LiftTo(100);
#elif MISSING_ROUTE_TIMEOUT == 13
  ClawOpen(300, 12.0);
#else
  ClawClose(500, 12.0);
#endif
}
#endif

static route::command requested = route::Wait(500, 100);
static bool following_started = false;
void autonomous_program::RunRoutine() {
  RunRoute({requested, route::Wait(0, 100)});
  if (info.step > 0)
    following_started = true;
}

void Check(const route::command& command, bool done, const char* status = "TIMEOUT",
           bool switch_stage = false) {
  requested = command;
  following_started = false;
  drive::straight_config s;
  s.motion.wheel_diameter_mm = 69.85;
  s.motion.motor_revs_per_wheel_rev = 1.2;
  s.left_forward.kS = 400;
  s.left_forward.kV = 16;
  s.left_forward.calibrated = true;
  s.right_forward = s.left_reverse = s.right_reverse = s.left_forward;
  drive::turn_config t;
  t.left_forward = t.right_forward = t.left_reverse = t.right_reverse = s.left_forward;
  t.settle_ms = t.rate_filter_ms = 0;
  drive::corner_config c;
  c.wheel_diameter_mm = 69.85;
  c.motor_revs_per_wheel_rev = 1.2;
  c.track_width_mm = 325;
  c.left_forward = c.right_forward = c.left_reverse = c.right_reverse = s.left_forward;
  autonomous_program p;
  p.Configure(s, t, c);
  drive::straight_feedback f;
  f.position.valid = f.velocity.valid = f.motors_online = f.inertial_ready = f.stopped = true;
  for (int tick = 0; tick <= 100 && !p.Info().finished; ++tick) {
    if (switch_stage && tick >= 3)
      f.heading_deg = command.kind == route::action::point ? 45 : 90;
    f.velocity.time_ms = tick * 20;
    f.velocity.sequence++;
    drive::pose pose;
    pose.valid = true;
    pose.heading_deg = f.heading_deg;  // 模拟IMU转向时，位姿朝向也必须同步。
    pose.time_ms = tick * 20;
    p.ObservePose(pose);
    p.Update(true, f, tick * 20);
  }
  if (std::strcmp(p.Info().status, done ? "DONE" : status)) {
    std::printf("Timeout test got %s, expected %s\n", p.Info().status, done ? "DONE" : status);
    std::fflush(stdout);
  }
  assert(p.Info().finished && p.Info().result == (done ? route_result::done : route_result::error));
  assert(std::strcmp(p.Info().status, done ? "DONE" : status) == 0);
  assert(p.Info().left_mV == 0 && p.Info().right_mV == 0);
  assert(following_started == done);
  if (switch_stage)
    assert(p.Info().motion == autonomous_motion::straight);
  if (!done && !std::strcmp(status, "TIMEOUT"))
    assert(p.Info().time_ms == command.timeout_ms);
}

int main() {
  using namespace route;
  Check(DriveStraight(100, 100, 100), false);
  Check(Turn(90, 100, 100), false);
  Check(TurnToHeading(90, 100, 100), false);
  Check(DriveCorner(100, 90, 100, 100, 100), false);
  Check(MoveToPoint(100, 100, 100, 100, 100), false);
  Check(Swing(90, drive::pivot_side::left, 100, 100), false);
  Check(DriveTurnDrive(100, 90, 100, turn_mode::stop, 150, 100, 100), false);
  Check(DriveTurnDrive(100, 90, 100, turn_mode::rolling, 150, 100, 100), false);
  Check(PushFor(500, 4, 100), false);
  Check(Wait(500, 100), false);
  Check(MoveToPoint(100, 100, 100, 100, 200), false, "TIMEOUT", true);
  Check(DriveTurnDrive(0, 90, 100, turn_mode::stop, 150, 100, 200), false, "TIMEOUT", true);
  Check(Wait(100, 100), true);  // 等待恰好达到期限时允许正常完成。
  Check(Wait(0, 100), true);
  Check(DriveStraight(0, 100, 100), true);
  for (double invalid :
       {0., -1., std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
    Check(Wait(0, invalid), false, "TIMEOUT CONFIG");
  std::puts(
      "Route timeout checks passed: all actions, exact cutoff, no following motion, Wait boundary and "
      "invalid budgets.");
}
