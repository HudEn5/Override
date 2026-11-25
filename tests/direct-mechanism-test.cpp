#include "autonomous.h"
#include "mechanisms.h"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <limits>

// 默认测试固定的机构组合；BARTMOSS_IMPORTED_ROUTE仅在核验导入的10号路线时按需启用，
// 避免后续用户改路线参数，就把合法的新路线误判成旧路线回归失败。
#ifndef BARTMOSS_IMPORTED_ROUTE
static int scenario = 0;
static route::command requested = route::ClawClose(500, 12, 1000);
void autonomous_program::RunRoutine() {
  using namespace route;
  using mechanism::channel;
  if (scenario == 0)
    RunRoute({ClawClose(500, 12, 1000), WaitMechanism(channel::claw, 1000), Wait(500, 1000),
              LiftTo(100, 6000), DriveStraight(300, 540, 5000), ClawOpen(300, 12, 1000)});
  else if (scenario == 1)
    RunRoute({requested, DriveStraight(300, 540, 5000)});
  else if (scenario == 2)
    RunRoute({ClawClose(30, 12, 100), WaitMechanism(channel::claw, 100),
              StartMechanism(channel::claw, mechanisms::preset::claw_close, 1000)});
  else if (scenario == 3)
    RunRoute({LiftTo(100, 100)});
  else if (scenario == 4)
    RunRoute({ClawOpen(0, 4.5, 100)});
  else
    RunRoute({ClawClose(500, 12, 50), DriveStraight(300, 540, 5000)});
}
#endif

struct robot {
  autonomous_program program;
  mechanisms::lift_adapter arm;
  mechanisms::claw_adapter claw{1};
  drive::straight_feedback frame;
  mechanisms::feedback arm_frame;
  manual::output applied;
  double left_rpm = 0, right_rpm = 0, arm_deg = 0, arm_rate = 0;
  int now = 0;
  bool claw_online = true, arm_online = true, overlap = false, drive_before_close = false;
  bool saw_12 = false, saw_6 = false, saw_open = false;
  robot() {
    drive::straight_config straight;
    straight.motion.wheel_diameter_mm = 69.85;
    straight.motion.motor_revs_per_wheel_rev = 1.2;
    straight.left_forward.kS = 400;
    straight.left_forward.kV = 16;
    straight.left_forward.calibrated = true;
    straight.right_forward = straight.left_reverse = straight.right_reverse = straight.left_forward;
    drive::turn_config turn;
    turn.left_forward = turn.right_forward = turn.left_reverse = turn.right_reverse = straight.left_forward;
    program.Configure(straight, turn);
    assert(program.BindMechanism(mechanism::channel::arm, arm));
    assert(program.BindMechanism(mechanism::channel::claw, claw));
    frame.motors_online = frame.inertial_ready = frame.position.valid = frame.velocity.valid = frame.stopped =
        true;
    arm_frame.arm_online = arm_frame.arm_valid = true;
    arm.Observe(arm_frame, false, 0);
    claw.Observe(true, false, 0);
    program.Update(false, frame, 0);
  }
  void Tick(bool allowed = true, bool fresh_arm = true, bool fresh_claw = true) {
    now += 10;
    const auto old = program.Info();
    const auto motor = [](int mv) {
      return std::copysign(std::max(0., std::fabs(static_cast<double>(mv)) - 400) / 16, mv);
    };
    left_rpm += (motor(old.left_mV) - left_rpm) * .2;
    right_rpm += (motor(old.right_mV) - right_rpm) * .2;
    frame.position.left_deg += left_rpm * .06;
    frame.position.right_deg += right_rpm * .06;
    frame.velocity.left_rpm = left_rpm;
    frame.velocity.right_rpm = right_rpm;
    frame.velocity.time_ms = now;
    ++frame.velocity.sequence;
    frame.stopped = std::fabs(left_rpm) < 5 && std::fabs(right_rpm) < 5;
    arm_rate += (applied.arm_mV / 120.0 - arm_rate) * .2;
    arm_deg += arm_rate * .01;
    arm_frame.left_deg = arm_frame.right_deg = arm_deg;
    arm_frame.left_deg_s = arm_frame.right_deg_s = arm_rate;
    arm_frame.arm_online = arm_online;
    if (fresh_arm)
      arm.Observe(arm_frame, allowed, now);
    if (fresh_claw)
      claw.Observe(claw_online, allowed, now);
    program.Update(allowed, frame, now);
    applied = manual::output();
    program.ApplyMechanisms(applied);
    saw_12 |= applied.claw_mV == 12000;
    saw_6 |= applied.claw_mV == 6000;
    saw_open |= applied.claw_mV == -12000;
    const bool driving = program.Info().left_mV != 0 || program.Info().right_mV != 0;
    overlap |= driving && applied.arm_mV != 0;
    drive_before_close |= driving && now < 1010;
  }
  void Run() {
    for (int tick = 0; tick < 1500 && !program.Info().finished; ++tick)
      Tick();
  }
};

int main() {
  robot normal;
  normal.Tick();
  assert(normal.applied.claw_mV == 12000);  // 首个自动周期已经闭爪，不沿用菜单禁用状态。
  normal.Run();
  if (normal.program.Info().result != route_result::done) {
    std::printf("Imported route got %s at step %u, arm %.2f target %.2f t=%d\n", normal.program.Info().status,
                normal.program.Info().step, normal.arm.LiftInfo().position_deg(),
                normal.arm.LiftInfo().target_deg, normal.now);
    std::fflush(stdout);
  }
  assert(normal.program.Info().result == route_result::done);
  assert(normal.overlap && !normal.drive_before_close && normal.saw_12 && normal.saw_open);
  assert(normal.arm.LiftInfo().target_deg == 100);
  assert(std::fabs(normal.arm.LiftInfo().position_deg() - 100) <= 2);
  assert(normal.applied.arm_mV == 0 && normal.applied.claw_mV == 0);
  assert(normal.program.Info().left_mV == 0 && normal.program.Info().right_mV == 0);

  robot cancel;
  while (!cancel.overlap && cancel.now < 2000)
    cancel.Tick();
  assert(cancel.overlap);
  cancel.Tick(false);
  assert(cancel.program.Info().result == route_result::cancelled);
  assert(cancel.applied.arm_mV == 0 && cancel.applied.claw_mV == 0);
  assert(cancel.program.Info().left_mV == 0 && cancel.program.Info().right_mV == 0);

  robot offline;
  offline.claw_online = false;
  offline.Tick();
  assert(offline.program.Info().result == route_result::error && offline.program.Info().step == 0);
  assert(offline.program.Info().left_mV == 0 && offline.applied.claw_mV == 0);

  robot lost;
  lost.Tick();
  lost.claw_online = false;
  lost.Tick();
  assert(lost.program.Info().result == route_result::error && lost.applied.claw_mV == 0);

  robot arm_lost;
  while (!arm_lost.overlap && arm_lost.now < 2000)
    arm_lost.Tick();
  arm_lost.arm_online = false;
  arm_lost.Tick();
  assert(arm_lost.program.Info().result == route_result::error);
  assert(arm_lost.program.Info().left_mV == 0 && arm_lost.applied.arm_mV == 0);

  robot stale;
  while (!stale.overlap && stale.now < 2000)
    stale.Tick();
  stale.Tick(true, false);
  assert(stale.program.Info().result == route_result::error);
  assert(stale.program.Info().left_mV == 0 && stale.applied.arm_mV == 0);

#ifndef BARTMOSS_IMPORTED_ROUTE
  scenario = 1;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const route::command invalid[] = {route::LiftTo(601, 1000),       route::LiftTo(-1, 1000),
                                    route::LiftTo(nan, 1000),       route::ClawClose(500, 13, 1000),
                                    route::ClawClose(500, 0, 1000), route::ClawClose(500, nan, 1000),
                                    route::ClawOpen(-1, 12, 1000),  route::ClawOpen(5001, 12, 1000)};
  for (const auto& command : invalid) {
    requested = command;
    robot rejected;
    rejected.Tick();
    assert(rejected.program.Info().result == route_result::error);
    assert(rejected.program.Info().step == 0 && rejected.program.Info().left_mV == 0);
    assert(rejected.applied.arm_mV == 0 && rejected.applied.claw_mV == 0);
  }
  scenario = 2;
  robot preset;
  preset.Run();
  assert(preset.program.Info().result == route_result::done && preset.saw_12 && preset.saw_6);
  scenario = 3;
  robot arm_timeout;
  arm_timeout.Run();
  assert(arm_timeout.program.Info().result == route_result::error);
  assert(arm_timeout.now == 110 && arm_timeout.applied.arm_mV == 0);
  scenario = 4;
  robot zero;
  zero.Run();
  assert(zero.program.Info().result == route_result::done && zero.now == 10);
  scenario = 5;
  robot claw_timeout;
  claw_timeout.Run();
  assert(claw_timeout.program.Info().result == route_result::error && claw_timeout.now == 60);
  assert(claw_timeout.applied.claw_mV == 0 && claw_timeout.program.Info().left_mV == 0);
#endif
  std::puts(
      "Direct mechanisms passed: first-cycle close, imported sequence, overlap/join, limits, deadlines, "
      "faults and disable cancellation.");
}
