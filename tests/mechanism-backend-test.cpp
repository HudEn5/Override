#include "mechanisms.h"
#include <cassert>
#include <cstring>
#include <limits>

using drive::motion_state;
using mechanism::channel;
using mechanisms::preset::lift_raised;
using mechanisms::preset::lift_start;

static mechanisms::feedback Frame(double left = 80, double right = -25) {
  mechanisms::feedback frame;
  frame.arm_online = frame.arm_valid = frame.claw_online = true;
  frame.left_deg = left;
  frame.right_deg = right;
  return frame;
}

int main() {
  // 启动零点允许两电机原始读数不同；菜单期间只观察，不发运动输出。
  mechanisms::lift_adapter lift;
  auto frame = Frame();
  lift.Observe(frame, false, 0);
  assert(lift.LiftInfo().homed && lift.LiftInfo().position_deg() == 0);
  assert(lift.Info().command.motor_mV == 0);
  assert(!lift.Start(lift_raised, 3000, 0));
  lift.Observe(frame, true, 10);
  assert(lift.Start(lift_raised, 3000, 10));
  assert(lift.Info().command.motor_mV == 12000);
  frame = Frame(320, 215);
  for (int now = 20; now <= 220; now += 10) {
    lift.Observe(frame, true, now);
    lift.Update(now);
  }
  assert(lift.Info().state == motion_state::done);
  assert(lift.LiftInfo().position_deg() == 240);
  assert(lift.Info().command.motor_mV == 0);
  assert(lift.Info().command.stop == manual::stopping::hold);
  lift.Cancel();
  assert(lift.Start(lift_start, 3000, 220));
  assert(lift.Info().command.motor_mV == -2500);
  lift.Cancel();
  assert(lift.Info().command.motor_mV == 0);

  // 已完成后仍检查离线；恢复连接不能重新把高处设为零点。
  mechanisms::lift_adapter disconnected;
  disconnected.Observe(Frame(), true, 0);
  assert(disconnected.Start(lift_raised, 3000, 0));
  auto bad = Frame();
  bad.arm_online = false;
  disconnected.Observe(bad, true, 10);
  disconnected.Update(10);
  assert(disconnected.Info().state == motion_state::error);
  assert(disconnected.Info().command.motor_mV == 0);
  disconnected.Observe(Frame(200, 200), true, 20);
  assert(!disconnected.LiftInfo().homed);
  assert(!disconnected.Start(lift_raised, 3000, 20));

  // 调度器把路线中显式指定的预算原样传给真实后端。
  mechanisms::lift_adapter timed;
  mechanism::scheduler schedule;
  assert(schedule.Bind(channel::arm, timed));
  timed.Observe(Frame(), true, 0);
  assert(schedule.Start(channel::arm, lift_raised, 150, 0));
  for (int now = 10; now < 150; now += 10) {
    timed.Observe(Frame(), true, now);
    assert(schedule.Update(now));
  }
  timed.Observe(Frame(), true, 150);
  assert(!schedule.Update(150));
  assert(schedule.Info().fault);
  assert(timed.Info().command.motor_mV == 0);

  mechanisms::lift_adapter stalled;
  stalled.Observe(Frame(), true, 0);
  assert(stalled.Start(lift_raised, 3000, 0));
  for (int now = 10; now <= 600; now += 10) {
    stalled.Observe(Frame(), true, now);
    stalled.Update(now);
  }
  assert(stalled.Info().state == motion_state::error);
  assert(std::strcmp(stalled.Info().reason, "STALL") == 0);

  mechanisms::lift_adapter stale;
  stale.Observe(Frame(), true, 0);
  assert(!stale.Start(lift_raised, 3000, 10));
  assert(stale.Info().command.motor_mV == 0);
  stale.Observe(Frame(), true, 110);
  assert(!stale.LiftInfo().homed);

  mechanisms::lift_adapter limits;
  limits.Observe(Frame(), true, 0);
  assert(!limits.Start(55, 3000, 0));
  assert(limits.Info().command.motor_mV == 0);
  assert(!limits.Start(lift_raised, -1, 0));
  assert(!limits.Start(lift_raised, std::numeric_limits<double>::quiet_NaN(), 0));
  assert(limits.Start(lift_raised, 3000, 0));
  limits.Observe(Frame(683, 578), true, 10);
  limits.Update(10);
  assert(limits.Info().state == motion_state::error);
  assert(std::strcmp(limits.Info().reason, "POSITION LIMIT") == 0);

  for (int direction : {-1, 1}) {
    mechanisms::claw_adapter claw(direction);
    claw.Observe(true, true, 0);
    assert(claw.Start(mechanisms::preset::claw_close, 1000, 0));
    assert(claw.Info().command.motor_mV == direction * 6000);
    for (int now = 10; now <= 200; now += 10) {
      claw.Observe(true, true, now);
      claw.Update(now);
    }
    assert(claw.Info().state == motion_state::done);
    assert(claw.Info().command.motor_mV == 0);
    claw.Cancel();
    assert(claw.Start(mechanisms::preset::claw_open, 1000, 200));
    assert(claw.Info().command.motor_mV == -direction * 6000);
    claw.Observe(true, false, 210);
    assert(claw.Info().command.motor_mV == 0);
    assert(claw.Info().state == motion_state::cancelled);
    claw.Observe(true, true, 220);
    assert(!claw.Start(55, 1000, 220));
    assert(!claw.Start(mechanisms::preset::claw_close, 1000, 230));
  }
  mechanisms::claw_adapter claw_timeout(1);
  claw_timeout.Observe(true, true, 0);
  assert(claw_timeout.Start(mechanisms::preset::claw_open, 100, 0));
  claw_timeout.Observe(true, true, 100);
  claw_timeout.Update(100);
  assert(claw_timeout.Info().state == motion_state::error);
  assert(claw_timeout.Info().command.motor_mV == 0);
  mechanisms::claw_adapter claw_offline(1);
  claw_offline.Observe(false, true, 0);
  assert(!claw_offline.Start(mechanisms::preset::claw_close, 1000, 0));

  // 与Wait/调度器一致：恰好在截止时完成可成功，迟到或尚未完成均撤压报超时。
  for (bool late : {false, true}) {
    mechanisms::claw_controller boundary;
    boundary.observe(true, true, 0);
    assert(boundary.ClawClose(200, 200));
    boundary.observe(true, true, 100);
    boundary.observe(true, true, 150);
    boundary.observe(true, true, late ? 210 : 200);
    assert(boundary.info().result == (late ? mechanisms::state::error : mechanisms::state::done));
    assert(boundary.info().millivolts == 0);
    mechanisms::lift_controller lift_boundary;
    lift_boundary.observe(Frame(), true, 0);
    assert(lift_boundary.zero_here(true));
    assert(lift_boundary.LiftTo(240, 300));
    for (int now : {100, 150, 200, 250})
      lift_boundary.observe(Frame(320, 215), true, now);
    lift_boundary.observe(Frame(320, 215), true, late ? 310 : 300);
    assert(lift_boundary.info().result == (late ? mechanisms::state::error : mechanisms::state::done));
    assert(lift_boundary.info().left_mV == 0 && lift_boundary.info().right_mV == 0);
  }

  // 原毫米入口保留：没有三点实测标定就拒绝驱动，不猜高度换算。
  mechanisms::lift_controller uncalibrated;
  uncalibrated.observe(Frame(), true, 0);
  assert(uncalibrated.zero_here(true));
  assert(!uncalibrated.LiftHeightTo(200, 3000));
  assert(std::strcmp(uncalibrated.info().status, "HEIGHT NOT CAL") == 0);
}
