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
  mechanisms::arm_tuning_test tuning(4000, 8000, 2500, 65, 800);
  mechanisms::lift_info tuning_position;
  tuning_position.homed = true;
  tuning_position.left_deg = tuning_position.right_deg = 20;
  manual::input tuning_input;
  manual::output tuning_output;
  tuning_input.arm_up = true;
  tuning.Update(true, tuning_input, Frame(), tuning_position, 0, tuning_output);
  assert(tuning_output.arm_mV == 0);  // 进入测试档时已经按住 R1，须先松开。
  tuning_input.arm_up = false;
  tuning_input.test_next = true;
  assert(tuning.Update(true, tuning_input, Frame(), tuning_position, 10, tuning_output).up_mV == 5000);
  assert(tuning.Update(true, tuning_input, Frame(), tuning_position, 20, tuning_output).up_mV == 5000);
  tuning_input.arm_up = true;
  tuning.Update(true, tuning_input, Frame(), tuning_position, 30, tuning_output);
  assert(tuning_output.arm_mV == 5000);
  tuning.Update(true, tuning_input, Frame(), tuning_position, 830, tuning_output);
  assert(tuning_output.arm_mV == 0);
  tuning.Update(true, tuning_input, Frame(), tuning_position, 840, tuning_output);
  assert(tuning_output.arm_mV == 0);  // 持续按住到时间，也只允许一次脉冲。
  tuning_input.arm_up = false;
  tuning.Update(true, tuning_input, Frame(), tuning_position, 850, tuning_output);
  tuning_input.arm_down = true;
  tuning.Update(true, tuning_input, Frame(), tuning_position, 860, tuning_output);
  assert(tuning_output.arm_mV == -2500);
  tuning_position.left_deg = 0;
  tuning.Update(true, tuning_input, Frame(), tuning_position, 870, tuning_output);
  assert(tuning_output.arm_mV == 0 && tuning_output.arm_stop == manual::stopping::hold);

  // 启动零点允许两电机原始读数不同；菜单期间只观察，不发运动输出。
  mechanisms::lift_adapter lift;
  auto frame = Frame();
  lift.Observe(frame, false, 0);
  assert(lift.LiftInfo().homed && lift.LiftInfo().position_deg() == 0);
  assert(lift.Info().command.motor_mV == 0);
  assert(!lift.Start(lift_raised, 3000, 0));
  lift.Observe(frame, true, 10);
  assert(lift.Start(lift_raised, 3000, 10));
  assert(lift.Info().command.motor_mV == 6000);
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

  // 手臂保持时，单帧速度突刺不会让后台调度报错或丢掉零位。
  mechanisms::lift_adapter holding;
  mechanism::scheduler held_schedule;
  assert(held_schedule.Bind(channel::arm, holding));
  holding.Observe(Frame(), true, 0);
  assert(held_schedule.Start(channel::arm, lift_start, 1000, 0));
  for (int now = 10; now <= 200; now += 10) {
    holding.Observe(Frame(), true, now);
    assert(held_schedule.Update(now));
  }
  assert(held_schedule.Done(channel::arm));
  auto speed_spike = Frame();
  speed_spike.left_deg_s = 800;
  holding.Observe(speed_spike, true, 210);
  assert(held_schedule.Update(210) && held_schedule.Done(channel::arm));
  assert(holding.LiftInfo().homed && holding.Info().command.motor_mV == 0);
  holding.Observe(Frame(), true, 220);
  assert(held_schedule.Update(220) && holding.LiftInfo().homed);

  // 升降中坏帧立即撤压，有效数据恢复后继续。连续坏帧超过窗口才报故障。
  mechanisms::lift_controller moving;
  moving.observe(Frame(), true, 0);
  assert(moving.zero_here(true) && moving.LiftTo(45, 4000));
  moving.observe(speed_spike, true, 10);
  assert(moving.busy() && moving.info().left_mV == 0 && !std::strcmp(moving.info().status, "ARM WAIT"));
  moving.observe(Frame(), true, 20);
  assert(moving.busy() && moving.info().left_mV > 0);
  for (int now = 30; now < 130; now += 10) {
    moving.observe(speed_spike, true, now);
    assert(moving.busy() && moving.info().left_mV == 0);
  }
  moving.observe(speed_spike, true, 130);
  assert(moving.info().result == mechanisms::state::error && !std::strcmp(moving.info().status, "ARM SENSOR"));

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

  // 55° 带载抬升停滞超过 600 ms 后仍可到位，持续卡住则由动作超时撤压。
  for (bool blocked : {false, true}) {
    mechanisms::lift_controller loaded;
    loaded.observe(Frame(), true, 0);
    assert(loaded.zero_here(true));
    const auto plateau = Frame(133, 26.6);
    loaded.observe(plateau, true, 10);
    assert(loaded.LiftTo(55, 4000));
    const int initial_mV = loaded.info().left_mV;
    for (int now = 20; now <= 710; now += 10) {
      loaded.observe(plateau, true, now);
      assert(loaded.busy());
      assert(loaded.info().left_mV == loaded.info().right_mV);
      assert(loaded.info().left_mV <= 6000);
      if (now <= 130)
        assert(loaded.info().left_mV == initial_mV);
      if (now == 180)
        assert(loaded.info().left_mV > initial_mV && loaded.info().left_mV < 6000);
      if (now >= 230)
        assert(loaded.info().left_mV == 6000);
    }
    const auto destination = blocked ? plateau : Frame(134.6, 28.8);
    const int deadline = blocked ? 4010 : 920;
    for (int now = 720; now <= deadline; now += 10) {
      loaded.observe(destination, true, now);
      if (now < deadline)
        assert(loaded.busy());
    }
    assert(loaded.info().left_mV == 0 && loaded.info().right_mV == 0);
    if (blocked)
      assert(loaded.info().result == mechanisms::state::error && !std::strcmp(loaded.info().status, "TIMEOUT"));
    else
      assert(loaded.at_target());
  }

  // 下降停滞也继续到动作截止，保持原下降电压上限。
  mechanisms::lift_controller blocked_down;
  blocked_down.observe(Frame(), true, 0);
  assert(blocked_down.zero_here(true));
  const auto raised = Frame(120, 15);
  blocked_down.observe(raised, true, 10);
  assert(blocked_down.LiftTo(10, 1000));
  for (int now = 20; now < 1010; now += 10) {
    blocked_down.observe(raised, true, now);
    assert(blocked_down.busy() && blocked_down.info().left_mV == -2500);
  }
  blocked_down.observe(raised, true, 1010);
  assert(!std::strcmp(blocked_down.info().status, "TIMEOUT") && blocked_down.info().left_mV == 0);

  // 回零时位置稳定即可完成，即使 V5 速度读数跳动。真实位移须重新计时。
  mechanisms::lift_controller return_to_zero;
  return_to_zero.observe(Frame(), true, 0);
  assert(return_to_zero.zero_here(true));
  return_to_zero.observe(raised, true, 10);
  assert(return_to_zero.LiftTo(0, 4000));
  for (int now = 20; now <= 310; now += 10) {
    auto near_zero = Frame(now < 110 ? 81.5 : 83.0, now % 20 ? -24.8 : -25.4);
    near_zero.right_deg_s = now % 20 ? 16 : -16;
    return_to_zero.observe(near_zero, true, now);
    assert(return_to_zero.info().left_mV == 0);
    if (now < 310)
      assert(return_to_zero.busy());
  }
  assert(return_to_zero.at_target());

  // 检查上升带载电压、双向减速、到位保持和停稳确认。
  mechanisms::lift_controller assisted;
  assisted.observe(Frame(), true, 0);
  assert(assisted.zero_here(true) && assisted.LiftTo(65, 4000));
  assisted.observe(Frame(117.8, 12.8), true, 100);
  assert(assisted.info().left_mV == 6000 && assisted.info().right_mV == 6000);
  auto rising = Frame(142, 37);
  rising.left_deg_s = rising.right_deg_s = 80;
  assisted.observe(rising, true, 200);
  assert(assisted.info().reference_deg_s > 0 && assisted.info().reference_deg_s < 10);
  assert(assisted.info().left_mV == 5000);
  rising.left_deg += 2;
  rising.right_deg += 2;
  assisted.observe(rising, true, 210);
  assert(assisted.info().reference_deg_s == 0 && assisted.info().left_mV == 0 && assisted.busy());
  auto rebound = Frame(143.2, 38.6);
  rebound.left_deg_s = rebound.right_deg_s = -25;
  assisted.observe(rebound, true, 220);
  assert(assisted.info().left_mV == 0 && assisted.busy());  // 短时回弹不能重新上电或提前开爪。
  for (int now = 300; now <= 500; now += 100)
    assisted.observe(Frame(145, 40), true, now);
  assert(assisted.at_target() && assisted.info().left_mV == 0);
  assert(assisted.LiftTo(10, 4000));
  assert(assisted.info().left_mV == -2500 && assisted.info().right_mV == -2500);
  auto falling = Frame(95, -10);
  falling.left_deg_s = falling.right_deg_s = -90;
  assisted.observe(falling, true, 550);
  assert(assisted.info().reference_deg_s < 0 && assisted.info().reference_deg_s > -15);
  assert(assisted.info().left_mV > -2500);
  falling.left_deg -= 4;
  falling.right_deg -= 4;
  assisted.observe(falling, true, 560);
  assert(assisted.info().reference_deg_s == 0 && assisted.info().left_mV == 0 && assisted.busy());
  auto placed = Frame(90, -15);
  placed.right_deg_s = 16;  // 已进入保持后，位置稳定即可确认放料高度。
  for (int now : {600, 670}) {
    assisted.observe(placed, true, now);
    assert(assisted.busy());
  }
  assisted.observe(placed, true, 680);
  assert(assisted.at_target() && assisted.info().left_mV == 0);

  // 50° 抬升复现：速度突刺时保持基础电压，从下方进入内窗口才保持，回落超窗立即补回。
  mechanisms::lift_controller displaced;
  displaced.observe(Frame(), true, 0);
  assert(displaced.zero_here(true) && displaced.LiftTo(50, 4000));
  auto approach_spike = Frame(126.6, 23);
  approach_spike.left_deg_s = approach_spike.right_deg_s = 67.4;
  displaced.observe(approach_spike, true, 100);
  assert(displaced.busy() && displaced.info().left_mV == 5000);
  displaced.observe(Frame(128, 23.6), true, 160);
  displaced.observe(Frame(128, 23.6), true, 200);
  assert(displaced.busy() && displaced.info().left_mV >= 5000);
  const auto acquired = Frame(129.4, 24.4);
  displaced.observe(acquired, true, 300);
  assert(displaced.busy() && displaced.info().left_mV == 0);
  displaced.observe(Frame(126.2, 22.2), true, 320);
  assert(displaced.busy() && displaced.info().left_mV >= 5000);
  displaced.observe(acquired, true, 340);
  auto close_with_noise = Frame(130.8, 24.4);
  close_with_noise.right_deg_s = 16;
  for (int now : {440, 540, 630}) {
    displaced.observe(close_with_noise, true, now);
    assert(displaced.busy() && displaced.info().left_mV == 0);
  }
  displaced.observe(close_with_noise, true, 640);
  assert(displaced.at_target());

  // 单侧还在推进时不启动两侧停滞补力。
  mechanisms::lift_controller one_side;
  one_side.observe(Frame(), true, 0);
  assert(one_side.zero_here(true));
  one_side.observe(Frame(130, 25), true, 10);
  assert(one_side.LiftTo(58, 4000));
  for (int now = 20; now <= 710; now += 10) {
    one_side.observe(Frame(130 + (now - 10) / 80.0, 25), true, now);
    assert(one_side.busy());
    assert(one_side.info().left_mV == static_cast<int>(5000 + 29 * one_side.info().reference_deg_s));
  }

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
