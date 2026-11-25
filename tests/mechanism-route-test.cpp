#include "autonomous.h"
#include <cassert>
#include <cstring>
#include <limits>
#include <cstdio>

// 模拟适配器仅编译进宿主机测试，不接入实车main。
struct simulated : mechanism::controller {
  mechanism::status value;
  double duration = 300, started = 0, failed_after = -1;
  int starts = 0, cancels = 0, updates = 0, voltage = 6000;
  bool reject = false, bad_output = false;
  bool Start(int preset, double /*timeout_ms*/, double now) override {
    ++starts;
    started = now;
    value = mechanism::status();
    value.state = duration == 0 ? drive::motion_state::done : drive::motion_state::running;
    value.reason = "SIM RUN";
    value.command.motor_mV = bad_output ? 12001 : duration == 0 ? 0 : voltage;
    value.command.piston = preset != 0;
    value.command.stop = manual::stopping::hold;
    return !reject;
  }
  void Update(double now) override {
    ++updates;
    if (value.state != drive::motion_state::running)
      return;
    if (failed_after >= 0 && now - started >= failed_after) {
      value.state = drive::motion_state::error;
      value.reason = "SIM JAM";
    } else if (now - started >= duration) {
      value.state = drive::motion_state::done;
      value.reason = "DONE";
      value.command.motor_mV = 0;
    }
  }
  mechanism::status Info() const override {
    return value;
  }
  void Cancel() override {
    ++cancels;
    value.command.motor_mV = 0;
    value.command.stop = manual::stopping::coast;
    if (value.state == drive::motion_state::running)
      value.state = drive::motion_state::cancelled;
  }
};
static int scenario = 0;
void autonomous_program::RunRoutine() {
  using namespace route;
  using mechanism::channel;
  if (scenario == 0)
    RunRoute({StartMechanism(channel::arm, 1, 3000), StartMechanism(channel::piston, 1, 1000),
              DriveStraightWithTrigger(120, 100, 60, channel::claw, 1, 2000, 5000),
              WaitMechanism(channel::arm, 3000), Turn(30, 100, 5000), DriveStraight(-60, 100, 5000)});
  else if (scenario == 1)
    RunRoute({StartMechanism(channel::arm, 1, 100), DriveStraight(500, 100, 5000)});
  else if (scenario == 2)
    RunRoute({StartMechanism(channel::arm, 1, 1000), StartMechanism(channel::arm, 2, 1000)});
  else if (scenario == 3)
    RunRoute({WaitMechanism(channel::arm, 1000), DriveStraight(100, 100, 5000)});
  else if (scenario == 4 || scenario == 5)
    RunRoute({StartMechanism(channel::arm, 1, 200)});  // 路线末端自动等待。
  else if (scenario == 6)
    RunRoute({DriveStraightWithTrigger(120, 100, 60, channel::claw, 1, 1000, 5000)});
  else if (scenario == 7)
    RunRoute({DriveStraightWithTrigger(0, 100, 0, channel::claw, 1, 1000, 5000)});
  else if (scenario == 8)
    RunRoute({DriveStraightWithTrigger(120, 100, 130, channel::claw, 1, 1000, 5000)});
  else if (scenario == 9)
    RunRoute({StartMechanism(channel::arm, 1, std::numeric_limits<double>::quiet_NaN())});
  else if (scenario == 10)
    RunRoute({StartMechanism(channel::arm, 1, 1000), WaitMechanism(channel::arm, 40),
              DriveStraight(100, 100, 5000)});
  else if (scenario == 13)
    RunRoute({StartMechanism(channel::arm, 1, 1000), StartMechanism(channel::arm, 2, 1000)});
  else if (scenario == 14)
    RunRoute({DriveStraightWithTrigger(-120, 100, 60, channel::claw, 1, 1000, 5000)});
  else if (scenario == 15)
    RunRoute({DriveStraightWithTrigger(120, 100, 120, channel::claw, 1, 1000, 5000)});
  else if (scenario == 16)
    RunRoute({DriveStraightWithTrigger(120, 100, 120, channel::claw, 1, 1000, 5000)});
  else
    RunRoute({StartMechanism(channel::arm, 1, 5000), DriveStraight(500, 100, 5000)});
}
struct fixture {
  autonomous_program program;
  simulated arm, claw, piston;
  drive::straight_feedback frame;
  manual::output output;
  double left = 0, right = 0;
  int now = 0;
  bool overlapped = false, trigger_seen = false;
  fixture(bool bind = true) {
    drive::straight_config s;
    s.motion.wheel_diameter_mm = 69.85;
    s.motion.motor_revs_per_wheel_rev = 1.2;
    s.left_forward.kS = 400;
    s.left_forward.kV = 16;
    s.left_forward.calibrated = true;
    s.right_forward = s.left_reverse = s.right_reverse = s.left_forward;
    drive::turn_config t;
    t.left_forward = t.right_forward = t.left_reverse = t.right_reverse = s.left_forward;
    program.Configure(s, t);
    frame.motors_online = frame.inertial_ready = frame.position.valid = frame.velocity.valid = frame.stopped =
        true;
    piston.voltage = 0;
    piston.duration = 0;
    if (bind) {
      assert(program.BindMechanism(mechanism::channel::arm, arm));
      assert(program.BindMechanism(mechanism::channel::claw, claw));
      assert(program.BindMechanism(mechanism::channel::piston, piston));
    }
  }
  void Step(bool permitted = true) {
    auto rpm = [](int mv) { return std::copysign(std::max(0., std::abs(mv) - 400.) / 16, mv); };
    left += (rpm(output.left_mV) - left) * .25;
    right += (rpm(output.right_mV) - right) * .25;
    frame.position.left_deg += left * .12;
    frame.position.right_deg += right * .12;
    frame.heading_deg += (right - left) * 6 * 69.85 / 1.2 / 325 * .02;
    now += 20;
    frame.velocity.time_ms = now;
    ++frame.velocity.sequence;
    frame.velocity.left_rpm = left;
    frame.velocity.right_rpm = right;
    frame.stopped = std::fabs(left) < 5 && std::fabs(right) < 5;
    const int old_starts = claw.starts;
    program.Update(permitted, frame, now);
    output = manual::output();
    output.left_mV = program.Info().left_mV;
    output.right_mV = program.Info().right_mV;
    program.ApplyMechanisms(output);
    if (old_starts == 0 && claw.starts) {
      trigger_seen = true;
      assert(std::fabs(program.Info().drive.motion.distance_mm) >= 59 || scenario == 7);
    }
    overlapped = overlapped || (output.arm_mV && (output.left_mV || output.right_mV));
  }
  void Run() {
    Step();
    for (int i = 0; i < 1000 && program.Info().active; ++i)
      Step();
    assert(program.Info().finished && !program.Info().active);
    assert(output.left_mV == 0 && output.right_mV == 0 && output.arm_mV == 0 && output.claw_mV == 0);
  }
};
int main() {
  for (double duration : {100., 2400.}) {
    scenario = 0;
    fixture f;
    f.arm.duration = duration;
    f.Run();
    std::fprintf(stderr, "parallel duration %.0f: %s step %u time %d overlap %d trigger %d\n", duration,
                 f.program.Info().status, f.program.Info().step, f.now, f.overlapped, f.trigger_seen);
    assert(f.program.Info().result == route_result::done && f.overlapped && f.trigger_seen);
    assert(f.program.Info().step == 6 && f.arm.starts == 1 && f.claw.starts == 1 && f.output.toggle_piston);
    assert(f.output.arm_stop == manual::stopping::hold && f.output.claw_stop == manual::stopping::hold);
    // 模拟主循环：自动气动状态交回手动后保持，X下一次新按仍可切换。
    manual::driver_control driver(false);
    driver.SetPistonState(f.output.toggle_piston);
    manual::input input;
    assert(driver.Update(input, true, 5, 6000, 12000, 4000, 6000).toggle_piston);
    input.toggle = true;
    assert(!driver.Update(input, true, 5, 6000, 12000, 4000, 6000).toggle_piston);
  }
  for (int kind : {1, 2, 3, 5, 6, 8, 9, 10}) {
    scenario = kind;
    fixture f(kind != 5 && kind != 6);
    f.arm.duration = 1000;
    f.Run();
    assert(f.program.Info().result == route_result::error);
    if (kind == 1)
      assert(!std::strcmp(f.program.Info().status, "MECH TIMEOUT"));
    if (kind == 2)
      assert(!std::strcmp(f.program.Info().status, "MECH BUSY"));
    if (kind == 3)
      assert(!std::strcmp(f.program.Info().status, "MECH NOT STARTED"));
    if (kind == 5)
      assert(!std::strcmp(f.program.Info().status, "MECH NOT BOUND"));
    if (kind == 10)
      assert(!std::strcmp(f.program.Info().status, "TIMEOUT"));
  }
  for (double duration : {100., 200.}) {
    scenario = 4;
    fixture f;
    f.arm.duration = duration;
    f.Run();
    assert(f.program.Info().result == route_result::done && f.now == 20 + duration);
  }
  for (int kind : {7, 13, 14, 16}) {
    scenario = kind;
    fixture f;
    if (kind == 13)
      f.arm.duration = 0;
    f.Run();
    assert(f.program.Info().result == route_result::done);
    if (kind == 13)
      assert(f.arm.starts == 2);
    else
      assert(f.claw.starts == 1);
  }
  for (int interruption = 0; interruption < 5; ++interruption) {
    scenario = interruption == 4 ? 15 : 11;
    fixture f;
    f.arm.duration = 3000;
    if (interruption == 2)
      f.arm.failed_after = 40;
    f.Step();
    f.Step();
    if (interruption == 0)
      f.program.CancelRoutine();
    else if (interruption == 1)
      f.Step(false);
    else if (interruption == 2)
      f.Step();
    else if (interruption == 3) {
      f.frame.motors_online = false;
      f.Step();
    } else {
      f.program.CancelRoutine();
      assert(f.claw.starts == 0);
    }
    assert(f.program.Info().finished && f.program.Info().result != route_result::done);
    manual::output out;
    f.program.ApplyMechanisms(out, true);
    assert(out.arm_mV == 0 && out.claw_mV == 0 && f.program.Info().left_mV == 0);
    if (interruption != 4)
      assert(out.arm_stop == manual::stopping::coast);
    out.arm_mV = 12000;
    f.program.ApplyMechanisms(out, true);
    assert(out.arm_mV == 12000);  // 新手控指令优先，不被取消输出覆盖。
  }
  mechanism::scheduler bus;
  simulated arm;
  assert(bus.Bind(mechanism::channel::arm, arm));
  assert(!bus.Bind(mechanism::channel::claw, arm));
  assert(!bus.Start(static_cast<mechanism::channel>(9), 1, 100, 0));
  bus.Reset(0);
  arm.bad_output = true;
  assert(!bus.Start(mechanism::channel::arm, 1, 100, 0));
  manual::output out;
  bus.Apply(out);
  assert(out.arm_mV == 0 && !std::strcmp(bus.Info().fault, "MECH OUTPUT"));
  arm.bad_output = false;
  bus.Reset(0);
  assert(bus.Start(mechanism::channel::arm, 1, 100, 0));
  assert(!bus.Update(-1));
  bus.Apply(out);
  assert(out.arm_mV == 0);
  // 启动动作也必须推进时钟基准，不能在100ms启动后接受50ms更新。
  bus.Reset(0);
  assert(bus.Start(mechanism::channel::arm, 1, 1000, 100));
  assert(!bus.Update(50));
  assert(!std::strcmp(bus.Info().fault, "MECH CLOCK"));
  bus.Apply(out);
  assert(out.arm_mV == 0);
  // 允许从快照重放已完成请求；清理旧动作不能覆盖传入的同一个引用。
  bus.Reset(0);
  arm.duration = 0;
  mechanism::request replay;
  replay.preset = 2;
  assert(bus.Start(mechanism::channel::arm, replay, 1000, 10));
  const auto& saved = bus.Info().channels[0].action;
  assert(bus.Start(mechanism::channel::arm, saved, 1000, 20));
  assert(bus.Info().channels[0].preset == 2 && bus.Info().channels[0].action.preset == 2);
  bus.Reset(std::numeric_limits<double>::quiet_NaN());
  assert(bus.Info().fault && !std::strcmp(bus.Info().fault, "MECH CLOCK"));
  std::puts(
      "Mechanism routes passed: overlap, signed distance triggers, joins, independent deadlines, ownership, "
      "faults, cancellation and manual handoff.");
}
