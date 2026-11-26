#include "startup-mode.h"
#include <cassert>
#include <cstdio>
#include <limits>
#include <cstring>

struct fixture {
  runtime::startup_mode Selection;
  manual::driver_control Driver{false};
  manual::input input;
  manual::output output;
  runtime::permissions access;
  bool available = true, field = false, sticks = false, stopped = true, ready = true, finished = false;
  double now = 0;
  void Step() {
    now += 20;
    Selection.Update(input, available, field, sticks, stopped, ready, finished, now);
    access = runtime::SessionPermissions(Selection, available, field, false, false, false, available,
                                         runtime::competition_stage::none);
    output = Driver.Update(input, access.manual, 5, 12000, 12000, 4000, 6000);
    assert(!(access.manual && access.autonomous));
  }
  void Choose(int key) {
    input = manual::input();
    Step();
    input.test_start = key == 1;
    input.test_cancel = key == 2;
    input.toggle = key == 4;
    Step();
  }
  void Release() {
    input = manual::input();
    Step();
  }
};
int main() {
  fixture teaching;
  teaching.Step();
  teaching.input.test_next = true;
  teaching.Step();
  assert(teaching.Selection.Mode() == runtime::local_mode::teach && !teaching.access.manual);
  teaching.Release();
  assert(teaching.access.manual && !teaching.access.tests && !teaching.access.telemetry);
  teaching.field = true;
  teaching.Step();
  assert(teaching.Selection.Mode() == runtime::local_mode::select && !teaching.access.manual);
  runtime::teach_recorder recorder;
  manual::input keys;
  drive::pose measured;
  measured.valid = true;
  measured.x_mm = -120; measured.y_mm = 80; measured.heading_deg = 93;
  keys.test_start = true;
  assert(recorder.Update(true, keys, measured, 0).count == 1);
  assert(recorder.Update(true, keys, measured, 20).count == 1);
  keys.test_start = false;
  recorder.Update(false, keys, measured, 200);
  keys.test_start = true;
  assert(recorder.Update(true, keys, measured, 300).count == 1);  // 旧位姿拒绝记点。
  keys.test_start = false;
  keys.test_positive = true;
  const auto first = recorder.Update(true, keys, measured, 320).packet;
  assert(first.total == 1 && first.index == 1 && first.point.x_mm == -120 && first.point.heading_deg == 93);
  keys.test_positive = false;
  recorder.Update(true, keys, measured, 330);
  keys.test_start = true;
  measured.time_ms = 340;
  measured.x_mm = 310;
  assert(recorder.Update(true, keys, measured, 340).count == 2);
  keys.test_start = false;
  keys.test_positive = true;
  const auto replay = recorder.Update(true, keys, measured, 350).packet;
  assert(replay.index == 1 && replay.total == 2 && replay.batch != first.batch);
  assert(recorder.Update(true, keys, measured, 840).packet.index == 1);
  const auto last = recorder.Update(true, keys, measured, 850).packet;
  assert(last.index == 2 && last.point.x_mm == 310);
  assert(recorder.Update(false, keys, measured, 860).packet.index == 0);
  keys = manual::input();
  measured.needs_reset = true;
  keys.test_start = true;
  measured.time_ms = 900;
  assert(recorder.Update(true, keys, measured, 900).count == 2);
  measured.needs_reset = false;
  for (unsigned i = 2; i <= runtime::teach_recorder::capacity; ++i) {
    measured.time_ms += 20;
    keys.test_start = false;
    recorder.Update(true, keys, measured, measured.time_ms);
    keys.test_start = true;
    const auto& stored = recorder.Update(true, keys, measured, measured.time_ms + 10);
    assert(stored.count == (i < runtime::teach_recorder::capacity ? i + 1 : i));
  }
  // 模拟掉线后的定位锁存：反馈恢复也不能显示自动就绪；X仍可选择。
  drive::straight_feedback frame;
  frame.motors_online = frame.inertial_ready = frame.position.valid = frame.velocity.valid = frame.stopped =
      true;
  drive::pose pose;
  pose.valid = true;
  assert(runtime::AutomaticStartStatus(frame, pose, 0) == nullptr);
  frame.motors_online = frame.velocity.valid = frame.stopped = false;
  assert(!std::strcmp(runtime::AutomaticStartStatus(frame, pose, 20), "MOTOR OFF"));
  frame.motors_online = frame.velocity.valid = frame.stopped = true;
  pose.valid = false;
  pose.needs_reset = true;
  const char* status = runtime::AutomaticStartStatus(frame, pose, 40);
  assert(!std::strcmp(status, "POSE RESET:RESTART"));
  fixture locked;
  locked.ready = status == nullptr;
  locked.Choose(1);
  assert(locked.Selection.Mode() == runtime::local_mode::select);
  locked.Release();
  locked.Choose(4);
  locked.Release();
  assert(locked.access.tests && !locked.access.autonomous);
  pose.needs_reset = false;
  assert(!std::strcmp(runtime::AutomaticStartStatus(frame, pose, 40), "POSE WAIT"));
  pose.valid = true;
  assert(runtime::AutomaticStartStatus(frame, pose, 200) == nullptr);
  assert(!std::strcmp(runtime::AutomaticStartStatus(frame, pose, 201), "POSE WAIT"));
  assert(!std::strcmp(runtime::AutomaticStartStatus(frame, pose, -1), "POSE WAIT"));
  pose.x_mm = std::numeric_limits<double>::quiet_NaN();
  assert(!std::strcmp(runtime::AutomaticStartStatus(frame, pose, 40), "POSE WAIT"));
  using runtime::local_mode;
  for (int mask = 1; mask < 8; ++mask) {
    fixture held;
    held.input.test_start = mask & 1;
    held.input.test_cancel = mask & 2;
    held.input.toggle = mask & 4;
    held.input.arm_up = held.input.claw_close = true;
    for (int i = 0; i < 20; ++i) {
      held.Step();
      assert(held.Selection.Mode() == local_mode::select);
      assert(!held.access.manual && !held.access.tests && !held.access.autonomous);
      assert(!held.output.arm_enabled && held.output.claw_mV == 0 && !held.output.toggle_piston);
    }
    held.available = false;
    held.Step();
    held.available = true;
    held.Step();
    assert(held.Selection.Mode() == local_mode::select &&
           !held.access.manual);  // 初始化/重连后持键不跳过菜单。
  }
  fixture manual;
  manual.Choose(2);
  assert(manual.Selection.Mode() == local_mode::manual && !manual.access.manual);
  manual.Release();
  assert(manual.access.manual && !manual.access.tests && !manual.access.autonomous);
  fixture test;
  test.Choose(4);
  for (int i = 0; i < 20; ++i) {
    test.Step();
    assert(!test.access.tests && !test.output.toggle_piston);
  }
  test.Release();
  assert(test.access.tests && test.access.manual && !test.output.toggle_piston);
  test.input.toggle = true;
  test.Step();
  assert(test.output.toggle_piston);
  test.Step();
  assert(test.output.toggle_piston);
  test.input.toggle = false;
  test.input.test_cancel = true;
  test.Step();
  for (int i = 0; i < 49; ++i)
    test.Step();
  assert(test.Selection.Mode() == local_mode::test);
  test.Step();
  assert(test.Selection.Mode() == local_mode::select && !test.access.manual);
  test.Step();
  assert(test.Selection.Mode() == local_mode::select);
  fixture auto_run;
  auto_run.finished = true;
  auto_run.Choose(1);
  auto_run.Step();
  assert(!auto_run.access.autonomous);
  auto_run.Release();
  assert(auto_run.access.autonomous);  // 上轮finished不跳过新一轮。
  auto_run.finished = false;
  auto_run.Step();
  assert(auto_run.access.autonomous);
  auto_run.finished = true;
  auto_run.Step();
  assert(auto_run.Selection.ReturningToManual() && auto_run.Selection.ReturnRemainingMs() == 3000);
  for (int i = 0; i < 149; ++i) {
    auto_run.Step();
    assert(auto_run.access.autonomous && !auto_run.access.manual);
  }
  auto_run.Step();
  assert(auto_run.Selection.Mode() == local_mode::manual && auto_run.access.manual);
  for (int i = 0; i < 10; ++i)
    auto_run.Step();
  assert(!auto_run.access.autonomous);
  fixture cooldown_cancel;
  cooldown_cancel.Choose(1);
  cooldown_cancel.Release();
  cooldown_cancel.finished = true;
  cooldown_cancel.Step();
  assert(cooldown_cancel.Selection.ReturningToManual());
  cooldown_cancel.input.test_cancel = true;
  cooldown_cancel.Step();
  assert(cooldown_cancel.Selection.Mode() == local_mode::manual &&
         !cooldown_cancel.Selection.ReturningToManual());
  fixture early_cancel;
  early_cancel.Choose(1);
  early_cancel.input.test_cancel = true;
  early_cancel.Step();
  assert(early_cancel.Selection.Mode() == local_mode::manual && !early_cancel.access.autonomous);
  early_cancel.Release();
  assert(early_cancel.access.manual);
  fixture takeover;
  takeover.Choose(1);
  takeover.Release();
  takeover.sticks = true;
  takeover.input.y_axis = 50;
  takeover.Step();
  assert(takeover.access.manual && !takeover.access.autonomous && takeover.output.left_mV > 0);
  fixture not_ready;
  not_ready.ready = false;
  not_ready.Choose(1);
  assert(not_ready.Selection.Mode() == local_mode::select);
  not_ready.ready = true;
  not_ready.Step();
  assert(not_ready.Selection.Mode() == local_mode::select);
  not_ready.Release();
  not_ready.input.test_start = true;
  not_ready.Step();
  not_ready.Release();
  assert(not_ready.access.autonomous);
  fixture disconnect;
  disconnect.Choose(2);
  disconnect.Release();
  disconnect.available = false;
  disconnect.Step();
  assert(disconnect.Selection.Mode() == local_mode::select && !disconnect.access.manual);
  disconnect.available = true;
  disconnect.input.test_start = true;
  disconnect.Step();
  assert(disconnect.Selection.Mode() == local_mode::select);
  for (int stage = 0; stage < 3; ++stage)
    for (int bits = 0; bits < 64; ++bits) {
      runtime::startup_mode selection;
      const bool cfg = bits & 1, en = bits & 2, driver = bits & 4, automatic = bits & 8,
                 controller = bits & 16;
      const auto expected =
          runtime::Permissions(runtime::program_mode::competition, cfg, true, en, driver, automatic,
                               controller, static_cast<runtime::competition_stage>(stage));
      const auto actual = runtime::SessionPermissions(selection, cfg, true, en, driver, automatic, controller,
                                                      static_cast<runtime::competition_stage>(stage));
      assert(actual.manual == expected.manual && actual.autonomous == expected.autonomous);
      assert(!actual.tests && !actual.telemetry && !actual.calibration);
    }
  fixture field;
  field.Choose(1);
  field.Release();
  field.field = true;
  field.Step();
  assert(field.Selection.Mode() == local_mode::select && !field.access.autonomous);
  field.field = false;
  field.input.test_start = true;
  field.Step();
  assert(field.Selection.Mode() == local_mode::select);
  fixture broken;
  broken.Choose(2);
  broken.Release();
  broken.now = -40;
  broken.Step();
  assert(!broken.access.manual && broken.Selection.Mode() == local_mode::select);
  broken.now = std::numeric_limits<double>::quiet_NaN();
  broken.Step();
  assert(!broken.access.manual);
  std::puts(
      "Startup mode checks passed: Tungsten A-auto/B-manual/X-test, release gates, piston isolation, auto "
      "return, long-B menu, takeover, disconnect and field authority.");
}
