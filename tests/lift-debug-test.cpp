#include "mechanisms.h"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <limits>
#include <type_traits>
#include <utility>

template <class T> struct accepts_lift_without_timeout {
  template <class U> static auto test(int)
      -> decltype(std::declval<U&>().LiftTo(15), std::true_type());
  template <class> static std::false_type test(...);
  static constexpr bool value = decltype(test<T>(0))::value;
};
static_assert(!accepts_lift_without_timeout<mechanisms::lift_controller>::value,
              "LiftTo requires an explicit timeout");
static_assert(!accepts_lift_without_timeout<mechanisms::lift_request>::value,
              "Queued LiftTo requires an explicit timeout");
template <class T> struct accepts_height_without_timeout {
  template <class U> static auto test(int)
      -> decltype(std::declval<U&>().LiftHeightTo(100), std::true_type());
  template <class> static std::false_type test(...);
  static constexpr bool value = decltype(test<T>(0))::value;
};
static_assert(!accepts_height_without_timeout<mechanisms::lift_controller>::value,
              "LiftHeightTo requires an explicit timeout");
static_assert(!accepts_height_without_timeout<mechanisms::lift_request>::value,
              "Queued LiftHeightTo requires an explicit timeout");

mechanisms::lift_config PDConfig() {
  mechanisms::lift_config settings;
  settings.kI = 0;  // 既有助推、共轴与保护断言继续对照原 PD 行为。
  return settings;
}

struct fixture {
  mechanisms::lift_controller lift;
  mechanisms::feedback frame;
  double now = 0;
  explicit fixture(const mechanisms::lift_config& settings = PDConfig()) {
    lift.configure(settings);
    frame.arm_online = frame.arm_valid = true;
    Observe();
  }
  void Observe(bool allowed = true) { lift.observe(frame, allowed, now); }
  void Tick(bool allowed = true) { now += 20; Observe(allowed); }
};

mechanisms::lift_config PIDConfig() {
  mechanisms::lift_config settings;
  settings.kI = 20;
  settings.start_up_ms = 0;
  settings.stall_ms = 30000;  // 静态反馈专用于观测积分，不提前触发独立的堵转测试。
  return settings;
}

struct pid_fixture : fixture {
  explicit pid_fixture(const mechanisms::lift_config& settings = PIDConfig()) : fixture(settings) {}
};

mechanisms::lift_request CurrentLiftDebug() {
  mechanisms::lift_request request;
  assert(mechanisms::StartLiftDebug(request));
  assert(request.kind == mechanisms::lift_command_kind::angle);
  assert(std::isfinite(request.target) && std::isfinite(request.timeout_ms) && request.timeout_ms > 0);
  return request;
}

void CheckDebugEntry() {
  // 调参数字只来自真实入口；检查请求和控制器一致，不重复抄写用户的目标与超时。
  const mechanisms::lift_request request = CurrentLiftDebug();
  fixture debug{mechanisms::lift_config()};
  assert(debug.lift.zero_here() && mechanisms::StartLiftDebug(debug.lift));
  assert(debug.lift.busy() && debug.lift.info().target_deg == request.target);
  const mechanisms::lift_info before = debug.lift.info();
  assert(!mechanisms::StartLiftDebug(debug.lift));
  assert(debug.lift.info().target_deg == before.target_deg && debug.lift.info().result == before.result);
  assert(debug.lift.info().elapsed_ms == before.elapsed_ms);
  assert(debug.lift.info().left_mV == before.left_mV && debug.lift.info().right_mV == before.right_mV);
}

void CheckStartupZero() {
  mechanisms::lift_controller startup;
  assert(!startup.zero_here(true));  // 不能在首个有效反馈之前伪造零点。
  mechanisms::feedback frame;
  frame.arm_online = frame.arm_valid = true;
  frame.left_deg = -314.5; frame.right_deg = 826;
  frame.left_deg_s = 20; frame.right_deg_s = -25;
  startup.observe(frame, false, 0);
  assert(!startup.zero_here());  // 普通调零仍要求权限和停稳。
  assert(startup.zero_here(true));  // 上电参考不依赖菜单使能或编码器原始绝对值。
  assert(startup.info().homed && startup.info().position_deg() == 0);
  assert(startup.info().left_mV == 0 && startup.info().right_mV == 0);
  frame.left_deg += 20; frame.right_deg += 20;
  frame.left_deg_s = frame.right_deg_s = 0;
  startup.observe(frame, true, 20);
  assert(startup.info().left_deg == 20 && startup.info().right_deg == 20);
  assert(startup.LiftTo(60, 3000));  // 后续角度相对于本次捕获的两台原始值。
  startup.cancel();
  frame.arm_online = false; startup.observe(frame, true, 40);
  assert(!startup.info().homed && !startup.zero_here(true));
  frame.arm_online = true; startup.observe(frame, true, 60);
  assert(!startup.info().homed && !startup.LiftTo(60, 3000));
  assert(!std::strcmp(startup.info().status, "NO ZERO"));  // 重连本身不会重新定义零点。

  fixture moving;
  moving.frame.left_deg_s = moving.frame.right_deg_s = 20; moving.Tick();
  assert(!moving.lift.zero_here());
  assert(moving.lift.zero_here(true));

  for (int defect = 0; defect < 4; ++defect) {
    mechanisms::lift_controller invalid;
    mechanisms::feedback bad;
    bad.arm_online = bad.arm_valid = true;
    if (defect == 0) bad.arm_online = false;
    if (defect == 1) bad.arm_valid = false;
    if (defect == 2) bad.left_deg = std::numeric_limits<double>::quiet_NaN();
    if (defect == 3) bad.right_deg_s = std::numeric_limits<double>::quiet_NaN();
    invalid.observe(bad, false, 0);
    assert(!invalid.zero_here(true) && !invalid.info().homed);
    assert(invalid.info().left_mV == 0 && invalid.info().right_mV == 0);
  }
}

void CheckPressEdge() {
  mechanisms::press_edge edge;
  assert(!edge.update(true, true));  // 开机已按住不触发。
  assert(!edge.update(true, true));
  assert(!edge.update(false, true));
  assert(edge.update(true, true));
  assert(!edge.update(true, true));  // 一次按下只发一次命令。
  assert(!edge.update(false, true));
  assert(edge.update(true, true));
  assert(!edge.update(true, false));
  assert(!edge.update(true, true));  // 失权期间按住，恢复后也不能触发。
  assert(!edge.update(false, true));
  assert(edge.update(true, true));
  assert(!edge.update(false, false));
  assert(!edge.update(true, true));  // 恢复的首个采样已按下，仍须松开再按。
  assert(!edge.update(false, true));
  assert(edge.update(true, true));
}

void CheckPID() {
  pid_fixture timed;
  assert(timed.lift.zero_here() && timed.lift.LiftTo(10, 30000));
  assert(timed.lift.info().left_mV == 1000);  // 首次 dt=0，不补算此前空闲时间。
  timed.Tick();
  assert(timed.lift.info().left_mV == 1004);  // 20 * 10 * 0.020 = 4mV。
  timed.Observe();
  assert(timed.lift.info().left_mV == 1004);  // 同一时刻重复观察不能重复积分。
  timed.now += 40; timed.Observe();
  assert(timed.lift.info().left_mV == 1012);  // 按秒积分，40ms 新增 8mV。

  pid_fixture capped;
  assert(capped.lift.zero_here() && capped.lift.LiftTo(10, 30000));
  for (int i = 0; i < 600; ++i) capped.Tick();
  assert(capped.lift.busy() && capped.lift.info().left_mV == 3000);  // P1000 + I上限2000。
  mechanisms::lift_config symmetric = PIDConfig();
  symmetric.maximum_down_mV = 12000;
  mechanisms::lift_pid integral;
  for (int i = 0; i < 600; ++i) integral.update(-10, 0, 20, symmetric, true);
  assert(std::fabs(integral.update(-10, 0, 0, symmetric, true) + 3000) < 1e-9);
  integral.reset();
  assert(integral.update(-10, 0, 0, symmetric, true) == -1000);
  integral.reset();
  for (int i = 0; i < 10; ++i) integral.update(10, 0, 20, symmetric, true);
  integral.update(0, 20, 20, symmetric, true);  // 穿过零误差但仍运动，不能依赖 settling 清积分。
  assert(integral.update(-10, 0, 20, symmetric, true) == -1004);

  pid_fixture saturated;
  assert(saturated.lift.zero_here() && saturated.lift.LiftTo(150, 30000));
  for (int i = 0; i < 10; ++i) saturated.Tick();
  assert(saturated.lift.info().left_mV == 12000);
  saturated.frame.left_deg = saturated.frame.right_deg = 60;
  saturated.Tick();
  assert(saturated.lift.info().left_mV == 9036);  // 饱和期间未积存；仅本次90度误差新增36mV。

  pid_fixture down;
  assert(down.lift.zero_here());
  down.frame.left_deg = down.frame.right_deg = 100; down.Tick();
  assert(down.lift.LiftTo(0, 30000));
  for (int i = 0; i < 10; ++i) down.Tick();
  assert(down.lift.info().left_mV == -2500);
  down.frame.left_deg = down.frame.right_deg = 20; down.Tick();
  assert(down.lift.info().left_mV == -2008);  // 下降饱和同样不能 windup。

  pid_fixture reversed;
  assert(reversed.lift.zero_here());
  reversed.frame.left_deg = reversed.frame.right_deg = 20; reversed.Tick();
  assert(reversed.lift.LiftTo(30, 30000));
  for (int i = 0; i < 10; ++i) reversed.Tick();
  assert(reversed.lift.info().left_mV == 1040);
  reversed.frame.left_deg = reversed.frame.right_deg = 40; reversed.Tick();
  assert(reversed.lift.info().left_mV == -1004);  // 误差换向清除旧正积分。
  reversed.frame.left_deg = reversed.frame.right_deg = 20; reversed.Tick();
  assert(reversed.lift.info().left_mV == 1004);

  mechanisms::lift_config boundary_settings = PIDConfig();
  boundary_settings.maximum_deg = 150;  // 保留旧150度边界冻结积分的回归。
  pid_fixture boundary(boundary_settings);
  assert(boundary.lift.zero_here());
  boundary.frame.left_deg = 150; boundary.frame.right_deg = 140; boundary.Tick();
  assert(boundary.lift.LiftTo(150, 30000));
  for (int i = 0; i < 10; ++i) boundary.Tick();
  assert(boundary.lift.info().left_mV == 0 && boundary.lift.info().right_mV == 0);
  boundary.frame.left_deg = 149; boundary.frame.right_deg = 139; boundary.Tick();
  assert(boundary.lift.info().left_mV == 602);  // 边界冻结积分；离开后仅新增2.4mV。

  pid_fixture reset;
  assert(reset.lift.zero_here() && reset.lift.LiftTo(10, 30000));
  for (int i = 0; i < 10; ++i) reset.Tick();
  assert(reset.lift.info().left_mV == 1040);
  reset.lift.cancel();
  assert(reset.lift.info().left_mV == 0 && reset.lift.info().result == mechanisms::state::cancelled);
  assert(reset.lift.LiftTo(10, 30000) && reset.lift.info().left_mV == 1000);
  reset.Tick();
  assert(reset.lift.info().left_mV == 1004);
  reset.lift.fault("TEST FAULT");
  assert(reset.lift.info().left_mV == 0 && reset.lift.info().result == mechanisms::state::error);
  assert(reset.lift.LiftTo(10, 30000) && reset.lift.info().left_mV == 1000);
  reset.Tick();
  reset.frame.left_deg = reset.frame.right_deg = 10; reset.Tick();
  for (int i = 0; i < 10; ++i) reset.Tick();
  assert(reset.lift.at_target() && reset.lift.info().left_mV == 0);
  reset.frame.left_deg = reset.frame.right_deg = 0; reset.Tick();
  assert(reset.lift.LiftTo(10, 30000) && reset.lift.info().left_mV == 1000);

  mechanisms::lift_config boosted_settings = PIDConfig();
  boosted_settings.start_up_ms = 100;
  pid_fixture boosted(boosted_settings);
  assert(boosted.lift.zero_here() && boosted.lift.LiftTo(10, 30000));
  for (int i = 0; i < 4; ++i) {
    boosted.Tick();
    assert(boosted.lift.info().left_mV == 12000);
  }
  boosted.Tick();
  assert(boosted.lift.info().left_mV == 1004);  // 助推期间不累积积分。

  for (int defect = 0; defect < 6; ++defect) {
    mechanisms::lift_config invalid_settings = PIDConfig();
    if (defect == 0) invalid_settings.kI = -1;
    if (defect == 1) invalid_settings.kI = std::numeric_limits<double>::quiet_NaN();
    if (defect == 2) invalid_settings.kI = std::numeric_limits<double>::infinity();
    if (defect == 3) invalid_settings.maximum_integral_mV = -1;
    if (defect == 4) invalid_settings.maximum_integral_mV = 12001;
    if (defect == 5) invalid_settings.maximum_integral_mV = std::numeric_limits<double>::infinity();
    pid_fixture invalid(invalid_settings);
    assert(!invalid.lift.LiftTo(10, 30000));
    assert(!std::strcmp(invalid.lift.info().status, "CONFIG"));
    assert(invalid.lift.info().left_mV == 0 && invalid.lift.info().right_mV == 0);
  }
}

void CheckPanel() {
  using mechanisms::debug_action;
  mechanisms::debug_panel panel;
  // 开机预按及持续按住都不能误启动。
  assert(panel.Update(true, 160, 208, true) == debug_action::none);
  assert(panel.Update(true, 160, 208, true) == debug_action::none);
  assert(panel.Update(false, 160, 208, true) == debug_action::none);
  assert(panel.Update(true, 160, 208, true) == debug_action::start);
  assert(panel.Update(true, 0, 208, true) == debug_action::none);
  assert(panel.Update(true, 320, 208, true) == debug_action::none);
  panel.Update(false, 0, 0, true);

  const int xs[] = {-1, 0, 159, 160, 319, 320, 479, 480};
  const int ys[] = {207, 208, 239, 240};
  for (int y : ys) for (int x : xs) {
    debug_action expected = debug_action::none;
    if (x >= 0 && x < 480 && y >= 208 && y < 240)
      expected = x < 160 ? debug_action::zero : x < 320 ? debug_action::start : debug_action::cancel;
    assert(panel.Update(true, x, y, true) == expected);
    assert(panel.Update(true, x, y, true) == debug_action::none);
    panel.Update(false, x, y, true);
  }
  // 在按钮外按下后拖入，不构成新的点击。
  assert(panel.Update(true, 500, 208, true) == debug_action::none);
  assert(panel.Update(true, 160, 208, true) == debug_action::none);
  panel.Update(false, 160, 208, true);
  // 失权后按住，再恢复权限，仍须松开并重新按下。
  assert(panel.Update(true, 160, 208, false) == debug_action::none);
  assert(panel.Update(true, 160, 208, true) == debug_action::none);
  panel.Update(false, 160, 208, false);
  assert(panel.Update(true, 320, 239, true) == debug_action::cancel);
  assert(panel.Update(true, 320, 239, false) == debug_action::none);
  assert(panel.Update(true, 320, 239, true) == debug_action::none);
}

void CheckStartup() {
  fixture timed;
  assert(timed.lift.zero_here() && timed.lift.LiftTo(15, 3000));
  assert(timed.lift.info().left_mV == 12000 && timed.lift.info().right_mV == 12000);
  assert(!std::strcmp(timed.lift.info().status, "STARTUP"));
  for (int i = 0; i < 4; ++i) {
    timed.Tick();
    assert(timed.lift.info().left_mV == 12000 && timed.lift.info().right_mV == 12000);
  }
  timed.Tick();  // 100ms 边界当周期结束助推，不等待下个周期。
  assert(timed.now == 100 && timed.lift.info().left_mV == 1500 && timed.lift.info().right_mV == 1500);
  timed.Tick();
  assert(timed.lift.info().left_mV == 1500);  // 没运动也不能反复助推。

  fixture rising;
  assert(rising.lift.zero_here() && rising.lift.LiftTo(150, 3000));
  rising.frame.left_deg = rising.frame.right_deg = 1;
  rising.Tick();
  assert(!std::strcmp(rising.lift.info().status, "MOVING"));
  assert(rising.lift.info().left_mV == 12000 && rising.lift.info().right_mV == 12000);
  rising.frame.left_deg = rising.frame.right_deg = 50;
  rising.Tick();  // 退出助推后允许 12V，但误差缩小时仍须按闭环减小输出。
  assert(rising.lift.info().left_mV == 10000 && rising.lift.info().right_mV == 10000);

  fixture moved;
  assert(moved.lift.zero_here() && moved.lift.LiftTo(15, 3000));
  moved.frame.left_deg = 1; moved.frame.right_deg = 0.5;
  moved.Tick();  // 任一侧达到 1 度即停止两侧助推。
  assert(moved.lift.info().left_mV == 1425 && moved.lift.info().right_mV == 1425);
  moved.frame.left_deg = moved.frame.right_deg = 0;
  moved.Tick();
  assert(moved.lift.info().left_mV == 1500);  // 回落不重启。

  fixture tolerance;
  assert(tolerance.lift.zero_here() && tolerance.lift.LiftTo(2.5, 3000));
  tolerance.frame.left_deg = 0.6; tolerance.frame.right_deg = 0.4;
  tolerance.Tick();  // 平均位置进入容差；尚未上移 1 度也结束助推。
  assert(tolerance.lift.info().left_mV == 0 && tolerance.lift.info().right_mV == 0);
  assert(!std::strcmp(tolerance.lift.info().status, "SETTLING"));

  fixture near;
  assert(near.lift.zero_here() && near.lift.LiftTo(2, 3000));
  assert(near.lift.info().left_mV == 0 && near.lift.info().right_mV == 0);

  fixture downward;
  assert(downward.lift.zero_here());
  downward.frame.left_deg = downward.frame.right_deg = 20;
  downward.Tick();
  assert(downward.lift.LiftTo(15, 3000));
  assert(downward.lift.info().left_mV == -500 && downward.lift.info().right_mV == -500);

  fixture short_timeout;
  assert(short_timeout.lift.zero_here() && short_timeout.lift.LiftTo(15, 20));
  short_timeout.Tick();
  assert(!std::strcmp(short_timeout.lift.info().status, "TIMEOUT"));
  assert(short_timeout.lift.info().left_mV == 0 && short_timeout.lift.info().right_mV == 0);

  fixture disabled_boost;
  mechanisms::lift_config settings = PDConfig();
  settings.start_up_ms = 0;
  disabled_boost.lift.configure(settings);
  assert(disabled_boost.lift.zero_here() && disabled_boost.lift.LiftTo(15, 3000));
  assert(disabled_boost.lift.info().left_mV == 1500);
  fixture invalid;
  settings.start_up_mV = 12001;
  invalid.lift.configure(settings);
  assert(!invalid.lift.LiftTo(15, 3000));
  assert(!std::strcmp(invalid.lift.info().status, "CONFIG"));
}

void CheckSharedAxis() {
  // 复现实车差值：共轴编码器差 10.8 度不再被误当成两侧高低差。
  fixture shared;
  assert(shared.lift.zero_here());
  shared.frame.left_deg = 40.8; shared.frame.right_deg = 30;
  shared.Tick();
  assert(std::fabs(shared.lift.info().position_deg() - 35.4) < 1e-9);
  assert(shared.lift.LiftTo(60, 3000));
  assert(shared.lift.busy() && shared.lift.info().left_mV == shared.lift.info().right_mV);
  shared.frame.left_deg = 65.4; shared.frame.right_deg = 54.6;
  shared.Tick();
  assert(!std::strcmp(shared.lift.info().status, "SETTLING"));
  for (int i = 0; i < 10; ++i) shared.Tick();
  assert(shared.lift.at_target());
  assert(shared.lift.info().left_mV == 0 && shared.lift.info().right_mV == 0);

  fixture rate;
  assert(rate.lift.zero_here());
  rate.frame.left_deg = 40; rate.frame.right_deg = 30;
  rate.frame.left_deg_s = 100; rate.frame.right_deg_s = 20;
  rate.Tick();
  assert(rate.lift.LiftTo(60, 3000));
  assert(rate.lift.info().left_mV == 2020 && rate.lift.info().right_mV == 2020);

  fixture moving;
  assert(moving.lift.zero_here());
  moving.frame.left_deg = moving.frame.right_deg = 60;
  moving.frame.left_deg_s = 20; moving.frame.right_deg_s = -20;
  moving.Tick();
  assert(moving.lift.LiftTo(60, 3000));
  for (int i = 0; i < 15; ++i) moving.Tick();
  assert(moving.lift.busy() && !moving.lift.at_target());  // 平均速度为零不等于两台都停稳。

  mechanisms::lift_config legacy_limit = PDConfig();
  legacy_limit.maximum_deg = 150;
  fixture upper(legacy_limit);
  assert(upper.lift.zero_here());
  upper.frame.left_deg = 150; upper.frame.right_deg = 140;
  upper.Tick();
  assert(upper.lift.LiftTo(150, 3000));
  assert(upper.lift.info().left_mV == 0 && upper.lift.info().right_mV == 0);

  fixture lower;
  assert(lower.lift.zero_here());
  lower.frame.left_deg = 0; lower.frame.right_deg = 10;
  lower.Tick();
  assert(lower.lift.LiftTo(0, 3000));
  assert(lower.lift.info().left_mV == 0 && lower.lift.info().right_mV == 0);

  fixture stalled;
  assert(stalled.lift.zero_here() && stalled.lift.LiftTo(60, 3000));
  stalled.frame.left_deg = 10;  // 一台编码器不再进展时，原堵转保护仍分别检查。
  for (int i = 0; i < 30; ++i) stalled.Tick();
  assert(!std::strcmp(stalled.lift.info().status, "STALL"));
  assert(stalled.lift.info().left_mV == 0 && stalled.lift.info().right_mV == 0);
}

void CheckDefaultLiftRange() {
  const mechanisms::lift_config settings;
  const double limit = settings.maximum_deg, tolerance = settings.tolerance_deg;
  const double other = std::max(settings.minimum_deg, limit - 5 * tolerance);
  fixture accepted(settings);
  assert(accepted.lift.zero_here() && accepted.lift.LiftTo(limit, 3000));
  assert(accepted.lift.busy() && accepted.lift.info().left_mV > 0);
  assert(accepted.lift.info().left_mV == accepted.lift.info().right_mV);

  fixture rejected(settings);
  assert(rejected.lift.zero_here() && !rejected.lift.LiftTo(limit + 1, 3000));
  assert(!std::strcmp(rejected.lift.info().status, "TARGET LIMIT"));
  assert(rejected.lift.info().left_mV == 0 && rejected.lift.info().right_mV == 0);

  for (int side = 0; side < 2; ++side) {
    fixture boundary(settings);
    assert(boundary.lift.zero_here() && boundary.lift.LiftTo(limit, 3000));
    boundary.frame.left_deg = side == 0 ? limit : other;
    boundary.frame.right_deg = side == 1 ? limit : other;
    boundary.Tick();
    assert(boundary.lift.busy());
    assert(boundary.lift.info().left_mV == 0 && boundary.lift.info().right_mV == 0);

    fixture beyond(settings);
    assert(beyond.lift.zero_here() && beyond.lift.LiftTo(limit, 3000));
    beyond.frame.left_deg = side == 0 ? limit + tolerance : other;
    beyond.frame.right_deg = side == 1 ? limit + tolerance : other;
    beyond.Tick();  // 恰好处于容差边界，仅禁止继续上行。
    assert(beyond.lift.busy());
    assert(beyond.lift.info().left_mV == 0 && beyond.lift.info().right_mV == 0);
    if (side == 0) beyond.frame.left_deg = limit + tolerance + 1;
    else beyond.frame.right_deg = limit + tolerance + 1;
    beyond.Tick();
    assert(!beyond.lift.busy() && beyond.lift.info().result == mechanisms::state::error);
    assert(!std::strcmp(beyond.lift.info().status, "POSITION LIMIT"));
    assert(beyond.lift.info().left_mV == 0 && beyond.lift.info().right_mV == 0);
  }
}

mechanisms::lift_config SyntheticHeightConfig() {
  // 仅供纯逻辑测试的合成数据，不能作为实车高度标定或默认换算比例。
  mechanisms::lift_config settings = PDConfig();
  settings.maximum_deg = 150;  // 合成表包含151度非法点的旧行程回归。
  settings.height.point_count = 3;
  settings.height.points[0].height_mm = 0;
  settings.height.points[0].motor_deg = 0;
  settings.height.points[1].height_mm = 100;
  settings.height.points[1].motor_deg = 40;
  settings.height.points[2].height_mm = 200;
  settings.height.points[2].motor_deg = 100;
  return settings;
}

void CheckHeightMapping() {
  fixture uncalibrated;
  assert(uncalibrated.lift.zero_here());
  assert(!uncalibrated.lift.LiftHeightTo(100, 3000));
  assert(!std::strcmp(uncalibrated.lift.info().status, "HEIGHT NOT CAL"));
  assert(!uncalibrated.lift.busy());
  assert(uncalibrated.lift.info().left_mV == 0 && uncalibrated.lift.info().right_mV == 0);

  const mechanisms::lift_config settings = SyntheticHeightConfig();
  const double heights[] = {0, 50, 100, 150, 200};
  const double angles[] = {0, 20, 40, 70, 100};
  for (int i = 0; i < 5; ++i) {
    fixture mapped;
    mapped.lift.configure(settings);
    assert(mapped.lift.zero_here() && mapped.lift.LiftHeightTo(heights[i], 3000));
    assert(mapped.lift.info().height_requested);
    assert(mapped.lift.info().requested_height_mm == heights[i]);
    assert(std::fabs(mapped.lift.info().target_deg - angles[i]) < 1e-9);
  }

  const double rejected[] = {-1, 201, std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::infinity()};
  for (double height : rejected) {
    fixture outside;
    outside.lift.configure(settings);
    assert(outside.lift.zero_here());
    assert(!outside.lift.LiftHeightTo(height, 3000));
    assert(!std::strcmp(outside.lift.info().status, "HEIGHT LIMIT"));
    assert(outside.lift.info().left_mV == 0 && outside.lift.info().right_mV == 0);
  }

  fixture no_zero;
  no_zero.lift.configure(settings);
  assert(!no_zero.lift.LiftHeightTo(100, 3000));
  assert(!std::strcmp(no_zero.lift.info().status, "NO ZERO"));
  assert(no_zero.lift.info().left_mV == 0 && no_zero.lift.info().right_mV == 0);

  fixture deadline;
  deadline.lift.configure(settings);
  assert(deadline.lift.zero_here() && deadline.lift.LiftHeightTo(50, 20));
  deadline.Tick();
  assert(!std::strcmp(deadline.lift.info().status, "TIMEOUT"));
  assert(deadline.lift.info().elapsed_ms == 20);
  assert(deadline.lift.info().left_mV == 0 && deadline.lift.info().right_mV == 0);

  fixture bad_timeout;
  bad_timeout.lift.configure(settings);
  assert(bad_timeout.lift.zero_here());
  assert(!bad_timeout.lift.LiftHeightTo(100, 0));
  assert(!std::strcmp(bad_timeout.lift.info().status, "TIMEOUT CONFIG"));
  assert(bad_timeout.lift.info().left_mV == 0 && bad_timeout.lift.info().right_mV == 0);

  // 忙时拒绝高度请求必须先于检查标定，不能终止原有角度动作。
  fixture angle_busy;
  assert(angle_busy.lift.zero_here() && angle_busy.lift.LiftTo(60, 3000));
  const int angle_voltage = angle_busy.lift.info().left_mV;
  assert(!angle_busy.lift.LiftHeightTo(100, 3000));
  assert(angle_busy.lift.busy() && angle_busy.lift.info().target_deg == 60);
  assert(!angle_busy.lift.info().height_requested);
  assert(angle_busy.lift.info().left_mV == angle_voltage && angle_busy.lift.info().right_mV == angle_voltage);

  fixture height_busy;
  height_busy.lift.configure(settings);
  assert(height_busy.lift.zero_here() && height_busy.lift.LiftHeightTo(150, 3000));
  const int height_voltage = height_busy.lift.info().left_mV;
  assert(!height_busy.lift.LiftHeightTo(-1, 0));
  assert(!height_busy.lift.LiftTo(60, 3000));
  assert(height_busy.lift.busy() && height_busy.lift.info().target_deg == 70);
  assert(height_busy.lift.info().height_requested && height_busy.lift.info().requested_height_mm == 150);
  assert(height_busy.lift.info().left_mV == height_voltage && height_busy.lift.info().right_mV == height_voltage);
  height_busy.lift.cancel();
  assert(height_busy.lift.LiftTo(60, 3000));
  assert(!height_busy.lift.info().height_requested);  // 新角度动作不保留旧毫米目标标签。
}

void CheckBadHeightTables() {
  for (int defect = 0; defect < 10; ++defect) {
    mechanisms::lift_config settings = SyntheticHeightConfig();
    switch (defect) {
      case 0: settings.height.point_count = 2; break;
      case 1: settings.height.point_count = 4; break;
      case 2: settings.height.points[1].height_mm = settings.height.points[0].height_mm; break;
      case 3: settings.height.points[2].height_mm = 50; break;
      case 4: settings.height.points[2].motor_deg = settings.height.points[1].motor_deg; break;
      case 5: settings.height.points[1].motor_deg = -1; break;
      case 6: settings.height.points[2].motor_deg = 151; break;
      case 7: settings.height.points[1].height_mm = std::numeric_limits<double>::quiet_NaN(); break;
      case 8: settings.height.points[1].motor_deg = std::numeric_limits<double>::infinity(); break;
      case 9: settings.height.points[0].height_mm = -1; break;
    }
    fixture invalid;
    invalid.lift.configure(settings);
    assert(invalid.lift.zero_here());
    assert(!invalid.lift.LiftHeightTo(100, 3000));
    assert(!std::strcmp(invalid.lift.info().status, "HEIGHT CONFIG"));
    assert(invalid.lift.info().left_mV == 0 && invalid.lift.info().right_mV == 0);
  }
}

mechanisms::lift_authority AllowedLiftAuthority(double time_ms) {
  mechanisms::lift_authority authority;
  authority.manual_allowed = authority.closed_loop_allowed = authority.start_ready = true;
  authority.time_ms = time_ms;
  return authority;
}

void PrimeLiftMailbox(mechanisms::lift_mailbox& mailbox) {
  mailbox.publish(AllowedLiftAuthority(0), mechanisms::lift_request());
  // 来源交接用 reset 撤销旧控制，与当周期的新命令分开表达。
  mechanisms::lift_packet packet = mailbox.take();
  assert(packet.reset && !packet.overflow);
  assert(packet.command.kind == mechanisms::lift_command_kind::none);
  packet = mailbox.take();
  assert(!packet.reset && packet.command.kind == mechanisms::lift_command_kind::none);
}

void CheckLiftMailbox() {
  using mechanisms::lift_command_kind;
  mechanisms::lift_mailbox mailbox;
  PrimeLiftMailbox(mailbox);
  const mechanisms::lift_request angle = CurrentLiftDebug();
  mailbox.publish(AllowedLiftAuthority(10), angle);
  mailbox.publish(AllowedLiftAuthority(20), mechanisms::lift_request());
  mailbox.publish(AllowedLiftAuthority(30), mechanisms::lift_request());
  mechanisms::lift_packet packet = mailbox.take();
  assert(packet.command.kind == lift_command_kind::angle && packet.command.target == angle.target);
  assert(packet.command.timeout_ms == angle.timeout_ms && packet.command_time_ms == 10);
  assert(packet.authority.time_ms == 30 && !packet.overflow && !packet.reset);
  assert(mailbox.take().command.kind == lift_command_kind::none);  // 事件仅消费一次。

  mechanisms::lift_request zero;
  zero.zero_here();
  mailbox.publish(AllowedLiftAuthority(40), zero);
  mailbox.publish(AllowedLiftAuthority(50), mechanisms::lift_request());
  packet = mailbox.take();
  assert(packet.command.kind == lift_command_kind::zero && packet.command_time_ms == 40);
  mechanisms::lift_request height;
  assert(height.LiftHeightTo(150, 2000));
  mailbox.publish(AllowedLiftAuthority(60), height);
  packet = mailbox.take();
  assert(packet.command.kind == lift_command_kind::height && packet.command.target == 150);
  assert(packet.command.timeout_ms == 2000 && !packet.overflow);

  // 显式取消覆盖旧动作，后续新动作不能覆盖尚未消费的取消。
  mechanisms::lift_request cancel;
  cancel.cancel();
  mailbox.publish(AllowedLiftAuthority(70), angle);
  mailbox.publish(AllowedLiftAuthority(80), cancel);
  mailbox.publish(AllowedLiftAuthority(90), height);
  packet = mailbox.take();
  assert(packet.command.kind == lift_command_kind::cancel && packet.command_time_ms == 80);
  assert(!packet.overflow);
  assert(mailbox.take().command.kind == lift_command_kind::none);

  // 首个自动命令可与来源交接同包；worker 先 reset 旧控制，再执行这次显式动作。
  mechanisms::lift_mailbox first_auto;
  mechanisms::lift_authority automatic = AllowedLiftAuthority(10);
  automatic.manual_allowed = false; automatic.autonomous = true;
  first_auto.publish(automatic, angle);
  first_auto.publish(automatic, mechanisms::lift_request());
  packet = first_auto.take();
  assert(packet.reset && !packet.overflow && packet.authority.autonomous);
  assert(packet.command.kind == lift_command_kind::angle && packet.command.target == angle.target);
  assert(packet.command.timeout_ms == angle.timeout_ms);
  assert(packet.command_time_ms == 10);
  packet = first_auto.take();
  assert(!packet.reset && packet.command.kind == lift_command_kind::none);

  // 手控/闭环失权、场控或 autonomous 来源切换均清除旧事件，恢复不重放旧 start。
  for (int change = 0; change < 4; ++change) {
    mechanisms::lift_mailbox guarded;
    PrimeLiftMailbox(guarded);
    guarded.publish(AllowedLiftAuthority(10), angle);
    mechanisms::lift_authority denied = AllowedLiftAuthority(20);
    if (change == 0) denied.manual_allowed = false;
    if (change == 1) denied.closed_loop_allowed = false;
    if (change == 2) denied.field_connected = true;
    if (change == 3) denied.autonomous = true;
    guarded.publish(denied, mechanisms::lift_request());
    packet = guarded.take();
    assert(packet.reset && !packet.overflow && packet.command.kind == lift_command_kind::none);
    guarded.publish(AllowedLiftAuthority(30), mechanisms::lift_request());
    guarded.publish(AllowedLiftAuthority(40), mechanisms::lift_request());
    packet = guarded.take();
    assert(packet.reset && packet.command.kind == lift_command_kind::none && !packet.overflow);
    assert(packet.authority.manual_allowed && packet.authority.closed_loop_allowed && !packet.authority.field_connected);
    assert(!packet.authority.autonomous);
    packet = guarded.take();
    assert(!packet.reset && packet.command.kind == lift_command_kind::none);
    guarded.publish(AllowedLiftAuthority(50), angle);
    packet = guarded.take();
    assert(!packet.reset && packet.command.kind == lift_command_kind::angle);
  }

  // 尚未消费的旧 start 在切自动时被清除，本次显式 height 仍可被接受。
  mechanisms::lift_mailbox handoff;
  PrimeLiftMailbox(handoff);
  handoff.publish(AllowedLiftAuthority(10), angle);
  automatic.time_ms = 20;
  handoff.publish(automatic, height);
  automatic.time_ms = 30;
  handoff.publish(automatic, mechanisms::lift_request());
  packet = handoff.take();
  assert(packet.reset && !packet.overflow && packet.command.kind == lift_command_kind::height);
  assert(packet.command_time_ms == 20 && packet.authority.time_ms == 30);

  mechanisms::lift_mailbox cancelled_session;
  PrimeLiftMailbox(cancelled_session);
  cancelled_session.publish(AllowedLiftAuthority(10), cancel);
  cancelled_session.publish(automatic, angle);
  packet = cancelled_session.take();
  assert(packet.reset && !packet.overflow && packet.command.kind == lift_command_kind::angle);

  // 两个未消费事件无法安全排序时，锁存 overflow + cancel；心跳不能清除此故障。
  mailbox.publish(AllowedLiftAuthority(100), angle);
  mailbox.publish(AllowedLiftAuthority(110), height);
  mailbox.publish(AllowedLiftAuthority(120), mechanisms::lift_request());
  mailbox.publish(AllowedLiftAuthority(130), zero);
  packet = mailbox.take();
  assert(packet.overflow && packet.command.kind == lift_command_kind::cancel);
  assert(packet.command_time_ms == 110);  // 心跳和排在取消之后的动作不改写事件时间。
  packet = mailbox.take();
  assert(!packet.overflow && packet.command.kind == lift_command_kind::none);
  mailbox.publish(AllowedLiftAuthority(140), height);
  packet = mailbox.take();
  assert(!packet.overflow && packet.command.kind == lift_command_kind::height);

  // overflow 属于旧来源；新来源 reset 可清除此锁存并接受本次显式动作。
  mailbox.publish(AllowedLiftAuthority(150), angle);
  mailbox.publish(AllowedLiftAuthority(160), height);
  automatic.time_ms = 170;
  mailbox.publish(automatic, angle);
  packet = mailbox.take();
  assert(packet.reset && !packet.overflow && packet.command.kind == lift_command_kind::angle);
  assert(packet.command_time_ms == 170);
}

int main() {
  CheckDebugEntry();
  CheckStartupZero();
  CheckPressEdge();
  CheckPID();
  CheckPanel();
  CheckStartup();
  CheckSharedAxis();
  CheckDefaultLiftRange();
  CheckHeightMapping();
  CheckBadHeightTables();
  CheckLiftMailbox();
  fixture no_zero;
  assert(!no_zero.lift.LiftTo(60, 3000));
  assert(!std::strcmp(no_zero.lift.info().status, "NO ZERO"));
  assert(no_zero.lift.info().left_mV == 0 && no_zero.lift.info().right_mV == 0);

  // 固定60度/3000ms单独检查超时边界，不随用户调试目标或时间的修改而改变。
  fixture timeout;
  assert(timeout.lift.zero_here());
  assert(timeout.lift.LiftTo(60, 3000));
  assert(timeout.lift.busy() && timeout.lift.info().target_deg == 60);
  assert(timeout.lift.info().left_mV > 0 && timeout.lift.info().right_mV > 0);
  assert(!timeout.lift.LiftTo(120, 6000));  // 新参数不能覆盖运行中的目标和原超时。
  for (int i = 1; i < 150; ++i) {
    // 缓慢向目标移动；保持在目标窗口外，观察 timeout 本身的精确边界。
    timeout.frame.left_deg = timeout.frame.right_deg = i * 0.1;
    timeout.frame.left_deg_s = timeout.frame.right_deg_s = 1;
    timeout.Tick();
    assert(timeout.lift.busy());
  }
  assert(timeout.now == 2980);
  timeout.Tick();
  assert(timeout.now == 3000 && timeout.lift.info().elapsed_ms == 3000);
  assert(timeout.lift.info().result == mechanisms::state::error);
  assert(!std::strcmp(timeout.lift.info().status, "TIMEOUT"));
  assert(timeout.lift.info().left_mV == 0 && timeout.lift.info().right_mV == 0);

  fixture cancelled;
  assert(cancelled.lift.zero_here() && cancelled.lift.LiftTo(60, 3000));
  cancelled.lift.cancel();
  assert(cancelled.lift.info().result == mechanisms::state::cancelled);
  assert(cancelled.lift.info().left_mV == 0 && cancelled.lift.info().right_mV == 0);

  fixture disabled;
  assert(disabled.lift.zero_here() && disabled.lift.LiftTo(60, 3000));
  disabled.Tick(false);
  assert(disabled.lift.info().result == mechanisms::state::cancelled);
  assert(disabled.lift.info().left_mV == 0 && disabled.lift.info().right_mV == 0);
  disabled.Tick(true);
  assert(!disabled.lift.busy());  // 权限恢复不能自动重启。

  fixture offline;
  assert(offline.lift.zero_here() && offline.lift.LiftTo(60, 3000));
  offline.frame.arm_online = false;
  offline.Tick();
  assert(offline.lift.info().result == mechanisms::state::error && !offline.lift.info().homed);
  assert(!std::strcmp(offline.lift.info().status, "MOTOR OFF"));
  assert(offline.lift.info().left_mV == 0 && offline.lift.info().right_mV == 0);
  offline.frame.arm_online = true;
  offline.Tick();
  assert(!offline.lift.LiftTo(60, 3000));
  assert(!std::strcmp(offline.lift.info().status, "NO ZERO"));

  std::puts("Lift debug checks: PID, shared axis, calibrated height mapping, explicit deadlines, mailbox handoff and interlocks.");
}
