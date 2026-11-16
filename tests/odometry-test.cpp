#include "drive-control.h"
#include <cassert>
#include <cstdio>
#include <limits>
#include <initializer_list>

namespace {
constexpr double PI = 3.14159265358979323846;
constexpr double MM_PER_DEG = PI * 69.85 / (360 * 1.2);
void Near(double actual, double expected) { assert(std::fabs(actual - expected) < 1e-7); }
drive::odometry Ready(double heading = 0) {
  drive::odometry result;
  assert(result.Configure(69.85, 1.2, 325));
  assert(result.SetPose(10, 20, heading));
  result.Update(500, -700, 359, 0, true);
  assert(result.Info().valid);
  return result;
}
void Lost(const drive::odometry& value) {
  assert(!value.Info().valid && value.Info().needs_reset);
  Near(value.Info().x_mm, 10); Near(value.Info().y_mm, 20);
  Near(value.Info().time_ms, 0);
}
}

int main() {
  auto value = Ready();
  value.Update(560, -640, 359, 20, true);
  Near(value.Info().x_mm, 10 + 60 * MM_PER_DEG); Near(value.Info().y_mm, 20);
  value.Update(500, -700, 359, 40, true);
  Near(value.Info().x_mm, 10);
  assert(value.SetPose(100, 200, 90));
  value.Update(560, -640, 359, 60, true);
  Near(value.Info().x_mm, 100); Near(value.Info().y_mm, 200 + 60 * MM_PER_DEG);
  // 重复时间不覆盖编码器基准；下一帧只计入一次位移。
  value.Update(600, -600, 359, 60, true);
  value.Update(620, -580, 359, 80, true);
  Near(value.Info().y_mm, 200 + 120 * MM_PER_DEG);
  assert(!value.SetPose(std::numeric_limits<double>::quiet_NaN(), 0, 0));
  Near(value.Info().x_mm, 100);

  // 编码器给中心弧长，连续IMU角度跨360度不会跳变；整圆返回起点。
  value = Ready();
  const double radius = 300, step_angle = 2 * PI / 180;
  const double left_step = (radius - 325.0 / 2) * step_angle / MM_PER_DEG;
  const double right_step = (radius + 325.0 / 2) * step_angle / MM_PER_DEG;
  for (int i = 1; i <= 180; ++i) {
    value.Update(500 + i * left_step, -700 + i * right_step, 359 + i * 2, i * 20, true);
    assert(value.Info().valid);
    const double angle = i * step_angle;
    Near(value.Info().x_mm, 10 + radius * std::sin(angle));
    Near(value.Info().y_mm, 20 + radius * (1 - std::cos(angle)));
  }
  Near(value.Info().heading_deg, 360);
  // 单侧静止的swing与原地转向均不需要额外定位轮。
  value = Ready();
  value.Update(500, -700 + 325 * step_angle / MM_PER_DEG, 361, 20, true);
  Near(value.Info().x_mm, 10 + 162.5 * std::sin(step_angle));
  Near(value.Info().y_mm, 20 + 162.5 * (1 - std::cos(step_angle)));
  value = Ready();
  value.Update(480, -680, 361, 20, true);
  Near(value.Info().x_mm, 10); Near(value.Info().y_mm, 20);
  value.Update(500, -700, 359, 40, true);
  Near(value.Info().heading_deg, 0);

  value = Ready();
  value.Update(500, -700, 359, 20, false); Lost(value);
  value.Update(520, -680, 361, 40, true); Lost(value);
  assert(value.SetPose(1000, 2000, -90));
  assert(!value.Info().valid && !value.Info().needs_reset);
  value.Update(10000, 20000, -999, 1000, true);
  Near(value.Info().x_mm, 1000); Near(value.Info().heading_deg, -90);
  value.Update(10060, 20060, -999, 1020, true);
  Near(value.Info().y_mm, 2000 - 60 * MM_PER_DEG);

  value = Ready(); value.Update(500, -700, 359, 201, true); Lost(value);
  value = Ready(); value.Update(500, -700, 359, -1, true); Lost(value);
  value = Ready(); value.Update(1500, -700, 359, 20, true); Lost(value);
  value = Ready(); value.Update(500, -700, 0, 20, true); Lost(value);
  value = Ready(); value.Update(std::numeric_limits<double>::infinity(), -700, 359, 20, true); Lost(value);
  value = Ready(); value.Update(500, -700, 359, 50, true);
  value.Update(500, -700, 359, 40, true);
  assert(value.Info().needs_reset); Near(value.Info().time_ms, 50);
  // 600rpm读数每20ms刷新，而主循环每10ms调用，不能误判丢定位。
  value = Ready();
  for (int i = 1; i <= 100; ++i) {
    const double position = (i / 2) * 72;
    value.Update(500 + position, -700 + position, 359, i * 10, true);
    assert(value.Info().valid);
  }
  Near(value.Info().x_mm, 10 + 3600 * MM_PER_DEG);
  drive::odometry waiting;
  assert(waiting.Configure(69.85, 1.2, 325));
  waiting.Update(0, 0, 0, 0, false);
  assert(!waiting.Info().valid && !waiting.Info().needs_reset);
  waiting.Update(100, 200, 300, 5000, true);
  assert(waiting.Info().valid); Near(waiting.Info().x_mm, 0);
  for (double bad : {0.0, -1.0, std::numeric_limits<double>::infinity()}) {
    assert(!waiting.Configure(bad, 1.2, 325));
    assert(!waiting.Configure(69.85, bad, 325));
    assert(!waiting.Configure(69.85, 1.2, bad));
    assert(!waiting.SetPose(0, 0, 0));
  }
  std::puts("Odometry passed: exact arcs, frame reset, cached feedback, continuous IMU, loss latch and recovery.");
}
