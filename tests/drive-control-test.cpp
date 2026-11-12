#include "drive-control.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

int main() {
  drive::telemetry_switch telemetry;
  assert(!telemetry.Update(true, true));  // 启动时按住 Y 不开始。
  assert(!telemetry.Update(false, true));
  assert(telemetry.Update(true, true));
  assert(telemetry.Update(true, true));  // 长按不反复切换。
  assert(telemetry.Update(false, true));
  assert(!telemetry.Update(true, true));
  assert(!telemetry.Update(false, true));
  assert(telemetry.Update(true, true));
  assert(!telemetry.Update(false, false));  // 掉线立即停止回传。
  assert(!telemetry.Update(true, true));  // 重连时按住 Y 不开始。
  assert(!telemetry.Update(false, true));
  assert(telemetry.Update(true, true));

  drive::motor_feedback motors[4];
  for (int i = 0; i < 4; ++i) {
    motors[i].online = true;
    motors[i].rpm = 100 + i * 20;
    motors[i].millivolts = 3000;
  }
  drive::velocity_sampler sampler;
  auto value = sampler.Update(motors, 0);
  assert(value.velocity_valid && value.online_mask == 15);
  assert(value.rpm == 130 && value.raw_average_rpm == 130);
  assert(value.millivolts == 3000 && !value.acceleration_valid);
  for (int i = 0; i < 4; ++i) motors[i].rpm += 60;
  value = sampler.Update(motors, 20);
  assert(std::fabs(value.rpm - 150) < 1e-8);
  assert(std::fabs(value.acceleration - 1000) < 1e-8);
  assert(value.acceleration_valid);
  // 不同 dt 下仍使用实际秒数求加速度。
  value = sampler.Update(motors, 60);
  assert(std::fabs(value.rpm - 170) < 1e-8);
  assert(std::fabs(value.acceleration - 500) < 1e-8);
  motors[2].online = false;
  value = sampler.Update(motors, 80);
  assert(value.online_mask == 11 && !value.velocity_valid && !value.acceleration_valid);
  motors[2].online = true;
  value = sampler.Update(motors, 100);
  assert(value.velocity_valid && value.rpm == 190 && !value.acceleration_valid);
  value = sampler.Update(motors, 100);
  assert(!value.acceleration_valid);
  value = sampler.Update(motors, 90);
  assert(!value.acceleration_valid);
  value = sampler.Update(motors, 500);
  assert(!value.acceleration_valid);
  motors[0].rpm = std::numeric_limits<double>::quiet_NaN();
  value = sampler.Update(motors, 520);
  assert(!value.velocity_valid && value.online_mask == 14);
  motors[0].rpm = -100;
  for (int i = 1; i < 4; ++i) motors[i].rpm = -100;
  value = sampler.Update(motors, 540);
  assert(value.rpm == -100 && !value.acceleration_valid);
  assert(!sampler.Update(motors, -1).velocity_valid);

  // 真实位置差分不受 SDK -1510 rpm 尖峰影响；原始值及质量标记仍可诊断。
  drive::velocity_sampler guarded;
  for (int i = 0; i < 4; ++i) {
    motors[i].rpm = 100;
    motors[i].position_valid = true;
    motors[i].position_deg = i * 50;
  }
  value = guarded.Update(motors, 0);
  assert(!value.encoder_velocity_valid && value.velocity_valid);
  for (int i = 0; i < 4; ++i) motors[i].position_deg += 12; // 20 ms 内 12 度，即 100 rpm。
  value = guarded.Update(motors, 20);
  assert(value.encoder_velocity_valid && std::fabs(value.encoder_rpm - 100) < 1e-8);
  motors[3].rpm = -1510;
  for (int i = 0; i < 4; ++i) motors[i].position_deg += 12;
  value = guarded.Update(motors, 40);
  assert(value.rejected_rpm_mask == 8 && value.raw_rpm[3] == -1510);
  assert(value.velocity_valid && std::fabs(value.rpm - 100) < 1e-8);
  assert(std::fabs(value.encoder_rpm - 100) < 1e-8);
  for (int i = 0; i < 4; ++i) { motors[i].position_deg += 12; motors[i].rpm = 100; }
  value = guarded.Update(motors, 60);
  assert(value.rejected_rpm_mask == 0 && value.velocity_valid);
  motors[0].position_deg += 10000;
  value = guarded.Update(motors, 80);
  assert(!value.encoder_velocity_valid); // 位置跳变也不能送入 TP。
  drive::velocity_sampler no_fallback;
  motors[3].rpm = -1510;
  value = no_fallback.Update(motors, 0);
  assert(!value.velocity_valid && !value.encoder_velocity_valid && value.rejected_rpm_mask == 8);
  motors[3].rpm = 100;
  value = no_fallback.Update(motors, 400);
  assert(!value.encoder_velocity_valid); // 长间隔不能差分出伪速度。
  for (int i = 0; i < 4; ++i) { motors[i].position_deg -= 6; motors[i].rpm = -50; }
  value = no_fallback.Update(motors, 420);
  assert(value.encoder_velocity_valid && std::fabs(value.encoder_rpm + 50) < 1e-8);
  motors[1].online = false;
  assert(!no_fallback.Update(motors, 440).encoder_velocity_valid);
  drive::velocity_sampler parked;
  for (int i = 0; i < 4; ++i) { motors[i].online = true; motors[i].rpm = 0; }
  parked.Update(motors, 0); parked.Update(motors, 20);
  motors[3].rpm = -1510;
  value = parked.Update(motors, 40);
  assert(value.encoder_velocity_valid && value.encoder_rpm == 0);
  assert(value.velocity_valid && value.rpm == 0 && value.rejected_rpm_mask == 8);

  // 以下参数仅为数学测试数据，不代表实车辨识结果。
  drive::feedforward_config config;
  drive::motor_feedforward missing(config);
  assert(!missing.Calculate(100, 0).valid);
  config.kS = 500;
  config.kV = 20;
  config.kA = 2;
  config.calibrated = true;
  drive::motor_feedforward ff(config);
  auto output = ff.Calculate(100, 50);
  assert(output.valid && !output.saturated && output.millivolts == 2600);
  assert(ff.Calculate(-100, -50).millivolts == -2600);
  assert(ff.Calculate(0, 0).millivolts == 0);
  assert(ff.Calculate(0, 50).millivolts == 600);
  output = ff.Calculate(1000, 0);
  assert(output.saturated && output.requested_mV == 20500 && output.millivolts == 12000);
  assert(ff.Calculate(-1000, 0).millivolts == -12000);
  assert(!ff.Calculate(std::numeric_limits<double>::infinity(), 0).valid);
  config.kV = -1;
  assert(!drive::motor_feedforward(config).Calculate(100, 0).valid);
  config.kV = 20;
  config.max_voltage = 12001;
  assert(!drive::motor_feedforward(config).Calculate(100, 0).valid);
  std::puts("Drive control checks passed: sampling, timestamps, dropout, feedforward and saturation.");
}
