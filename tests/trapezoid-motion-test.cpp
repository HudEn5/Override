#include "drive-straight.h"
#include "driver-control-logic.h"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <limits>

drive::feedforward_config Model() {
  drive::feedforward_config result;
  result.kS = 400; result.kV = 16; result.calibrated = true;
  return result;
}

struct fixture {
  int targets[3] = {100, 150, 200};
  drive::straight_controller test;
  manual::Output output;
  drive::velocity_feedback speed;
  drive::position_feedback position;
  double now = 0, heading = 0, distance_mm;
  bool enabled = true, online = true, imu = true;
  explicit fixture(double distance = 500) : distance_mm(distance) {
    drive::straight_config config;
    config.left_forward = config.right_forward = config.left_reverse = config.right_reverse = Model();
    config.motion.wheel_diameter_mm = 69.85; config.motion.motor_revs_per_wheel_rev = 1.2;
    test.Configure(config);
    speed.valid = position.valid = true;
    position.left_deg = 1234; position.right_deg = -567; // 不依赖编码器清零。
    Step();
  }
  void Step(double dt = 10, bool new_sample = true) {
    now += dt;
    if (new_sample) { ++speed.sequence; speed.time_ms = now; }
    drive::straight_feedback frame;
    frame.velocity = speed; frame.position = position; frame.motors_online = online;
    frame.inertial_ready = imu; frame.heading_deg = heading;
    frame.stopped = std::fabs(speed.left_rpm) < 5 && std::fabs(speed.right_rpm) < 5;
    test.Observe(frame, enabled, now); test.UpdateDriveStraight();
    output.L_voltage = test.Info().left_mV; output.R_voltage = test.Info().right_mV;
  }
  void Start(int direction = 1, int level = 0) {
    test.StartDriveStraight(direction * distance_mm, targets[level]);
    output.L_voltage = test.Info().left_mV; output.R_voltage = test.Info().right_mV;
  }
  bool Running() const { return test.Info().state == drive::straight_state::running; }
};

double PlantTarget(int voltage) {
  return std::fabs(static_cast<double>(voltage)) < 400 ? 0 :
      (voltage - (voltage > 0 ? 400.0 : -400.0)) / 16.0;
}

void Simulate(int direction, int level, double distance, bool friction = false) {
  fixture f(distance);
  f.Start(direction, level); assert(f.Running() && f.output.L_voltage == 0);
  double left = 0, right = 0, peak = 0;
  drive::velocity_sampler left_sampler, right_sampler;
  drive::motor_feedback lm[4], rm[4];
  for (int k = 0; k < 4; ++k) {
    lm[k].online = rm[k].online = lm[k].position_valid = rm[k].position_valid = true;
    lm[k].position_deg = f.position.left_deg; rm[k].position_deg = f.position.right_deg;
  }
  left_sampler.Update(lm, f.now - 20); right_sampler.Update(rm, f.now - 20);
  left_sampler.Update(lm, f.now); right_sampler.Update(rm, f.now);
  bool decelerated = false;
  for (int i = 0; i < 1500 && f.Running(); ++i) {
    double left_target = PlantTarget(f.output.L_voltage), right_target = PlantTarget(f.output.R_voltage);
    if (friction && std::fabs(left) < 1 && std::abs(f.output.L_voltage) < 700) left_target = 0;
    if (friction && std::fabs(right) < 1 && std::abs(f.output.R_voltage) < 700) right_target = 0;
    const double left_alpha = friction ? (f.output.L_voltage == 0 ? 0.4 : 0.08) : 0.2;
    const double right_alpha = friction ? (f.output.R_voltage == 0 ? 0.4 : 0.08) : 0.2;
    left += left_alpha * (left_target - left);
    right += right_alpha * (right_target - right);
    f.position.left_deg += left * 6 * 0.01;
    f.position.right_deg += right * 6 * 0.01;
    const bool sample = i % 2 == 0;
    if (sample) {
      for (int k = 0; k < 4; ++k) {
        lm[k].position_deg = f.position.left_deg; rm[k].position_deg = f.position.right_deg;
        lm[k].rpm = left; rm[k].rpm = right;
      }
      if (friction && i % 14 == 0) rm[3].rpm = -1510; // 原始异常不能污染 TP 内环。
      const drive::side_sample ls = left_sampler.Update(lm, f.now + 10);
      const drive::side_sample rs = right_sampler.Update(rm, f.now + 10);
      assert(ls.encoder_velocity_valid && rs.encoder_velocity_valid);
      f.speed.left_rpm = ls.encoder_rpm; f.speed.right_rpm = rs.encoder_rpm;
    }
    f.Step(10, sample);
    peak = std::fmax(peak, std::fabs(f.test.Info().motion.reference_rpm));
    decelerated |= f.test.Info().motion.phase == 3;
    assert(std::abs(f.output.L_voltage) <= 12000 && std::abs(f.output.R_voltage) <= 12000);
    assert(std::fabs(f.test.Info().motion.target_rpm) <= f.targets[level] + 1e-8);
  }
  if (std::strcmp(f.test.Info().status, "DONE") != 0)
    std::printf("Simulation failed: dir=%d rpm=%d distance=%.0f status=%s remaining=%.2f\n",
                direction, f.targets[level], distance, f.test.Info().status,
                f.test.Info().motion.remaining_mm);
  assert(std::strcmp(f.test.Info().status, "DONE") == 0);
  assert(std::fabs(f.test.Info().motion.remaining_mm) <= 5);
  assert(f.output.L_voltage == 0 && f.output.R_voltage == 0 && decelerated);
  assert(peak <= f.targets[level] && peak > 0);
  if (distance == 1000) assert(f.test.Info().motion.elapsed_ms > 3000);
  const double last = f.test.Info().motion.distance_mm;
  f.Step(); assert(!f.Running() && f.output.L_voltage == 0);
  assert(std::strcmp(f.test.Info().status, "DONE") == 0); // 终态不被 RELEASE 覆盖。
  f.Step(); assert(std::strcmp(f.test.Info().status, "DONE") == 0);
  assert(f.test.Info().motion.distance_mm == last);

}

int main() {
  for (int direction : {-1, 1})
    for (int level = 0; level < 3; ++level) Simulate(direction, level, 500);
  Simulate(1, 0, 1000); // 动作超时由规划时长决定，不继承 SP 的固定 3 秒。
  Simulate(1, 0, 500, true);
  Simulate(-1, 0, 500, true); // 含响应滞后、静摩擦及 -1510 rpm 尖峰的回归模型。
  fixture f;
  f.Start(); f.Step(); assert(f.Running());
  f.test.CancelDriveStraight(); f.Step();
  assert(!f.Running() && f.output.L_voltage == 0 && f.test.Info().motion.phase == 6);
  f.Start(-1); f.Step(101, false);
  assert(!f.Running() && f.output.L_voltage == 0 && std::strcmp(f.test.Info().status, "SPEED ERR") == 0);
  f.Step(); assert(!f.Running());
  fixture position; position.Start(); position.position.valid = false; position.Step();
  assert(position.output.L_voltage == 0 && std::strcmp(position.test.Info().status, "POSITION ERR") == 0);
  fixture moving; moving.speed.left_rpm = 10; moving.Step(); moving.Start();
  assert(moving.output.L_voltage == 0 && std::strcmp(moving.test.Info().status, "MOVING") == 0);
  fixture clock; clock.Start(); clock.Step(201);
  assert(clock.output.L_voltage == 0 && std::strcmp(clock.test.Info().status, "CLOCK ERR") == 0);
  for (int fault = 0; fault < 3; ++fault) {
    fixture cancel; cancel.Start(); cancel.Step();
    if (fault == 0) cancel.enabled = false;
    if (fault == 1) cancel.online = false;
    if (fault == 2) cancel.imu = false;
    cancel.Step(); assert(!cancel.Running() && cancel.output.L_voltage == 0 && cancel.output.R_voltage == 0);
  }
  fixture stalled; stalled.Start();
  for (int i = 0; i < 1000 && stalled.Running(); ++i) stalled.Step();
  assert(std::strcmp(stalled.test.Info().status, "TIMEOUT") == 0 && stalled.output.L_voltage == 0);
  stalled.Step(); assert(std::strcmp(stalled.test.Info().status, "TIMEOUT") == 0);
  fixture yaw; yaw.Start(); yaw.heading = 1; yaw.Step();
  assert(yaw.test.Info().motion.target_rpm > 0 && yaw.test.Info().motion.target_rpm < 5);
  assert(yaw.test.Info().velocity.left_target > 0 && yaw.test.Info().velocity.right_target > 0);
  assert(std::fabs(yaw.test.Info().heading.correction) <= yaw.test.Info().motion.target_rpm * 16 * 0.25 + 1e-9);
  std::puts("Trapezoid checks passed through the public straight controller: signed 500 mm at three caps, long moves, friction/spikes, cancellation, timeout and feedback faults.");
}
