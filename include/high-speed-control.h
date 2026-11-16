#pragma once

#include "test-control.h"

struct tuning_stage {
  double distance_mm, maximum_rpm, acceleration_rpm_s, deceleration_rpm_s;
  double position_gain_scale, position_correction_scale;
  tuning_stage(double distance = 0, double maximum = 0, double acceleration = 0, double deceleration = 0,
               double gain_scale = 1, double correction_scale = 1)
      : distance_mm(distance), maximum_rpm(maximum), acceleration_rpm_s(acceleration),
        deceleration_rpm_s(deceleration), position_gain_scale(gain_scale), position_correction_scale(correction_scale) {}
};
struct tuning_metrics {
  unsigned int run = 0, stage = 0;
  int direction = 1, state = 0;  // 0 就绪，1 运行，2 完成，3 取消，4 故障。
  double time_ms = 0, elapsed_ms = 0, peak_rpm = 0, peak_heading_deg = 0;
  double speed_rms_rpm = 0, high_voltage_ms = 0, at_speed_ms = 0;
  bool qualified = false;  // 完成且实际速度在目标附近连续保持至少 300 ms。
};

// 高速整定只执行一次完整直行，配置为低速配置的局部副本，不修改公共算法。
class high_speed_control {
 public:
  void Configure(const tuning_stage (&values)[4], const drive::straight_config& low_speed) {
    baseline = low_speed;
    for (unsigned int i = 0; i < 4; ++i) stages[i] = values[i];
    info = test_info(); metrics = tuning_metrics(); Select();
  }
  const test_info& Info() const { return info; }
  const tuning_metrics& Metrics() const { return metrics; }
  bool Update(const manual::Input& input, manual::Output& output, bool allowed,
              double now, const drive::straight_feedback& frame) {
    Straight.Observe(frame, allowed, now);
    const bool start = input.test_start && !pre_start;
    const bool up = input.test_next && !pre_up, down = input.test_previous && !pre_down;
    const bool positive = input.test_positive && !pre_positive, negative = input.test_negative && !pre_negative;
    const bool was_allowed = pre_allowed;
    pre_start = input.test_start; pre_up = input.test_next; pre_down = input.test_previous;
    pre_positive = input.test_positive; pre_negative = input.test_negative; pre_allowed = allowed;
    if (!allowed || output.L_voltage != 0 || output.R_voltage != 0) {
      if (info.running) Cancel(now);
      info.visible = false; return false;
    }
    info.visible = true; output.L_voltage = output.R_voltage = 0;
    if (input.test_cancel) { Cancel(now); return true; }
    if (!info.running && was_allowed) {
      if ((up || down || positive || negative) && frame.stopped) {
        if (input.test_next != input.test_previous) {
          if (up && metrics.stage < 3) ++metrics.stage;
          if (down && metrics.stage > 0) --metrics.stage;
        }
        if (input.test_positive != input.test_negative) {
          if (positive) info.direction = 1;
          if (negative) info.direction = -1;
        }
        Select();
      }
      const bool selecting = input.test_next || input.test_previous || input.test_positive || input.test_negative;
      if (start && !selecting) {
        if (!frame.stopped) info.status = "MOVING";
        else if (!ValidStage()) info.status = "HS CONFIG";
        else {
          // 每轮显式配置独立实例，低速的 PID、前馈和航向参数原样复制。
          auto cfg = baseline; const auto& s = stages[metrics.stage];
          cfg.motion.acceleration_rpm_s = s.acceleration_rpm_s;
          cfg.motion.deceleration_rpm_s = s.deceleration_rpm_s;
          // 仅对所选高速档位的配置副本生效，保留低速及 H3 对照参数。
          cfg.motion.position_kP *= s.position_gain_scale;
          cfg.motion.max_position_correction_rpm *= s.position_correction_scale;
          cfg.retain_heading = false;
          Straight.Configure(cfg); Straight.Observe(frame, allowed, now);
          const unsigned int stage = metrics.stage;
          metrics = tuning_metrics(); metrics.stage = stage; metrics.direction = info.direction;
          metrics.run = ++serial; metrics.time_ms = started_at = now; initial_heading = frame.heading_deg;
          squared_error = error_time = consecutive = 0; last_sequence = frame.velocity.sequence;
          last_sample_time = frame.velocity.time_ms;
          info.running = Straight.StartDriveStraight(info.request_mm, info.maximum_rpm);
          metrics.state = info.running ? 1 : 4; info.status = Straight.Info().status;
        }
      }
    }
    if (info.running) {
      Straight.UpdateDriveStraight();
      Measure(frame, now); info.status = Straight.Info().status;
      info.running = Straight.Info().state == drive::straight_state::running;
      metrics.state = info.running ? 1 : Straight.Info().state == drive::straight_state::done ? 2 : 4;
      metrics.qualified = metrics.state == 2 && metrics.at_speed_ms >= 300;
    }
    info.straight = Straight.Info();
    info.target_rpm = info.running ? info.straight.motion.target_rpm : 0;
    if (info.running) { output.L_voltage = info.straight.left_mV; output.R_voltage = info.straight.right_mV; }
    return true;
  }
 private:
  static bool Positive(double v) { return std::isfinite(v) && v > 0; }
  bool ValidStage() const {
    const auto& s = stages[metrics.stage];
    return Positive(s.distance_mm) && Positive(s.maximum_rpm) && s.maximum_rpm <= 600 &&
        Positive(s.acceleration_rpm_s) && Positive(s.deceleration_rpm_s) &&
        Positive(s.position_gain_scale) && Positive(s.position_correction_scale);
  }
  void Select() {
    Straight.Reset();
    const unsigned int stage = metrics.stage;
    metrics = tuning_metrics(); metrics.stage = stage; metrics.direction = info.direction;
    info.request_mm = info.direction * stages[stage].distance_mm;
    info.maximum_rpm = stages[stage].maximum_rpm; info.status = "READY";
    info.straight = drive::straight_info(); info.target_rpm = 0;
  }
  void Cancel(double now) {
    if (info.running) {
      Straight.CancelDriveStraight(); metrics.state = 3;
      metrics.time_ms = now; metrics.elapsed_ms = now - started_at; metrics.qualified = false;
    }
    info.running = false; info.status = "CANCEL"; info.target_rpm = 0; info.straight = Straight.Info();
  }
  void Measure(const drive::straight_feedback& frame, double now) {
    metrics.time_ms = now; metrics.elapsed_ms = now - started_at;
    if (std::isfinite(frame.heading_deg))
      metrics.peak_heading_deg = std::max(metrics.peak_heading_deg, std::fabs(frame.heading_deg - initial_heading));
    if (!frame.velocity.valid || frame.velocity.sequence == last_sequence) return;
    const double dt = frame.velocity.time_ms - last_sample_time;
    last_sequence = frame.velocity.sequence; last_sample_time = frame.velocity.time_ms;
    if (!(dt > 0 && dt <= 100) || !std::isfinite(dt)) { consecutive = 0; return; }
    const double actual = info.direction * (frame.velocity.left_rpm + frame.velocity.right_rpm) / 2;
    metrics.peak_rpm = std::max(metrics.peak_rpm, std::max(0.0, actual));
    const auto& v = Straight.Info().velocity;
    if (v.active) {
      squared_error += (v.left_error * v.left_error + v.right_error * v.right_error) / 2 * dt;
      error_time += dt; metrics.speed_rms_rpm = std::sqrt(squared_error / error_time);
    }
    if (actual >= info.maximum_rpm * 0.9 && std::fabs(Straight.Info().motion.reference_rpm) >= info.maximum_rpm * 0.95)
      consecutive += dt;
    else consecutive = 0;
    metrics.at_speed_ms = std::max(metrics.at_speed_ms, consecutive);
    if (std::abs(Straight.Info().left_mV) >= 11500 || std::abs(Straight.Info().right_mV) >= 11500)
      metrics.high_voltage_ms += dt;
  }
  tuning_stage stages[4] = {};
  drive::straight_config baseline;
  drive::straight_controller Straight;
  test_info info;
  tuning_metrics metrics;
  unsigned int serial = 0, last_sequence = 0;
  double started_at = 0, initial_heading = 0, squared_error = 0, error_time = 0, consecutive = 0, last_sample_time = 0;
  bool pre_start = false, pre_up = false, pre_down = false, pre_positive = false, pre_negative = false, pre_allowed = false;
};
