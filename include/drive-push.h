#pragma once

#include "drive-straight.h"

namespace drive {
enum class push_state { idle, running, done, cancelled, error };
struct push_config {
  double maximum_duration_ms = 5000;
  double stale_ms = 100, max_update_ms = 200;
  double wheel_tolerance_rpm = 5, settle_ms = 100, settle_timeout_ms = 1500;
};
struct push_info {
  push_state state = push_state::idle;
  const char* status = "READY";
  double time_ms = 0, elapsed_ms = 0, duration_ms = 0;
  double left_rpm = 0, right_rpm = 0;
  bool settling = false;
  int left_mV = 0, right_mV = 0;
};

// 机械定位专用定时前推：固定同向电压，不做位置/速度/航向闭环，允许接触后停转。
// 只计算输出；时间到先撤电压，确认停稳后才把底盘交给下一个动作。
class push_controller {
 public:
  void Configure(const push_config& value) { config = value; Reset(); have_frame = have_sample = false; }
  const push_info& Info() const { return info; }
  void Observe(const straight_feedback& value, bool permitted, double now_ms) {
    frame = value; allowed = permitted; now = now_ms; have_frame = true;
    if (value.velocity.valid && std::isfinite(value.velocity.time_ms) &&
        (!have_sample || value.velocity.sequence != sequence)) {
      received_at = now_ms; sequence = value.velocity.sequence; have_sample = true;
    }
  }
  bool StartPushFor(double duration_ms, double volts) {
    if (info.state == push_state::running) return false;
    if (!Positive(config.maximum_duration_ms) || !Positive(config.stale_ms) || !Positive(config.max_update_ms) ||
        !Positive(config.wheel_tolerance_rpm) || !Nonnegative(config.settle_ms) || !Positive(config.settle_timeout_ms) ||
        !Nonnegative(duration_ms) || duration_ms > config.maximum_duration_ms || !Positive(volts) || volts < .001 || volts > 12)
      return Fail("PUSH CONFIG");
    if (!Check()) return false;
    if (!frame.stopped || std::fabs(frame.velocity.left_rpm) > config.wheel_tolerance_rpm ||
        std::fabs(frame.velocity.right_rpm) > config.wheel_tolerance_rpm) return Fail("MOVING");
    info = push_info(); info.duration_ms = duration_ms; info.time_ms = now;
    started_at = previous_time = now; settled_at = -1;
    voltage = static_cast<int>(std::lround(volts * 1000));
    info.state = duration_ms == 0 ? push_state::done : push_state::running;
    info.status = duration_ms == 0 ? "DONE" : "PUSH";
    Update(); return info.state != push_state::error;
  }
  void Update() {
    if (info.state != push_state::running) return;
    if (!Check()) return;
    if (now < previous_time || now - previous_time > config.max_update_ms) { Fail("CLOCK ERR"); return; }
    previous_time = now; info.time_ms = now; info.elapsed_ms = now - started_at;
    info.left_rpm = frame.velocity.left_rpm; info.right_rpm = frame.velocity.right_rpm;
    if (info.elapsed_ms < info.duration_ms) {
      info.left_mV = info.right_mV = voltage; info.status = "PUSH"; return;
    }
    info.left_mV = info.right_mV = 0; info.settling = true; info.status = "PUSH SETTLE";
    const bool stopped = frame.stopped && std::fabs(info.left_rpm) <= config.wheel_tolerance_rpm &&
                         std::fabs(info.right_rpm) <= config.wheel_tolerance_rpm;
    if (!stopped) settled_at = -1;
    else if (settled_at < 0) settled_at = now;
    if (stopped && now - settled_at >= config.settle_ms) {
      info.state = push_state::done; info.status = "DONE"; info.settling = false; return;
    }
    if (info.elapsed_ms >= info.duration_ms + config.settle_timeout_ms) Fail("PUSH STOP TIMEOUT");
  }
  void Cancel() {
    if (info.state == push_state::running) { info.state = push_state::cancelled; info.status = "CANCEL"; }
    info.left_mV = info.right_mV = 0; info.settling = false;
  }
  void Reset() { Cancel(); info = push_info(); }
 private:
  static bool Positive(double value) { return std::isfinite(value) && value > 0; }
  static bool Nonnegative(double value) { return std::isfinite(value) && value >= 0; }
  bool Check() {
    if (!have_frame || !allowed) return Fail("DISABLED");
    if (!Nonnegative(now)) return Fail("CLOCK ERR");
    if (!frame.motors_online) return Fail("MOTOR OFF");
    if (!frame.inertial_ready || !std::isfinite(frame.heading_deg)) return Fail("IMU ERROR");
    if (!frame.position.valid || !std::isfinite(frame.position.left_deg) || !std::isfinite(frame.position.right_deg))
      return Fail("POSITION ERR");
    if (!have_sample || !frame.velocity.valid || !Nonnegative(frame.velocity.time_ms) ||
        !std::isfinite(frame.velocity.left_rpm) || !std::isfinite(frame.velocity.right_rpm) ||
        now < received_at || now - received_at > config.stale_ms) return Fail("SPEED ERR");
    return true;
  }
  bool Fail(const char* reason) {
    info.state = push_state::error; info.status = reason; info.left_mV = info.right_mV = 0; info.settling = false;
    return false;
  }
  push_config config;
  push_info info;
  straight_feedback frame;
  bool allowed = false, have_frame = false, have_sample = false;
  unsigned int sequence = 0;
  int voltage = 0;
  double now = 0, started_at = 0, previous_time = 0, received_at = 0, settled_at = -1;
};
}  // 底盘动作命名空间
