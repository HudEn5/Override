#pragma once

#include <algorithm>
#include <cmath>

namespace drive {
struct heading_config {
  double kP = 80.0;       // mV/度，初始值需要实车验证。
  double kD = 10.0;       // mV/(度/秒)，抑制转向摆动。
  double rate_filter_ms = 60.0;
  double max_correction = 500.0;
  double max_fraction = 0.25;  // 修正不超过较小侧基础电压的四分之一。
  double max_error = 15.0;     // 偏离过大时停车，等待人工重新定向。
};

struct heading_info {
  double time_ms = 0.0;
  double angle = 0.0;
  double target = 0.0;
  double error = 0.0;
  double correction = 0.0;
  bool ready = false;
  bool locked = false;
  bool active = false;
};

// 连续惯导角逆时针为正；前进、停车、后退共用同一参考角。
class heading_hold {
 public:
  void Configure(const heading_config& value) { config = value; Reset(); }
  void Reset() { info = heading_info(); have_previous = false; rate = 0.0; }
  const heading_info& Info() const { return info; }

  // 停车只记录角度，不输出转向；重新行驶才修正停车产生的偏差。
  bool Update(bool running, bool ready, double angle, double now,
              int& left, int& right) {
    info.active = false;
    info.correction = 0.0;
    info.ready = ready && std::isfinite(angle) && std::isfinite(now);
    if (!info.ready) { Reset(); return false; }
    info.ready = true;
    info.time_ms = now;
    info.angle = angle;
    if (running && !info.locked) { info.target = angle; info.locked = true; }
    info.error = info.locked ? info.target - angle : 0.0;
    const double dt = now - previous_time;
    if (running && have_previous && dt > 0.0 && dt <= 200.0) {
      const double measured_rate = (angle - previous_angle) * 1000.0 / dt;
      rate += dt / (config.rate_filter_ms + dt) * (measured_rate - rate);
    } else rate = 0.0;
    previous_time = now;
    previous_angle = angle;
    have_previous = true;
    if (!running) return true;
    if (std::fabs(info.error) > config.max_error) return false;
    // 留出电压余量，同时避免修正使任一侧反转；正反行驶用同一转向符号。
    const double smaller = std::min(std::fabs(static_cast<double>(left)),
                                    std::fabs(static_cast<double>(right)));
    const double headroom = 12000.0 - std::max(std::fabs(static_cast<double>(left)),
                                              std::fabs(static_cast<double>(right)));
    const double limit = std::max(0.0, std::min(config.max_correction,
        std::min(config.max_fraction * smaller, headroom)));
    info.correction = std::max(-limit, std::min(limit, config.kP * info.error - config.kD * rate));
    const int correction = static_cast<int>(std::lround(info.correction));
    left -= correction;
    right += correction;
    info.active = true;
    return true;
  }

 private:
  heading_config config;
  heading_info info;
  bool have_previous = false;
  double previous_angle = 0.0;
  double previous_time = 0.0;
  double rate = 0.0;
};
}  // 底盘控制命名空间
