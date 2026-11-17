#include "runtime-info.h"
#include <atomic>
#include <cassert>
#include <cstdio>
#include <thread>

int main() {
  std::atomic<bool> finished(false);
  std::atomic<unsigned int> reads(0);
  const auto reader = [&] {
    do {
      const auto value = ReadRuntimeInfo();
      if (value.configured) {
        assert(value.output.left_mV == static_cast<int>(value.tick));
        assert(value.output.right_mV == -static_cast<int>(value.tick));
        assert(value.dt == value.tick * 0.5 && value.test.request_deg == value.tick + 1.0);
        assert(value.test.corner.distance_mm == value.tick * 2.0);
        assert(value.pose.x_mm == value.tick * 5.0 && value.pose.y_mm == -(value.tick * 2.0));
        assert(value.pose.heading_deg == value.tick * .25 && value.pose.valid && !value.pose.needs_reset);
#if BARTMOSS_CALIBRATION
        assert(value.calibration.target_rpm == value.tick * 3.0);
#endif
#if BARTMOSS_HIGH_SPEED
        assert(value.tuning.peak_rpm == value.tick * 4.0);
#endif
        reads++;
      }
    } while (!finished.load());
  };
  std::thread one(reader), two(reader);
  for (unsigned int i = 1; i <= 30000; ++i) {
    runtime_info value;
    value.configured = true;
    value.tick = i;
    value.output.left_mV = i;
    value.output.right_mV = -static_cast<int>(i);
    value.dt = i * 0.5;
    value.test.request_deg = i + 1.0;
    value.test.corner.distance_mm = i * 2.0;
    value.pose.x_mm = i * 5.0;
    value.pose.y_mm = -static_cast<double>(i) * 2;
    value.pose.heading_deg = i * .25;
    value.pose.valid = true;
#if BARTMOSS_CALIBRATION
    value.calibration.target_rpm = i * 3.0;
#endif
#if BARTMOSS_HIGH_SPEED
    value.tuning.peak_rpm = i * 4.0;
#endif
    PublishRuntimeInfo(value);
  }
  finished.store(true);
  one.join();
  two.join();
  assert(reads.load() > 0);
  std::puts(
      "Runtime snapshot checks passed: one publisher, two readers, coherent multi-field copies under a "
      "shared mutex.");
}
