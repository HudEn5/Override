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
        assert(value.output.L_voltage == static_cast<int>(value.tick));
        assert(value.output.R_voltage == -static_cast<int>(value.tick));
        assert(value.dt == value.tick * 0.5 && value.test.request_deg == value.tick + 1.0);
        assert(value.test.corner.distance_mm == value.tick * 2.0);
#if BARTMOSS_CALIBRATION
        assert(value.calibration.target_rpm == value.tick * 3.0);
#endif
        reads++;
      }
    } while (!finished.load());
  };
  std::thread one(reader), two(reader);
  for (unsigned int i = 1; i <= 30000; ++i) {
    runtime_info value; value.configured = true; value.tick = i;
    value.output.L_voltage = i; value.output.R_voltage = -static_cast<int>(i);
    value.dt = i * 0.5; value.test.request_deg = i + 1.0; value.test.corner.distance_mm = i * 2.0;
#if BARTMOSS_CALIBRATION
    value.calibration.target_rpm = i * 3.0;
#endif
    PublishRuntimeInfo(value);
  }
  finished.store(true); one.join(); two.join();
  assert(reads.load() > 0);
  std::puts("Runtime snapshot checks passed: one publisher, two readers, coherent multi-field copies under a shared mutex.");
}
