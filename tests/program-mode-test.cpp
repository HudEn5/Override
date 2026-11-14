#include "program-mode.h"
#include <cassert>
#include <cstdio>

int main() {
  using runtime::competition_stage;
  using runtime::program_mode;
  for (int mode = 0; mode < 3; ++mode)
    for (int stage = 0; stage < 3; ++stage)
      for (int bits = 0; bits < 64; ++bits) {
        const bool configured = bits & 1, field = bits & 2, enabled = bits & 4;
        const bool driver = bits & 8, autonomous = bits & 16, controller = bits & 32;
        const auto p = runtime::Permissions(static_cast<program_mode>(mode), configured, field,
            enabled, driver, autonomous, controller, static_cast<competition_stage>(stage));
        assert(!(p.manual && p.autonomous));
        if (!configured) assert(!p.manual && !p.tests && !p.calibration && !p.autonomous);
        if (field) assert(!p.tests && !p.calibration && !p.telemetry);
        if (field && !enabled) assert(!p.manual && !p.autonomous);
        if (mode != 1) assert(!p.autonomous);
        if (mode == 1) assert(!p.tests && !p.calibration && !p.telemetry);
        if (mode == 2) assert(!p.tests);
        if (mode != 2) assert(!p.calibration);
        assert(!(p.tests && p.calibration));
        if (!controller) assert(!p.manual && !p.tests && !p.calibration && !p.telemetry);
        if (p.autonomous) assert(field && enabled && autonomous && !driver && stage == 1);
        if (field && driver && autonomous) assert(!p.manual && !p.autonomous);
      }
  const auto no_controller = runtime::Permissions(program_mode::competition, true, true, true,
      false, true, false, competition_stage::autonomous);
  assert(no_controller.autonomous); // 比赛自动不会因手柄断连被变成手控或停止。
  const auto no_field = runtime::Permissions(program_mode::competition, true, false, true,
      false, true, true, competition_stage::autonomous);
  assert(no_field.manual && !no_field.autonomous && !no_field.tests);
  const auto early_callback = runtime::Permissions(program_mode::competition, true, true, false,
      false, true, true, competition_stage::autonomous);
  assert(!early_callback.autonomous); // 回调先到也不能绕过场控禁用。
  std::puts("Environment checks passed: 576 combinations, isolated test/calibration/match modes, callbacks and controller-independent autonomous.");
}
