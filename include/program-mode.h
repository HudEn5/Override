#pragma once

namespace runtime {
enum class program_mode { test, competition, calibration };
enum class competition_stage { none, autonomous, driver };
struct permissions {
  bool manual = false;
  bool tests = false;
  bool calibration = false;
  bool autonomous = false;
  bool telemetry = false;
};

// 比赛自动不依赖手柄在线；测试动作和 Y 回传仅允许在无场控测试模式下使用。
inline permissions Permissions(program_mode mode, bool configured, bool field_connected,
                               bool enabled, bool driver, bool autonomous, bool controller,
                               competition_stage requested = competition_stage::none) {
  permissions result;
  result.manual = configured && controller && (!field_connected ||
      (enabled && driver && !autonomous && requested == competition_stage::driver));
  result.tests = configured && controller && !field_connected && mode == program_mode::test;
  result.calibration = configured && controller && !field_connected && mode == program_mode::calibration;
  result.telemetry = controller && !field_connected && mode != program_mode::competition;
  result.autonomous = configured && field_connected && enabled && autonomous && !driver &&
                      mode == program_mode::competition && requested == competition_stage::autonomous;
  return result;
}
}  // 运行环境命名空间
