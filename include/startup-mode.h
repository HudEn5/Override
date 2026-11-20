#pragma once

#include "driver-control-logic.h"
#include "drive-straight.h"
#include <cmath>

namespace runtime {
// 本地自动菜单要求定位可用；X分项距离/转向仍只依赖自身反馈，不自动清除失效坐标。
inline const char* AutomaticStartStatus(const drive::straight_feedback& frame, const drive::pose& pose,
                                        double now) {
  if (const char* status = drive::StartStatus(frame))
    return status;
  if (pose.needs_reset)
    return "POSE RESET:RESTART";
  if (!pose.valid || !std::isfinite(pose.x_mm) || !std::isfinite(pose.y_mm) ||
      !std::isfinite(pose.heading_deg) || !std::isfinite(pose.time_ms) || !std::isfinite(now) ||
      pose.time_ms < 0 || now < pose.time_ms || now - pose.time_ms > 200)
    return "POSE WAIT";
  return nullptr;
}
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
inline permissions Permissions(program_mode mode, bool configured, bool field_connected, bool enabled,
                               bool driver, bool autonomous, bool controller,
                               competition_stage requested = competition_stage::none) {
  permissions result;
  result.manual =
      configured && controller &&
      (!field_connected || (enabled && driver && !autonomous && requested == competition_stage::driver));
  result.tests = configured && controller && !field_connected && mode == program_mode::test;
  result.calibration = configured && controller && !field_connected && mode == program_mode::calibration;
  result.telemetry = controller && !field_connected && mode != program_mode::competition;
  result.autonomous = configured && field_connected && enabled && autonomous && !driver &&
                      mode == program_mode::competition && requested == competition_stage::autonomous;
  return result;
}
}  // 运行环境命名空间

namespace runtime {
enum class local_mode { select, manual, automatic, test };

// 主循环独占此状态。A/B/X只在菜单选择模式，选择按键不会传递到机构和测试。
class startup_mode {
 public:
  local_mode Mode() const {
    return mode;
  }
  bool Armed() const {
    return armed;
  }
  bool Automatic() const {
    return mode == local_mode::automatic && armed && automatic_issued;
  }
  bool ReturningToManual() const {
    return finished_since >= 0;
  }
  double ReturnRemainingMs() const {
    return return_remaining;
  }
  void Update(const manual::input& input, bool available, bool field_connected, bool sticks, bool stopped,
              bool auto_ready, bool route_finished, double now) {
    const bool keys = input.test_start || input.test_cancel || input.toggle;
    if (!available || field_connected || !std::isfinite(now) || now < previous_time) {
      Reset();
      previous_time = std::isfinite(now) ? now : 0;
      return;
    }
    previous_time = now;
    if (mode == local_mode::select) {
      if (release_required) {
        if (!keys && !sticks)
          release_required = false;
        return;
      }
      const int count = static_cast<int>(input.test_start) + input.test_cancel + input.toggle;
      if (count != 1) {
        if (count > 1)
          release_required = true;
        return;
      }
      if (input.test_start && !auto_ready) {
        release_required = true;
        return;
      }
      mode = input.test_start    ? local_mode::automatic
             : input.test_cancel ? local_mode::manual
                                 : local_mode::test;
      armed = automatic_issued = false;
      hold_since = finished_since = -1;
      return_remaining = 0;
      return;
    }
    if (mode == local_mode::automatic && input.test_cancel) {
      mode = local_mode::manual;
      armed = automatic_issued = false;
      hold_since = finished_since = -1;
      return_remaining = 0;
      return;
    }
    // 选择用X不切气动，选择用A不启动测试；先松开按键并回中摇杆。
    if (!armed) {
      if (!keys && !sticks)
        armed = true;
      else
        return;
    }
    if (mode == local_mode::automatic) {
      if (sticks) {
        mode = local_mode::manual;
        armed = !input.test_cancel;
        automatic_issued = false;
        hold_since = finished_since = -1;
        return_remaining = 0;
        return;
      }
      // 与Tungsten AutoRoute一致：本地自动结束，停车等3秒再进入手动，不阻塞主循环。
      if (automatic_issued && route_finished) {
        if (finished_since < 0)
          finished_since = now;
        return_remaining = std::fmax(0.0, 3000 - (now - finished_since));
        if (return_remaining == 0) {
          mode = local_mode::manual;
          automatic_issued = false;
          finished_since = -1;
        }
        return;
      }
      finished_since = -1;
      return_remaining = 0;
      // 先实际发出运行许可，才接受finished，避免旧路线终态跳过新一轮。
      if (auto_ready || automatic_issued)
        automatic_issued = true;
      return;
    }
    // B在测试里先取消动作；停车后继续按住一秒才回菜单，无需组合键。
    if (input.test_cancel && stopped && !sticks) {
      if (hold_since < 0)
        hold_since = now;
      if (now - hold_since >= 1000)
        Reset();
    } else
      hold_since = -1;
  }

 private:
  void Reset() {
    mode = local_mode::select;
    armed = automatic_issued = false;
    release_required = true;
    hold_since = finished_since = -1;
    return_remaining = 0;
  }
  local_mode mode = local_mode::select;
  bool armed = false, automatic_issued = false, release_required = true;
  double hold_since = -1, previous_time = 0, finished_since = -1, return_remaining = 0;
};

// 无场控由菜单分配权限；场控接入后直接使用比赛权限，不依赖菜单或手柄在线。
inline permissions SessionPermissions(const startup_mode& selection, bool configured, bool field_connected,
                                      bool enabled, bool driver, bool autonomous, bool controller,
                                      competition_stage requested) {
  if (field_connected)
    return Permissions(program_mode::competition, configured, true, enabled, driver, autonomous, controller,
                       requested);
  permissions result;
  if (!configured || !controller)
    return result;
  result.telemetry = true;  // 可在菜单先打开LOG，记录自动起步；此权限不允许电机动作。
  if (!selection.Armed())
    return result;
  result.manual = selection.Mode() == local_mode::manual || selection.Mode() == local_mode::test;
  result.tests = selection.Mode() == local_mode::test;
  result.autonomous = selection.Automatic();
  return result;
}
}  // 运行模式命名空间
