#pragma once

#include "high-speed-control.h"
#include "motion-presets.h"
#include "route-commands.h"

namespace config {
// X测试模式：true为三项运动测试（默认CNR），false恢复H1～H4直行整定。
constexpr bool HIGH_SPEED_CORNER_TEST = true;
constexpr double HIGH_SPEED_CORNER_BEFORE_MM = 500;
constexpr double HIGH_SPEED_CORNER_DEG = 90;  // 正角逆时针，左键可选负角对照。
constexpr double HIGH_SPEED_CORNER_AFTER_MM = 500;
constexpr double HIGH_SPEED_CORNER_MAX_RPM = 330;  // 单侧目标上限，弯内由预测限速。
inline test_settings HighSpeedCornerSettings() {
  test_settings result;
  result.initial_motion = test_motion::corner;
  result.straight_mm = 500; result.straight_rpm = route::DEFAULT_DRIVE_RPM;
  result.turn_deg = 90; result.turn_rpm = route::DEFAULT_TURN_RPM;
  result.before_mm = HIGH_SPEED_CORNER_BEFORE_MM; result.corner_deg = HIGH_SPEED_CORNER_DEG;
  result.after_mm = HIGH_SPEED_CORNER_AFTER_MM; result.corner_rpm = HIGH_SPEED_CORNER_MAX_RPM;
  return result;
}
static_assert(HIGH_SPEED_CORNER_BEFORE_MM >= 0 && HIGH_SPEED_CORNER_AFTER_MM >= 0 &&
              HIGH_SPEED_CORNER_DEG > 0 && HIGH_SPEED_CORNER_DEG <= 180 &&
              HIGH_SPEED_CORNER_MAX_RPM > 0 && HIGH_SPEED_CORNER_MAX_RPM <= 600,
              "Invalid high-speed corner settings");
static_assert(HIGH_SPEED_CORNER_ACCELERATION > 0 && HIGH_SPEED_CORNER_DECELERATION > 0 &&
              HIGH_SPEED_CORNER_WHEEL_ACCELERATION > 0 && HIGH_SPEED_CORNER_WHEEL_DECELERATION > 0 &&
              HIGH_SPEED_CORNER_CRUISE_RPM > 0 && HIGH_SPEED_CORNER_CRUISE_RPM <= HIGH_SPEED_CORNER_MAX_RPM &&
              HIGH_SPEED_CORNER_ENTRY_RPM > 0 && HIGH_SPEED_CORNER_ENTRY_RPM <= HIGH_SPEED_CORNER_CRUISE_RPM &&
              HIGH_SPEED_CORNER_CURVE_ACCELERATION > 0 && HIGH_SPEED_CORNER_EXIT_ACCELERATION > 0,
              "Invalid high-speed corner limits");
// 距离 mm、最高电机输出轴 rpm、加速 rpm/s、减速 rpm/s。每次 A 仅执行一轮。
// H3 保留已验证基线；H4 同速度、同加减速，仅将位置比例增益和纠偏上限乘二。
// 后两项省略时均为 1；速度 PID、前馈、制动预留和停稳判定均沿用原配置。
const tuning_stage HIGH_SPEED_STAGES[4] = {
    {1500, 200, 240, 240},
    {1500, 265, 320, 320},
    {1800, 330, 320, 320},
    {1800, 330, 320, 320, 2, 2}
};
}  // 高速整定配置命名空间
