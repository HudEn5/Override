#pragma once

#include "test-control.h"
#include "high-speed-control.h"
#include "performance-config.h"
#include "autonomous.h"

namespace config {
// 仅标定环境使用，日常测试不加载这些档位或长按手势。
constexpr int CALIBRATION_VOLTAGES[3] = {2000, 3000, 4000};
constexpr int CALIBRATION_TARGETS[3] = {100, 150, 200};
constexpr int CALIBRATION_TIMEOUT_MS = 3000;
constexpr int CALIBRATION_MODE_HOLD_MS = 1000;
}  // 标定参数命名空间

namespace config {
// 日常测试只改这里。直行距离为正数，方向由左/右键选择；正角逆时针。
constexpr double TEST_STRAIGHT_MM = 500;
constexpr double TEST_STRAIGHT_RPM = 100;
constexpr double TEST_TURN_DEG = 90;
constexpr double TEST_TURN_RPM = 100;
constexpr double TEST_CORNER_BEFORE_MM = 500;
constexpr double TEST_CORNER_DEG = 30;
constexpr double TEST_CORNER_AFTER_MM = 500;
constexpr double TEST_CORNER_RPM = 600;
inline test_settings TestSettings() {
  test_settings result;
  result.straight_mm = TEST_STRAIGHT_MM; result.straight_rpm = TEST_STRAIGHT_RPM;
  result.turn_deg = TEST_TURN_DEG; result.turn_rpm = TEST_TURN_RPM;
  result.before_mm = TEST_CORNER_BEFORE_MM; result.corner_deg = TEST_CORNER_DEG;
  result.after_mm = TEST_CORNER_AFTER_MM; result.corner_rpm = TEST_CORNER_RPM;
  return result;
}
static_assert(TEST_STRAIGHT_MM > 0 && TEST_STRAIGHT_RPM > 0 && TEST_STRAIGHT_RPM <= 600,
              "Invalid straight test settings");
static_assert(TEST_TURN_DEG > 0 && TEST_TURN_RPM > 0 && TEST_TURN_RPM <= 600,
              "Invalid turn test settings");
static_assert(TEST_CORNER_BEFORE_MM >= 0 && TEST_CORNER_AFTER_MM >= 0 &&
              TEST_CORNER_DEG > 0 && TEST_CORNER_DEG <= 180 && TEST_CORNER_RPM > 0 && TEST_CORNER_RPM <= 600,
              "Invalid corner test settings");
}  // 测试参数命名空间

namespace config {
// X测试模式：true为三项运动测试（默认CNR），false恢复H1～H4直行整定。
constexpr bool HIGH_SPEED_CORNER_TEST = true;
constexpr double HIGH_SPEED_CORNER_BEFORE_MM = 500;
constexpr double HIGH_SPEED_CORNER_DEG = 90;  // 正角逆时针，左键可选负角对照。
constexpr double HIGH_SPEED_CORNER_AFTER_MM = 500;
constexpr double HIGH_SPEED_CORNER_MAX_RPM = route::DEFAULT_DRIVE_RPM;  // 单侧目标上限，弯内由预测限速。
inline test_settings HighSpeedCornerSettings() {
  test_settings result;
  result.initial_motion = test_motion::corner;
  result.straight_mm = 1500; result.straight_rpm = route::DEFAULT_DRIVE_RPM;
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
