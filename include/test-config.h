#pragma once

#include "test-control.h"

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
