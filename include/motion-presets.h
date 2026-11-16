#pragma once

#include "drive-straight.h"
#include "drive-corner.h"

namespace config {
// 路线与高速测试共用同一套参数，普通低速配置不受影响。
constexpr double HIGH_SPEED_STRAIGHT_ACCELERATION = 320;
inline drive::straight_config HighSpeedStraightControl(const drive::straight_config& baseline) {
  auto result = baseline;
  result.motion.acceleration_rpm_s = HIGH_SPEED_STRAIGHT_ACCELERATION;
  result.motion.deceleration_rpm_s = HIGH_SPEED_STRAIGHT_ACCELERATION;
  return result;  // H3 基线，不采用尚有异常测速的新 H4 位置增益。
}
constexpr double HIGH_SPEED_CORNER_ACCELERATION = 320;
constexpr double HIGH_SPEED_CORNER_DECELERATION = 320;
constexpr double HIGH_SPEED_CORNER_WHEEL_ACCELERATION = 320;  // 预测预算，并非实测机械极限。
constexpr double HIGH_SPEED_CORNER_WHEEL_DECELERATION = 320;
constexpr double HIGH_SPEED_CORNER_CRUISE_RPM = 180;
constexpr double HIGH_SPEED_CORNER_ENTRY_RPM = 130;
constexpr double HIGH_SPEED_CORNER_CURVE_ACCELERATION = 120;
constexpr double HIGH_SPEED_CORNER_EXIT_ACCELERATION = 240;
inline drive::corner_config HighSpeedCornerControl(const drive::corner_config& baseline) {
  auto result = baseline;
  result.acceleration_rpm_s = HIGH_SPEED_CORNER_ACCELERATION;
  result.deceleration_rpm_s = HIGH_SPEED_CORNER_DECELERATION;
  result.prediction_enabled = true;
  result.prediction_wheel_acceleration_rpm_s = HIGH_SPEED_CORNER_WHEEL_ACCELERATION;
  result.prediction_wheel_deceleration_rpm_s = HIGH_SPEED_CORNER_WHEEL_DECELERATION;
  result.corner_rpm = HIGH_SPEED_CORNER_CRUISE_RPM;
  result.corner_floor_rpm = HIGH_SPEED_CORNER_ENTRY_RPM;
  result.corner_acceleration_rpm_s = HIGH_SPEED_CORNER_CURVE_ACCELERATION;
  result.exit_acceleration_rpm_s = HIGH_SPEED_CORNER_EXIT_ACCELERATION;
  return result;
}
}  // 运动配置命名空间
