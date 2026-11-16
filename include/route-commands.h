#pragma once

namespace route {
constexpr double DEFAULT_DRIVE_RPM = 330;
constexpr double DEFAULT_TURN_RPM = 140;
constexpr double DEFAULT_PUSH_VOLTS = 4.0;
enum class action { straight, turn, heading, corner, wait, push };
struct command {
  action kind;
  double first, second, third, speed;
};
// 单位统一为 mm、度、rpm、毫秒。正距离前进，正相对角逆时针。
inline command DriveStraight(double distance_mm, double maximum_rpm = DEFAULT_DRIVE_RPM) {
  return {action::straight, distance_mm, 0, 0, maximum_rpm};
}
inline command Turn(double angle_deg, double maximum_rpm = DEFAULT_TURN_RPM) {
  return {action::turn, angle_deg, 0, 0, maximum_rpm};
}
inline command TurnToHeading(double heading_deg, double maximum_rpm = DEFAULT_TURN_RPM) {
  return {action::heading, heading_deg, 0, 0, maximum_rpm};
}
// 一条动作连续通过直线、圆角、直线；两段独立直行加 Turn 会在中间停稳。
inline command DriveCorner(double before_mm, double angle_deg, double after_mm,
                           double maximum_rpm = DEFAULT_DRIVE_RPM) {
  return {action::corner, before_mm, angle_deg, after_mm, maximum_rpm};
}
inline command Wait(double milliseconds) { return {action::wait, milliseconds, 0, 0, 0}; }
// 只按时间前推用于机械定位；第二项可选，单位V；零时长跳过，单次最多5秒。
inline command PushFor(double milliseconds, double volts = DEFAULT_PUSH_VOLTS) {
  return {action::push, milliseconds, volts, 0, 0};
}
}  // 路线动作命名空间
