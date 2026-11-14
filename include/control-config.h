#pragma once

#include "vex.h"
#include "drive-control.h"
#include "heading-control.h"
#include "velocity-control.h"
#include "trapezoid-motion.h"
#include "drive-straight.h"
#include "drive-turn.h"
#include "drive-corner.h"
#include "program-mode.h"

// 智能端口填写主控上的编号 1～21；三线端口填写 'A'～'H'。
namespace config {
const char PROGRAM_VERSION[] = "1.6.0-tests";
const char CONTROLLER_VERSION[] = "VLT";  // 遥控器标明纯电压版本。
// 默认日常测试；标定与比赛分别通过独立构建参数启用。
#ifndef BARTMOSS_COMPETITION
#define BARTMOSS_COMPETITION 0
#endif
#ifndef BARTMOSS_CALIBRATION
#define BARTMOSS_CALIBRATION 0
#endif
#if (BARTMOSS_COMPETITION != 0 && BARTMOSS_COMPETITION != 1) || (BARTMOSS_CALIBRATION != 0 && BARTMOSS_CALIBRATION != 1)
#error Invalid Bartmoss environment
#endif
#if BARTMOSS_COMPETITION && BARTMOSS_CALIBRATION
#error Competition and calibration cannot be enabled together
#endif
constexpr runtime::program_mode PROGRAM_MODE = BARTMOSS_COMPETITION ? runtime::program_mode::competition :
    BARTMOSS_CALIBRATION ? runtime::program_mode::calibration : runtime::program_mode::test;

struct Motor {
  int port;
  vex::gearSetting gear;
  bool reversed;
};

// 底盘端口、齿轮比和反转方向按本赛季提供的设备配置填写。
const Motor LF = {12, vex::ratio6_1, true};
const Motor LM = {1, vex::ratio6_1, false};
const Motor LBD = {11, vex::ratio6_1, true};
const Motor LBU = {13, vex::ratio6_1, false};
const Motor RF = {6, vex::ratio6_1, false};
const Motor RM = {5, vex::ratio6_1, true};
const Motor RBD = {9, vex::ratio6_1, false};
const Motor RBU = {10, vex::ratio6_1, true};
const Motor LEFT_DRIVE[4] = {LF, LM, LBD, LBU};
const Motor RIGHT_DRIVE[4] = {RF, RM, RBD, RBU};

// 双电机手臂和夹爪；两侧手臂接收同一指令，反转方向由各自配置处理。
const Motor ArmL = {3, vex::ratio36_1, false};
const Motor ArmR = {2, vex::ratio36_1, true};
const Motor Claw = {15, vex::ratio18_1, true};  // 小电机
const Motor STRUCTURE_MOTORS[3] = {ArmL, ArmR, Claw};
const char TogglePiston = 'H';
const bool PISTON_INITIAL_STATE = false;
// 按键直接给电压，无目标角度、滤波、软件 PID 或电压渐变。
// 两个值均填写正数，单位 mV；动作方向由 ARM_UP_DIRECTION 统一处理。
constexpr int ARM_UP_VOLTAGE = 12000;
constexpr int ARM_DOWN_VOLTAGE = 4000;
// 夹爪仍使用 6 V 电压控制。
const int CLAW_VOLTAGE = 6000;
const int ARM_UP_DIRECTION = 1;      // 若上升方向相反，改为 -1
const int CLAW_CLOSE_DIRECTION = 1;  // 若夹紧方向相反，改为 -1
const vex::brakeType ARM_BRAKE_MODE = vex::hold;
const vex::brakeType CLAW_BRAKE_MODE = vex::hold;

// 惯导启动后静止校准，逆时针角度为正。
const int INERTIAL_PORT = 4;
// 驱动轮 2.75 英寸，单位换算为 mm。
constexpr double DRIVE_WHEEL_DIAMETER_MM = 69.85;
// 电机减速箱输出轴转数 / 轮子转数。直接连接为 1，外部再减速 3:1 为 3。
constexpr double DRIVE_MOTOR_REVS_PER_WHEEL_REV = 36.0 / 30.0;
constexpr double AUTO_TURN_KP = 1.6;  // rpm/deg；转向专用，直行航向增益不变。
constexpr double AUTO_TURN_KD = 0.2;  // rpm/(deg/s)
constexpr double AUTO_ANGLE_TOLERANCE = 0.8;  // deg

// 左摇杆上下控制前后运动；右摇杆左右控制转向。
const int YAxis = 3;
const int AngularAxis = 1;
const int JOYSTICK_DEADBAND = 5;  // 单位为百分比；死区外重新线性映射输出
const int MAX_VOLTAGE = 12000;
const int UPDATE_TIME = 10;  // 缩短输入采样和输出更新的等待时间，实际周期可在屏幕查看。
constexpr int TELEMETRY_INTERVAL_MS = 200;  // 回传最多 5 Hz；内部速度采样仍为约 50 Hz。
constexpr bool ARM_SERIAL_LOG = false;  // 默认关闭 BMV 手臂日志，避免挤占无线带宽。
const drive::heading_config DRIVE_HEADING = drive::heading_config();
const drive::velocity_pid_config DRIVE_VELOCITY_PID = drive::velocity_pid_config();
inline drive::trapezoid_config StraightMotion() {
  drive::trapezoid_config result;
  result.wheel_diameter_mm = DRIVE_WHEEL_DIAMETER_MM;
  result.motor_revs_per_wheel_rev = DRIVE_MOTOR_REVS_PER_WHEEL_REV;
  return result;
}
// 2026-11-12 根据当前硬件三轮 FF 验证更新：18 段、158 条运行后段样本。
// 左右与前进/后退分别拟合，保留四台电机平均值，没有剔除 LM。
// 已完成当前场地 100/150/200 rpm 的 SP 基础测试；跨场地一致性待验证。
inline drive::feedforward_config Feedforward(double kS, double kV) {
  drive::feedforward_config result;
  result.kS = kS;
  result.kV = kV;
  result.kA = 0.0;
  result.max_voltage = MAX_VOLTAGE;
  result.calibrated = true;  // 允许测试控制器计算；不表示完成了独立实车验证。
  return result;
}
const drive::feedforward_config FF_LEFT = Feedforward(372.573735, 17.900544);
const drive::feedforward_config FF_RIGHT = Feedforward(408.162564, 16.537990);
const drive::feedforward_config FF_LEFT_REVERSE = Feedforward(547.829766, 16.939618);
const drive::feedforward_config FF_RIGHT_REVERSE = Feedforward(537.033135, 15.870378);
inline drive::straight_config StraightControl() {
  drive::straight_config result;
  result.motion = StraightMotion();
  result.heading = DRIVE_HEADING;
  result.pid = DRIVE_VELOCITY_PID;
  result.left_forward = FF_LEFT; result.right_forward = FF_RIGHT;
  result.left_reverse = FF_LEFT_REVERSE; result.right_reverse = FF_RIGHT_REVERSE;
  return result;
}
inline drive::turn_config TurnControl() {
  drive::turn_config result;
  result.kP = AUTO_TURN_KP; result.kD = AUTO_TURN_KD;
  result.angle_tolerance_deg = AUTO_ANGLE_TOLERANCE;
  result.pid = DRIVE_VELOCITY_PID;
  // 四轮/侧原地转动的阻力高于直行；仅转向放宽补偿并加快积分。
  result.pid.kP = 20; result.pid.kI = 20;
  result.pid.max_correction = 2000; result.pid.max_integral = 1200;
  result.correction_fraction = 3.0;
  result.acceleration_rpm_s = 240;
  result.minimum_settle_rpm = 18;  // 尾段加快，最后 6° 内渐变回 12 rpm。
  result.minimum_finish_rpm = 12;
  result.minimum_rate_deg_s = 16;
  result.left_forward = FF_LEFT; result.right_forward = FF_RIGHT;
  result.left_reverse = FF_LEFT_REVERSE; result.right_reverse = FF_RIGHT_REVERSE;
  return result;
}
inline drive::corner_config CornerControl() {
  drive::corner_config result;
  result.wheel_diameter_mm = DRIVE_WHEEL_DIAMETER_MM;
  result.motor_revs_per_wheel_rev = DRIVE_MOTOR_REVS_PER_WHEEL_REV;
  result.track_width_mm = 325;  // 用户实测左右驱动轮接地中心线间距。
  result.radius_mm = 250;      // 参考曲率峰值对应半径；曲率渐入渐出。
  result.corner_rpm = 140;  // 弯内上限；实际按航向跟随情况降速，不能当作实车已验证速度。
  result.corner_floor_rpm = 100;  // 入弯偏好及反馈降速下限；预测限速可以低于此值。
  result.prediction_enabled = true;
  result.prediction_wheel_acceleration_rpm_s = 180;  // 预测用目标变化上限，尚非实测机械极限。
  result.prediction_wheel_deceleration_rpm_s = 120;
  result.prediction_margin = 0.85;  // 留出 15% 供航向修正和模型偏差使用。
  result.corner_acceleration_rpm_s = 60;
  result.entry_braking_delay_ms = 200;
  result.acceleration_rpm_s = 180;
  // 连续圆角终点容差按用户接受标准设为 1.5°；原地转向仍为 0.8°。
  result.angle_tolerance_deg = 1.5;
  result.exit_angle_tolerance_deg = 1.5;
  result.exit_rate_tolerance_deg_s = 4;
  result.finish_min_rpm = 12;  // 最末端保留已验证的低速，较远的终点接近段加快。
  result.finish_max_rpm = 30;
  result.finish_kP = 0.6;
  result.pid = TurnControl().pid;  // 整条连续路线使用同一个速度环，不在切点重置。
  result.left_forward = FF_LEFT; result.right_forward = FF_RIGHT;
  result.left_reverse = FF_LEFT_REVERSE; result.right_reverse = FF_RIGHT_REVERSE;
  return result;
}
const vex::brakeType BRAKE_MODE = vex::coast;

enum class Button { R1, R2, L1, L2, A, B, X, Y, Up, Down, Left, Right };
// 按住控制机构运动；气动采用按下沿切换。
const Button ArmUp = Button::R1;
const Button ArmDown = Button::R2;
const Button ClawClose = Button::L1;
const Button ClawOpen = Button::L2;
const Button Toggle = Button::X;
const Button TestStart = Button::A;
const Button TestCancel = Button::B;
const Button Telemetry = Button::Y;  // 按一次开始回传，再按一次停止。
const Button TestNext = Button::Up;
const Button TestPrevious = Button::Down;
const Button TestPositive = Button::Right;
const Button TestNegative = Button::Left;
}  // 配置命名空间
