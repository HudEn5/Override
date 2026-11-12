#pragma once

#include "vex.h"
#include "drive-control.h"
#include "heading-control.h"
#include "velocity-control.h"
#include "trapezoid-motion.h"

// 智能端口填写主控上的编号 1～21；三线端口填写 'A'～'H'。
namespace config {
const char PROGRAM_VERSION[] = "1.1.0-voltage";
const char CONTROLLER_VERSION[] = "VLT";  // 遥控器标明纯电压版本。

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
constexpr double SQUARE_SIDE_MM = 500.0;
constexpr double AUTO_DRIVE_RPM = 35.0;
constexpr double AUTO_TURN_RPM = 30.0;
constexpr double AUTO_TURN_FAST_RPM = 60.0;  // 大角度转向阶段最高转速
constexpr double AUTO_TURN_SLOW_ANGLE = 25.0;  // 剩余角度小于该值时使用精调参数
constexpr double AUTO_TURN_KP = 0.8;  // rpm/deg
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
constexpr int DRIVE_TEST_VOLTAGES[3] = {2000, 3000, 4000};  // 固定电压测试档位，单位 mV。
constexpr int DRIVE_TEST_TIMEOUT_MS = 3000;  // VTEST/FH/SP 最多 3 秒；TP 按规划时长加余量。
constexpr int FF_TEST_TARGETS[3] = {100, 150, 200};  // 电机输出轴 rpm。
constexpr int TEST_MODE_HOLD_MS = 1000;  // 停车长按下方向键切换测试模式。
// 前馈测试同时保持航向；停车不原地纠偏，重新起步仍沿用本轮最初方向。
const drive::heading_config FF_HEADING = drive::heading_config();
const drive::velocity_pid_config DRIVE_VELOCITY_PID = drive::velocity_pid_config();
inline drive::trapezoid_config TrapezoidTest() {
  drive::trapezoid_config result;
  result.wheel_diameter_mm = DRIVE_WHEEL_DIAMETER_MM;
  result.motor_revs_per_wheel_rev = DRIVE_MOTOR_REVS_PER_WHEEL_REV;
  result.distance_mm = SQUARE_SIDE_MM;  // TP 每段前进/后退 500 mm。
  return result;
}
const drive::trapezoid_config TP_TEST = TrapezoidTest();
// 2026-11-12 根据当前硬件三轮 FF 验证更新：18 段、158 条运行后段样本。
// 左右与前进/后退分别拟合，保留四台电机平均值，没有剔除 LM。
// 已完成当前场地 100/150/200 rpm 的 SP 基础测试；跨场地一致性待验证。
inline drive::feedforward_config TestFeedforward(double kS, double kV) {
  drive::feedforward_config result;
  result.kS = kS;
  result.kV = kV;
  result.kA = 0.0;
  result.max_voltage = MAX_VOLTAGE;
  result.calibrated = true;  // 允许测试控制器计算；不表示完成了独立实车验证。
  return result;
}
const drive::feedforward_config FF_TEST_LEFT = TestFeedforward(372.573735, 17.900544);
const drive::feedforward_config FF_TEST_RIGHT = TestFeedforward(408.162564, 16.537990);
const drive::feedforward_config FF_TEST_LEFT_REVERSE = TestFeedforward(547.829766, 16.939618);
const drive::feedforward_config FF_TEST_RIGHT_REVERSE = TestFeedforward(537.033135, 15.870378);
const vex::brakeType BRAKE_MODE = vex::coast;

enum class Button { R1, R2, L1, L2, A, B, X, Y, Up, Down, Left, Right };
// 按住控制机构运动；气动采用按下沿切换。
const Button ArmUp = Button::R1;
const Button ArmDown = Button::R2;
const Button ClawClose = Button::L1;
const Button ClawOpen = Button::L2;
const Button Toggle = Button::X;
const Button TurnTest = Button::A;
const Button SquareTest = Button::B;
const Button Telemetry = Button::Y;  // 按一次开始回传，再按一次停止。
const Button VoltageUp = Button::Up;
const Button VoltageDown = Button::Down;
const Button VoltageForward = Button::Right;
const Button VoltageReverse = Button::Left;
}  // 配置命名空间
