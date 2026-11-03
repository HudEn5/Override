#pragma once

#include "vex.h"
#include "arm-pid.h"

// 智能端口填写主控上的编号 1～21；三线端口填写 'A'～'H'。
namespace config {
const char PROGRAM_VERSION[] = "1.0.0";
const char CONTROLLER_VERSION[] = "V1";  // 遥控器使用短版本号，保留电流显示空间。

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
// 手臂角度均指电机编码器角度；每次启动前需将手臂放回起始位置。
constexpr double ARM_MIN_ANGLE = 0.0;
const bool ARM_UPPER_LIMIT_ENABLED = false;  // 取消目标角度上限，仍保留 0° 下限
constexpr double ARM_MAX_ANGLE = 90.0;       // 备用上限，仅在启用上限开关后生效
constexpr double ARM_TARGET_SPEED = 60.0;  // 按住按钮时目标角度每秒改变的度数
// 平滑目标的速度上限，单位 rpm；换算为角度速度后用于目标滤波。
constexpr double ARM_MAX_VELOCITY = 10.0;
constexpr double ARM_TARGET_FILTER_TIME = 0.06;  // 目标一阶滤波时间常数，s
// 初始参数尚未实车整定；角度 PID 持续输出电压，不调用内置位置动作。
constexpr manual::ArmPIDConfig ARM_PID = {
    600.0, 40.0, 20.0,  // kp、ki、kd，时间单位统一为秒
    6000.0, 12000.0,    // 电压上限 6 V；每秒最多改变 12 V
    1500.0, 5.0, 0.08   // 积分项上限、积分区间、微分滤波时间常数
};
// 目标角最多领先实测角的度数；手臂被卡住时目标不会越推越远。
constexpr double ARM_MAX_LEAD = 10.0;
// 按住时连续多少个控制周期未累计取得有效方向位移，就判定为堵转。
constexpr int ARM_STALL_UPDATES = 50;
// 两侧手臂编码器允许的最大位置差；超过后视为异常并停止手臂。
constexpr double ARM_MAX_DIFF = 10.0;
// 夹爪仍使用 6 V 电压控制。
const int CLAW_VOLTAGE = 6000;
const int ARM_UP_DIRECTION = 1;      // 若上升方向相反，改为 -1
const int CLAW_CLOSE_DIRECTION = 1;  // 若夹紧方向相反，改为 -1
const vex::brakeType ARM_BRAKE_MODE = vex::hold;
const vex::brakeType CLAW_BRAKE_MODE = vex::hold;

// 仅预留惯性传感器端口；当前差速手控不需要惯性传感器。
const int INERTIAL_PORT = 0;

// 左摇杆上下控制前后运动；右摇杆左右控制转向。
const int YAxis = 3;
const int AngularAxis = 1;
const int JOYSTICK_DEADBAND = 5;  // 单位为百分比；死区外重新线性映射输出
const int MAX_VOLTAGE = 12000;
const int UPDATE_TIME = 20;
const vex::brakeType BRAKE_MODE = vex::coast;

enum class Button { R1, R2, L1, L2, A, B, X, Y, Up, Down, Left, Right };
// 按住控制机构运动；气动采用按下沿切换。
const Button ArmUp = Button::R1;
const Button ArmDown = Button::R2;
const Button ClawClose = Button::L1;
const Button ClawOpen = Button::L2;
const Button Toggle = Button::X;
}  // 配置命名空间
