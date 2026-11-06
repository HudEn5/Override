#pragma once

#include "vex.h"

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
constexpr int ARM_DOWN_VOLTAGE = 6000;
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
const int UPDATE_TIME = 10;  // 缩短输入采样和输出更新的等待时间，实际周期可在屏幕查看。
const vex::brakeType BRAKE_MODE = vex::coast;

enum class Button { R1, R2, L1, L2, A, B, X, Y, Up, Down, Left, Right };
// 按住控制机构运动；气动采用按下沿切换。
const Button ArmUp = Button::R1;
const Button ArmDown = Button::R2;
const Button ClawClose = Button::L1;
const Button ClawOpen = Button::L2;
const Button Toggle = Button::X;
}  // 配置命名空间
