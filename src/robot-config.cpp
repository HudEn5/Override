#include "robot-config.h"
#include "control-config.h"

vex::brain Brain;
vex::controller Controller(vex::primary);
vex::competition Competition;

vex::motor* LF = nullptr;
vex::motor* LM = nullptr;
vex::motor* LBD = nullptr;
vex::motor* LBU = nullptr;
vex::motor* RF = nullptr;
vex::motor* RM = nullptr;
vex::motor* RBD = nullptr;
vex::motor* RBU = nullptr;
vex::motor* ArmL = nullptr;
vex::motor* ArmR = nullptr;
vex::motor* Claw = nullptr;
vex::digital_out* TogglePiston = nullptr;

namespace {
// 统一遍历具名设备；通过指针地址表写回各设备变量。
vex::motor** const LeftDrive[4] = {&LF, &LM, &LBD, &LBU};
vex::motor** const RightDrive[4] = {&RF, &RM, &RBD, &RBU};
bool arm_active = false;
manual::arm_pid ArmL_PID(config::ARM_PID);
manual::arm_pid ArmR_PID(config::ARM_PID);
manual::arm_target_filter ArmTarget;
double arm_filtered_target = 0.0;
double arm_L_voltage = 0.0;
double arm_R_voltage = 0.0;

bool RegisterPort(int port, bool required, bool (&used)[22]) {
  if (port == 0 && !required) return true;
  if (port < 1 || port > 21) {
    Brain.Screen.print("Assign smart ports 1-21");
    return false;
  }
  if (used[port]) {
    Brain.Screen.print("Duplicate smart port: %d", port);
    return false;
  }
  used[port] = true;
  return true;
}

vex::motor* CreateMotor(const config::Motor& setting, vex::brakeType brake) {
  if (setting.port == 0) return nullptr;
  // SDK 端口索引从 0 开始；配置文件使用主控上标注的端口编号。
  vex::motor* motor = new vex::motor(setting.port - 1, setting.gear, setting.reversed);
  motor->setStopping(brake);
  motor->stop();
  return motor;
}

vex::triport::port& ThreeWirePort(char port) {
  switch (port) {
    case 'A': return Brain.ThreeWirePort.A;
    case 'B': return Brain.ThreeWirePort.B;
    case 'C': return Brain.ThreeWirePort.C;
    case 'D': return Brain.ThreeWirePort.D;
    case 'E': return Brain.ThreeWirePort.E;
    case 'F': return Brain.ThreeWirePort.F;
    case 'G': return Brain.ThreeWirePort.G;
    default: return Brain.ThreeWirePort.H;  // 仅在端口校验通过后调用。
  }
}

int ReadAxis(int axis) {
  switch (axis) {
    case 1: return Controller.Axis1.position();
    case 2: return Controller.Axis2.position();
    case 3: return Controller.Axis3.position();
    case 4: return Controller.Axis4.position();
    default: return 0;
  }
}

bool Pressed(config::Button button) {
  switch (button) {
    case config::Button::R1: return Controller.ButtonR1.pressing();
    case config::Button::R2: return Controller.ButtonR2.pressing();
    case config::Button::L1: return Controller.ButtonL1.pressing();
    case config::Button::L2: return Controller.ButtonL2.pressing();
    case config::Button::A: return Controller.ButtonA.pressing();
    case config::Button::B: return Controller.ButtonB.pressing();
    case config::Button::X: return Controller.ButtonX.pressing();
    case config::Button::Y: return Controller.ButtonY.pressing();
    case config::Button::Up: return Controller.ButtonUp.pressing();
    case config::Button::Down: return Controller.ButtonDown.pressing();
    case config::Button::Left: return Controller.ButtonLeft.pressing();
    case config::Button::Right: return Controller.ButtonRight.pressing();
  }
  return false;
}

void SetVoltage(vex::motor* motor, int millivolts) {
  if (!motor) return;
  if (millivolts == 0) motor->stop();
  else motor->spin(vex::fwd, millivolts, vex::voltageUnits::mV);
}
}  // 匿名命名空间

namespace hardware {
bool Initialize() {
  // 先检查全部配置，再创建设备或写入输出。
  bool used[22] = {};
  Brain.Screen.setCursor(2, 1);
  for (int i = 0; i < 4; ++i) {
    if (!RegisterPort(config::LEFT_DRIVE[i].port, true, used) ||
        !RegisterPort(config::RIGHT_DRIVE[i].port, true, used)) return false;
  }
  for (int i = 0; i < 3; ++i) {
    if (!RegisterPort(config::STRUCTURE_MOTORS[i].port, true, used)) return false;
  }
  if (!RegisterPort(config::INERTIAL_PORT, false, used)) return false;

  if (config::TogglePiston < 'A' || config::TogglePiston > 'H') {
    Brain.Screen.print("Invalid three-wire port");
    return false;
  }

  for (int i = 0; i < 4; ++i) {
    *LeftDrive[i] = CreateMotor(config::LEFT_DRIVE[i], config::BRAKE_MODE);
    *RightDrive[i] = CreateMotor(config::RIGHT_DRIVE[i], config::BRAKE_MODE);
  }
  ArmL = CreateMotor(config::ArmL, config::ARM_BRAKE_MODE);
  ArmR = CreateMotor(config::ArmR, config::ARM_BRAKE_MODE);
  // 软件归零不会寻找机械起点；启动前必须人工放到约定的起始位置。
  if (ArmL) {
    ArmL->setPosition(0, vex::deg);
  }
  if (ArmR) {
    ArmR->setPosition(0, vex::deg);
  }
  Claw = CreateMotor(config::Claw, config::CLAW_BRAKE_MODE);
  TogglePiston = new vex::digital_out(ThreeWirePort(config::TogglePiston));
  TogglePiston->set(config::PISTON_INITIAL_STATE);
  return true;
}

manual::Input ReadInput() {
  manual::Input input;
  input.y_axis = ReadAxis(config::YAxis);
  input.angular_axis = ReadAxis(config::AngularAxis);
  // 默认视为不可用；只有两侧电机都在线且位置差在允许范围内才启用角度控制。
  input.arm_ready = false;
  if (ArmL && ArmR && ArmL->installed() && ArmR->installed()) {
    const double left = ArmL->position(vex::deg);
    const double right = ArmR->position(vex::deg);
    input.arm_angle = (left + right) / 2.0 * config::ARM_UP_DIRECTION;
    const double diff = left - right;
    input.arm_ready = (diff < 0 ? -diff : diff) <= config::ARM_MAX_DIFF;
  }
  input.arm_up = Pressed(config::ArmUp);
  input.arm_down = Pressed(config::ArmDown);
  input.claw_close = Pressed(config::ClawClose);
  input.claw_open = Pressed(config::ClawOpen);
  input.toggle = Pressed(config::Toggle);
  return input;
}

void Apply(const manual::Output& output, double dt) {
  for (int i = 0; i < 4; ++i) {
    SetVoltage(*LeftDrive[i], output.L_voltage);
    SetVoltage(*RightDrive[i], output.R_voltage);
  }
  if (ArmL && ArmR) {
    const double left = ArmL->position(vex::deg) * config::ARM_UP_DIRECTION;
    const double right = ArmR->position(vex::deg) * config::ARM_UP_DIRECTION;
    if (!output.arm_enabled || !ArmL->installed() || !ArmR->installed()) {
      // 只在退出角度控制时停止一次，避免反复改写保持位置。
      if (arm_active) {
        ArmL->stop(config::ARM_BRAKE_MODE);
        ArmR->stop(config::ARM_BRAKE_MODE);
      }
      arm_active = false;
      ArmL_PID.Reset(left);
      ArmR_PID.Reset(right);
      arm_filtered_target = (left + right) / 2.0;
      ArmTarget.Reset(arm_filtered_target);
      arm_L_voltage = arm_R_voltage = 0.0;
    } else {
      if (!arm_active) {
        ArmL_PID.Reset(left);
        ArmR_PID.Reset(right);
        ArmTarget.Reset((left + right) / 2.0);
      }
      const double requested = manual::LimitAngle(output.arm_target_angle,
          config::ARM_MIN_ANGLE, config::ARM_MAX_ANGLE,
          config::ARM_UPPER_LIMIT_ENABLED);
      arm_filtered_target = ArmTarget.Update(requested, dt,
          config::ARM_MAX_VELOCITY * 6.0, config::ARM_TARGET_FILTER_TIME);
      arm_L_voltage = ArmL_PID.Update(arm_filtered_target, left, dt);
      arm_R_voltage = ArmR_PID.Update(arm_filtered_target, right, dt);
      // 使能期间始终使用电压模式，包括零电压，避免与内置 hold 反复切换。
      ArmL->spin(vex::fwd, arm_L_voltage * config::ARM_UP_DIRECTION, vex::voltageUnits::mV);
      ArmR->spin(vex::fwd, arm_R_voltage * config::ARM_UP_DIRECTION, vex::voltageUnits::mV);
      arm_active = true;
    }
  }
  SetVoltage(Claw, output.claw_voltage);
  if (TogglePiston) TogglePiston->set(output.toggle_piston);
}

void GetArmPIDInfo(double& target, double& L_voltage, double& R_voltage) {
  target = arm_filtered_target;
  L_voltage = arm_L_voltage;
  R_voltage = arm_R_voltage;
}

void GetDriveMotorCount(int& left_count, int& right_count) {
  left_count = 0;
  right_count = 0;
  for (int i = 0; i < 4; ++i) {
    if (*LeftDrive[i] && (*LeftDrive[i])->installed()) ++left_count;
    if (*RightDrive[i] && (*RightDrive[i])->installed()) ++right_count;
  }
}
}  // 硬件接口命名空间
