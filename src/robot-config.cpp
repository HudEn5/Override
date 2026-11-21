#include "robot-config.h"
#include "control-config.h"
#include <cmath>

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
bool left_arm_moving = false, right_arm_moving = false;
vex::inertial* Inertial = nullptr;
vex::timer InertialTimer;
bool inertial_calibration_started = false;
vex::mutex lift_mutex;
mechanisms::lift_mailbox lift_mailbox;
mechanisms::lift_info lift_snapshot;

bool RegisterPort(int port, bool required, bool (&used)[22]) {
  if (port == 0 && !required)
    return true;
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
  if (setting.port == 0)
    return nullptr;
  // SDK 端口索引从 0 开始；配置文件使用主控上标注的端口编号。
  vex::motor* motor = new vex::motor(setting.port - 1, setting.gear, setting.reversed);
  motor->setStopping(brake);
  motor->stop();
  return motor;
}

vex::triport::port& ThreeWirePort(char port) {
  switch (port) {
    case 'A':
      return Brain.ThreeWirePort.A;
    case 'B':
      return Brain.ThreeWirePort.B;
    case 'C':
      return Brain.ThreeWirePort.C;
    case 'D':
      return Brain.ThreeWirePort.D;
    case 'E':
      return Brain.ThreeWirePort.E;
    case 'F':
      return Brain.ThreeWirePort.F;
    case 'G':
      return Brain.ThreeWirePort.G;
    default:
      return Brain.ThreeWirePort.H;  // 仅在端口校验通过后调用。
  }
}

int ReadAxis(int axis) {
  switch (axis) {
    case 1:
      return Controller.Axis1.position();
    case 2:
      return Controller.Axis2.position();
    case 3:
      return Controller.Axis3.position();
    case 4:
      return Controller.Axis4.position();
    default:
      return 0;
  }
}

bool Pressed(config::Button button) {
  switch (button) {
    case config::Button::R1:
      return Controller.ButtonR1.pressing();
    case config::Button::R2:
      return Controller.ButtonR2.pressing();
    case config::Button::L1:
      return Controller.ButtonL1.pressing();
    case config::Button::L2:
      return Controller.ButtonL2.pressing();
    case config::Button::A:
      return Controller.ButtonA.pressing();
    case config::Button::B:
      return Controller.ButtonB.pressing();
    case config::Button::X:
      return Controller.ButtonX.pressing();
    case config::Button::Y:
      return Controller.ButtonY.pressing();
    case config::Button::Up:
      return Controller.ButtonUp.pressing();
    case config::Button::Down:
      return Controller.ButtonDown.pressing();
    case config::Button::Left:
      return Controller.ButtonLeft.pressing();
    case config::Button::Right:
      return Controller.ButtonRight.pressing();
  }
  return false;
}

void SetVoltage(vex::motor* motor, int millivolts) {
  if (!motor)
    return;
  if (millivolts == 0)
    motor->stop();
  else
    motor->spin(vex::fwd, millivolts, vex::voltageUnits::mV);
}
void ApplyArmVoltage(vex::motor* motor, int millivolts, bool& moving) {
  if (millivolts != 0 && motor) {
    motor->spin(vex::fwd, millivolts * config::ARM_UP_DIRECTION, vex::voltageUnits::mV);
    moving = true;
  } else if (moving) {
    if (motor) motor->stop(config::ARM_BRAKE_MODE);
    moving = false;
  }
}
bool LiftManualAllowed(const mechanisms::lift_authority& authority, double now) {
  const double age = now - authority.time_ms;
  if (!std::isfinite(age) || age < 0 || age > mechanisms::LIFT_COMMAND_LEASE_MS ||
      !authority.manual_allowed || !Controller.installed()) return false;
  const bool field = Competition.isFieldControl() || Competition.isCompetitionSwitch();
  if (field != authority.field_connected) return false;
#if !BARTMOSS_CALIBRATION
  if (field && (!Competition.isEnabled() || !Competition.isDriverControl() || Competition.isAutonomous())) return false;
#endif
  return true;
}
bool LiftClosedLoopAllowed(const mechanisms::lift_authority& authority, double now) {
  const double age = now - authority.time_ms;
  if (!authority.closed_loop_allowed || !std::isfinite(age) || age < 0 || age > mechanisms::LIFT_COMMAND_LEASE_MS) return false;
  const bool field = Competition.isFieldControl() || Competition.isCompetitionSwitch();
  if (field != authority.field_connected) return false;
  if (!authority.autonomous) return !field && LiftManualAllowed(authority, now);
  return field ? Competition.isEnabled() && Competition.isAutonomous() && !Competition.isDriverControl() : Controller.installed();
}
}  // 匿名命名空间

namespace hardware {
bool Initialize() {
  // 先检查全部配置，再创建设备或写入输出。
  bool used[22] = {};
  Brain.Screen.setCursor(2, 1);
  for (int i = 0; i < 4; ++i) {
    if (!RegisterPort(config::LEFT_DRIVE[i].port, true, used) ||
        !RegisterPort(config::RIGHT_DRIVE[i].port, true, used))
      return false;
  }
  for (int i = 0; i < 3; ++i) {
    if (!RegisterPort(config::STRUCTURE_MOTORS[i].port, true, used))
      return false;
  }
  if (!RegisterPort(config::INERTIAL_PORT, false, used))
    return false;

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
  // 不修改 SDK 编码器计数；抬升线程用上电第一组有效读数建立一次相对零点。
  Claw = CreateMotor(config::Claw, config::CLAW_BRAKE_MODE);
  TogglePiston = new vex::digital_out(ThreeWirePort(config::TogglePiston));
  TogglePiston->set(config::PISTON_INITIAL_STATE);
  if (config::INERTIAL_PORT != 0) {
    Inertial = new vex::inertial(config::INERTIAL_PORT - 1, vex::turnType::left);
    if (Inertial->installed()) {
      Inertial->calibrate();
      InertialTimer.reset();
      inertial_calibration_started = true;
    }
  }
  return true;
}

manual::input ReadInput() {
  manual::input input;
  input.y_axis = ReadAxis(config::YAxis);
  input.angular_axis = ReadAxis(config::AngularAxis);
  input.arm_ready = ArmL && ArmR && ArmL->installed() && ArmR->installed();
  input.arm_up = Pressed(config::ArmUp);
  input.arm_down = Pressed(config::ArmDown);
  input.claw_close = Pressed(config::ClawClose);
  input.claw_open = Pressed(config::ClawOpen);
  input.toggle = Pressed(config::Toggle);
  input.test_start = Pressed(config::TestStart);
  input.test_cancel = Pressed(config::TestCancel);
  input.telemetry = Pressed(config::Telemetry);
  input.test_next = Pressed(config::TestNext);
  input.test_previous = Pressed(config::TestPrevious);
  input.test_positive = Pressed(config::TestPositive);
  input.test_negative = Pressed(config::TestNegative);
  return input;
}

void Apply(const manual::output& output, bool write_drive, bool brake_drive,
           const mechanisms::claw_info* claw_override) {
  if (write_drive) {
    for (int i = 0; i < 4; ++i) {
      // 测试停车直接刹车，避免先按普通手控停止方式再发一次刹车。
      if (brake_drive && output.left_mV == 0) {
        if (*LeftDrive[i])
          (*LeftDrive[i])->stop(vex::brake);
      } else
        SetVoltage(*LeftDrive[i], output.left_mV);
      if (brake_drive && output.right_mV == 0) {
        if (*RightDrive[i])
          (*RightDrive[i])->stop(vex::brake);
      } else
        SetVoltage(*RightDrive[i], output.right_mV);
    }
  }
  // ArmL/ArmR 由 LiftControlTask 独占，包括原 R1/R2 手动电压。
  const bool claw_running = claw_override && claw_override->result == mechanisms::state::running;
  const int claw_mV = claw_running ?
      (ClawOnline() ? claw_override->millivolts * config::CLAW_CLOSE_DIRECTION : 0) : output.claw_mV;
  SetVoltage(Claw, claw_mV);
  if (TogglePiston)
    TogglePiston->set(output.toggle_piston);
}
mechanisms::feedback ReadLiftFeedback() {
  mechanisms::feedback result;
  result.arm_online = ArmL && ArmR && ArmL->installed() && ArmR->installed();
  if (result.arm_online) {
    result.left_deg = ArmL->position(vex::deg) * config::ARM_UP_DIRECTION;
    result.right_deg = ArmR->position(vex::deg) * config::ARM_UP_DIRECTION;
    result.left_deg_s = ArmL->velocity(vex::rpm) * 6 * config::ARM_UP_DIRECTION;
    result.right_deg_s = ArmR->velocity(vex::rpm) * 6 * config::ARM_UP_DIRECTION;
    result.arm_valid = std::isfinite(result.left_deg) && std::isfinite(result.right_deg) &&
        std::isfinite(result.left_deg_s) && std::isfinite(result.right_deg_s);
  }
  return result;
}
bool ClawOnline() { return Claw && Claw->installed(); }

void PublishLiftRequest(const mechanisms::lift_authority& authority, const mechanisms::lift_request& request) {
  lift_mutex.lock(); lift_mailbox.publish(authority, request); lift_mutex.unlock();
}
mechanisms::lift_info ReadLiftInfo() {
  lift_mutex.lock();
  const mechanisms::lift_info result = lift_snapshot;
  lift_mutex.unlock();
  return result;
}
int LiftControlTask() {
  // 只构造一次控制器，不拷贝大的 runtime_info；mutex 内无 SDK 电机操作或等待。
  mechanisms::lift_controller lift;
  mechanisms::lift_packet packet;
  bool power_zero_pending = true;
  bool have_heartbeat = false;
  while (true) {
    const double cycle_start = vex::timer::systemHighResolution() / 1000.0;
    if (lift_mutex.try_lock()) {
      packet = lift_mailbox.take();
      lift_mutex.unlock();
    }
    // 必须在取包之后读时钟，避免新命令时间比旧 now 更新而被误判过期。
    const double now = vex::timer::systemHighResolution() / 1000.0;
    if (packet.reset) lift.cancel();
    packet.reset = false;
    const auto command = packet.command;
    packet.command = mechanisms::lift_request();  // 拿不到锁也不能重放上周期命令。
    const bool overflow = packet.overflow; packet.overflow = false;
    if (std::isfinite(packet.authority.time_ms) && packet.authority.time_ms >= 0) have_heartbeat = true;
    const double age = now - packet.authority.time_ms;
    const bool fresh = have_heartbeat && std::isfinite(age) && age >= 0 && age <= mechanisms::LIFT_COMMAND_LEASE_MS;
    const bool manual_allowed = LiftManualAllowed(packet.authority, now);
    const bool closed_loop_allowed = LiftClosedLoopAllowed(packet.authority, now);
    const bool up = Pressed(config::ArmUp), down = Pressed(config::ArmDown);
    const bool manual_held = manual_allowed && (up || down);
    const auto frame = ReadLiftFeedback();
    lift.observe(frame, closed_loop_allowed, now);
    if (power_zero_pending && lift.zero_here(true)) {
      power_zero_pending = false;
      lift.notice("POWER ZERO");
    }
    if (have_heartbeat && !fresh) lift.fault("MAIN STALE");
    else if (overflow) lift.fault("COMMAND OVERFLOW");
    else if (manual_held || command.kind == mechanisms::lift_command_kind::cancel) {
      lift.cancel(); if (command.kind != mechanisms::lift_command_kind::none) lift.notice("MANUAL/CANCEL");
    } else if (command.kind != mechanisms::lift_command_kind::none) {
      const double event_age = now - packet.command_time_ms;
      const bool sticks = !packet.authority.autonomous &&
          (std::abs(ReadAxis(config::YAxis)) > config::JOYSTICK_DEADBAND ||
           std::abs(ReadAxis(config::AngularAxis)) > config::JOYSTICK_DEADBAND);
      if (!std::isfinite(event_age) || event_age < 0 || event_age > mechanisms::LIFT_COMMAND_LEASE_MS) lift.notice("COMMAND STALE");
      else if (!closed_loop_allowed) lift.notice("MODE DISABLED");
      else if (!packet.authority.start_ready) lift.notice(packet.authority.start_blocked);
      else if (sticks) lift.notice("CENTER STICKS");
      else if (lift.busy()) lift.notice("BUSY");
      else {
        if (command.kind == mechanisms::lift_command_kind::zero) {
          lift.notice(lift.zero_here() ? "ZEROED" : "WAIT ARM STOP");
        } else {
          const bool accepted = command.kind == mechanisms::lift_command_kind::angle ? lift.LiftTo(command.target, command.timeout_ms) :
              lift.LiftHeightTo(command.target, command.timeout_ms);
          lift.notice(accepted ? "STARTED" : lift.info().status);
        }
      }
    }
    // SDK 读取或调度可能耗时：真正写入前重新检查时钟、许可和实时手动键。
    const double check_time = vex::timer::systemHighResolution() / 1000.0;
    const bool live_manual = LiftManualAllowed(packet.authority, check_time);
    const bool live_closed = LiftClosedLoopAllowed(packet.authority, check_time);
    const bool write_up = Pressed(config::ArmUp), write_down = Pressed(config::ArmDown);
    const double write_now = vex::timer::systemHighResolution() / 1000.0;
    const double write_age = write_now - packet.authority.time_ms;
    const bool work_fresh = std::isfinite(write_now) && write_now >= cycle_start &&
        write_now - cycle_start <= mechanisms::LIFT_COMMAND_LEASE_MS;
    const bool write_fresh = have_heartbeat && work_fresh && std::isfinite(write_age) &&
        write_age >= 0 && write_age <= mechanisms::LIFT_COMMAND_LEASE_MS;
    if (!work_fresh) lift.fault("CLOCK/STALE");
    else if (have_heartbeat && (!std::isfinite(write_age) || write_age < 0 || write_age > mechanisms::LIFT_COMMAND_LEASE_MS))
      lift.fault("MAIN STALE");
    else if (!live_closed || (live_manual && (write_up || write_down))) lift.cancel();
    int voltage = 0;
    if (write_fresh && frame.arm_online) {
      if (live_manual && (write_up || write_down)) voltage = write_up != write_down ? (write_up ? config::ARM_UP_VOLTAGE : -config::ARM_DOWN_VOLTAGE) : 0;
      else if (live_closed && lift.busy()) voltage = lift.info().left_mV;
    }
    ApplyArmVoltage(ArmL, voltage, left_arm_moving);
    ApplyArmVoltage(ArmR, voltage, right_arm_moving);
    if (lift_mutex.try_lock()) { lift_snapshot = lift.info(); lift_mutex.unlock(); }
    const double work_ms = vex::timer::systemHighResolution() / 1000.0 - cycle_start;
    // 10ms 为目标周期；实际 dt 显示在主控，不能把目标频率当实测频率。
    vex::wait(std::max(1.0, mechanisms::LIFT_CONTROL_PERIOD_MS - work_ms), vex::msec);
  }
  return 0;
}
void GetDriveMotorCount(int& left_count, int& right_count) {
  left_count = 0;
  right_count = 0;
  for (int i = 0; i < 4; ++i) {
    if (*LeftDrive[i] && (*LeftDrive[i])->installed())
      ++left_count;
    if (*RightDrive[i] && (*RightDrive[i])->installed())
      ++right_count;
  }
}

drive::feedback ReadDriveFeedback() {
  drive::feedback result;
  for (int i = 0; i < 4; ++i) {
    vex::motor* motors[2] = {*LeftDrive[i], *RightDrive[i]};
    drive::motor_feedback* values[2] = {&result.left[i], &result.right[i]};
    for (int side = 0; side < 2; ++side) {
      if (motors[side] && motors[side]->installed()) {
        values[side]->online = true;
        values[side]->rpm = motors[side]->velocity(vex::rpm);
        values[side]->millivolts = motors[side]->voltage(vex::voltageUnits::mV);
        // 八台均读取位置，供 TP 速度差分和 SDK 异常转速替代使用。
        values[side]->position_deg = motors[side]->position(vex::deg);
        values[side]->position_valid = std::isfinite(values[side]->position_deg);
        if (side == 0 && i < 2) {
          // 保留 LM 与同侧 LF 的独立位置、电流读数，便于和 SDK 转速交叉比较。
          values[side]->current_A = motors[side]->current(vex::amp);
          values[side]->diagnostic_valid =
              values[side]->position_valid && std::isfinite(values[side]->current_A);
        }
      }
    }
  }
  return result;
}

bool ReadDrivePosition(double& left_deg, double& right_deg) {
  double left_sum = 0.0;
  double right_sum = 0.0;
  for (int i = 0; i < 4; ++i) {
    vex::motor* left = *LeftDrive[i];
    vex::motor* right = *RightDrive[i];
    if (!left || !right || !left->installed() || !right->installed())
      return false;
    left_sum += left->position(vex::deg);
    right_sum += right->position(vex::deg);
  }
  if (!std::isfinite(left_sum) || !std::isfinite(right_sum))
    return false;
  left_deg = left_sum / 4.0;
  right_deg = right_sum / 4.0;
  return true;
}

InertialState ReadInertial(double& rotation) {
  if (!Inertial || !Inertial->installed()) {
    inertial_calibration_started = false;
    return InertialState::offline;
  }
  // 掉线或启动时未识别到传感器，需要重启程序完成静止校准。
  if (!inertial_calibration_started)
    return InertialState::fault;
  const double elapsed = InertialTimer.time(vex::msec);
  if (Inertial->isCalibrating())
    return elapsed < 8000 ? InertialState::calibrating : InertialState::fault;
  if (elapsed < 500)
    return InertialState::calibrating;
  rotation = Inertial->rotation(vex::deg);
  return std::isfinite(rotation) ? InertialState::ready : InertialState::fault;
}
}  // 硬件接口命名空间
