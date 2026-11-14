#include "control-config.h"
#include "robot-config.h"
#include "drive-sampling.h"
#include "autonomous.h"
#include "competition-control.h"
#include "runtime-info.h"
#if BARTMOSS_CALIBRATION
#include "calibration-config.h"
#elif !BARTMOSS_COMPETITION
#include "test-config.h"
#endif

static_assert(config::YAxis >= 1 && config::YAxis <= 4 && config::AngularAxis >= 1 && config::AngularAxis <= 4,
              "Joystick axes must be 1..4");
static_assert(config::JOYSTICK_DEADBAND >= 0 && config::JOYSTICK_DEADBAND < 100,
              "Joystick deadband must be 0..99");
static_assert(config::MAX_VOLTAGE > 0 && config::MAX_VOLTAGE <= 12000 && config::UPDATE_TIME > 0,
              "Invalid drive voltage or update period");
static_assert(config::TELEMETRY_INTERVAL_MS >= 20, "Telemetry interval must be at least 20 ms");
static_assert(config::INERTIAL_PORT >= 1 && config::INERTIAL_PORT <= 21 &&
              config::DRIVE_WHEEL_DIAMETER_MM > 0 && config::DRIVE_MOTOR_REVS_PER_WHEEL_REV > 0,
              "Invalid inertial port or drivetrain geometry");
static_assert(config::ARM_UP_VOLTAGE > 0 && config::ARM_UP_VOLTAGE <= 12000 &&
              config::ARM_DOWN_VOLTAGE > 0 && config::ARM_DOWN_VOLTAGE <= 12000 &&
              config::CLAW_VOLTAGE > 0 && config::CLAW_VOLTAGE <= 12000,
              "Arm and claw voltages must be 1..12000 mV");
static_assert((config::ARM_UP_DIRECTION == 1 || config::ARM_UP_DIRECTION == -1) &&
              (config::CLAW_CLOSE_DIRECTION == 1 || config::CLAW_CLOSE_DIRECTION == -1),
              "Mechanism direction must be 1 or -1");

int main() {
  Brain.Screen.print("Bartmoss v%s", config::PROGRAM_VERSION);
  const bool configured = hardware::Initialize();
  if (!configured) Controller.Screen.print("Configure ports");
  else {
    Brain.Screen.clearScreen(); Brain.Screen.print("Bartmoss v%s", config::PROGRAM_VERSION);
  }
  static manual::driver_control DriverControl(config::PISTON_INITIAL_STATE);
  static autonomous_program Autonomous;
  Autonomous.Configure(config::StraightControl(), config::TurnControl(), config::CornerControl());
  RegisterCompetitionCallbacks();
#if BARTMOSS_CALIBRATION
  static calibration_control Calibration(config::CALIBRATION_VOLTAGES, config::CALIBRATION_TIMEOUT_MS);
  Calibration.ConfigureFeedforward(config::CALIBRATION_TARGETS, config::FF_LEFT, config::FF_RIGHT,
                                   config::CALIBRATION_MODE_HOLD_MS);
  Calibration.ConfigureReverseFeedforward(config::FF_LEFT_REVERSE, config::FF_RIGHT_REVERSE);
  Calibration.ConfigureHeading(config::DRIVE_HEADING);
  Calibration.ConfigureVelocityPID(config::DRIVE_VELOCITY_PID);
#elif !BARTMOSS_COMPETITION
  static test_control Test;
  Test.Configure(config::TestSettings(), config::StraightControl(), config::TurnControl(), config::CornerControl());
#endif
  drive::telemetry_switch Telemetry;
  vex::thread sampling(drive::SamplingTask), telemetry_task(DriveTelemetry), display(ControllerInfoDisplay);
  vex::timer ControlTimer;
  double previous_time = ControlTimer.time(vex::msec);
  unsigned int tick = 0;
  while (true) {
    const double now = ControlTimer.time(vex::msec);
    const double dt = now - previous_time; previous_time = now;
    const bool connected = Competition.isFieldControl() || Competition.isCompetitionSwitch();
    const competition_request request = ReadCompetitionRequest();
    const auto access = runtime::Permissions(config::PROGRAM_MODE, configured, connected,
        Competition.isEnabled(), Competition.isDriverControl(), Competition.isAutonomous(), Controller.installed(), request.stage);
    const manual::Input input = hardware::ReadInput();
    const bool telemetry = Telemetry.Update(input.telemetry, access.telemetry);
    drive::SetTelemetryEnabled(telemetry);
    manual::Output output = DriverControl.Update(input, access.manual, config::JOYSTICK_DEADBAND, config::MAX_VOLTAGE,
        config::ARM_UP_VOLTAGE, config::ARM_DOWN_VOLTAGE, config::CLAW_VOLTAGE * config::CLAW_CLOSE_DIRECTION);
    drive::BeginCommandUpdate();
    drive::sample speed;
    const bool have_speed = drive::ReadLatestSample(speed);
    double heading = 0;
    const auto inertial = hardware::ReadInertial(heading);
    int left_count = 0, right_count = 0;
    hardware::GetDriveMotorCount(left_count, right_count);
    drive::straight_feedback frame;
    frame.motors_online = left_count == 4 && right_count == 4;
    frame.inertial_ready = inertial == hardware::InertialState::ready; frame.heading_deg = heading;
    frame.position.valid = hardware::ReadDrivePosition(frame.position.left_deg, frame.position.right_deg);
    frame.velocity.sequence = speed.sequence; frame.velocity.time_ms = speed.time_ms;
#if BARTMOSS_CALIBRATION
    // 标定保持原来的过滤测速口径，以便和已有前馈数据比较。
    frame.velocity.valid = have_speed && speed.left.velocity_valid && speed.right.velocity_valid;
    frame.velocity.left_rpm = speed.left.rpm; frame.velocity.right_rpm = speed.right.rpm;
    frame.stopped = frame.velocity.valid && std::fabs(speed.left.raw_average_rpm) < 5 &&
                    std::fabs(speed.right.raw_average_rpm) < 5;
#else
    // 日常动作和比赛动作统一使用编码器位移差分速度闭环。
    frame.velocity.valid = have_speed && speed.left.encoder_velocity_valid && speed.right.encoder_velocity_valid;
    frame.velocity.left_rpm = speed.left.encoder_rpm; frame.velocity.right_rpm = speed.right.encoder_rpm;
    frame.stopped = frame.velocity.valid && std::fabs(frame.velocity.left_rpm) < 5 && std::fabs(frame.velocity.right_rpm) < 5;
#endif
    bool test_owns_drive = false;
    int mode = 0;
    double target_rpm = 0;
    runtime_info info;
#if BARTMOSS_CALIBRATION
    test_owns_drive = Calibration.Update(input, output, access.calibration, frame.motors_online, now,
        frame.stopped, frame.inertial_ready, heading, frame.velocity);
    info.calibration = Calibration.Info();
    if (info.calibration.visible) {
      mode = info.calibration.speed_control ? 5 : info.calibration.feedforward ? 4 : 1;
      target_rpm = info.calibration.target_rpm;
    }
#elif !BARTMOSS_COMPETITION
    test_owns_drive = Test.Update(input, output, access.tests, now, frame);
    info.test = Test.Info();
    if (info.test.visible) {
      mode = info.test.selected == test_motion::straight ? 6 : info.test.selected == test_motion::turn ? 3 : 8;
      target_rpm = info.test.target_rpm;
    }
#endif
    Autonomous.Update(access.autonomous, frame, now);
    if (access.autonomous) {
      output.L_voltage = Autonomous.Info().left_mV; output.R_voltage = Autonomous.Info().right_mV;
      mode = 7;
      target_rpm = Autonomous.Info().motion == autonomous_motion::corner ? Autonomous.Info().corner.center_rpm :
          Autonomous.Info().motion == autonomous_motion::turn ? Autonomous.Info().turn.target_rpm : Autonomous.Info().drive.motion.target_rpm;
    }
    if (inertial == hardware::InertialState::calibrating) output.L_voltage = output.R_voltage = 0;
    // 唯一电机写入点；选择测试、完成、取消及场控禁用均保持明确的制动。
    hardware::Apply(output, true, (connected && !access.manual) || test_owns_drive ||
                    inertial == hardware::InertialState::calibrating);
    drive::PublishCommand(output.L_voltage, output.R_voltage, access.manual || access.autonomous, true, mode, target_rpm);
    info.input = input; info.output = output; info.configured = configured;
    info.enabled = access.manual || access.autonomous; info.telemetry = telemetry;
    info.tick = tick++; info.dt = dt; info.automatic = Autonomous.Info();
    info.autonomous_period = access.autonomous; info.phase = request;
    PublishRuntimeInfo(info);
    vex::wait(config::UPDATE_TIME, vex::msec);
  }
}
