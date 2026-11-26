#include "control-config.h"
#include "robot-config.h"
#include "drive-sampling.h"
#include "autonomous.h"
#include "competition-control.h"
#include "runtime-info.h"
#include "performance-config.h"
#include "startup-mode.h"
#include "test-config.h"

static_assert(config::YAxis >= 1 && config::YAxis <= 4 && config::AngularAxis >= 1 &&
                  config::AngularAxis <= 4,
              "Joystick axes must be 1..4");
static_assert(config::JOYSTICK_DEADBAND >= 0 && config::JOYSTICK_DEADBAND < 100,
              "Joystick deadband must be 0..99");
static_assert(config::MAX_VOLTAGE > 0 && config::MAX_VOLTAGE <= 12000 && config::UPDATE_TIME > 0,
              "Invalid drive voltage or update period");
static_assert(config::TELEMETRY_INTERVAL_MS >= 20, "Telemetry interval must be at least 20 ms");
static_assert(config::INERTIAL_PORT >= 1 && config::INERTIAL_PORT <= 21 &&
                  config::DRIVE_WHEEL_DIAMETER_MM > 0 && config::DRIVE_MOTOR_REVS_PER_WHEEL_REV > 0,
              "Invalid inertial port or drivetrain geometry");
static_assert(config::ARM_UP_VOLTAGE > 0 && config::ARM_UP_VOLTAGE <= 12000 && config::ARM_DOWN_VOLTAGE > 0 &&
                  config::ARM_DOWN_VOLTAGE <= 12000 && config::CLAW_VOLTAGE > 0 &&
                  config::CLAW_VOLTAGE <= 12000,
              "Arm and claw voltages must be 1..12000 mV");
static_assert((config::ARM_UP_DIRECTION == 1 || config::ARM_UP_DIRECTION == -1) &&
                  (config::CLAW_CLOSE_DIRECTION == 1 || config::CLAW_CLOSE_DIRECTION == -1),
              "Mechanism direction must be 1 or -1");

int main() {
  Brain.Screen.print("Bartmoss v%s", config::PROGRAM_VERSION);
  const bool configured = hardware::Initialize();
  if (!configured)
    Controller.Screen.print("Configure ports");
  else {
    Brain.Screen.clearScreen();
    Brain.Screen.print("Bartmoss v%s", config::PROGRAM_VERSION);
  }
  static manual::driver_control DriverControl(config::PISTON_INITIAL_STATE);
  static autonomous_program Autonomous;
  static drive::odometry Odometry;
  Odometry.Configure(config::DRIVE_WHEEL_DIAMETER_MM, config::DRIVE_MOTOR_REVS_PER_WHEEL_REV,
                     config::DRIVE_TRACK_WIDTH_MM);
  Odometry.SetPose(0, 0, 0);  // 已知起点；不清零电机/IMU。首次完整反馈建立传感器基准。
#if !BARTMOSS_CALIBRATION
  static runtime::startup_mode Selection;
  static runtime::teach_recorder Teach;
#endif
  Autonomous.Configure(config::AutoStraightControl(config::StraightControl()),
                       config::AutoTurnControl(config::TurnControl()),
                       config::RouteCornerControl(config::CornerControl()));
  Autonomous.ConfigurePoint(config::PointControl());
  static mechanisms::lift_adapter ArmController;
  static mechanisms::arm_tuning_test ArmTuning(config::ARM_TUNING_MIN_UP_MV, config::ARM_TUNING_MAX_UP_MV,
                                              config::ARM_TUNING_DOWN_MV, config::ARM_TUNING_MAX_DEG,
                                              config::ARM_TUNING_PULSE_MS);
  static mechanisms::claw_adapter ClawController(config::CLAW_CLOSE_DIRECTION);
  Autonomous.BindMechanism(mechanism::channel::arm, ArmController);
  Autonomous.BindMechanism(mechanism::channel::claw, ClawController);
  RegisterCompetitionCallbacks();
#if BARTMOSS_CALIBRATION
  static calibration_control Calibration(config::CALIBRATION_VOLTAGES, config::CALIBRATION_TIMEOUT_MS);
  Calibration.ConfigureFeedforward(config::CALIBRATION_TARGETS, config::FF_LEFT, config::FF_RIGHT,
                                   config::CALIBRATION_MODE_HOLD_MS);
  Calibration.ConfigureReverseFeedforward(config::FF_LEFT_REVERSE, config::FF_RIGHT_REVERSE);
  Calibration.ConfigureHeading(config::DRIVE_HEADING);
  Calibration.ConfigureVelocityPID(config::DRIVE_VELOCITY_PID);
#elif BARTMOSS_HIGH_SPEED
  static high_speed_control Test;
  Test.Configure(config::HIGH_SPEED_STAGES, config::StraightControl());
  static test_control RouteTest;
  RouteTest.Configure(
      config::HighSpeedCornerSettings(), config::RouteStraightControl(config::StraightControl()),
      config::RouteTurnControl(config::TurnControl()), config::RouteCornerControl(config::CornerControl()));
#else
  static test_control Test;
  Test.Configure(config::TestSettings(), config::StraightControl(), config::TurnControl(),
                 config::CornerControl());
#endif
  drive::telemetry_switch Telemetry;
  vex::thread sampling(drive::SamplingTask), telemetry_task(DriveTelemetry), display(ControllerInfoDisplay);
  vex::timer ControlTimer;
  double previous_time = ControlTimer.time(vex::msec);
  unsigned int tick = 0;
  bool route_field_owner = false;
  bool route_had_control = false;
  while (true) {
    const double now = ControlTimer.time(vex::msec);
    const double dt = now - previous_time;
    previous_time = now;
    const bool connected = Competition.isFieldControl() || Competition.isCompetitionSwitch();
    const competition_request request = ReadCompetitionRequest();
    const manual::input input = hardware::ReadInput();
    drive::sample speed;
    const bool have_speed = drive::ReadLatestSample(speed);
    double heading = 0;
    const auto inertial = hardware::ReadInertial(heading);
    int left_count = 0, right_count = 0;
    hardware::GetDriveMotorCount(left_count, right_count);
    drive::straight_feedback frame;
    frame.motors_online = left_count == 4 && right_count == 4;
    frame.inertial_ready = inertial == hardware::InertialState::ready;
    frame.heading_deg = heading;
    frame.position.valid = hardware::ReadDrivePosition(frame.position.left_deg, frame.position.right_deg);
    frame.velocity.sequence = speed.sequence;
    frame.velocity.time_ms = speed.time_ms;
#if BARTMOSS_CALIBRATION
    // 标定保持原来的过滤测速口径，以便和已有前馈数据比较。
    frame.velocity.valid = have_speed && speed.left.velocity_valid && speed.right.velocity_valid;
    frame.velocity.left_rpm = speed.left.rpm;
    frame.velocity.right_rpm = speed.right.rpm;
    frame.stopped = frame.velocity.valid && std::fabs(speed.left.raw_average_rpm) < 5 &&
                    std::fabs(speed.right.raw_average_rpm) < 5;
#else
    // 日常动作和比赛动作统一使用编码器位移差分速度闭环。
    frame.velocity.valid =
        have_speed && speed.left.encoder_velocity_valid && speed.right.encoder_velocity_valid;
    frame.velocity.left_rpm = speed.left.encoder_rpm;
    frame.velocity.right_rpm = speed.right.encoder_rpm;
    frame.stopped = frame.velocity.valid && std::fabs(frame.velocity.left_rpm) < 5 &&
                    std::fabs(frame.velocity.right_rpm) < 5;
#endif
    // 所有模式持续推算；MoveToPoint与本地自动就绪读取此位姿，不积分已滤波转速。
    Odometry.Update(frame.position.left_deg, frame.position.right_deg, heading, now,
                    configured && frame.motors_online && frame.position.valid && frame.inertial_ready);
#if !BARTMOSS_CALIBRATION
    const bool initialization_ready = inertial != hardware::InertialState::calibrating;
    const bool sticks = std::abs(input.y_axis) > config::JOYSTICK_DEADBAND ||
                        std::abs(input.angular_axis) > config::JOYSTICK_DEADBAND;
    const char* auto_status = runtime::AutomaticStartStatus(frame, Odometry.Info(), now);
    const bool auto_ready = auto_status == nullptr;
#endif
#if BARTMOSS_CALIBRATION
    const auto access = runtime::Permissions(
        config::PROGRAM_MODE, configured, connected, Competition.isEnabled(), Competition.isDriverControl(),
        Competition.isAutonomous(), Controller.installed(), request.stage);
#else
    Selection.Update(input, configured && Controller.installed() && initialization_ready, connected, sticks,
                     frame.stopped, auto_ready, Autonomous.Info().finished, now);
    const auto access = runtime::SessionPermissions(
        Selection, configured && initialization_ready, connected, Competition.isEnabled(),
        Competition.isDriverControl(), Competition.isAutonomous(), Controller.installed(), request.stage);
#endif
    const bool telemetry = Telemetry.Update(input.telemetry, access.telemetry);
    const bool arm_tuning = config::ARM_TUNING_TEST && config::PROGRAM_MODE == runtime::program_mode::test &&
                            access.tests;
    drive::SetTelemetryEnabled(telemetry);
    manual::output output =
        DriverControl.Update(input, access.manual && !arm_tuning, config::JOYSTICK_DEADBAND,
                             config::MANUAL_DRIVE_MAX_VOLTAGE, config::ARM_UP_VOLTAGE,
                             config::ARM_DOWN_VOLTAGE, config::CLAW_VOLTAGE * config::CLAW_CLOSE_DIRECTION);
    drive::BeginCommandUpdate();
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
#else
#if BARTMOSS_HIGH_SPEED
    if (config::HIGH_SPEED_CORNER_TEST) {
      test_owns_drive = RouteTest.Update(input, output, access.tests && !arm_tuning, now, frame);
      info.test = RouteTest.Info();
    } else {
      test_owns_drive = Test.Update(input, output, access.tests && !arm_tuning, now, frame);
      info.test = Test.Info();
      info.tuning = Test.Metrics();
      info.high_speed_stages = true;
    }
#else
    test_owns_drive = Test.Update(input, output, access.tests && !arm_tuning, now, frame);
    info.test = Test.Info();
#endif
    if (info.test.visible) {
      mode = info.test.selected == test_motion::straight ? 6
             : info.test.selected == test_motion::turn   ? 3
                                                         : 8;
      target_rpm = info.test.target_rpm;
    }
#endif
#if !BARTMOSS_CALIBRATION
    info.local_selection = !connected;
    info.local_mode = Selection.Mode();
    info.controls_armed = Selection.Armed();
    info.teaching = Teach.Update(!connected && access.manual && Selection.Mode() == runtime::local_mode::teach,
                                 input, Odometry.Info(), now);
    info.auto_ready = auto_ready;
    info.auto_status = auto_status;
    info.initializing = !initialization_ready;
    info.returning_to_manual = Selection.ReturningToManual();
    info.return_remaining_ms = Selection.ReturnRemainingMs();
    info.route_debug = !connected && Selection.Mode() == runtime::local_mode::automatic;
    if (!connected && !access.autonomous && Autonomous.Info().active)
      Autonomous.CancelRoutine();
#endif
    const bool execute_route = access.autonomous;
    // 本地自动和场控自动切换时建立新运行周期，不能沿用另一来源的半条路线。
    if (connected != route_field_owner) {
      Autonomous.Update(false, frame, now);
      route_field_owner = connected;
    }
    const auto mechanism_frame = hardware::ReadMechanismFeedback();
    ArmController.Observe(mechanism_frame, execute_route, now);
    ClawController.Observe(mechanism_frame.claw_online, execute_route, now);
    Autonomous.ObservePose(Odometry.Info());
    Autonomous.Update(execute_route, frame, now);
    // 后台机构与底盘共用主循环写入点；取消时仅移交停止方式，不覆盖新的手控指令。
    if (execute_route || route_had_control)
      Autonomous.ApplyMechanisms(output, !execute_route);
    info.arm_tuning = ArmTuning.Update(arm_tuning, input, mechanism_frame, ArmController.LiftInfo(), now, output);
    if (execute_route)
      DriverControl.SetPistonState(output.toggle_piston);
    route_had_control = execute_route;
    if (execute_route) {
      output.left_mV = Autonomous.Info().left_mV;
      output.right_mV = Autonomous.Info().right_mV;
      mode = 7;
      target_rpm =
          Autonomous.Info().motion == autonomous_motion::rolling  ? Autonomous.Info().rolling.center_rpm
          : Autonomous.Info().motion == autonomous_motion::corner ? Autonomous.Info().corner.center_rpm
          : Autonomous.Info().motion == autonomous_motion::turn   ? Autonomous.Info().turn.target_rpm
                                                                  : Autonomous.Info().drive.motion.target_rpm;
    }
    if (inertial == hardware::InertialState::calibrating)
      output.left_mV = output.right_mV = 0;
    // 唯一电机写入点；选择测试、完成、取消及场控禁用均保持明确的制动。
    hardware::Apply(output, true,
                    (connected && !access.manual) || test_owns_drive || arm_tuning || execute_route ||
                        (!access.manual && !access.autonomous) ||
                        inertial == hardware::InertialState::calibrating,
                    info.local_selection && info.controls_armed && info.local_mode == runtime::local_mode::teach);
    drive::PublishCommand(output.left_mV, output.right_mV, access.manual || execute_route, true, mode,
                          target_rpm);
    info.pose = Odometry.Info();
    info.input = input;
    info.output = output;
    info.configured = configured;
    info.enabled = access.manual || execute_route;
    info.telemetry = telemetry;
    info.tick = tick++;
    info.dt = dt;
    info.automatic = Autonomous.Info();
    info.lift = ArmController.LiftInfo();
    info.claw = ClawController.ClawInfo();
    info.autonomous_period = execute_route;
    info.phase = request;
    PublishRuntimeInfo(info);
    vex::wait(config::UPDATE_TIME, vex::msec);
  }
}
