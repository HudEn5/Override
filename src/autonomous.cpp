#include "autonomous.h"

void autonomous_program::Update(bool permitted, const drive::straight_feedback& feedback, double now) {
  if (permitted && (!std::isfinite(now) || now < 0 || (previously_permitted && now < route_time))) {
    previously_permitted = true;
    FinishRoutine("CLOCK ERR");
    Publish();
    return;
  }
  frame = feedback;
  route_time = now;
  info.time_ms = std::isfinite(now) ? now : info.time_ms;
  Chassis.Observe(feedback, permitted, now);
  Turning.Observe(feedback, permitted, now);
  Cornering.Observe(feedback, permitted, now);
  Rolling.Observe(feedback, permitted, now);
  Pushing.Observe(feedback, permitted, now);
  if (!permitted) {
    if (info.active)
      FinishRoutine("DISABLED", route_result::cancelled);
    info.active = false;
    previously_permitted = false;
    Publish();
    return;
  }
  if (!previously_permitted) {
    Chassis.Reset();
    Turning.Reset();
    Cornering.Reset();
    Rolling.Reset();
    info.motion = autonomous_motion::none;
    Pushing.Reset();
    info.active = true;
    info.finished = false;
    info.step = 0;
    ++info.run;
    point_stage = point_phase::idle;
    route_step_started = false;
    rolling_span = 1;
    info.waiting = false;
    info.result = route_result::running;
    waiting_ready = true;
    info.status = "RUN";
  }
  previously_permitted = true;
  if (info.active && waiting_ready) {
    if (!feedback.motors_online || !feedback.inertial_ready || !feedback.velocity.valid ||
        !feedback.position.valid || !feedback.stopped) {
      info.status = "WAIT READY";
      Publish();
      return;
    }
    waiting_ready = false;
  }
  if (point_stage != point_phase::idle && !PointPoseReady()) {
    FinishRoutine("POSE ERROR");
    Publish();
    return;
  }
  if (!OwnershipValid()) {
    FinishRoutine("CONTROL OWNER");
    return;
  }
  switch (info.motion) {
    case autonomous_motion::straight:
      Chassis.UpdateDriveStraight();
      break;
    case autonomous_motion::turn:
      Turning.UpdateTurn();
      break;
    case autonomous_motion::corner:
      Cornering.UpdateCorner();
      break;
    case autonomous_motion::rolling:
      Rolling.Update();
      break;
    case autonomous_motion::push:
      Pushing.Update();
      break;
    case autonomous_motion::none:
      break;
  }
  if (ActiveState() == drive::motion_state::error || ActiveState() == drive::motion_state::cancelled)
    FinishRoutine(ActiveStatus(), ActiveState() == drive::motion_state::cancelled ? route_result::cancelled
                                                                                  : route_result::error);
  if (info.active)
    RunRoutine();
  if (!OwnershipValid())
    FinishRoutine("CONTROL OWNER");
  Publish();
}

// 每周期只推进当前动作；Wait 使用主循环时钟，不调用阻塞等待或创建额外任务。
void autonomous_program::RunRoute(std::initializer_list<route::command> commands) {
  if (commands.size() == 0) {
    FinishRoutine("EMPTY", route_result::done);
    return;
  }
  while (info.active && info.step < commands.size()) {
    const auto& command = *(commands.begin() + info.step);
    if (!route_step_started) {
      if (!std::isfinite(command.timeout_ms) || command.timeout_ms <= 0) {
        FinishRoutine("TIMEOUT CONFIG");
        return;
      }
      action_started_at = route_time;
      bool started = false;
      switch (command.kind) {
        case route::action::drive_turn_drive:
          if (!std::isfinite(command.before_mm) || command.before_mm < 0 ||
              !std::isfinite(command.after_mm) || command.after_mm < 0 || !std::isfinite(command.angle_deg) ||
              std::fabs(command.angle_deg) > 180) {
            FinishRoutine("ROUTE CONFIG");
            return;
          }
          transfer_phase = 0;
          if (command.transition == route::turn_mode::stop) {
            started = StartDriveStraight(command.before_mm, command.maximum_rpm);
          } else if (command.transition == route::turn_mode::rolling) {
            drive::rolling_leg legs[drive::MAX_ROLLING_TURNS];
            rolling_span = 0;
            for (unsigned index = info.step; index < commands.size(); ++index) {
              const auto& next = *(commands.begin() + index);
              if (next.kind != route::action::drive_turn_drive ||
                  next.transition != route::turn_mode::rolling)
                break;
              if (rolling_span == drive::MAX_ROLLING_TURNS) {
                FinishRoutine("ROLL TOO LONG");
                return;
              }
              auto& leg = legs[rolling_span++];
              leg.before_mm = next.before_mm;
              leg.angle_deg = next.angle_deg;
              leg.after_mm = next.after_mm;
              leg.maximum_rpm = next.maximum_rpm;
              leg.entry_mm_s = next.entry_speed_mm_s;
              leg.timeout_ms = next.timeout_ms;
            }
            started = StartRollingPath(legs, rolling_span);
          }
          break;
        case route::action::straight:
          started = StartDriveStraight(command.distance_mm, command.maximum_rpm);
          break;
        case route::action::turn:
          started = StartTurnRelative(command.angle_deg, command.maximum_rpm);
          break;
        case route::action::heading:
          started = StartTurnToHeading(command.angle_deg, command.maximum_rpm);
          break;
        case route::action::corner:
          started =
              StartDriveCorner(command.before_mm, command.angle_deg, command.after_mm, command.maximum_rpm);
          break;
        case route::action::point:
          point_realignments = 0;
          started = StartPoint(command);
          break;
        case route::action::swing:
          info.motion = autonomous_motion::turn;
          Turning.Configure(config::SwingControl(turn_settings));
          Turning.Observe(frame, true, route_time);
          started = Turning.StartSwingRelative(command.angle_deg, command.maximum_rpm, command.fixed_side);
          break;
        case route::action::push:
          started = StartPushFor(command.duration_ms, command.volts);
          break;
        case route::action::wait:
          if (!std::isfinite(command.duration_ms) || command.duration_ms < 0) {
            FinishRoutine("WAIT CONFIG");
            return;
          }
          info.motion = autonomous_motion::none;
          info.waiting = true;
          wait_started = route_time;
          started = true;
          break;
        default:
          FinishRoutine("ROUTE CONFIG");
          return;
      }
      if (!started) {
        if (!info.active)
          return;
        FinishRoutine(ActiveStatus());
        return;
      }
      route_step_started = true;
    }
    bool done = false;
    if (command.kind == route::action::wait) {
      info.status = "WAIT";
      done = route_time - wait_started >= command.duration_ms;
    } else {
      info.status = ActiveStatus();
      done = ActiveState() == drive::motion_state::done;
    }
    // 组合阶段不重置总计时；rolling块内由执行器逐命令计时。
    const bool rolling =
        command.kind == route::action::drive_turn_drive && command.transition == route::turn_mode::rolling;
    const double elapsed = route_time - action_started_at;
    if (!rolling && (elapsed > command.timeout_ms || (!done && elapsed >= command.timeout_ms))) {
      FinishRoutine("TIMEOUT");
      return;
    }
    if (command.kind == route::action::drive_turn_drive && command.transition == route::turn_mode::stop &&
        done && transfer_phase < 2) {
      ++transfer_phase;
      const bool started = transfer_phase == 1 ? StartTurnRelative(command.angle_deg, command.maximum_rpm)
                                               : StartDriveStraight(command.after_mm, command.maximum_rpm);
      if (!started)
        FinishRoutine(transfer_phase == 1 ? Turning.Info().status : Chassis.Info().status);
      return;
    }
    if (command.kind == route::action::point && done) {
      if (!PointPoseReady()) {
        FinishRoutine("POSE ERROR");
        return;
      }
      const double distance = std::hypot(command.x_mm - position.x_mm, command.y_mm - position.y_mm);
      if (distance <= point_settings.position_tolerance_mm) {
        point_stage = point_phase::idle;
      } else if (point_stage == point_phase::facing) {
        // 原地转向也可能让中心移动；启动直行前按最新位置复核目标方位。
        // 只在转向阶段重新对准，不给直行增加横向反馈或在终点绕回。
        if (point_settings.max_heading_realignments > 0 &&
            std::fabs(PointBearingError(command)) > PointTurnTolerance(distance)) {
          if (point_realignments >= point_settings.max_heading_realignments) {
            FinishRoutine("POINT ALIGN");
            return;
          }
          ++point_realignments;
          if (!StartPoint(command) && info.active)
            FinishRoutine(Turning.Info().status);
          return;
        }
        auto settings = straight_settings;
        settings.motion.position_tolerance_mm =
            std::min(settings.motion.position_tolerance_mm, point_settings.position_tolerance_mm / 2);
        Chassis.Configure(settings);
        Chassis.Observe(frame, true, route_time);
        info.motion = autonomous_motion::straight;
        if (!Chassis.StartDriveStraight(distance, command.maximum_rpm)) {
          FinishRoutine(Chassis.Info().status, Chassis.Info().state == drive::straight_state::cancelled
                                                   ? route_result::cancelled
                                                   : route_result::error);
          return;
        }
        point_stage = point_phase::driving;
        return;
      } else {
        FinishRoutine("POINT ERROR");
        return;
      }
    }
    if (!done)
      return;
    info.step +=
        command.kind == route::action::drive_turn_drive && command.transition == route::turn_mode::rolling
            ? rolling_span
            : 1;
    route_step_started = false;
    info.waiting = false;
  }
  if (info.active)
    FinishRoutine();
}

void autonomous_program::Publish() {
  info.drive = Chassis.Info();
  info.turn = Turning.Info();
  info.corner = Cornering.Info();
  info.rolling = Rolling.Info();
  info.push = Pushing.Info();
  info.left_mV = info.right_mV = 0;
  if (!info.active)
    return;
  switch (info.motion) {
    case autonomous_motion::straight:
      info.left_mV = info.drive.left_mV;
      info.right_mV = info.drive.right_mV;
      break;
    case autonomous_motion::turn:
      info.left_mV = info.turn.left_mV;
      info.right_mV = info.turn.right_mV;
      break;
    case autonomous_motion::corner:
      info.left_mV = info.corner.left_mV;
      info.right_mV = info.corner.right_mV;
      break;
    case autonomous_motion::rolling:
      info.left_mV = info.rolling.left_mV;
      info.right_mV = info.rolling.right_mV;
      break;
    case autonomous_motion::push:
      info.left_mV = info.push.left_mV;
      info.right_mV = info.push.right_mV;
      break;
    case autonomous_motion::none:
      break;
  }
}

drive::motion_state autonomous_program::ActiveState() const {
  switch (info.motion) {
    case autonomous_motion::straight:
      return Chassis.Info().state;
    case autonomous_motion::turn:
      return Turning.Info().state;
    case autonomous_motion::corner:
      return Cornering.Info().state;
    case autonomous_motion::rolling:
      return Rolling.Info().state;
    case autonomous_motion::push:
      return Pushing.Info().state;
    case autonomous_motion::none:
      return drive::motion_state::idle;
  }
  return drive::motion_state::error;
}

const char* autonomous_program::ActiveStatus() const {
  switch (info.motion) {
    case autonomous_motion::straight:
      return Chassis.Info().status;
    case autonomous_motion::turn:
      return Turning.Info().status;
    case autonomous_motion::corner:
      return Cornering.Info().status;
    case autonomous_motion::rolling:
      return Rolling.Info().status;
    case autonomous_motion::push:
      return Pushing.Info().status;
    case autonomous_motion::none:
      return "ROUTE CONFIG";
  }
  return "CONTROL OWNER";
}

bool autonomous_program::OwnershipValid() const {
  using drive::motion_state;
  return (Chassis.Info().state != motion_state::running || info.motion == autonomous_motion::straight) &&
         (Turning.Info().state != motion_state::running || info.motion == autonomous_motion::turn) &&
         (Cornering.Info().state != motion_state::running || info.motion == autonomous_motion::corner) &&
         (Rolling.Info().state != motion_state::running || info.motion == autonomous_motion::rolling) &&
         (Pushing.Info().state != motion_state::running || info.motion == autonomous_motion::push);
}

void autonomous_program::CancelAll() {
  Chassis.CancelDriveStraight();
  Turning.CancelTurn();
  Cornering.CancelCorner();
  Rolling.Cancel();
  Pushing.Cancel();
}

// 位姿与IMU可有不同零点；只把目标相对位姿的转角交给原IMU转向控制器。
bool autonomous_program::PointPoseReady() const {
  return position.valid && !position.needs_reset && std::isfinite(position.x_mm) &&
         std::isfinite(position.y_mm) && std::isfinite(position.heading_deg) &&
         std::isfinite(position.time_ms) && position.time_ms >= 0 && route_time >= position.time_ms &&
         route_time - position.time_ms <= point_settings.pose_stale_ms;
}
bool autonomous_program::StartPoint(const route::command& command) {
  if (!PointPoseReady()) {
    FinishRoutine("POSE ERROR");
    return false;
  }
  if (!std::isfinite(command.x_mm) || !std::isfinite(command.y_mm) || !std::isfinite(command.maximum_rpm) ||
      command.maximum_rpm <= 0 || command.maximum_rpm > 600 || !std::isfinite(command.turn_rpm) ||
      command.turn_rpm <= 0 || command.turn_rpm > 600 ||
      !std::isfinite(point_settings.position_tolerance_mm) || point_settings.position_tolerance_mm <= 0 ||
      !std::isfinite(point_settings.pose_stale_ms) || point_settings.pose_stale_ms <= 0 ||
      !std::isfinite(point_settings.turn_finish_speed_scale) || point_settings.turn_finish_speed_scale <= 0 ||
      point_settings.max_heading_realignments > 2) {
    FinishRoutine("POINT CONFIG");
    return false;
  }
  const double dx = command.x_mm - position.x_mm, dy = command.y_mm - position.y_mm;
  const double distance = std::hypot(dx, dy);
  if (!std::isfinite(distance)) {
    FinishRoutine("POINT CONFIG");
    return false;
  }
  const double delta = distance <= point_settings.position_tolerance_mm ? 0 : PointBearingError(command);
  point_stage = point_phase::facing;
  auto settings = turn_settings;
  settings.angle_tolerance_deg = PointTurnTolerance(distance);
  // 严格角度窗口不变；补角下限小幅上调，仍随容差缩放且不超过普通转向配置。
  settings.minimum_finish_rpm *=
      std::min(1.0, point_settings.turn_finish_speed_scale * settings.angle_tolerance_deg /
                        turn_settings.angle_tolerance_deg);
  Turning.Configure(settings);
  Turning.Observe(frame, true, route_time);
  info.motion = autonomous_motion::turn;
  return Turning.StartTurnRelative(delta, command.turn_rpm);
}

// 位姿朝向与IMU零点可能不同，只返回相对转角，保持原坐标转换约定。
double autonomous_program::PointBearingError(const route::command& command) const {
  double delta = std::fmod(
      std::atan2(command.y_mm - position.y_mm, command.x_mm - position.x_mm) * 180 / 3.14159265358979323846 -
          std::fmod(position.heading_deg, 360.0),
      360.0);
  if (delta <= -180)
    delta += 360;
  if (delta > 180)
    delta -= 360;
  return delta;
}

double autonomous_program::PointTurnTolerance(double distance_mm) const {
  // 半径一半分给横向误差、一半给沿程误差，不放宽原角度窗口。
  return distance_mm <= point_settings.position_tolerance_mm
             ? turn_settings.angle_tolerance_deg
             : std::min(turn_settings.angle_tolerance_deg,
                        std::atan2(point_settings.position_tolerance_mm / 2, distance_mm) * 180 /
                            3.14159265358979323846);
}
