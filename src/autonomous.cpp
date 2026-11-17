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
    info.motion = autonomous_motion::none;
    Pushing.Reset();
    info.active = true;
    info.finished = false;
    info.step = 0;
    ++info.run;
    point_stage = point_phase::idle;
    route_step_started = false;
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
  if (info.motion == autonomous_motion::straight) {
    Chassis.UpdateDriveStraight();
    if (Chassis.Info().state == drive::straight_state::error ||
        Chassis.Info().state == drive::straight_state::cancelled)
      FinishRoutine(Chassis.Info().status, Chassis.Info().state == drive::straight_state::cancelled
                                               ? route_result::cancelled
                                               : route_result::error);
  } else if (info.motion == autonomous_motion::turn) {
    Turning.UpdateTurn();
    if (Turning.Info().state == drive::turn_state::error ||
        Turning.Info().state == drive::turn_state::cancelled)
      FinishRoutine(Turning.Info().status, Turning.Info().state == drive::turn_state::cancelled
                                               ? route_result::cancelled
                                               : route_result::error);
  } else if (info.motion == autonomous_motion::corner) {
    Cornering.UpdateCorner();
    if (Cornering.Info().state == drive::corner_state::error ||
        Cornering.Info().state == drive::corner_state::cancelled)
      FinishRoutine(Cornering.Info().status, Cornering.Info().state == drive::corner_state::cancelled
                                                 ? route_result::cancelled
                                                 : route_result::error);
  } else if (info.motion == autonomous_motion::push) {
    Pushing.Update();
    if (PushState() == drive::push_state::error || PushState() == drive::push_state::cancelled)
      FinishRoutine(Pushing.Info().status, Pushing.Info().state == drive::push_state::cancelled
                                               ? route_result::cancelled
                                               : route_result::error);
  }
  if (info.active)
    RunRoutine();
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
      bool started = false;
      switch (command.kind) {
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
          started = StartPoint(command);
          break;
        case route::action::swing:
          info.motion = autonomous_motion::turn;
          Turning.Configure(turn_settings);
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
        const char* error = info.motion == autonomous_motion::straight ? Chassis.Info().status
                            : info.motion == autonomous_motion::turn   ? Turning.Info().status
                            : info.motion == autonomous_motion::corner ? Cornering.Info().status
                            : info.motion == autonomous_motion::push   ? Pushing.Info().status
                                                                       : "ROUTE CONFIG";
        FinishRoutine(error);
        return;
      }
      route_step_started = true;
    }
    bool done = false;
    if (command.kind == route::action::wait) {
      info.status = "WAIT";
      done = route_time - wait_started >= command.duration_ms;
    } else if (info.motion == autonomous_motion::straight) {
      info.status = Chassis.Info().status;
      done = DriveStraightState() == drive::straight_state::done;
    } else if (info.motion == autonomous_motion::turn) {
      info.status = Turning.Info().status;
      done = TurnState() == drive::turn_state::done;
    } else if (info.motion == autonomous_motion::corner) {
      info.status = Cornering.Info().status;
      done = CornerState() == drive::corner_state::done;
    } else if (info.motion == autonomous_motion::push) {
      info.status = Pushing.Info().status;
      done = PushState() == drive::push_state::done;
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
    ++info.step;
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
  info.push = Pushing.Info();
  info.left_mV = info.active ? (info.motion == autonomous_motion::corner     ? info.corner.left_mV
                                : info.motion == autonomous_motion::turn     ? info.turn.left_mV
                                : info.motion == autonomous_motion::push     ? info.push.left_mV
                                : info.motion == autonomous_motion::straight ? info.drive.left_mV
                                                                             : 0)
                             : 0;
  info.right_mV = info.active ? (info.motion == autonomous_motion::corner     ? info.corner.right_mV
                                 : info.motion == autonomous_motion::turn     ? info.turn.right_mV
                                 : info.motion == autonomous_motion::push     ? info.push.right_mV
                                 : info.motion == autonomous_motion::straight ? info.drive.right_mV
                                                                              : 0)
                              : 0;
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
      !std::isfinite(point_settings.pose_stale_ms) || point_settings.pose_stale_ms <= 0) {
    FinishRoutine("POINT CONFIG");
    return false;
  }
  const double dx = command.x_mm - position.x_mm, dy = command.y_mm - position.y_mm;
  const double distance = std::hypot(dx, dy);
  if (!std::isfinite(distance)) {
    FinishRoutine("POINT CONFIG");
    return false;
  }
  double delta = distance <= point_settings.position_tolerance_mm
                     ? 0
                     : std::fmod(std::atan2(dy, dx) * 180 / 3.14159265358979323846 -
                                     std::fmod(position.heading_deg, 360.0),
                                 360.0);
  if (delta <= -180)
    delta += 360;
  if (delta > 180)
    delta -= 360;
  point_stage = point_phase::facing;
  auto settings = turn_settings;
  // 分配半径的一半给横向误差，另一半给沿程误差，保留原更严格的限制。
  if (distance > point_settings.position_tolerance_mm)
    settings.angle_tolerance_deg = std::min(
        settings.angle_tolerance_deg,
        std::atan2(point_settings.position_tolerance_mm / 2, distance) * 180 / 3.14159265358979323846);
  settings.minimum_finish_rpm *=
      std::min(1.0, settings.angle_tolerance_deg / turn_settings.angle_tolerance_deg);
  Turning.Configure(settings);
  Turning.Observe(frame, true, route_time);
  info.motion = autonomous_motion::turn;
  return Turning.StartTurnRelative(delta, command.turn_rpm);
}
