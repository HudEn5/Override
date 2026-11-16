#include "autonomous.h"

void autonomous_program::Update(bool permitted, const drive::straight_feedback& feedback, double now) {
  if (permitted && (!std::isfinite(now) || now < 0 || (previously_permitted && now < route_time))) {
    previously_permitted = true; FinishRoutine("CLOCK ERR"); Publish(); return;
  }
  route_time = now;
  info.time_ms = std::isfinite(now) ? now : info.time_ms;
  Chassis.Observe(feedback, permitted, now);
  Turning.Observe(feedback, permitted, now);
  Cornering.Observe(feedback, permitted, now);
  Pushing.Observe(feedback, permitted, now);
  if (!permitted) {
    if (info.active) FinishRoutine("DISABLED");
    info.active = false;
    previously_permitted = false;
    Publish();
    return;
  }
  if (!previously_permitted) {
    Chassis.Reset();
    Turning.Reset(); Cornering.Reset(); info.motion = autonomous_motion::none;
    Pushing.Reset();
    info.active = true; info.finished = false; info.step = 0; ++info.run;
    route_step_started = false; info.waiting = false; info.result = 1;
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
  if (info.motion == autonomous_motion::straight) {
    Chassis.UpdateDriveStraight();
    if (Chassis.Info().state == drive::straight_state::error ||
        Chassis.Info().state == drive::straight_state::cancelled) FinishRoutine(Chassis.Info().status);
  } else if (info.motion == autonomous_motion::turn) {
    Turning.UpdateTurn();
    if (Turning.Info().state == drive::turn_state::error ||
        Turning.Info().state == drive::turn_state::cancelled) FinishRoutine(Turning.Info().status);
  } else if (info.motion == autonomous_motion::corner) {
    Cornering.UpdateCorner();
    if (Cornering.Info().state == drive::corner_state::error ||
        Cornering.Info().state == drive::corner_state::cancelled) FinishRoutine(Cornering.Info().status);
  } else if (info.motion == autonomous_motion::push) {
    Pushing.Update();
    if (PushState() == drive::push_state::error || PushState() == drive::push_state::cancelled)
      FinishRoutine(Pushing.Info().status);
  }
  if (info.active) RunRoutine();
  Publish();
}

// 每周期只推进当前动作；Wait 使用主循环时钟，不调用阻塞等待或创建额外任务。
void autonomous_program::RunRoute(std::initializer_list<route::command> commands) {
  if (commands.size() == 0) { FinishRoutine("EMPTY"); return; }
  while (info.active && info.step < commands.size()) {
    const auto& command = *(commands.begin() + info.step);
    if (!route_step_started) {
      bool started = false;
      switch (command.kind) {
        case route::action::straight: started = StartDriveStraight(command.first, command.speed); break;
        case route::action::turn: started = StartTurnRelative(command.first, command.speed); break;
        case route::action::heading: started = StartTurnToHeading(command.first, command.speed); break;
        case route::action::corner:
          started = StartDriveCorner(command.first, command.second, command.third, command.speed); break;
        case route::action::push: started = StartPushFor(command.first, command.second); break;
        case route::action::wait:
          if (!std::isfinite(command.first) || command.first < 0) { FinishRoutine("WAIT CONFIG"); return; }
          info.motion = autonomous_motion::none; info.waiting = true; wait_started = route_time; started = true; break;
        default: FinishRoutine("ROUTE CONFIG"); return;
      }
      if (!started) {
        const char* error = info.motion == autonomous_motion::straight ? Chassis.Info().status :
            info.motion == autonomous_motion::turn ? Turning.Info().status :
            info.motion == autonomous_motion::corner ? Cornering.Info().status :
            info.motion == autonomous_motion::push ? Pushing.Info().status : "ROUTE CONFIG";
        FinishRoutine(error); return;
      }
      route_step_started = true;
    }
    bool done = false;
    if (command.kind == route::action::wait) {
      info.status = "WAIT"; done = route_time - wait_started >= command.first;
    } else if (info.motion == autonomous_motion::straight) {
      info.status = Chassis.Info().status; done = DriveStraightState() == drive::straight_state::done;
    } else if (info.motion == autonomous_motion::turn) {
      info.status = Turning.Info().status; done = TurnState() == drive::turn_state::done;
    } else if (info.motion == autonomous_motion::corner) {
      info.status = Cornering.Info().status; done = CornerState() == drive::corner_state::done;
    } else if (info.motion == autonomous_motion::push) {
      info.status = Pushing.Info().status; done = PushState() == drive::push_state::done;
    }
    if (!done) return;
    ++info.step; route_step_started = false; info.waiting = false;
  }
  if (info.active) FinishRoutine();
}

void autonomous_program::Publish() {
  info.drive = Chassis.Info(); info.turn = Turning.Info();
  info.corner = Cornering.Info();
  info.push = Pushing.Info();
  info.left_mV = info.active ? (info.motion == autonomous_motion::corner ? info.corner.left_mV :
      info.motion == autonomous_motion::turn ? info.turn.left_mV :
      info.motion == autonomous_motion::push ? info.push.left_mV :
      info.motion == autonomous_motion::straight ? info.drive.left_mV : 0) : 0;
  info.right_mV = info.active ? (info.motion == autonomous_motion::corner ? info.corner.right_mV :
      info.motion == autonomous_motion::turn ? info.turn.right_mV :
      info.motion == autonomous_motion::push ? info.push.right_mV :
      info.motion == autonomous_motion::straight ? info.drive.right_mV : 0) : 0;
}
