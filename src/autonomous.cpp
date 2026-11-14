#include "autonomous.h"

void autonomous_program::Update(bool permitted, const drive::straight_feedback& feedback, double now) {
  Chassis.Observe(feedback, permitted, now);
  Turn.Observe(feedback, permitted, now);
  Corner.Observe(feedback, permitted, now);
  if (!permitted) {
    if (info.active) { Chassis.CancelDriveStraight(); Turn.CancelTurn(); Corner.CancelCorner(); info.status = "DISABLED"; }
    info.active = false;
    previously_permitted = false;
    Publish();
    return;
  }
  if (!previously_permitted) {
    Chassis.Reset();
    Turn.Reset(); Corner.Reset(); info.motion = autonomous_motion::none;
    info.active = true; info.finished = false; info.step = 0; ++info.run;
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
    Turn.UpdateTurn();
    if (Turn.Info().state == drive::turn_state::error ||
        Turn.Info().state == drive::turn_state::cancelled) FinishRoutine(Turn.Info().status);
  } else if (info.motion == autonomous_motion::corner) {
    Corner.UpdateCorner();
    if (Corner.Info().state == drive::corner_state::error ||
        Corner.Info().state == drive::corner_state::cancelled) FinishRoutine(Corner.Info().status);
  }
  if (info.active) RunRoutine();
  Publish();
}

void autonomous_program::Publish() {
  info.drive = Chassis.Info(); info.turn = Turn.Info();
  info.corner = Corner.Info();
  info.left_mV = info.active ? (info.motion == autonomous_motion::corner ? info.corner.left_mV :
      info.motion == autonomous_motion::turn ? info.turn.left_mV : info.drive.left_mV) : 0;
  info.right_mV = info.active ? (info.motion == autonomous_motion::corner ? info.corner.right_mV :
      info.motion == autonomous_motion::turn ? info.turn.right_mV : info.drive.right_mV) : 0;
}
