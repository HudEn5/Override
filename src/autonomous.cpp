#include "autonomous.h"

void autonomous_program::Update(bool permitted, const drive::straight_feedback& feedback, double now) {
  Chassis.Observe(feedback, permitted, now);
  if (!permitted) {
    if (info.active) { Chassis.CancelDriveStraight(); info.status = "DISABLED"; }
    info.active = false;
    previously_permitted = false;
    info.drive = Chassis.Info();
    return;
  }
  if (!previously_permitted) {
    Chassis.Reset();
    info.active = true; info.finished = false; info.step = 0; ++info.run;
    waiting_ready = true;
    info.status = "RUN";
  }
  previously_permitted = true;
  if (info.active && waiting_ready) {
    if (!feedback.motors_online || !feedback.inertial_ready || !feedback.velocity.valid ||
        !feedback.position.valid || !feedback.stopped) {
      info.status = "WAIT READY";
      info.drive = Chassis.Info();
      return;
    }
    waiting_ready = false;
  }
  Chassis.UpdateDriveStraight();
  if (Chassis.Info().state == drive::straight_state::error ||
      Chassis.Info().state == drive::straight_state::cancelled) FinishRoutine(Chassis.Info().status);
  if (info.active) RunRoutine();
  info.drive = Chassis.Info();
}
