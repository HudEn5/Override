#include "autonomous.h"
#include <cassert>
#include <cstdio>
#include <cstring>

void autonomous_program::RunRoutine() {
  if (info.step == 0) {
    assert(StartDriveCorner(500,90,500,100));
    assert(!StartDriveStraight(100,100) && !StartTurnRelative(90,60));
    info.step = 1;
  } else if (info.step == 1 && CornerState() == drive::corner_state::done) {
    assert(StartDriveStraight(100,60)); info.step = 2;
  } else if (info.step == 2 && DriveStraightState() == drive::straight_state::done) FinishRoutine();
}
int main() {
  drive::straight_config s; s.motion.wheel_diameter_mm = 69.85; s.motion.motor_revs_per_wheel_rev = 1.2;
  s.left_forward.kS = 400; s.left_forward.kV = 16; s.left_forward.calibrated = true;
  s.right_forward = s.left_reverse = s.right_reverse = s.left_forward;
  drive::turn_config t;
  drive::corner_config c; c.wheel_diameter_mm = 69.85; c.motor_revs_per_wheel_rev = 1.2; c.track_width_mm = 325;
  c.prediction_enabled = true;
  c.left_forward = s.left_forward; c.right_forward = s.right_forward; c.left_reverse = s.left_reverse; c.right_reverse = s.right_reverse;
  for (int scenario = 0; scenario < 4; ++scenario) {
    autonomous_program p; p.Configure(s,t,c);
    drive::straight_feedback f;
    f.motors_online = f.position.valid = f.velocity.valid = f.inertial_ready = f.stopped = true;
    double left = 0, right = 0;
    bool interrupted = false;
    for (int i = 0; i < 2000 && !p.Info().finished; ++i) {
      const auto prev = p.Info();
      const auto motor = [](int voltage) { return std::copysign(std::max(0.0,std::fabs(static_cast<double>(voltage))-400)/16,voltage); };
      left += (motor(prev.left_mV) - left) * 0.25; right += (motor(prev.right_mV) - right) * 0.25;
      f.position.left_deg += left * 0.12; f.position.right_deg += right * 0.12;
      f.heading_deg += (right-left) * (69.85*3.141592653589793/72) / 325 * 180 / 3.141592653589793 * 0.02;
      f.velocity.left_rpm=left; f.velocity.right_rpm=right;
      f.velocity.time_ms=i*20; f.velocity.sequence++;
      f.stopped=std::fabs(left)<5 && std::fabs(right)<5;
      if (scenario && prev.corner.phase==2) {
        if(scenario==1) f.inertial_ready=false;
        if(scenario==3) p.CancelCorner();
        p.Update(scenario!=2,f,i*20);
        assert(!p.Info().active && p.Info().left_mV==0 && p.Info().right_mV==0);
        interrupted=true; break;
      }
      p.Update(true,f,i*20);
    }
    if(scenario) assert(interrupted);
    else {
      assert(p.Info().finished && p.Info().step==2 && std::strcmp(p.Info().status,"DONE")==0);
      assert(std::fabs(f.heading_deg-90)<=0.8);
      const auto run=p.Info().run;
      for(int i=0;i<20;++i) p.Update(true,f,40000+i*20);
      assert(p.Info().run==run && p.Info().left_mV==0);
    }
  }
  std::puts("Autonomous corner checks passed: public API, exclusive ownership, follow-on straight, terminal latch, IMU/disable/cancel stop.");
}
