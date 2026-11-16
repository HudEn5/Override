#include "autonomous.h"
#include "performance-config.h"
#include <cassert>
#include <cstdio>
#include <cstring>

// 直接链接src/route.cpp，验证实际下载的动作表及三段高速控制器衔接。
double FullRoute(double response, double drag, bool stress_case=false, bool fast=true) {
  drive::straight_config s;s.motion.wheel_diameter_mm=69.85;s.motion.motor_revs_per_wheel_rev=1.2;
  s.left_forward.kS=400;s.left_forward.kV=16;s.left_forward.calibrated=true;
  s.right_forward=s.left_reverse=s.right_reverse=s.left_forward;
  drive::turn_config t;t.left_forward=t.right_forward=t.left_reverse=t.right_reverse=s.left_forward;
  t.pid.kP=t.pid.kI=20;t.pid.max_correction=2000;t.pid.max_integral=1200;t.correction_fraction=3;
  autonomous_program p;p.Configure(fast?config::AutoStraightControl(s):config::RouteStraightControl(s),
                                  fast?config::AutoTurnControl(t):config::RouteTurnControl(t));
  drive::straight_feedback f;f.position.valid=f.velocity.valid=f.motors_online=f.stopped=f.inertial_ready=true;
  double left=0,right=0,x=0,y=0;bool seen[3]={false,false,false};unsigned int previous_step=0;
  unsigned int immediate_handoffs=0;
  const double mm_s_per_rpm=69.85*3.141592653589793/(60*1.2);
  int tick=0;
  for(;tick<2000 && !p.Info().finished;++tick) {
    const auto old=p.Info();
    const auto motor=[&](int mv){return std::copysign(std::max(0.,std::fabs(static_cast<double>(mv))-400-drag)/16,mv);};
    left+=(motor(old.left_mV)*.98-left)*response;right+=(motor(old.right_mV)*1.02-right)*response;
    f.position.left_deg+=left*.12;f.position.right_deg+=right*.12;
    f.heading_deg+=(right-left)*mm_s_per_rpm/325*180/3.141592653589793*.02;
    const double distance=(left+right)*.5*mm_s_per_rpm*.02;
    x+=distance*std::cos(f.heading_deg*3.141592653589793/180);
    y+=distance*std::sin(f.heading_deg*3.141592653589793/180);
    f.velocity.left_rpm=left;f.velocity.right_rpm=right;
    f.velocity.time_ms=tick*20;++f.velocity.sequence;f.stopped=std::fabs(left)<5 && std::fabs(right)<5;
    p.Update(true,f,tick*20);const auto a=p.Info();
    assert(!a.waiting && std::abs(a.left_mV)<=12000 && std::abs(a.right_mV)<=12000);
    if(fast && config::AUTO_SETTLE_MS==0) {
      // 到位/零速已满足的当周期必须接下段，不能再停留在SETTLE计时。
      if(old.step==0 && a.drive.motion.elapsed_ms>=a.drive.motion.duration_ms &&
          std::fabs(a.drive.motion.remaining_mm)<=5 && std::fabs(left)<=5 && std::fabs(right)<=5) {
        assert(a.step>=1);++immediate_handoffs;
      }
      if(old.step==1 && a.turn.elapsed_ms>=a.turn.profile_duration_ms &&
          std::fabs(a.turn.error_deg)<=.8 && std::fabs(a.turn.rate_deg_s)<=2.5 &&
          std::fabs(left)<=5 && std::fabs(right)<=5) {
        assert(a.step>=2);++immediate_handoffs;
      }
    }
    if(a.step<3) {
      seen[a.step]=true;
      assert(a.motion==(a.step==1?autonomous_motion::turn:autonomous_motion::straight));
    }
    if(a.step!=previous_step && a.step<3) {
      assert(f.stopped); // 每段到位停稳后在同一调度周期衔接，无额外Wait。
      if(a.step==1)assert(std::fabs(a.drive.motion.distance_mm-1500)<=5);
      if(a.step==2)assert(std::fabs(a.drive.heading.target-90)<=.8);
    }
    previous_step=a.step;
  }
  const auto end=p.Info();
  if(stress_case && end.result!=2) {
    assert(end.finished && end.result==4 && std::strcmp(end.status,"TIMEOUT")==0);
    assert(end.left_mV==0 && end.right_mV==0);
    std::printf("Actual route stress model: response %.2f drag %.0f stopped on TIMEOUT at step %u, no follow-on movement.\n",response,drag,end.step);
    return 0;
  }
  if(end.result!=2)std::printf("Actual route failed: response %.2f drag %.0f step %u status %s\n",response,drag,end.step,end.status);
  assert(end.finished && end.result==2 && end.step==3 && seen[0] && seen[1] && seen[2]);
  if(fast && config::AUTO_SETTLE_MS==0)assert(immediate_handoffs==2);
  assert(std::fabs(end.drive.motion.remaining_mm)<=5 && std::fabs(end.turn.error_deg)<=.8);
  assert(x>1350 && x<1650 && y>1350 && y<1650);
  for(int n=0;n<20;++n){f.velocity.time_ms=(tick+n)*20;++f.velocity.sequence;p.Update(true,f,(tick+n)*20);}
  assert(p.Info().run==1 && p.Info().step==3 && p.Info().left_mV==0 && p.Info().right_mV==0);
  std::printf("Actual 1500/90/1500 route: %s response %.2f drag %.0f DONE %.0fms, endpoint model (%.0f,%.0f)mm, heading %.2fdeg.\n",
      fast?"fast-handoff":"route1",response,drag,end.time_ms,x,y,f.heading_deg);
  return end.time_ms;
}
int main() {
  autonomous_program p;
  drive::straight_config s; s.motion.wheel_diameter_mm = 69.85; s.motion.motor_revs_per_wheel_rev = 1.2;
  s.left_forward.kS = 400; s.left_forward.kV = 16; s.left_forward.calibrated = true;
  s.right_forward = s.left_reverse = s.right_reverse = s.left_forward;
  drive::corner_config c; c.wheel_diameter_mm = 69.85; c.motor_revs_per_wheel_rev = 1.2; c.track_width_mm = 325;
  c.left_forward = c.right_forward = c.left_reverse = c.right_reverse = s.left_forward;
  p.Configure(config::HighSpeedStraightControl(s), drive::turn_config(), config::HighSpeedCornerControl(c));
  drive::straight_feedback frame;
  frame.position.valid = frame.velocity.valid = frame.motors_online = frame.stopped = frame.inertial_ready = true;
  p.Update(false, frame, 0); assert(p.Info().run == 0 && !p.Info().active);
  p.Update(true, frame, 10);
  assert(p.Info().active && p.Info().result == 1 && p.Info().run == 1);
  p.Update(false, frame, 20);
  assert(!p.Info().active && p.Info().left_mV == 0 && p.Info().right_mV == 0);
  for (int i = 0; i < 20; ++i) p.Update(false, frame, 30 + i * 10);
  assert(p.Info().run == 1);
  for(double response:{.15,.25})for(double drag:{0.,200.}) {
    const double before=FullRoute(response,drag,false,false);
    const double after=FullRoute(response,drag);
    if(config::AUTO_SETTLE_MS<250)assert(after<before);
    std::printf("Handoff comparison: %.0f -> %.0f ms, saved %.0f ms.\n",before,after,before-after);
  }
  FullRoute(.15,600,true); // 超出既有直行摩擦模型覆盖时，仍需验证超时撤输出。
  std::puts("Route entry checks passed: actual route builds, explicit enable required and disable stops.");
}
