#include "performance-config.h"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <limits>

// 电压驱动负载模型：与规划器分离，用实际轮速积分航向，允许轮距/响应和摩擦变化。
struct turn_result {
  bool done = false;
  double elapsed = 0, error = 0, peak_rpm = 0, overshoot = 0, profile_ms = 0;
  double last_degrees_ms = -1, window_ms = -1;
};
drive::turn_config TurnSettings() {
  drive::turn_config c;
  c.kP = 2; c.kD = .25; c.acceleration_rpm_s = 640;
  c.minimum_settle_rpm = 18; c.minimum_finish_rpm = 12; c.minimum_rate_deg_s = 16;
  c.correction_fraction = 3; c.pid.kP = 20; c.pid.kI = 20;
  c.pid.max_correction = 2000; c.pid.max_integral = 1200;
  c.left_forward.kS = 372.573735; c.left_forward.kV = 17.900544;
  c.right_forward.kS = 408.162564; c.right_forward.kV = 16.537990;
  c.left_reverse.kS = 547.829766; c.left_reverse.kV = 16.939618;
  c.right_reverse.kS = 537.033135; c.right_reverse.kV = 15.870378;
  c.left_forward.calibrated = c.right_forward.calibrated = c.left_reverse.calibrated = c.right_reverse.calibrated = true;
  return c;
}
drive::turn_config Turn2Settings() {
  auto c=config::RouteTurnControl(TurnSettings());
  c.angular_acceleration_deg_s2=900;c.angular_deceleration_deg_s2=1200;
  c.profile_braking_delay_ms=120;c.rate_filter_ms=60;
  c.profile_finish_kP=6;c.minimum_finish_rpm=30;
  c.minimum_settle_rpm=50;c.minimum_rate_deg_s=32;
  return c;
}
drive::turn_config Turn1Settings() {
  auto c=Turn2Settings();
  c.profile_finish_kP=0;c.minimum_rate_deg_s=16;
  c.minimum_settle_rpm=30;c.minimum_finish_rpm=24;
  return c;
}
turn_result RunTurn(drive::turn_config cfg, double angle, double cap, double response = .25,
                    double friction = 900, double width = 1, double right_response = .25, bool cached = false) {
  drive::turn_controller c; c.Configure(cfg);
  drive::straight_feedback f;
  f.velocity.valid = f.motors_online = f.inertial_ready = f.stopped = true;
  f.heading_deg = 350;
  c.Observe(f, true, 0); assert(c.StartTurnRelative(angle, cap));
  const double rate_per_rpm = 6 * cfg.wheel_diameter_mm / cfg.motor_revs_per_wheel_rev / cfg.track_width_mm;
  const double dt = cached ? .01 : .02;
  const double left_response = cached ? 1-std::sqrt(1-response) : response;
  const double right_step = cached ? 1-std::sqrt(1-right_response) : right_response;
  double left = 0, right = 0, previous = 0, physical_angle = 350;
  turn_result result;
  for (int tick = 1; tick <= (cached ? 1300 : 650); ++tick) {
    const auto old = c.Info();
    const auto motor = [&](int mv, bool l) {
      const auto ff = l ? (mv < 0 ? cfg.left_reverse : cfg.left_forward) :
                          (mv < 0 ? cfg.right_reverse : cfg.right_forward);
      return std::copysign(std::max(0.0, std::fabs(static_cast<double>(mv)) - ff.kS - friction) / ff.kV, mv);
    };
    left += (motor(old.left_mV, true) * .97 - left) * left_response;
    right += (motor(old.right_mV, false) * 1.03 - right) * right_step;
    physical_angle += (right - left) * rate_per_rpm / (2 * width) * dt;
    if (!cached || tick%2==0) {
      // 模拟20ms SDK刷新、10ms主循环及0.01度IMU量化，不能假定每次读取都有新数据。
      f.heading_deg = cached ? std::round(physical_angle*100)/100 : physical_angle;
      f.velocity.left_rpm = left; f.velocity.right_rpm = right;
      f.velocity.time_ms = tick * dt * 1000; ++f.velocity.sequence;
      f.stopped = std::fabs(left) < 5 && std::fabs(right) < 5;
    }
    c.Observe(f, true, tick * dt * 1000); c.UpdateTurn();
    const auto v = c.Info();
    assert(std::abs(v.left_mV) <= 12000 && std::abs(v.right_mV) <= 12000);
    assert(std::fabs(v.target_rpm) <= cap + 1e-6);
    if (cfg.profile_enabled && std::strcmp(v.status, "TURN") == 0) {
      // 分别核对加速/减速预算，反向制动按最保守总变化上限检查。
      const double budget = (previous * v.target_rpm < 0 ?
          std::max(cfg.angular_acceleration_deg_s2, cfg.angular_deceleration_deg_s2) :
          std::fabs(v.target_rpm) > std::fabs(previous) ? cfg.angular_acceleration_deg_s2 : cfg.angular_deceleration_deg_s2);
      assert(std::fabs(v.target_rpm - previous) <= budget / rate_per_rpm * dt + 1e-6);
      assert(v.profile_phase >= 1 && v.profile_phase <= 4);
      assert(std::fabs(v.reference_rate_deg_s) <= cap * rate_per_rpm + 1e-6);
    }
    previous = v.target_rpm;
    result.elapsed = v.elapsed_ms; result.error = v.error_deg;
    result.peak_rpm = std::max(result.peak_rpm, std::fabs((right-left)/2));
    result.overshoot = std::max(result.overshoot, angle > 0 ? -v.error_deg : v.error_deg);
    result.profile_ms = v.profile_duration_ms;
    if (result.last_degrees_ms < 0 && std::fabs(v.error_deg) <= 6) result.last_degrees_ms = v.elapsed_ms;
    if (result.window_ms < 0 && std::fabs(v.error_deg) <= cfg.angle_tolerance_deg) result.window_ms = v.elapsed_ms;
    if (v.state != drive::turn_state::running) {
      result.done = v.state == drive::turn_state::done;
      if (!result.done) std::printf("Turn failed: angle %.0f cap %.0f lag %.2f/%.2f friction %.0f width %.2f %s error %.2f\n",
          angle, cap, response, right_response, friction, width, v.status, v.error_deg);
      break;
    }
  }
  return result;
}
drive::straight_feedback Ready(double time = 0, double angle = 350) {
  drive::straight_feedback f;
  f.velocity.valid = f.motors_online = f.inertial_ready = f.stopped = true;
  f.heading_deg = angle; f.velocity.time_ms = time;
  f.velocity.sequence = static_cast<unsigned int>(time/20+1);
  return f;
}
void ProfileContracts() {
  auto cfg = config::RouteTurnControl(TurnSettings());
  const double conversion = 6 * cfg.wheel_diameter_mm / cfg.motor_revs_per_wheel_rev / cfg.track_width_mm;
  // 理想跟随验证参考角加速度、独立减速度、短弯三角形和长弯匀速段。
  for (double angle : {-360., -90., 0., .5, 30., 90., 360.}) for (double cap : {60., 540.}) {
    drive::turn_controller c; c.Configure(cfg);
    auto f = Ready(); c.Observe(f, true, 0); assert(c.StartTurnRelative(angle, cap));
    motion::TrapezoidProfile reference(angle, cap*conversion,
        cfg.angular_acceleration_deg_s2, cfg.angular_deceleration_deg_s2);
    assert(std::fabs(c.Info().profile_duration_ms-reference.duration()*1000)<1e-6);
    if (std::fabs(angle)==90 && cap==540) assert(reference.cruiseTime()<1e-9);
    if (std::fabs(angle)==360 && cap==60) assert(reference.cruiseTime()>0);
    double previous_rate=0; int acceleration_samples=0,deceleration_samples=0;
    for (int tick=1;tick<650 && c.Info().state==drive::turn_state::running;++tick) {
      const double now=tick*20;
      const auto r=reference.sample(now/1000);
      f=Ready(now,350+r.position);
      f.velocity.left_rpm=-r.velocity/conversion; f.velocity.right_rpm=r.velocity/conversion;
      f.stopped=std::fabs(r.velocity/conversion)<=5;
      c.Observe(f,true,now); c.UpdateTurn(); const auto v=c.Info();
      assert(std::fabs(v.reference_deg-f.heading_deg)<1e-6);
      assert(std::fabs(v.reference_rate_deg_s-r.velocity)<1e-6);
      const double slope=(std::fabs(r.velocity)-std::fabs(previous_rate))/.02;
      assert(slope<=cfg.angular_acceleration_deg_s2+1e-6);
      assert(slope>=-cfg.angular_deceleration_deg_s2-1e-6);
      acceleration_samples+=slope>1; deceleration_samples+=slope<-1; previous_rate=r.velocity;
    }
    assert(c.Info().state==drive::turn_state::done);
    if (std::fabs(angle)>=30) assert(acceleration_samples && deceleration_samples);
  }
  // 新增配置参数的无效值必须拒绝，不能带着NaN/非法几何数据输出。
  for (int item=0;item<7;++item) for (double invalid : {-1.,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}) {
    auto bad=cfg;
    double* fields[]={&bad.wheel_diameter_mm,&bad.motor_revs_per_wheel_rev,&bad.track_width_mm,
        &bad.angular_acceleration_deg_s2,&bad.angular_deceleration_deg_s2,&bad.profile_braking_delay_ms,&bad.profile_finish_kP};
    *fields[item]=invalid;
    drive::turn_controller c;c.Configure(bad);c.Observe(Ready(),true,0);
    assert(!c.StartTurnRelative(90,540) && c.Info().left_mV==0 && c.Info().right_mV==0);
  }
  for (int fault=0;fault<5;++fault) {
    drive::turn_controller c;c.Configure(cfg);c.Observe(Ready(),true,0);assert(c.StartTurnRelative(90,540));
    auto f=Ready(20);c.Observe(f,true,20);c.UpdateTurn();
    f=Ready(40);
    if(fault==0)f.motors_online=false;
    if(fault==1)f.inertial_ready=false;
    if(fault==2)f.velocity.valid=false;
    c.Observe(f,fault!=3,fault==4?260:40);c.UpdateTurn();
    assert(c.Info().state==drive::turn_state::error && c.Info().left_mV==0 && c.Info().right_mV==0);
    c.Observe(Ready(280),true,280);c.UpdateTurn();assert(c.Info().state==drive::turn_state::error);
  }
  drive::turn_controller c;c.Configure(cfg);c.Observe(Ready(),true,0);
  assert(c.StartTurnToHeading(10,540) && c.Info().target_deg==370);
  assert(!c.StartTurnRelative(-90,540));c.CancelTurn();assert(c.Info().left_mV==0 && c.Info().right_mV==0);
  c.Observe(Ready(20,10),true,20);assert(c.StartTurnToHeading(350,540) && c.Info().target_deg==-10);
  assert(std::fabs(c.Info().reference_deg-10)<1e-6 && c.Info().reference_rate_deg_s==0);
  c.Reset();assert(c.Info().state==drive::turn_state::idle && !c.Info().profiled);
  cfg.timeout_ms=80;c.Configure(cfg);c.Observe(Ready(),true,0);assert(c.StartTurnRelative(90,540));
  for(int t=20;t<=80;t+=20){c.Observe(Ready(t),true,t);c.UpdateTurn();}
  assert(c.Info().state==drive::turn_state::error && std::strcmp(c.Info().status,"TIMEOUT")==0);
  cfg.timeout_ms=12000; cfg.profile_braking_delay_ms=1e308;
  c.Configure(cfg);c.Observe(Ready(),true,0);assert(c.StartTurnRelative(90,540));
  c.Observe(Ready(20),true,20);c.UpdateTurn();
  assert(c.Info().state==drive::turn_state::error && std::strcmp(c.Info().status,"PROFILE ERR")==0);
  assert(c.Info().left_mV==0 && c.Info().right_mV==0);
  // 单独切换末端补角时，同一规划在参考结束前必须给出相同输出。
  auto fast=config::RouteTurnControl(TurnSettings());auto old=fast;
  old.profile_finish_kP=0;old.minimum_rate_deg_s=16;
  old.minimum_settle_rpm=30;old.minimum_finish_rpm=24;
  drive::turn_controller a,b;a.Configure(old);b.Configure(fast);
  auto f=Ready();a.Observe(f,true,0);b.Observe(f,true,0);
  assert(a.StartTurnRelative(90,540) && b.StartTurnRelative(90,540));
  for(int time=20;time<a.Info().profile_duration_ms;time+=20) {
    f=Ready(time,350+std::min(83.,time*.12));
    a.Observe(f,true,time);b.Observe(f,true,time);a.UpdateTurn();b.UpdateTurn();
    assert(a.Info().target_rpm==b.Info().target_rpm);
    assert(a.Info().left_mV==b.Info().left_mV && a.Info().right_mV==b.Info().right_mV);
  }
  for(int time=580;time<=1500;time+=20) {
    f=Ready(time,433);a.Observe(f,true,time);b.Observe(f,true,time);a.UpdateTurn();b.UpdateTurn();
  }
  if(config::TURN_FAST_FINISH) assert(b.Info().target_rpm>a.Info().target_rpm+5);
  std::puts("Angular profile contracts passed: references, slopes, geometry, faults, restart/wrap and unchanged outputs before finish correction.");
}
int main(int argc, char**) {
  if (!config::EXPERIMENTAL_FAST_ROUTE || !config::TURN_ANGULAR_PROFILE) {
    assert(!config::RouteTurnControl(TurnSettings()).profile_enabled);
    std::puts("Angular profile disabled by configuration; legacy turn regression remains enabled.");
    return 0;
  }
  if (argc > 1) {
    for (double finish_p : {4., 6., 8.}) for (int floor_case=0;floor_case<4;++floor_case) {
      auto cfg = config::RouteTurnControl(TurnSettings());
      cfg.profile_finish_kP = finish_p; cfg.minimum_rate_deg_s = 32;
      cfg.minimum_settle_rpm=floor_case==0?30:floor_case==3?50:40;
      cfg.minimum_finish_rpm=floor_case<2?24:30;
      double sum = 0, overshoot = 0; unsigned int count = 0; bool done = true;
      for (double angle : {-120., -90., -30., 30., 90., 120.}) for (double friction : {0.,900.})
        for (double width : {.9,1.,1.1}) for (int response = 0; response < 3; ++response) {
          const auto v = RunTurn(cfg, angle, 540, response==1?.12:.25,friction,width,response==2?.12:.25);
          sum += v.elapsed; overshoot = std::max(overshoot, v.overshoot); done = done && v.done; ++count;
        }
      const auto nominal = RunTurn(cfg,90,540);
      std::printf("FinishP %.1f floors %.0f/%.0f: done %d mean %.0f nominal %.0f last6-to-window %.0f last6-to-DONE %.0f overshoot %.2f\n",finish_p,cfg.minimum_settle_rpm,cfg.minimum_finish_rpm,done,sum/count,nominal.elapsed,
          nominal.window_ms-nominal.last_degrees_ms,nominal.elapsed-nominal.last_degrees_ms,overshoot);
    }
    return 0;
  }
  unsigned int cases = 0; double old_time = 0, new_time = 0, max_error = 0, max_overshoot = 0;
  double turn2_time=0,turn2_window_tail=0,new_window_tail=0;
  double turn1_time=0,turn1_tail=0,new_tail=0;
  double auto_time=0,auto_error=0;
  for (double angle : {-150., -120., -90., -30., 30., 90., 120., 150.}) for (double friction : {0., 900.})
    for (double width : {.9, 1., 1.1}) for (int response = 0; response < 3; ++response) {
      const double l = response == 1 ? .12 : .25, r = response == 2 ? .12 : .25;
      const auto before = RunTurn(TurnSettings(), angle, 540, l, friction, width, r);
      const auto turn1 = RunTurn(Turn1Settings(), angle, 540, l, friction, width, r);
      const auto turn2 = RunTurn(Turn2Settings(), angle, 540, l, friction, width, r);
      const auto after = RunTurn(config::RouteTurnControl(TurnSettings()), angle, 540, l, friction, width, r);
      const auto automatic = RunTurn(config::AutoTurnControl(TurnSettings()), angle, 540, l, friction, width, r);
      assert(before.done && turn1.done && after.done && std::fabs(after.error) <= .8);
      assert(automatic.done && std::fabs(automatic.error)<=.8);
      assert(turn2.done);
      turn2_time+=turn2.elapsed;turn2_window_tail+=turn2.window_ms-turn2.last_degrees_ms;
      new_window_tail+=after.window_ms-after.last_degrees_ms;
      auto_time+=automatic.elapsed;auto_error=std::max(auto_error,std::fabs(automatic.error));
      turn1_time+=turn1.elapsed;turn1_tail+=turn1.elapsed-turn1.last_degrees_ms;
      new_tail+=after.elapsed-after.last_degrees_ms;
      old_time += before.elapsed; new_time += after.elapsed;
      max_error = std::max(max_error, std::fabs(after.error)); max_overshoot = std::max(max_overshoot, after.overshoot); ++cases;
    }
  std::printf("Angular profile: %u paired cases, mean %.0f -> %.0f ms, max endpoint %.3f deg, max overshoot %.3f deg.\n",
      cases, old_time/cases, new_time/cases, max_error, max_overshoot);
  std::printf("Continuous braking vs turn2: %u cases mean %.0f -> %.0fms, final 6deg to angle window %.0f -> %.0fms.\n",
      cases,turn2_time/cases,new_time/cases,turn2_window_tail/cases,new_window_tail/cases);
  if(config::TURN_FAST_FINISH) assert(new_time<turn2_time && new_window_tail<turn2_window_tail);
  const auto before = RunTurn(TurnSettings(), 90, 540);
  std::printf("Automatic settle %.0fms: %u cases DONE, mean %.0fms, max endpoint %.3fdeg; isolated tests remain %.0fms.\n",
      config::AutoTurnControl(TurnSettings()).settle_ms,cases,auto_time/cases,auto_error,
      config::RouteTurnControl(TurnSettings()).settle_ms);
  const auto turn1=RunTurn(Turn1Settings(),90,540);
  const auto turn2=RunTurn(Turn2Settings(),90,540);
  const auto after = RunTurn(config::RouteTurnControl(TurnSettings()), 90, 540);
  std::printf("Nominal 90 vs turn2: %.0f -> %.0fms; final 6deg to angle window %.0f -> %.0fms.\n",
      turn2.elapsed,after.elapsed,turn2.window_ms-turn2.last_degrees_ms,after.window_ms-after.last_degrees_ms);
  std::printf("Finish correction %u cases: turn1 -> candidate mean %.0f -> %.0f ms, last 6deg-to-DONE %.0f -> %.0f ms; nominal 90 %.0f -> %.0f ms, last 6deg %.0f -> %.0f ms.\n",
      cases,turn1_time/cases,new_time/cases,turn1_tail/cases,new_tail/cases,turn1.elapsed,after.elapsed,
      turn1.elapsed-turn1.last_degrees_ms,after.elapsed-after.last_degrees_ms);
  std::printf("Nominal last 6deg-to-first-angle-window: %.0f -> %.0f ms; angle %.1f deg, settling %.0f ms unchanged.\n",
      turn1.window_ms-turn1.last_degrees_ms,after.window_ms-after.last_degrees_ms,
      config::RouteTurnControl(TurnSettings()).angle_tolerance_deg,config::RouteTurnControl(TurnSettings()).settle_ms);
  if(config::TURN_FAST_FINISH) assert(new_time<turn1_time && new_tail<turn1_tail);
  std::printf("Nominal 90: %.0f -> %.0f ms, reference %.0f ms, peak actual %.1f motor rpm, overshoot %.2f deg.\n",
      before.elapsed, after.elapsed, after.profile_ms, after.peak_rpm, after.overshoot);
  assert(after.elapsed<before.elapsed && max_overshoot<=1.5);
  for (double angle : {-360.,-180.,-5.,-.5,.5,5.,180.,360.}) for(double cap:{60.,100.,540.}) {
    const auto r=RunTurn(config::RouteTurnControl(TurnSettings()),angle,cap);
    assert(r.done && std::fabs(r.error)<=.8);
  }
  unsigned int cached_cases=0;
  double cached_error=0;
  for(double angle:{-150.,-120.,-90.,-30.,30.,90.,120.,150.})for(double cap:{60.,100.,540.})
    for(double friction:{0.,900.})for(int lag=0;lag<3;++lag) {
      const auto r=RunTurn(config::AutoTurnControl(TurnSettings()),angle,cap,
          lag==1?.12:.25,friction,1,lag==2?.12:.25,true);
      assert(r.done && std::fabs(r.error)<=.8);
      cached_error=std::max(cached_error,std::fabs(r.error));++cached_cases;
    }
  std::printf("Cached feedback: %u signed/cap/load/response cases DONE, max endpoint %.3fdeg.\n",cached_cases,cached_error);
  ProfileContracts();
}
