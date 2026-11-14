#include "drive-corner.h"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <limits>

drive::corner_config Settings() {
  drive::corner_config c; c.wheel_diameter_mm = 69.85; c.motor_revs_per_wheel_rev = 1.2;
  c.track_width_mm = 325;
  c.left_forward.kS = 400; c.left_forward.kV = 16; c.left_forward.calibrated = true;
  c.right_forward = c.left_reverse = c.right_reverse = c.left_forward;
  c.pid.kP = 20; c.pid.kI = 20; c.pid.max_correction = 2000; c.pid.max_integral = 1200;
  return c;
}
drive::straight_feedback Frame(double now) {
  drive::straight_feedback f;
  f.velocity.valid = f.position.valid = f.motors_online = f.inertial_ready = f.stopped = true;
  f.velocity.sequence = static_cast<unsigned int>(now / 20 + 1); f.velocity.time_ms = now;
  return f;
}
drive::corner_info Simulate(double angle, double width_scale, double drag,
                            double left_response = 0.25, double right_response = 0.25, bool fast = false,
                            bool governed = false) {
  auto cfg = Settings();
  const double maximum = governed ? 600 : fast ? 150 : 100;
  if (fast) {
    cfg.corner_rpm = 65; cfg.acceleration_rpm_s = 180;
    cfg.exit_angle_tolerance_deg = 1.5; cfg.exit_rate_tolerance_deg_s = 4;
    cfg.angle_tolerance_deg = 1.5;
    cfg.finish_min_rpm = 12; cfg.finish_max_rpm = 30; cfg.finish_kP = 0.6;
  }
  if (governed) {
    cfg.corner_rpm=140; cfg.corner_floor_rpm=65;
    cfg.corner_acceleration_rpm_s=60; cfg.entry_braking_delay_ms=200;
  }
  drive::corner_controller c; c.Configure(cfg);
  auto f = Frame(0); c.Observe(f, true, 0); assert(c.StartDriveCorner(500, angle, 500, maximum));
  assert(!c.StartDriveCorner(500, -angle, 500, 100));
  double left = 0, right = 0, last_l = 0, last_r = 0, previous_heading = 0;
  bool incoming = false, corner = false, outgoing = false;
  const double mm_per_rpm = 69.85 * 3.141592653589793 / 72;
  for (int i = 1; i <= 1000; ++i) {
    const auto old = c.Info();
    const auto motor = [&](int voltage) {
      return std::copysign(std::max(0.0, std::fabs(static_cast<double>(voltage)) - drag) / 16, voltage);
    };
    left += (motor(old.left_mV) * 0.97 - left) * left_response;
    right += (motor(old.right_mV) * 1.03 - right) * right_response;
    f.position.left_deg += left * 6 * 0.02; f.position.right_deg += right * 6 * 0.02;
    f.heading_deg += (right - left) * mm_per_rpm / (325 * width_scale) * 180 / 3.141592653589793 * 0.02;
    f.velocity.left_rpm = left; f.velocity.right_rpm = right;
    f.velocity.sequence++; f.velocity.time_ms = i * 20;
    f.stopped = std::fabs(left) < 5 && std::fabs(right) < 5;
    c.Observe(f, true, i * 20); c.UpdateCorner();
    const auto v = c.Info();
    if (v.state == drive::corner_state::error) {
      std::printf("corner failed angle %.0f width %.2f drag %.0f response %.2f/%.2f phase %d distance %.1f error %.2f: %s\n",
                  angle, width_scale, drag, left_response, right_response, v.phase, v.distance_mm, v.error_deg, v.status);
    }
    assert(v.state != drive::corner_state::error);
    assert(std::abs(v.left_mV) <= 12000 && std::abs(v.right_mV) <= 12000);
    assert(std::fabs(v.left_target_rpm) <= maximum + 1e-6 && std::fabs(v.right_target_rpm) <= maximum + 1e-6);
    if (v.state == drive::corner_state::running && v.phase != 4) {
      assert(std::fabs(v.left_target_rpm - last_l) <= cfg.acceleration_rpm_s * 0.02 + 1e-6);
      assert(std::fabs(v.right_target_rpm - last_r) <= cfg.acceleration_rpm_s * 0.02 + 1e-6);
      if (v.distance_mm > 450 && v.distance_mm < 500 + 1.5 * std::fabs(angle) * 3.141592653589793 / 180 * 250 + 50) {
        assert(v.left_target_rpm > 0 && v.right_target_rpm > 0);
        assert((left + right) / 2 > 5); // 两个切点与弯内实际中心速度均未停。
      }
    }
    if (v.phase == 1) incoming = true;
    if (v.phase == 2) corner = true;
    if (v.phase == 3) outgoing = true;
    if (old.phase == 2 && v.phase == 3) assert(std::fabs(v.reference_deg - angle) < 1e-6);
    if (v.distance_mm > 480 && v.distance_mm < 520) assert(std::fabs(v.reference_deg) < 1.0);
    assert(std::fabs(v.reference_deg - previous_heading) < 10);
    previous_heading = v.reference_deg; last_l = v.left_target_rpm; last_r = v.right_target_rpm;
    if (v.state == drive::corner_state::done) break;
  }
  assert(incoming && corner && outgoing);
  assert(c.Info().state == drive::corner_state::done && std::fabs(c.Info().remaining_mm) <= 5);
  assert(std::fabs(c.Info().error_deg) <= cfg.angle_tolerance_deg && c.Info().left_mV == 0 && c.Info().right_mV == 0);
  return c.Info();
}
int main() {
  for (double angle : {-90.0, 90.0})
    for (double width : {0.9, 1.0, 1.1}) Simulate(angle, width, 400);
  Simulate(90, 1, 1000);
  Simulate(90, 1, 400, 0.2, 0.3);
  Simulate(-90, 1, 400, 0.3, 0.2);
  Simulate(90, 1, 400, 0.1, 0.35);
  Simulate(-90, 1, 400, 0.35, 0.1);
  for (double angle : {-90.0, 90.0})
    for (double width : {0.9, 1.0, 1.1}) Simulate(angle, width, 400, 0.25, 0.25, true);
  Simulate(90, 1, 1000, 0.25, 0.25, true);
  Simulate(90, 1, 400, 0.1, 0.35, true);
  Simulate(-90, 1, 400, 0.35, 0.1, true);
  for (double angle : {-90.0,90.0})
    for (double width : {0.9,1.0,1.1}) Simulate(angle,width,400,0.25,0.25,true,true);
  Simulate(90,1,1000,0.1,0.35,true,true);
  Simulate(-90,1,400,0.35,0.1,true,true);
  const auto baseline = Simulate(90,1,400);
  const auto quicker = Simulate(90,1,400,0.25,0.25,true);
  assert(quicker.elapsed_ms < baseline.elapsed_ms); // 只证明此负载模型提速，不代替实车结果。
  drive::corner_controller c; c.Configure(Settings()); assert(!c.StartDriveCorner(500,90,500,100));
  auto f = Frame(0); c.Observe(f, true, 0);
  assert(!c.StartDriveCorner(500,0,500,100)); assert(!c.StartDriveCorner(-1,90,500,100));
  assert(!c.StartDriveCorner(500,90,500,std::numeric_limits<double>::quiet_NaN()));
  for (int fault = 0; fault < 8; ++fault) {
    drive::corner_controller x; x.Configure(Settings()); f = Frame(0);
    x.Observe(f,true,0); assert(x.StartDriveCorner(500,90,500,100));
    f = Frame(20); double now = 20;
    if (fault == 0) f.motors_online = false;
    if (fault == 1) f.inertial_ready = false;
    if (fault == 2) f.velocity.valid = false;
    if (fault == 3) f.position.valid = false;
    if (fault == 4) now = 240;
    if (fault == 5) f.heading_deg = 20;
    if (fault == 6) f.position.left_deg = 10000;
    x.Observe(f,fault!=7,now); x.UpdateCorner();
    assert(x.Info().state == drive::corner_state::error && x.Info().left_mV == 0 && x.Info().right_mV == 0);
    x.Observe(Frame(260),true,260); x.UpdateCorner(); assert(x.Info().state == drive::corner_state::error);
  }
  c.Observe(Frame(0),true,0); assert(c.StartDriveCorner(500,90,500,100)); c.CancelCorner();
  assert(c.Info().state == drive::corner_state::cancelled && c.Info().left_mV == 0);
  auto bad = Settings(); bad.track_width_mm = 0; c.Configure(bad); c.Observe(Frame(0),true,0);
  assert(!c.StartDriveCorner(500,90,500,100));
  drive::corner_controller stale; stale.Configure(Settings()); f = Frame(0);
  stale.Observe(f,true,0); assert(stale.StartDriveCorner(500,90,500,100));
  stale.Observe(f,true,120); stale.UpdateCorner();
  assert(stale.Info().state == drive::corner_state::error && stale.Info().left_mV == 0);
  drive::corner_controller stuck; auto cfg = Settings(); cfg.timeout_ms = 1000; stuck.Configure(cfg);
  stuck.Observe(Frame(0),true,0); assert(stuck.StartDriveCorner(500,90,500,100));
  for (int i = 1; i <= 50; ++i) { stuck.Observe(Frame(i*20),true,i*20); stuck.UpdateCorner(); }
  assert(stuck.Info().state == drive::corner_state::error && std::strcmp(stuck.Info().status,"TIMEOUT")==0);
  c.Configure(Settings()); f = Frame(0); f.stopped = false;
  c.Observe(f,true,0); assert(!c.StartDriveCorner(500,90,500,100));
  bad = Settings(); bad.radius_mm = 160; c.Configure(bad); c.Observe(Frame(0),true,0);
  assert(!c.StartDriveCorner(500,90,500,100));
  // 没有离开直线时，终点仍检查完整目标角，而非距离窗口内的中间参考角。
  drive::corner_controller end; end.Configure(Settings()); end.Observe(Frame(0),true,0);
  assert(end.StartDriveCorner(0,5,0,100));
  f=Frame(20); const double length=1.5*5*3.141592653589793/180*250;
  f.position.left_deg=f.position.right_deg=(length-4)/(69.85*3.141592653589793/(360*1.2));
  f.heading_deg=4;
  end.Observe(f,true,20); end.UpdateCorner();
  assert(end.Info().state==drive::corner_state::error && std::strcmp(end.Info().status,"END HEADING")==0);
  // 用户接受连续圆角误差 1.5°，验证边界与完成稳定时间；不是直接强制 DONE。
  for (double error : {1.49,1.51}) {
    auto accepted = Settings(); accepted.angle_tolerance_deg = 1.5;
    drive::corner_controller x; x.Configure(accepted); x.Observe(Frame(0),true,0);
    assert(x.StartDriveCorner(0,5,0,100));
    for (int tick=1;tick<=100;++tick) {
      auto value=Frame(tick*20);
      value.position.left_deg=value.position.right_deg=(length-4)/(69.85*3.141592653589793/(360*1.2));
      value.heading_deg=5-error;
      x.Observe(value,true,tick*20); x.UpdateCorner();
    }
    if(error<1.5) assert(x.Info().state==drive::corner_state::done);
    else assert(x.Info().state==drive::corner_state::error && std::strcmp(x.Info().status,"END HEADING")==0);
    assert(x.Info().left_mV==0 && x.Info().right_mV==0);
  }
  // 单独验证反馈减速：不靠解除硬保护来维持运行。
  auto governor_cfg=Settings(); governor_cfg.corner_rpm=140; governor_cfg.corner_floor_rpm=65;
  governor_cfg.corner_acceleration_rpm_s=60; governor_cfg.governor_lookahead_ms=0;
  drive::corner_controller governed; governed.Configure(governor_cfg);
  governed.Observe(Frame(0),true,0); assert(governed.StartDriveCorner(0,90,500,600));
  for(int i=1;i<=120;++i) {governed.Observe(Frame(i*20),true,i*20); governed.UpdateCorner();}
  assert(governed.Info().center_rpm>130);
  for(int i=121;i<=160;++i) {
    auto value=Frame(i*20); value.heading_deg=-6;
    governed.Observe(value,true,i*20); governed.UpdateCorner();
  }
  assert(governed.Info().state==drive::corner_state::running);
  assert(governed.Info().center_rpm>0 && governed.Info().center_rpm<=115.0001);
  auto lost=Frame(3220); lost.heading_deg=-16;
  governed.Observe(lost,true,3220); governed.UpdateCorner();
  assert(governed.Info().state==drive::corner_state::error && std::strcmp(governed.Info().status,"HEADING ERR")==0);
  assert(governed.Info().left_mV==0 && governed.Info().right_mV==0);
  // 预测配置非法时拒绝启动；故障不会留下此前路线的电压。
  for (double margin : {0.,1.1,std::numeric_limits<double>::quiet_NaN()}) {
    auto invalid = Settings(); invalid.prediction_enabled = true; invalid.prediction_margin = margin;
    drive::corner_controller x; x.Configure(invalid); x.Observe(Frame(0),true,0);
    assert(!x.StartDriveCorner(500,30,500,600));
    assert(x.Info().state == drive::corner_state::error && x.Info().left_mV == 0 && x.Info().right_mV == 0);
  }
  std::puts("Corner checks passed: continuous positive wheels, C1 heading reference, slew/limits, bidirectional loads, width error, endpoints and faults.");
}
