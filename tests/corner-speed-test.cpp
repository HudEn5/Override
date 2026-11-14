// 当前几何/增益与四组前馈快照，加上 1.4.5 入弯/弯内速度调节，不代表实车极限。

#include "drive-corner.h"
#include <cstdio>
#include <cassert>
#include <cstdlib>
#include <cmath>
#include <algorithm>
struct Run {bool pass;double time,peak,err;const char* reason;};
drive::corner_config Config(){drive::corner_config c; c.pid.kP=20;
c.pid.kI=20;
c.pid.max_correction=2000;
c.pid.max_integral=1200;
c.track_width_mm=325;
c.radius_mm=250;
c.corner_rpm=140;
c.corner_floor_rpm=65;
c.corner_acceleration_rpm_s=60;
c.entry_braking_delay_ms=200;
c.acceleration_rpm_s=180;
c.angle_tolerance_deg=1.5;
c.exit_angle_tolerance_deg=1.5;
c.exit_rate_tolerance_deg_s=4;
c.finish_min_rpm=12;
c.finish_max_rpm=30;
c.finish_kP=0.6;
c.wheel_diameter_mm=69.85;
c.motor_revs_per_wheel_rev=36.0 / 30.0;
c.left_forward.kS=372.573735;
c.left_forward.kV=17.900544;
c.left_forward.calibrated=true;
c.right_forward.kS=408.162564;
c.right_forward.kV=16.537990;
c.right_forward.calibrated=true;
c.left_reverse.kS=547.829766;
c.left_reverse.kV=16.939618;
c.left_reverse.calibrated=true;
c.right_reverse.kS=537.033135;
c.right_reverse.kV=15.870378;
c.right_reverse.calibrated=true; return c;}
Run Sim(double maximum,double curve,double angle,double width,double drag,double la,double ra,bool prediction=false,double floor=130){
 auto cfg=Config();cfg.corner_rpm=curve;cfg.prediction_enabled=prediction;
 if(prediction) cfg.corner_floor_rpm=floor;
 drive::corner_controller c;c.Configure(cfg);
 drive::straight_feedback f;f.velocity.valid=f.position.valid=f.motors_online=f.inertial_ready=f.stopped=true;
 f.velocity.sequence=1;c.Observe(f,true,0);if(!c.StartDriveCorner(500,angle,500,maximum))return {false,0,0,0,c.Info().status};
 double l=0,r=0,peak=0;const double mm=cfg.wheel_diameter_mm*3.141592653589793/(60*cfg.motor_revs_per_wheel_rev);
 for(int tick=1;tick<=1000;++tick){
  const auto old=c.Info();
  auto motor=[&](int mv,bool left){const auto model=left?(mv<0?cfg.left_reverse:cfg.left_forward):(mv<0?cfg.right_reverse:cfg.right_forward);return std::copysign(std::min(600.0,std::max(0.0,std::fabs((double)mv)-model.kS-drag)/model.kV),mv);};
  l+=(motor(old.left_mV,true)*.97-l)*la;r+=(motor(old.right_mV,false)*1.03-r)*ra;
  f.position.left_deg+=l*.12;f.position.right_deg+=r*.12;f.heading_deg+=(r-l)*mm/(325*width)*180/3.141592653589793*.02;
  f.velocity.left_rpm=l;f.velocity.right_rpm=r;f.velocity.time_ms=tick*20;f.velocity.sequence++;f.stopped=std::fabs(l)<5&&std::fabs(r)<5;
  c.Observe(f,true,tick*20);c.UpdateCorner();const auto v=c.Info();
  if(std::abs(v.left_mV)>12000||std::abs(v.right_mV)>12000||std::fabs(v.left_target_rpm)>maximum+1e-6||std::fabs(v.right_target_rpm)>maximum+1e-6)return {false,v.elapsed_ms,peak,v.error_deg,"LIMIT"};
  peak=std::max(peak,std::max(std::fabs(l),std::fabs(r)));
  if(v.state==drive::corner_state::error)return {false,v.elapsed_ms,peak,v.error_deg,v.status};
  if(v.distance_mm>450&&v.distance_mm<550+1.5*std::fabs(angle)*3.141592653589793/180*cfg.radius_mm&&v.state==drive::corner_state::running&&v.phase!=4&&((l+r)/2<5||v.left_target_rpm<=0||v.right_target_rpm<=0))return {false,v.elapsed_ms,peak,v.error_deg,"STOP"};
  if(v.state==drive::corner_state::done)return {true,v.elapsed_ms,peak,v.error_deg,"DONE"};
 }return {false,20000,peak,c.Info().error_deg,"TIMEOUT"};
}
bool Sweep(double cap,double curve,Run& worst){worst={true,0,0,0,"DONE"};for(double a:{-90.,90.})for(double w:{.9,1.,1.1})for(double d:{0.,600.})for(int response=0;response<3;++response){
 auto r=Sim(cap,curve,a,w,d,response==1?.1:.25,response==2?.1:(response==1?.35:.25));
 if(!r.pass){worst=r;return false;}worst.time=std::max(worst.time,r.time);worst.peak=std::max(worst.peak,r.peak);worst.err=std::max(std::fabs(worst.err),std::fabs(r.err));}return true;}
int main(){
 Run worst;
 const bool pass=Sweep(600,140,worst);
 if(!pass) std::printf("High-speed model failed: %s error %.3f at %.0f ms\n",worst.reason,worst.err,worst.time);
 assert(pass && worst.err<=1.5);
 std::printf("High-speed checks passed: 600 rpm cap / 140 rpm corner, 36 direction/width/friction/response cases; model peak %.2f rpm, worst time %.0f ms.\n",worst.peak,worst.time);
 unsigned int runs=0; double longest=0;
 for(double floor:{100.,130.})for(double a:{-180.,-150.,-120.,-90.,-60.,-30.,-15.,15.,30.,60.,90.,120.,150.,180.})for(double w:{.9,1.,1.1})
 for(double drag:{0.,600.})for(int response=0;response<3;++response){
  const auto r=Sim(600,140,a,w,drag,response==1?.1:.25,response==2?.1:(response==1?.35:.25),true,floor);
  if(!r.pass)std::printf("Prediction model failed: angle %.0f width %.1f drag %.0f response %d %s error %.3f at %.0f ms\n",a,w,drag,response,r.reason,r.err,r.time);
  assert(r.pass && std::fabs(r.err)<=1.5);longest=std::max(longest,r.time);++runs;
 }
 std::printf("Prediction models passed: %u signed-angle/load/response cases, worst %.0f ms.\n",runs,longest);
}
