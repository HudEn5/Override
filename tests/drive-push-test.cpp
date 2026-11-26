#include "autonomous.h"
#include <cassert>
#include <cstdio>
#include <limits>
#include <cstring>

struct fixture {
  drive::push_controller Push;
  drive::straight_feedback frame;
  double now = 0;
  fixture() {
    Push.Configure(drive::push_config());
    frame.motors_online = frame.inertial_ready = frame.position.valid = frame.velocity.valid = frame.stopped =
        true;
    Observe();
  }
  void Observe(bool allowed = true, bool sample = true) {
    if (sample) {
      frame.velocity.sequence++;
      frame.velocity.time_ms = now;
    }
    Push.Observe(frame, allowed, now);
  }
  void Tick(bool allowed = true, bool sample = true) {
    now += 20;
    Observe(allowed, sample);
    Push.Update();
  }
};
int main() {
  fixture dock;
  assert(dock.Push.StartPushFor(240, 3, true));
  for (int i = 0; i < 11; ++i) {
    dock.Tick();
    assert(dock.Push.Info().left_mV == 3000);
  }
  dock.Tick();
  assert(dock.Push.Info().state == drive::push_state::done && dock.Push.Info().left_mV == 0);

  fixture late_contact;
  assert(late_contact.Push.StartPushFor(200, 3, true));
  late_contact.frame.stopped = false;
  for (int i = 0; i < 9; ++i)
    late_contact.Tick();
  late_contact.frame.stopped = true;
  late_contact.Tick();  // 到时刚停，撤压后继续等有效的静止采样。
  for (int i = 0; i < 5; ++i)
    late_contact.Tick(true, false);
  assert(late_contact.Push.Info().settling && late_contact.Push.Info().left_mV == 0);
  late_contact.Tick();  // 新样本和上一帧相隔 120 ms，重新确认连续静止。
  assert(late_contact.Push.Info().settling);
  for (int i = 0; i < 4; ++i)
    late_contact.Tick();
  late_contact.Tick();
  assert(late_contact.Push.Info().state == drive::push_state::done);
  fixture contact;
  assert(contact.Push.StartPushFor(1000, 4));
  assert(!contact.Push.StartPushFor(200, 3));  // 同一动作不能被覆盖。
  for (int i = 0; i < 50; ++i) {
    assert(contact.Push.Info().left_mV == 4000 && contact.Push.Info().right_mV == 4000);
    assert(contact.Push.Info().state == drive::push_state::running);
    contact.frame.heading_deg += .2;  // 允许机械导向纠正姿态，不用航向环抵抗定位机构。
    contact.Tick();
  }
  assert(contact.now == 1000 && contact.Push.Info().settling);
  assert(contact.Push.Info().left_mV == 0 && contact.Push.Info().right_mV == 0);
  for (int i = 0; i < 5; ++i)
    contact.Tick();
  assert(contact.Push.Info().state == drive::push_state::done && !contact.Push.Info().settling);
  for (int i = 0; i < 20; ++i)
    contact.Tick();
  assert(contact.Push.Info().elapsed_ms == 1100 && contact.Push.Info().left_mV == 0);
  fixture zero;
  assert(zero.Push.StartPushFor(0, 4));
  assert(zero.Push.Info().state == drive::push_state::done && zero.Push.Info().left_mV == 0);
  for (double duration :
       {-1.0, 5001.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
    fixture bad;
    assert(!bad.Push.StartPushFor(duration, 4));
    assert(bad.Push.Info().state == drive::push_state::error && bad.Push.Info().left_mV == 0);
  }
  for (double voltage : {-12.1, -.0001, 0.0, .0001, 12.1, std::numeric_limits<double>::quiet_NaN()}) {
    fixture bad;
    assert(!bad.Push.StartPushFor(1000, voltage));
    assert(bad.Push.Info().right_mV == 0);
  }
  for (int fault = 0; fault < 8; ++fault) {
    fixture f;
    assert(f.Push.StartPushFor(1000, 4));
    f.Tick();
    if (fault == 0) {
      f.Push.Cancel();
      assert(f.Push.Info().state == drive::push_state::cancelled);
    } else if (fault == 1) {
      f.frame.motors_online = false;
      f.Tick();
    } else if (fault == 2) {
      f.frame.inertial_ready = false;
      f.Tick();
    } else if (fault == 3) {
      f.frame.position.valid = false;
      f.Tick();
    } else if (fault == 4) {
      f.frame.velocity.valid = false;
      f.Tick();
    } else if (fault == 5) {
      for (int i = 0; i < 6; ++i)
        f.Tick(true, false);
    } else if (fault == 6)
      f.Tick(false);
    else {
      f.now = -1;
      f.Observe();
      f.Push.Update();
    }
    assert(f.Push.Info().state == (fault == 0 ? drive::push_state::cancelled : drive::push_state::error));
    assert(f.Push.Info().left_mV == 0 && f.Push.Info().right_mV == 0);
  }
  fixture moving;
  moving.frame.stopped = false;
  moving.frame.velocity.left_rpm = 20;
  moving.Observe();
  assert(!moving.Push.StartPushFor(1000, 4) && !std::strcmp(moving.Push.Info().status, "MOVING"));
  fixture coast;
  assert(coast.Push.StartPushFor(1000, 4));
  coast.frame.stopped = false;
  coast.frame.velocity.left_rpm = coast.frame.velocity.right_rpm = 30;
  for (int i = 0; i < 125; ++i) {
    coast.Tick();
    if (coast.now >= 1000)
      assert(coast.Push.Info().left_mV == 0 && coast.Push.Info().right_mV == 0);
  }
  assert(coast.Push.Info().state == drive::push_state::error &&
         !std::strcmp(coast.Push.Info().status, "PUSH STOP TIMEOUT"));
  std::puts(
      "Push checks passed: full timed force while stalled, heading self-alignment, exact cutoff, settle, "
      "zero/invalid args, exclusive ownership, feedback loss, cancel and stop timeout.");
}
