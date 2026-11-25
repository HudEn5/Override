#pragma once

#include "drive-control.h"
#include "driver-control-logic.h"

// 非阻塞机构适配接口：由main调用，只计算输出，不创建线程或写电机。
namespace mechanism {
enum class channel { arm, claw, piston };
enum class operation { preset, lift_angle, claw_open, claw_close };
struct request {
  operation kind = operation::preset;
  int preset = 0;
  double target = 0;  // 抬升为电机度，夹爪为通电毫秒。
  double volts = 0;   // 仅直接夹爪动作使用，正数表示电压大小。
};
struct output {
  int motor_mV = 0;
  bool piston = false;
  manual::stopping stop = manual::stopping::configured;
};
struct status {
  drive::motion_state state = drive::motion_state::idle;
  const char* reason = "READY";  // 仅使用不可变静态字符串，不能指向可变/临时缓冲区。
  output command;
};
class controller {
 public:
  virtual ~controller() {}
  // preset由机构作者定义为具名编号；不能将未知零点的角度当成默认动作。
  virtual bool Start(int preset, double timeout_ms, double now_ms) = 0;
  virtual bool Start(const request& action, double timeout_ms, double now_ms) {
    return action.kind == operation::preset && Start(action.preset, timeout_ms, now_ms);
  }
  virtual void Update(double now_ms) = 0;
  virtual status Info() const = 0;
  virtual void Cancel() = 0;
};
struct action_info {
  bool claimed = false;
  int preset = 0;
  request action;
  double started_ms = 0, timeout_ms = 0;
  status feedback;
};
struct info {
  action_info channels[3];
  const char* fault = nullptr;
};

// 固定三个通道，无堆分配；适配器由调用方持有，生命周期须长于整个自动程序。
class scheduler {
 public:
  static bool Valid(channel value) {
    const int i = static_cast<int>(value);
    return i >= 0 && i < 3;
  }
  bool Bind(channel value, controller& adapter) {
    if (!Valid(value) || Busy())
      return false;
    // 禁止同一实例被两个通道同时调度。
    for (unsigned i = 0; i < 3; ++i)
      if (adapters[i] == &adapter && i != static_cast<unsigned>(value))
        return false;
    if (data.channels[static_cast<int>(value)].claimed)
      return false;
    adapters[static_cast<int>(value)] = &adapter;
    return true;
  }
  const info& Info() const {
    return data;
  }
  bool Available(channel value) const {
    return Valid(value) && adapters[static_cast<int>(value)] &&
           data.channels[static_cast<int>(value)].feedback.state != drive::motion_state::running;
  }
  bool Start(channel value, int preset, double timeout_ms, double now) {
    request action;
    action.preset = preset;
    return Start(value, action, timeout_ms, now);
  }
  bool Start(channel value, request requested, double timeout_ms, double now) {
    if (data.fault)
      return false;
    if (!Valid(value) || !ValidRequest(value, requested) || !std::isfinite(timeout_ms) || timeout_ms <= 0 ||
        !std::isfinite(now) || now < 0)
      return Fail("MECH CONFIG");
    if (now < previous_time)
      return Fail("MECH CLOCK");
    const unsigned i = static_cast<unsigned>(value);
    if (!adapters[i])
      return Fail("MECH NOT BOUND");
    if (!Available(value))
      return Fail("MECH BUSY");
    previous_time = now;
    auto& action = data.channels[i];
    if (action.claimed)
      adapters[i]->Cancel();
    action = action_info();
    action.claimed = true;
    action.preset = requested.preset;
    action.action = requested;
    action.started_ms = now;
    action.timeout_ms = timeout_ms;
    const bool accepted = adapters[i]->Start(requested, timeout_ms, now);
    if (!accepted && adapters[i]->Info().state != drive::motion_state::error)
      return Fail("MECH START");
    if (!Read(i))
      return false;
    if (!accepted)
      return Fail("MECH START");
    return true;
  }
  bool Update(double now) {
    if (data.fault)
      return false;
    if (!std::isfinite(now) || now < previous_time)
      return Fail("MECH CLOCK");
    previous_time = now;
    for (unsigned i = 0; i < 3; ++i) {
      auto& action = data.channels[i];
      if (!action.claimed)
        continue;
      const bool was_done = action.feedback.state == drive::motion_state::done;
      if (now - action.started_ms > action.timeout_ms && !was_done)
        return Fail("MECH TIMEOUT");
      adapters[i]->Update(now);
      if (!Read(i))
        return false;
      if (was_done && action.feedback.state != drive::motion_state::done)
        return Fail("MECH STATE");
      if (action.feedback.state == drive::motion_state::running &&
          now - action.started_ms >= action.timeout_ms)
        return Fail("MECH TIMEOUT");
    }
    return true;
  }
  bool Busy() const {
    for (const auto& action : data.channels)
      if (action.claimed && action.feedback.state == drive::motion_state::running)
        return true;
    return false;
  }
  bool Done(channel value) const {
    return Valid(value) && data.channels[static_cast<int>(value)].claimed &&
           data.channels[static_cast<int>(value)].feedback.state == drive::motion_state::done;
  }
  bool Claimed(channel value) const {
    return Valid(value) && data.channels[static_cast<int>(value)].claimed;
  }
  void Cancel(bool preserve_completed_stop = false) {
    for (unsigned i = 0; i < 3; ++i) {
      auto& action = data.channels[i];
      if (!action.claimed)
        continue;
      const auto completed_stop = action.feedback.command.stop;
      const bool completed = action.feedback.state == drive::motion_state::done;
      adapters[i]->Cancel();
      const auto stopped = adapters[i]->Info();
      // 取消撤掉电压，保留气动最后状态；停止方式由适配器明确给出。
      if (ValidStop(stopped.command.stop))
        action.feedback.command.stop = stopped.command.stop;
      if (preserve_completed_stop && completed)
        action.feedback.command.stop = completed_stop;
      action.feedback.command.motor_mV = 0;
      if (action.feedback.state == drive::motion_state::running) {
        action.feedback.state = drive::motion_state::cancelled;
        action.feedback.reason = "CANCEL";
      }
    }
  }
  void Reset(double now) {
    Cancel();
    data = info();
    previous_time = 0;
    if (!std::isfinite(now) || now < 0)
      Fail("MECH CLOCK");
    else
      previous_time = now;
  }
  void Apply(manual::output& target, bool stopping_only = false) const {
    if (data.channels[0].claimed && (!stopping_only || target.arm_mV == 0)) {
      target.arm_enabled = true;
      target.arm_mV = stopping_only ? 0 : data.channels[0].feedback.command.motor_mV;
      target.arm_stop = data.channels[0].feedback.command.stop;
    }
    if (data.channels[1].claimed && (!stopping_only || target.claw_mV == 0)) {
      target.claw_mV = stopping_only ? 0 : data.channels[1].feedback.command.motor_mV;
      target.claw_stop = data.channels[1].feedback.command.stop;
    }
    if (data.channels[2].claimed && !stopping_only)
      target.toggle_piston = data.channels[2].feedback.command.piston;
  }

 private:
  static bool ValidRequest(channel value, const request& action) {
    switch (action.kind) {
      case operation::preset:
        return action.preset >= 0;
      case operation::lift_angle:
        return value == channel::arm && std::isfinite(action.target);
      case operation::claw_open:
      case operation::claw_close:
        return value == channel::claw && std::isfinite(action.target) && action.target >= 0 &&
               std::isfinite(action.volts) && action.volts > 0 && action.volts <= 12;
    }
    return false;
  }
  static bool ValidStop(manual::stopping stop) {
    const int i = static_cast<int>(stop);
    return i >= 0 && i <= 3;
  }
  bool Read(unsigned i) {
    auto& action = data.channels[i];
    action.feedback = adapters[i]->Info();
    const auto& s = action.feedback;
    if (!s.reason || !ValidStop(s.command.stop) || s.command.motor_mV < -12000 ||
        s.command.motor_mV > 12000 || (i == 2 && s.command.motor_mV != 0))
      return Fail("MECH OUTPUT");
    if (s.state == drive::motion_state::error || s.state == drive::motion_state::cancelled)
      return Fail(s.reason);
    if (s.state != drive::motion_state::running && s.state != drive::motion_state::done)
      return Fail("MECH STATE");
    return true;
  }
  bool Fail(const char* reason) {
    data.fault = reason ? reason : "MECH ERROR";
    Cancel();
    return false;
  }
  controller* adapters[3] = {};
  info data;
  double previous_time = 0;
};
}  // 机构调度命名空间
