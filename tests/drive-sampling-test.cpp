#include "drive-sampling.h"
#include "robot-config.h"
#include <atomic>
#include <cassert>
#include <cstdio>
#include <functional>
#include <thread>

namespace {
struct stop_sampling {};
std::atomic<int> clock_ms(0);
std::atomic<int> wait_budget(1);
std::function<void()> read_hook;

void Sample(int count = 1) {
  wait_budget.store(count);
  try { drive::SamplingTask(); }
  catch (const stop_sampling&) {}
}

drive::sample Latest() {
  drive::sample value;
  assert(drive::ReadLatestSample(value));
  return value;
}

void Check(const drive::sample& value) {
  // 多字段必须来自同一次指令发布，不能拼接不同线程更新的半包。
  assert(value.left_command_mV == -value.right_command_mV);
  assert(value.target_rpm == value.left_command_mV);
  assert(value.mode == 2);
  assert(value.diagnostic_mask == 3);
  assert(value.LM_deg == 66 && value.LF_deg == 44);
  assert(value.LM_current_A == 2.4 && value.LF_current_A == 1.2);
}
}

namespace vex {
double timer::time(timeUnits) { return clock_ms.fetch_add(10); }
void wait(double, timeUnits) {
  if (wait_budget.fetch_sub(1) <= 1) throw stop_sampling();
  std::this_thread::yield();
}
}

namespace hardware {
drive::feedback ReadDriveFeedback() {
  if (read_hook) read_hook();
  drive::feedback value;
  for (int i = 0; i < 4; ++i) {
    value.left[i].online = value.right[i].online = true;
    value.left[i].rpm = value.right[i].rpm = 42;
    if (i < 2) {
      value.left[i].diagnostic_valid = true;
      value.left[i].position_deg = i == 0 ? 44 : 66;
      value.left[i].current_A = i == 0 ? 1.2 : 2.4;
    }
  }
  return value;
}
}

int main() {
  drive::PublishCommand(100, -100, true, true, 2, 100);
  Sample();
  assert(Latest().command_valid);
  Check(Latest());

  // 模拟主线程已经开始更新电机，但还没有发布新指令的窗口。
  drive::BeginCommandUpdate();
  Sample();
  assert(!Latest().command_valid);
  drive::PublishCommand(200, -200, true, true, 2, 200);
  Sample();
  assert(Latest().command_valid);
  Check(Latest());

  // 模拟采样读设备期间恰好发生一次完整输出更新，即使最终已经发布也不能标有效。
  read_hook = [] {
    drive::BeginCommandUpdate();
    drive::PublishCommand(300, -300, true, true, 2, 300);
  };
  Sample();
  assert(!Latest().command_valid);
  read_hook = std::function<void()>();
  Sample();
  assert(Latest().command_valid);

  drive::SetTelemetryEnabled(false);
  Sample();
  drive::sample queued;
  unsigned int dropped;
  assert(!drive::PopSample(queued, dropped));
  drive::SetTelemetryEnabled(true);
  Sample();
  assert(drive::PopSample(queued, dropped));
  Check(queued);

  // 四个宿主机线程并发发布、采样、消费队列、读取完整快照。
  drive::PublishCommand(0, 0, true, true, 2, 0);
  Sample();
  std::atomic<bool> done(false);
  std::thread sampler([&] { Sample(30000); done.store(true); });
  std::thread writer([] {
    for (int i = 1; i <= 30000; ++i) {
      drive::BeginCommandUpdate();
      drive::PublishCommand(i, -i, true, true, 2, i);
    }
  });
  std::thread consumer([&] {
    drive::sample value;
    unsigned int lost;
    while (!done.load()) {
      if (drive::PopSample(value, lost)) Check(value);
      std::this_thread::yield();
    }
  });
  while (!done.load()) { Check(Latest()); std::this_thread::yield(); }
  writer.join(); sampler.join(); consumer.join();
  Check(Latest());
  std::puts("Sampling concurrency checks passed: coherent snapshots, update windows and queue consumption.");
}
