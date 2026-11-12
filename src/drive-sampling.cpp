#include "drive-sampling.h"
#include "robot-config.h"
#include "control-config.h"
#include <stdio.h>

namespace drive {
namespace {
constexpr unsigned int CAPACITY = 8;
sample queue[CAPACITY];
unsigned int head = 0;
unsigned int count = 0;
unsigned int lost = 0;
vex::mutex queue_mutex;
int left_command = 0;
int right_command = 0;
bool command_enabled = false;
bool command_known = false;
bool command_updating = false;
unsigned int command_version = 0;
bool telemetry_enabled = false;
unsigned int telemetry_generation = 0;
double last_telemetry_ms = -1.0;
int command_mode = 0;
double command_target_rpm = 0.0;
sample latest;
bool latest_available = false;
}  // 匿名命名空间

void SetTelemetryEnabled(bool enabled) {
  queue_mutex.lock();
  if (enabled != telemetry_enabled) {
    telemetry_enabled = enabled;
    ++telemetry_generation;
    // 开关切换时清除旧样本，重新开启后不补发此前的数据。
    head = count = lost = 0;
    last_telemetry_ms = -1.0;
  }
  queue_mutex.unlock();
}

void BeginCommandUpdate() {
  queue_mutex.lock();
  command_updating = true;
  ++command_version;
  queue_mutex.unlock();
}

void PublishCommand(int left_mV, int right_mV, bool enabled, bool manual_drive,
                    int mode, double target_rpm) {
  queue_mutex.lock();
  if (left_command != left_mV || right_command != right_mV ||
      command_enabled != enabled || command_known != manual_drive ||
      command_mode != mode || command_target_rpm != target_rpm) ++command_version;
  left_command = left_mV;
  right_command = right_mV;
  command_enabled = enabled;
  command_known = manual_drive;
  command_mode = mode;
  command_target_rpm = target_rpm;
  command_updating = false;
  queue_mutex.unlock();
}

bool ReadLatestSample(sample& value) {
  queue_mutex.lock();
  const bool available = latest_available;
  if (available) value = latest;
  queue_mutex.unlock();
  return available;
}

void SamplingTask() {
  vex::timer timer;
  velocity_sampler left_sampler, right_sampler;
  unsigned int sequence = 0;
  double previous_time = 0.0;
  while (true) {
    sample value;
    queue_mutex.lock();
    const unsigned int version = command_version;
    const unsigned int generation = telemetry_generation;
    value.left_command_mV = left_command;
    value.right_command_mV = right_command;
    value.enabled = command_enabled;
    value.command_valid = command_known && !command_updating;
    value.mode = command_mode;
    value.target_rpm = command_target_rpm;
    queue_mutex.unlock();
    const double start = timer.time(vex::msec);
    const feedback raw = hardware::ReadDriveFeedback();
    const double end = timer.time(vex::msec);
    value.sequence = sequence++;
    // 电机依次读取，使用本次读取窗口的中点作为时间戳。
    value.time_ms = (start + end) / 2.0;
    value.dt_ms = value.sequence ? value.time_ms - previous_time : 0.0;
    previous_time = value.time_ms;
    value.left = left_sampler.Update(raw.left, value.time_ms);
    value.right = right_sampler.Update(raw.right, value.time_ms);
    if (raw.left[1].diagnostic_valid) {
      value.LM_deg = raw.left[1].position_deg;
      value.LM_current_A = raw.left[1].current_A;
      value.diagnostic_mask |= 1u;
    }
    if (raw.left[0].diagnostic_valid) {
      value.LF_deg = raw.left[0].position_deg;
      value.LF_current_A = raw.left[0].current_A;
      value.diagnostic_mask |= 2u;
    }
    queue_mutex.lock();
    // 采样过程中指令发生变化或底盘由位置测试接管时，排除指令关联数据。
    value.command_valid = value.command_valid && !command_updating && version == command_version;
    latest = value;
    latest_available = true;  // 屏幕读取不受 Y 回传开关影响。
    // 采样不受回传开关影响，只按回传间隔选取新样本进入发送队列。
    if (telemetry_enabled && generation == telemetry_generation &&
        (last_telemetry_ms < 0 || value.time_ms - last_telemetry_ms >= config::TELEMETRY_INTERVAL_MS)) {
      last_telemetry_ms = value.time_ms;
      if (count == CAPACITY) {
        head = (head + 1) % CAPACITY;
        --count;
        ++lost;
      }
      queue[(head + count) % CAPACITY] = value;
      ++count;
    }
    queue_mutex.unlock();
    const double wait_ms = 20.0 - (end - start);
    vex::wait(wait_ms > 1.0 ? wait_ms : 1.0, vex::msec);
  }
}

bool PopSample(sample& value, unsigned int& dropped) {
  queue_mutex.lock();
  const bool available = count != 0;
  if (available) {
    value = queue[head];
    head = (head + 1) % CAPACITY;
    --count;
  }
  dropped = lost;
  queue_mutex.unlock();
  return available;
}

void PrintSample(const sample& value, unsigned int dropped) {
  const unsigned int acceleration_mask = (value.left.acceleration_valid ? 1u : 0u) |
                                        (value.right.acceleration_valid ? 2u : 0u);
  // DS2 在原 27 列后追加模式和目标 rpm，接收脚本同时兼容旧 DSV。
  printf("DS2,%u,%.1f,%.1f,%u,%u,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.0f,%.0f,"
         "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%u,%d,%d,%d,%d,%d,%.1f\n",
         value.sequence, value.time_ms, value.dt_ms,
         value.left.online_mask, value.right.online_mask, acceleration_mask,
         value.left.raw_average_rpm, value.right.raw_average_rpm,
         value.left.rpm, value.right.rpm, value.left.acceleration, value.right.acceleration,
         value.left.millivolts, value.right.millivolts,
         value.left.raw_rpm[0], value.left.raw_rpm[1], value.left.raw_rpm[2], value.left.raw_rpm[3],
         value.right.raw_rpm[0], value.right.raw_rpm[1], value.right.raw_rpm[2], value.right.raw_rpm[3],
         dropped, value.command_valid, value.enabled,
         value.left_command_mV, value.right_command_mV, value.mode, value.target_rpm);
  // 与参考代码的 std::endl 一样逐行刷新，避免多行积在标准输出缓冲区。
  fflush(stdout);
  // 与 DS2 用序号和时间关联。单独成行，保持已有速度协议和旧 CSV 的兼容性。
  printf("DLM,%u,%.1f,%.2f,%.3f,%.2f,%.3f,%u\n",
         value.sequence, value.time_ms, value.LM_deg, value.LM_current_A,
         value.LF_deg, value.LF_current_A, value.diagnostic_mask);
  fflush(stdout);
}
}  // 底盘采样命名空间
