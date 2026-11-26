#include "control-config.h"
#include "robot-config.h"
#include "runtime-info.h"

namespace {
runtime_info latest;
vex::mutex snapshot_mutex;
}
void PublishRuntimeInfo(const runtime_info& value) {
  snapshot_mutex.lock();
  latest = value;
  snapshot_mutex.unlock();
}
runtime_info ReadRuntimeInfo() {
  snapshot_mutex.lock();
  const runtime_info value = latest;
  snapshot_mutex.unlock();
  return value;
}

runtime::teach_packet ReadTeachPacket() {
  snapshot_mutex.lock();
  const auto packet = latest.teaching.packet;
  snapshot_mutex.unlock();
  return packet;
}
