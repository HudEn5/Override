#include "drive-control.h"

namespace drive {
namespace {
constexpr double PI = 3.14159265358979323846;
bool Positive(double v) { return std::isfinite(v) && v > 0; }
}

bool odometry::Configure(double diameter, double ratio, double track) {
  value = pose(); seeded = configured = false;
  if (!Positive(diameter) || !Positive(ratio) || !Positive(track)) return false;
  mm_per_degree = (PI / 360) * diameter / ratio;
  // 720rpm允许少量瞬时超调；角速度再留50%轮距/滑移余量，仅用于拒绝明显跳变。
  max_heading_rate = 720 * (6 * diameter / ratio / track) * 1.5;
  configured = Positive(mm_per_degree) && Positive(max_heading_rate);
  return configured;
}

bool odometry::SetPose(double x, double y, double heading) {
  if (!configured || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(heading)) return false;
  // 正常跟踪时保留当前传感器基准；失效后需用下一组完整反馈重新建立基准。
  if (!value.valid) seeded = false;
  value.x_mm = x; value.y_mm = y; value.heading_deg = heading; value.needs_reset = false;
  return true;
}

void odometry::Lose() {
  value.valid = false;
  value.needs_reset = value.needs_reset || seeded;
}

void odometry::Update(double left, double right, double imu, double now, bool ready) {
  if (!configured || !ready || !std::isfinite(left) || !std::isfinite(right) ||
      !std::isfinite(imu) || !std::isfinite(now) || now < 0) { Lose(); return; }
  // 反馈丢失期间可能发生未知运动，恢复通信不能冒充定位已恢复。
  if (value.needs_reset) return;
  if (!seeded) {
    previous_left = left; previous_right = right; previous_imu = imu;
    value.time_ms = now; value.valid = seeded = true; return;
  }
  const double dt = (now - value.time_ms) / 1000;
  if (dt == 0) return;  // 重复时间不能重复积分，也不覆盖位移基准。
  if (dt < 0 || dt > .2) { Lose(); return; }
  const double dl = left - previous_left, dr = right - previous_right, dh = imu - previous_imu;
  // SDK可能缓存一帧传感器读数；留20ms刷新余量，避免高速下将正常阶跃误判为重置。
  const double sensor_dt = dt + .02;
  if (!std::isfinite(dl) || !std::isfinite(dr) || !std::isfinite(dh) ||
      std::fabs(dl) > 4320 * sensor_dt + 1 || std::fabs(dr) > 4320 * sensor_dt + 1 ||
      std::fabs(dh) > max_heading_rate * sensor_dt + 1) { Lose(); return; }
  const double radians = dh * (PI / 180), half = radians / 2;
  // 编码器给中心弧长，IMU给旋转量；sinc修正有限转角，直线处采用稳定极限。
  const double sinc = std::fabs(half) < 1e-6 ? 1 - half * half / 6 : std::sin(half) / half;
  const double distance = (dl / 2 + dr / 2) * mm_per_degree * sinc;
  const double middle = value.heading_deg * (PI / 180) + half;
  const double x = value.x_mm + distance * std::cos(middle);
  const double y = value.y_mm + distance * std::sin(middle);
  const double heading = value.heading_deg + dh;
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(heading)) { Lose(); return; }
  value.x_mm = x; value.y_mm = y; value.heading_deg = heading; value.time_ms = now;
  previous_left = left; previous_right = right; previous_imu = imu;
}
}  // 底盘控制命名空间
