#pragma once

#include "drive-control.h"

namespace drive {
struct sample {
  unsigned int sequence = 0;
  double time_ms = 0.0;
  double dt_ms = 0.0;
  side_sample left;
  side_sample right;
  int left_command_mV = 0;
  int right_command_mV = 0;
  bool command_valid = false;
  bool enabled = false;
  int mode = 0;  // 0 手控，1 标定电压，2 旧前馈，3 转向，4 FH，5 SP，6 直行，7 比赛自动，8 圆角。
  double target_rpm = 0.0;
  double LM_deg = 0.0;
  double LM_current_A = 0.0;
  double LF_deg = 0.0;
  double LF_current_A = 0.0;
  unsigned int diagnostic_mask = 0;  // 第 0 位 LM，第 1 位 LF；3 表示两台都有效。
};

// 独立读取电机，不写电压。串口打印由单独的回传线程消费队列。
void SamplingTask();
void SetTelemetryEnabled(bool enabled);
// 首次写底盘输出前调用；实际输出完成后必须用 PublishCommand 结束更新。
void BeginCommandUpdate();
void PublishCommand(int left_mV, int right_mV, bool enabled, bool manual_drive,
                    int mode = 0, double target_rpm = 0.0);
bool ReadLatestSample(sample& value);
bool PopSample(sample& value, unsigned int& dropped);
void PrintSample(const sample& value, unsigned int dropped);
}  // 底盘采样命名空间
