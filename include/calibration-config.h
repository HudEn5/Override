#pragma once

namespace config {
// 仅标定环境使用，日常测试不加载这些档位或长按手势。
constexpr int CALIBRATION_VOLTAGES[3] = {2000, 3000, 4000};
constexpr int CALIBRATION_TARGETS[3] = {100, 150, 200};
constexpr int CALIBRATION_TIMEOUT_MS = 3000;
constexpr int CALIBRATION_MODE_HOLD_MS = 1000;
}  // 标定参数命名空间
