# Bartmoss 代码结构 · v1.6.0

## 实际执行流程

主循环读取硬件反馈 → 判断比赛/测试权限 → 手控生成输出 → 测试或比赛动作接管底盘 →
统一 hardware::Apply 写电机 → 发布完整状态快照。

采样线程只读取设备和发布反馈；显示与回传线程分别读取带 mutex 的快照。
运行控制器不会读取屏幕或等待串口。任何线程都不绕过主循环写底盘电压。

## 文件职责

| 文件 | 职责 |
| --- | --- |
| src/main.cpp | 初始化、权限判断、反馈组装、统一输出与状态发布 |
| include/control-config.h | 硬件、手控电压、三种公共控制器和环境宏 |
| include/test-config.h | 日常直行/转向/圆角的距离、角度、最高速度 |
| include/calibration-config.h | 独立标定档位、长按时间和运行超时 |
| include/robot-config.h、src/robot-config.cpp | 硬件初始化、手柄/传感器读取和电压执行 |
| include/driver-control-logic.h | 底盘摇杆混控、双臂电压、夹爪、气动 |
| include/test-control.h、src/test-control.cpp | 统一三项测试，选择/开始/取消及完成锁存 |
| include/calibration-control.h | 仅 CAL 使用的 VTEST/FH/SP 三种诊断 |
| include/program-mode.h | TEST/CAL/COMP 的手控、诊断、自动与回传许可 |
| include/competition-control.h、src/competition-control.cpp | 用 mutex 发布比赛阶段回调 |
| include/autonomous.h、src/autonomous.cpp | 比赛动作调度、互斥执行及生命周期 |
| src/route.cpp | 正式比赛路线，目前 EMPTY |
| include/drive-control.h | 反馈结构、编码器差分测速、异常过滤和电机前馈 |
| include/drive-sampling.h、src/drive-sampling.cpp | 约 50 Hz 采样、最新反馈、带锁回传队列 |
| include/velocity-control.h | 单侧速度 PID 数学计算及抗饱和 |
| include/drive-velocity.h | 左右/正反向前馈与速度 PID 的公共执行层 |
| include/heading-control.h | 直行航向保持 |
| include/trapezoid-profile.h | 梯形位置/速度参考与速度变化限制 |
| include/trapezoid-motion.h | 距离闭环、梯形执行及终点停稳 |
| include/drive-straight.h | 距离/最高速度的公共直行控制器 |
| include/drive-turn.h | 相对转向和绝对朝向的公共原地转向控制器 |
| include/drive-corner.h | 连续直行/圆角/直行的公共控制器 |
| include/corner-profile.h | 基于曲率变化和左右轮能力的提前速度规划 |
| include/runtime-info.h、src/runtime-info.cpp | 主循环/显示/回传间的整包状态快照及 mutex |
| src/display.cpp | 主控和遥控器显示，不调用动作控制器 |
| src/telemetry.cpp | 唯一串口打印线程及原有数据协议 |
| include/vex.h | VEX SDK 头文件入口 |

## 公共动作接口

比赛路线在 autonomous_program::RunRoutine 中逐步调用：

```cpp
StartDriveStraight(500, 100);                 // 距离 mm，最高输出轴 rpm
StartTurnRelative(90, 100);                  // 相对角，逆时针为正
StartTurnToHeading(0, 100);                  // 最短方向到绝对朝向
StartDriveCorner(500, 30, 500, 600);          // 前直线、转角、后直线、单侧最高 rpm
```

每次只启动一个动作，在后续主循环等对应状态 done 后再进入下一步，不能把上述示例连续调用当作路线。
日常测试使用同样的底层控制器；测试按键逻辑不进入比赛路线。

## 维护与验证

scripts/build.ps1 构建四种输出：Default/Test/Competition/Calibration。
scripts/monitor-drive-websocket.mjs 只接收并保存数据；configure-device.ps1 和 sync-github.ps1 管理本机配置与同步。
scripts/test-host.sh 运行 tests 下的控制器、调度和并发测试；mocks 是宿主机 SDK 替身。
.vscode 是 VEX 工程、代码补全和任务设置；makefile 与 vex/ 是构建规则。

README 面向当前操作；LAB_NOTES 保留真实实验历史，历史参数不代表当前配置。
当前手臂使用纯电压控制。
