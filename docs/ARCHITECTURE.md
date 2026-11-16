# Bartmoss 代码结构

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
| src/route.cpp | 调试与比赛共用的顺序动作表 |
| include/route-commands.h | 直行、相对/绝对转向、圆角、Wait命令与默认速度 |
| include/startup-mode.h | 无场控A自动/B手动/X测试选择、松键门控、返回菜单与场控权限交接 |
| include/motion-presets.h | 路线与高速测试共用的H3/高速圆角参数副本 |
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
| include/drive-push.h | 固定电压定时前推、撤电压后停稳及故障保护 |
| include/corner-profile.h | 基于曲率变化和左右轮能力的提前速度规划 |
| include/runtime-info.h、src/runtime-info.cpp | 主循环/显示/回传间的整包状态快照及 mutex |
| src/display.cpp | 主控和遥控器显示，不调用动作控制器 |
| src/telemetry.cpp | 唯一串口打印线程及原有数据协议 |
| include/vex.h | VEX SDK 头文件入口 |

## 公共动作接口

路线在 autonomous_program::RunRoutine 中使用 RunRoute 顺序编排：

```cpp
using namespace route;
RunRoute({DriveStraight(500), Wait(200), Turn(90), DriveStraight(500)});
```

每次只启动一个动作，后续周期检查完成后推进；Wait由主循环时钟计时，零毫秒等待可直接推进。
完成锁存，启动失败/取消/反馈故障/超时停止全路线，场控禁用立即取消；不创建阻塞路线线程。
底层 StartDriveStraight/StartTurnRelative/StartTurnToHeading/StartDriveCorner 仍保留供调度器使用。
自动与比赛实例均采用 motion-presets；菜单A请求一次自动，X进入测试，选择键不直接写电机。
场控连接时SessionPermissions绕过本地选择，直接使用比赛权限；标定构建保留独立许可。
完整示例及单位见 [路线指南](ROUTE_GUIDE.md)。

## 维护与验证

scripts/build.ps1 构建五种输出：Default/Test/Competition/Calibration/HighSpeed。
scripts/monitor-drive-websocket.mjs 只接收并保存数据；configure-device.ps1 和 sync-github.ps1 管理本机配置与同步。
scripts/test-host.sh 运行 tests 下的控制器、调度和并发测试；mocks 是宿主机 SDK 替身。
.vscode 是 VEX 工程、代码补全和任务设置；makefile 与 vex/ 是构建规则。

README 面向当前操作；LAB_NOTES 保留真实实验历史，历史参数不代表当前配置。
当前手臂使用纯电压控制。
