# Bartmoss 代码结构

## 实际执行流程

主循环读取硬件反馈 → 判断比赛/测试权限 → 手控生成输出 → 测试或比赛动作接管底盘 →
统一 hardware::Apply 写电机 → 发布完整状态快照。

采样线程只读取设备和发布反馈；显示与回传线程分别读取带 mutex 的快照。
运行控制器不会读取屏幕或等待串口。任何线程都不绕过主循环写底盘电压。

## 文件职责

`include` 共20个头文件，按功能合并，移除旧文件而不保留转发空壳。相关数学实现留在所属模块，避免每个结构和小配置单独占一个文件。

| 文件 | 职责 |
| --- | --- |
| control-config.h | 硬件、手控电压、底盘几何、公共控制器配置和构建模式 |
| performance-config.h | 共用运动预设、高速实验参数与自动交接参数 |
| test-config.h | 普通测试、高速整定、独立标定的全部档位与设置 |
| robot-config.h | 硬件初始化、反馈读取与电压执行 |
| driver-control-logic.h | 摇杆混控、机械臂、夹爪和气动 |
| drive-control.h | 反馈结构、差分测速、异常过滤、前馈及位姿接口 |
| drive-sampling.h | 采样、反馈快照与带锁回传队列 |
| drive-velocity.h | 单侧速度PID、双侧执行层和航向保持 |
| trapezoid-motion.h | 梯形参考、速度限制、距离闭环与停稳 |
| drive-straight.h | 公共直行控制器 |
| drive-turn.h | 相对转角、绝对航向与角速度规划跟踪 |
| drive-corner.h | 曲率与轮速预测规划、连续圆角控制器 |
| autonomous.h | 路线命令、动作调度和定时前推控制器 |
| test-control.h | 普通测试的选择、启动、取消和完成锁存 |
| high-speed-control.h | 高速整定流程与指标 |
| calibration-control.h | 独立标定的VTEST/FH/SP流程 |
| startup-mode.h | 模式权限、启动菜单、松键门控与场控交接 |
| competition-control.h | 带mutex的比赛回调快照 |
| runtime-info.h | 主循环向显示与回传发布的完整状态快照 |
| vex.h | SDK入口 |

`src/main.cpp` 统一调度并写电机，`src/route.cpp` 只编排路线。硬件、采样、测试调度、比赛回调、显示与遥测各有对应实现文件。

`src/odometry.cpp` 只有一个实现文件，不另拆位姿头文件、配置文件或线程。主循环用左右平均编码器增量与连续IMU转角更新位置；显示和遥测通过已有mutex读取整包 `runtime_info::pose`，不会读写估计器实例。校准等待、有效和失效锁存分别显示WAIT/OK/RESET，运行中失效后必须在已知位置显式重设。原距离/转向控制器仍独立；MoveToPoint读取传入的位姿快照编排原转向和直行，并检查全局终点与定位有效性。Swing复用turn_controller，固定侧制动，单侧几何换算与原地转向隔离。

## 公共动作接口

路线在 autonomous_program::RunRoutine 中使用 RunRoute 顺序编排：

```cpp
using namespace route;
RunRoute({DriveStraight(500), Wait(200), Turn(90), DriveStraight(500)});
```

每次只启动一个动作，后续周期检查完成后推进；Wait由主循环时钟计时，零毫秒等待可直接推进。
完成锁存，启动失败/取消/反馈故障/超时停止全路线，场控禁用立即取消；不创建阻塞路线线程。
底层 StartDriveStraight/StartTurnRelative/StartTurnToHeading/StartDriveCorner 仍保留供调度器使用。
自动与比赛实例均采用 performance-config 中的共用预设；菜单A请求一次自动，X进入测试，选择键不直接写电机。
场控连接时SessionPermissions绕过本地选择，直接使用比赛权限；标定构建保留独立许可。
完整示例及单位见 [路线指南](ROUTE_GUIDE.md)。

## 维护与验证

scripts/build.ps1 构建五种输出：Default/Test/Competition/Calibration/HighSpeed。
scripts/monitor-drive-websocket.mjs 只接收并保存数据；configure-device.ps1 和 sync-github.ps1 管理本机配置与同步。
scripts/test-host.sh 运行 tests 下的控制器、调度和并发测试；mocks 是宿主机 SDK 替身。
.vscode 是 VEX 工程、代码补全和任务设置；makefile 与 vex/ 是构建规则。

README 面向当前操作；LAB_NOTES 保留真实实验历史，历史参数不代表当前配置。
当前手臂使用纯电压控制。


## 本地产物与历史

build根目录只放活动默认产物及构建目录；日志自动进入logs，采集仍在data。旧试验放archive，备份保留backups，阶段实验文档在docs/history。历史记录中的旧路径通过本地logs/workspace-cleanup.json查询迁移位置；可重建测试文件的删除记录在logs/deleted-generated-files.json。


## 接口与编码约定（2.1.0-api1）

- 内部类型用snake_case，方法用PascalCase，变量/字段用snake_case；LF/ArmL/Controller等参考工程硬件名保留。数值字段带mm、deg、rpm、mV、ms等单位；PushFor对外仍用V。通用数学规划器以秒为时间单位，调用边界显式换算。
- .clang-format固定2空格、110列及统一排版；不按类继续拆头文件。
- drive::motion_state统一基础动作结果，各控制器保留state别名；route_result区分空闲/执行/完成/取消/故障。显示文字不参与路线结果判定，只有遥测出口把枚举转数字，协议编号保持原样。
- route::command使用distance_mm、angle_deg、x_mm、y_mm、duration_ms、volts、maximum_rpm、turn_rpm、fixed_side等明确字段，路线工厂函数签名不变。禁止用first/second和浮点数代替枚举传参。
- CancelRoutine立即发布零输出；各Cancel子动作同步更新快照。ResetControl只清PID/输出状态，ConfigurePID额外丢弃旧反馈序号和时钟，首次Observe建立新基准。
- PointControl独立配置5mm终点容差和200ms定位新鲜度。点位动作把部分总误差预算分给朝向、部分给沿程，根据距离收紧转向和直行容差，同时降低严格转向末端的最低补角转速；普通Turn/DriveStraight参数不变。全局超差仍报错，不放宽终点判定。
- 直行的max_heading_speed_fraction限制目标转速纠偏，heading.max_voltage_fraction限制电压纠偏；两者单独配置，默认均0.25。
