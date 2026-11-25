# Bartmoss 代码结构

## 实际执行流程

主循环读取硬件反馈 → 判断比赛/测试权限 → 手控生成输出 → 测试或比赛动作接管底盘 →
统一 hardware::Apply 写电机 → 发布完整状态快照。

起步反馈诊断共用drive::StartStatus，先判断设备和反馈有效性，再判断MOVING。本地自动菜单另由AutomaticStartStatus检查位姿失效锁存和200ms新鲜度；auto_status经原mutex快照传给显示，不增加线程或打印频率。X分项测试仍不依赖全局位姿。

采样线程只读取设备和发布反馈；显示与回传线程分别读取带 mutex 的快照。
运行控制器不会读取屏幕或等待串口。任何线程都不绕过主循环写底盘电压。

## 文件职责

`include` 共23个头文件，按功能合并，移除旧文件而不保留转发空壳。相关数学实现留在所属模块，避免每个结构和小配置单独占一个文件；独立控制器按职责分开，不以文件总数限制合理拆分。

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
| drive-rolling.h | 多段连续移动转向、独立rolling_info快照 |
| mechanism-control.h | 三通道机构适配器、独立超时、输出与取消仲裁 |
| mechanisms.h | 同事的抬升PID、定时夹爪、目标预设与直接参数动作适配器 |
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
RunRoute({DriveStraight(500,540,5000), Wait(200,1000), Turn(90,540,3000), DriveStraight(500,540,5000)});
```

每次只启动一个底盘动作，后续周期检查完成后推进；机构可独立并行。Wait由主循环时钟计时，零毫秒等待可直接推进。
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
手控手臂保持纯电压控制；自动机构采用双电机平均角度PID，主循环读反馈并更新，不另开写电机线程。首次有效反馈仅建立一次相对零点，反馈失效后不自动重建零点；夹爪DONE仅表示通电时间结束。

直接LiftTo/ClawOpen/ClawClose转换成有类型的机构请求，仍由同一个scheduler处理独立预算、通道互斥、错误和取消；不在autonomous.cpp另造一套机构后台任务。主循环在Autonomous.Update前更新机构反馈/权限，首次闭爪不沿用菜单禁用状态。运行快照同时带抬升和夹爪数据，屏幕线程从原mutex快照显示，未增加头文件/线程或无线打印。


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


structure1将rolling_controller独立于旧corner_controller文件和状态快照。autonomous_program只用info.motion选择控制器：corner=3、rolling=5；更新、终态检查和电压发布均按该枚举，取消旧圆角不影响移动转向，CancelRolling仅取消移动转向。所有启动接口检查Busy，周期前后检查运行控制器与动作编号一致；不一致以CONTROL OWNER结束并撤输出。全局结束、禁用或取消通过CancelAll撤销所有控制器，主循环仍是唯一电机写入者。

DriveTurnDrive(stop)仍编排原直行/原地转向/直行，rolling模式复用连续速度执行层，相邻组合命令一次规划并执行，保持速度环状态。旧corner_info/DCR的remaining_mm为全程余量；新rolling_info/DRL使用segment_remaining_mm，并记录completed_turns/total_turns，接收脚本分别保存corner.csv与rolling.csv。DRA保留原0～4编号并增加rolling=5，不增加线程或打印频率。


chain1移除专用方形接口与控制分支。RunRoute启动rolling命令时向后扫描全部相邻rolling组合，固定容量16弯，启动前验证完整连续块并复制到控制器；超过容量或任一后续参数非法都在起步前结束。中间直段长度为上一项after加下一项before，before=0表示直接接续；每弯有独立角度、速度上限和入角速度，目标朝向相对连续块起点累加，不逐弯按实测角重置。

规划先从末端向前传播减速/响应距离预算，再向前限制短段加速能力；不重置速度环、不在中间发布零输出或等待停稳。末端保留距离/角度/低速度验收；最后后直线必须大于距离容差，过短报ROLL END DIST。超时按当前直线与随后转向计算，不把多个正常动作总耗时误作单动作超时。连续块完成后DRA步号一次推进对应命令数，DRL以completed_turns/total_turns显示内部进度。Turn/Wait/其他动作是明确停车边界，下一连续块另从当前实际朝向起步。


timeout1最初为九种底盘工厂引入必填timeout_ms，当前扩展为十五种路线工厂，末尾预算全部必填。RunRoute在命令启动时记录action_started_at，MoveToPoint与stop内部阶段交接不重置；超时通过FinishRoutine统一撤输出。rolling_leg携带每项预算，执行器依据共享直线的after/before边界切换timed_leg，仅切换时钟、不重置PID或停车；全部后续预算在起步前验证，最后一项包含停车验收。已有内部反馈与控制器超时保护保留，显式命令预算为额外上限。


point3只在MoveToPoint的facing阶段结束时复核最新目标方位；允许max_heading_realignments（默认2）次原地重新对准，再按新距离进入原固定航向直行。计数仅新Point命令启动时归零，action_started_at不重置，超限POINT ALIGN；驱动阶段不返回转向。PointBearingError/PointTurnTolerance共用原坐标和角度预算计算，5mm终态标准保持。


通用机构调度在mechanism-control.h，真实抬升/夹爪算法及适配器在mechanisms.h；不增加线程或实现文件。autonomous_program每周期更新机构调度器和当前底盘动作；mechanism::info经原runtime_info mutex整包复制。main在hardware::Apply前合并机构输出，自动退出时只交接停止方式，新手控指令优先；气动状态同步回driver_control，防止模式切换翻转。原DRA动作编号保持，新增机构命令不增加底盘motion枚举或遥测频率。机构Start/Update都推进同一单调时钟，Reset拒绝非法时间；启动请求按值复制，避免清理旧动作时覆盖引用到旧快照的参数。
