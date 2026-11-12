# Bartmoss v1.4.0-route-test · 2026–2027 控制程序

台式机与实验室笔记本通过 GitHub `main` 同步，操作见 [DEVICE_SYNC.md](DEVICE_SYNC.md)。
另一台电脑的 Codex 可从 [LAPTOP_PROMPT.md](LAPTOP_PROMPT.md) 接手；实测记录写入 [LAB_NOTES.md](LAB_NOTES.md)。

VEX V5 八电机差速底盘工程，支持手动控制、Y 键直线加原地右转测试和自动直线梯形速度规划，手臂采用直接电压控制。
沿用参考工程 Tungsten 的设备命名。当前版本用于实机调试，尚未完成整机验证。

## 分支交接：梯形速度及测试代码

此版本单独保存在 `feature/trapezoid-speed-tests`，包含梯形速度规划、Y 键直线与转弯测试、
电脑端回归及使用说明。日常同步仍使用 `main`；只拉取 `main` 不会合入本分支。
本分支基于原有 `92d212e`，没有合并后来 `main` 上的改动。

队友需要查看本版时，先保存并提交自己分支的工作，再在本地执行：

~~~sh
git fetch origin
git switch --track origin/feature/trapezoid-speed-tests
~~~

如果本地已经有这个分支，改用 `git switch feature/trapezoid-speed-tests`。
各自在自己的分支开发，确认后通过 Pull Request 合并；若双方改动同一段代码，仍需核对并解决冲突。

## 手柄操控

| 输入 | 动作 |
| --- | --- |
| 左摇杆上下 Axis3 | 前进 / 后退 |
| 右摇杆左右 Axis1 | 转向 |
| 按住 R1 | 两台手臂电机立即接收上升电压，默认 12 V |
| 按住 R2 | 两台手臂电机立即接收下降电压，默认反向 6 V |
| 松开 R1/R2 或同时按下 | 停止手臂，使用 VEX 内置 hold 保持松手时的位置 |
| 按住 L1 / L2 | 夹爪夹紧 / 松开，默认 6 V |
| X 按下沿 | 切换 H 口气动状态 |

底盘按“左 = 前后 + 转向，右 = 前后 − 转向”混控。
摇杆死区 5%，最大输出 ±12 V，超限时左右同比例缩小。
夹爪松手使用 hold，底盘停止使用 coast。
X 按住不会反复切换，启动或重连时已按住 X 不计为新一次按下。

## 手柄底盘路线测试

配置有效、手柄在线且未连接场控时，**按一次 Y 启动整条路线，松开 Y 后继续运行，再按一次 Y 立即中止**。
启动前保持两根底盘摇杆在 5% 死区内，松开手臂、夹爪、气动键和 B。
上电或恢复连接时已按住 Y 不会启动，需先松开再按；方向键不再作为测试入口。

| 顺序 | 动作 | 规划参数 |
| --- | --- | --- |
| 1 | 直线前进，左右各 +3 圈电机输出轴 | 最高 120 RPM，加减速度均为 120 RPM/s；参考曲线为加速 1 s、匀速 0.5 s、减速 1 s |
| 2 | 直线段满足停稳判据后，保持刹停等待 | 0.6 s |
| 3 | 原地右转，左侧 +0.75 圈、右侧 −0.75 圈 | 最高 60 RPM，加减速度均为 60 RPM/s；当前距离为三角形参考曲线，约 1.732 s |
| 4 | 转弯段完成后自动停车 | 不自动重复路线 |

参数位于 [control-config.h](include/control-config.h)。直线和转弯复用同一梯形规划及八电机反馈跟随器；
实际完成时间还取决于编码器位置和速度达到停止判据，表中时长不是实测结果。
测试单位是**电机输出轴圈数**，不等于车轮圈数或厘米；轮径为 0 也可运行这条测试路线。
当前未标定轮径、轮距，也没有 IMU 闭环；路线按左右输出轴目标转数控制，不保证精确车身转角。

运行或等待期间，再按 Y、按 B、推动任一底盘摇杆超过死区、按任一手臂/夹爪/气动键、
手柄断连或接入场控，都会中止并立即刹停。测试期间手臂和夹爪停止，气动保持原状态。
完成、取消或故障后不会自行重启；重新满足启动条件并再次按 Y 才启动新一轮。
停车后摇杆回中时持续制动，主动推动摇杆才恢复原手动底盘控制。
普通摇杆仍使用直接电压混控，不会启动梯形路线。

用于肉眼观察时，先编译并把 Bartmoss 下载到主控槽位 1，在主控运行该程序后拔掉 USB，
保持手柄与主控无线连接，清出直行和转弯空间，再按 Y 观察“直线—停顿—原地右转—停车”。
当前不做轨迹缓存或数据采集，拔 USB 后不会保存完整测试曲线。
首次可先架空底盘核对八台电机方向，再落地观察；本轮实际下载和实车结果见 [LAB_NOTES.md](LAB_NOTES.md)。

主控和手柄显示 `STRAIGHT` / `PAUSE` / `TURN` / `DONE` 路线阶段；
主控还显示 `ACCEL` / `CRUISE` / `DECEL` / `SETTLE` 速度阶段、参考值、左右实际 RPM 和目标误差。
右转时左侧位置与 RPM 为正，右侧为负；完成或取消后保留最后一段状态供查看。

## 手臂纯电压模式

R1/R2 直接决定输出电压；运动期间不使用自定义 PID、角度目标、滤波或电压变化率限制。
两个电机接收相同逻辑电压，ArmR 的反转配置补偿安装方向。
上升和下降电压可以分别调整：

~~~cpp
constexpr int ARM_UP_VOLTAGE = 12000;   // 上升电压，单位 mV
constexpr int ARM_DOWN_VOLTAGE = 6000;  // 下降电压，填写正数
~~~

参数位于 include/control-config.h，允许范围 1～12000 mV。
ARM_UP_DIRECTION 控制统一动作方向，默认 1。
实际速度取决于电压、负载和传动，直接电压输出不保证固定转速。

编码器仅用于显示启动后的相对位移，不要求启动时手臂放在固定位置。
本版本没有 0° 下限、90° 上限、目标领先限制、编码器位置差检查或软件堵转计数；
接近机械行程终点时需人工松开按键。
两侧手臂电机必须都在线才接受升降指令，任一侧掉线时两侧停止。
停止时仅记录一次内置 hold 位置，避免每个循环改写保持点。
内置 hold 负责静止保持；程序没有运动期间的自定义位置控制器。

## 硬件配置

| 设备 | 端口 | 内部齿轮比 | 反转 |
| --- | --- | --- | --- |
| LF | 12 | 6:1 | true |
| LM | 1 | 6:1 | false |
| LBD | 11 | 6:1 | true |
| LBU | 13 | 6:1 | false |
| RF | 6 | 6:1 | false |
| RM | 5 | 6:1 | true |
| RBD | 9 | 6:1 | false |
| RBU | 10 | 6:1 | true |
| ArmL | 3 | 36:1 | false |
| ArmR | 2 | 36:1 | true |
| Claw | 15 | 18:1 | true |
| TogglePiston | 三线口 H | — | — |

智能端口按主控编号 1～21 填写，创建设备前校验范围及重复端口。
设备和按键均配置在 include/control-config.h；惯性传感器端口为 0，不创建设备。

## 运行与编译

在 VS Code 中打开仓库根目录，使用 VEX 扩展编译和下载。
程序名 Bartmoss，V5 平台，槽位 1，SDK 为 V5_20240802_15_00_00。

~~~powershell
.\scripts\build.ps1
~~~

脚本支持 -SdkPath / -ToolchainPath 指定 SDK 和工具链，输出 build/Bartmoss.bin。
控制循环每轮等待 10 ms；实际周期还包含设备通信耗时。
主控和遥控器显示及串口打印在独立线程执行，控制线程仅采集输入、计算输出和发布快照。

未连接场控时，配置有效且手柄连接即可调试。
连接场控后，手动时段接受手柄输入，自动时段执行下述可配置的直线任务。
禁用时停止运动；手柄断连时停止手动输出，气动保持状态。
场控禁用期间的实际电机行为由 VEX 系统接管。

## 自动直线梯形速度规划

已将迁移包四电机工程的梯形算法接入当前八电机底盘。长距离使用加速、匀速、减速三段，
短距离自动降为三角形，支持负距离后退。主循环逐步执行任务，不阻塞手臂停机和显示更新。
该功能用于按距离自动走直线；手动摇杆仍使用原有电压混控。

先在 [control-config.h](include/control-config.h) 填入以下配置：

| 配置 | 当前值 | 含义 |
| --- | --- | --- |
| `AUTONOMOUS_TRAPEZOID_ENABLED` | `false` | 改为 `true` 后自动阶段才启动任务 |
| `AUTONOMOUS_DISTANCE_CM` | `0` | 目标距离，正数前进、负数后退 |
| `DRIVE_WHEEL_DIAMETER_CM` | `0` | 实测轮径，单位 cm；0 会拒绝启动 |
| `DRIVE_MOTOR_REVS_PER_WHEEL_REV` | `1` | 外部电机转数 / 车轮转数，不重复计算内部齿轮箱 |
| `DRIVE_MAX_SPEED_CM_PER_SEC` | `30` | 最高线速度，单位 cm/s |
| `DRIVE_ACCELERATION_CM_PER_SEC2` | `30` | 加速度，单位 cm/s² |
| `DRIVE_DECELERATION_CM_PER_SEC2` | `30` | 减速度，单位 cm/s² |

当前没有实测轮径和指定路线，因此保留未启用配置。填写参数并重新编译、下载后，
接入场控或比赛开关，在每次已启用的自动阶段进入时启动一次任务。
任务完成或失败后保持刹停，直到切换时段；不会在同一自动阶段重复启动。
未连接场控时保持手动控制，可按一次 Y 进入上述测试路线，不会因上电自动行驶。

每侧四台电机接收相同 RPM 指令，反馈使用该侧四台编码器的相对位移平均值。
启动不清零底盘编码器，且要求八台电机在线、车辆静止；按每台电机的反转配置统一逻辑正向。
启动前应确认八台电机的逻辑 forward 均使车辆前进、编码器增加。
运动使用目标速度前馈、位置 P 修正、左右同步修正，并限制最终指令速度与加减速。
请求线速度超过八台电机中最低齿轮箱转速能力时拒绝启动。
位置在容差内且八台电机低速稳定 0.1 秒才算完成；默认超时为规划时长加 3 秒。
任一电机掉线、异常反馈、超时、控制更新间隔超过 0.25 秒或离开自动时段时立即刹停。
故障退出的刹车不受正常减速斜率限制。

主控底行 `AUTO` 状态：`IDLE` 未启用，`RUN` 运动中，`DONE` 完成，`CONFIG` 参数无效，
`TIMEOUT` 超时，`MOTOR` 电机掉线，`FEEDBACK` 异常读数，`CANCEL` 中断，`MOVING` 启动时未静止。
自动阶段不接受手柄底盘、手臂和夹爪运动指令，气动保持原状态。显示线程只读取状态快照。

供后续自动路线调用的非阻塞接口为 [StartDriveMotion](src/robot-config.cpp)、
[UpdateDriveMotion](src/robot-config.cpp) 和 [CancelDriveMotion](src/robot-config.cpp)，
均由主控制线程调用。自定义参数使用 `motion::TrapezoidOptions`；
运动期间调用 `hardware::Apply(output, false)` 避免手控电压覆盖底盘 RPM。
实际位置精度仍需实车验证，车轮打滑无法通过电机编码器识别。

## 诊断与监测

主控标题为 Bartmoss v1.4.0-route-test，通常显示手臂电压模式与 AUTO 状态；
按一次 Y 进入路线测试后显示路线阶段、底盘参考、反馈、指令和误差。
遥控器第三行为 VLT，第一行 U 表示手臂指令电压（V），L/R 为两侧相对编码器角度。
第二行 1/2/E/C 分别表示 R1、R2、手臂允许控制及配置状态。
主控显示的 U 单位为 mV，dt 为实际主循环间隔。
电压字段表示程序指令；0 表示不再输出运动电压并进入 hold，不代表保持期间电机不通电。

USB 数据线直连主控后，可只读监测：

~~~powershell
.\scripts\monitor-arm.ps1 -Port Auto -DurationSeconds 60
~~~

使用主控用户控制台接口（PID_0501、MI_02），不使用遥控器命令接口。
日志保存在 build/arm-monitor-*.csv，格式为：
tick,R1,R2,enabled,arm_mV,left_deg,right_deg,left_online,right_online,left_A,right_A。
本版本串口行以 BMV, 开头，与旧角度控制 BM5, 格式区分。

底盘测试额外以约 20 Hz 输出 `BMD,` 行，可从 V5 用户串口控制台读取。
首字段为 `BMD`，后续字段顺序为：

~~~text
run,tick,result,phase,feedback_valid,elapsed_s,target_deg,reference_deg,left_deg,right_deg,reference_rpm,left_cmd_rpm,right_cmd_rpm,left_rpm,right_rpm
~~~

`run` 区分每段运动，`phase` 为参考曲线阶段；`reference_rpm` 是左侧规划速度，
`left_cmd_rpm` / `right_cmd_rpm` 是含位置和左右同步修正后的电机指令，
`left_rpm` / `right_rpm` 是实际平均反馈。位置以本次起点为零，单位为电机输出轴角度；
`feedback_valid=0` 时该次反馈不可用于判断实际跟随效果。
原 [monitor-arm.ps1](scripts/monitor-arm.ps1) 仍采集手臂 `BMV,` 数据。

这些即时串口行不构成主控上的轨迹缓存。当前使用方式是下载后拔 USB、通过手柄启动并肉眼观察路线，
本轮不增加底盘采集脚本，也不能事后恢复未采集的完整曲线。

## 测试和文件

Linux / WSL 可运行独立控制逻辑测试：

~~~sh
mkdir -p build
g++ -std=c++11 -Wall -Wextra -Werror -pedantic -I include tests/driver-control-test.cpp -o build/driver-control-test
./build/driver-control-test
~~~

测试覆盖 40,401 种整数摇杆组合，以及手臂即时正反向电压、长按恒定输出、
松手及同时按键、掉线、禁用、夹爪独立运行和气动按下沿。
测试及编译通过不能代替实车验证。

Windows 可运行原手控回归、504 组梯形数学约束、八电机跟随、硬件适配层替身、Y 键状态机和路线阶段测试：

~~~powershell
.\scripts\test-motion-profile.ps1 -CompilerPath 'g++、clang++ 或 Zig 可执行文件路径'
~~~

Linux / WSL：

~~~sh
g++ -std=c++11 -Wall -Wextra -Werror -pedantic -I tests/fakes -I include tests/trapezoid-profile-test.cpp src/robot-config.cpp -o build/trapezoid-profile-test
./build/trapezoid-profile-test
g++ -std=c++11 -Wall -Wextra -Werror -pedantic -I include tests/drive-test-test.cpp -o build/drive-test-test
./build/drive-test-test
g++ -std=c++11 -Wall -Wextra -Werror -pedantic -I include tests/drive-test-route-test.cpp -o build/drive-test-route-test
./build/drive-test-route-test
~~~

替身测试覆盖前后和短距离运动、外部传动比、非零编码器起点、不同电机响应、周期抖动、
速度及加减速约束、非法参数、八个端口分别掉线、堵转超时、场控中断、自动与手动底盘输出隔离。
替身不模拟实际惯性、抓地、电流限制或机械安装方向。
按键状态机测试覆盖初始预持 Y 与重连、Y 按下沿启停、松 Y 后继续、长按不重复触发、
B / 非中性输入 / 权限丢失取消、重新进入条件，以及场控自动运动隔离。
路线阶段测试覆盖直线完成后等待 0.6 s、只发起一次转弯、转弯完成停车、取消、故障及异常时间。
转弯硬件回归覆盖正反输出轴转数、左右反号、遥测单位、限速限加速度及转弯后恢复直线。
本轮最终版本的测试、构建、USB 下载及实车结果以 [LAB_NOTES.md](LAB_NOTES.md) 的实际记录为准。

- [include/control-config.h](include/control-config.h)：设备、按键、电压及控制周期参数。
- [include/driver-control-logic.h](include/driver-control-logic.h)：底盘混控、手臂与夹爪电压及气动状态。
- [include/trapezoid-profile.h](include/trapezoid-profile.h)：从原工程移植的梯形/三角形数学规划与速度变化率限制。
- [include/drive-motion.h](include/drive-motion.h)：八电机反馈、非阻塞跟随及任务结果。
- [include/drive-test.h](include/drive-test.h)：Y 单键启停状态机、接管和取消条件。
- [include/drive-test-route.h](include/drive-test-route.h)：直线、停稳等待、原地转弯的非阻塞阶段协调。
- [src/robot-config.cpp](src/robot-config.cpp)：端口校验、输入采样、直线/转弯硬件适配及机构输出。
- [src/main.cpp](src/main.cpp)：运行条件、路线调度、控制循环、独立显示线程与遥测。
- scripts/、vex/、makefile、.vscode/：构建、只读监测及工程配置。

切换前的完整 PID 工程和校验清单保存在本地 build/backups/ 中。
构建产物、本地备份、参考代码和日志不随程序提交。
