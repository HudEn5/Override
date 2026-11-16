# Bartmoss · 底层 2.0

2026–2027 赛季 VEX V5 机器人控制程序，面向八电机差速底盘、双电机机械臂、夹爪和气动机构。
提供手动驾驶、底盘运动控制、独立实车测试、标定和遥测功能。

**版本：`2.0.0` · 发布标签：`core-v2.0.0`。** 启动通过菜单选择A自动、B手动、X测试，本地与比赛自动共用`src/route.cpp`。
本版本作为自动路线编排的底层基线。软件与模型检查已经覆盖核心控制、调度和并发；实际验证范围及限制见下文。

## 功能

- **手动控制**：前后/转向混控、双臂直接电压控制、夹爪及气动控制。
- **距离直行**：梯形速度规划、编码器位置反馈、惯导航向保持、前馈与速度 PID。
- **原地转向**：相对转角或绝对朝向控制，支持速度变化限制和停稳判定。
- **连续圆角**：直行、转弯、直行连续衔接，根据曲率变化及左右轮加减速约束提前限速。
- **定时前推定位**：`PushFor(1000)`默认4V前推一秒，用于对接物理定位机构，到时制动并停稳。
- **独立运行环境**：日常测试、比赛、标定分别构建，共用底层运动控制器。
- **遥测与并发保护**：通过 VEX VS Code WebSocket 接收数据；共享快照和采样队列使用 mutex，电机由主循环统一输出。

## 快速开始

### 开发环境

| 项目 | 要求 |
| --- | --- |
| 主控与手柄 | VEX V5 Brain、V5 Controller |
| 固件开发 | Visual Studio Code、VEX Robotics 扩展及其 C++ 工具链 |
| VEX SDK | `V5_20240802_15_00_00` |
| 构建脚本 | Windows PowerShell |
| 数据接收 | Node.js 22 或更高版本，仅采集时需要 |
| 离线测试 | Linux / WSL、支持 C++11 的 g++ |

SDK 和工具链由开发环境提供，不包含在仓库中。

### 获取与构建

```powershell
git clone https://github.com/HudEn5/Override.git
cd Override
.\scripts\build.ps1 -Rebuild
```

使用 VS Code 打开仓库根目录，确认 VEX 扩展已识别工程、SDK 和工具链。
脚本支持 `-SdkPath`、`-ToolchainPath` 指定安装位置；多电脑配置见 [设备与同步说明](DEVICE_SYNC.md)。

默认输出为 `build/Bartmoss.bin`，程序名 **Bartmoss**，下载槽位 **1**。
通过VEX扩展下载后，核对主控显示`2.0.0`，保持车辆静止等待惯导校准完成，再从A/B/X菜单选择模式。

### 运行一次底盘测试

1. 在未接场控或比赛开关时运行程序，等待惯导校准和车辆停稳。
2. 在菜单按 X 进入测试，松开选择键；使用上/下选择项目、右/左选择方向。
3. 按一下 A 启动动作，松开后程序继续执行到 `DONE` 或故障停止。
4. 按 B 可取消；拨动摇杆可取消并立即接回手动驾驶。

普通测试配置默认STR；当前高速配置默认CNR 90°。菜单按A直接执行整条自动路线，B进入手动。
手动/测试停车后长按B一秒返回菜单；接入场控后由场控接管。完整说明见 [启动模式](docs/STARTUP_MODES.md)。

## 高速运动测试

低速配置保留，另提供独立 HighSpeed 构建：

```powershell
.\scripts\build.ps1 -Rebuild -Mode HighSpeed
```

输出`build/highspeed/Bartmoss.bin`，不会覆盖默认下载文件`build/Bartmoss.bin`。
菜单A执行 `src/route.cpp` 的整条路线，初始路线为前进500 mm → 连续逆时针圆角90° → 前进500 mm。
路线编排、Wait、无场控测试和比赛接入见 [路线指南](docs/ROUTE_GUIDE.md)。
在 `include/high-speed-config.h` 将 `HIGH_SPEED_CORNER_TEST` 设为 false，X测试切换为 H1～H4 直行整定：
200/265/330/330 rpm 四档、1.5～1.8 米行程；H3 保留基线，H4 在同速同加减速下增强位置纠偏，用于对照验证。
上/下选择档位，右/左选方向，A 单次开始、B 取消、Y 回传；只在无场控时启用。
包含持续达速、实际峰值、航向误差和速度跟踪指标；候选参数尚待实车验证。
切换与完整操作说明见 [高速整定指南](docs/HIGH_SPEED_TUNING.md)。

## 硬件配置

硬件定义集中在 [include/control-config.h](include/control-config.h)。接线或齿轮箱变化后应同步修改配置。

| 设备 | 主控端口 | 电机内部减速比 | 软件反转 |
| --- | --- | --- | --- |
| LF / LM / LBD / LBU | 12 / 1 / 11 / 13 | 6:1 | true / false / true / false |
| RF / RM / RBD / RBU | 6 / 5 / 9 / 10 | 6:1 | false / true / false / true |
| ArmL / ArmR | 3 / 2 | 36:1 | false / true |
| Claw | 15 | 18:1 | true |
| 惯性传感器 | 4 | — | 逆时针为正 |
| TogglePiston | 三线口 H | — | — |

底盘几何参数：轮径 **69.85 mm（2.75 英寸）**，电机输出轴转数/轮子转数为 **36/30**，左右轮距 **325 mm**。
定位反馈来自驱动电机编码器与惯导，未使用独立轮式里程计。

## 手柄操作

### 手动驾驶与机构

| 输入 | 动作 |
| --- | --- |
| 左摇杆 Axis3 | 前进 / 后退 |
| 右摇杆 Axis1 | 转向 |
| R1 / R2 | 双臂上升 12 V / 下降 4 V，松开后 hold |
| L1 / L2 | 夹爪正反向 6 V，松开后 hold |
| X | 切换 H 口气动状态 |

### 日常测试环境（TEST）

| 输入 | 动作 |
| --- | --- |
| 上 / 下短按 | 停稳时循环选择 STR 直行、TURN 原地转向、CNR 连续圆角 |
| 右 / 左短按 | 选择正 / 负方向：直行前进/后退，转向与圆角逆时针/顺时针 |
| A | 启动选中的测试 |
| B | 取消，优先于 A |
| Y | 开关回传，启动时默认关闭 |
| 摇杆 | 取消测试并接回手控 |

运行中选择键与 A 不改变当前动作。按住 A 不重复执行；启动或重连时已经按住 A 不会起步。
取消、禁用或故障后，需要重新松开并按 A。接入场控或比赛开关后，日常测试立即退出。

遥控器显示项目、目标参数、状态及航向误差。圆角运行时 `V` 为目标中心轮速，`P` 为预测速度上限，
两者均非实测速度；减速跟随期间 V 可能暂时高于 P。

## 参数配置

| 配置文件 | 内容 |
| --- | --- |
| [control-config.h](include/control-config.h) | 接线、方向、手控电压、前馈/PID、几何及运动控制参数 |
| [test-config.h](include/test-config.h) | 三项日常测试的距离、转角和最高速度 |
| [calibration-config.h](include/calibration-config.h) | 专用标定档位、手势时间及超时 |
| [motion-presets.h](include/motion-presets.h) | 路线与高速测试共用的加减速和圆角规划配置 |
| [high-speed-config.h](include/high-speed-config.h) | X测试项目、高速档位与候选参数 |
| [route-commands.h](include/route-commands.h) | 路线接口、默认转速与前推电压 |

显式Test构建保留的低速参数如下；默认高速构建的X测试为500mm/330rpm、90°/140rpm、500/90°/500圆角/330rpm上限：

| 项目 | 距离 / 相对转角 | 最高电机输出轴转速 |
| --- | --- | --- |
| STR | 500 mm | 100 rpm |
| TURN | 90° | 单侧 100 rpm |
| CNR | 直行 500 mm → 30° 圆角 → 直行 500 mm | 单侧 600 rpm 上限 |

测试配置中的距离与转角填写正数，方向由手柄选择。连续圆角始终向前行驶，方向键选择左右转弯。
600 rpm 是单侧目标上限，实际速度还受距离、加减速、弯速和预测约束限制。

低速圆角最小参考曲率半径为250mm，曲率渐入渐出；它不是固定半径圆弧。
`corner_rpm=140`、`corner_floor_rpm=100`，预测限速可以低于 floor。
预测单侧加速/减速约束为 180/120 rpm/s，余量系数 0.85，并预览 200 ms 响应距离。
这些约束是软件规划参数，尚非实测电机能力标定值。

## 运行环境与比赛接入

```powershell
.\scripts\build.ps1 -Rebuild -Mode Test
.\scripts\build.ps1 -Rebuild -Mode Competition
.\scripts\build.ps1 -Rebuild -Mode Calibration
```

| 构建模式 | 输出 | 用途 |
| --- | --- | --- |
| Default（不传 -Mode） | `build/Bartmoss.bin` | 当前为高速配置，A/B/X菜单 |
| Test | `build/test/Bartmoss.bin` | 三项日常运动测试和手控 |
| Competition | `build/competition/Bartmoss.bin` | 场控自动与比赛手动阶段 |
| Calibration | `build/calibration/Bartmoss.bin` | VTEST / FH / SP 标定与诊断 |
| HighSpeed | `build/highspeed/Bartmoss.bin` | 路线调试或高速参数测试 |

显式模式使用独立输出目录。构建不会自动下载，也不会切换 VEX 扩展默认下载文件。
如果使用扩展默认下载入口切换环境，修改 control-config.h 中的宏后重新执行默认构建：

| 环境 | BARTMOSS_COMPETITION | BARTMOSS_CALIBRATION | BARTMOSS_HIGH_SPEED |
| --- | --- | --- | --- |
| TEST | 0 | 0 | 0 |
| 高速 / 路线调试 | 0 | 0 | 1 |
| COMP | 1 | 0 | 0 |
| CAL | 0 | 1 | 0 |

三个专用宏最多一个为1。比赛回调发布阶段状态，主循环根据场控使能和模式分配控制权。
非标定构建在场控连接时均使用比赛权限，菜单选择失效；比赛自动不依赖手柄在线，场控期间不启用测试和Y回传。

自动路线位于 [src/route.cpp](src/route.cpp)，顺序写动作即可，底层自动等待完成：

```cpp
using namespace route;
RunRoute({
    DriveStraight(500),
    Wait(200),
    Turn(90),
    DriveStraight(500),
});
```

默认直行330 rpm、原地转向140 rpm，距离mm、角度度、等待毫秒；`Turn`为相对角，正角逆时针。
`PushFor(1000)`为定位机构提供一秒定时前推，默认4V；可选第二参数指定电压V，不自动识别对接成功。
连续不停顿使用 `DriveCorner(500, 90, 500)`，它也是当前文件默认路线。`TurnToHeading`支持绝对朝向。
测试与比赛共用高速参数，低速三项测试保留；任一动作故障或取消会停止整条路线。
`Wait` 不阻塞主循环。完整接口、按钮和验证范围见 [路线指南](docs/ROUTE_GUIDE.md)。

## 遥测与标定

### 采集数据

遥控器通过 USB 连接电脑，并与主控保持无线连接；VS Code 的 VEX 扩展开启本机 WebSocket 服务。
仓库工作区设置使用 `127.0.0.1:7071`。接收脚本只读取数据，不发送运动指令。

```powershell
node scripts/monitor-drive-websocket.mjs 120
```

连接成功后按 Y 开启回传，用 A 启动测试；结束后按 Y 关闭。
数据保存到`build/data/drive-websocket-时间戳*`，内部采样约50Hz；默认高速配置回传最多2Hz，普通/标定构建最多5Hz。
高速构建仅发送当前动作及路线状态，基础速度/电机诊断CSV只有表头属预期行为。

| 输出 | 内容 |
| --- | --- |
| `.log` / `.csv` | 原始回传 / 八电机速度、指令与有效性 |
| `-profile.csv` / `-heading.csv` | 直行规划 / 航向保持 |
| `-velocity.csv` / `-quality.csv` | 速度 PID / 测速质量 |
| `-turn.csv` / `-corner.csv` | 原地转向 / 连续圆角 |
| `-motors.csv` | LM/LF 电机诊断 |
| `-route.csv` / `-push.csv` | 自动路线步骤与结果 / 定时前推电压、计时和停稳 |

控制快照与电机采样具有各自时间戳，同序号不代表物理上同时读取。

### 专用标定环境

CAL 环境提供固定电压 VTEST、前馈加航向保持 FH、速度闭环 SP。
停稳后长按下键 1 秒切换模式，短按上/下换档；按住右/左前进/后退，松开刹车，每段最多 3 秒。
电压档位为 2/3/4 V，转速档位为 100/150/200 rpm。A/B 或摇杆退出动作，Y 开关回传。
日常 TEST 环境不包含该标定入口。

## 验证与当前限制

运行宿主机测试：

```sh
bash scripts/test-host.sh
```

在 Windows 运行脚本测试：

```powershell
node tests/drive-websocket-test.mjs
node tests/sync-github.test.mjs
```

当前软件验证包括30个C++测试程序、当前高速圆角432组与原地转向72组角度/速度/负载模型，
以及原预测504组等回归、576种基础环境组合、按键与动作生命周期、反馈故障和采样/回调/快照并发一致性检查。
Default、Test、Competition、Calibration、HighSpeed 五种 V5 SDK 构建均已通过。

- 高速直行已有多轮实车数据；高速90°连续圆角已有一次DONE、最终航向误差约1°的记录。
- 新启动入口、定时前推、真实场控切换和跨场地一致性尚未完成实车验证；30°/120°当前高速角度模型通过，实车待确认。
- 90°圆角实车过程中曾有10.2°跟踪滞后，模型通过不能保证任意速度/角度均不触发保护。
- 驱动轮打滑会影响编码器估计的地面距离，惯导航向保持不能修正已发生的横向位移。
- 圆角终点使用 5 mm 编码器距离窗口、1.5° 惯导角度窗口和持续停稳条件；原地转向角度窗口为 0.8°。
  完成阈值不等于已经证明的地面定位精度。
- 圆角中途航向误差超过15°、设备掉线、反馈无效或超时会停止动作；2/5Hz快照不足以排除样本间瞬态。

历次实车现象和验证范围记录在 [LAB_NOTES.md](LAB_NOTES.md)。

## 项目结构

```text
include/    硬件配置、控制算法、状态与接口
src/        主循环、硬件适配、路线、显示与回传
scripts/    构建、数据接收、软件测试与 Git 同步工具
tests/      宿主机测试及硬件替身
docs/       代码结构说明
vex/        VEX 构建规则
.vscode/    工程、工具链补全与任务配置
```

构建产物、采集数据和本地备份保存在被 Git 忽略的 `build/` 中；SDK 与本机配置不随仓库分发。
修改接线或控制参数时同步更新配置与文档，提交前运行相关软件测试和 V5 构建。

- [代码结构与公共接口](docs/ARCHITECTURE.md)
- [设备配置与多电脑同步](DEVICE_SYNC.md)
- [实验与验证记录](LAB_NOTES.md)
- [路线编排与动作接口](docs/ROUTE_GUIDE.md)
- [启动模式与按钮](docs/STARTUP_MODES.md)
- [版本记录](CHANGELOG.md)
