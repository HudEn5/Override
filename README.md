# Bartmoss

2026–2027 赛季 VEX V5 机器人控制程序，面向八电机差速底盘、双电机机械臂、夹爪和气动机构。
提供手动驾驶、底盘运动控制、独立实车测试、标定和遥测功能。

**当前版本：`1.6.0-tests` · 开发中。** 默认启动日常测试环境；正式比赛自动路线尚未配置。
当前版本已通过软件测试和 V5 SDK 构建，测试界面重构与预测限速仍需实车验证。

## 功能

- **手动控制**：前后/转向混控、双臂直接电压控制、夹爪及气动控制。
- **距离直行**：梯形速度规划、编码器位置反馈、惯导航向保持、前馈与速度 PID。
- **原地转向**：相对转角或绝对朝向控制，支持速度变化限制和停稳判定。
- **连续圆角**：直行、转弯、直行连续衔接，根据曲率变化及左右轮加减速约束提前限速。
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
通过 VEX 扩展下载后，核对主控显示 `1.6.0-tests` 和 `TEST`，保持车辆静止等待惯导校准完成。

### 运行一次底盘测试

1. 在未接场控或比赛开关时运行 TEST 程序，等待惯导校准和车辆停稳。
2. 使用上/下键选择 STR、TURN 或 CNR，使用右/左键选择正/负方向。
3. 按一下 A 启动动作，松开后程序继续执行到 `DONE` 或故障停止。
4. 按 B 可取消；拨动摇杆可取消并立即接回手动驾驶。

默认选中 STR。测试当前 30° 连续圆角时，从默认项目短按上键两次选中 CNR，再按 A。

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

当前日常测试参数：

| 项目 | 距离 / 相对转角 | 最高电机输出轴转速 |
| --- | --- | --- |
| STR | 500 mm | 100 rpm |
| TURN | 90° | 单侧 100 rpm |
| CNR | 直行 500 mm → 30° 圆角 → 直行 500 mm | 单侧 600 rpm 上限 |

测试配置中的距离与转角填写正数，方向由手柄选择。连续圆角始终向前行驶，方向键选择左右转弯。
600 rpm 是单侧目标上限，实际速度还受距离、加减速、弯速和预测约束限制。

圆角当前最小参考曲率半径为 250 mm，曲率渐入渐出；它不是固定半径圆弧。
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
| Default（不传 -Mode） | `build/Bartmoss.bin` | 使用头文件默认环境，当前为 TEST |
| Test | `build/test/Bartmoss.bin` | 三项日常运动测试和手控 |
| Competition | `build/competition/Bartmoss.bin` | 场控自动与比赛手动阶段 |
| Calibration | `build/calibration/Bartmoss.bin` | VTEST / FH / SP 标定与诊断 |

显式模式使用独立输出目录。构建不会自动下载，也不会切换 VEX 扩展默认下载文件。
如果使用扩展默认下载入口切换环境，修改 control-config.h 中的宏后重新执行默认构建：

| 环境 | BARTMOSS_COMPETITION | BARTMOSS_CALIBRATION |
| --- | --- | --- |
| TEST | 0 | 0 |
| COMP | 1 | 0 |
| CAL | 0 | 1 |

两个宏不能同时为 1。比赛回调发布阶段状态，主循环根据场控使能和模式分配控制权。
比赛自动不依赖手柄在线，比赛环境不启用日常测试与 Y 回传。

正式自动路线位于 [src/route.cpp](src/route.cpp)，当前为 `EMPTY`，不会自行执行测试路线。
路线通过非阻塞接口编排动作：

| 动作 | 独立调用示例 | 完成判定 |
| --- | --- | --- |
| 直行 | `StartDriveStraight(500, 100)` | `DriveStraightState()` |
| 相对转向 | `StartTurnRelative(90, 100)` | `TurnState()` |
| 绝对朝向 | `StartTurnToHeading(0, 100)` | `TurnState()` |
| 连续圆角 | `StartDriveCorner(500, 30, 500, 600)` | `CornerState()` |

同一时刻只执行一个动作，等待对应状态为 `done` 后再进入下一步；避免在主循环中用阻塞等待编排路线。
更多接口与执行流程见 [代码结构](docs/ARCHITECTURE.md)。

## 遥测与标定

### 采集数据

遥控器通过 USB 连接电脑，并与主控保持无线连接；VS Code 的 VEX 扩展开启本机 WebSocket 服务。
仓库工作区设置使用 `127.0.0.1:7071`。接收脚本只读取数据，不发送运动指令。

```powershell
node scripts/monitor-drive-websocket.mjs 120
```

连接成功后按 Y 开启回传，用 A 启动测试；结束后按 Y 关闭。
数据保存到 `build/data/drive-websocket-时间戳*`，内部采样约 50 Hz，回传最多 5 Hz。

| 输出 | 内容 |
| --- | --- |
| `.log` / `.csv` | 原始回传 / 八电机速度、指令与有效性 |
| `-profile.csv` / `-heading.csv` | 直行规划 / 航向保持 |
| `-velocity.csv` / `-quality.csv` | 速度 PID / 测速质量 |
| `-turn.csv` / `-corner.csv` | 原地转向 / 连续圆角 |
| `-motors.csv` | LM/LF 电机诊断 |

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

当前软件验证包括 23 个 C++ 测试程序、504 个预测运动模型工况、576 种环境组合、
按键与动作生命周期、反馈故障处理，以及采样/回调/运行快照的并发一致性检查。
Default、Test、Competition、Calibration 四种 V5 SDK 构建均已通过。

- 当前测试重构版与预测限速仍待实车验证；真实场控切换和跨场地一致性也尚未完成验证。
- 驱动轮打滑会影响编码器估计的地面距离，惯导航向保持不能修正已发生的横向位移。
- 圆角终点使用 5 mm 编码器距离窗口、1.5° 惯导角度窗口和持续停稳条件；原地转向角度窗口为 0.8°。
  完成阈值不等于已经证明的地面定位精度。
- 圆角中途航向误差超过 15°、设备掉线、反馈无效或超时会停止动作；5 Hz 回传不足以排除样本间瞬态。

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
