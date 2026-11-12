# Bartmoss v1.2.0-control · 2026–2027 控制程序

台式机与实验室笔记本通过 GitHub `main` 同步，操作见 [DEVICE_SYNC.md](DEVICE_SYNC.md)。
另一台电脑的 Codex 可从 [LAPTOP_PROMPT.md](LAPTOP_PROMPT.md) 接手；实测记录写入 [LAB_NOTES.md](LAB_NOTES.md)。

VEX V5 八电机差速底盘手控工程，手臂采用直接电压控制。
沿用参考工程 Tungsten 的设备命名。当前版本用于实机调试，尚未完成整机验证。

## 自动、手动和测试环境

参考 Tungsten 的 `main.cpp`、`route.cpp`、`chassis::MovementDrive()` 分层：
注册比赛入口，路线只下达动作，统一控制循环负责电机输出。

| 层 | Bartmoss 文件 | 职责 |
| --- | --- | --- |
| 比赛接入 | `src/competition-control.cpp` | 注册 `Competition.autonomous(autonomous)` 和 `Competition.drivercontrol(usercontrol)` |
| 正式路线 | `src/route.cpp` | 非阻塞编排比赛自动动作，默认 EMPTY，不自动执行测试路线 |
| 自动生命周期 | `src/autonomous.cpp` | 等待就绪、每自动时段执行一次、离开自动时段取消 |
| 手控 | `include/driver-control-logic.h` | Axis3/Axis1、手臂、夹爪、气动的原有手控 |
| 手柄测试 | `include/voltage-test.h`、`src/turn-test.cpp` | VTEST/FH/SP/TP 与 A/B，仅在测试环境无场控时启用 |
| 公共直行 | `include/drive-straight.h` | 距离/最高速度接口，共用梯形、位置/航向修正、前馈和速度 PID |

Tungsten 由常驻底盘线程在自动目标与手柄目标间切换；Bartmoss 保持同一个主控制线程拥有电机输出。
两个 VEX 回调只通过 mutex 发布阶段快照，不读写底盘或创建运动线程。
主循环同时检查回调阶段、场控连接、Enabled、Autonomous/DriverControl，禁用立即撤销动作。
比赛自动不依赖手柄在线，不接受 A/B、方向键测试或 Y 回传；测试版本不会因场控进入自动时段执行比赛路线。
控制线程持续存在，两个比赛回调均立即返回；SDK 的阶段切换不终止常驻采样/显示线程。
主控显示 `TEST` / `COMP`，`CB=1` 为自动回调、`CB=2` 为手动回调，冒号后为通知次数。

默认仍是测试环境。在 `include/control-config.h` 中，`BARTMOSS_COMPETITION=0` 为测试、1 为比赛。
通过 VEX 扩展日常下载时，修改该默认宏并完整重编译，下载后核对主控 TEST/COMP 标识。
也可分别构建，不覆盖默认下载文件：

~~~powershell
.\scripts\build.ps1 -Rebuild -Mode Test
.\scripts\build.ps1 -Rebuild -Mode Competition
~~~

输出分别为 `build/test/Bartmoss.bin` 和 `build/competition/Bartmoss.bin`；两个显式模式使用独立对象目录。
脚本无 `-Mode` 时沿用头文件默认值，输出 `build/Bartmoss.bin`，与现有 VEX 下载流程相同。
VS Code 也提供对应的两项构建任务；构建不会自动下载或运行程序，也不会自动切换扩展的下载文件。

## 公共直行接口与比赛路线

纯控制器先由主循环传入反馈和运行许可，路线代码只需调用：

~~~cpp
StartDriveStraight(500, 100);   // 前进 500 mm，最高输出轴 100 rpm
StartDriveStraight(-500, 100);  // 后退；这两行不能连续调用来代替等待完成
~~~

正在运动时再次 Start 返回 false，不覆盖已有动作；零距离为无运动的 DONE。
单位、前馈/PID、加减速和容差仍来自现有配置；TP 与比赛实例使用同一个 straight_controller 实现。
比赛每个动作锁定当时朝向，TP 保留同一测试会话的参考朝向。退出、故障或取消均清空速度环输出。
全部接口仅由主线程调用；主循环继续执行 Update，不能在路线中写 while/wait 阻塞它。

正式路线位于 `src/route.cpp` 的 `autonomous_program::RunRoutine()`，默认只 `FinishRoutine("EMPTY")`。
以下是替换函数体后的单段直行示例，当前默认程序没有启用它：

~~~cpp
switch (info.step) {
  case 0:
    if (!StartDriveStraight(500, 100)) {
      FinishRoutine("START ERROR");
      return;
    }
    info.step = 1;
    break;
  case 1:
    if (DriveStraightState() == drive::straight_state::done)
      FinishRoutine();
    break;
}
~~~

路线在硬件、惯导和速度反馈就绪、车辆静止后才启动。运动错误会终止整条路线，
同一自动时段完成/失败后不重复执行，下一次自动时段重新开始。
SDK 比赛回调注册及阶段切换只做了软件与编译验证，尚未通过真实场控/比赛开关进行实车验证。

## 手柄操控

| 输入 | 动作 |
| --- | --- |
| 左摇杆上下 Axis3 | 前进 / 后退 |
| 右摇杆左右 Axis1 | 转向 |
| 按住 R1 | 两台手臂电机立即接收上升电压，默认 12 V |
| 按住 R2 | 两台手臂电机立即接收下降电压，默认反向 4 V |
| 松开 R1/R2 或同时按下 | 停止手臂，使用 VEX 内置 hold 保持松手时的位置 |
| 按住 L1 / L2 | 夹爪夹紧 / 松开，默认 6 V |
| X 按下沿 | 切换 H 口气动状态 |
| A 按下沿 | 启动或取消逆时针 90° 底盘测试 |
| B 按下沿 | 启动边长 500 mm 的正方形测试 |
| Y 按下沿 | 开始 / 停止数据回传，启动时默认关闭 |
| 短按方向键上 / 下 | 测试升档 / 降档，下键松手时降档 |
| 停车长按下方向键 1 秒 | 依次切换 VTEST / FH / SP / TP，不使用组合键 |
| 按住方向键右 / 左 | 当前测试模式前进 / 后退，松开停止 |

底盘按“左 = 前后 + 转向，右 = 前后 − 转向”混控。
摇杆死区 5%，最大输出 ±12 V，超限时左右同比例缩小。
夹爪松手使用 hold，底盘停止使用 coast。
X 按住不会反复切换，启动或重连时已按住 X 不计为新一次按下。

## 惯导转向与正方形测试

惯导连接主控 4 号口，水平固定。启动程序后保持整车静止，主控底行显示 IMU CAL，
校准完成显示 READY。惯导启动时缺失或运行中掉线后，重新连接并重启程序完成校准。

- 松开摇杆、停稳后按 A，单次逆时针转 90°。
- 按 B，重复四次“前进 500 mm、逆时针转 90°”，最终朝向回到启动时的方向。
- 自动运行中按 A/B 或拨动摇杆可取消。测试仅用于无场控的手柄调试。
- 惯导异常、底盘电机掉线或单段超过 12 秒时终止动作。

半米测距需要在 include/control-config.h 填写 DRIVE_WHEEL_DIAMETER_MM 和
DRIVE_MOTOR_REVS_PER_WHEEL_REV。后者是电机减速箱输出轴转数除以轮子转数，
直接连接为 1，主动 12 齿带动从动 36 齿为 3。当前已配置 69.85 mm 轮径与 36/30 外部减速比。轮径填 0 时 B 显示 SET WHEEL，
A 键转向仍可使用。

距离由左右驱动编码器平均位移换算，转向和直行朝向由惯导闭环修正。
每个电机使用 SDK 的转速控制，外层用距离比例控制和航向 PD，靠近目标降速。
当前采用最高 35 rpm 前进，原地转向大角度阶段最高 60 rpm；剩余 50° 到 25° 之间平滑减速，25° 内使用原有精调参数。误差和实测速度持续稳定 250 ms 后进入下一段。
转角目标始终基于路线起始朝向，依次为 90/180/270/360 度。

运行时遥控器显示 S（当前边数）、D（本边编码器估计距离 mm）、A（相对起始转角 deg）
和 E（当前航向误差）。主控底行持续显示状态、距离和角度。
此为四边路线验证，驱动轮打滑和轮径误差会造成位置闭合偏差，完成后需实测回到起点的距离。
当前控制参数通过离线模拟检查，仍需实车整定。

## 固定电压采集

默认模式为固定电压、档位为 2 V。上键按下升档，下键短按松手后降档，范围为 2、3、4 V。
停车长按下键 1 秒切换前馈模式，继续按住只切换一次，松手不附带降档。
按住方向键右给左右底盘 +选定电压，按住方向键左给左右底盘 -选定电压。
松开后主动刹车；每次连续运行最多 3 秒，TIMEOUT 后必须松开运行键才能再次启动。
左右同时按下会停止，直接换向也要求先松手。运行中换档不生效，停止后重新按上 / 下换档。
遥控器显示 VTEST、选定电压和 READY/FWD/REV/TIMEOUT/RELEASE 状态，第二行显示左右指令电压。

先运行电脑接收脚本，再按 Y 打开回传；固定电压测试不自动开启或关闭回传。
在地面正常负载下逐档做前进、停稳、后退，每档可重复测试。松开运行键停车，Y 仅开关日志。
测试指令写入回传的 left_command_mV/right_command_mV，继续保留实测电压与八台转速。
固定电压测试使用开环电压，不使用 SDK 转速控制、航向修正、速度 PID 或前馈。

拨动摇杆立即退出电压测试并接回正常驾驶，之后须松开运行键才能重新测试。
运行时按 A/B 会先退出电压测试，该次按下不会启动自动动作，需松开后重新按 A/B。
自动动作中按住电压运行键会取消自动动作，松开后重新按才能启动电压测试。
启动或重连时已按住运行键不会启动；场控接入、手控禁用、底盘电机掉线时停止测试。
惯导校准期间仍保持底盘静止。档位和单次时间上限配置在
`DRIVE_TEST_VOLTAGES`、`DRIVE_TEST_TIMEOUT_MS`。

## 前馈转速测试

车停稳、左右运行键松开后，长按下方向键约 1 秒，遥控器由 VTEST 切换为 FH（前馈加航向保持）。
短按上 / 下选择 100、150、200 rpm，上键按下升档、下键松手降档。
按住右键前进，按住左键后退；松开刹车，每次仍最多运行 3 秒。
每次停车长按下键 1 秒，依次切换 VTEST → FH → SP → TP → VTEST；每次切换后先松开下键。
VTEST 独立记忆电压档位，FH/SP/TP 共用转速档位，启动默认固定电压。
上下同时按下不会切换模式；运行、车辆尚未停稳、校准、掉线或禁用期间不允许切换。
停车判据为两侧测速有效且转速绝对值都小于 5 rpm；TP 使用位置差分速度，其余模式使用原始 SDK 平均值。
长按期间出现移动会取消手势。

遥控器第一行显示 FH 或 SP、选定 / 运行中的有符号目标 rpm 和状态，第二行显示 L/R 实测滤波 rpm。
测速不可用时显示 --，第三行显示航向误差 H、参考是否锁定及 LOG:ON/OFF。
Y 开关回传与测试模式独立，电脑接收命令保持不变。
CSV 中 mode=4 表示 FH，mode=5 表示 SP；旧版 mode=2 为纯前馈。
target_rpm 为运行中的带符号中心目标转速，停止时为 0；航向外环可能让两侧目标不同。

4 号口惯导校准完成后，第一次运行锁定当时方向。同一轮的停车、前后换向和换档均保留该参考角，
不把停车产生的偏差作为新的参考。松手仅执行 brake，不主动原地转向；下次行驶时修正偏差。
拨动摇杆、按 A/B、禁用、设备掉线，或长按下键切换测试模式会清除参考。
需要手动重新确定方向时，退出测试再进入即可。Y 只控制回传，不清除参考。
航向使用比例加角速度阻尼，初始参数在 `include/heading-control.h` 的 heading_config 中，
通过 `config::FF_HEADING` 配置：kP=80 mV/度、kD=10 mV/(度/秒)、角速度滤波 60 ms。
修正限制为 500 mV、较小侧基础电压的 25% 及电压余量三者的最小值。
偏差超过 15 度显示 HDG ERROR 并停车；惯导无效显示 IMU WAIT，恢复后必须松开运行键再启动。
这些航向初值已做软件及实车基础测试，停车换向偏转仍存在，按用户要求暂缓优化。
保持角度无法消除已有的横向位置偏移。

前馈按当前硬件的三轮正常负载 FF 验证数据更新：每段排除最初 1 秒，保留有效样本、四台电机平均值，
不剔除 LM；18 个片段等权拟合，共 158 条运行后段样本，左右、正反向分开处理。
前进：左 kS=372.573735 mV、kV=17.900544 mV/rpm，右 kS=408.162564 mV、kV=16.537990 mV/rpm。
后退：左 kS=547.829766 mV、kV=16.939618 mV/rpm，右 kS=537.033135 mV、kV=15.870378 mV/rpm。
目标单位是电机减速箱输出轴 rpm，kA 暂为 0；拟合使用指令电压，适用测试范围 100～200 rpm。
参数配置在 FF_TEST_LEFT/RIGHT 和 FF_TEST_LEFT_REVERSE/RIGHT_REVERSE，目标档位在 FF_TEST_TARGETS。
已完成当前硬件 100、150、200 rpm 的 SP 单轮基础测试，结果见 LAB_NOTES.md；
尚未完成重复性及跨场地验证，不能把离线拟合误差视为闭环转速精度。
FH 基础电压使用 kS*方向+kV*目标rpm，再叠加惯导航向修正；不调用 SDK 转速控制或软件速度 PID。
两侧模型无效或任一侧电压超出上限时停止，显示 FF ERROR，松开运行键后才能重试。
参数仅在本测试模式启用，正常手控和 A/B 自动动作不使用这组初值。
本模式用于独立验证目标 / 实际速度差，未承诺闭环跟踪精度，尚待实车验证。

## 速度闭环测试（SP）

SP 在前馈和现有航向保持基础上增加左右独立的速度 PID，普通手控与 A/B 自动动作仍使用原来的逻辑。
速度 PID 实现在 `include/velocity-control.h`，由 `config::DRIVE_VELOCITY_PID` 配置。
调试初值为 kP=6 mV/rpm、kI=2 mV/(rpm·秒)、kD=0；已完成三个档位的实车基础验证，尚未精细整定。
速度反馈使用原有四台电机平均值和 40 ms 低通滤波，不剔除 LM。
航向外环先生成差速电压，再按各侧、各方向的 kV 换算成差速转速目标；速度内环跟踪这组目标，
而非强迫两侧转速相等。当前仍没有增加目标速度规划；后续可把规划器目标接入速度控制层。

PID 仅在新速度样本到达时更新，使用样本实际时间差，重复快照不重复积分。
微分对测量值求导，避免目标阶跃引发冲击；D 项目前关闭。
积分项限制为 ±300 mV，饱和方向不继续积分。速度修正最多 ±800 mV，
同时限制在该侧基础电压一半及 12 V 余量以内；总电压不超过 ±12 V。
松手、模式切换、摇杆接管或故障会清空积分，恢复后不能自动启动。
没有有效测速、超过 100 ms 未收到新序号样本，或运行中样本时间差不在 (0, 200] ms 内，
显示 SPEED ERR 并停车，必须松开运行键后重试。
这些保护不包含碰撞识别，测试仍按住右键前进 / 左键后退，松手刹车，每段最多 3 秒。

首次实车验证先用 100 rpm，采集目标、实际速度和 PID 修正电压；再判断是否增加档位或调整增益。
2026-11-12 采集的 `drive-websocket-1794483839438` 中，用户报告 200 rpm 后退撞到障碍，
整段 -200 rpm 已标记为不用于调参，原始数据保留。此轮没有重新拟合前馈或继续调整换向航向参数。

## 梯形速度规划测试（TP）

合入同事 `feature/trapezoid-speed-tests`（`af8e5e7`）的 `TrapezoidProfile` 和 `limitVelocity` 数学实现。
以 main 的设备、按键、采样和控制参数为准，程序名 Bartmoss；Y 仍仅开关回传，普通手控与 A/B 动作不变。
同事分支的 Y 键路线、20 Hz BMD 输出和 SDK 转速执行器未接入本程序；其完整实现仍保留在原分支。

启动后保持静止完成惯导校准，停车长按下键依次进入 TP，每次切换后松开下键。
短按上/下选择最高 100、150、200 rpm；按住右键走 +500 mm，按住左键走 -500 mm。
完成后自动刹车，继续按住不会重复启动，必须松开后再按；途中松手立即取消并刹车。
运行中换向要求先松手，摇杆/A/B/掉线/禁用的接管和互锁沿用现有测试设置。

距离采用已有 69.85 mm 轮径和 36/30 外部传动比，从每次起点的相对编码器位移计算，不清零电机编码器。
规划统一用 mm、mm/s、秒，随后换成电机输出轴 rpm；位置纠偏用左右平均位移，航向仍由 4 号口惯导反馈。
规划参考加减速度均为 120 rpm/s；距离 500 mm、位置比例增益 0.25 rpm/mm、纠偏上限 20 rpm、
位置窗口 5 mm、左右滤波转速窗口各 5 rpm、停稳时间 250 ms。
v1.1.1 起 TP 速度环使用八台电机位置差分，每侧四台取平均、20 ms 低通，
不依赖低速下偶发尖峰的 SDK velocity 值；屏幕 TP 实测速度也来自该源。
距离制动包络另预留 200 ms 响应时间，减速参考实际剩余距离，不只跟随参考曲线的时间。
进入终点阶段后，在 5 mm 窗口外以至少 15 rpm、最多 20 rpm 的目标回调，仍受所选档位及变化率限制。
这是调试初值，用于避免先前 1～4 rpm 回调长期不收敛；不保证真实地面上的 5 mm 精度。
参数在 `config::TP_TEST` / `drive::trapezoid_config`；最高转速是上限，短距离会自动变成三角形。
当前 500 mm、120 rpm/s 配置下，150 和 200 rpm 档的参考峰值都约 140 rpm，不会强行加速到选定上限。

控制顺序为：梯形位置/速度参考 → 平均位置误差修正与速度变化率限制 → 惯导航向差速 →
共用电压前馈与速度 PID。SP 与 TP 共用 `include/drive-velocity.h`，FF/PID 数值未重新调整，kA 仍为 0。
TP 在低速时还把航向差速对应的目标偏移限制在中心目标的 25% 内，避免静摩擦电压使目标跨过零点。
规划结束后还需实际位移和速度满足窗口并持续稳定，不能仅按规划时间宣布完成。
进入终点位置窗口后直接 brake 等待停稳；取消和故障同样立即制动。
DONE/TIMEOUT/规划错误终态保留在屏幕，继续按住不会重复运动；下次起步或切换模式才更新。
超时为规划时长 +3 秒，不继承 VTEST/FH/SP 的固定 3 秒运行上限。
所有输出仍由主循环统一发送，显示、回传只读取由 mutex 保护的快照。

TP 仅用于无场控测试环境，比赛自动从独立的 route.cpp 调用公共直行层；本阶段不新增弧线或转弯路线。
数学规划、合成负载跟随、完成判据、取消和故障检查通过；尚未执行合并版的实车下载或验证。
现有前馈只完成 100～200 rpm 的基础实测，TP 经过的低速区与终点表现仍需测试；首次用 100 rpm 档。

## 手臂纯电压模式

R1/R2 直接决定输出电压；运动期间不使用自定义 PID、角度目标、滤波或电压变化率限制。
两个电机接收相同逻辑电压，ArmR 的反转配置补偿安装方向。
上升和下降电压可以分别调整：

~~~cpp
constexpr int ARM_UP_VOLTAGE = 12000;   // 上升电压，单位 mV
constexpr int ARM_DOWN_VOLTAGE = 4000;  // 下降电压，填写正数
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
| Inertial | 4 | — | 逆时针为正 |

智能端口按主控编号 1～21 填写，创建设备前校验范围及重复端口。
设备和按键均配置在 include/control-config.h，惯导端口为 4。

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
连接场控后，仅在已启用的手动时段输出；禁用、自动时段或手柄断连时电机停止，气动保持状态。
比赛模式支持独立自动路线，当前路线默认 EMPTY；场控禁用期间的实际电机行为由 VEX 系统接管。

## 诊断与监测

主控标题为 Bartmoss v1.2.0-control，并显示 TEST/COMP；底行显示对应测试或比赛自动状态。
遥控器第三行显示 VLT、LOG:ON/OFF（回传开关）和 M（两侧手臂在线状态）。
第一行 U 表示手臂指令电压（V），L/R 为两侧相对编码器角度。
第二行 1/2/E/C 分别表示 R1、R2、手臂允许控制及配置状态。
主控显示的 U 单位为 mV，dt 为实际主循环间隔。
电压字段表示程序指令；0 表示不再输出运动电压并进入 hold，不代表保持期间电机不通电。

手臂 BMV 日志默认关闭。如需使用，先将 `ARM_SERIAL_LOG` 设为 true 并重新下载，
再按 Y 开启回传。USB 数据线直连主控后，可只读监测：

~~~powershell
.\scripts\monitor-arm.ps1 -Port Auto -DurationSeconds 60
~~~

使用主控用户控制台接口（PID_0501、MI_02），不使用遥控器命令接口。
日志保存在 build/arm-monitor-*.csv，格式为：
tick,R1,R2,enabled,arm_mV,left_deg,right_deg,left_online,right_online,left_A,right_A。
本版本串口行以 BMV, 开头，与旧角度控制 BM5, 格式区分。

## 底盘速度采样与前馈

独立采样线程以约 20 ms 的目标间隔读取八台电机的速度和实测电压。
启动时默认不输出任何速度 / BMV 日志。遥控器按一次 Y 开始回传，再按一次 Y 停止；
LOG:ON/OFF 显示当前开关状态。长按只切换一次，启动或重连时已按住 Y 不触发，
遥控器掉线后关闭回传。开关仅控制日志，不控制机器人运动。
回传最多每 200 ms 一条速度数据（5 Hz），由 `TELEMETRY_INTERVAL_MS` 配置，内部采样仍约 50 Hz。
打开和关闭时清空发送队列，不补发关闭期间的数据。串口由独立线程输出，不等待屏幕刷新。
sequence 是内部采样序号，因此正常回传时通常相隔约 10；dt_ms 仍是内部采样间隔。
检查传输缺失应结合 time_ms 和回传间隔，不能把主动降频当作无线丢包。
电机速度单位为内部减速箱输出轴 rpm，加速度为 rpm/s，电压为 mV。
轮径和外部传动比确定后，再与路线规划的 mm/s 做换算。
左、右侧各四台电机全部有效时才计算该侧平均值，并保留每台的原始速度。
SDK 转速绝对值超过 720 rpm（600 rpm 底盘留 20% 余量）或非有限时标记异常。
有限但超量程的值具有可信位置差分时，用同一台电机的位置差分替代后进入 SDK 滤波；
没有可信替代或 SDK 读数非有限，则该 SDK 速度无效。
原始可读 SDK 值仍保留在 DS2，不能把被标记的尖峰拿去拟合前馈；TP 始终使用独立的位置差分速度。
位置差分要求前后样本、八台位置和实际时间间隔有效，异常位置跳变或长间隔会使该速度无效并停止 TP。
滤波与加速度使用实际采样时间，首样本、掉线恢复及超过 200 ms 的间隔后，加速度标为无效。

新版输出使用 DS2 格式（29 列），在原 DSV 27 列之后追加 mode、target_rpm。
mode：0 手控，1 固定电压测试，2 旧版纯前馈，3 A/B 测试，4 FH，5 SP，6 TP，7 比赛自动。
接收脚本兼容旧 DSV，为旧数据补 mode=-1（未知）、target_rpm=0，不推测旧版的转速目标。
主控无法直接连接电脑时，可使用遥控器和 VS Code 转发：

1. 遥控器通过 USB 连接电脑，并与主控保持无线连接。
2. 在 VS Code 打开本工程，确认 VEX 插件选中了遥控器，运行包含底盘采样的 Bartmoss。
3. 使用 Node.js 22 或更新版本执行以下命令；不需要安装额外依赖。

~~~powershell
node scripts/monitor-drive-websocket.mjs 60
~~~

看到数据通道已连接后按 Y 开始回传，测试完成按 Y 停止；脚本仍会在指定时间后结束。
工作区设置启用本机 `127.0.0.1:7071` 的 VEX WebSocket 服务和用户终端。
脚本自动查询当前 VS Code 实例编号，不沿用 Tungsten 脚本中固定的编号。
它只接收设备数据，不向设备发送指令；唯一发送的查询是向插件询问已选设备。
完整采样保存为 `build/drive-websocket-*.csv`，原始输出同时保存为同名 `.log`。
诊断版还会输出 DLM 行，同序号、同时间戳的 LM/LF 角度和电流保存在同名 `-motors.csv`。
列为 sequence,time_ms,LM_deg,LM_current_A,LF_deg,LF_current_A,diagnostic_mask。
FH/SP/TP 模式另输出 DHD 行，接收脚本保存到同名 `-heading.csv`：
sequence,heading_time_ms,heading_deg,target_deg,error_deg,correction_mV,locked,ready,active。
航向数据由主控制循环读取并通过互斥锁发布快照，串口输出仍集中在原来的单一线程。
sequence 关联本次发送的速度行；heading_time_ms 是航向快照的主循环时间，可能与速度采样时间不同，
两个线程的计时器也有启动偏移，不能假设两者严格同时。停车时 active=0，但仍回传误差，
便于比较停车前后的偏转；5 Hz 回传不能还原短于采样间隔的瞬时过程。
SP/TP 模式还输出 DVL 行，保存到同名 `-velocity.csv`：
sequence,velocity_time_ms,left_target_rpm,right_target_rpm,left_error_rpm,right_error_rpm,left_correction_mV,right_correction_mV,active。
sequence 同样关联本次发送的速度行；velocity_time_ms 是最近一次 PID 使用的速度样本时刻，
不保证等于本次发送的速度行时刻。左右 error=对应目标-对应实测速度；active=0 时闭环补偿未生效。
TP 另输出 DTP 行，保存到同名 `-profile.csv`：
sequence,motion_time_ms,elapsed_ms,reference_mm,reference_rpm,distance_mm,target_rpm,remaining_mm,duration_ms,running,phase。
motion_time_ms 为主循环快照时间，sequence 关联本次发送的速度行；同样不假设物理上严格同时。
phase：0 空闲、1 加速、2 匀速、3 减速、4 等待停稳、5 完成、6 取消、7 故障。
reference 为纯规划参考；target_rpm 含位置纠偏和变化率限制，DVL 两侧目标还包含航向修正。
所有记录继续共用 Y 开关、200 ms 回传间隔和同一串口线程，不新增持续打印线程。
v1.1.1 另输出 DSQ，保存到 `-quality.csv`：
sequence,time_ms,left_rejected_mask,right_rejected_mask,left_encoder_valid,right_encoder_valid,left_encoder_rpm,right_encoder_rpm,left_sdk_valid,right_sdk_valid。
异常掩码第 0～3 位分别对应各侧四台电机；右侧第 3 位（8）对应 RBU。
encoder_rpm 是 TP 真正使用的速度，DS2 的 left_rpm/right_rpm 仍为清理后的 SDK 滤波速度。
接收脚本在 TP 模式优先显示同序号的 DSQ 速度；老固件没有该行时显示 --，原始 CSV 仍可接收。
diagnostic_mask 第 0 位表示 LM 有效，第 1 位表示 LF 有效，3 表示两台均有效。
角度为电机减速箱输出轴累计度数，电流单位 A。诊断与 Y 开关、5 Hz 回传及快照互斥锁共用。
可离线用相邻有效记录的角度差 / (6 × 实际时间差秒数) 计算区间平均 rpm，再和同一时间区间的 SDK 转速比较。
比较时排除起步、换向、缺口和无效样本。位置与 SDK 转速来自同一编码器，不能代替独立机械转速测量。
电机角度、电流与其他反馈依次读取，不保证物理上严格同时；本诊断只读取数据，不改变底盘输出。
新版 WebSocket 脚本会单独报告诊断行数；旧版固件没有 DLM 时不能完成角度、电流交叉诊断。
脚本可拼接跨消息的数据行，兼容 DS2、旧 DSV 及对应的 `#web#` 前缀。
连接成功仅表示接通插件，必须收到有效速度样本才能确认实车回传成功。
零字节、其他版本的输出或不完整数据均会明确报错，不作为有效实测。

主控能通过 USB 直接连接电脑时，也可读取其用户串口：

~~~powershell
.\scripts\monitor-drive.ps1 -Port Auto -DurationSeconds 60
~~~

日志保存在 build/drive-monitor-*.csv。包含时间、原始及滤波轮速、加速度、实测电压、
八台单独轮速、手控电压指令和有效标记。左右 mask 为 15 表示四台数据都有效。
acceleration_mask 的第 0、1 位分别表示左、右加速度有效。
command_valid 表示采样全过程处于已发布、未更新的电压指令状态。
主线程在任何底盘输出写入前调用 BeginCommandUpdate，在完成输出后调用 PublishCommand；
采样与输出更新有重叠时标为 0，自动转速测试期间也为 0。该标记不表示各电机在物理上同时采样。
dropped 为串口消费落后时的累计丢样数，序号可用于发现缺失区间。
八台电机依次读取，时间戳采用读取窗口中点。首次接车需核对速度和实测电压的正负号。
辨识时剔除掉线、丢样和指令切换区间，并对原始数据统一做时间对齐和滤波。

include/drive-control.h 提供独立的 motor_feedforward 类，其 Calculate 接口接收目标 rpm 和 rpm/s，
返回 kS*方向 + kV*速度 + kA*加速度 对应的电压，以及有效、饱和标记。
静止目标电压为 0，起步零速时按目标加速度确定静摩擦方向。
左右两侧可使用各自的 feedforward_config，便于接入梯形规划和之后的速度 PID。
模型默认 calibrated=false，实测辨识完成后再填写 kS/kV/kA 并启用。
当前运行程序启用速度采样，并可手动进入前馈测试模式。
基础数学测试使用合成系数，测试模式使用尚待独立验证的实车初值；跟随误差和跨场地重复性待实测。

## 多线程共享数据

主控制循环、速度采样、串口回传和屏幕显示分别运行。
`controller_display_mutex` 保护整包 ControllerInfo，读写双方都在持同一把锁时完成整包复制。
`queue_mutex` 保护采样队列、最新速度快照、指令元数据、更新标记、版本号和回传开关。
锁内只做短时状态访问，不持锁读取电机、刷新屏幕、打印串口或等待。
两把锁不嵌套持有。电机输出仅由主线程执行，设备指针在启动其他线程前初始化，随后不替换。
`phase_mutex` 另保护比赛回调的阶段与代次；只在锁内复制数据，不持锁执行路线或设备操作。
完整软件快照不代表八台电机的物理采样时刻完全一致；采样与指令对应仍需查看时间和有效标记。

## 测试和文件

Linux / WSL 可运行独立控制逻辑测试：

~~~sh
mkdir -p build
g++ -std=c++11 -Wall -Wextra -Werror -pedantic -I include tests/driver-control-test.cpp -o build/driver-control-test
./build/driver-control-test
g++ -std=c++11 -Wall -Wextra -Werror -pedantic -I include tests/drive-control-test.cpp -o build/drive-control-test
./build/drive-control-test
g++ -std=c++11 -Wall -Wextra -Werror -pedantic -I include tests/voltage-test-test.cpp -o build/voltage-test-test
./build/voltage-test-test
g++ -std=c++11 -Wall -Wextra -Werror -pedantic -I include tests/feedforward-test-test.cpp -o build/feedforward-test-test
./build/feedforward-test-test
g++ -std=c++11 -Wall -Wextra -Werror -pedantic -I include tests/heading-control-test.cpp -o build/heading-control-test
./build/heading-control-test
g++ -std=c++11 -Wall -Wextra -Werror -pedantic -I include tests/velocity-control-test.cpp -o build/velocity-control-test
./build/velocity-control-test
g++ -std=c++11 -Wall -Wextra -Werror -pedantic -I include tests/trapezoid-profile-test.cpp -o build/trapezoid-profile-test
./build/trapezoid-profile-test
g++ -std=c++11 -Wall -Wextra -Werror -pedantic -I include tests/trapezoid-motion-test.cpp -o build/trapezoid-motion-test
./build/trapezoid-motion-test
g++ -std=c++11 -Wall -Wextra -Werror -pedantic -I include tests/drive-straight-test.cpp -o build/drive-straight-test
./build/drive-straight-test
g++ -std=c++11 -Wall -Wextra -Werror -pedantic -I include tests/program-mode-test.cpp -o build/program-mode-test
./build/program-mode-test
g++ -std=c++11 -Wall -Wextra -Werror -pedantic -I include src/autonomous.cpp tests/autonomous-test.cpp -o build/autonomous-test
./build/autonomous-test
g++ -std=c++11 -Wall -Wextra -Werror -pedantic -I include src/autonomous.cpp src/route.cpp tests/empty-autonomous-test.cpp -o build/empty-autonomous-test
./build/empty-autonomous-test
g++ -std=c++11 -Wall -Wextra -Werror -pedantic -pthread -I tests/competition-mocks -I include src/competition-control.cpp tests/competition-control-test.cpp -o build/competition-control-test
./build/competition-control-test
node tests/drive-websocket-test.mjs
g++ -std=c++11 -O2 -Wall -Wextra -Werror -pedantic -pthread -I tests/sampling-mocks -I include src/drive-sampling.cpp tests/drive-sampling-test.cpp -o build/drive-sampling-test
./build/drive-sampling-test
~~~

测试覆盖 40,401 种整数摇杆组合，以及手臂即时正反向电压、长按恒定输出、
松手及同时按键、掉线、禁用、夹爪独立运行和气动按下沿。
测试及编译通过不能代替实车验证。

- include/control-config.h：设备、按键、电压及控制周期参数。
- include/driver-control-logic.h：底盘混控、手臂与夹爪电压及气动状态。
- src/robot-config.cpp：端口校验、输入采样、双电机输出和停止保持。
- src/main.cpp：运行条件、控制循环、独立显示线程与遥测。
- include/drive-control.h：速度采样计算和独立前馈模型。
- src/drive-sampling.cpp：采样线程、缓冲队列和 DSV 遥测。
- include/voltage-test.h：电压 / 前馈测试、长按切换、档位、限时停止和输入接管。
- include/heading-control.h：前馈测试的跨换向航向参考、转向修正及限幅。
- include/velocity-control.h：独立速度 PID、积分防饱和和速度反馈接口。
- include/drive-velocity.h：SP/TP 共用的前馈、电压速度环和反馈时效检查。
- include/trapezoid-profile.h：来自同事分支的纯梯形数学规划和速度变化率限制。
- include/trapezoid-motion.h：直线参考跟随、平均位置纠偏、终点停稳与超时。
- include/drive-straight.h：与测试按键无关的公共非阻塞直行控制器。
- src/competition-control.cpp：VEX 比赛回调和互斥保护的阶段通知。
- src/autonomous.cpp、src/route.cpp：比赛生命周期与独立路线。
- include/program-mode.h：测试/比赛权限及场控状态互锁。
- scripts/、vex/、makefile、.vscode/：构建、只读监测及工程配置。

切换前的完整 PID 工程和校验清单保存在本地 build/backups/ 中。
构建产物、本地备份、参考代码和日志不随程序提交。
