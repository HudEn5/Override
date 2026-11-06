# Bartmoss v1.1.0-voltage · 2026–2027 手动控制

VEX V5 八电机差速底盘手控工程，手臂采用直接电压控制。
沿用参考工程 Tungsten 的设备命名。当前版本用于实机调试，尚未完成整机验证。

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
连接场控后，仅在已启用的手动时段输出；禁用、自动时段或手柄断连时电机停止，气动保持状态。
本工程没有自动动作，场控禁用期间的实际电机行为由 VEX 系统接管。

## 诊断与监测

主控标题为 Bartmoss v1.1.0-voltage，底行显示 VOLTAGE MODE。
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

- include/control-config.h：设备、按键、电压及控制周期参数。
- include/driver-control-logic.h：底盘混控、手臂与夹爪电压及气动状态。
- src/robot-config.cpp：端口校验、输入采样、双电机输出和停止保持。
- src/main.cpp：运行条件、控制循环、独立显示线程与遥测。
- scripts/、vex/、makefile、.vscode/：构建、只读监测及工程配置。

切换前的完整 PID 工程和校验清单保存在本地 build/backups/ 中。
构建产物、本地备份、参考代码和日志不随程序提交。
