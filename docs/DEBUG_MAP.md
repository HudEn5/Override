# 抬升、夹爪调参与按键定位

`抬升机构封装` 分支保存机构封装调试快照，基于先保留本地工作、再快进同步到 `origin/main` 的 `d902245`（`2.1.0-beta.1`）后移植的机构增量，程序版本不变。最新底盘仍为[路线7](../src/route.cpp)：`MoveToPoint(300, 300, 400, 300, 10000)`；底盘参数和原 R1/R2、L1/L2、X 操作保留。

## 现在怎样启动抬升

1. 上电前，把抬升放在希望作为零点的起始位置。线程取得第一组有效的左右编码器读数后，分别保存为本次零点，**不需要点击主控屏**，也不向硬件编码器写零。
2. 等待初始化完成，在菜单按 B 进入手动，松开选择键并让摇杆回中。
3. 按一下 Y，自动执行[统一调参入口](../include/mechanisms.h#L12)。按住 Y 不连发；启动、重连或恢复权限时已经按住 Y，也不会直接起步，需先松开再按。
4. R1/R2 保留手动接管；主控屏的 `CANCEL LIFT` 也可取消。屏幕按钮只是可选入口。

菜单 A 启动原底盘自动路线时，也会并行触发一次同一个抬升调用，见[并行开关](../include/mechanisms.h#L25)。两者各自非阻塞运行，抬升 timeout 不会延长底盘动作预算。当前[路线文件](../src/route.cpp)仍没有可编排的Lift步骤接口，不能在动作表里插入抬升步骤。只测抬升时用 B → Y 即可，不必启动底盘。

当前程序首行带 `LIFTPID`。源码修改后须重新编译，再由用户通过 VS Code 下载；编译通过不表示主控已加载。此分支为调试快照，尚无实车证据可以排除抬升尾段抖动，或确认 `Memory Permission Error` 根因已经解决，不作为抬升验收完成。

**自动取零点只记录开机姿态，不会寻找机械最低点。** 每次上电都会重新建立相对零点；编码器掉线或反馈异常后不会擅自重新取零。若要复现相同高度，应在相同机械起始位置重新上电。

## 最常改的数字

抬升和夹爪的参数、封装调用集中在 [mechanisms.h](../include/mechanisms.h)，目前 293 行。状态、PID 算法及线程邮箱在 [mechanism-support.h](../include/mechanism-support.h)。

| 点击定位 | 当前代码 | 修改作用 |
| --- | --- | --- |
| [抬升调参入口](../include/mechanisms.h#L13) | `LiftTo(240, 3000)` | `240` 是当前用户填写的相对开机零点**平均电机角度，单位°**；`3000` 是总 timeout，单位 ms。Y 和 A 共用这里 |
| [A 同时抬升](../include/mechanisms.h#L25) | `LIFT_WITH_ROUTE=true` | `false` 关闭随底盘自动触发，Y 单独抬升入口保留 |
| [抬升 PID](../include/mechanisms.h#L35) | `kP=100, kI=20, kD=8` | 位置误差、积分和速度阻尼；`kI=0` 可关闭积分 |
| [积分上限](../include/mechanisms.h#L36) | `maximum_integral_mV=2000` | 限制积分输出，电压饱和时停止同向累积 |
| [闭环电压上限](../include/mechanisms.h#L37) | 上升 `12000`、下降 `2500` | 单位 mV，分别为 12V、2.5V；与 R1/R2 手动电压分开 |
| [起步助推](../include/mechanisms.h#L43) | `12000mV / 100ms / 1°` | 从静止向上启动时助推一次；任一侧前进 1° 或满 100ms 即转 PID |
| [到位条件](../include/mechanisms.h#L38) | `2° / 8°每秒 / 200ms` | 平均角度误差、两台各自停稳阈值及连续确认时间 |
| [软件行程](../include/mechanisms.h#L34) | `0～600°` | 当前保存的允许范围，目标仍由上面的调用单独指定，尚未实测为机械行程 |
| [打开夹爪](../include/mechanisms.h#L17) | `ClawOpen(200, 1000)` | 打开通电 200ms，总 timeout 1000ms |
| [关闭夹爪](../include/mechanisms.h#L21) | `ClawClose(200, 1000)` | 关闭通电 200ms，总 timeout 1000ms |
| [自动夹爪电压](../include/mechanisms.h#L48) | `voltage_mV=6000` | 自动开合 6V；不改变 L1/L2 手动配置 |
| [线程周期](../include/mechanisms.h#L24) | `LIFT_CONTROL_PERIOD_MS=10` | 目标周期 10ms；实际调度间隔由状态里的 dt 观察 |

`LiftTo`、`LiftHeightTo`、`ClawOpen`、`ClawClose` 都要求显式 timeout；省略参数会编译失败。完成、取消、失权或故障后清零控制输出，不会自动重试。定时夹爪的 `DONE` 只表示通电时长结束，不能证明夹爪物理到位。

## 角度与毫米高度

**目前的 `240` 不是 240mm，也不是抬升关节角度。** 两台电机共轴，控制使用各自减去开机读数后的平均输出轴角度，两台始终收到相同电压。

[毫米入口](../include/mechanisms.h#L110)的流程是：输入高度 → 根据三点实测表插值为目标电机角度 → PID 算电压 → 独立线程刷新两台电机。不使用重力补偿。未提供实测数据时，[标定点数](../include/mechanisms.h#L29)保持 `0`，毫米调用会报 `HEIGHT NOT CAL`，不会猜测比例。

以后提供同一起始零点的三组数据即可：`实际升高 mm / 屏幕平均 LIFT 角度°`，例如零点、一处中间位置、一处较高位置；请提供真实测量值。三点要随高度和角度严格递增，毫米目标只能在标定区间内插值。填好表后，将[调参入口](../include/mechanisms.h#L13)改用 `LiftHeightTo(目标毫米, timeout毫秒)`。

## 封装与线程定位

| 点击定位 | 职责 |
| --- | --- |
| [zero_here](../include/mechanisms.h#L81) | 保存当前两台编码器读数为相对参考；启动专用调用只执行一次 |
| [LiftTo](../include/mechanisms.h#L91) | 接受相对零点的目标电机角度和显式 timeout，忙时拒绝覆盖 |
| [LiftHeightTo](../include/mechanisms.h#L110) | 验证实测三点表，将毫米转换为电机角度 |
| [lift_pid](../include/mechanism-support.h#L36) | 按实际 dt 积分、位置 P 与速度 D，处理积分限幅和饱和 |
| [LiftControlTask](../src/robot-config.cpp#L275) | 单个长期线程采反馈、自动取零、处理手动接管、计算并唯一写两台抬升电机 |
| [Y 和 A 的调用接入](../src/main.cpp#L243) | Y 单次请求；A 自动首动作启动时提交同一抬升请求 |
| [ClawOpen / ClawClose](../include/mechanisms.h#L245) | 主循环执行的定时开合，屏幕按钮调用上面的统一调参入口 |

旧 `ARM SKEW` 左右角度差保护已移除，不会因为原始 L/R 相差旧 10°而停止。仍保留逐台行程边界、各编码器有效性、更新时钟、单编码器无进展、timeout 和权限取消；任一侧触及方向边界时两台一起停。PID 与输出线程不能替代这些反馈检查。

## 按键与设备端口

按键本身没有 V5 智能端口，下表为代码配置的受控设备；实物接线尚未在本轮核验。

| 输入 | 功能 | 主控设备端口与定位 |
| --- | --- | --- |
| 左摇杆 Axis3 | 前进/后退 | 左 12/1/11/13；右 6/5/9/10，[底盘接线](../include/control-config.h#L46) |
| 右摇杆 Axis1 | 转向；取消本地底盘自动/测试并接管 | 同上八台底盘电机 |
| R1 | 按住手动上升 12V；接管自动抬升 | 左臂 3、右臂 2，[机构接线](../include/control-config.h#L59) |
| R2 | 按住手动下降 4V；松开 hold | 左臂 3、右臂 2，[手动电压](../include/control-config.h#L67) |
| Y | 本地手动/测试中，按一下执行抬升调参入口 | 左臂 3、右臂 2，[Y 接入](../src/main.cpp#L243) |
| L1 | 按住关闭电机夹爪 6V；松开 hold | 智能口 15，[夹爪配置](../include/control-config.h#L61) |
| L2 | 按住打开电机夹爪 6V；松开 hold | 智能口 15 |
| X | 菜单选 TEST；进入手动/测试后，按一次切换气动状态 | 三线口 H，[气动配置](../include/control-config.h#L63)；菜单选择不切阀 |
| A | 菜单选 AUTO，原底盘路线与抬升并行；TEST 中仍启动底盘单项 | 底盘八电机，AUTO 另加臂 3/2 |
| B | 菜单选 MANUAL；取消本地自动/测试；停车长按 1s 回菜单 | 模式/权限，无独立设备端口 |
| 上/下 | TEST 中选项目/档位 | 无独立设备端口 |
| 右/左 | TEST 中选方向 | 无独立设备端口 |

如果指原 X 按键，则是 H 口气动的单次切换；L1/L2 是按住控制 15 号电机夹爪。本轮保留这两套操作，尚未确认用户报告无反应的是哪一套，也不能仅凭未收到数据判断夹爪故障已解决。

Y 在非 Calibration 的本地模式改为抬升后，LOG 自动开启，场控连接时仍关闭；Calibration 的 Y 保持原回传开关。回传频率仍为高速最多 2Hz、其他最多 5Hz，不能把回传间隔当成 PID 刷新周期。

## 软件检查与实车限制

- 此前机构接入阶段，Default、Test、Competition、Calibration、HighSpeed 五种 SDK 构建通过；抬升与夹爪测试源通过 V5 C++11 的 `-Wall -Wextra -Werror -pedantic -fsyntax-only` 检查。当前分支发布时的实际检查另见[实验记录](../LAB_NOTES.md)。
- C++ 测试源尚未在宿主机运行；语法检查不表示断言已执行。
- 实际解析器和仓库协议测试在 V8 内存中执行 91 项断言通过；未运行 Node 文件/WebSocket 集成和可选历史磁盘日志回归。最终构建结果见[实验记录](../LAB_NOTES.md)。
- 既有软件检查不能排除尾段抖动或证明 `Memory Permission Error` 已修复。实际抬升幅度、负载下 PID、线程周期和夹爪动作仍待实车核验。
