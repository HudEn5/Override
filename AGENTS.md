# Override 项目交接

## 当前状态

- 当前开发版本2.1.0-api1，分支feat/odometry，前一程序提交2249f44；main发布基线为92eba1f（2.0.6-route3），core-v2.0.0为历史标签。不要自动切回main或用远端覆盖开发分支。
- 原路线为1500mm直行→原地逆时针90°→1500mm直行；新MoveToPoint与Swing只有注释示例，尚待实车验证。20个头文件、11个实现文件，避免继续拆分小配置文件。
- api1阶段35个C++程序与五种SDK构建已通过；整理复核出现autonomous-turn/startup-route间歇断言失败，尚未定位，详见LAB_NOTES，不能声称本轮全套通过。软件模型不等于实车验收。开发记录入口LAB_NOTES.md，阶段原始记录在docs/history。

## 开始工作与同步

- 阅读README.md、DEVICE_SYNC.md、LAB_NOTES.md，核对Git根目录、分支、origin与未提交修改。
- 唯一仓库https://github.com/HudEn5/Override.git；日常发布至main，但当前开发分支保留到用户要求发布。仅干净main允许快进同步；不自动切分支、覆盖、重置或强推。
- 保留用户编辑器改动；.vex-device.json、SDK、工具链、本机路径、令牌、build产物与串口日志不得提交。reference/Tungsten仅本地参考，不随程序发布。
- 台式机与笔记本以Git同步；同一账号不代表本地文件相同。笔记本Role Laptop防止实验时后台更新源码。

## 架构与行为约束

- main是唯一电机写入者；路线状态机非阻塞，禁止在RunRoutine中调用vex::wait或创建另一个路线线程。
- 采样、运行快照和比赛回调使用mutex；新状态放进已有整包快照，锁内只复制，屏幕/串口操作在锁外。
- 启动校准完成后A自动、B手动、X测试，选择后松键并回中；B/摇杆中断本地自动，手动/测试停车长按B一秒回菜单。菜单X不触发气动。本地自动结束停车3秒再回手动。
- Default/HighSpeed/Test/Competition连接场控时忽略菜单，由比赛使能/回调分配权限；自动不依赖手柄。禁用撤输出，来源切换取消原动作；拔场控回菜单。Calibration保持独立许可。
- 日常X测试保留STR/TURN/CNR；test-config.h可切H1～H4。标定VTEST/FH/SP仅Calibration启用。
- Y按需LOG，默认OFF，场控连接时关闭；高速最多2Hz，普通/标定最多5Hz。禁止为调试增加遥控器高频打印。

## 参数与新动作

- 端口/方向/硬件与前馈在control-config.h；运动实验参数在performance-config.h；测试档位在test-config.h。现有2.75英寸轮、30:36传动、325mm轮距、IMU4号口；540电机rpm为450轮端rpm。
- 当前原地角加速1200、角减速1000度/秒²，预留100ms，角速度滤波20ms，低速阈值48度/秒，最低60/30rpm。原地0.8°、轮速5rpm、角速度2.5度/秒；自动确认0ms，X单项250ms。详见TURN_TUNING。
- MoveToPoint使用主循环位姿快照先相对转向再直行；全局X前/Y左、mm/逆时针度，5mm终点容差，定位失效/过期200ms或终点超差停车。Swing固定侧主动brake，活动侧角速度/rpm换算为双侧的一半。两者沿用当前参数、未实车整定。
- 位姿只由main更新；SetPose不重置硬件传感器。反馈丢失/跳变后锁存需重置，不自动冒充恢复。原距离动作不依赖位姿，PushFor不自动归零。

## 构建、验证与下载

- SDK V5_20240802_15_00_00，Bartmoss槽位1。scripts/build.ps1 -Rebuild，支持Default/Test/Competition/Calibration/HighSpeed与EnvironmentRoot；自动保存日志到build/logs。
- 运行适当的宿主机/协议检查与SDK构建，记录实际通过和未验证项。不提交构建产物；提交前核对diff并更新LAB_NOTES。用户要求上传时才推送，不因整理工作区自动发布。
- 用户明确自行从VS Code下载；助手不得自动下载或运行机器人。
- 已验证下载description必须保留2026-2027 Bartmoss v1.7.0-highspeed。旧v1.6.0-tests描述曾触发vexcom 3221225477/0xC0000005，后改描述用户确认下载恢复；内部机制未查证，不凭猜测归因串口或工具链。
- build保留活动产物、logs、data、backups、archive；临时试验目录不散放根目录。原始采集与未核实备份不随意删除；历史记录中的旧build路径可能已归档，按build/logs/workspace-cleanup.json查迁移清单。

- api1统一内部命名/单位/格式、基础动作状态和路线结果，取消立即归零，ResetControl与ConfigurePID区分反馈重建；PointControl独立5mm/200ms，点位按距离收紧末端误差预算。路线函数和协议编号保留，不增加头文件。新api-consistency-test覆盖对应行为；历史间歇断言原因仍未定位，不能自动归因于本次修正。
