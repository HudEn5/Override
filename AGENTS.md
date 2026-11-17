# Override 项目交接

- 最新2.1.0-point1：route::MoveToPoint(x,y,直行rpm=540,转向rpm=540)通过ObservePose快照先相对转向再重算距离直行，点位终点复用5mm容差，定位无效/过期200ms或终点超差停车；不增加循环重试。route::Swing(角度,pivot_side::left/right,rpm=540)复用turn_controller，单侧角速度换算减半、固定侧输出始终0并brake。默认路线不变，仅注释示例；20头文件不增加，无新线程，唯一main写电机。新增组合/坐标零点/失效/左右正反模型，实车待验。

- 最新本地候选2.1.0-turn3：依据旧实车尾段数据调整高速原地转向，仅改performance-config.h的角加速1200/角减速1000度每秒²、制动预留100ms、角速度滤波20ms、低速补偿阈值48度每秒、最低60/30rpm。原0.8度/零速门槛、自动0ms与X单项250ms确认不变；低速Test、直行、圆角及位姿不变。144组完整模型及144组SDK缓存模型通过，候选尚未下载/实车。前一份含位姿的本地基线f43a4d8；GitHub main仍92eba1f。

- main基线2.0.6-route3已推送并核对远端92eba1f0ec2c7923323be5330e8f5db8ef05f707。当前开发分支feat/odometry，2.1.0-odom1：位姿仅旁路观察，原动作控制与参数不变；include由30合并为20个文件，无兼容空壳。位姿类型复用drive-control.h，实现src/odometry.cpp，主循环唯一写者，经runtime_info mutex发布，主控XYH及按需DPO回传；未实车验证、未加入MoveToPoint。

- 最新2.0.6-route3：用户撤回连续圆角，要求严格直行1500→原地逆时针90→直行1500，零速达标即交接。已恢复route2原地动作表并删除临时圆角半径/切点草稿。AUTO_SETTLE_MS=0，5mm/0.8度/两侧5rpm/转向角速度2.5度每秒门槛保留，达到门槛当周期推进，无额外停稳计时；X单项仍250ms。四组实际路线模型验证两个交接点即时推进，108组自动转向模型DONE、最大终点0.798度，尚待实车。不能把目标/电压0当成实测轮速0；2Hz旧数据不能精确测定停车瞬间。
- 当前版本2.0.5-route2：主循环的Autonomous用AutoStraightControl/AutoTurnControl，AUTO_SETTLE_MS=100，仅自动直行/原地转向连续停稳确认从250缩到100ms；X用原Route配置250ms，圆角250ms及本地结束3秒回手动流程保持原样。5mm/0.8度/5rpm/转向2.5度每秒门槛保留，仍为停车原地转向；改AUTO_SETTLE_MS=250可恢复route1确认时间。旧源码和构建产物在build/backups/before-fast-handoff-20261117；候选尚待实车。
- 当前默认正式路线为1500mm直行→原地逆时针90度→1500mm直行，菜单A执行整条，无额外Wait；X仍为原单项测试。沿用540电机rpm上限、直行400/400rpm/s和turn2快速补角，保留每段停稳；整条新路线未实车。原圆角源码/构建产物保存在build/backups/before-stop-turn-route-20261117，用户自行从VS Code下载。
- 当前底层基线为2.0，程序与VERSION均为2.0.0，发布标签core-v2.0.0；已有v1.0.0/v1.1.0-voltage等历史标签保留，不移动覆盖。
- 已发布基线为2.0.6-route3。用户确认轮端450rpm，30:36传动对应代码540电机rpm；include/performance-config.h的EXPERIMENTAL_FAST_ROUTE=true启用实验副本。基线performance-config.h、低速测试和H1～H4保留。当前路线/X三项默认直线、圆角单侧及原地转向均540电机rpm，弯内180及入弯130/曲率预测保留；不能把540解释为弯内中心恒速或短段必达速度。STR测试1500mm，TURN仍90度。上一版turn1原地90度已有3次完整DONE，turn2八次90度已DONE，但用户报告单次物理偏差，其他角度/负载待验，下载仍由用户执行。
- 原地转向新增角速度梯形/短弯三角形参考，复用trapezoid-motion.h；按69.85mm轮径、1.2传动比、325mm轮距换算左右轮速，角度/角速度跟踪和原速度环纠偏。实验参数角加速900、角减速1200度/秒²、制动预留120ms、P3/D0.10、末端最低30/24rpm，原0.8度及停稳窗口不变。profile_enabled=true时旧acceleration_rpm_s不再控制斜率；TURN_ANGULAR_PROFILE=false可单独切回用户最后P2/D0.25/640rpm/s、540rpm上限方案。只在main更新/写电机，无新线程；DTA和-turn-profile.csv记录参考及阶段，原DTN兼容、mutex快照与2Hz开关回传保留。详见docs/TURN_TUNING.md；改动前源码及当时构建产物在build/backups/before-turn-profile-20261116。
- turn2仅在角速度参考结束后（phase4）启用profile_finish_kP=6、最低补角50/30rpm、最低转速权重阈值minimum_rate_deg_s=32；角加减速/制动预留/0.8度与250ms停稳保留，前段同反馈输出逐周期相同。TURN_FAST_FINISH=false可单独恢复上一条turn1补角参数，TURN_ANGULAR_PROFILE=false仍恢复原PD方案。旧源码及构建产物在build/backups/before-turn-finish-20261116。108组模型turn1→turn2均DONE、平均1556→1345ms，标准90度1900→1520ms，最后6度到角度窗口360→200ms；这些不是实车成绩。

- 用户使用台式机开发，也携带 Windows 64 位笔记本去实验室，通过笔记本 USB 连接 VEX V5 主控。
- 唯一代码仓库为 `https://github.com/HudEn5/Override.git`，日常同步分支为 `main`。
- 本仓库根目录是 Bartmoss 八电机底盘、双臂电压控制工程。此前迁移包内的四电机 `test` 工程是另一套程序；不要将两者互相覆盖。
- 开始工作时阅读 `DEVICE_SYNC.md`、`LAB_NOTES.md` 和 `README.md`，检查当前工作目录、Git 根目录、分支、origin 和未提交修改。
- 同一 GPT 账号不能代替 Git 同步，也不要假设另一台设备的会话、磁盘文件或 USB 设备在当前会话中可用。
- 在干净工作区开始一轮工作前，运行 `scripts/sync-github.ps1` 或 `git pull --ff-only origin main`。有本地修改或历史分叉时，保留工作并说明情况；不要自动丢弃、强推、重置或覆盖。
- 笔记本使用 `scripts/configure-device.ps1 -Role Laptop`，实验期间不后台更新源码。台式机使用 `-Role Desktop`，自动同步仅拉取，不自动提交或推送。
- 当前 SDK 为 `V5_20240802_15_00_00`，程序名 Bartmoss，下载槽位 1。构建使用 `scripts/build.ps1 -Rebuild`；可用 `-EnvironmentRoot` 复用之前迁移包中的工具链和 SDK。
- 日常 TEST 只保留直行/转向/圆角：上/下选择、右/左选择方向、A 开始、B 取消、Y 回传；参数在 `include/test-config.h`。VTEST/FH/SP 仅在独立 Calibration 构建启用，不恢复旧 A/B 或 TP 模式入口。
- 用户要求调试并上传后，核对 diff、运行相应检查、更新 `LAB_NOTES.md`，只提交项目文件，再正常推送至 `origin/main`。不提交 SDK、扩展、驱动、令牌、`.env`、本机路径、构建产物或串口日志。
- Git 同步不触发 VEX 下载或运行；这些操作按用户当次指令执行。报告当前提交号、编译结果和实际完成的 USB/实车验证，区分未验证项目。
- 用户明确要求由本人通过 VS Code 下载程序；助手只修改、编译和采集，不直接下载，除非用户之后明确改变这一要求。

- 底层2.0默认BARTMOSS_HIGH_SPEED=1，保留独立HighSpeed构建；显式Test构建置0以使用原低速测试参数。高速档位在include/test-config.h，实例局部覆盖加减速度及位置纠偏倍率，H3保留实车基线，H4为同速同加减速、位置纠偏乘二的候选；勿混同之前H4的400rpm/s数据。
- 当前启动菜单仿Tungsten实际映射：初始化校准期间锁住选择和运动，之后A自动、B手动、新增X测试；选择后松开A/B/X并回中摇杆才进入。本地自动执行一次src/route.cpp，结束停车等3000ms再回手动（已修正上一版立即返回的差异），场控自动不走此倒计时。B或摇杆可立即中断；手动/测试停车长按B一秒回菜单。开机/重连已按住键不能直接选择；选择用X不会切气动，进入手动/测试后X仍为原气动键。详见docs/STARTUP_AUDIT.md，不能笼统宣称与参考逐行完全一致或实车已验证。
- HIGH_SPEED_ROUTE_DEBUG已移除，自动由启动菜单选择。X测试中HIGH_SPEED_CORNER_TEST=true提供STR/TURN/CNR三项，默认CNR；false恢复H1～H4。原测试配置保留。只改src/route.cpp的RunRoute动作表即可调路线，默认DriveStraight(1500)→Turn(90)→DriveStraight(1500)，当前实验档直行/原地上限均540rpm；Wait单位毫秒且非阻塞。禁止在RunRoutine中直接vex::wait或另开路线线程。
- 除独立Calibration外，Default/HighSpeed/Test/Competition构建连接场控时都由SessionPermissions统一按比赛回调及使能分配权限，忽略菜单；自动不依赖手柄，场控连接时关闭回传。场控接入/拔出会先取消原自动周期，不能继续另一来源的半条路线。拔场控回菜单；Calibration保持原权限。Y可在本地菜单先开LOG，运动权限仍关闭。
- 高档调试继续2 Hz精简回传，DRA记录run/step/等待/结果并保存-route.csv，同时保留动作原有协议；自动结束回手动后继续发送终态DRA，不把旧运动曲线混入新的X测试。显示H档仅用于H1～H4，三项测试中的STR不误显示H1。
- include/performance-config.h 是路线与高速测试的共用参数来源：HighSpeedStraightControl采用H3的320/320 rpm/s、原位置增益；HighSpeedCornerControl采用直线及预测轮加减速预算320/320 rpm/s、弯内180 rpm、入弯偏好130 rpm、弯内加速120 rpm/s、出弯加速240 rpm/s。主循环正式自动与路线调试均显式接入这些配置；普通低速测试保留原参数。新原地转向默认上限140 rpm及复杂组合路线尚未实车验证。
- 路线新增PushFor(毫秒,电压V=4.0)机械定位动作：单次0～5000ms、0跳过，固定同向电压，无位置/速度/航向PID，不因接触停转提前结束。到时撤电压、轮速低于5rpm保持100ms后继续；撤电压后1500ms未停稳则全路线故障停止。反馈异常/取消/场控禁用终止输出。只在main写电机；Push状态为autonomous_motion=4、DPS及-push.csv记录。默认路线仅注释示例未启用；不检测是否卡入，不自动重置坐标，默认4V尚待实车确认。

- 已知下载兼容问题（2026-11-16，用户已确认 VS Code 下载恢复）：`.vscode/vex_project_settings.json` 的 `project.description` 为 `2026-2027 Bartmoss v1.6.0-tests` 时，vexcom 曾以 3221225477 / 0xC0000005 崩溃；改为 `2026-2027 Bartmoss v1.7.0-highspeed` 后，同一二进制和原工具可下载，用户最终确认 VS Code 按钮下载成功。修改该描述字段后要验证实际 VS Code 下载兼容性，避免复用已知失败字符串。再次遇到同码时先核对描述参数并做受控对照；不要未经验证归因于串口占用、压缩、字符串长度或下载器版本。原下载器与用户终端设置已恢复，内部崩溃机制尚未查证。详见 LAB_NOTES 的最终结论。
