# Override 项目交接

## 当前状态

- LF已由用户从12移到16号口，ratio6_1/true不变；不得恢复成12。其余左侧1/11/13与右侧6/5/9/10保持。
- readiness1：测试/H档先检查MOTOR OFF/IMU WAIT/POSITION ERR/SPEED ERR再判断MOVING；本地A自动额外检查位姿valid/needs_reset/200ms新鲜度。POSE RESET:RESTART要求回已知起点再重启，不自动清坐标；X独立测试和场控许可不受本地位姿门控影响。auto_status沿原mutex快照发布。RouteTurnControl关闭实验档恢复240rpm/s，当前高速配置不变。

- 当前开发版2.2.0-dev.1，本次同步目标为远端main和feat/odometry。机构开发前基线见v2.1.0-beta.2（3918b74）。当前开发分支feat/odometry。同步仅快进，个人编辑器配置保留在本地。
- 用户授权吸收合并RAR后，当前TEST_ROUTE=10：12V闭爪500ms并等待完成，再等待500ms，LiftTo100度与直行300mm并行，12V开爪300ms，末端等所有机构完成。0～9号原路线保留，路线7仍为MoveToPoint修正复测；历史8.28mm误差及Swing单轮DONE不代表新机构验收。
- 手控底盘满杆12V，自动540电机rpm上限，23个头文件/11个实现文件。所有route动作显式timeout，A/B/X和场控权限不变；不要自动下载或启动机器人。
- 最新integrity1核查：常驻41项及15种缺timeout检查完整复跑通过，五种SDK构建通过，机构调度/后端ASan+UBSan通过；首轮宿主机GCC11发生内部崩溃，原命令复查和完整复跑通过但根因未定位。历史间歇断言/旧Clang崩溃记录同样保留。预发布不代表完整实车验收，日志入口LAB_NOTES.md，历史阶段参数勿当现用配置。


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

- 固定1500/90/1500基线测试在route-entry-test中定义；同文件另一编译实例CURRENT_ROUTE直接链接用户src/route.cpp检查启动/禁用。不要再把基线三段断言套在用户单条MoveToPoint路线上。用户已授权在route.cpp加入小范围高速路线；TEST_ROUTE=0保留600/600/400/300，1～6分别为往返、左L、右L、小正方形、原地转角、三点闭合。不得未经要求切换默认路线或改动底层参数。

- 用户明确要求直行与转向职责分开：MoveToPoint仍先原地转向、再直行；直行保持固定航向，不按终点横向误差动态转向。已撤回point2的GuidePoint/SetPointSteering、位姿投影进度及相应配置。600/600高速终点偏移问题尚未修复，不得声称撤回后已达标。

- ramp1按用户要求仅放缓路线直行斜率：ROUTE_STRAIGHT_ACCELERATION/DECELERATION=240/240电机rpm/s，原400/400；最高540电机/450轮端rpm保留。原地转向、低速/标定、H1～H4、圆角、速度PID/FF/航向保持增益及终点判据不改；case6与用户Wait保持。模型不能证明实车5mm达标，须复测。

- ramp2针对case6第三次转向尾段近静止：PointControl.turn_finish_speed_scale=1.5，将点位按容差缩放的补角下限提高50%，最大不超过普通Turn最低补角配置；设1恢复旧行为。角度窗口/零速门槛/5mm容差不改，仍先转后直，无横向转向，无新增Wait。576组短远点位/方向/响应/阻力模型通过，普通转向配置不变，实车待验。

- 用户明确要求多弯全程降速衔接，禁止替换原Turn；已移除RollingSquare专用接口，路线4用通用DriveTurnDrive显式编排，ROLLING_ROUTE=true连续/false原零速。DriveTurnDrive(before,angle,after,turn_mode::stop/rolling,entry_mm_s=150,maxrpm=540)默认stop；相邻rolling组合自动合并，后续before=0沿用上项after，双方非零相加。此授权不改变其他Turn/MoveToPoint语义；移动转向有额外位移，不保证全局5mm闭合。

- structure1仅整理控制器边界：drive-rolling.h独立rolling_controller/rolling_info，drive-corner.h保留旧圆角与预测；不用布尔标记混选快照。info.motion为唯一调度选择源，corner=3/rolling=5，Busy与OwnershipValid防止冲突；CancelCorner/CancelRolling分别取消，全局结束用CancelAll。新DRL/rolling.csv的segment_remaining_mm为当前直段，旧DCR/corner.csv仍为整程余量，采集脚本兼容历史记录。运动参数、Turn与路线写法保持，本次不自动下载/推送。

- chain1连续块最多16弯，固定容量且无堆分配；启动前验证全部后续参数，超限ROLL TOO LONG。每弯独立方向/上限/入角速度，从末端反推减速预算并正向限制短段加速；连续块内不重建PID/等待停车，仅末端验收。最后after必须>距离容差，否则ROLL END DIST；前/中间距离可0。Turn/Wait/stop或独立动作打断连续块并先停稳。DRA完成后按rolling_span推进命令数，内部进度用DRL计数。当前21个头文件、11个实现文件，无形状专用函数；实车待验证。

- timeout1所有九种route工厂最后必填timeout_ms，移除全部前置默认参数，漏参必须编译失败；command构造也必填预算。预算必须有限>0，普通组合共享整项总计时，rolling每项从自身before起到turn/after结束，上一after完成才开始下一项before，最后一项包含停稳。超时TIMEOUT立即撤输出停止路线，后续不执行；内部控制器保护仍可更早失败。路线4每项显式5000ms，原参数不改，新预算待实车。宿主机入口新增route-timeout及9项漏参编译检查，总38程序。

- 用户要求先实车测试MoveToPoint和Swing，路线7保留MoveToPoint(300,300,400,300,10000)，本轮实测POINT ERR（推算8.28mm超5mm）；左固定Swing已实测DONE（1524ms，停后约0.2度偏差，用户观察左轮未动）；右固定原配置1513ms/停后约0.54度，swing1复测1273ms/停后约0.30度；左固定新参数复测亦DONE（1273ms，停后约0.51度）；当前TEST_ROUTE保持8，左右各单轮新参数验证完成，重复性待验。新增8左固定+90/9右固定-90的单次Swing，均300rpm/5000ms；0～6路线原样保留。每次单动作验证，先点位后两侧Swing，用户自行下载，重复搬车复位需重启建立原点，勿把推算坐标当实测误差。

- swing1用户要求整定Swing慢尾段，仅route::action::swing取config::SwingControl(turn_settings)：位置反馈尺度4（kP3→12、finish6→24）、角速度阻尼1.4（原0.1），原规划/最低轮速/0.8度精度及timeout不改。Turn/MTP/rolling不经过该配置；drive-turn.h源码保持。576配对模型1832→1761ms、最后6度457→417ms，有17组慢超过20ms；36缓存模型通过，实车待复测，不承诺减半。右侧新参数已单轮DONE并缩短240ms，左侧新参数亦DONE并缩短251ms，当前TEST_ROUTE保持8；左右各单轮，尚非重复性验收。

- point3用户要求修MoveToPoint。转后按最新位姿复核目标方位，必要时在facing阶段重新原地对准，PointControl.max_heading_realignments=2（0旧行为对照，>2拒绝），上限后POINT ALIGN停车。直行固定航向/5mm判据/总timeout不改，不增加终点重试/横向转向/坐标改写；授权仅直行前对准。当前TEST_ROUTE=7，仍300/300/400/300/10000；Swing/Turn保持，实车待复测。

- 用户要求手控底盘减半：MANUAL_DRIVE_MAX_VOLTAGE=6000只控制手控混控上限，改回12000恢复；MAX_VOLTAGE=12000仍供自动控制，机构不受影响。半电压不承诺实际半转速，默认构建已生成，用户自行下载。

- mechanism1：StartMechanism/WaitMechanism/DriveStraightWithTrigger预算必填；后台独立计时，路线末端自动join。机构故障/超时/禁用取消全路线，手控接管优先、气动状态延续。模拟器仅tests，无新线程/高频打印；Cancel幂等撤压。mechanism2已在main绑定同事b70b202的真实抬升/夹爪适配器，算法/参数合在mechanisms.h，不整合分支的独立电机线程、Y抬升和自动开日志。lift_start/lift_raised对应0/240电机度，范围0～600，PID100/20/8，上升12V下降2.5V；夹爪open/close均6V/200ms，结束与取消hold。首次有效反馈设一次偏移零点，启动前放机械起点，失效不自动重置；毫米映射未标定。Start接口增加timeout_ms原样接收路线预算，主循环Observe后调度Update，main仍唯一写电机者。默认路线7、LF16、Y回传及手控电压保持；piston自动后端尚未绑定。40宿主机入口及12漏参检查、五种SDK通过，真实机构待上机；未下载/启动/推送。

- mechanism3覆盖上段默认路线约定：用户授权对比RAR择优合并，现TEST_ROUTE=10、0～9保留；新增route::LiftTo(deg,timeout)、ClawOpen/Close(duration_ms,volts,timeout)，参数必填，直接动作非阻塞，仍走同一mechanism::scheduler。电压正数<=12V，方向由Open/Close和CLAW_CLOSE_DIRECTION处理；预设6V不继承直接动作的12V。request.kind/target/volts经快照保存。Observe在Autonomous.Update前，启动失败/运行故障/取消均联动全路线；禁止带回原压缩包忽略返回值或独立写电机线程。主控185/210行显示lift/claw整包快照，Y和无线打印保持。新增direct-mechanism固定组合回归；真实src/route.cpp链接的BARTMOSS_IMPORTED_ROUTE专项已跑通，但不放常驻脚本，后续用户改路线不得强加10号旧参数；常驻current-route只核对启动/禁用。常驻41入口+15漏timeout检查，详情LAB_NOTES。无新增头文件/实现文件/线程，未下载或上传。

- integrity1：scheduler.Start成功受理时同步推进previous_time，Reset拒绝NaN/负时钟；请求按值复制，防止重复使用Info内旧请求引用时被清理覆盖。抬升与夹爪统一为截止时刻已完成可DONE，迟到或截止时仍运行则TIMEOUT撤压；不延长预算，不改控制增益/电压/路线。新增时间回退、快照重放、非法Reset、恰好完成/迟到/未完成边界回归。README/ROUTE_GUIDE/LAB_NOTES首页统一默认路线10与2.2.0-dev.1，历史记录原样保留。下载兼容description仍保留旧v1.7.0字符串，不当作版本错误修改。具体失败及复查日志见LAB_NOTES最新条目，未实车/下载/推送。
