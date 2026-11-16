# Override 项目交接

- 当前底层基线为2.0，程序与VERSION均为2.0.0，发布标签core-v2.0.0；已有v1.0.0/v1.1.0-voltage等历史标签保留，不移动覆盖。

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

- 底层2.0默认BARTMOSS_HIGH_SPEED=1，保留独立HighSpeed构建；显式Test构建置0以使用原低速测试参数。高速档位在include/high-speed-config.h，实例局部覆盖加减速度及位置纠偏倍率，H3保留实车基线，H4为同速同加减速、位置纠偏乘二的候选；勿混同之前H4的400rpm/s数据。
- 当前启动菜单仿Tungsten实际映射：初始化校准期间锁住选择和运动，之后A自动、B手动、新增X测试；选择后松开A/B/X并回中摇杆才进入。本地自动执行一次src/route.cpp，结束停车等3000ms再回手动（已修正上一版立即返回的差异），场控自动不走此倒计时。B或摇杆可立即中断；手动/测试停车长按B一秒回菜单。开机/重连已按住键不能直接选择；选择用X不会切气动，进入手动/测试后X仍为原气动键。详见docs/STARTUP_AUDIT.md，不能笼统宣称与参考逐行完全一致或实车已验证。
- HIGH_SPEED_ROUTE_DEBUG已移除，自动由启动菜单选择。X测试中HIGH_SPEED_CORNER_TEST=true提供STR/TURN/CNR三项，默认CNR；false恢复H1～H4。原测试配置保留。只改src/route.cpp的RunRoute动作表即可调路线，默认DriveCorner(500,90,500)；直行默认330 rpm、原地相对转向140 rpm、Wait单位毫秒且非阻塞。禁止在RunRoutine中直接vex::wait或另开路线线程。
- 除独立Calibration外，Default/HighSpeed/Test/Competition构建连接场控时都由SessionPermissions统一按比赛回调及使能分配权限，忽略菜单；自动不依赖手柄，场控连接时关闭回传。场控接入/拔出会先取消原自动周期，不能继续另一来源的半条路线。拔场控回菜单；Calibration保持原权限。Y可在本地菜单先开LOG，运动权限仍关闭。
- 高档调试继续2 Hz精简回传，DRA记录run/step/等待/结果并保存-route.csv，同时保留动作原有协议；自动结束回手动后继续发送终态DRA，不把旧运动曲线混入新的X测试。显示H档仅用于H1～H4，三项测试中的STR不误显示H1。
- include/motion-presets.h 是路线与高速测试的共用参数来源：HighSpeedStraightControl采用H3的320/320 rpm/s、原位置增益；HighSpeedCornerControl采用直线及预测轮加减速预算320/320 rpm/s、弯内180 rpm、入弯偏好130 rpm、弯内加速120 rpm/s、出弯加速240 rpm/s。主循环正式自动与路线调试均显式接入这些配置；普通低速测试保留原参数。新原地转向默认上限140 rpm及复杂组合路线尚未实车验证。
- 路线新增PushFor(毫秒,电压V=4.0)机械定位动作：单次0～5000ms、0跳过，固定同向电压，无位置/速度/航向PID，不因接触停转提前结束。到时撤电压、轮速低于5rpm保持100ms后继续；撤电压后1500ms未停稳则全路线故障停止。反馈异常/取消/场控禁用终止输出。只在main写电机；Push状态为autonomous_motion=4、DPS及-push.csv记录。默认路线仅注释示例未启用；不检测是否卡入，不自动重置坐标，默认4V尚待实车确认。

- 已知下载兼容问题（2026-11-16，用户已确认 VS Code 下载恢复）：`.vscode/vex_project_settings.json` 的 `project.description` 为 `2026-2027 Bartmoss v1.6.0-tests` 时，vexcom 曾以 3221225477 / 0xC0000005 崩溃；改为 `2026-2027 Bartmoss v1.7.0-highspeed` 后，同一二进制和原工具可下载，用户最终确认 VS Code 按钮下载成功。修改该描述字段后要验证实际 VS Code 下载兼容性，避免复用已知失败字符串。再次遇到同码时先核对描述参数并做受控对照；不要未经验证归因于串口占用、压缩、字符串长度或下载器版本。原下载器与用户终端设置已恢复，内部崩溃机制尚未查证。详见 LAB_NOTES 的最终结论。
