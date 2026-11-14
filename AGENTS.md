# Override 项目交接

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
