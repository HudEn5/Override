我现在使用 Windows 64 位笔记本，要去实验室通过本机 USB 测试 VEX V5 小车。请直接帮我准备和维护本机开发环境。

唯一代码仓库是 https://github.com/HudEn5/Override.git ，分支 main，程序名 Bartmoss，是八电机底盘和双臂电压控制工程。台式机的对应仓库位于 D:\vex\Override。此前传过来的迁移包包含另一套四电机 test 程序；不要用它覆盖 Override 源码。

请依次完成：

1. 确认命令确实在这台笔记本本地运行，检查 Git、VS Code、VEX 扩展、SDK、工具链和 USB 驱动，以及 GitHub 仓库访问权限。
2. 若尚未克隆，将仓库克隆到一个新的可写目录；若已存在，先检查 origin、分支和本地修改，保留任何未上传工作，再安全同步 main。
3. 读取仓库的 AGENTS.md、DEVICE_SYNC.md、LAB_NOTES.md 和 README.md；运行 scripts/configure-device.ps1 -Role Laptop，让实验期间不后台更换源码。
4. 可以复用之前收到的 VEX Windows 迁移包中的 SDK、工具链和扩展。自动寻找它的实际解压路径；找到后通过 configure-device.ps1 的 -EnvironmentRoot 参数记录本机路径。不要提交这些安装组件或机器配置。
5. 在 VS Code 中打开 Override 仓库根目录，并编译 Bartmoss。报告当前完整提交号、源码目录、编译结果，以及 USB 主控是否确实被识别。
6. 以后按我的调试要求修改、编译并完成我指示的 USB 测试；每轮把真实现象和参数记录到 LAB_NOTES.md。经过对应验证后，检查 diff，只提交本轮项目文件并推送 origin/main，以便台式机接收。
7. 若出现两端修改或历史冲突，保留双方工作后处理，不强推、不自动丢弃本地修改。Git 同步本身不要下载或启动机器人程序。

不要假设你能看到台式机的聊天记录或磁盘。以仓库文件、当前 Git 状态和本机实际检查结果为准。
