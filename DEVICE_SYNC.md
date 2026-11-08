# 台式机与实验室笔记本同步

代码以 [HudEn5/Override](https://github.com/HudEn5/Override) 的 `main` 分支为准。
流程是：台式机提交推送 → 笔记本拉取 → 笔记本调参、编译及 USB 测试 → 笔记本提交推送 → 台式机拉取。
每次换设备前，完成提交和推送；另一台开始编辑前，先拉取。
同一时段让一台电脑负责修改，另一台接收更新，可以减少冲突。

## 笔记本首次准备

1. 安装 Git、VS Code、Codex 扩展并登录各自账号，确认 GitHub 账号对仓库有写权限。VEX 扩展、SDK、工具链和 USB 驱动须在笔记本上准备，Git 不同步这些安装组件。
2. 在一个新目录克隆仓库，不覆盖之前收到的迁移包或本地调参文件：

   ```powershell
   git clone https://github.com/HudEn5/Override.git C:\VEX\Override
   cd C:\VEX\Override
   .\scripts\configure-device.ps1 -Role Laptop
   ```

3. 用 VS Code 打开 `C:\VEX\Override` 仓库根目录。确认项目名为 **Bartmoss**，不要继续编辑旧迁移包中的 `test`。
4. 检查 `git config user.name` 和 `git config user.email`；缺失时为此仓库设置自己的提交身份。通过 VS Code 的 GitHub 登录流程完成认证，不把密码或令牌写进项目。
5. 如果沿用之前的迁移包，记录它的实际解压位置：

   ```powershell
   .\scripts\configure-device.ps1 -Role Laptop -EnvironmentRoot 'C:\实际位置\vex-windows-日期编号'
   .\scripts\build.ps1 -Rebuild
   ```

   本机配置保存在被 Git 忽略的 `.vex-device.json` 中。该配置让构建脚本使用迁移包内的 SDK 和工具链，不会把旧 `test` 源码导入仓库。
   VEX 扩展下载时也要选本仓库，并确认它能找到对应 SDK、工具链和笔记本的 USB 主控。

## 每次去实验室

台式机结束工作时，先提交并推送。笔记本联网后，在干净工作区执行：

```powershell
git status --short --branch
.\scripts\sync-github.ps1
git log -1 --oneline
.\scripts\build.ps1 -Rebuild
```

如果同步提示 `paused`，先保存和处理笔记本自己的修改，不强制覆盖。
实验室无网络时，可调试已经拉取的版本并本地提交，联网后再推送。
USB 下载和运行必须由笔记本本地的 VEX 工具执行；UU 远程桌面和 GitHub 拉取都不会把程序自动写入主控。

完成一轮调参和验证后：

1. 保存文件，重新编译，记录真实现象和参数到 `LAB_NOTES.md`。
2. 用 `git diff` 和 `git status` 检查改动，在 VS Code 源代码管理中只暂存本轮项目文件。
3. 提交并推送：

   ```powershell
   git commit -m "lab: 描述本轮调试结果"
   git push origin main
   git rev-parse HEAD
   ```

4. 确认 GitHub 上已出现该提交。只 commit 未 push 时，台式机仍收不到代码。
   推送若提示远端有新提交，停止并让 Codex 比较两端改动、保留双方工作后处理，不使用强推。

## 台式机接收更新

台式机的正式仓库是 `D:\vex\Override`。首次配置：

```powershell
cd D:\vex\Override
.\scripts\configure-device.ps1 -Role Desktop
```

用 VS Code 打开这个目录，在工作区受信任且允许自动任务时，任务 `VEX: desktop GitHub auto pull` 每 60 秒检查一次。
也可手动启动并保持终端运行：

```powershell
.\scripts\sync-github.ps1 -Auto
```

检查一次可运行 `scripts/sync-github.ps1`。它仅从指定仓库快进更新干净的 `main`：有未提交文件、本地提交、分叉或错误时保留本地工作，不自动提交、推送、切换分支或重置。
关闭任务或终端后检查停止；台式机关机或离线期间不会更新，恢复联网后再拉取。
需要暂停自动拉取时，可暂时运行 `scripts/configure-device.ps1 -Role Laptop`；正在运行的自动任务会在下一轮退出。

## 给另一台电脑的 GPT

新会话先读取仓库中的 `AGENTS.md`、本文件和 `LAB_NOTES.md`。可复制 [LAPTOP_PROMPT.md](LAPTOP_PROMPT.md) 的内容。
会话记录是否可见取决于使用的产品和同步能力，不能据此判断文件版本相同。
[OpenAI 官方说明](https://learn.chatgpt.com/docs/projects)明确区分聊天同步与本地文件访问。
两台电脑的 `git rev-parse HEAD` 相同，且各自工作区无修改，才表示它们当前已跟踪的代码版本一致。
