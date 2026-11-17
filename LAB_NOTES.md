# 当前实验记录

当前开发版 **2.1.0-point1**，程序提交2249f44，分支feat/odometry；main发布基线92eba1f。新位姿、turn3、MoveToPoint与Swing已通过软件检查，尚未完成本轮实车验证。

## 当前参数与待测事项

- 硬件/前馈以include/control-config.h为准；速度与规划以include/performance-config.h为准；测试档位在include/test-config.h。
- 最高540电机rpm（轮端450rpm），原地角加减速1200/1000度每秒²；制动预留100ms，滤波20ms，最低60/30rpm。原0.8°与零速门槛保留，自动无额外确认，X单项250ms。
- 验证顺序：静止/前后/原地转向核对位姿 → 单条短距离MoveToPoint → 左右Swing → 再编排路线。
- MoveToPoint终点5mm容差与当前高速参数待实车确认；Swing固定侧可能滑移。模型结果不可替代物理测量。
- 由用户从VS Code下载；采集按Y开关，场控连接时关闭。记录现场版本、负载、电池、原始数据文件、终点测量及故障状态。

## 阶段原始记录

| 记录 | 范围 |
| --- | --- |
| [底盘与速度控制](docs/history/LAB_DRIVE_FOUNDATION.md) | 前馈、速度环、直行、圆角与早期高速开发 |
| [原地转向与路线](docs/history/LAB_TURN_AND_ROUTES.md) | turn1/turn2、实车八次转向、路线交接与基线发布 |
| [位姿与新动作](docs/history/LAB_ODOMETRY_AND_ACTIONS.md) | 位姿、turn3、MoveToPoint/Swing实现及模型结果 |

旧参数、备份路径及实车/模型结果按当时版本原样保留，当前操作说明以README与docs为准。

## 2026-11-17：工作区整理

用户授权删除外层Override.zip，并执行文档合并、日志归位和临时文件清理。控制程序、参考工程、原始采集数据、VEX下载配置及用户编辑器修改保持原样。旧实验记录按阶段保存，未核实的备份保留并做清单；迁移明细见本地build/logs/workspace-cleanup.json，44个可重建文件的删除见deleted-generated-files.json。24份备份无整目录重复，206/340个文件匹配Git对象，快照完整保留；原始采集新增build/data/INDEX.md轮次索引，不移动原数据。

## 后续每轮记录格式

日期/现场版本与提交号；硬件、负载与电池；修改参数及单位；构建结果；是否下载；数据路径；外部测量与现象；结论及待验证项。请分别标明实车、软件模型与尚未验证的内容。

验证：861个受保护文件逐项SHA256一致，源码/测试/参考/原始数据/备份及VEX编辑器配置未改变。PowerShell构建记录成功与失败均进入build/logs；第一次Default重编译发生clang8前端内部崩溃，未改源码和工具链重试通过。宿主机首批在旧autonomous-turn启动断言失败，同源独立重编译连续3次通过，未修改断言或生产代码；完整复核结果追加在本节。自动审批曾拒绝批量清理，改为明确单文件删除与边界校验后的归档操作，无递归删除。未下载、未实车、未上传。

本轮完整回归没有一次全通过：第二批在startup-route的Start就绪断言失败，日志保留于build/logs内最新带时间戳的host-tests日志。此前34项通过是程序开发阶段记录，不作为本轮无异常结论。失败原因尚未定位，保留源码/断言并列为待查；当前只确认脚本能正确记录、传播非零失败状态以及默认SDK构建重试成功。后续先排查该间歇断言，不直接将重试通过当成修复。


## 2026-11-17：接口行为与编码统一（2.1.0-api1）

- 用户授权执行统一性整改。CancelRoutine和子动作取消即时发布零输出；修复外层active=false但Info仍残留506mV的复现。速度ResetControl保留当前采样基准、仅清控制积分；ConfigurePID冷配置清旧序号/时钟，修复时钟重起且序号复用时Fresh失败的复现。
- 新PointControl独立position_tolerance_mm=5/pose_stale_ms=200，取消与沿程容差、转向时钟预算的隐式绑定。点位按距离把误差预算分给横向朝向和沿程，各取总容差的一半且保留原更严格值；严格朝向下相应降低末端最低转速。普通运动参数与硬件端口/方向保留。
- command采用明确单位字段及pivot_side枚举；基础控制器使用共享motion_state别名，路线result和点位阶段用枚举，取消/错误结果根据状态决定，字符串只显示。手控/速度/航向快照明确mV/rpm/deg，内部类型snake_case、方法PascalCase，硬件名保留。增加.clang-format，使用已有VS Code C++扩展的格式工具，未更换SDK/下载器。
- 新增api-consistency-test，覆盖立即取消、热重置与冷配置、重复序号、新时钟、按距离的朝向要求、独立定位超时与可配置转速纠偏。Point/Swing扩展到300mm、1500mm、反向目标和1500/1500对角点位，模型5mm终点要求均通过。
- 全套35个宿主机程序通过（格式/命名调整后）；扩展点位单项通过。旧间歇断言未在该轮复现，不将此视为根因已经定位。 SDK各模式与协议检查结果在完成后追加。未下载/上机/推送。

最终验证：35个C++宿主机程序通过；300mm/1500mm/反向与对角点位专项通过；Node所有遥测协议检查通过；Test/Competition/Calibration/HighSpeed/Default五种SDK重编译均通过，Default最后生成。.clang-format检查通过，include仍20个。没有下载、上机或上传GitHub。
