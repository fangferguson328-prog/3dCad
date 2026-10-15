# Harmo 联调日志使用说明

本次增加诊断日志，保留原有运动目标、速度、调用顺序、SDK 返回值、等待与错误处理流程。没有连接或操作真实设备。

## Windows 上怎么使用

1. 同步并重新编译这四个源码文件：`HarmoRobot.h`、`HarmoRobot.cpp`、`mainwindow.h`、`mainwindow.cpp`。使用原来能够连接 Harmo 的 Visual Studio 工程和 SDK 配置；没有新增源文件需要加入工程。Harmo 两个文件保留 GB18030，MainWindow 保留原有 UTF-8 编码。
2. 启动程序后，顶部菜单栏有 **Harmo日志**。点击会打开日志目录，状态栏显示当前日志文件的完整路径。
3. 日志优先保存到 `3dCad.exe` 所在目录的 `logs/harmo-年月日-时分秒-毫秒-进程号.log`。该位置不可写时，自动回退到 Qt 返回的本地应用数据目录下的 `logs`；用菜单进入即可，不需要猜 AppData 路径。
4. 正常连接、采集、绘制并进行现场联调。每次点击“开始加工”都会产生一个新的 `run` 编号；同次程序启动的记录保存在同一文件中，每行写入后调用 flush。
5. 报错后保留整个当前 `.log` 文件。即使随后点击清错，清错前采集到的信息也已写在文件中。查找问题时先搜索对应 `RUN_BEGIN` 和 `FIRST_FAILURE`。

日志创建或写入失败会记录诊断错误，界面状态栏、日志菜单或加工开始／结束时会提示；不会用空文件冒充正常日志。日志不自动删除，联调完成后可自行归档。

## 怎么定位是哪一步

`context` 会标明轨迹序号、类型和阶段，例如：

```text
context="track=1/1 type=ARC stage=arc_execute"
```

| context / stage | 位置或动作 |
|---|---|
| `run/reset_error` | 开工时清错；先记录清错前快照 |
| `run/enable_power` | 使能 |
| `run/work_start` | MoveJ 到配置的工作起始关节位置 |
| `line_approach` / `arc_approach` | MoveP 到直线／圆弧起点 |
| `line_execute` | MoveL 走直线 |
| `arc_execute` | MoveC 经过中点到圆弧终点 |
| `line_retract` / `arc_retract` | MoveL 从终点退离；本工程当前配置 100 mm |
| `capture=...` | 采集时的相机结果、法兰位姿查询与矩阵 |
| `manual/...` | 界面上的连接、断开、清错、使能等人工操作 |

不要再依赖旧的 1426 / 1531 / 1551 行号：新增日志后行号已经移动。按上述阶段及函数名查找更稳定。

每行还包括时间戳、`run`、`cmd` 和 `motion_cmd`：

- `run`：第几次开始加工，启动后未开始加工时为 0。
- `cmd`：该 Harmo 实例下的控制调用序号，包含心跳、使能和清错。
- `motion_cmd`：最近一条运动调用的编号；心跳不会覆盖它。结合 `context` 关联运动及其后续状态。
- `SEND`：即将调用的运动指令及实际转换为 float 的参数。
- `RESULT`：SDK 返回的布尔值与该 SDK 调用的耗时（不包含随后既有的 Sleep 或运动等待）。`sdk_return=true` 不等于运动完成。
- `FIRST_FAILURE`：该 run 中第一次观察到的 SDK／查询失败、控制器异常或错误信号，只标记一次，后续错误仍有具体日志。如果它出现在 `run/reset_error` 的 `before_reset_error`，可能是开工前已存在的报警，不能归因于本次尚未下发的圆弧。

## 报错时重点看哪些字段

| 事件 | 关键内容 |
|---|---|
| `SEND command=MOVEJ` | 目标 6 个关节角，单位度 |
| `SEND command=MOVEP/MOVEL` | 目标基座 XYZ/RPY、TCP 配置、速度和坐标系参数 |
| `SEND command=MOVEC` | 经过点及终点基座 XYZ/RPY、TCP、速度、`arc_type`、`direction`、`coord_type` |
| `STATE` | 使能、错误、IO 板、运行模式、急停、在线／离线、运动及运动线程状态 |
| `ERROR_CODE` | `motor_raw` 原始轴号、`error_code_dec` 十进制错误码、`error_code_hex` 十六进制错误码 |
| `POSITION` | `actual_joints_deg` 实际关节角；`actual_tcp_base_XYZ_RPY_mm_deg` 按当前 TCP 配置查询得到的实际位姿 |
| `TCP_CONFIG` / `cached_tcp_XYZ_RPY_mm_deg` | 各次查询／指令采用的工具 TCP 偏置 |
| `TRACK_INPUT` / `ARC_INPUT_POINT` | 原始相机轨迹点（米）、X/Z 方向、三组圆弧数组长度、连续轨迹标志 |
| `RUN_TRANSFORMS` | 本次加工实际采用的 `tcp2BaseRT`、`camera2tcpRT`、`camera2BaseRT`，平移单位毫米 |
| `CAPTURE_*` | 相机采集与法兰查询是否成功、采集时采用的矩阵 |
| `CLOUD_FILE_LOAD` / `RESTORED_TRACKS` | 文件加载结果和从同名 `.traj` 恢复的轨迹数量 |

状态、错误码和实际位置查询各自记录 `query_ok`。任何一项返回 false，其数据写为 `unavailable`，不能把它解释成零角度、基座原点或无报警。三个查询按顺序执行，属于诊断采样，不是控制器同一时刻的原子快照。

SDK 头文件给出的状态含义：

- `robot`：0 正常，1 错误，2 警告。
- `motion`：0 无运动，1 运动中，2 运动完成。
- `motion_thread`：0 无执行，1 执行中，2 执行完成。
- `direction`：0 无约束，1 固定；仅记录 SDK 原始参数。

`motor_raw` 的编号规则及错误码含义需要结合 Harmo 控制器／厂商说明解释，代码不猜测它是否从 0 开始编号。速度记录 SDK 原始参数，不沿用旧代码中不一致的速度注释。

以下仅为日志结构示意，省略号不是实际采集值：

```text
... run=1 cmd=12 motion_cmd=12 context="track=1/1 type=ARC stage=arc_execute" SEND command=MOVEC pass_base_XYZ_RPY_mm_deg=[...] end_base_XYZ_RPY_mm_deg=[...] ...
... run=1 cmd=12 motion_cmd=12 context="track=1/1 type=ARC stage=arc_execute" RESULT command=MOVEC sdk_return=false elapsed_ms=...
... run=1 cmd=12 motion_cmd=12 context="track=1/1 type=ARC stage=arc_execute" FIRST_FAILURE reason="MOVEC SDK returned false"
... ERROR_CODE query_ok=true motor_raw=... error_code_dec=... error_code_hex=0x...
... POSITION query_ok=true actual_joints_deg=[...] actual_tcp_base_XYZ_RPY_mm_deg=[...]
```

如果 `RESULT sdk_return=true`，但后续 `STATE robot=1`，也会标记异常并记录错误码和位置。首次运动状态轮询会采集一次，后续状态变化时再次采集；相同状态不会持续刷完整快照。

## 验证与边界

运行：

```bash
python3 diagnostics/test_harmo_logging.py
```

10 组离线测试通过，覆盖发送失败、SDK 成功但控制器报警、三类查询失败、清错前证据、状态去重、日志目录回退、写入／flush 失败、首次故障去重及单行转义。另用删除关键日志的临时变体校准测试，确认缺少失败快照或清错前快照会被检出。

测试编译实际 `HarmoRobot.h/.cpp`，Qt／SDK 使用边界替身，文件操作使用真实临时文件；没有验证真实 Qt ABI、Windows DLL、网络或机械臂运动。MainWindow 插桩经过独立源码审查。仍需在 Windows 完整编译。

新增诊断会增加只读 SDK 查询和写盘耗时。原有错误后继续下发、Sleep 和仅以 `motion==1` 等待的流程保持原样，因此 `WAIT_EXIT` 只表示离开等待，`RUN_END outcome="function_returned"` 只表示函数返回，均不保证轨迹成功。日志也不能证明整个关节路径可达。

最近采集标识记录的是采集尝试，不能代替 `CAPTURE_CAMERA` / `CAPTURE_FLANGE_QUERY` 的成功标志。实时采集不会清空旧的文件名，所以 `last_loaded_file` 只是最近加载记录，日志不会将它冒充当前实时点云的来源。
