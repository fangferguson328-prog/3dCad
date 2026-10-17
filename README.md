# 3dCad：源码上传与协作开发说明

本工程是 Qt/C++ 桌面程序，当前整理范围是 **Windows x64 开发与运行**。采用「GitHub 保存源码，SDK 单独共享」：只克隆源码还不能编译，需要将配套 SDK 包解压到工程根目录，恢复 `includes/`。

本次整理保留原目录与原始源码字节，没有删除旧文件或修改业务代码。Git 仅跟踪下列源码、资源与开发资料，本机 SDK 和生成文件由 `.gitignore` 排除。

## 上传哪些文件

| 文件或目录 | 用途 | 处理方式 |
| --- | --- | --- |
| 根目录手写 `*.cpp`、`*.h`、`*.hpp` | 主程序、机器人、相机、规划算法等；含 `tinyxml2.*` 和 `nanoflann.hpp` | 上传。`ui_*.h` 是例外，不上传 |
| `3dCad.pro` | qmake 构建入口及依赖路径 | 上传，推荐作为统一构建入口 |
| `3dCad.sln`、`3dCad.vcxproj`、`3dCad.vcxproj.filters` | Visual Studio 工程 | 上传；本地需重新选择 Qt 安装路径 |
| `mainwindow.ui`、`iocontrol.ui`、`weldiosetdlg.ui` | Qt Designer 界面源文件 | 上传 |
| `resources.qrc`、`shaders/` | 编入程序的 shader 资源 | 上传 |
| `deployment/` | 构建、打包、部署脚本及清单 | 上传；生成的完整性记录除外 |
| `diagnostics/` | 已有测试与问题诊断资料 | 上传，方便协作验证 |
| 根目录 `*.md`、`licenses/`、`.gitignore` | 开发说明、已有分析文档与第三方许可 | 上传 |
| `log4cpp.conf` | Efort SDK 日志配置 | 上传，区别于生成的 `.log` 文件 |
| `0414_config.example.json` | 已去除本机地址、路径的相机配置示例 | 上传；使用前确认设备参数与标定值 |
| `pcd/test_cube.obj`、`pcd/test_plane.obj`、`pcd/test_cylinder_mesh.obj` | 三个离线测试模型 | 上传，约 0.7 MiB；它们不是 MSVC 编译产物 |

`EfortRobot.cpp/.h` 当前是很小的占位代码，`.vcxproj` 仍引用它们，因此保留；不要因为不在 `.pro` 的编译列表里就删掉。

## 必需但单独共享的 SDK

配套文件：`_share/3dCad-sdk-windows-x64.zip`。解压到工程根目录后，下面的相对路径必须保持不变。精确选择规则见 `deployment/sdk-manifest.json`；压缩包中的 `sdk-integrity.json` 记录文件大小与 SHA-256。

| 依赖 | 必须恢复的目录/文件 |
| --- | --- |
| GLM | `includes/glm/glm.pri`、`includes/glm/glm/`、许可；其中 `glm/detail/glm.cpp` 也参与编译 |
| GLFW | `includes/glfw/include/`，`lib-vc2015/glfw3dll.lib` 和 `glfw3.dll` |
| Intel RealSense | `includes/Intel RealSense SDK 2.0/include/`，`lib/x64/realsense2.lib`，`bin/x64/realsense2.dll` |
| OpenCV 4.7.0 | `includes/opencv/build/include/`，`build/x64/vc16/lib/opencv_world470.lib`、`opencv_world470d.lib`，对应 DLL 和 videoio 插件 |
| Efort SDK V2.5 | `includes/SDK V2.5/sdk-x64/include/`，`libs/x64/` 中 EftSdk、log4cpp、rlibcpp-bcc、rlibcpp-tool 的 Debug/Release LIB、DLL，以及日志组件 |
| KW 相机 SDK | `includes/KW_SDK_D_1.0/SDK_1.0/SDK_windows_1.0/lib/CPP/` 的头文件、`camera` 和 `enumerate` 的 LIB/DLL |
| Harmo 机器人 SDK | `includes/robot/64/` 的四个头文件及 `libharmorobotservice.lib/.dll` |

SDK 包保留当前目录中找到的相关许可文件。没有找到部分厂商 SDK 的独立许可文件；共享时沿用团队已有的厂商授权范围。

Qt、MSVC 和 Windows SDK 不在该依赖包中，由各开发者安装。不要把根目录的旧 `camera.dll/.lib` 当作标准依赖：构建实际引用 KW SDK 下的文件，根目录 DLL 与 SDK 中版本不同。

OpenCV Debug DLL 约 119.6 MiB，超过 GitHub 普通 Git 单文件 100 MiB 限制。因此 SDK 包也不要提交到源码仓库，可通过团队共享盘等渠道分发。参见 [GitHub 大文件说明](https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-large-files-on-github)。

## 不上传的文件

`.gitignore` 已排除以下内容；「不上传」不等于需要删除本地文件。

| 类别 | 当前路径 | 原因 |
| --- | --- | --- |
| IDE 缓存与个人设置 | `.vs/`、`*.pro.user`、`*.vcxproj.user`、`*.qtvscr`、`.DS_Store` | 约 3.9 GiB 的 VS 缓存可重新生成 |
| 本工程构建结果 | `debug/`、`release/`、`x64/`、`3dCad/`、`_build/` | 旧 EXE、OBJ、PDB 和已复制的运行库；SDK 的原件另行保留 |
| 自动生成代码 | `ui_*.h`、`moc_*`、`qrc_*.cpp`、`Makefile*`、`.qmake.stash` | 从 `.ui`、头文件、`.qrc`、`.pro` 重新生成 |
| 日志 | `log/`、`logs/`、`*.log`、`*.log.*` | 本机运行输出 |
| 机器人程序输出 | `demo_program.xpl`、`output.xpl`、`modified_output.xpl`、个人 `*.traj` | 程序生成的文件或个人保存的轨迹 |
| 大型采集数据 | `pcd/` 中除三个测试模型外的文件 | 程序通过文件选择器加载，不是启动固定输入 |
| 本机相机配置 | `0414_config.json` | 含设备地址、历史路径、现场参数；由示例或设备管理员提供 |
| SDK 完整目录与历史备份 | `includes/`、`3dCad.zip` | Git 中忽略，只有必要 SDK 子集单独共享 |
| 旧 Win32 辅助文件 | `Qt_32.props`、`msvc_make.bat`、`cpp.hint` | 不作为本次 x64 统一构建入口 |

SDK 最小包不包含 `SDK V2.0`、32 位库、SDK 示例程序/日志/缓存、OpenCV 全部源码、RealSense 完整安装目录及压缩备份。这些不是当前 x64 构建的输入，但仍留在原工程中。

## 新成员如何开始

1. 克隆源码，或解压 `_share/3dCad-github-source.zip` 到空目录。解压 SDK 包到同一根目录，使 `3dCad.pro` 和 `includes/` 同级。
2. 安装 Windows x64 开发工具。现有构建脚本支持 Qt **5.14.2 msvc2017_64** 或 **5.15.x msvc2019_64**；使用 MSVC C++ 桌面开发工具和 Windows SDK，现有 VS 工程指定 v142。
3. 需要 KW 相机功能时，将 `0414_config.example.json` 复制为 `0414_config.json`，按实际设备配置参数。示例只提供结构，不代表其他设备的有效标定。
4. 用 Qt Creator 打开 `3dCad.pro`，选择匹配的 Windows MSVC x64 Kit，配置构建目录后编译。不要选择 MinGW、Win32 或 macOS Kit。若使用 Visual Studio，需要本机 Qt VS Tools 和正确的 Qt 路径；工程目前仍写着原开发机的 `D:\Qt\...`。
5. 运行时需要匹配构建模式的 DLL、Qt 平台插件和 MSVC 运行库。直接打开窗口也可能需要相机/机器人 SDK DLL，因为它们是链接依赖。发布 Release 版本可使用下面的已有打包流程。

已有 Release 打包流程（Python 3 + Windows PowerShell）：

```powershell
# 在恢复 SDK、准备本机 0414_config.json 后，从工程根目录执行
python deployment/package-windows.py --dry-run
python deployment/package-windows.py --output _share/windows-release-source.zip
Expand-Archive _share/windows-release-source.zip -DestinationPath _share/windows-release-source
& .\_share\windows-release-source\deployment\build-windows.ps1 -QtDir C:\Qt\5.14.2\msvc2017_64
```

`build-windows.ps1` 要求由打包器生成的 `deployment/package-integrity.json`，因此不能直接对刚克隆的源码目录运行。每次改源码后重新打包，再解压到一个新目录构建。这里的完整性记录是生成产物，不应提交到 Git。

现有 `package-manifest.json` 和 `deploy-runtime.ps1` 只覆盖 Release，且没有复制 `log4cpp.conf`。使用上述流程后，**另将源码根目录的 `log4cpp.conf` 复制到生成的 `app/` 目录**，并为 SDK 日志配置 `LOG_FILE_DIR` 指向可写目录；这次整理没有修改既有构建脚本。静态检查确认 SDK 引用了这个配置，尚未在 Windows 上验证缺失时的具体行为。

## 运行配置还需留意

- shader 通过 Qt 资源编入程序；不需要运行时单独复制 `shaders/`，但源码构建必须保留。
- `0414_config.json` 按进程当前工作目录读取；IDE 启动时应配置工作目录，或把文件放到实际运行目录。`demo_program.xpl` 也写到当前工作目录。
- 相机和机器人地址仍有硬编码：`rscamera.cpp`、`mainwindow.cpp`、`mainwindow.ui`；文件对话框也保留原开发机的默认路径。多人使用不同设备时需要分别配置，当前并未完全参数化。
- `camera2tcpRT`、`startJoint1` 至 `startJoint6` 通过 Qt `QSettings` 保存到本机设置，不在工程文件夹里，克隆代码不会带上这些标定/关节参数。
- 当前 macOS 环境只完成文件、依赖路径与压缩包完整性核对，**未验证 Windows 编译、界面启动或硬件联调**。

## Git 上传预览

`_share/3dCad-github-source.zip` 是按 `.gitignore` 筛选的源码快照，不包含 SDK 或本机配置；可以将它解压到空目录作为新仓库。`.gitignore` 只对 Git 操作生效，不会替浏览器手工拖拽上传过滤文件。

在准备好的源码目录执行以下命令，可先审阅要提交的文件：

```bash
git init -b main
git add --dry-run .
```

检查列表后再进行首次提交与关联远程仓库。后续 SDK 有更新时，通过配套清单和新依赖包同步给团队；不要使用 `git add -f includes/` 绕过忽略规则。

## 标定数据采集（RealSense / Harmo）

连接 Harmo 后，点击菜单栏 **标定数据采集**。在窗口中填写棋盘格的内角点数量和实际方格边长，点击 **新建批次并启动相机** 选择保存位置。当前彩色流固定为 **1920×1080、RGB8、30 fps**；启动前请关闭占用该相机的 Windows 相机、RealSense Viewer 等软件。存在多台相机时，先仅保留本次标定的相机。

固定棋盘格和相机支架，由现场人员通过示教器调整拍摄位置，退出零力/拖动示教并等待机械臂停稳，再点击 **采集一组**。窗口显示有效样本数量和本次结果。一次批次使用同一台相机、同一流配置和同一组原生内参；需要新建批次时，结束当前窗口后重新打开。

采集窗口收到心跳断线事件后会禁用采集，且在选择目录后、相机启动完成和采集入口再次核对连接；断线后需结束窗口，回到主窗口重新连接再打开采集。相机的晚到回调不会恢复“就绪”状态。已经完成位姿校验、排队写入的有效样本仍可能完成保存，计数会反映结果。目录选择使用 Qt 窗口，避免 Windows 原生目录选择框不派发 `QTimer` 而暂停 GUI 线程的心跳（见 [Qt 文件对话框说明](https://doc.qt.io/qt-5/qfiledialog.html#getExistingDirectory)）。

采集查询直接向 Harmo SDK 传入六个零 TCP 参数，保存 `sSpaceCoordinate` 六维位姿，并同时保留 SDK 返回的关节、法兰、工具字段。加工工具的缓存参数保持原值。采集窗口与加工互斥，相机和文件写入在工作线程执行，机械臂查询在原线程执行。

每组在取帧前、后分别读取两次位姿，以全体样本的最大差检查静止：位置 **0.10 mm**、姿态 **0.05°**、关节角 **0.05°**，软件时间括号最长 **3 秒**。读取失败、控制器异常、处于运动/转矩模式、时间次序错误或静止检查失败时，显示原因并拒绝登记有效样本。相机预热后持续取流，拍照前排空队列并丢弃数个不同的新帧。这个机制是**静止条件下的软件采样包络**，不提供机械臂与相机的硬件曝光同步保证。

一个批次的目录结构如下：

```text
handeye-<UTC时间>-<随机标识>/
  session.json
  sample-000001/
    color.png
    sample.json
  sample-000002/
    color.png
    sample.json
```

`color.png` 是 SDK 彩色帧的无损 RGB 保存，预览缩放不影响文件；源图不经过镜像、对齐、裁剪、缩放或去畸变。`sample.json` 保存同一个样本编号、相机序列号、实际分辨率和内参、畸变模型及系数、设备帧号和设备时间域、主机 UTC/单调时间、四次零 TCP 位姿和静止校验结果。内参仅适用于记录的相机及分辨率；当前三维点云使用的其他彩色流分辨率需读取其各自内参。可用时还保存深度到彩色的设备外参，具体以元数据中的可用性标记为准。

程序将图片和元数据写入独立临时目录，完成后发布整组文件，并用原子替换更新 `session.json`。编号和 SHA-256 用来验证一一对应关系，失败不会增加样本计数；磁盘异常且无法回滚时会封锁该批次。程序崩溃后可能留下 `.pending-*` 或尚未登记的目录，以 `session.json` 中的完整样本为有效索引。结束窗口会先完成当前已排队操作再释放相机。传输数据时请打包**整个批次目录**，保留目录结构和 JSON 文件。

可以使用 Python 3 检查采集结果并导出图片路径与位姿的对应表：

```powershell
python diagnostics/validate_calibration_dataset.py "D:\标定数据\handeye-..."
python diagnostics/validate_calibration_dataset.py "D:\标定数据\handeye-..." --export-csv "D:\标定数据\对应表.csv"
```

本次开发环境可运行采集策略和 SDK 边界测试、数据文件校验测试；**Windows Qt 完整构建和真实设备采集仍需现场验证**。Qt 文件保存测试工程为 `diagnostics/calibration_dataset_store_test.pro`，只需要 Qt Core/Gui，无需连接设备。
