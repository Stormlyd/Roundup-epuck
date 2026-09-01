# Roundup e-puck

当前版本：`simple1`（2026-09-01）

`simple1` 是四车三阶段围捕的首个实机源码基线：一台目标车与三台追捕车依次
执行 `PURSUIT → SURROUND → CAPTURE → CAPTURED`。程序是 Windows 原生
Qt/qmake 应用，不需要 ROS、ROS 2、Webots、Python 或 colcon。

> `simple1` 已通过软件闭环和 Windows Release 构建验证，但自定义机器人固件的
> 轮速比例、接收坐标轴与朝向仍需通过架空轮和短直线实验完成最终标定。

## simple1 主要改动

- 按 e-puck2 的轮径、轮距与电机步数修正差速轮速度换算。
- 修复 `SURROUND` 阶段内圈追捕车无法向外恢复到包围槽位的问题。
- 目标靠墙时先回到可以容纳完整三角包围环的区域。
- 追捕车通过目标外侧极坐标路径换位，并加入连续分离和切向避碰。
- 所有参与车辆加入场地边界软制动。
- 四台参与车只有在完整且及时的新反馈到达后才推进状态机；反馈停更
  100 ms 先发送零速，达到 250 ms 后进入故障停止。
- 增加墙边、簇拥、通道阻塞、反馈时序、边界和轮速标定回归测试。

围捕参数集中在 [`manager/ZooidPursuitProfile.h`](manager/ZooidPursuitProfile.h)，
核心控制位于
[`manager/ZooidPursuitControl.cpp`](manager/ZooidPursuitControl.cpp)。

## 软件环境

- Windows 10/11
- Qt 5.9.7，MinGW 5.3 32-bit kit
- MinGW 5.3 32-bit 编译工具
- Qt Core、Gui、Widgets、SerialPort、OpenGL、Sql、Multimedia 和
  MultimediaWidgets 模块
- Roundup `ZooidReceiver` USB CDC 接收器和四台已激活机器人

Qt 5.9 的资源编译器不可靠地支持中文路径，请把仓库放在纯英文路径，例如：

```text
D:\ros2\Roundup-epuck
```

完整环境说明见
[`docs/WINDOWS_SOFTWARE_SETUP.md`](docs/WINDOWS_SOFTWARE_SETUP.md)。

## 构建

在仓库根目录打开 PowerShell：

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\build_windows.ps1 -Configuration Release
```

脚本会依次运行核心测试、调用 qmake/MinGW 构建、执行 `windeployqt`，并检查
必要的 Qt、串口和 SQLite 运行库。默认可运行目录是：

```text
.build-windows-deploy\release
```

构建脚本默认查找以下工具路径；安装在其他位置时可通过 `-QtRoot` 和
`-MinGwRoot` 参数覆盖：

```text
D:\Qt\5.9.7\mingw53_32
D:\Qt\Tools\mingw530_32
```

## 直接运行

双击部署目录中的 `ZooidManager.exe`。必须保留整个 `release` 目录，不能只复制
EXE，因为程序还依赖同目录的 Qt DLL、`platforms`、`sqldrivers` 和 `json`。

部署目录必须可写。首次运行会在程序旁创建 `sql/ZooidManager.db3`，初始后台
密码是 `admin`，首次启动后应立即修改。打包发布时不要包含本机已经生成的
`sql/*.db3`。

Git 默认忽略 EXE、DLL、数据库和 `.build-*` 目录，因此 GitHub 中保存的是可复现
构建的源码。需要分发程序时，应将完整部署目录另行压缩为 GitHub Release 附件。

## 接收器检查

只连接 Roundup 接收器，拔掉无关串口设备，然后运行：

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\check_receiver_driver.ps1
```

预期结果包含 `ReadyForQSerialPort=True`。Qt SerialPort 需要 USB CDC/COM 驱动，
不要使用 Zadig 将接收器替换为 WinUSB 或 libusbK。

## 实机安全与操作

1. 首次运行必须架空四台机器人的轮子，确认左右轮方向和四个朝向。
2. 清空实验场地，让停止按钮和物理断电手段始终可达，操作员不得离场。
3. 至少四台机器人必须在线、已激活且持续提供新反馈。
4. 开始任务时按硬件 ID 排序：最小 ID 为目标车，后面三个 ID 为追捕车；
   多余机器人保持零速。
5. 架空轮验证后再进行 1 秒短启动和 3–5 秒地面测试。
6. 出现掉线、反馈故障、方向错误或异常加速时立即停止，并现场确认全部车辆停车。

状态阶段、几何半径、理论轮速和详细验收流程见
[`docs/TEST_MODE.md`](docs/TEST_MODE.md)。

## 已知限制

- 仓库不包含当前机器人端自定义固件，现有 `+2007` 协议值是否严格等于
  steps/s 需要通过架空轮转数或短直线距离确认。
- 接收坐标轴和航向继续沿用现有协议，尚未根据单台实车四航向实验做全局翻转。
- 软件闭环仿真不能替代定位噪声、打滑、通信抖动和实体碰撞条件下的实机验收。

## 版本维护

`main` 保存已验证基线；每轮实机调参从独立分支开始，测试通过后再合并并打标签：

```powershell
git switch -c codex/hardware-tuning-simple2
# 修改、测试并重新构建
git add <本轮源码和文档>
git commit -m "simple2: tune physical pursuit control"
git switch main
git merge --ff-only codex/hardware-tuning-simple2
git tag -a simple2 -m "Roundup e-puck simple2"
git push origin main
git push origin simple2
```

提交版本以 Git 标签和 README 为准，构建产物不直接进入源码提交。
