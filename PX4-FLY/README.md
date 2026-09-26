# PX4-FLY 四旋翼仿真

本工程适配本机 **PX4 v1.14.3 + Gazebo Classic 11 + ROS 2 Humble**。采用官方 Iris 四旋翼动力学和传感器，在机头安装固定前视 RGB 摄像头，提供键盘起飞、平移、升降、偏航、悬停、降落及实时三维坐标显示。PX4 的 EKF2、姿态控制器和位置控制器始终参与飞行。

## 运行方法

在三个独立终端分别运行以下命令。脚本自动加载 ROS 环境；不要再额外启动 MicroXRCEAgent。

终端 1：启动 Gazebo、PX4 和 DDS 通信。

```bash
cd ~/Desktop/PX4-FLY
./scripts/start_px4_gazebo.sh
```

终端 2：打开实时相机窗口。

```bash
cd ~/Desktop/PX4-FLY
./scripts/camera.sh
```

终端 3：键盘控制。保持此终端焦点，等出现坐标，按 `T` 起飞。

```bash
cd ~/Desktop/PX4-FLY
./scripts/keyboard.sh
```

先按 `X` 降落，等显示 `armed=False`，再退出键盘节点和仿真。键盘节点 `Ctrl-C` 会请求降落并最多等待 40 秒；再次 `Ctrl-C` 可结束等待。若节点突然崩溃，PX4 在 Offboard 心跳超时后执行降落。终端 1 的 `Ctrl-C` 会结束整个仿真及其子进程。

当前仅面向单机单飞行器 SITL。启动脚本发现已有 PX4、Gazebo 服务端或 Agent 时会停止并提示，防止端口冲突。

## 键盘与坐标

| 按键 | 作用 |
|---|---|
| `T` | 请求 Offboard、解锁，起飞至当前位置上方 1.5 m；双机脚本目标机使用 3 m |
| `W / S` | 沿当前机头方向前进 / 后退，每次 0.25 m |
| `A / D` | 相对当前机头向左 / 向右，每次 0.25 m |
| `R / F` | 上升 / 下降，每次 0.25 m |
| `J / L` | 左转 / 右转，每次 10 度 |
| `I / K` | 前倾 / 后仰，约 5 度的小角度俯仰输入 |
| `U / O` | 向左 / 向右倾斜，约 5 度的横滚输入 |
| `Space` | 将当前位置及航向设为保持目标 |
| `X` | 自动降落 |
| `?` 或 `H` | 显示按键帮助 |

按键改变目标位置；松开后到达目标并悬停，长按使用系统键盘重复。连续输入的水平目标最多领先当前位置 2 m；高度目标限制为 EKF 原点上方约 0.3～20 m。起飞前会等待有效定位和至少 1.2 秒 Offboard 心跳，只有 PX4 反馈已解锁、处于 Offboard 模式后才接受移动。

这是辅助飞行控制：WASD 控制位置，J/L 控制偏航；I/K/U/O 将小角度倾斜意图转换为水平加速度交给 PX4，并保持高度。这些倾斜按键应长按（依赖系统键盘重复），最后一次输入 0.6 秒后恢复位置保持，也可按空格立即保持。实际倾角受 PX4 动态响应影响，不是精确锁定角度或手动油门/特技模式。固定相机随整个机体倾斜，无云台稳定。

控制节点每秒显示实际 ENU 坐标（E 东、N 北、U 上，单位 m）及 RPY（横滚、俯仰、航向，单位度）。位置原点是 PX4 EKF 初始化位置，不是 Gazebo 世界原点；显示的航向仍采用 PX4 北向零度、顺时针为正的定义。

PX4 原始位置和目标使用 NED（北、东、下），ENU 转换为 `(E,N,U)=(y,x,-z)`。ROS 2 话题：

| 话题 | 内容 |
|---|---|
| `/front_camera/image_raw` | RGB 图像，640×480，设定 20 Hz，实际帧率受虚拟机渲染性能限制 |
| `/front_camera/camera_info` | 相机内参，水平视场角 80 度 |
| `/fmu/out/vehicle_local_position` | EKF2 三维 NED 位置、速度、航向和有效性 |
| `/fmu/out/vehicle_attitude` | PX4 FRD 到 NED 的姿态四元数 |
| `/fmu/out/vehicle_odometry` | PX4 估计里程计 |
| `/fmu/out/vehicle_status` | 解锁和飞行模式状态 |

控制输入利用 PX4 1.14 uCDR 的 `timestamp=0` 接收时间语义，由飞控填写当前时间，避免虚拟机仿真降速导致系统时钟与飞控时钟漂移、误触发 Offboard 超时。节点仍检查定位/状态接收的新鲜度，停止节点仍会停止心跳。

相机安装在机体前方 0.24 m、上方 0.02 m，光心再向前 0.04 m，朝机体 +X 水平前视。修改 `models/iris/iris.sdf` 中 `front_camera_link` 的 `pose` 可改变固定安装角（弧度），修改后重启仿真即可。`frame_name` 是图像坐标系标识；本工程尚未发布完整 TF 树。

## 构建与环境

```bash
cd ~/Desktop/PX4-FLY
./scripts/build.sh
```

从 GitHub 新克隆后先初始化消息子模块：

```bash
git clone --recurse-submodules <your-repository-url> PX4-FLY
```

如果已经普通克隆：

```bash
git submodule update --init --recursive
```

`external/px4_msgs` 是独立检出的 `release/1.14` 消息包，与本机 PX4 匹配。原有 `~/px4_ros_ws/src/px4_msgs` 为新版 main，消息布局与 v1.14 不匹配，因此运行本项目必须优先使用本项目安装目录。没有修改用户原来的 ROS 工作空间或 PX4 源代码。

默认使用 `~/PX4-Autopilot/build/px4_sitl_default` 已编译的飞控和插件。可通过 `PX4_DIR` / `PX4_BUILD` 指定其他路径，但需要保持 PX4 1.14 消息兼容。若该路径没有编译产物：

```bash
cd ~/PX4-Autopilot
make px4_sitl_default sitl_gazebo-classic
```

虚拟机性能不足时可隐藏 Gazebo 外部视角窗口，仍可打开相机：

```bash
HEADLESS=1 ./scripts/start_px4_gazebo.sh
```

相机传感器仍需要可用的图形显示/渲染环境，`HEADLESS=1` 仅关闭 Gazebo GUI。可在虚拟机设置中启用 3D 加速。

读取话题时用本项目环境和传感器 QoS：

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/PX4-FLY/install/local_setup.bash
ros2 topic echo /fmu/out/vehicle_local_position --qos-reliability best_effort
ros2 topic hz /front_camera/image_raw
```

## 文件说明

| 文件/目录 | 用途 |
|---|---|
| `models/iris/iris.sdf` | 官方 Iris 机架、电机、IMU/GPS/气压计/磁力计及新增固定相机、ROS 图像插件 |
| `models/iris/model.config` | Gazebo 模型名称及 SDF 入口 |
| `models/fpv_target/fpv_target.sdf` | 无摄像头的 FPV-like 目标机，使用独立 Gazebo/PX4 TCP 4560 链路 |
| `models/tracker_iris/tracker_iris.sdf` | 带固定前视相机的追踪机，使用独立 Gazebo/PX4 TCP 4561 链路 |
| `models/iris/meshes/iris.stl` | 机架外观网格 |
| `models/iris/meshes/iris_prop_cw.dae` | 顺时针旋翼网格 |
| `models/iris/meshes/iris_prop_ccw.dae` | 逆时针旋翼网格 |
| `worlds/training.world` | 地面、光照、四旋翼及前方红绿蓝标志物 |
| `worlds/dual_uav.world` | 双机世界：目标机、追踪机、搜索区域和安全距离标记 |
| `config/sitl.rc` | 调用 PX4 官方启动脚本后设置键盘仿真参数、速度限制、Offboard 丢失降落 |
| `scripts/start_px4_gazebo.sh` | 启动 Agent、Gazebo、PX4；维护进程并在退出时清理 |
| `scripts/start_dual_uav.sh` | 启动双 Gazebo 模型、两个 PX4 实例和隔离的 DDS 命名空间 |
| `scripts/start_all.sh` | 一条命令自动打开双机仿真、键盘、相机、红色检测和跟随窗口 |
| `scripts/keyboard.sh` | 加载本工程环境并启动键盘控制 |
| `scripts/dual_keyboard.sh` | 在双机实验中控制目标机实例 0 |
| `scripts/camera.sh` | 启动 ROS 2 image_tools 实时图像查看器 |
| `scripts/dual_camera.sh` | 查看追踪机 `/tracker/front_camera/image_raw` |
| `scripts/follow.sh` | 启动追踪机自主搜索、追踪和气球破裂后降落节点 |
| `scripts/red_detector.sh` | 启动红气球 HSV 颜色检测器 |
| `scripts/build.sh` | 编译匹配的消息包及键盘控制包 |
| `scripts/smoke_test.py` | 真实 SITL 起飞、移动、转向、倾斜、降落测试，保存相机图与结果到 runtime/verification |
| `px4_fly_control/px4_fly_control/keyboard_control.py` | 键盘处理、状态检查、20 Hz Offboard 设定值、飞行命令、实时坐标和姿态 |
| `px4_fly_control/px4_fly_control/safe_visual_follow.py` | 只基于追踪机 EKF、YOLO 框和可选测距的搜索/追踪/破裂状态机 |
| `px4_fly_control/px4_fly_control/yolo_detector.py` | 将 Ultralytics 自定义权重转换成统一检测消息接口 |
| `px4_fly_control/px4_fly_control/red_color_detector.py` | 从追踪机相机中提取红气球检测框，不需要训练模型 |
| `px4_fly_control/px4_fly_control/__init__.py` | Python 包入口标记 |
| `px4_fly_control/package.xml` | ROS 2 包信息、依赖和构建类型 |
| `px4_fly_control/setup.py` | Python 包安装、注册 `keyboard_control` 可执行程序 |
| `px4_fly_control/setup.cfg` | ROS 2 可执行文件安装目录 |
| `px4_fly_control/resource/px4_fly_control` | ament 包索引标记 |
| `external/px4_msgs/` | PX4 1.14 ROS 2 消息定义及其上游构建/许可证文件 |
| `.gitignore` | 忽略构建、缓存、运行日志 |
| `build/`、`install/`、`log/` | colcon 自动生成的编译、安装和构建日志 |
| `runtime/` | 隔离的 PX4 参数、飞行 ULog、Gazebo/Agent 日志，自动生成 |

`NAV_DLL_ACT=0` 允许未连接地面站的仿真；`COM_RC_IN_MODE=4` 禁用遥控器输入检查；`COM_OBL_RC_ACT=4` 保留 Offboard 丢失自动降落。没有关闭 EKF 或一般解锁检查。这些参数仅用于此工程 SITL。

## 已完成验证

2026-09-24 在本机编译通过两个 ROS 包，并在真实 PX4 SITL + Gazebo 进程中完成自动起飞、前进、右转 30 度、前倾和自动降落测试。测得悬停高度约 1.40 m（目标 1.50 m），前倾俯仰最小约 -5.41 度，降落后已确认自动锁定。另通过交互终端验证 T 起飞、U 倾斜和 Ctrl-C 降落退出。

相机真实帧为 640×480 RGB，已检查内容非空且随移动/转向变化。本机测试约 5～6 FPS；SDF 的 20 Hz 是设定频率，不保证虚拟机达到此帧率。截图和原始测试结果在 `runtime/verification/`。

重新运行完整测试（会自动操控仿真飞机，先启动仿真，且不要同时运行键盘控制）：

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/PX4-FLY/install/local_setup.bash
cd ~/Desktop/PX4-FLY
PYTHONNOUSERSITE=1 python3 scripts/smoke_test.py
```

`PYTHONNOUSERSITE=1` 避免本机用户目录 NumPy 2 与系统 OpenCV 的二进制版本冲突；没有修改系统 Python 或用户安装的软件包。

## 双机安全跟随实验

双机版本用于红气球追踪实验。目标机是无摄像头的 FPV-like 模型，由键盘人工控制；追踪机是带固定前视相机的 Iris，由 `safe_visual_follow` 自主控制。目标机先由键盘按 `T` 起飞；目标机确认进入飞行状态后计时 10 秒，追踪机才解锁起飞并飞向 `search_north/search_east` 提供的粗略方向。锁定红球后高速闭合，只有检测到气球破裂事件才降落。

先启动双机仿真。默认会打开 Gazebo 外部上帝视角窗口，窗口中同时显示目标机和追踪机；不要加 `HEADLESS=1`：

```bash
cd ~/Desktop/PX4-FLY
./scripts/start_dual_uav.sh
```

只有在没有图形界面、只想运行后台仿真时才使用 `HEADLESS=1`。再开三个终端：

```bash
# 终端 A：目标机操作界面，保持此终端焦点后按 T 起飞
cd ~/Desktop/PX4-FLY && ./scripts/dual_keyboard.sh

# 终端 B：查看追踪机固定相机
cd ~/Desktop/PX4-FLY && ./scripts/dual_camera.sh

# 终端 C：启动追踪机 GPS 搜索和安全视觉跟随
cd ~/Desktop/PX4-FLY && ./scripts/follow.sh
```

也可以只执行一个命令自动打开全部窗口：

```bash
cd ~/Desktop/PX4-FLY
./scripts/start_all.sh
```

它会自动打开 Gazebo 上帝视角、目标机键盘、追踪机第一视角、红气球识别和自主跟随窗口。目标机键盘窗口仍需要获得焦点后按 `T` 起飞、按 `X` 降落，因为键盘输入必须由该窗口接收。没有图形界面时可使用 `./scripts/start_all.sh --headless`，但不会打开 Gazebo 和相机图形窗口。

也可以只使用一个终端启动全部功能，同时保留 Gazebo 上帝视角：

```bash
cd ~/Desktop/PX4-FLY
./scripts/start_all.sh --single-terminal
```

Gazebo 上帝视角和追踪机第一视角会作为图形窗口打开；当前终端用于目标机键盘输入。按 `Ctrl-C` 会先请求降落，再清理仿真、检测器和跟踪节点。

目标机键盘操作与单机相同，终端 A 就是目标机的操作界面，不会显示独立的图形面板。Gazebo 窗口是两架无人机的上帝视角，终端 B 的 `showimage` 窗口是追踪机固定摄像头的第一视角。双机的 PX4 通信隔离如下：目标机实例 0 使用 `/fmu/*`，追踪机实例 1 使用 `/px4_1/fmu/*`；相机只发布到 `/tracker/front_camera/image_raw`。退出追踪节点或仿真前让其自动降落，启动脚本退出时会清理两个 PX4、Gazebo 和 DDS Agent 进程。

### YOLO 检测接口

`yolo_detector` 是可选适配器，需要自行准备针对无人机数据训练的 Ultralytics 权重；通用 `yolov8n.pt` 不保证包含无人机类别：

```bash
python3 -m pip install --user ultralytics
cd ~/Desktop/PX4-FLY
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 run px4_fly_control yolo_detector --ros-args \
  -p model:=/absolute/path/to/drone.pt
```

给定粗略 GPS 搜索中心时，把它传给追踪节点（单位为度和米）；例如：

```bash
ros2 run px4_fly_control safe_visual_follow --ros-args \
  -p search_latitude:=31.2304 -p search_longitude:=121.4737 \
  -p search_radius:=30.0
```

节点用追踪机自己的 `/px4_1/fmu/out/vehicle_global_position` 做 WGS84 到本地 N/E 的转换。没有传 GPS 参数时，`search_north/search_east/search_size` 作为 SITL 本地坐标搜索区域。

检测器向 `/tracker/yolo/detection` 发布 `px4_fly_interfaces/msg/VisualDetection`。消息保留原始图像时间戳、相机坐标系和处理完成时间，并包含归一化检测框、置信度及是否触碰图像边缘。也可以接入其他检测器，只要遵循这个消息格式。控制器用相机时间对应的 PX4 姿态历史进行几何补偿，再以恒加速度模型预测短时目标位置。旧帧、重复帧、时间域不一致的帧会被丢弃；短暂丢失先限速预测和重捕获，不会因视觉丢失直接降落。

```bash
ros2 topic pub --qos-profile sensor_data -r 5 \
  /tracker/yolo/detection px4_fly_interfaces/msg/VisualDetection \
  "{detected: true, image_width: 640, image_height: 480, center_x: 0.5, center_y: 0.5, width: 0.08, height: 0.08, confidence: 0.95}"
```

`safe_visual_follow` 的状态是 `WAIT_TARGET -> STARTING -> TAKEOFF -> SEARCH -> LOCKING/APPROACH/FINAL/PREDICT/REACQUIRE -> LANDING`。目标机只有实际离地超过 0.5 m 后才开始 10 秒计时；执行机只读取自己的 PX4 里程计、姿态和相机观测，不读取目标机 PX4 遥测。视觉丢失会先刹车、短时预测并在最后观测区域重捕获，持续搜索期间不会自动降落。

### 红气球颜色检测（当前推荐）

目标机模型下方已经固定了一根 1 m 绳子和红色气球。当前推荐直接使用 HSV 颜色检测，不需要准备 YOLO 数据集或训练权重。启动追踪节点后，在另一个终端运行：

```bash
cd ~/Desktop/PX4-FLY
./scripts/red_detector.sh
```

颜色检测器同样向 `/tracker/yolo/detection` 发布带时间戳的检测消息，因此不需要修改 `safe_visual_follow`。它只保留接近圆形的红色区域；如果红色背景造成误检，可以调高最小区域：

```bash
./scripts/red_detector.sh --ros-args -p min_area:=80.0
```

红气球是视觉标记，不会给追踪机提供目标机位置、速度或通信信息；追踪机仍然只看到相机画面。

气球破裂由 Gazebo 独立接触裁判插件判定：它以追踪机固定相机光心与气球球面的连续扫掠距离为依据，确认几何接触后发布 `/tracker/balloon/burst` 并隐藏气球。控制器不使用目标真值或单目距离阈值触发破裂，收到确认后才降落。`target_hint_north/target_hint_east` 可覆盖默认的本地 N/E 粗略方向。
