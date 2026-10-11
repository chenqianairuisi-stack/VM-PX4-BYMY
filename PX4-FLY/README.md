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

单机 Iris 相机安装在机体前方 0.24 m、上方 0.02 m，光心再向前 0.04 m，朝机体 +X 水平前视。修改 `models/iris/iris.sdf` 中 `front_camera_link` 的 `pose` 可改变固定安装角（弧度），修改后重启仿真即可。`frame_name` 是图像坐标系标识；本工程尚未发布完整 TF 树。

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
| `models/landing_lidar/model.sdf` | 目标机朝下的近地测距传感器，用于减速降落和着陆检测 |
| `models/tracker_iris/tracker_iris.sdf` | 带固定前视相机的追踪机，使用独立 Gazebo/PX4 TCP 4561 链路 |
| `models/stable_gps/model.sdf` | 追踪机的低漂移 GPS 仿真传感器，减少原地旋转时的虚假定位漂移 |
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
| `scripts/verify_target_landing.py` | 双机 SITL 中测量目标机触地后的实际回升，并确认自动锁定 |
| `px4_fly_control/px4_fly_control/keyboard_control.py` | 键盘处理、状态检查、20 Hz Offboard 设定值、飞行命令、实时坐标和姿态 |
| `px4_fly_control/px4_fly_control/safe_visual_follow.py` | 只基于追踪机 EKF、相机检测框与气球接触事件的搜索/追踪状态机 |
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

双机版本用于红气球追踪实验。目标机是无摄像头的 FPV-like 模型，由键盘人工控制；追踪机是带固定前视相机的 Iris，由 `safe_visual_follow` 自主控制。目标机先由键盘按 `T` 起飞；目标机确认进入飞行状态后计时 10 秒，追踪机才解锁起飞，在固定位置原地旋转搜索。第一帧可靠红球观测即开始限速闭合，第二帧完成加速锁定，飞行中持续根据新画面修正，只有检测到气球破裂事件才降落。

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

# 终端 C：启动追踪机原地视觉搜索和跟随
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

旧版 `search_latitude/search_longitude/search_radius` 和 `search_north/search_east/search_size` 参数暂时保留兼容，但固定位置搜索不会使用它们，也不会飞向 GPS 或本地搜索航点。

检测器向 `/tracker/yolo/detection` 发布 `px4_fly_interfaces/msg/VisualDetection`。消息保留原始图像时间戳、相机坐标系和处理完成时间，并包含归一化检测框、置信度及是否触碰图像边缘。也可以接入其他检测器，只要遵循这个消息格式。控制器用相机时间对应的 PX4 姿态历史进行几何补偿，再用带加速度过程噪声的恒速度 Kalman 滤波估计目标位置与速度。制导同时外推目标和追踪机的相对运动，按距离及速度估计的不确定度选择 0.12～0.8 秒预瞄时间，机头持续对准当前视线。旧帧、重复帧、时间域不一致的帧会被丢弃；短暂漏帧时沿预测轨迹继续飞行，超过 0.25 秒未见有效目标会停止盲目加速并平滑降低闭合速度，超过 0.85 秒才进入制动/定点重捕获，不会因视觉丢失直接降落。追踪日志每秒输出处理帧率 `frames`、有效识别帧率 `valid`、锁定帧数 `lock` 和不可追踪原因 `track`。

```bash
ros2 topic pub --qos-profile sensor_data -r 5 \
  /tracker/yolo/detection px4_fly_interfaces/msg/VisualDetection \
  "{detected: true, image_width: 480, image_height: 360, center_x: 0.5, center_y: 0.5, width: 0.08, height: 0.08, confidence: 0.95}"
```

`safe_visual_follow` 的状态是 `WAIT_TARGET -> STARTING -> TAKEOFF -> SEARCH -> LOCKING/APPROACH/FINAL -> BRAKING/EDGE_RECOVERY/REACQUIRE -> LANDING`。目标机只有实际离地超过 0.5 m 后才开始 10 秒计时；执行机只读取自己的 PX4 里程计、姿态和相机观测，不读取目标机 PX4 遥测。首次搜索先稳定悬停，再以 0.35 rad/s 原地旋转整圈；未找到则继续原地旋转，首次可靠识别到目标后才开始限速平移。追踪中视觉丢失时，如果最后一帧明确显示近距离目标从画面边缘离开，先向该边缘做不超过约 0.65 m 的限幅修正；仍未找到则先按限加速度速度指令减速，速度降低后固定当前位置并原地转一圈，之后保持悬停。减速期间有惯性位移，不能保证失去画面瞬间就静止。重新看到目标时恢复追踪，不因视觉丢失自动降落。

### 红气球颜色检测（当前推荐）

目标机模型下方有一根 1 m 绳子和红色气球，两者只是机身视觉部件，没有独立质量或碰撞体，不会把目标机顶离地面。目标机另有朝下的近地测距传感器，PX4 在最后约 1 m 切换到 0.2 m/s 下降并加快着陆确认。当前推荐直接使用 HSV 颜色检测，不需要准备 YOLO 数据集或训练权重。启动追踪节点后，在另一个终端运行：

```bash
cd ~/Desktop/PX4-FLY
./scripts/red_detector.sh
```

颜色检测器同样向 `/tracker/yolo/detection` 发布带时间戳的检测消息，因此不需要修改 `safe_visual_follow`。它只保留接近圆形的红色区域；如果红色背景造成误检，可以调高最小区域：

```bash
./scripts/red_detector.sh --ros-args -p min_area:=80.0
```

红气球是视觉标记，不会给追踪机提供目标机位置、速度或通信信息；追踪机仍然只看到相机画面。

气球破裂由 Gazebo 独立接触裁判插件判定：它以追踪机固定相机光心与气球球面的连续扫掠距离为依据，确认几何接触后发布 `/tracker/balloon/burst` 并隐藏气球。控制器不使用目标真值或单目距离阈值触发破裂，收到确认后才降落。旧版 `target_hint_north/target_hint_east` 参数也保留兼容，但不会影响当前原地搜索。

### 转向平滑与预测追踪

键盘 J/L 仍每次改变目标航向 10°，实际发布的航向会连续加速、减速；键盘和追踪控制器分别使用各自的参数及仿真时间限制偏航角速度、角加速度，暂停仿真时发布的航向不会继续前进。PX4 的 NED 航向直接用于控制，仅补偿相机安装方向，避免把异步遥测的时间差固定成航向偏差。水平键盘位置目标最多领先实际位置 2 m。

追踪机的闭合速度默认 4.5 m/s，总水平速度上限 7 m/s，速度指令加速度 3 m/s²、jerk 6 m/s³；接近目标时平滑降低闭合速度。目标速度前馈配合相对运动预瞄，预瞄只补偿视线横向相对运动，位移不超过目标距离的 35%，避免近距离反向修正。这里只做短时恒速度预测，急转、遮挡和低帧率仍会影响追踪效果。

丢失目标时，若最后一帧显示近处气球从画面边缘消失，先按该画面方向做一次短距离修正并转头寻找，避免错过突然横向转弯的目标。修正仍未找回或最后画面方向不明确时，记录固定 NED 位置并交给 PX4 位置保持；飞机需要实际制动，惯性可能带来短暂偏移。速度低于 0.2 m/s 且距离保持点小于 0.25 m 后，同向旋转 360° 搜索，转完仍未找到则在该点悬停。找到目标后从当前实测速度恢复追踪。

追踪机相机外壳安装在机身前方 0.17 m、上方 0.02 m，通过实体安装座连接机身，并由固定关节约束；光心相对机身的 NED 偏移为 `(0.21, 0, -0.02)` m。追踪机相机输出 480×360，设定 30 Hz；实际 `frames` 仍受 Gazebo 渲染速度约束。模型、视觉几何默认值和控制器参数同步，接触插件使用相机链接实际位置。追踪机仍使用 PX4 EKF 和位置控制；它的 GPS 仿真传感器保留测量噪声，但降低随机游走，避免虚假的定位漂移让定点搜索绕圈。目标机和单机 Iris 的 GPS 模型不变。

追踪节点 `safe_visual_follow` 默认参数（键盘节点保留原来的偏航限幅）：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `lock_frames` | 2 | 第一帧开始低速闭合，第二帧允许完整接近速度 |
| `lock_speed` | 1.4 m/s | 未完成两帧锁定时的闭合速度上限 |
| `yaw_rate_limit` | 0.7 rad/s | 追踪航向最大变化速度，约 40°/s |
| `yaw_acceleration_limit` | 1.2 rad/s² | 偏航角加速度上限 |
| `command_acceleration` | 3.0 m/s² | 追踪速度指令的加速度上限 |
| `command_jerk` | 6.0 m/s³ | 加速度变化上限 |
| `max_follow_speed` | 7.0 m/s | 总水平追踪速度上限 |
| `approach_speed` | 4.5 m/s | 远距离闭合速度上限 |
| `fresh_timeout` / `prediction_timeout` | 0.25 / 0.85 s | 短暂漏帧继续预测，之后停止盲目加速、渐减速并重搜 |
| `prediction_sigma_limit` / `prediction_relative_sigma_limit` | 2.7 m / 0.3 | 允许的深度估计误差下限及距离比例；远处目标不会被固定 1 m 门槛误拒 |
| `search_yaw_rate` | 0.35 rad/s | 首次定点搜索的参考角速度 |
| `reacquire_yaw_rate` | 0.6 rad/s | 原地一圈搜索的参考角速度 |
| `edge_recovery_duration` | 0.7 s | 近距离脱靶后的短修正持续时间 |
| `control_lookahead` | 0.12 s | 最短预瞄时间 |
| `max_control_lookahead` | 0.8 s | 最长预瞄时间 |

`config/sitl.rc` 将追踪机实例 1 的 `MPC_XY_VEL_MAX` 设为 7 m/s、`MC_YAWRATE_MAX` 设为 60°/s；目标机保持 5 m/s、45°/s。它们是速度上限，不是 PID 增益。目标机另设 `MPC_LAND_CRWL=0.2` m/s、`LNDMC_TRIG_TIME=0.3` s；近地测距有效时，PX4 在最后约 1 m 自动减速并检测触地。修改后执行 `./scripts/build.sh`，退出并重启仿真及控制节点，让 PX4 启动参数和节点参数一同生效。

目标机降落回弹验证需先启动双机仿真，且不要同时运行另一个目标机键盘节点：

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
PYTHONNOUSERSITE=1 python3 scripts/verify_target_landing.py
```

脚本读取 Gazebo 中目标机的真实机体高度，按 `X` 所对应的命令降落，以首次真实触地后的最高点计算回升，超过 3 cm 或未自动锁定即报错；验证脚本不参与正常追踪控制。

运行几何、预测、指令连续性及 ROS 节点边界回归测试（不解锁飞机）：

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
PYTHONNOUSERSITE=1 python3 -m pytest px4_fly_control/test -q
```

2026-10-06 验证：4 个 ROS 包构建通过，26 项回归测试通过。双机 PX4 SITL + Gazebo 分别完成“目标转向并平移后悬停”和“追踪期间目标持续前移并叠加横向正弦运动”两组测试，均由接触插件确认气球接触，追踪机自动降落，测试结束两机均已锁定。两组追踪机最大机体偏航角速度分别为 22.4°/s、24.5°/s，最大倾角分别为 14.4°、16.3°。测试记录位于 `runtime/verification/tracking_smooth/` 和 `runtime/verification/tracking_moving/`；这些结果限于上述仿真工况。

同日追加减少多余动作的回归：30 项测试通过，包括近距离预瞄不会反向、单帧漏检保持制导、重新捕获先刹车后扫视。移动目标 SITL 再次确认接触并自动降落，两机均锁定，追踪阶段约 4.1 秒；记录位于 `runtime/verification/direct_tracking/`。该次轨迹长度与首末位置直线距离之比约 1.011，仅用于描述这次测试，不代表所有目标机动下的最优轨迹。

最新提速、相机安装和定点整圈搜索版本：33 项回归测试通过，覆盖加速响应、安装座与机身网格连接、相机偏移一致性、固定位置指令、跨越 ±180° 的整圈旋转及恢复追踪。SITL 人为中断相机观测后，实测旋转 359.9°、最大三维位置保持误差约 0.29 m，恢复观测后完成气球接触并降落；记录在 `runtime/verification/hold_circle/`。最终移动目标测试实测峰值水平追踪速度 3.21 m/s、约 4.56 秒确认接触，两机降落锁定；记录在 `runtime/verification/faster_final/`。7 m/s 是配置上限，短距离、视场和目标估计的不确定度会限制实际速度，上述测试未验证满速飞行。

2026-10-11 追加验证：4 个 ROS 包构建通过，41 项回归测试通过。目标机 `X` 降落的真实 Gazebo 机体轨迹在首次触地后未测得回升，接地速度约 0.26 m/s，PX4 自动锁定；修复前在相同类型测试中曾回升约 0.55 m。追踪机首次原地搜索连续两次完成约 360°，Gazebo 机体水平最大偏移分别约 0.10 m 和 0.18 m。受控近距离脱靶测试在 2.60 m 处输入一帧从画面边缘消失的观测后，执行约 0.54 m 限幅修正，并重新识别目标、恢复最终接近状态；此测试在恢复追踪后主动降落，没有声称完成接触。另在移动目标全流程测试中，Gazebo 插件确认气球接触，两机均自动降落锁定。记录位于 `runtime/verification/2026_10_10/`、`runtime/verification/2026_10_11/full_moving_tracking.log` 和 `runtime/verification/faster_final/result.json`。这些数值仅代表所测 SITL 工况，不能保证任意急转时必中。

当前版本验证：4 个 ROS 包构建通过，45 项回归测试通过。13 m 左右的静止远目标从首次接近到接触约 4.82 仿真秒，追踪机位移约 12.3 m，峰值水平速度约 3.85 m/s；连续移动目标也由接触插件确认接触。单终端带 Gazebo 与相机窗口运行时，追踪日志在接近阶段显示约 14～24 帧/秒，完成接触。另用测试脚本在约 1.5 m/s 时强制断开识别 2 仿真秒，控制器进入 `BRAKING`，恢复画面后重新接近并最终接触；该验证只覆盖所测断帧与目标轨迹。记录位于 `runtime/verification/2026_10_11/static_far_after/`、`moving_after/`、`dropout_after/`，以及 `runtime/dual/follow.log`。
