# PX4-FLY

基于 **PX4 SITL、Gazebo Classic 和 ROS 2 Humble** 的无人机仿真与视觉跟随项目。

项目使用 PX4 官方 Iris 四旋翼模型，支持键盘控制、相机视觉检测、双机协同、红色目标识别和自主跟随。

## 功能特性

- PX4 SITL + Gazebo Classic 四旋翼仿真
- ROS 2 Offboard 键盘控制
- 起飞、降落、移动、旋转和姿态控制
- 机载 RGB 前视相机
- HSV 红色目标检测
- YOLO 检测器接口
- 双无人机仿真环境
- 视觉目标跟踪与自主跟随
- 气球接触检测与自动降落
- 自动化起飞、移动、转向和降落测试

## 技术栈

| 组件 | 版本 |
|---|---|
| Ubuntu | 22.04 |
| PX4 Autopilot | v1.14.x |
| Gazebo Classic | 11 |
| ROS 2 | Humble |
| Python | 3.10+ |
| DDS Agent | Micro XRCE-DDS Agent |

## 项目结构

```text
PX4-FLY/
├── config/                  # PX4 和 Gazebo 配置
├── datasets/                # 数据集说明
├── external/px4_msgs/      # PX4 1.14 ROS 2 消息
├── models/                  # Gazebo 无人机和目标模型
├── worlds/                  # 单机和双机仿真世界
├── px4_fly_control/         # ROS 2 控制与视觉节点
├── scripts/                 # 启动、构建和测试脚本
├── runtime/                 # 运行日志和测试结果
└── README.md
```

## 环境准备

请提前安装：

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11
- PX4 Autopilot 1.14.x
- `colcon`
- `ros-humble-image-tools`
- `ros-humble-cv-bridge`
- `ros-humble-gazebo-ros-pkgs`

PX4 默认路径为：

```text
~/PX4-Autopilot
```

如果 PX4 位于其他目录，可以通过环境变量指定：

```bash
export PX4_DIR=/path/to/PX4-Autopilot
export PX4_BUILD=$PX4_DIR/build/px4_sitl_default
```

## 获取项目

```bash
git clone <your-repository-url>
cd <your-repository>/PX4-FLY
git submodule update --init --recursive
```

如果 PX4 尚未编译：

```bash
cd ~/PX4-Autopilot
make px4_sitl_default sitl_gazebo-classic
```

## 编译 ROS 2 工作空间

```bash
cd PX4-FLY
./scripts/build.sh
```

编译完成后加载环境：

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```

## 单机仿真

### 1. 启动 PX4、Gazebo 和 DDS

```bash
cd PX4-FLY
./scripts/start_px4_gazebo.sh
```

### 2. 打开相机画面

在新的终端中运行：

```bash
cd PX4-FLY
./scripts/camera.sh
```

### 3. 启动键盘控制

```bash
cd PX4-FLY
./scripts/keyboard.sh
```

保持键盘控制终端处于焦点状态，等待状态信息正常后按 `T` 起飞。

## 键盘控制

| 按键 | 功能 |
|---|---|
| `T` | 解锁并起飞 |
| `W / S` | 前进 / 后退 |
| `A / D` | 左移 / 右移 |
| `R / F` | 上升 / 下降 |
| `J / L` | 左转 / 右转 |
| `I / K` | 前倾 / 后仰 |
| `U / O` | 左倾 / 右倾 |
| `Space` | 保持当前位置和航向 |
| `X` | 自动降落 |
| `H` 或 `?` | 显示帮助 |

结束仿真前建议先按 `X` 降落，确认无人机已经解除解锁，再按 `Ctrl-C` 退出。

## 双机视觉跟随

双机实验包含：

- 目标机：由键盘控制
- 追踪机：通过相机和视觉检测自动跟随
- 红色气球：作为视觉目标
- Gazebo 接触插件：判断目标是否被接触

### 一键启动

```bash
cd PX4-FLY
./scripts/start_all.sh
```

该命令会自动打开：

- Gazebo 上帝视角
- 目标机键盘控制
- 追踪机相机画面
- 红色目标检测
- 自主跟随节点

获得目标机键盘窗口焦点后：

```text
T：起飞
X：降落
```

### 分步启动

```bash
./scripts/start_dual_uav.sh
```

然后分别启动：

```bash
./scripts/dual_keyboard.sh
./scripts/dual_camera.sh
./scripts/red_detector.sh
./scripts/follow.sh
```

没有图形界面时可以使用：

```bash
./scripts/start_all.sh --headless
```

## 视觉检测

### 红色目标检测

红色检测器不需要训练模型：

```bash
./scripts/red_detector.sh
```

可以调整最小检测区域：

```bash
./scripts/red_detector.sh --ros-args -p min_area:=80.0
```

### YOLO 检测

安装 Ultralytics：

```bash
python3 -m pip install --user ultralytics
```

启动 YOLO 检测器：

```bash
ros2 run px4_fly_control yolo_detector --ros-args \
  -p model:=/absolute/path/to/drone.pt
```

检测结果统一发布到：

```text
/tracker/yolo/detection
```

消息类型：

```text
px4_fly_interfaces/msg/VisualDetection
```

## 常用 ROS 2 话题

| 话题 | 说明 |
|---|---|
| `/front_camera/image_raw` | 单机相机图像 |
| `/tracker/front_camera/image_raw` | 双机追踪机相机图像 |
| `/front_camera/camera_info` | 单机相机内参 |
| `/tracker/yolo/detection` | 视觉检测结果 |
| `/tracker/balloon/burst` | 气球接触事件 |
| `/fmu/*` | 目标机 PX4 话题 |
| `/px4_1/fmu/*` | 追踪机 PX4 话题 |

查看相机帧率：

```bash
ros2 topic hz /front_camera/image_raw
```

查看 PX4 本地位置：

```bash
ros2 topic echo /fmu/out/vehicle_local_position \
  --qos-reliability best_effort
```

## 自动化测试

启动单机仿真后，运行：

```bash
cd PX4-FLY
PYTHONNOUSERSITE=1 python3 scripts/smoke_test.py
```

测试内容包括：

- 通信和相机检查
- 自动起飞
- 前进移动
- 航向旋转
- 姿态倾斜
- 自动降落
- 相机图像有效性检查

测试结果保存在：

```text
runtime/verification/
```

运行单元测试：

```bash
colcon test --packages-select px4_fly_control
colcon test-result --verbose
```

## 常见问题

### PX4 或 Gazebo 端口冲突

确认旧的仿真进程已经退出：

```bash
pkill -f px4
pkill -f gzserver
pkill -f MicroXRCEAgent
```

然后重新启动仿真。

### 虚拟机运行卡顿

关闭 Gazebo GUI：

```bash
HEADLESS=1 ./scripts/start_px4_gazebo.sh
```

相机渲染仍然需要可用的图形环境，建议在虚拟机中开启 3D 加速。

### NumPy 或 OpenCV 版本冲突

运行脚本前设置：

```bash
export PYTHONNOUSERSITE=1
```

## 安全说明

本项目主要用于 PX4 SITL 和 Gazebo 仿真环境。

在连接真实无人机之前，请先确认：

- Offboard 失联保护参数
- 自动降落逻辑
- 解锁和降落流程
- 控制指令的坐标系和单位
- 仿真与真实硬件之间的差异

PX4、ROS 2、Gazebo 和 `px4_msgs` 遵循其各自项目的许可证。
```

另外我检查到当前仓库的 `.gitmodules` 位于 `PX4-FLY/.gitmodules`，而 Git 子模块配置通常应放在仓库根目录。上传前建议把它移动到根目录，否则别人执行 `git clone --recurse-submodules` 时可能无法自动拉取 `external/px4_msgs`。
