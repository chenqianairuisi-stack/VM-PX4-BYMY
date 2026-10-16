# 虚拟交会验证

`virtual_contact` 是无接触的 Gazebo/PX4 SITL 实验。追踪机沿用视觉预测跟随，在独立仿真裁判报告进入交会区域后请求降落。普通 `drone` 跟随模式保持原有行为。

## 判定与运行

Gazebo 裁判检查两机 `base_link` 原点的三维距离不大于 0.5 m、两机高度均超过 1.8 m。无需目标位于相机视场内。该事件表示进入虚拟交会区域；模型碰撞仍由 Gazebo 物理引擎处理。

控制器在已解锁且处于活动状态时接受交会事件，随后请求 PX4 降落，不要求视觉锁定。Gazebo 真值不进入位置、速度或偏航制导。虚拟模式使用 `stand_off: 0.0`，取消目标周围的保持距离、径向退让和制动距离约束，继续朝目标接近；速度/偏航指令仍平滑。

在可靠视觉估计进入 1.6 m 时，冻结目标位置和速度，以匀速轨迹预测接近，期间忽略新的视觉检测。预测最多持续 1.5 秒，未收到交会事件则刹停并原地持续旋转搜索，恢复视觉后重新接近。0.5 m 是事件触发距离，不是停靠距离。参数在 `config/drone_virtual_contact.yaml`；普通跟随模式仍使用 `config/drone_follow.yaml`。

交互运行：

```bash
cd ~/Desktop/PX4-FLY
./scripts/start_virtual_contact.sh
```

终端按 `T` 起飞目标机；追踪机自动起飞、识别和接近。虚拟交会后追踪机自动请求降落；目标机仍由键盘控制，按 `X` 降落。

## 自动测试

先冷启动仿真，再启动一个 YOLO 节点；不要同时运行其他键盘或跟随控制节点：

```bash
# 仿真终端
FOLLOW_MODE=virtual_contact HEADLESS=1 ./scripts/start_dual_uav.sh

# 检测器终端
./scripts/yolo_detector.sh

# 测试终端
source /opt/ros/humble/setup.bash
source install/local_setup.bash
PYTHONNOUSERSITE=1 python3 scripts/verify_virtual_contact.py \
  --moving --output runtime/virtual_contact_moving
```

测试会操控两机，并在结束时请求两机降落。静止目标测试去掉 `--moving`，并使用独立输出目录和冷启动的仿真。

移动工况包含目标提前移动、视觉锁定后持续移动 3 个仿真秒、随后降低输入频率但继续移动直至触发事件。判定要求锁定后的目标位移至少 1 m、事件时目标水平速度至少 0.2 m/s、跟随或预测接近状态占比至少 85%，以及最终两机均降落上锁。距离和速度来自 Gazebo 评估数据，控制器不读取这些测试数据。

## 2026-10-14 旧版 3.5 m 阈值结果（不代表当前配置）

移动目标测试通过，未修改生产控制器或距离参数来完成该次验证。84 项现有回归测试通过。

| 指标 | 移动工况实测 |
|---|---:|
| 锁定后观测时长 | 6.54 仿真秒 |
| 目标 / 追踪机位移 | 11.48 / 19.05 m |
| 目标 / 追踪机峰值水平速度 | 3.23 / 4.83 m/s |
| 移动跟随阶段锁定率 | 100% |
| 虚拟交会时距离 | 3.489 m |
| 事件时目标 / 追踪机水平速度 | 0.247 / 1.244 m/s |
| 包含降落过程的最小间距 | 2.917 m |
| 两机降落上锁 | 是 |

记录见 `runtime/virtual_contact_moving/result.json`、`samples.json` 和保存的 Gazebo/PX4 日志。`source_manifest.json` 记录该次使用的代码、模型和配置哈希。

此前静止目标测试的虚拟交会距离约 3.501 m（ROS 采样有延迟；Gazebo 裁判记录为 3.498 m），全流程最小间距约 3.056 m，两机均降落上锁，记录在 `runtime/virtual_contact_verification/`。两次结果只覆盖这些轨迹；移动测试的事件发生在目标减速后的持续运动阶段，未验证高速状态下直接触发交会。
