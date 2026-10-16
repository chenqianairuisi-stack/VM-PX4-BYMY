# 无人机 YOLO 数据集与训练

实际截图位于 `~/Desktop/drone_yolo_dataset`。共享训练包位于
`~/VM-shared/VM-PX4-BYMY/drone_training`（Windows 的 `VM-PX4-BYMY\drone_training`）。

这是 Gazebo Classic 用本项目 FPV 目标机网格渲染的相机截图，不是网络无人机照片。
只标注机身与旋翼，不包含气球和绳子。静态配对空背景和有目标画面做像素差分标注；
采集时移除目标阴影，避免把阴影框进去。空背景另存为负样本，标签为空文件。
14 个相机方向场景按 10/2/2 划分训练、验证、测试，每场景 50 张正样本和 7 张负样本。
不同姿态、2–14 m 距离、画面位置和高度随机化；固定种子记录在采集程序中。
这是首版仿真数据；未包含真实无人机、复杂遮挡、运动模糊和动态旋翼，不能据此承诺真实场景效果。

## Windows RTX 4060 训练

1. 使用宿主机现有的 Python / Conda YOLO 环境，且 PyTorch 能访问 NVIDIA GPU；不要求固定 Python 版本。
2. 打开共享文件夹的 `drone_training`，双击 `train_windows_gpu.cmd`。
3. 脚本查找 PATH、Python Launcher、Conda 和常见安装目录中的现有环境，验证 YOLO 导入和 CUDA 运算。
   选中可用环境后指定 `device=0` 训练。找不到完整环境时，利用现有 Miniconda 在
   `%LOCALAPPDATA%\PX4-FLY\drone-training-env` 创建独立 Python 3.11 环境，安装 CUDA 12.6 版
   PyTorch 2.7.1、Ultralytics 8.3.203 和导出依赖，不改动其他环境，也不会切换 CPU。
   首次需下载数 GB 依赖，安装过程记录在 `setup.log`；完成后自动开始训练。
   检查结果在 `environment_check.log`，选中环境记录在 `environment_probe.json`。
   若环境安装在自定义路径，可在 `python_path.txt` 写入完整的 `python.exe` 路径后重试。
4. 默认 YOLOv8n，100 epochs，640 分辨率，batch 8；早停 patience 25。
   训练日志在 `training.log`，图表在 `runs/drone*`，最终结果在 `trained_model`。
5. `trained_model/drone.pt` 是训练后的权重；`drone.onnx` 是 416 输入的虚拟机推理模型。
   `metrics.json` 是独立 test 集的指标。基础 `yolov8n.pt` 只是 COCO 预训练初始化，
   本身不是无人机专用模型，不能拿它替代训练结果。

不要同时启动多个训练窗口。导出 ONNX 需要可用的 onnx 包，Ultralytics 可能联网安装导出依赖。
可在现有 YOLO 环境手动调整：`python train_drone.py --data dataset --epochs 100 --batch 8`。
显存不足时减小 `--batch`，不要改成 CPU。

## 虚拟机导入与跟随

```bash
cd ~/Desktop/PX4-FLY
./scripts/import_drone_model.sh
./scripts/start_all.sh --single-terminal
```

目标键盘按 T 起飞；默认无人机模式加载机身模型、显示识别框，追踪机延迟起飞并搜索。
跟随参数位于 `config/drone_follow.yaml`：静止目标跟随间距设为 6 m，闭合速度 2.8 m/s、总水平速度上限 5 m/s；目标运动时会按当前速度预留制动距离，减少追踪机冲过目标的风险。
单目通过约 0.60 m 的机体横向尺寸估距，姿态变化会引入误差；6 m 是静止目标的控制设定值，不是实测最小间距保证。
无人机模式不会响应气球接触事件。本机已经完成首轮 SITL 起飞、静止跟随、目标机移动及双机降落测试。
高速回归测试中目标机峰值水平速度 3.31 m/s、追踪机 4.01 m/s；跟随阶段锁定率 92.1%，最小实测间距 4.16 m，最终间距 6.57 m，两机均正常降落并锁定。该数据只代表当前仿真工况，不是任意急转或遮挡下的安全保证。
预测横向回归中目标先接近再横移并反向，锁定跟随率 100%，最长丢锁 0 s，最小间距 3.03 m，最终间距 6.69 m；其中 6 帧使用了已关联目标的 bearing-only 更新。结果在 `runtime/predictive_lateral_third/result.json`。

虚拟机运行的是 CPU ONNX 推理。游戏本的 RTX 4060 未直通到 VMware，训练在 Windows。
`./scripts/setup_yolo_inference.sh` 可重建虚拟机的隔离推理环境。
`YOLO_MODEL=/absolute/path/drone.onnx ./scripts/yolo_detector.sh` 可指定模型。
缺少模型时一键启动会提前退出，不会继续打开不工作的跟随任务。

旧的红球实验可用 `FOLLOW_MODE=balloon ./scripts/start_all.sh --single-terminal`。

## 重新采集

```bash
./scripts/collect_drone_dataset.sh --output ~/Desktop/drone_yolo_dataset_new --per-scene 50
```

采集使用独立 ROS_DOMAIN_ID=55 / Gazebo 11355 端口，不启动 PX4、不操控飞行器。
复现参数和框像素坐标在 `capture_manifest.json`。标签为 YOLO 格式，类别 0 = drone。
`data.yaml` 保持可移植，训练脚本运行时生成绝对路径的 `data.local.yaml`。
`preview.jpg` 用于查看部分标注；`dataset_report.json` 记录数量、范围和文件匹配检查。

## 已完成的模型训练与部署评估（2026-10-12）

- GPU：NVIDIA GeForce RTX 4060 Laptop GPU。
- Windows 独立 test 集（640 输入）：precision 0.9994、recall 1.0、mAP50 0.995、mAP50-95 0.8072。
- 虚拟机部署 ONNX（416 输入、confidence 0.55）：114 张图，100 个目标，90 个框 IoU≥0.5，
  10 个漏检/未匹配目标、1 个未匹配预测，匹配前有效预测平均 IoU 0.881；CPU 平均推理约 43 ms。
  这组固定阈值结果与 Windows 的 mAP 不是同一指标。
- ROS 相机和推理一起运行时帧率更低，取决于虚拟机渲染和调度。
- 模型文件：`weights/drone.pt`、`weights/drone.onnx`；训练结果：`weights/metrics.json`。
- 导入后修复了 NumPy bool 与 ROS bool 的兼容问题，54 项回归测试通过。
- 仿真验证程序：`scripts/verify_drone_follow.py`，在双机仿真和 YOLO 已启动、没有其他键盘/跟随节点时运行。
  结果存入 `runtime/drone_follow_verification`，仿真真值仅用于记录和测试退出条件，不传给跟随器。

早期 4 m 间距配置从冷启动验证：目标移动 1.53 m，追踪机移动 1.76 m，30 秒观测阶段的采样点
均处于锁定跟随状态；最小实测间距 3.51 m，最终间距 4.36 m，双机正常降落并自动锁定。
随后完成高速双机回归：目标移动 16.79 m，追踪机移动 16.70 m，接受观测 459 帧，平均测距绝对误差 0.53 m；结果在 `runtime/drone_follow_verification/result.json`，完整日志在 `runtime/fast_follow_trial_fourth.log`。

2026-10-13：提速版本的 4 个 ROS 包构建通过，56 项回归测试通过。高速试验为目标连续直线移动后停止，复现时先启动双机仿真和单个 YOLO 检测节点，再加载 ROS 环境运行 `python3 scripts/verify_drone_follow.py --fast`；不要同时运行键盘或跟随节点。该脚本会操控两架仿真飞机，并在结束时请求降落。
