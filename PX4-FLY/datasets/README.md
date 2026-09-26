# YOLO 无人机数据集

把数据集放在本目录下，例如 `datasets/drone_yolo/`，然后告诉我完整路径即可。我不需要你把压缩包内容粘贴到聊天里。

推荐使用 YOLO 检测格式，只有一个类别 `drone`：

```text
drone_yolo/
  data.yaml
  images/
    train/
    val/
  labels/
    train/
    val/
```

每张图片对应一个同名 `.txt` 标签。每行格式为：

```text
class_id center_x center_y width height
```

五个值都归一化到 `0..1`。单类别时 `class_id` 永远是 `0`，例如：

```text
0 0.52 0.47 0.18 0.22
```

`data.yaml` 示例：

```yaml
path: /home/chen-qian/Desktop/PX4-FLY/datasets/drone_yolo
train: images/train
val: images/val
names:
  0: drone
```

建议覆盖不同距离、大小、偏航角、背景、光照、运动模糊和部分遮挡。仿真项目可以先录制 500 到 2000 张追踪机相机图像，再人工检查框；训练集和验证集按场景或时间段拆分，不要把连续相邻帧全部放进两个集合。

收到目录后，我可以检查标签、生成训练命令并把权重接到 `yolo_detector`。通用的 `yolov8n.pt` 可以用于验证管线，但不能保证识别你的自定义无人机。
