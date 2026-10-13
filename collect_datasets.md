# Franka 数据采集流程

## 1. 采集前检查

- Franka、夹爪、四台 RealSense 和遥操作器均已连接。
- 六维力传感器位于 `/dev/ttyACM0`。
- 四台相机依次使用端口 `5000`（wrist front）、`5001`（wrist back）、`5002`（base）、`5003`（local）。请确认相机枚举顺序与安装位置一致。
- 所有命令从项目的 `frankateleop` 目录运行：

```bash
cd /home/xxg/projests/franka/frankateleop
```

可以在启动机械臂节点之前单独检查六维力传感器：

```bash
conda activate polymetis
python teleop/scripts/read_force_sensor.py
```

确认六个方向的数据正常后按 `Ctrl+C` 退出。检查程序和机器人节点不能同时占用同一个串口。

## 2. 启动服务

分别打开终端，按以下顺序启动；每个进程都需要保持运行：

```bash
# 终端 1：Franka 服务
./1_launch_robot.sh


# 终端 2：夹爪服务
./2_launch_gripper.sh

# 终端 3：四台相机服务
conda activate polymetis
python teleop/experiments/launch_camera_nodes.py --hostname=127.0.0.1

# 终端 4：机器人节点，同时读取 /dev/ttyACM0 六维力传感器
./3_launch_node.sh

# 终端 5：遥操作和数据保存
./4_run_env.sh
```

相机节点默认开启 RGB 和深度。仅需 RGB 时，将终端 3 的命令替换为：

```bash
conda activate polymetis
python teleop/experiments/launch_camera_nodes.py --hostname=127.0.0.1 --no-depth
```

`--no-depth` 对本次启动的全部相机生效：不启用深度流，也不读取、传输或保存深度图。
`4_run_env.sh` 读取 5000～5003 四个端口，并自动根据相机响应决定是否保存深度：

| 端口 | RGB 键名 | 深度键名（仅开启深度时保存） |
|---|---|---|
| 5000 | `wrist_rgb_front` | `wrist_depth_front` |
| 5001 | `wrist_rgb_back` | `wrist_depth_back` |
| 5002 | `base_rgb` | `base_depth` |
| 5003 | `local_rgb` | `local_depth` |

RGB-only 模式下只保存上表的 4 个 RGB 图像字段，没有深度字段，也不写入 `None` 或占位深度图。
启动 `4_run_env.sh` 前，需要四路相机节点都已启动并保持运行。
需要深度时省略 `--no-depth` 或使用 `--depth`。切换模式需要停止并重新启动相机节点；建议在两次采集之间切换。

检查各端口对应的相机画面时，保持相机节点运行，在另一个 `polymetis` 终端执行：

```bash
python scripts/save_camera_frames.py
# 或指定需要截图的端口和保存目录
python scripts/save_camera_frames.py --ports 5000 5001 5002 5003 --output-dir ./camera_snapshots
```

脚本从每个端口依次保存一张 RGB 图片，命名为 `5000.png`、`5001.png` 等。
默认输出到 `scripts/camera_snapshots/<时间戳>/`，支持 RGB-D 和仅 RGB 两种模式。
某路连接超时会报告并继续处理其他端口；已有同名文件不会覆盖。

如果力传感器端口不是 `/dev/ttyACM0`，需要在 `3_launch_node.sh` 的启动命令末尾增加：

```bash
--force_sensor_port=/dev/ttyACM1
```

## 3. 开始和停止录制

在 `4_run_env.sh` 弹出的键盘窗口中操作：

- 按 `S`：先用最近约 0.5 秒（500 帧）的六维力均值标零，清空旧力觉历史，等待 50 帧新数据后新建 episode 并开始保存。按键前请让末端保持静止且不要施加外力。
- 完成一次演示后按 `Q`：停止当前 episode。
- 再次按 `S`：开始下一个 episode。
- 结束全部采集时，在各终端按 `Ctrl+C` 退出。

数据默认保存在：

```text
~/bc_data/teleop/MMDD_HHMMSS/*.pkl
```

每个 `.pkl` 是一帧，包含 wrist front、wrist back、base、local 四路 RGB（默认还包含对应深度）、机器人状态、控制指令，以及：

```text
ee_external_wrench = [Fx, Fy, Fz, Mx, My, Mz]
```

其中力的单位为 `N`，力矩的单位为 `Nm`。

`ee_external_wrench` 保存当前最新的一帧六维力，形状为 `(6,)`。

此外，每帧 observation 还保存以下原始力觉窗口，均保持传感器原始帧率，不进行降采样或插值：

- `ee_external_wrench_before_50frames`：观测时刻之前最近的连续 50 帧，形状 `(50, 6)`。
- `ee_external_wrench_between_observations`：上一次 observation 之后到本次 observation 为止的全部原始帧，归入本次 observation，形状 `(K, 6)`；第一次 observation 为空数组。
- `ee_external_wrench_history_1s`：观测时刻之前 1 秒内的全部原始帧，形状 `(N, 6)`。
- `ee_external_wrench_history_2s`：观测时刻之前 2 秒内的全部原始帧，形状 `(M, 6)`。

每行数据顺序均为 `[Fx, Fy, Fz, Mx, My, Mz]`。所有窗口都是纯历史数据，获取时不等待未来力觉帧，因此不会为力觉窗口额外阻塞控制循环。