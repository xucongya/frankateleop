# SpaceMouse 遥操作与数据采集

本次仅新增入口脚本和 `teleop/teleop/spacemouse_collection/` 模块，已有 GELLO 采集代码不修改。

## 启动

沿用现有四路相机节点、Polymetis 机器人及夹爪底层服务的启动方式。相机端口仍为 `5000 / 5001 / 5002 / 5003`，机器人 IP 默认 `192.168.1.100`，机器人/夹爪服务端口为 `50051 / 50052`，力传感器仍为 `/dev/ttyACM0`。

四路相机需要独立启动，`5_launch_node_sm.sh` 和 `6_run_env_spacemouse.sh` 不会启动相机服务。如果相机节点尚未运行，先在独立终端中运行下面的命令，并保持该终端运行。当前仅采集四路 RGB，使用 `--no-depth`：

```bash
cd /home/xxg/projests/franka/frankateleop
conda activate polymetis
python teleop/experiments/launch_camera_nodes.py --hostname=127.0.0.1 --no-depth
```

等待四路相机均显示 `Starting camera server on port ...`，端口为 `5000`、`5001`、`5002`、`5003`。已经正常运行的相机服务不需要重复启动。

使用 SpaceMouse 时，在两个终端分别运行下面两个脚本，替代原来的 `3_launch_node.sh` 和 `4_run_env.sh`。两套遥操作入口不要同时运行，它们使用同一个 `6001` 端口和机器人。

终端一：

```bash
cd /home/xxg/projests/franka/frankateleop
bash 5_launch_node_sm.sh
```

这个节点复用原 FR3 初始化行为：机械臂回到 home、打开夹爪、启动关节阻抗控制。出现 `SpaceMouse node ready` 后，再启动终端二：

```bash
cd /home/xxg/projests/franka/frankateleop
bash 6_run_env_spacemouse.sh
```

脚本自动激活 `polymetis` conda 环境，通过脚本自身位置加载本项目代码；运行时不依赖 `/datapool-2/user/xxg/franka` 参考目录。

## 操作

鼠标六轴归一化后，采用参考实现的映射 `[y, x, -z, ry, rx, -rz]`。平移和旋转增量在机器人基坐标系下施加，末端定义沿用当前 Polymetis 模型。这里不引入参考工程额外的工具偏移或载荷补偿配置。

| 输入 | 功能 |
| --- | --- |
| 推拉 / 扭转鼠标 | 末端平移 / 旋转，通过逆运动学得到关节目标 |
| 鼠标左键 | 循环切换四档速度，初始为最低档 |
| 鼠标右键 | 切换夹爪关闭 / 打开 |
| 采集窗口内按 `S` | 力传感器清零，开始新一条采集 |
| 采集窗口内按 `Q` | 停止本条采集，仍可继续遥操作 |
| 终端按 `Ctrl+C` | 退出当前进程，机器人控制器保持最后的关节目标 |

速度系数为 `0.05 / 0.15 / 0.30 / 0.50`，对应轴阈值为 `0.48 / 0.45 / 0.42 / 0.40`。基础平移速度 `0.10 m/s`、基础旋转速度 `0.60 rad/s`；超过阈值的轴按正负方向产生增量，支持多轴同时操作。

| 档位 | 每个激活平移轴的目标速度 | 每个激活旋转轴的目标角速度 |
| --- | --- | --- |
| 1（初始） | 5 mm/s | 0.03 rad/s |
| 2 | 15 mm/s | 0.09 rad/s |
| 3 | 30 mm/s | 0.18 rad/s |
| 4 | 50 mm/s | 0.30 rad/s |

位移按实际循环间隔计算，计算使用的时间上限为 `0.1 s`，与 `--hz` 设置的循环频率上限分开。长时间阻塞后不累计补发全部位移；按 `S` 清零力传感器的等待时间也不计入下一次运动。实际执行仍受节点的单次关节、平移、旋转限幅及机械臂跟踪能力约束，多轴同时操作时还会受到增量范数限制。

采集循环上限默认沿用现有 GELLO 的 `100 Hz`。参考 SpaceMouse 工程使用 `20 Hz`，可以通过 `--hz 20` 选择；实际保存帧率还取决于四路图像读取、写盘等耗时。

## 数据保持一致

默认保存位置仍为：

```text
~/bc_data/teleop/MMDD_HHMMSS/<ISO时间戳>.pkl
```

新流程直接复用已有 `RobotEnv`、`fr3Robot.get_observations()`、力传感器清零/采样窗口和 `save_frame()`，保存顺序仍是当前观测加本次控制指令，再执行动作并读取下一帧。

- 四路 RGB 字段仍为 `wrist_rgb_front`、`wrist_rgb_back`、`base_rgb`、`local_rgb`。不调整颜色、分辨率或数组类型；相机关闭深度时，不添加深度字段。
- 保留 `joint_positions`、`joint_velocities`、`ee_pos_quat`、`gripper_position`，以及已有六个关节力矩字段。
- 保留 `ee_external_wrench`、`ee_external_wrench_before_50frames`、`ee_external_wrench_between_observations`、`ee_external_wrench_history_1s`、`ee_external_wrench_history_2s`。
- `control` 仍是 8 维：前 7 维为关节目标角度，最后一维为夹爪关闭比例（`0` 打开、`1` 关闭）。观测中的夹爪值仍是原来的打开比例，两者语义与已有代码一致。
- 不向数据中新增鼠标轴、速度档位或笛卡尔动作字段。鼠标产生的笛卡尔增量只用于求解关节目标，保存的是实际发送的 8 维指令。

## 稍后自行检查鼠标

列出可识别的 Linux input 设备：

```bash
bash 6_run_env_spacemouse.sh --list-devices
```

仅观察鼠标轴和按钮，不连接机器人/相机，也不保存数据：

```bash
bash 6_run_env_spacemouse.sh --dry-run --hz 20
```

默认通过 `/sys/class/input/event*/device/name` 自动识别 SpaceMouse，包括 Wireless BT。存在多个匹配设备时，用上面的列表选择实际路径：

```bash
bash 6_run_env_spacemouse.sh --device /dev/input/eventN --dry-run --hz 20
```

`eventN` 需替换成列表中的实际名称；重插或重启后编号可能变化。当前用户需要该设备的读取权限。遇到权限错误时，应为对应设备配置用户访问权限；不要直接用 root 运行整套采集。读取错误或事件溢出会终止采集进程。对于 Wireless BT 的绝对轴（EV_ABS），每次读取内核当前状态，保持偏转时不会因没有新事件而被错误清零；松手后跟随实际状态回零。相对轴（EV_REL）仍使用 0.25 秒无事件清零逻辑。

## 参数示例

```bash
# 更换机器人 IP 或力传感器端口
FR3_ROBOT_IP=192.168.1.100 FORCE_SENSOR_PORT=/dev/ttyACM0 bash 5_launch_node_sm.sh

# 自定义保存根目录；数据仍进入其 teleop 子目录
bash 6_run_env_spacemouse.sh --data-dir /datapool-2/raw_datasets --hz 20

# 自定义基础速度
bash 6_run_env_spacemouse.sh --translation-speed 0.10 --rotation-speed 0.60

# 查看完整参数
bash 5_launch_node_sm.sh --help
bash 6_run_env_spacemouse.sh --help
```

节点默认限制每次关节变化不超过 `0.015 rad`、平移增量范数不超过 `0.005 m`、旋转增量范数不超过 `0.03 rad`，关节限位留出 `0.05 rad`。IK 失败或目标触及限位时保持之前的关节目标。测量关节与目标关节的最大差值超过 `0.2 rad` 时暂停增加运动目标，并提示检查机器人状态。

按本次要求，仅完成代码添加，未运行测试、入口脚本或机器人实机验证。

## 启动时提示 Resource temporarily unavailable

如果在显示鼠标设备和 pygame 欢迎信息后、出现 `Ready` 前退出，旧版错误提示 `Resource temporarily unavailable` 可能来自相机请求超时。pygame 欢迎信息本身不是报错。

新版 SpaceMouse 入口会打印机器人连接状态，以及各路相机的名称和地址；相机超时会明确显示例如 `Camera wrist_front at tcp://127.0.0.1:5000 did not respond within 5 seconds`。先按上面的命令启动四路相机，再重新运行 `6_run_env_spacemouse.sh`。如果相机进程已经运行，查看其终端报错、绑定地址和端口配置。程序需要四路相机全部可读才会进入遥操作，不会跳过缺失图像进行采集。

## 已 Ready，但操作鼠标没有明显运动

先退出 6 号脚本，再运行 `bash 6_run_env_spacemouse.sh --dry-run --hz 20`。推动并保持鼠标上盖偏转，观察 `axes` 和 `dxyz` / `drot`；左、右按钮分别改变 `speed`、`gripper_closing`。`events(axis/button)` 为读到的累计轴事件/按钮事件数量，`peak_abs_axis` 为本次运行读到的最大归一化轴绝对值，用来区分没有读到轴输入和松手后的回零。只有按钮变化且轴值峰值一直为零时，平移/旋转输入尚未读到。

默认最低档阈值为 `0.48`，归一化轴值的绝对值必须大于该阈值才产生运动。初始档平移目标速度为每个激活轴 `5 mm/s`；例如实际循环间隔为 `0.05 s` 时，每次请求 `0.25 mm`。先根据输入输出定位，不要仅凭没有明显运动就提高速度。

需要查看完整控制链路时，退出 dry-run 后自行启动：

```bash
bash 6_run_env_spacemouse.sh --debug
```

这个命令会启用实际遥操作，并每约 0.5 秒打印一次诊断信息：`axes` 为映射后的鼠标轴，`dxyz_mm` / `drot_rad` 为本次请求的末端增量，`target_dq` 为相邻指令的最大关节目标变化，`measured_dq` 为相邻观测的最大实际关节变化，`tracking_error` 为目标与实测关节的最大差值（关节量均为 rad），`loop_hz` 为实际循环频率，`loop_dt_ms` 为本轮实际时间间隔，`motion_dt_ms` 为限幅后用于计算运动的间隔。`status` 会显示 IK 或限位拒绝原因。这些信息仅输出到终端，不改变采集数据。

## 最高档仍然很慢

旧版将运动时间限制为 `1 / hz`。设置 `--hz 100` 时，每次最多按 `0.01 s` 计算；如果四路图像和其他操作使实际循环只有 `20 Hz`，最高档每次只产生 `0.5 mm` 的单轴平移请求，相当于约 `10 mm/s`，低于设定的 `50 mm/s`。

现已改为使用实际循环间隔（最多 `0.1 s`）：在同样的 `20 Hz` 情况下，最高档每次请求约 `2.5 mm`，对应设定的 `50 mm/s`，之后仍经过原来的节点限幅。更新后只需退出并重新启动 6 号脚本，5 号节点和相机服务可以保持运行。采集循环上限、保存字段、观测/动作顺序和 8 维绝对关节动作格式保持原样。
