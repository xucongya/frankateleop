# Franka GELLO 遥操作环境配置文档（franka_hzk）

> 配置日期：2026-06-29
> 配置者环境：hzk 用户，与 ntz 隔离 2 （！！！！！！！！！！！！！！！！！！！重要，请使用自己的user，本文档使用的hzk作为user，后文中的hzk都要换成自己的user。！！！！！！！！！！！！！！！！！！！！！！！！！）
> 分支：`force-modality-collection`（力模态采集）
> 对本文档的操作有问题来找hzk，不要动ntz下面的文件和环境。！！！！！！！！！！
---

## 0. 背景与核心约束

- 项目原件 `/home/hzk/franka` 的**全部文件归属于另一个用户 `ntz`**（最早在 `/home/ntz/franka` 构建后整体拷贝过来的）。
- **硬性约束：绝不修改 ntz 的任何文件或环境，只能动 hzk 自己的。**
  - 因此**不能** `sudo chown`（改归属也算动 ntz 的文件）。
  - 解决办法：把整个项目**复制一份到 `/home/hzk/franka_hzk`**（hzk 所有），所有安装/编译都在副本上做。ntz 原件只读、绝不改。
- conda 环境装在 `/home/hzk/miniconda3`（hzk 私有），与 ntz 完全隔离。
- 最简单的就是直接拷贝硬盘再复制到自己的user下。
### 机器拓扑（重要）

| 角色 | 主机 | IP | 跑什么 |
|---|---|---|---|
| **客户端** | tzhRobot（hzk） | 192.168.1.101 | 本副本 franka_hzk；脚本 `3`（机器人节点）、`4`（遥操作+采集）、相机节点、GELLO 主臂 |
| **服务端** | ros（pnp） | 192.168.1.100 | `run_server` + 夹爪（脚本 `1`、`2`）；通过 libfranka 连机器人 FCI |
| 机器人 FCI | Franka FR3 | 172.16.0.2 | 由服务端 .100 直连 |

- `run_server` 在 .100 上绑定 `0.0.0.0:50051`；客户端 .101 用 `--robot_ip=192.168.1.100` 连它（**这是对的，别改成 127.0.0.1**）。

---

## 1. 完整配置步骤（按顺序）

### 1.1 安装 Miniconda（hzk 私有）
```bash
curl -fsSL -o /tmp/miniconda.sh https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash /tmp/miniconda.sh -b -p /home/hzk/miniconda3
/home/hzk/miniconda3/bin/conda init bash
# 新开终端或 source ~/.bashrc 生效
```

### 1.2 创建 polymetis conda 环境（最慢、最易出问题的一步）
```bash
source /home/xxg/miniconda3/etc/profile.d/conda.sh
# 新版 conda 对 defaults 频道要先接受 ToS，否则直接报错：
conda tos accept --override-channels \
  --channel https://repo.anaconda.com/pkgs/main \
  --channel https://repo.anaconda.com/pkgs/r
# 用项目自带 environment.yml 建环境（Python 3.8 / torch 1.13.1 / numpy 1.23.5 等一堆钉死的旧版本）
conda env create -f /home/xxg/projects/franka_xxg//frankateleop/polymetis/polymetis/environment.yml
```
- 解算 + 下载大约几分钟（habitat-sim 236MB、pytorch 等）。libmamba 求解器能解开，无版本冲突。

### 1.3 复制项目到 franka_hzk（隔离）
```bash
rsync -a \
  --exclude 'libfranka/build/' \
  --exclude 'frankateleop/polymetis/polymetis/build/' \
  /home/xxg/projects/franka/ /home/xxg/projects/franka_xxg/
# 注：保留 .git（polymetis 版本号要用 git）；两个 build 目录排除掉，后面重建
# 复制后所有文件归 hzk，0 个 ntz 文件
```

### 1.4 重建 libfranka（系统工具链 + 系统 Eigen/Poco）
```bash
cd /home/xxg/projects/franka_xxg//libfranka
# 关键：libfranka 的 SetVersionFromGit.cmake 要求本目录有独立 .git 且带版本 tag，
# 否则报 "cannot find a tag in git"。手动初始化并打 0.19.0 标签（和原 .so 版本一致）：
git init -q
git -c user.email="setup@hzk" -c user.name="hzk-setup" commit -q --allow-empty -m "libfranka 0.19.0"
git tag 0.19.0

mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF \
  -DCMAKE_C_COMPILER=/usr/bin/cc -DCMAKE_CXX_COMPILER=/usr/bin/c++ \
  -DEigen3_DIR=/usr/share/eigen3/cmake \
  -DPoco_DIR=/usr/lib/x86_64-linux-gnu/cmake/Poco ..
make -j"$(nproc)"
# 产物：libfranka.so.0.19.0，以及 FrankaConfig.cmake（路径全部指向 franka_hzk，隔离成立）
```

### 1.5 安装 polymetis Python 包
```bash
conda activate polymetis
cd /home/xxg/projects/franka_xxg/frankateleop/polymetis/polymetis
pip install -e .
```

### 1.6 编译 polymetis C++（含 run_server）
```bash
conda activate polymetis
cd /home/xxg/projects/franka_xxg/frankateleop/polymetis/polymetis
mkdir -p build && cd build
cmake -DBUILD_FRANKA=ON \
      -DFranka_DIR=/home/xxg/projects/franka_xxg/libfranka/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_C_COMPILER=/usr/bin/cc -DCMAKE_CXX_COMPILER=/usr/bin/c++ ..
make -j"$(nproc)"
# 产物：run_server、franka_panda_client、franka_hand_client、torch_isolation/*.so
```
⚠️ 工程师手册里那条 cmake 有笔误，**别照抄**：
- `Dfranka_DIR=...` → 应为 `-DFranka_DIR=...`（少了 `-D`，且 `Franka` 首字母大写）
- `-dCMAKE_BUILD_TYPE=Release` → 应为 `-DCMAKE_BUILD_TYPE=Release`（大写 `-D`）
- 若写错，`Franka_DIR` 不生效会回退到**空的子模块路径**，`find_package(Franka)` 直接失败。

### 1.7 安装 teleop + DynamixelSDK + requirements
```bash
conda activate polymetis
cd /home/xxg/projects/franka_xxg/frankateleop/teleop
pip install -e .
pip install -e third_party/DynamixelSDK/python

# requirements 里 dm_control / numpy-quaternion 可能升级 numpy，破坏 polymetis 的 C++ 扩展，
# 必须锁定 numpy：
echo "numpy==1.23.5" > /tmp/constraints.txt
pip install -r requirements.txt -c /tmp/constraints.txt

# pyspacemouse 2.0 用了 Py3.10 才有的 dataclass(slots=)，Py3.8 装会崩，降级：
pip install "pyspacemouse<2"

# OpenCV（requirements 里没列，相机节点/数据处理要用）：
pip install opencv-python      # 若报 libGL 缺失，改用 opencv-python-headless
```

---

## 2. 遇到的问题与解决办法（踩坑记录）

### ① git "dubious ownership"
- 现象：任何 git 命令报 `detected dubious ownership in repository`。
- 原因：原件归 ntz，当前是 hzk。
- 处理：勘察阶段用 `git -c safe.directory='*' ...` 只读绕过；副本 franka_hzk 归 hzk，无此问题。

### ② Permission denied 写不了 egg-info（最根本的坑）
- 现象：`pip install -e` 报 `Cannot update time stamp of directory 'polymetis.egg-info'`。
- 原因：源码树归 ntz，hzk 不在 ntz 组，无写权限。
- 处理：**这就是必须复制到 franka_hzk 的根本原因**。不能 chown（会动 ntz）。

### ③ conda defaults 频道 ToS
- 现象：`conda env create` 立即报需要接受 Terms of Service。
- 处理：先 `conda tos accept ...`（见 1.2）。

### ④ libfranka "cannot find a tag in git"
- 现象：cmake 配置 libfranka 时 `SetVersionFromGit.cmake:29` 报错。
- 原因：它把 `GIT_DIR` 写死成 `<libfranka>/.git`，要求 libfranka 自己是带 tag 的 git 仓库；rsync 没有这个独立 .git。
- 处理：在 libfranka 内 `git init` + 空提交 + `git tag 0.19.0`（见 1.4）。

### ⑤ cmake 命令笔误（来自工程师手册）
- `Dfranka_DIR` / `-dCMAKE_BUILD_TYPE` 都是错的，详见 1.6。

### ⑥ `import polymetis` 警告 "Failed to load libtorchscript_pinocchio.so from CONDA_PREFIX"
- **无害**。我们没把 .so 装进 CONDA_PREFIX，它会自动回退到 build 目录加载，功能正常。ntz 当初也是这样。

### ⑦ numpy ABI 风险
- 装 requirements 时若 numpy 被升到 ≠1.23.5，polymetis 的 C++ 扩展会崩。务必用约束文件锁定（见 1.7）。

### ⑧ pyspacemouse 2.0 与 Python 3.8 不兼容
- 现象：`TypeError: dataclass() got an unexpected keyword argument 'slots'`。
- 处理：`pip install "pyspacemouse<2"`（装到了 1.1.5）。仅影响 `--agent=spacemouse`。

### ⑨ oculus_reader 缺失（可选）
- 现象：`--agent=quest` 时 `ModuleNotFoundError: No module named 'oculus_reader'`。
- 说明：Quest VR 手柄才用，不在 requirements 里，**主力 GELLO 遥操作不需要**。需要再单独装。

### ⑩ mujoco 被升级 2.2.0 → 3.2.3
- dm_control 拉的，**只影响仿真**，真机不用，不必处理。

### ⑪ gRPC "failed to connect to all addresses"（VPN/代理坑，排查最久）
- 现象：跑 `3_launch_node.sh` 报 `StatusCode.UNAVAILABLE, failed to connect to all addresses`；但裸 TCP 测 `192.168.1.100:50051` 又是通的。
- 根因：本机开着 **Clash/VPN 代理**（`http_proxy=https_proxy=http://127.0.0.1:7897`），而 `no_proxy` 里**没有** 192.168.1.100。gRPC 尊重 http_proxy，于是把"连局域网机器人服务端"也丢给代理转发 → 失败。
- **和 VPN 开不开无关**，是 `no_proxy` 名单没带机器人 IP（ntz 那边能连正是因为他的 no_proxy 带了这个网段）。
- 处理：把机器人服务端 IP 加进 `no_proxy`。已写进 `franka_hzk/frankateleop/3_launch_node.sh`：
  ```bash
  export no_proxy="192.168.1.100,127.0.0.1,localhost,::1${no_proxy:+,${no_proxy}}"
  export NO_PROXY="$no_proxy"
  ```
  验证：加上后 gRPC 立刻连通，拿到 `hz=1000`。

### ⑫ GELLO 主臂串口 Permission denied
- 现象：`SerialException: [Errno 13] could not open port /dev/serial/by-id/usb-FTDI_..._FTBM7TAW-...: Permission denied`。
- 原因：串口设备属 `root:dialout`，hzk 不在 dialout 组。
- 处理（二选一）：
  ```bash
  sudo usermod -aG dialout hzk     # 永久，需重新登录生效（推荐）
  # 或临时：
  sudo chmod a+rw /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBM7TAW-if00-port0
  ```

### ⑬ import cv2 / OpenCV
- 没装：`pip install opencv-python`。
- 装了但报 `libGL.so.1: cannot open shared object file`（无显示器常见）：改 `opencv-python-headless`，或 `sudo apt install -y libgl1`。
- 这里可能出现在使用./4_run_env.sh的时候卡在需要/dev/serial/by-id/usb-FTDI_..._FTBM7TAW-...: 连接不上，本质上是opencv的库除了问题，而不是主臂连接不上。

### ⑭ run_env 卡死在 camera.read()（常被误判成 cv2 问题）
- 现象：`4_run_env.sh` 卡住，traceback 停在 `camera.read() → socket.recv()`，只能 Ctrl+C。
- 根因：**相机节点没起来**，`run_env` 连不到端口 5000/5001 就一直阻塞。**不是 cv2 的问题**。
- `run_env.py` 非 mock 模式**写死要连两个相机**（wrist=5000、base=5001）。
- 处理：
  - 有 RealSense → 先在另一终端启动相机节点：
    ```bash
    conda activate polymetis
    cd ~/franka_hzk/frankateleop
    python teleop/experiments/launch_camera_nodes.py --hostname 127.0.0.1
    ```
    ⚠️ 该脚本默认 hostname 是**残留的伯克利外网 IP `128.32.175.167`**，必须用 `--hostname 127.0.0.1` 覆盖，否则绑不上端口。需接够 2 个相机（端口从 5000 顺序分配）。
  - 不想用相机先验证遥操作 → 需改 `run_env.py` 让相机可选（写死了，要手动注释/加开关）。

### ⑮ 目录用错
- 现象：从 `~/franka`（ntz 原件）跑脚本。
- 注意：**所有脚本统一在 `~/franka_hzk/frankateleop` 下跑**。`no_proxy` 等修复只在 franka_hzk 的脚本里，ntz 那份没有。（不过 Python 包是 editable 装的，import 仍解析到 franka_hzk。）

---

## 3. 启动使用流程

**服务端 .100（pnp@ros）**——先起：
```bash
./1_launch_robot.sh     # run_server + franka 客户端，停在 "Connected." 别关
./2_launch_gripper.sh   # 夹爪
```

**客户端 .101（hzk@tzhRobot），都在 ~/franka_hzk/frankateleop 下**：
```bash
# （首次）确保 dialout 权限已生效；如需相机另开终端跑 launch_camera_nodes.py --hostname 127.0.0.1
./3_launch_node.sh      # 机器人 ZMQ 节点（已内置 no_proxy 修复） 等待出现到timed out 在进行下一步

    ```bash
    conda activate polymetis
    cd ~/franka_hzk/frankateleop
    python teleop/experiments/launch_camera_nodes.py --hostname 127.0.0.1 # 打开相机节点，出现到timed out再进行下一步

    ```
./4_run_env.sh          # GELLO 遥操作 + 数据采集（--use-save-interface）
```

启动顺序：服务端 `1`→`2`，客户端（相机节点→）`3`→`4`。

---

## 4. 关键信息速查

| 项 | 值 |
|---|---|
| 项目副本 | `/home/hzk/franka_hzk` |
| conda | `/home/hzk/miniconda3`，env `polymetis`（Python 3.8.0） |
| 关键版本 | torch 1.13.1 / numpy 1.23.5（锁定）/ mujoco 3.2.3 / grpcio 1.46.0 / hydra-core 1.0.6 |
| 编译器 | 系统 `/usr/bin/c++`（gcc 12.3.0） |
| libfranka | 0.19.0 → `/home/hzk/franka_hzk/libfranka/build` |
| Franka_DIR | `/home/hzk/franka_hzk/libfranka/build` |
| run_server | `/home/hzk/franka_hzk/frankateleop/polymetis/polymetis/build/run_server` |
| 服务端 gRPC | `192.168.1.100:50051`（绑 0.0.0.0） |
| 机器人 FCI | `172.16.0.2`（由 .100 直连） |
| GELLO 主臂串口 | `/dev/serial/by-id/usb-FTDI_..._FTBM7TAW-...`（左）、`...FTAUMOPA...`（右） |
| 代理修复 | `no_proxy` 必须含 `192.168.1.100` |

---

## 5. 当前状态

- ✅ 环境配置完成，核心模块全部可导入；`run_env.py --mock` 控制循环跑通。
- ✅ gRPC 连接服务端正常（no_proxy 修复后 hz=1000）。
- ⏳ 待办：dialout 串口权限（用户执行 sudo）；相机节点启动（如需采集）；如只测遥操作可让相机可选。

