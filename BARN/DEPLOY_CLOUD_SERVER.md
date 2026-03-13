# 云服务器部署指南：BARN + DDR-opt

在云服务器上无图形界面（headless）运行 BARN 评测与 DDR-opt 导航栈的完整要求与步骤。

---

## 一、服务器系统要求

### 1. 操作系统

| 项目 | 要求 |
|------|------|
| **推荐系统** | **Ubuntu 20.04 LTS**（ROS Noetic）或 **Ubuntu 18.04 LTS**（ROS Melodic） |
| 架构 | x86_64（amd64） |
| 建议 | 使用纯净 Ubuntu Server，避免与已有 ROS/Gazebo 版本冲突 |

### 2. 硬件建议

| 资源 | 最低 | 推荐（单次跑一个 world） |
|------|------|---------------------------|
| **CPU** | 2 核 | 4 核及以上 |
| **内存** | 4 GB | 8 GB 及以上（Gazebo + ROS 较吃内存） |
| **磁盘** | 10 GB 可用 | 20 GB+（系统 + ROS + Gazebo + 工作空间） |
| **GPU** | 不需要 | 可选，Gazebo 无头模式不依赖 GPU |

说明：云上无显示器时务必使用 **无头模式**（不启动 Gazebo GUI），否则可能因缺少显示而报错。

### 3. 网络

- 需能访问外网（`apt`、`pip`、`rosdep`、部分 git 仓库）。
- 若使用公司/校园内网，请确保可访问 `raw.githubusercontent.com`、`github.com`、ROS 与 Ubuntu 软件源。

---

## 二、软件依赖一览

- **ROS**：Melodic（Ubuntu 18.04）或 Noetic（Ubuntu 20.04）
- **Gazebo**：与 ROS 配套（如 `gazebo_ros_pkgs`），通常随 ROS 安装
- **CMake** ≥ 3.0.2
- **Python** ≥ 3.6（Noetic 默认 Python3；Melodic 需确认 `python3` 可用）
- **Python 包**：`defusedxml`、`rospkg`、`netifaces`、`numpy`
- **ROS 包**：`laser_geometry`、`tf2_sensor_msgs`、以及 BARN/Jackal 的 `rosdep` 依赖
- **DDR-opt**：OSQP、OSQP-Eigen（见下文）

---

## 三、部署步骤

### 步骤 0：准备环境（可选但推荐）

```bash
# 更新系统
sudo apt update && sudo apt upgrade -y

# 若担心无显示器导致 GUI 相关错误，可装 Xvfb（虚拟显示）
sudo apt install -y xvfb
# 之后可用：xvfb-run -a python3 run.py --world_idx 0 --nav_stack ddr_opt
```

### 步骤 1：安装 ROS

**Ubuntu 20.04（ROS Noetic）：**

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Ubuntu 18.04（ROS Melodic）：**

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y ros-melodic-desktop-full
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 步骤 2：安装 Gazebo 与 ROS 依赖

Noetic：

```bash
sudo apt install -y \
  ros-noetic-gazebo-ros-pkgs \
  ros-noetic-gazebo-ros-control \
  ros-noetic-robot-state-publisher \
  ros-noetic-joint-state-publisher \
  ros-noetic-xacro \
  ros-noetic-tf2-sensor-msgs \
  ros-noetic-laser-geometry \
  ros-noetic-move-base \
  ros-noetic-navfn \
  ros-noetic-base-local-planner
```

Melodic 将上面命令中的 `noetic` 改为 `melodic` 即可。

### 步骤 3：安装 OSQP 与 OSQP-Eigen（DDR-opt 需要）

```bash
# 安装 OSQP
cd /tmp
wget https://github.com/osqp/osqp/releases/download/v0.6.3/osqp-v0.6.3.tar.gz
tar -xzf osqp-v0.6.3.tar.gz && cd osqp-v0.6.3
mkdir build && cd build
cmake -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX:PATH=/usr/local ..
make -j$(nproc)
sudo make install
cd /tmp

# 安装 OSQP-Eigen
wget https://github.com/robotology/osqp-eigen/archive/refs/tags/v0.8.1.tar.gz -O osqp-eigen-0.8.1.tar.gz
tar -xzf osqp-eigen-0.8.1.tar.gz && cd osqp-eigen-0.8.1
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX:PATH=/usr/local ..
make -j$(nproc)
sudo make install
```

### 步骤 4：Python 依赖

```bash
pip3 install --user defusedxml rospkg netifaces numpy
# 若用虚拟环境，先激活再执行上述命令
```

### 步骤 5：创建工作空间并拉取代码

请将 `JACKAL_WS` 和 `BARN_REPO`、`DDR_OPT_REPO` 替换为你实际路径（或先克隆到某目录再建工作空间）。

```bash
export ROS_DISTRO=noetic   # 18.04 用 melodic
mkdir -p ~/jackal_ws/src
cd ~/jackal_ws/src

# BARN 相关（按你实际仓库调整）
# 若你已有本仓库，可把 BARN 和 DDR-opt 拷贝到 src 下，或使用 git clone
git clone https://github.com/jackal/jackal.git --branch ${ROS_DISTRO}-devel
git clone https://github.com/jackal/jackal_simulator.git --branch ${ROS_DISTRO}-devel
git clone https://github.com/jackal/jackal_desktop.git --branch ${ROS_DISTRO}-devel
git clone https://github.com/utexas-bwi/eband_local_planner.git

# BARN challenge 仓库（若你用的是 the-barn-challenge，则克隆；否则把本地 BARN 的 pipeline 等拷贝到 src 下）
# git clone https://github.com/Daffan/the-barn-challenge.git

# DDR-opt（克隆或拷贝到 src 下）
# git clone https://github.com/ZJU-FAST-Lab/DDR-opt.git
```

**若你在本机已有完整项目**（BARN + DDR-opt + barn_ddr_bridge），可在服务器上：

- 用 `git clone` 你的私有仓库，或  
- 用 `scp`/`rsync` 把整个 `benchmark` 目录同步到服务器，再把 BARN 的 `pipeline`（含 `jackal_helper`、`barn_ddr_bridge`）和 DDR-opt 各包放到 `~/jackal_ws/src` 下，保证目录结构一致。

示例（按你实际路径改）：

```bash
# 假设你把 isaaclab 项目同步到了服务器
cd ~/jackal_ws/src
ln -s /path/to/isaaclab/benchmark/BARN/pipeline/jackal_helper   .
ln -s /path/to/isaaclab/benchmark/BARN/pipeline/barn_ddr_bridge .
# 以及 BARN 的 worlds、run.py 等；DDR-opt 的 plan_manager、mpc_controller 等
# 或直接拷贝整个 BARN/pipeline 和 DDR-opt 到 src
```

### 步骤 6：安装 rosdep 依赖并编译

```bash
cd ~/jackal_ws
source /opt/ros/${ROS_DISTRO}/setup.bash

# 若未初始化过 rosdep（仅第一次）
sudo rosdep init
rosdep update

rosdep install -y --from-paths src --ignore-src --rosdistro=${ROS_DISTRO}
catkin_make
# 或：catkin build
source devel/setup.bash
```

若编译报错，可尝试将 `jackal_helper/CMakeLists.txt` 中 `-std=c++11` 改为 `-std=c++17` 再 `catkin_make`。

### 步骤 7：无头运行测试（不打开 Gazebo 界面）

**仅 BARN + DWA（默认）：**

```bash
cd ~/jackal_ws
source devel/setup.bash
cd src/<你的 BARN pipeline 目录>   # 即包含 run.py 的目录

# 无头、world 0、默认 DWA
python3 run.py --world_idx 0 --out out.txt
```

**BARN + DDR-opt：**

```bash
python3 run.py --world_idx 0 --nav_stack ddr_opt --out out.txt
```

若仍有与显示相关的错误，可加虚拟显示：

```bash
xvfb-run -a python3 run.py --world_idx 0 --nav_stack ddr_opt --out out.txt
```

成功时终端会打印类似：`Navigation succeeded with time ...` 或 `Navigation collided ...` / `Navigation timeout ...`。

### 步骤 8：批量测试与报告（可选）

若你有 `test.sh` 和 `report_test.py`：

```bash
# 按你仓库的 test.sh 用法执行，例如 50 个 world、每个 10 次
bash test.sh

# 生成报告
python3 report_test.py --out_path res/ddr_opt_out.txt
```

---

## 四、常见问题

### 1. Gazebo 启动报错 / 无显示

- 不要加 `--gui`，在云服务器上始终使用无头模式。
- 若仍有 `DISPLAY` 相关错误，使用：`xvfb-run -a python3 run.py ...`。

### 2. `rosdep install` 失败

- 检查网络与软件源；可临时换国内 ROS 镜像再 `rosdep update`。
- 若某个包找不到，可手动 `apt install ros-${ROS_DISTRO}-<包名>`。

### 3. DDR-opt 编译找不到 OSQP / OSQP-Eigen

- 确认 OSQP、OSQP-Eigen 已安装到 `/usr/local`，并重新 `source /usr/local` 或确保 CMake 能找到它们。
- 若用非默认路径，需在 DDR-opt 的 CMake 中指定 `OSQP_DIR`、`osqp-eigen` 路径。

### 4. 找不到 `barn_ddr_bridge` 或 `plan_manager`

- 确认工作空间 `src` 下同时包含 BARN 相关包与 DDR-opt 相关包，且已执行 `catkin_make` 和 `source devel/setup.bash`。
- 用 `rospack find barn_ddr_bridge`、`rospack find plan_manager` 检查是否在同一个 workspace。

### 5. 内存不足 / 进程被 kill

- 关闭其他占用内存的服务；或换更大内存实例。
- 先只跑单个 world 确认流程，再考虑批量。

---

## 五、一键检查脚本（可选）

在项目根目录或 pipeline 目录下保存为 `check_server_env.sh`，用于快速检查环境：

```bash
#!/bin/bash
echo "=== ROS ==="
source /opt/ros/noetic/setup.bash 2>/dev/null || source /opt/ros/melodic/setup.bash 2>/dev/null
rosversion -d
echo "=== Gazebo ==="
gzserver --version 2>/dev/null || echo "gzserver not found"
echo "=== Python ==="
python3 --version
pip3 list | grep -E "rospkg|numpy|defusedxml|netifaces" || true
echo "=== Workspace ==="
source ~/jackal_ws/devel/setup.bash 2>/dev/null && rospack find barn_ddr_bridge 2>/dev/null && rospack find plan_manager 2>/dev/null && echo "Workspace OK" || echo "Source workspace or add packages"
```

运行：`bash check_server_env.sh`。

---

按上述系统要求与步骤，即可在云服务器上无图形界面稳定运行 BARN 与 DDR-opt 测试。若你提供当前仓库在服务器上的实际路径（例如 BARN 和 DDR-opt 各在哪个目录），我可以按你的结构写一份更贴合的 `步骤 5` 拷贝/克隆命令。
