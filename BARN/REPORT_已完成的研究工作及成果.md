# 2．已完成的研究工作及成果

## 2.1 问题建模与描述

### 2.1.1 导航问题描述

本文研究地面移动机器人在已知或部分已知障碍环境中的自主导航问题。机器人从给定的起点出发，在满足运动学与动力学约束、且不与障碍物发生碰撞的前提下，在限定时间内到达目标点。该问题可概括为：在二维平面内，给定起点 \(S\)、终点 \(G\) 以及障碍物分布，为差分驱动机器人规划一条安全、可行的运动轨迹，使机器人能够从 \(S\) 到达 \(G\)，并在过程中保持与障碍物的安全距离。

与无人机编队不同，地面机器人通常具有非完整约束（如差分驱动只能沿车体朝向运动），因此需要在问题建模中显式考虑运动模型与约束。本文主要关注上层规划与局部轨迹跟踪，将底层电机控制抽象为对线速度与角速度指令的跟踪。

### 2.1.2 系统运动模型建模

本文采用的机器人平台为 Jackal 差速驱动机器人。差速驱动模型将机器人抽象为在平面内运动的刚体，其位形由位置 \((x, y)\) 与朝向角 \(\theta\) 描述。

**（1）运动学模型**

在平面内，设机器人质心坐标为 \((x, y)\)，朝向角为 \(\theta\)（与 \(x\) 轴夹角），线速度为 \(v\)，角速度为 \(\omega\)，则运动学方程为：

$$\dot{x} = v \cos\theta,\quad \dot{y} = v \sin\theta,\quad \dot{\theta} = \omega \tag{1}$$

该模型满足非完整约束 \(\dot{x}\sin\theta - \dot{y}\cos\theta = 0\)，即机器人不能横向侧移，只能沿车体朝向方向运动。

**（2）控制约束**

在实际控制中，\(v\) 与 \(\omega\) 需满足幅值约束，以反映电机与结构的物理限制：

$$|v| \leq v_{\max},\quad |\omega| \leq \omega_{\max} \tag{2}$$

在 BARN 的 Jackal 配置中，典型参数为 \(v_{\max} = 0.5\,\text{m/s}\)，\(\omega_{\max} = 1.57\,\text{rad/s}\)；加速度限制为 \(\dot{v}\)、\(\dot{\omega}\) 有界，在局部规划器中通过仿真步长与最大加速度参数体现。

**（3）离散化与仿真**

为便于规划与仿真，将式 (1) 在时间步长 \(\Delta t\) 下进行欧拉离散化，得到：

$$x_{k+1} = x_k + v_k \cos\theta_k \cdot \Delta t,\quad y_{k+1} = y_k + v_k \sin\theta_k \cdot \Delta t,\quad \theta_{k+1} = \theta_k + \omega_k \cdot \Delta t \tag{3}$$

在 Gazebo 中建立与上述运动学一致的仿真环境，机器人通过发布几何消息 `Twist`（即 \((v, \omega)\)）到 `/cmd_vel` 话题实现运动控制。

### 2.1.3 导航问题的 MDP 形式建模

为与后续基于学习的或基于优化的方法统一表述，可将导航问题形式化为马尔可夫决策过程（MDP）。定义如下：

- **状态空间 \(\mathcal{S}\)**：包含机器人当前位姿、速度信息以及局部环境表示（如激光雷达扫描、栅格代价地图或点云）。在 BARN 评测中，状态可由位姿与激光数据构成；若使用代价地图，则状态中还包含全局/局部代价地图的编码。
- **动作空间 \(\mathcal{A}\)**：对应控制输入 \((v, \omega)\)，可为连续空间或离散化后的若干动作；在 DWA 中为在满足式 (2) 下的速度对采样。
- **状态转移**：由运动学模型 (1) 与仿真/真实环境的动力学共同决定；环境中障碍物视为静态或给定动态模型。
- **回报与终止**：设单步回报为 \(r_t\)。若在限定时间 \(T_{\max}\) 内到达目标邻域（如与目标距离小于给定半径 \(d_{\text{goal}}\)），则任务成功，可定义终止回报 \(R_{\text{success}}\)；若发生碰撞则终止并给出 \(R_{\text{collision}}\)；若超时则终止并给出 \(R_{\text{timeout}}\)。其余步可设 \(r_t = 0\) 或小负值以鼓励缩短时间。形式地：

$$r_t = \begin{cases} R_{\text{success}} & \text{到达目标且无碰撞} \\ R_{\text{collision}} & \text{发生碰撞} \\ R_{\text{timeout}} & \text{超时未达目标} \\ c_{\text{time}} & \text{否则（如 } c_{\text{time}} \leq 0 \text{ 表示时间惩罚）} \end{cases} \tag{4}$$

在此 MDP 框架下，传统方法（如全局路径规划 + 局部避障）可视为一种确定性策略；基于优化的方法（如 DDR-opt）则在每步求解带约束的轨迹优化问题，等价于在有限时域内对动作序列进行优化。

---

## 2.2 基于 BARN 的 Benchmark 搭建

### 2.2.1 BARN 简介与任务设定

BARN（Benchmark for Autonomous Robot Navigation）源自 ICRA BARN Navigation Challenge，是一套用于评估移动机器人导航算法的标准测试环境。本文基于 BARN 搭建评测流程，并在其静态障碍场景下对传统导航方案进行测试与对比。

BARN 提供多组 Gazebo 仿真世界（world）：静态障碍场景对应 world 索引 0–299，每个世界由栅格化障碍与通道组成，起终点固定；DynaBARN 动态场景对应索引 300–359，包含动态障碍。本文的评测与指标定义主要针对静态场景（0–299）。在默认配置下，机器人初始位姿为 \([-2.25, 3, 1.57]\)（单位：米、弧度），目标点相对于起点为 \([0, 10]\)，即终点为 \([-2.25, 13]\)。机器人平台为 Jackal，配备前向激光雷达（如 UST10），仿真通过 ROS 与 Gazebo 完成。

**（2）世界与坐标**

BARN 的世界由栅格单元构成，每个单元对应真实尺度可通过路径文件与 Gazebo 坐标的映射得到。设栅格坐标为 \((x_{\text{grid}}, y_{\text{grid}})\)，Gazebo 坐标为 \((x_g, y_g)\)，映射关系为（与 `run.py` 中 `path_coord_to_gazebo_coord` 一致）：

$$x_g = x_{\text{grid}} \cdot (2R) + r_{\text{shift}},\quad y_g = y_{\text{grid}} \cdot (2R) + c_{\text{shift}} \tag{5}$$

其中 \(R = 0.075\) 为单元半径，\(r_{\text{shift}} = -R - 30 \times 2R\)，\(c_{\text{shift}} = R + 5\)。参考路径以栅格坐标形式存储在 `path_files/path_<world_idx>.npy` 中，经式 (5) 转换后用于计算路径长度与最优时间 \(T_{\text{opt}}\)。

### 2.2.2 Benchmark 实现流程

Benchmark 的完整流程由脚本 `run.py` 与 Gazebo 仿真、导航栈协同完成，主要步骤包括：

（1）**环境加载**：根据 world 索引选择世界文件（如 `BARN/world_0.world`），设置 Jackal 激光参数（如 `JACKAL_LASER_MODEL=ust10`），并通过 `roslaunch` 启动 Gazebo 与机器人模型（无头模式不启动 GUI，适用于服务器批量测试）。

（2）**初始状态校验**：将机器人重置到初始位姿，并确认无碰撞、位置误差在阈值内后再进入导航阶段。

（3）**导航栈启动与目标下发**：根据参数选择导航栈（如 `--nav_stack dwa` 或 `--nav_stack ddr_opt`）。DWA 方案通过 `move_base` 的 Action 接口发送目标位姿；DDR-opt 方案通过 `barn_ddr_bridge` 将目标发布到 `/move_base_simple/goal`。两种方式下目标均在 `odom` 坐标系下给出。

（4）**运行与监控**：以固定频率读取机器人位姿与碰撞状态，判断是否到达目标（与目标距离小于 1 m）、发生碰撞或超过 100 s。满足任一终止条件后结束本次运行。

（5）**日志输出**：将本次运行的 world 索引、成功标志、碰撞标志、超时标志、实际耗时以及导航指标写入输出文件（如 `out.txt`），便于后续汇总。

批量测试时，使用 `test.sh` 在 50 个均匀采样的 world（如索引 0, 6, 12, …, 294）上各运行 10 次，生成汇总日志；再通过 `report_test.py` 计算各 world 及总体的平均指标。

### 2.2.3 测试指标定义

单次运行的结局分为三类：**成功**（在 100 s 内到达目标且未碰撞）、**碰撞**、**超时**（未在 100 s 内到达且未碰撞）。在此基础上定义如下指标。

**（1）参考最优时间**

对静态 BARN 场景，每个 world 预存参考路径点序列，经式 (5) 转为 Gazebo 坐标后逐段求欧氏距离并求和得到路径长度 \(L_{\text{path}}\)。设定参考速度为 \(v_{\text{ref}} = 2\,\text{m/s}\)，则参考最优时间为：

$$T_{\text{opt}} = \frac{L_{\text{path}}}{v_{\text{ref}}} = \frac{L_{\text{path}}}{2} \tag{6}$$

**（2）导航指标（nav_metric）**

综合成功性与效率，定义为：

$$\text{nav\_metric} = \mathbb{1}_{\text{success}} \cdot \frac{T_{\text{opt}}}{\text{clip}(T_{\text{actual}}, 2T_{\text{opt}}, 8T_{\text{opt}})} \tag{7}$$

其中 \(\mathbb{1}_{\text{success}}\) 表示本次是否成功；\(T_{\text{actual}}\) 为实际耗时（秒）。\(\text{clip}(T_{\text{actual}}, 2T_{\text{opt}}, 8T_{\text{opt}})\) 将分母限制在 \([2T_{\text{opt}}, 8T_{\text{opt}}]\) 内，既避免除零，又对过慢或过快的异常值进行截断。因此仅在成功时 nav_metric 非零，且实际时间越接近 \(T_{\text{opt}}\) 则指标越接近 1。

**（3）汇总指标**

对 50 个 world、每 world 10 次运行的结果进行统计，得到以下五类平均值（与 `report_test.py` 输出一致）：

- **Avg Time**：所有成功次数的实际耗时的平均值；
- **Avg Metric**：所有单次运行的 nav_metric 先按 world 平均再对 world 求平均；
- **Avg Success**：成功率（成功次数 / 总次数）；
- **Avg Collision**：碰撞率；
- **Avg Timeout**：超时率。

上述指标从安全性（碰撞、超时）、任务完成度（成功率）与效率（时间、nav_metric）多角度衡量导航算法性能。

### 2.2.4 与当前问题的对应关系

在当前问题设定下，BARN 静态场景对应“已知障碍分布、差分驱动机器人、仅考虑静态障碍”的导航 MDP：状态由机器人位姿与激光（或由激光得到的代价地图/点云）提供；动作为 \((v, \omega)\) 或由上层规划给出的轨迹；终止条件与回报与 2.1.3 一致。通过统一使用 BARN 的 world 集合与上述指标，可公平比较不同导航栈（如 A*+DWA 与 DDR-opt）在相同环境与同一评测协议下的表现。

---

## 2.3 传统方案测试

### 2.3.1 基于搜索与采样的 A*+DWA 导航系统

#### 2.3.1.1 系统架构

本文采用的第一种传统方案为 ROS 标准导航栈 **move_base**，其全局规划器为基于栅格的 **NavfnROS**（Dijkstra），局部规划器为 **DWA（Dynamic Window Approach）**，二者配合实现“全局路径 + 局部避障与跟踪”。为表述简洁，下文中将该方案称为“A*+DWA”，其中“A*”泛指与 NavfnROS 同类的栅格图搜索全局规划方法。

move_base 的输入为当前代价地图（全局与局部）、机器人位姿与目标位姿；输出为发送给底层的 `cmd_vel`。全局规划器在栅格地图上搜索从起点到终点的最短路径，局部规划器在机器人周围采样若干 \((v, \omega)\)，在短时域内前向仿真得到轨迹，并根据与障碍物的距离、与全局路径的贴合度、速度大小等构造评价函数，选取最优控制量执行。

#### 2.3.1.2 全局规划器（NavfnROS）原理与算法

**（1）栅格地图构建**

为便于搜索，将可行空间离散化为二维栅格地图。设栅格分辨率为 \(\delta\)，每个栅格 \((i, j)\) 对应物理区域的一个小格。障碍物所在栅格及其膨胀邻域被赋予高代价值（如 254），自由空间为 0，形成代价地图 \(C(i,j)\)。这样，路径规划问题转化为在栅格图上寻找从起点栅格到终点栅格、且经过栅格代价值可接受的一条路径。

**（2）Dijkstra 扩展与代价传播**

NavfnROS 采用 Dijkstra 算法在栅格图上进行扩展。设起点栅格为 \(s\)，终点栅格为 \(g\)。每个栅格 \(n\) 维护从 \(s\) 到 \(n\) 的累积代价 \(g(n)\)。邻接关系通常取 4 邻域或 8 邻域，从 \(n\) 到邻居 \(n'\) 的边权可为欧氏距离或与代价相关的权重。Dijkstra 从 \(s\) 开始，每次从“待扩展集合”中取出当前 \(g\) 值最小的节点进行扩展，更新其邻居的 \(g\) 值并加入待扩展集合，直到扩展至 \(g\) 或待扩展集合为空（无解）。

为便于实现，引入**开集（Open Set）**与**闭集（Closed Set）**：

- **开集**：待扩展的栅格节点，按 \(g\) 值排序（如优先队列）；
- **闭集**：已扩展过的栅格节点，不再参与扩展。

**（3）算法步骤与伪代码**

NavfnROS 风格的栅格 Dijkstra 全局规划可描述为如下步骤：

（1）将起点 \(s\) 加入开集，令 \(g(s) = 0\)，闭集为空；  
（2）若开集为空，则规划失败，退出；  
（3）从开集中取出 \(g\) 值最小的节点 \(n\)；若 \(n\) 为终点 \(g\)，则沿父节点回溯得到路径，算法结束；  
（4）将 \(n\) 加入闭集；对 \(n\) 的每个邻居 \(n'\)（非障碍、未在闭集中），计算 \(g' = g(n) + w(n, n')\)，其中 \(w(n,n')\) 为边权；若 \(n'\) 不在开集中则加入开集并设 \(g(n') = g'\)、父节点为 \(n\)；若 \(n'\) 已在开集中且 \(g' < g(n')\)，则更新 \(g(n') = g'\) 并更新父节点；  
（5）转步骤（2）。

下表给出形式化伪代码。

**表 2-1 基于 Dijkstra 的栅格全局路径规划算法**

| 算法 2-1：栅格 Dijkstra 全局规划 |
|----------------------------------|
| **输入**：起点栅格 \(s\)，终点栅格 \(g\)，代价地图 \(C\) |
| **输出**：从 \(s\) 到 \(g\) 的栅格路径，或失败 |
| 1. 开集 \(\gets \{s\}\)，\(g(s) \gets 0\)，\(\text{parent}(s) \gets \emptyset\)，闭集 \(\gets \emptyset\) |
| 2. **while** 开集非空 **do** |
| 3.   \(n \gets \arg\min_{n' \in \text{开集}} g(n')\)；从开集中移除 \(n\) |
| 4.   **if** \(n = g\) **then** 沿 \(\text{parent}\) 回溯构造路径，**return** 路径 |
| 5.   闭集 \(\gets \text{闭集} \cup \{n\}\) |
| 6.   **for** 每个邻居 \(n'\) **do** |
| 7.    **if** \(n' \in \text{闭集}\) **or** \(C(n')\) 为障碍 **then** **continue** |
| 8.    \(g' \gets g(n) + w(n, n')\) |
| 9.    **if** \(n' \notin \text{开集}\) **then** 开集 \(\gets \text{开集} \cup \{n'\}\)，\(g(n') \gets g'\)，\(\text{parent}(n') \gets n\) |
| 10.   **else if** \(g' < g(n')\) **then** \(g(n') \gets g'\)，\(\text{parent}(n') \gets n\) |
| 11. **return** 失败 |

在 BARN 中，全局代价地图由激光与膨胀层更新，NavfnROS 在该地图上执行上述逻辑，得到的路径作为局部规划器的参考线。

#### 2.3.1.3 局部规划器（DWA）原理与算法

**（1）动态窗口**

DWA 在速度空间 \((v, \omega)\) 上进行采样与评价。考虑到机器人的加速度限制，当前时刻可行的 \((v, \omega)\) 并非整个矩形 \([v_{\min}, v_{\max}] \times [-\omega_{\max}, \omega_{\max}]\)，而是一个随当前速度变化的“动态窗口”。设当前速度为 \((v_c, \omega_c)\)，最大线加速度为 \(a_{x,\max}\)，最大角加速度为 \(a_{\theta,\max}\)，仿真时长为 \(\Delta t\)，则下一时刻可达的线速度范围为：

$$v \in [v_c - a_{x,\max}\Delta t,\, v_c + a_{x,\max}\Delta t] \cap [v_{\min}, v_{\max}] \tag{8}$$

角速度同理。将上述范围与障碍物约束（见下）进一步缩小，得到当前步可选的动态窗口 \(\mathcal{V}\)。

**（2）轨迹前向仿真**

对动态窗口内采样的每组 \((v, \omega)\)，使用运动学模型 (3) 在时间 \(t \in [0, T_{\text{sim}}]\) 内前向仿真（\(T_{\text{sim}}\) 为局部规划器参数，如 2 s），得到一条轨迹 \(\tau\)。仿真步长由 `sim_granularity`、`angular_sim_granularity` 等参数控制。采样数由 `vx_samples`、`vtheta_samples` 决定（如 6×20），形成若干候选轨迹。

**（3）轨迹评价函数**

对每条轨迹 \(\tau\) 计算代价值，代价值越低越优。常用分量包括：

1）**障碍物代价** \(J_{\text{obs}}\)：轨迹上各点到最近障碍物的距离的负值或倒数，障碍过近则代价剧增。在实现中通过代价地图查询轨迹经过栅格的代价值并累加，权重为 `occdist_scale`（如 0.1）。

2）**路径贴合度** \(J_{\text{path}}\)：轨迹与全局路径的偏差（如到路径的最近距离），权重为 `pdist_scale`（如 0.75）。

3）**目标接近度** \(J_{\text{goal}}\)：轨迹末端或全程对目标的接近程度（如到目标的距离），权重为 `gdist_scale`（如 1.0），同时影响朝向目标的速度选择。

总代价可表为加权和：

$$J(\tau) = w_{\text{obs}} J_{\text{obs}}(\tau) + w_{\text{path}} J_{\text{path}}(\tau) + w_{\text{goal}} J_{\text{goal}}(\tau) \tag{9}$$

若某条轨迹与障碍物发生碰撞，则将其代价设为无穷大，不参与比较。最终选取使 \(J(\tau)\) 最小的 \((v, \omega)\) 作为当前控制输出。

**（4）DWA 算法步骤与伪代码**

（1）获取当前位姿、速度与全局路径、局部代价地图；  
（2）根据式 (8) 与障碍物约束确定动态窗口 \(\mathcal{V}\)；  
（3）在 \(\mathcal{V}\) 内按参数离散采样得到多组 \((v, \omega)\)；  
（4）对每组 \((v, \omega)\) 用式 (3) 前向仿真得到 \(\tau\)，若 \(\tau\) 碰撞则设 \(J = +\infty\)，否则按式 (9) 计算 \(J(\tau)\)；  
（5）选择 \(J\) 最小的 \((v, \omega)\) 发布为 `cmd_vel`；  
（6）下一控制周期重复（1）–（5）。

**表 2-2 动态窗口法（DWA）局部规划**

| 算法 2-2：DWA 局部规划 |
|------------------------|
| **输入**：当前位姿与速度、全局路径、局部代价地图、参数（\(T_{\text{sim}}\)、采样数、权重） |
| **输出**：当前控制 \((v^*, \omega^*)\) |
| 1. 根据加速度与障碍物约束计算动态窗口 \(\mathcal{V}\) |
| 2. 在 \(\mathcal{V}\) 内采样得到集合 \(\{(v_i, \omega_i)\}\) |
| 3. **for** 每个 \((v_i, \omega_i)\) **do** |
| 4.   用运动学 (3) 前向仿真得到轨迹 \(\tau_i\) |
| 5.   **if** \(\tau_i\) 与障碍物碰撞 **then** \(J_i \gets +\infty\) **else** \(J_i \gets w_{\text{obs}} J_{\text{obs}}(\tau_i) + w_{\text{path}} J_{\text{path}}(\tau_i) + w_{\text{goal}} J_{\text{goal}}(\tau_i)\) |
| 6. \((v^*, \omega^*) \gets \arg\min_{(v_i, \omega_i)} J_i\) |
| 7. **return** \((v^*, \omega^*)\) |

#### 2.3.1.4 在本 Benchmark 下的实现要点

在 BARN pipeline 中，通过 `run.py` 默认参数或 `--nav_stack dwa` 启动 move_base，使用的 launch 文件为 `move_base_DWA.launch`，全局与局部代价地图、DWA 参数均从 `jackal_helper` 的 configs 目录加载（如 `base_local_planner_params.yaml` 中 `dwa: true`、`sim_time: 2.0`、`vx_samples: 6`、`vtheta_samples: 20` 等），里程计话题映射为 `odometry/filtered`。无需额外桥接包，目标通过 move_base 的 Action 接口发送。

#### 2.3.1.5 测试与指标分析

（**说明**：以下需在完成批量测试后填入实际数据。若尚未运行，可先保留表格结构，待执行 `test.sh` 与 `report_test.py` 后替换为真实结果。）

实验设置：在 50 个均匀采样的 BARN 世界上（world 索引 0, 6, 12, …, 294），每个 world 运行 10 次，共 500 次。单次运行超时时间为 100 s，成功判据为与目标距离小于 1 m 且无碰撞。

**如何生成 A*+DWA 的测试结果**：

1. 在已配置好的 ROS + Gazebo + BARN 工作空间中，进入 BARN pipeline 目录（包含 `run.py` 与 `test.sh`）。  
2. 确保 `test.sh` 中调用的是默认导航栈（不传 `--nav_stack ddr_opt`），输出重定向到单独文件，例如：`... python3 run.py --world_idx ... --out res/dwa_out.txt ...`  
3. 执行完整批量测试：`bash test.sh`（具体以你仓库中 `test.sh` 的写法为准）。  
4. 汇总结果：`python3 report_test.py --out_path res/dwa_out.txt`，终端将打印 Avg Time、Avg Metric、Avg Success、Avg Collision、Avg Timeout。  
5. 将上述五个数值填入下表“A*+DWA”一行，并据此撰写 2.3.1.5 的指标分析段落（成功率、碰撞率、超时率、效率与 nav_metric 的关系）。

| 方案        | Avg Time (s) | Avg Metric | Avg Success | Avg Collision | Avg Timeout |
|-------------|--------------|------------|-------------|---------------|-------------|
| A*+DWA      | （待填入）   | （待填入） | （待填入）  | （待填入）    | （待填入）  |
| DDR-opt     | （待填入）   | （待填入） | （待填入）  | （待填入）    | （待填入）  |

---

### 2.3.2 基于优化的 DDR-opt 导航系统

#### 2.3.2.1 系统架构

DDR-opt 是一种面向差分驱动机器人（Differential Drive Robot）的**轨迹优化框架**，通过数值优化在连续空间内直接求解满足动力学约束与避障的轨迹，而非“全局图搜索 + 局部采样”的两层结构。其核心包括：利用激光或点云构建局部环境表示（如 SDF）、后端轨迹优化、以及 NMPC 控制器的跟踪。在 BARN 上通过桥接包 `barn_ddr_bridge` 将 Gazebo 与 Jackal 的接口与 DDR-opt 的输入输出对接，实现“BARN 世界 + DDR-opt 导航栈”的完整评测链路。

#### 2.3.2.2 轨迹优化问题建模

**（1）优化变量**

设规划时域内轨迹由一系列位姿与时间或控制序列参数化。一种常见方式是将轨迹离散为 \(N\) 个路点，每个路点包含位置与时间：\(q_i = (x_i, y_i, \theta_i, t_i)\) 或仅位置时间 \((x_i, y_i, t_i)\)，优化变量为 \(X = (q_1, q_2, \ldots, q_N)^\top\)。另一种方式为对控制序列 \((v_k, \omega_k)\) 进行优化。DDR-opt 针对差速模型，将运动学与动力学约束直接纳入优化，变量形式以具体实现为准。

**（2）目标函数**

目标函数通常包含以下几类项的加权和：

1）**时间最短**：\(\sum_i (t_{i+1} - t_i)\) 或总时间 \(T\)，使轨迹尽量快捷。  
2）**控制平滑/能量**：如 \(\sum_k (v_k^2 + \omega_k^2)\) 或加速度的范数，使控制量平滑、易于跟踪。  
3）**轨迹光滑**：如对位姿序列的二阶差分惩罚，使路径曲率连续。

可写为：

$$\min_X \;\; f_{\text{obj}}(X) = w_T \cdot T + w_{\text{smooth}} \cdot J_{\text{smooth}}(X) + \ldots \tag{10}$$

**（3）约束条件**

1）**运动学约束**：差速模型 (1) 的离散形式，即相邻路点之间满足 \(x_{i+1} = x_i + \int_{t_i}^{t_{i+1}} v\cos\theta\,dt\) 等，或对控制序列与状态递推关系进行约束。  
2）**速度与角速度约束**：\(|v| \leq v_{\max}\)，\(|\omega| \leq \omega_{\max}\)，对应式 (2)。  
3）**障碍物约束**：利用 SDF（符号距离场）或点云距离，要求轨迹上点到障碍物的距离 \(\geq d_{\text{safe}}\)。通常对路点或轨迹上的采样点施加 \(d(q_i, \mathcal{O}) \geq d_{\text{safe}}\)。  
4）**边界与初末态**：起点与当前状态一致，终点与目标一致或落在目标邻域内。

上述问题一般为非线性规划（NLP）。通过离散化与适当的线性化，可转化为二次规划（QP）或带线性/二次约束的 QP，由 OSQP 等求解器求解。DDR-opt 使用 OSQP 与 OSQP-Eigen 进行数值求解。

**（4）NMPC 闭环**

在实际运行中，DDR-opt 以 NMPC 方式工作：每个控制周期根据当前状态与目标，在有限时域内求解上述优化问题，取最优控制序列的第一项作为当前 \((v, \omega)\) 输出，下一周期重新求解，形成闭环。

#### 2.3.2.3 算法流程与伪代码

**表 2-3 DDR-opt 轨迹优化与 NMPC 闭环**

| 算法 2-3：DDR-opt 导航栈（高层流程） |
|--------------------------------------|
| **输入**：当前状态（位姿、速度）、目标位姿、局部点云/SDF |
| **输出**：控制量 \((v, \omega)\) |
| 1. 根据激光/点云更新局部 SDF 或距离场 |
| 2. 以当前状态为初值、目标为终值，构建有限时域轨迹优化问题（目标 (10)、运动学与障碍物等约束） |
| 3. 将问题离散化/线性化为 QP 或 NLP |
| 4. 调用 OSQP（或其它求解器）求解得到最优轨迹/控制序列 |
| 5. 取最优控制序列的第一个控制量 \((v^*, \omega^*)\) 作为输出 |
| 6. **return** \((v^*, \omega^*)\)；下一周期从步骤 1 重复 |

#### 2.3.2.4 在 BARN 上的集成方式

为在 BARN 的 Gazebo 仿真中运行 DDR-opt，需解决接口不一致问题：BARN 提供激光话题（如 `/front/scan`）与 Jackal 的 `/cmd_vel`；DDR-opt 需要点云形式的局部环境与固定坐标系下的目标，并输出自己的控制话题。`barn_ddr_bridge` 完成以下转换：

- **LaserScan → PointCloud2**：将 `/front/scan` 转换为 DDR-opt 所需的 `/laser_simulator/local_pointcloud`，供 SDF 或距离场构建使用。  
- **DDR 控制 → Twist**：订阅 DDR-opt 的 MPC 输出（如 `/ddr_cmd`），转换为 `geometry_msgs/Twist` 并发布到 `/cmd_vel`，驱动 Jackal 模型。  
- **TF**：发布静态变换 `odom` → `world`，使 DDR-opt 的“world”坐标系与 BARN 的 odom 一致。

在同一 catkin 工作空间中同时编译 BARN 相关包（含 `jackal_helper`、`barn_ddr_bridge`）与 DDR-opt 相关包（如 `plan_manager`、`mpc_controller` 等）后，通过 `run.py --nav_stack ddr_opt` 启动 `barn_ddr_bridge` 的 `ddr_opt_barn.launch`，即可在相同 world 与起终点下运行 DDR-opt 并记录与 2.2.3 相同的指标。

#### 2.3.2.5 测试与指标分析

（**说明**：同样需在完成批量测试后填入实际数据。）

**如何生成 DDR-opt 的测试结果**：

1. 确保工作空间中已正确编译 DDR-opt 及其依赖（如 OSQP、OSQP-Eigen），以及 `barn_ddr_bridge`。  
2. 在 pipeline 目录下执行批量测试时，为每次 `run.py` 指定 `--nav_stack ddr_opt`，并将输出写入单独文件，例如：`... python3 run.py --world_idx ... --nav_stack ddr_opt --out res/ddr_opt_out.txt ...`  
3. 运行完整的 50×10 次测试（与 A*+DWA 使用相同的 world 集合与次数）。  
4. 汇总：`python3 report_test.py --out_path res/ddr_opt_out.txt`，将终端输出的五类平均值填入上表“DDR-opt”一行。  
5. 根据表中两行数据撰写 2.3.3 的对比分析：从成功率、碰撞率、超时率、平均耗时与 Avg Metric 比较两种方案，并简要讨论差异原因（如优化类方法在光滑性与约束满足上的特点，与基于采样的 DWA 在狭窄通道或极端场景下的表现差异）。

---

## 2.3.3 实验设置与结果对比（待补全）

- **实验环境**：Ubuntu 20.04 / 18.04，ROS Noetic / Melodic，Gazebo，无头模式（无 GUI）。BARN 静态 world 50 个，每 world 10 次，超时 100 s。  
- **结果表格**：见 2.3.1.5 中表格，完成测试后填入两行数据。  
- **对比分析**：在填入数据后，可从以下方面撰写 1–2 段文字：  
  （1）成功率与安全性（碰撞率、超时率）；  
  （2）效率（Avg Time、Avg Metric）；  
  （3）在不同 world 上的稳定性（若有 per-world 统计可简要说明）；  
  （4）两种方法在算法范式上的差异（搜索+采样 vs 轨迹优化）如何体现在指标上。

---

## 附录：如何运行测试并补全报告

1. **单次验证**（任选一 world）：  
   - A*+DWA：`python3 run.py --world_idx 0 --out out.txt`  
   - DDR-opt：`python3 run.py --world_idx 0 --nav_stack ddr_opt --out out.txt`  
   查看终端输出的成功/碰撞/超时与 nav_metric。

2. **批量测试**：  
   - 修改或确认 `test.sh` 中对 `run.py` 的调用及 `--out` 路径（DWA 与 DDR-opt 使用不同输出文件）。  
   - 执行 `bash test.sh`，等待 50×10×2 次运行完成（若两种方案都测）。  
   - 对每个输出文件运行：  
     `python3 report_test.py --out_path res/dwa_out.txt`  
     `python3 report_test.py --out_path res/ddr_opt_out.txt`  
   - 将打印的五个平均值填入 2.3.1.5 的表格，并补写 2.3.2.5 与 2.3.3 的分析段落。

3. **无图形界面环境**（如云服务器）：  
   - 使用无头模式（不加 `--gui`）；若仍有显示相关报错，可使用：  
     `xvfb-run -a python3 run.py --world_idx 0 --nav_stack ddr_opt --out out.txt`  
   详细环境要求与步骤见 `DEPLOY_CLOUD_SERVER.md`。
