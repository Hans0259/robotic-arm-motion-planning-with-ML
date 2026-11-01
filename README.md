 <h1>ECE 780 Project</h1>

`dobot_planning_scene` 是在Dobot官方ROS2包上新添加的包， 当前实现通过 `/apply_planning_scene` 服务一次性添加 **10 个随机球形障碍物**。

## 目录结构

```text
dobot_planning_scene/
├── config/
│   └── obstacles.yaml          # 预留的箱体、网格障碍物配置
├── dobot_planning_scene/
│   ├── random_scene.py        # 生成随机场景并调用 MoveIt 服务
│   └── spheres_generator.py   # 构造球形 CollisionObject
├── launch/
│   └── planning_scene.launch.py
├── meshes/
│   └── support.STL            # 预留的支架模型
├── package.xml
├── setup.cfg
└── setup.py
```

## 环境与编译

以下命令以 ROS 2 Humble、工作空间 `~/dobot_ws` 和 CR10 模型为例。需要提前安装 ROS 2、MoveIt 2、RViz 2、Git 和 colcon。

### 获取源码

在新环境中创建工作空间，并克隆本项目的 `2026_10_27` 分支（当前开发分支，包含 `dobot_planning_scene`）：

```bash
mkdir -p ~/dobot_ws/src
cd ~/dobot_ws/src
git clone --branch 2026_10_27 https://github.com/harryshi0920/dobot_ws.git DOBOT_6Axis_ROS2_V4
```

仓库本身包含各个 ROS 2 功能包，因此放在工作空间的 `src/DOBOT_6Axis_ROS2_V4` 下。本文档位于仓库根目录，规划场景包位于 `dobot_planning_scene/` 子目录。

### 编译工作空间
在工作空间根目录编译：

```bash
cd ~/dobot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

如果已经完成整个工作空间的编译，后续只修改本包时可以使用：

```bash
colcon build --symlink-install --packages-select dobot_planning_scene
source install/setup.bash
```

新增模型或配置文件后，需要重新构建本包以更新安装目录中的资源。

## 启动方法

### 一键启动 MoveIt 演示和障碍物

```bash
cd ~/dobot_ws
source install/setup.bash
ros2 launch dobot_planning_scene planning_scene.launch.py
```

此 launch 文件会依次包含以下启动动作：

1. 通过 `dobot_moveit/moveit_demo.launch.py` 启动 `${DOBOT_TYPE}_moveit/demo.launch.py`。
2. 启动随机场景节点，等待 `/apply_planning_scene` 服务可用后提交障碍物。

两者的初始化没有固定延时，场景节点通过等待服务来配合 MoveIt 启动。此入口使用 MoveIt 演示配置，不会启动本仓库的真机通信驱动。

成功后会看到日志：

```text
Random spheres generate successfully
```

场景节点在服务返回后退出，这是正常行为；MoveIt 和 RViz 仍继续运行。在 RViz 的 MotionPlanning 显示中启用场景几何体显示，即可观察球形障碍物并尝试规划。

### 向已经运行的 MoveIt 添加障碍物

如果 MoveIt 已经启动且提供 `/apply_planning_scene` 服务，只需在已加载 ROS 2 和工作空间环境的终端执行：

```bash
ros2 run dobot_planning_scene random_scene
```

独立运行时默认节点名为 `random_scene`；使用本包 launch 启动时，节点名为 `planning_scene_loader`。

## 随机场景的生成规则
`spheres_generator.py`为生成球体function，`random_scene.py`包含随即算法

所有球体都使用 `base_link` 坐标系，长度单位为米。

| 配置项 | 当前值 | 代码位置 |
| --- | --- | --- |
| 球体数量 | `10` | `random_scene.py` 中的 `main()` |
| 随机种子 | `None`，每次随机生成 | `random_scene.py` 中的 `main()` |
| 半径 | `0.05`～`0.2` | `generate_random_spheres()` |
| 球心 X | `-1.3`～`1.3` | `generate_random_spheres()` |
| 球心 Y | `-0.5`～`0.5` | `generate_random_spheres()` |
| 球心 Z | `0.1`～`0.5` | `generate_random_spheres()` |
| 障碍物 ID | `sphere_0`～`sphere_9` | `generate_random_spheres()` |
| 坐标系 | `base_link` | `spheres_generator.py` |

这些设置目前是 Python 代码中的常量，**尚未提供对应的 ROS 参数或 launch 参数**。如需固定场景，可将 `main()` 中的 `random_seed` 改为固定整数；如需改变数量或分布范围，修改对应代码后重新构建并加载工作空间环境。

随机生成过程不会检查球体之间或球体与机器人之间是否相交，也不会保证目标点一定可达。场景导致规划失败时，可调整分布范围或数量，或重新运行节点生成场景。


> **README 增补章节**

## 当前实现状态（2026-11-01）

项目基于 **ROS 2 Humble + MoveIt 2 + OMPL/RRTConnect**，使用随机球形障碍物创建规划场景，再采集关节空间运动轨迹，保存为 NumPy `.npz` 数据集，以供后续机器人路径生成模型训练。

- `spheres_generator.py`：将球心位置、半径和 ID 编码为 MoveIt 2 `CollisionObject`。
- `random_scene.py`：`generate_random_spheres()` 生成随机球体；`apply_scene()` 添加场景物体；`remove_spheres()` 删除已知 ID 的球体。
- `planning_scene.launch.py`：原有演示的一键启动入口；**仅用于原有演示**，会自动添加随机球体。
- `ml_motion_planning/dataset.py`：校验样本结构，将多个样本压缩保存为 `.npz`。
- `ml_motion_planning/test_dataset.py`：用虚拟的线性插值轨迹测试文件结构，不作为避障训练标签。
- `ml_motion_planning/collect_dataset.py`：复用 `random_scene.py`，调用 MoveIt 2 规划服务，重采样轨迹、过滤无效样本，并生成 `train_dataset.npz`。

### 目录结构

```text
~/dobot_ws/
├── src/
│   └── DOBOT_6Axis_ROS2_V4/
│       ├── cr10_moveit/
│       │   ├── config/
│       │   │   ├── cr10_robot.srdf
│       │   │   └── ompl_planning.yaml
│       │   └── launch/
│       ├── dobot_moveit/
│       │   └── launch/moveit_demo.launch.py
│       ├── dobot_planning_scene/
│       │   ├── dobot_planning_scene/
│       │   │   ├── random_scene.py
│       │   │   └── spheres_generator.py
│       │   └── launch/planning_scene.launch.py
│       └── ml_motion_planning/
│           ├── dataset.py
│           ├── test_dataset.py
│           ├── collect_dataset.py
│           └── data/
│               ├── test_dataset.npz
│               └── train_dataset.npz   # 运行 collect_dataset.py 后生成
├── build/
├── install/
└── log/
```


### 3. 编译并加载工作空间

**Terminal 1 — 编译（首次）**

```bash
cd ~/dobot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source ~/dobot_ws/install/setup.bash
```

仅修改 `dobot_planning_scene` 包后，可执行：

```bash
cd ~/dobot_ws
colcon build --symlink-install --packages-select dobot_planning_scene
source ~/dobot_ws/install/setup.bash
```

修改独立的 `ml_motion_planning/*.py` 通常不需要 `colcon build`，直接使用 `python3` 运行；但 `dobot_planning_scene` 必须可以被 Python 导入。

```bash
python3 -c 'from dobot_planning_scene.random_scene import generate_random_spheres, apply_scene, remove_spheres; print("Import successful")'
```

## 启动方式

### A. 原有演示：MoveIt 2 + 一次性随机场景

此方式保留原 README 中的功能：

```bash
cd ~/dobot_ws
source /opt/ros/humble/setup.bash
source ~/dobot_ws/install/setup.bash
ros2 launch dobot_planning_scene planning_scene.launch.py
```

该 Launch 会调用项目的 MoveIt 演示 Launch，并提交一组球形障碍物。用于原有 RViz 场景演示。**不要同时用它作为数据采集的启动入口**，以免它额外添加的障碍物与数据集记录不一致。

### B. Stage 3 数据收集：只启动 MoveIt 2

数据收集脚本会自主生成、删除和添加随机球体，因此应在**干净的规划场景**中启动 MoveIt 2，而**不单独运行** `planning_scene.launch.py` 或 `random_scene` 节点。

**Terminal 1 — 启动 MoveIt 2（保持运行）**

```bash
cd ~/dobot_ws
source /opt/ros/humble/setup.bash
source ~/dobot_ws/install/setup.bash
ros2 launch dobot_moveit moveit_demo.launch.py
```

此命令来自仓库的 `dobot_moveit/launch/moveit_demo.launch.py`；它依赖本机实际的 `DOBOT_TYPE` 等 Launch 配置。如果你已经有一条能正确启动 `/move_group` 的命令，也可以沿用已验证的启动方式。无需启动真实机械臂执行控制器来采集规划轨迹。

**Terminal 2 — 确认服务可用**

```bash
source /opt/ros/humble/setup.bash
source ~/dobot_ws/install/setup.bash
ros2 node list | grep move_group
ros2 service list | grep -E 'plan_kinematic_path|apply_planning_scene|check_state_validity|get_planning_scene'
```

预期至少有：

```text
/apply_planning_scene
/check_state_validity
/get_planning_scene
/plan_kinematic_path
```

这些服务负责场景更新、状态有效性检查、读取状态和规划。`collect_dataset.py` 使用 `/plan_kinematic_path` **只规划、不发送执行命令**。

## Stage 3：收集 RRTConnect 训练轨迹

### 1. 确认 MoveIt 2 配置

已验证的规划参数：

| 项目 | 值 |
| --- | --- |
| Planning group | `cr10_group` |
| Planner ID | `RRTConnectkConfigDefault` |
| Joint order | `joint1 joint2 joint3 joint4 joint5 joint6` |
| Dataset DOF | `6` |
| Resampled waypoints | `32` |
| Maximum encoded obstacles | `20` |
| Obstacle reference frame | `base_link` |

从运行中的 `/move_group` 所加载 URDF 读取的位置上下限（rad）：

| Joint | Lower | Upper |
| --- | ---: | ---: |
| joint1 | -3.14 | 3.14 |
| joint2 | -3.14 | 3.14 |
| joint3 | -2.861 | 2.861 |
| joint4 | -3.14 | 3.14 |
| joint5 | -3.14 | 3.14 |
| joint6 | -3.14 | 3.14 |

当前采集脚本根据这些限制随机采样 Start / Goal，再使用 `GetStateValidity` 过滤无效状态。切换机器人或 URDF 后必须重新核对这些值。

### 2. 第一次检查：单条样本

**Terminal 2 — 在 MoveIt 2 保持运行时：**

```bash
cd ~/dobot_ws/src/DOBOT_6Axis_ROS2_V4/ml_motion_planning
source /opt/ros/humble/setup.bash
source ~/dobot_ws/install/setup.bash

python3 collect_dataset.py \
  --group cr10_group \
  --joints joint1 joint2 joint3 joint4 joint5 joint6 \
  --planner-id RRTConnectkConfigDefault \
  --samples 1 \
  --max-attempts 30 \
  --obstacles 5 \
  --output data/train_dataset.npz
```

程序为每次尝试生成一组球体和随机有效 Start/Goal；仅当关节空间直线被离散碰撞检测判为无效、RRTConnect 找到有效规划路径、且重采样后检查通过，才接受一条样本。`--samples` 是目标样本数量，`--max-attempts` 是最多尝试次数，**不保证达到目标数**。

### 3. 小规模批量测试（推荐：5 条）

```bash
python3 collect_dataset.py \
  --group cr10_group \
  --joints joint1 joint2 joint3 joint4 joint5 joint6 \
  --samples 5 \
  --max-attempts 50 \
  --obstacles 5 \
  --output data/train_dataset.npz
```

已实测：**13 次尝试成功接受 5 条样本**（随机结果因种子、场景状态、配置而异）。日志示例：

```text
Straight path is valid; skipping
Accepted 1/5 (attempt 2)
...
Accepted 5/5 (attempt 13)
Saved 5 samples to data/train_dataset.npz
```

### 4. 更大一些的批量测试（目标 100 条）

```bash
python3 collect_dataset.py \
  --group cr10_group \
  --joints joint1 joint2 joint3 joint4 joint5 joint6 \
  --samples 100 \
  --max-attempts 500 \
  --obstacles 5 \
  --output data/train_dataset.npz
```

**2026-11-01 实测结果：**

```text
Accepted 81/100 (attempt 500)
Saved 81 samples to data/train_dataset.npz
Accepted 81 samples out of 500 attempts
```

对应 **81 / 500 = 16.2%** 的总样本接受率；这不是 RRTConnect 本身的规划成功率。许多候选任务因直线路径有效被跳过，也有可能因状态无效、规划失败或轨迹检查失败被过滤。若达到 `--max-attempts` 上限，脚本仍会保存已经接受的样本。

> 每次运行并保存时会**覆盖**同名 `train_dataset.npz`，不会追加到旧文件。若一次收集 0 条，当前版本不保存新文件，之前的数据文件保持不变。

### 5. 查看数据格式

在 `ml_motion_planning/` 目录运行：

```bash
python3 - <<'PY'
import numpy as np

data = np.load('data/train_dataset.npz')
for key in data.files:
    print(f'{key}: {data[key].shape}')
print('Actual obstacles in first sample:', int(data['obstacle_mask'][0].sum()))
PY
```

如果有 `N` 条样本，数据结构为：

| Key | Shape | Meaning |
| --- | --- | --- |
| `start` | `(N, 6)` | Start joint configuration (rad) |
| `goal` | `(N, 6)` | Goal joint configuration (rad) |
| `obstacles` | `(N, 20, 4)` | Padded sphere `[x, y, z, radius]` (m) |
| `obstacle_mask` | `(N, 20)` | 1 = real sphere; 0 = padding |
| `trajectory` | `(N, 32, 6)` | RRTConnect joint-space path (rad) |

`test_dataset.npz` 由 `test_dataset.py` 使用 `np.linspace()` 创建，仅验证保存接口，不是模型的正式测试集。正式训练还需划分独立 Train/Validation/Test，最好按**场景**分组，避免数据泄漏。

### 6. 检查每条路径相对于直线的偏差

```bash
python3 - <<'PY'
import numpy as np

data = np.load('data/train_dataset.npz')
for i, (start, goal, trajectory) in enumerate(zip(data['start'], data['goal'], data['trajectory']), 1):
    straight = np.linspace(start, goal, len(trajectory))
    deviation = np.linalg.norm(trajectory - straight, axis=1).max()
    rrt_length = np.linalg.norm(np.diff(trajectory, axis=0), axis=1).sum()
    direct_length = np.linalg.norm(goal - start)
    ratio = rrt_length / direct_length if direct_length > 1e-10 else float('nan')
    print(f'Sample {i}: Max Deviation = {deviation:.4f}, Path Ratio = {ratio:.4f}')
PY
```

在最早的 5 条非直线样本中，观测到 Path Ratio **1.0497–1.1629**。Path Ratio > 1 说明关节空间路径比起终点直线更长；**仅凭这一指标无法证明球体是绕障原因**。`GetStateValidity` 同时可能捕捉自碰撞、其他环境物体及约束问题。



## LOG
### 2026-10-27
1. 更改ompl_planning.yaml为github上 moveit/moveit_resource/blob/humble/panda_moveit_config/config/ompl_planning.yaml中的结构。原ompl_planning.yaml会使MoveitConfigsBuilder自动在外面包一层ompl
2. 更改URDF的joint limit，原代码中的joint limit设置为+-360。更改为+-180

### 2026-10-28
1. 增加dobot_planning_scene/dobot_planning_scene/spheres_generator.py和random_scene.py

### 2026-11-1
- 新增 `ml_motion_planning/dataset.py`、`test_dataset.py`、`collect_dataset.py`。
- 复用已有 `generate_random_spheres()`、`apply_scene()`、`remove_spheres()`，通过 MoveIt 2 `GetMotionPlan` 服务调用 RRTConnect。
- 将规划轨迹重采样为 32 个 6-DOF 关节空间路点并保存到 `train_dataset.npz`。
- 从固定零位起点转为依据当前 `/move_group` URDF 关节限制采样有效随机 Start/Goal。
- 100 条目标 / 500 次尝试的批量测试实际保存 **81 条**样本。

