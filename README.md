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

### 目录结构

```text
~/dobot_ws/
├── src/
│   └── DOBOT_6Axis_ROS2_V4/
│       │
│       ├── cr10_moveit/
│       │   ├── config/
│       │   │   ├── cr10_robot.srdf
│       │   │   └── ompl_planning.yaml
│       │   └── launch/
│       │
│       ├── dobot_moveit/
│       │   └── launch/
│       │       └── moveit_demo.launch.py
│       │
│       ├── dobot_planning_scene/
│       │   ├── dobot_planning_scene/
│       │   │   ├── __init__.py
│       │   │   ├── random_scene.py          # 随机生成障碍物场景
│       │   │   └── spheres_generator.py     # 创建球形障碍物
│       │   ├── launch/
│       │   │   └── planning_scene.launch.py # 启动 Planning Scene
│       │   ├── meshes/
│       │   ├── resource/
│       │   ├── test/
│       │   ├── package.xml
│       │   ├── setup.cfg
│       │   └── setup.py
│       │
│       ├── dobot_rviz/
│       │
│       ├── image/
│       │
│       ├── misc/
│       │
│       ├── ml_motion_planning/
│       │   ├── data/
│       │   │   └── train_dataset.npz        # OMPL Expert Trajectories 数据集
│       │   │
│       │   ├── dataset.py                   # 定义数据结构、轨迹处理及 NPZ 保存
│       │   ├── collect_dataset.py           # 批量生成场景并调用 RRTConnect 收集轨迹
│       │   ├── torch_dataset.py             # NPZ → PyTorch Dataset / DataLoader
│       │   ├── flow_model.py                # 定义预测 Velocity Field 的 MLP 网络
│       │   ├── train_flow.py                # Conditional Flow Matching 训练程序
│       │   └── flow_model.pth               # 训练完成的神经网络参数
│       │
│       ├── servo_action/
│       ├── .gitignore
│       ├── LICENSE
│       ├── README_EN.md
│       └── README_ZH.md
│
├── build/                                   # ROS2 编译文件
├── install/                                 # ROS2 安装及环境文件
└── log/                                     # ROS2 运行和编译日志
```

**Terminal 1 — 启动 MoveIt 2（保持运行）**

```bash
cd ~/dobot_ws
source install/setup.bash
ros2 launch dobot_moveit moveit_demo.launch.py
```

###  批量测试

**Terminal 2 — 在 MoveIt 2 保持运行时：**


ml_motion_planning/
├── data/
│   └── train_dataset.npz     # Stage 3 收集的 RRTConnect 轨迹数据
├── collect_dataset.py       # 生成随机场景并调用 RRTConnect 收集轨迹
├── dataset.py               # 定义每条样本的格式并保存 NPZ
├── torch_dataset.py         # NPZ → PyTorch Dataset / DataLoader
├── flow_model.py            # 条件速度场 MLP 网络
├── train_flow.py            # Flow Matching 训练循环
└── flow_model.pth           # 训练后生成的网络参数文件

```bash
cd ~/dobot_ws/src/DOBOT_6Axis_ROS2_V4/ml_motion_planning

source /opt/ros/humble/setup.bash
source ~/dobot_ws/install/setup.bash
#	批量生成随机场景并调用 RRTConnect 收集轨迹
python3 collect_dataset.py 
#	将 NPZ 数据转换为 PyTorch Dataset，用于后续training
python3 torch_dataset.py
#	定义预测 Velocity Field 的 MLP 神经网络
python3 flow_model.py
#	使用 Conditional Flow Matching 训练神经网络
python3 train_flow.py
#   通过训练好的神经网络，用 Euler Integration Inference 结果
python3 inference.py
#   观察joints在每一个step的位置，判断学习结果
python3 visualize_trajectory.py


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
- 100 条目标 / 500 次尝试的批量测试实际保存 **81 条**样本

### 2026-11-02

**Stage 4 — Conditional Flow Matching**

**已完成（Completed）：**
- 新增 `ml_motion_planning/torch_dataset.py`，使用 PyTorch `Dataset` 和 `DataLoader` 读取 `train_dataset.npz`，支持 Single Sample 和 Mini-Batch。
- 新增 `ml_motion_planning/flow_model.py`，建立 Conditional Flow Matching MLP，网络结构为 `285 → 512 → 512 → 192`。
- 新增 `ml_motion_planning/train_flow.py`，实现 Flow Matching Training，包括 Gaussian Noise Sampling、Time Sampling、Velocity Prediction、MSE Loss 和 Adam Optimization。
- 使用 10 条 RRTConnect 轨迹完成 100 Epochs 的初步训练，Training Loss 从 Epoch 10 的 **1.7852** 降至 Epoch 100 的 **1.0650**。
- 成功生成 `ml_motion_planning/flow_model.pth`，保存训练后的神经网络权重。
- 验证 PyTorch 数据维度：Trajectory `(B, 32, 6)`，模型输出 Velocity `(B, 192)`。

**待完成（Future Work）：**
- **Stage 4.4 — Euler Integration Inference：** 新增 `inference.py`，加载 `flow_model.pth`，通过 Euler Integration 从 Gaussian Noise 生成完整的 6-DOF Robot Trajectory。
- **Stage 4.5 — Trajectory Visualization：** 可视化生成轨迹，并与 RRTConnect Expert Trajectory 进行比较。
- **Stage 4.6 — MoveIt2 Collision Validation：** 检查 ML 生成轨迹的 Joint Limits、Start/Goal Accuracy 和 Collision-Free Validity。
- **Stage 4.7 — Performance Comparison：** 比较 Flow Matching 与 RRTConnect 的 Planning Time、Path Length、Collision-Free Rate 和 Goal Reaching Error。
- **Dataset Improvement：** 扩大训练数据规模，划分 Training / Validation / Test Dataset，并增加 Normalization 和 Start/Goal Constraints

