# Go2 MuJoCo RL 底层强化学习控制交付工作流说明

本文档用于交接当前已经验证可用的 Go2 MuJoCo 底层强化学习控制流程。

目标读者：

```text
1. 后续要继续做 SLAM、导航、路径跟踪、视觉导航的人。
2. 想复现当前 MuJoCo 中 Go2 可稳定前后左右平移和旋转控制的人。
3. 想理解 policy_rough.pt、go2.yaml、go2_mujoco_rl_policy_runner.py 分别负责什么的人。
```

当前结论：

```text
当前工程已经完成：
1. 加载 TheoBounac/Deploy_SimToReal_RL_Go2 的 Go2 预训练低层 RL policy。
2. 在 MuJoCo 中通过 PD 缓起让 Go2 从初始姿态平滑站立。
3. 按 TheoBounac 实机部署脚本的 52 维 observation 格式拼接网络输入。
4. 使用 policy 输出的 12 维 action 生成 12 个目标关节角。
5. 使用 MuJoCo 高频 PD 内环驱动 12 个关节。
6. 前进、后退、左右平移、原地旋转都已经在 MuJoCo 中测试为可正常操控且稳定。
```

注意

```text
这套流程当前是 MuJoCo Sim2Sim 验证流程，不是直接上实机流程。
不要把 go2_mujoco_rl_policy_runner.py 直接连接到真实 Go2 的 LowCmd。
真实 Go2 上机必须另走宇树 SDK2、安全模式、遥控器接管、急停和架空测试流程。
```

---

## 1. 当前交付文件架构

推荐保持项目目录结构不变：

```text
/home/ros/unitree_dev
├── projects/go2_control_demos/
│   ├── go2_mujoco_rl_policy_runner.py
│   └── README.md
│
├── projects/open_source_deploy_simtoreal_rl_go2/
│   ├── README.md
│   ├── doc/
│   │   ├── Isaaclab.md
│   │   ├── Deploy.md
│   │   └── Deploy_with_Kalman_filter.md
│   ├── deploy_real/
│   │   ├── deploy_real_isaaclab.py
│   │   ├── node_kalman.py
│   │   ├── configs/
│   │   │   ├── config.py
│   │   │   └── go2.yaml
│   │   └── common/
│   │       ├── command_helper.py
│   │       ├── remote_controller.py
│   │       └── rotation_helper.py
│   └── pre_train/
│       ├── policy_rough.pt
│       └── policy_rough_2.pt
│
├── src/unitree_mujoco/
│   └── unitree_robots/go2/
│       ├── scene.xml
│       ├── go2.xml
│       └── assets/
│
└── docs/
    ├── Go2_MuJoCo_RL底层控制交付工作流说明.md
    └── Go2_RL底层强化学习控制方案评估与执行计划.md
```

### 1.1 最小必须打包的文件

如果只交付“MuJoCo 中运行 Go2 RL 底层控制”的最小包，必须包含：

```text
projects/go2_control_demos/go2_mujoco_rl_policy_runner.py
projects/open_source_deploy_simtoreal_rl_go2/deploy_real/configs/go2.yaml
projects/open_source_deploy_simtoreal_rl_go2/pre_train/policy_rough.pt
src/unitree_mujoco/unitree_robots/go2/scene.xml
src/unitree_mujoco/unitree_robots/go2/go2.xml
src/unitree_mujoco/unitree_robots/go2/assets/
docs/Go2_MuJoCo_RL底层控制交付工作流说明.md
```

原因：

```text
runner 负责控制逻辑。
go2.yaml 负责告诉 runner policy 的动作缩放、obs 维度、cmd 缩放等训练相关参数。
policy_rough.pt 是真正的神经网络策略。
scene.xml / go2.xml / assets 是 MuJoCo Go2 机器人和场景模型。
```

### 1.2 推荐一起打包的文件

为了方便别人继续追溯训练来源和实机部署参考，推荐额外包含：

```text
projects/open_source_deploy_simtoreal_rl_go2/README.md
projects/open_source_deploy_simtoreal_rl_go2/doc/Isaaclab.md
projects/open_source_deploy_simtoreal_rl_go2/doc/Deploy.md
projects/open_source_deploy_simtoreal_rl_go2/deploy_real/deploy_real_isaaclab.py
projects/open_source_deploy_simtoreal_rl_go2/deploy_real/common/rotation_helper.py
projects/go2_control_demos/README.md
docs/Go2_RL底层强化学习控制方案评估与执行计划.md
```

这些文件不是 MuJoCo runner 运行的直接依赖，但它们说明：

```text
policy_rough.pt 是如何训练和部署的。
真实 Go2 部署脚本如何拼 observation。
projected gravity 在原作者代码里如何计算。
实机部署时如何使用 Unitree SDK2、ROS2 和 Kalman filter。
```

---

## 2. 这套流程到底在做什么

当前流程是一套：

```text
训练好的低层 RL policy
  -> MuJoCo Go2 仿真
  -> 目标速度命令 cmd
  -> 12 个目标关节角
  -> 高频 PD 控制
  -> Go2 稳定运动
```

它不是宇树高层 Sport API。

两者区别：

```text
宇树高层控制：
  你给 vx / vy / yaw_rate
  宇树机器人内部高层控制器自动生成步态和电机控制

当前低层 RL 控制：
  你给 vx / vy / yaw_rate
  神经网络 policy 根据机器人状态输出 12 维 action
  action 被转换成 12 个目标关节角
  外部 PD 控制器驱动 12 个关节
```

当前底层控制闭环：

```text
MuJoCo 机器人状态
  -> 52 维 observation
  -> policy_rough.pt
  -> 12 维 action
  -> target_q = default_q + action * action_scale
  -> 关节软限幅
  -> MuJoCo qpos 顺序转换
  -> PD 力矩 tau = kp * (q_des - q) - kd * dq
  -> MuJoCo actuator ctrl
  -> mj_step
  -> 下一帧机器人状态
```

---

## 3. policy_rough.pt 是怎么来的，起什么作用

文件位置：

```text
projects/open_source_deploy_simtoreal_rl_go2/pre_train/policy_rough.pt
```

来源：

```text
policy_rough.pt 来自 TheoBounac/Deploy_SimToReal_RL_Go2 项目。
该项目的目标是在 IsaacLab 中训练 Unitree Go2 的低层强化学习运动控制 policy，
然后通过 Unitree SDK2 Python 部署到真实 Go2。
```

训练端参考文件：

```text
projects/open_source_deploy_simtoreal_rl_go2/doc/Isaaclab.md
```

TheoBounac 文档中给出的训练命令形式：

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
  --task=Isaac-Velocity-Rough-Unitree-Go2-v0 \
  --num_envs=40
```

这表示：

```text
训练环境：IsaacLab
任务类型：速度跟踪 locomotion
机器人：Unitree Go2
地形：rough / rough terrain 相关任务
算法：rsl_rl PPO 训练流程
输出结果：可部署的 TorchScript policy
```

在当前 MuJoCo 流程中，`policy_rough.pt` 的作用是：

```text
输入：52 维 observation
输出：12 维 action
```

其中：

```text
52 维 observation 描述机器人当前状态和期望速度命令。
12 维 action 表示每个关节相对默认站姿的动作偏移。
```

注意：

```text
policy_rough.pt 不是“完整控制器”。
它只是神经网络本体。
完整控制器还必须包括：
1. observation 拼接方式
2. action 缩放方式
3. default_q 默认关节角
4. 关节顺序映射
5. PD 控制器
6. 控制频率
7. 安全限幅
8. 摔倒保护
```

所以不能只把 `policy_rough.pt` 拿出来单独运行。

---

## 4. go2.yaml 的作用

文件位置：

```text
projects/open_source_deploy_simtoreal_rl_go2/deploy_real/configs/go2.yaml
```

它是 TheoBounac 实机部署流程的配置文件。当前 MuJoCo runner 会读取其中一部分关键字段。

核心字段：

```yaml
policy_path: "policy_rough.pt"
action_scale: 0.25
cmd_scale: [0.8, 0.8, 1]
num_actions: 12
num_obs: 52
max_cmd: [1, 1, 1]
```

### 4.1 policy_path

```yaml
policy_path: "policy_rough.pt"
```

含义：

```text
告诉 runner 默认加载 pre_train/policy_rough.pt。
```

当前代码实际解析为：

```text
projects/open_source_deploy_simtoreal_rl_go2/pre_train/policy_rough.pt
```

如果想测试另一个策略，可以用命令行覆盖：

```bash
--policy projects/open_source_deploy_simtoreal_rl_go2/pre_train/policy_rough_2.pt
```

### 4.2 action_scale

```yaml
action_scale: 0.25
```

含义：

```text
policy 输出的 action 不是直接关节角。
它必须乘以 action_scale，再加到默认关节角上。
```

公式：

```text
target_q = default_q + action * action_scale
```

当前：

```text
action_scale = 0.25 rad
```

如果这个值不一致，会导致：

```text
动作太小：狗不动或拖腿
动作太大：抖动、抽搐、摔倒、action clip 频繁
```

### 4.3 cmd_scale 和 max_cmd

```yaml
cmd_scale: [0.8, 0.8, 1]
max_cmd: [1, 1, 1]
```

当前 runner 中速度命令进入 obs 的方式：

```text
obs[13:16] = cmd * cmd_scale * max_cmd
```

其中 `cmd` 是：

```text
[cmd_vx, cmd_vy, cmd_yaw]
```

所以如果用户输入：

```bash
--cmd-vx 0.05
```

则网络实际看到的 vx 命令大约是：

```text
0.05 * 0.8 * 1 = 0.04
```

注意：

```text
cmd_scale 和 max_cmd 是训练/部署配置的一部分。
不要随便改。
如果改了，policy 看到的命令分布会和训练时不一致。
```

### 4.4 num_obs 和 num_actions

```yaml
num_obs: 52
num_actions: 12
```

含义：

```text
policy 输入必须是 52 维。
policy 输出必须是 12 维。
```

当前 runner 启动时会检查：

```text
如果 num_obs 不是 52，直接报错。
如果 num_actions 不是 12，直接报错。
```

这是为了防止把错误 policy 或错误配置混进来。

### 4.5 leg_joint2motor_idx 和 default_angles

```yaml
leg_joint2motor_idx: [3,0,9,6,4,1,10,7,5,2,11,8]
default_angles: [-0.1, 0.8, -1.5, 0.1, 0.8, -1.5, -0.1, 1, -1.5, 0.1, 1, -1.5]
```

这两个字段主要服务于 TheoBounac 的真实 Go2 部署脚本。

当前 MuJoCo runner 没有直接照搬这两个数组，而是显式定义了 policy 顺序：

```python
POLICY_JOINT_NAMES = [
    "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
    "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
    "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",
]
```

以及 policy 顺序下的默认角：

```python
POLICY_DEFAULT_Q = np.array([
    0.1, -0.1, 0.1, -0.1,
    0.8, 0.8, 1.0, 1.0,
    -1.5, -1.5, -1.5, -1.5,
])
```

原因：

```text
TheoBounac 的 go2.yaml default_angles 是实机电机顺序。
policy 实际使用的是另一套 IsaacLab / 部署脚本中的 action 顺序。
MuJoCo qpos 顺序和 actuator 顺序又各自不同。
所以 runner 必须显式管理三套顺序之间的映射。
```

---

## 5. go2_mujoco_rl_policy_runner.py 的作用

文件位置：

```text
projects/go2_control_demos/go2_mujoco_rl_policy_runner.py
```

它是当前交付流程的核心文件。

它负责把：

```text
TheoBounac 的实机低层 RL policy
```

接到：

```text
本地 unitree_mujoco 的 Go2 MuJoCo 模型
```

中运行。

它不是训练脚本。

它是：

```text
策略部署 runner
仿真控制器
Sim2Sim 适配层
安全接入器
```

### 5.1 runner 的主要模块

#### 路径与配置

```python
PROJECT_ROOT
GO2_SCENE_XML
THEO_PROJECT
THEO_CONFIG
THEO_POLICY
```

作用：

```text
定位 MuJoCo Go2 场景、TheoBounac 项目、go2.yaml 和 policy_rough.pt。
```

#### TheoConfig

读取 `go2.yaml` 中当前 runner 需要的字段：

```text
policy_path
action_scale
cmd_scale
max_cmd
num_obs
num_actions
```

#### RunnerConfig

保存 MuJoCo runner 的运行参数：

```text
仿真时长
站立时间
稳定等待时间
policy 推理周期
PD kp/kd
action clip
足端力 scale / clip / binary
摔倒检测阈值
键盘控制开关
固定速度命令
关节映射验证模式
```

#### JointOrder

这是非常关键的类。

作用：

```text
自动读取 MuJoCo 模型中的 joint / qpos / qvel / actuator 顺序，
并建立 policy 顺序、MuJoCo qpos 顺序、MuJoCo actuator 顺序之间的转换。
```

为什么必须有它：

```text
policy 输出的 12 维 action 有自己的顺序。
MuJoCo qpos[7:19] 有自己的顺序。
MuJoCo data.ctrl actuator 又有自己的顺序。
这三者不一致。
如果顺序错了，Go2 会出现抽搐、腿反向运动、前进变后退、站不稳。
```

#### 状态机

当前状态机：

```text
PD_STAND_UP
STABILIZE
RL_CONTROL
DAMPING_STOP
```

流程：

```text
1. PD_STAND_UP:
   从 MuJoCo 初始姿态平滑插值到 policy 默认站姿。

2. STABILIZE:
   保持默认站姿一小段时间，让机身速度和关节速度稳定。

3. RL_CONTROL:
   policy 接管。
   低频推理 action。
   高频 PD 控制关节。

4. DAMPING_STOP:
   如果高度过低或 roll/pitch 过大，进入安全阻尼态。
```

---

## 6. 52 维 observation 是怎么拼的

这是整套流程最重要的部分。

当前 policy 训练/部署时要求的 observation 是 52 维。

当前 runner 拼接方式：

```text
0:4     foot_force
4:7     base_lin_vel
7:10    base_ang_vel
10:13   projected_gravity
13:16   cmd
16:28   q - default_q
28:40   dq
40:52   last_action
```

### 6.1 obs[0:4] foot_force

来源：

```text
MuJoCo contact force
```

处理：

```text
1. 遍历 MuJoCo contact。
2. 找到足端 geom 的接触。
3. 使用 mj_contactForce 读取法向力。
4. 按 LowState 常见顺序整理为 [FR, FL, RR, RL]。
5. 再按 TheoBounac 的顺序重排为 [foot[1], foot[3], foot[0], foot[2]]。
6. 默认除以 100，并 clip 到 0~5。
```

相关参数：

```bash
--foot-force-scale 100
--foot-force-clip 5
--foot-force-binary
--foot-contact-threshold 20
```

注意：

```text
TheoBounac 实机脚本直接使用 foot_force / 100。
MuJoCo 的接触力可能有尖峰，所以当前多加了 clip。
如果怀疑足端力幅值导致抖动，可以临时加 --foot-force-binary 变成 0/1 接触指示器。
```

### 6.2 obs[4:7] base_lin_vel

来源：

```text
MuJoCo data.qvel[0:3]
```

处理：

```text
MuJoCo free joint 的 qvel[0:3] 是世界坐标系线速度。
policy 需要机身坐标系下的 base linear velocity。
所以需要用 base 四元数旋转到 body frame。
```

公式：

```text
v_body = R(q)^T * v_world
```

### 6.3 obs[7:10] base_ang_vel

来源：

```text
MuJoCo data.qvel[3:6]
```

处理：

```text
不要再次旋转。
```

原因：

```text
MuJoCo free joint 的 qvel[3:6] 已经更接近机身局部角速度，
语义上和实机 IMU gyroscope 更接近。
如果再 rotate_world_to_body 一次，会重复旋转，导致 policy 看到错误角速度。
```

这是当前已经修正过的关键点。

### 6.4 obs[10:13] projected_gravity

含义：

```text
世界坐标系单位重力 [0, 0, -1] 在机器人机身坐标系中的表达。
```

公式：

```text
g_body = R(q)^T * [0, 0, -1]
```

作用：

```text
让 policy 知道机身当前姿态。
```

如果 projected_gravity 错了，常见现象：

```text
站不稳
身体补偿方向错误
原地抖动
很快摔倒
```

### 6.5 obs[13:16] cmd

内容：

```text
[cmd_vx, cmd_vy, cmd_yaw]
```

处理：

```text
obs[13:16] = cmd * cmd_scale * max_cmd
```

命令来源：

```text
1. 命令行固定参数：--cmd-vx / --cmd-vy / --cmd-yaw
2. MuJoCo viewer 键盘：--enable-keyboard-cmd
```

键盘：

```text
↑/↓  前进/后退
←/→  左右平移
Q/E  左右旋转
空格/R 速度命令清零
```

### 6.6 obs[16:28] q - default_q

内容：

```text
当前 12 个关节角 - policy 默认关节角
```

注意：

```text
必须是 policy joint order。
不能直接拿 MuJoCo qpos[7:19] 塞进去。
```

### 6.7 obs[28:40] dq

内容：

```text
当前 12 个关节速度
```

注意：

```text
也必须转换成 policy joint order。
```

### 6.8 obs[40:52] last_action

内容：

```text
上一帧 policy 输出的 12 维 action
```

注意：

```text
这里不是 PD ctrl。
不是 target_q。
不是当前关节角。
不是当前关节速度。
```

作用：

```text
提供动作历史，让 policy 具备运动连续性。
```

---

## 7. action 是怎么变成关节控制的

policy 输出：

```text
action: 12 维
```

先做 action 限幅：

```text
clipped_action = clip(action, -action_clip, action_clip)
```

默认：

```text
action_clip = 3.0
```

然后转成目标关节角：

```text
target_policy = default_policy + clipped_action * action_scale
```

当前：

```text
action_scale = 0.25
```

再做关节软限幅：

```text
target_policy = soft_clip_policy_targets(target_policy)
```

再转换为 MuJoCo qpos 顺序：

```text
target_qpos = policy_to_qpos(target_policy)
```

最后用 PD 控制：

```text
tau = kp * (target_qpos - q) - kd * dq
```

并把 qpos 顺序力矩转换成 actuator 顺序：

```text
ctrl = qpos_to_actuator_vec(tau)
```

最后写入：

```text
data.ctrl
```

---

## 8. 控制频率

当前 MuJoCo 模型物理步长：

```text
sim_dt = 0.002 s
```

也就是：

```text
500 Hz
```

当前 policy 推理周期：

```text
policy_dt = 0.02 s
```

也就是：

```text
50 Hz
```

所以：

```text
每 1 次 policy 推理
MuJoCo 底层 PD 会执行约 10 次
```

代码中叫：

```text
policy_decimation = round(policy_dt / sim_dt)
```

为什么要这样：

```text
神经网络不需要每 0.002 秒都推理。
低层物理仿真和 PD 控制需要更高频率。
这符合四足机器人常见的“低频策略 + 高频 PD 内环”结构。
```

---

## 9. 安全机制

当前 runner 的安全机制包括：

```text
1. PD 缓起
2. policy 接管前稳定等待
3. action clip
4. target_q 关节软限幅
5. 速度命令限幅
6. 摔倒检测
7. DAMPING_STOP 阻尼安全态
```

### 9.1 PD_STAND_UP

作用：

```text
避免 policy 直接从趴下、半蹲或姿态混乱状态接管。
```

实现：

```text
target_q(t) = start_q * (1 - alpha) + default_q * alpha
```

其中：

```text
alpha 使用 smoothstep 从 0 平滑增加到 1。
```

默认：

```text
stand_time = 2.5 s
stand_kp = 60
stand_kd = 5
```

### 9.2 STABILIZE

作用：

```text
站起来后保持默认姿态，让速度和姿态稳定，再让 policy 接管。
```

默认：

```text
stabilize_time = 1.5 s
```

### 9.3 DAMPING_STOP

触发条件：

```text
base 高度低于 fall_height
或 pitch / roll 超过 fall_angle
```

默认：

```text
fall_height = 0.13 m
fall_angle = 45 deg
```

DAMPING_STOP 控制方式：

```text
进入 DAMPING_STOP 时记录当前关节角 q_hold。
短时间使用很弱的 kp 保持 q_hold。
kp 在约 2 秒内衰减到 0。
全程保留 -kd * dq 阻尼。
```

公式：

```text
tau = hold_kp(t) * (q_hold - q) - damping_kd * dq
```

默认：

```text
damping_kd = 2.0
damping_hold_kp_initial = 8.0
damping_hold_decay_time = 2.0 s
```

为什么不直接 ctrl=0：

```text
如果直接 ctrl=0，机器人可能完全软塌并发生剧烈碰撞。
如果目标角突然变 0，关节可能被拉向不自然姿态。
当前弱保持 + 阻尼释放更平滑。
```

---

## 10. 运行环境

当前推荐使用已有虚拟环境：

```text
/home/ros/unitree_dev/.venv-unitree
```

运行时统一使用干净环境前缀：

```bash
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python ...
```

原因：

```text
当前 WSL 默认 shell 里可能残留 DPVO/conda 的 LD_LIBRARY_PATH 和 PYTHONPATH。
这些变量可能污染 MuJoCo、torch、yaml、OpenGL 等依赖。
```

最小 Python 依赖：

```text
mujoco
torch
numpy
pyyaml
```

如果在新机器上重建环境，可以参考：

```bash
cd /home/ros/unitree_dev
python3 -m venv .venv-unitree
.venv-unitree/bin/pip install --upgrade pip
.venv-unitree/bin/pip install mujoco numpy pyyaml torch
```

如果新机器没有图形界面，仍然可以先跑 headless 测试。

---

## 11. 运行命令

所有命令都从工程根目录运行：

```bash
cd /home/ros/unitree_dev
```

### 11.1 语法检查

```bash
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python -m py_compile \
  projects/go2_control_demos/go2_mujoco_rl_policy_runner.py
```

### 11.2 无窗口零速度自检

```bash
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/go2_mujoco_rl_policy_runner.py \
  --headless-test --headless-duration 60
```

通过标准：

```text
state=RL_CONTROL
没有进入 DAMPING_STOP
height 稳定
pitch / roll 较小
clips 最好为 0
```

### 11.3 固定命令测试

前进：

```bash
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/go2_mujoco_rl_policy_runner.py \
  --headless-test --headless-duration 30 --cmd-vx 0.05
```

后退：

```bash
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/go2_mujoco_rl_policy_runner.py \
  --headless-test --headless-duration 30 --cmd-vx -0.05
```

左平移：

```bash
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/go2_mujoco_rl_policy_runner.py \
  --headless-test --headless-duration 30 --cmd-vy 0.05
```

右平移：

```bash
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/go2_mujoco_rl_policy_runner.py \
  --headless-test --headless-duration 30 --cmd-vy -0.05
```

左转：

```bash
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/go2_mujoco_rl_policy_runner.py \
  --headless-test --headless-duration 30 --cmd-yaw 0.05
```

右转：

```bash
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/go2_mujoco_rl_policy_runner.py \
  --headless-test --headless-duration 30 --cmd-yaw -0.05
```

绕圈：

```bash
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/go2_mujoco_rl_policy_runner.py \
  --headless-test --headless-duration 30 --cmd-vx 0.05 --cmd-yaw 0.05
```

### 11.4 打开 MuJoCo viewer

零速度站稳：

```bash
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/go2_mujoco_rl_policy_runner.py
```

固定速度打开 viewer：

```bash
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/go2_mujoco_rl_policy_runner.py \
  --cmd-vx 0.05
```

键盘控制：

```bash
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/go2_mujoco_rl_policy_runner.py \
  --enable-keyboard-cmd --cmd-step 0.02
```

键盘说明：

```text
↑/↓  前进/后退
←/→  左右平移
Q/E  左右旋转
空格/R 速度命令清零
```

建议：

```text
初次操作时 cmd-step 用 0.02。
一次只按一下方向键。
观察稳定后再继续加。
不要一开始连续猛按。
```

### 11.5 关节映射验证

```bash
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/go2_mujoco_rl_policy_runner.py \
  --verify-joints --verify-joint FR_thigh_joint
```

可验证关节：

```text
FL_hip_joint
FR_hip_joint
RL_hip_joint
RR_hip_joint
FL_thigh_joint
FR_thigh_joint
RL_thigh_joint
RR_thigh_joint
FL_calf_joint
FR_calf_joint
RL_calf_joint
RR_calf_joint
```

这个模式用于检查：

```text
policy 顺序 -> MuJoCo qpos 顺序 -> actuator 顺序
```

是否正确。

---

## 12. 当前已经验证通过的能力

当前本地测试结论：

```text
1. 零速度站立稳定。
2. 前进稳定。
3. 后退稳定。
4. 左右平移稳定。
5. 左右旋转稳定。
6. 小速度绕圈稳定。
7. MuJoCo viewer 中可以实时观察 Go2 运动。
8. headless 模式可以用于无窗口自动验收。
```

当前建议的安全速度范围：

```text
cmd_vx:  -0.05 到 0.10 起步
cmd_vy:  -0.05 到 0.05 起步
cmd_yaw: -0.05 到 0.05 起步
```

后续可以逐步增大，但每次增大后都要重新观察：

```text
height
pitch
roll
max|action|
clips
是否进入 DAMPING_STOP
```

---

## 13. 给后续开发者的入口

### 13.1 如果要做路径点跟踪

不要改 policy。

应该在 runner 上层写一个路径点控制器：

```text
当前位置 x, y, yaw
目标点 x_goal, y_goal
计算距离误差和朝向误差
生成 cmd_vx / cmd_vy / cmd_yaw
喂给 go2_mujoco_rl_policy_runner.py
```

推荐新建：

```text
projects/go2_control_demos/go2_rl_waypoint_follower.py
```

控制结构：

```text
waypoint follower
  -> cmd_vx / cmd_vy / cmd_yaw
  -> RL runner
  -> policy
  -> PD
  -> MuJoCo Go2
```

### 13.2 如果要接 ROS2 / Nav2

推荐做法：

```text
ROS2 /cmd_vel
  -> bridge
  -> cmd_vx / cmd_vy / cmd_yaw
  -> go2_mujoco_rl_policy_runner.py
```

不要让 Nav2 直接操作关节。

Nav2 应该只输出：

```text
geometry_msgs/Twist
```

低层稳定运动继续交给 RL policy。

### 13.3 如果要接 SLAM / 视觉定位

推荐职责划分：

```text
ORB-SLAM3 / DPVO / VGGT:
  负责估计相机位姿、轨迹、地图。

导航规划模块:
  根据当前位置和目标点生成期望速度。

RL runner:
  根据期望速度和机器人状态生成稳定关节控制。
```

也就是：

```text
视觉定位不直接控制腿。
视觉定位给出位姿。
规划器给出速度命令。
RL policy 负责底层运动。
```

### 13.4 如果要换 policy

可以换：

```bash
--policy path/to/new_policy.pt
```

但必须确认：

```text
1. 新 policy 输入仍然是 52 维。
2. 新 policy 输出仍然是 12 维。
3. observation 顺序完全一致。
4. default_q 一致。
5. action_scale 一致。
6. cmd_scale 一致。
7. 关节顺序一致。
```

如果新 policy 来自不同 IsaacLab 任务，通常不能直接替换。

必须先重新确认该训练环境的 observation 定义。

---

## 14. 最容易出错的地方

### 14.1 observation 维度和顺序

不能只看维度是 52。

还必须确认每一维含义一致。

错一个维度，policy 输出就可能完全错误。

### 14.2 角速度坐标系

当前正确处理：

```text
base_lin_vel: qvel[0:3] 从 world 转 body
base_ang_vel: qvel[3:6] 直接使用，不再旋转
```

不要把角速度重复旋转。

### 14.3 projected_gravity 四元数顺序

MuJoCo root 四元数：

```text
[w, x, y, z]
```

很多库使用：

```text
[x, y, z, w]
```

不要混用。

### 14.4 关节顺序

当前至少有三套顺序：

```text
policy joint order
MuJoCo qpos order
MuJoCo actuator order
```

必须通过 `JointOrder` 转换。

不要直接假设 12 维数组顺序一样。

### 14.5 action_scale

不要把 action 直接当关节角。

必须：

```text
target_q = default_q + action * action_scale
```

### 14.6 控制频率

不要让 policy 每个 MuJoCo step 都推理。

当前结构：

```text
MuJoCo 500 Hz
policy 50 Hz
PD 内环约 10 次
```

### 14.7 足端力

默认：

```text
foot_force / 100 后 clip 到 0~5
```

如果出现莫名抖动，可测试：

```bash
--foot-force-binary
```

但二值模式只是排查工具，不一定优于默认模式。

### 14.8 实机误用

当前文件不要直接上实机。

真实 Go2 低层控制必须额外处理：

```text
Unitree SDK2 通信
LowCmd CRC
运动模式释放
遥控器接管
急停
电机保护
架空测试
真实传感器噪声和延迟
```

---

## 15. 推荐交付说明

如果把这套代码交给别人，建议这样说明：

```text
这是一个已经在 MuJoCo 中验证可用的 Go2 低层 RL locomotion runner。

核心文件是：
projects/go2_control_demos/go2_mujoco_rl_policy_runner.py

它使用 TheoBounac/Deploy_SimToReal_RL_Go2 提供的：
projects/open_source_deploy_simtoreal_rl_go2/pre_train/policy_rough.pt

配置来自：
projects/open_source_deploy_simtoreal_rl_go2/deploy_real/configs/go2.yaml

MuJoCo 机器人模型来自：
src/unitree_mujoco/unitree_robots/go2/

运行前先读：
docs/Go2_MuJoCo_RL底层控制交付工作流说明.md
```

交付后的第一条验证命令：

```bash
cd /home/ros/unitree_dev

env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/go2_mujoco_rl_policy_runner.py \
  --headless-test --headless-duration 60
```

如果这条通过，再开 viewer：

```bash
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/go2_mujoco_rl_policy_runner.py \
  --enable-keyboard-cmd --cmd-step 0.02
```

---

## 16. 后续建议路线

当前底层控制已经稳定后，建议后续按这个顺序继续：

```text
第 1 步：给 runner 增加 CSV 日志
  记录 t, x, y, z, yaw, pitch, roll, cmd, action, clips。

第 2 步：写自动 motion_test
  自动执行前进、后退、横移、旋转、绕圈命令并生成测试报告。

第 3 步：写 waypoint follower
  给 3 个目标点，让 Go2 在 MuJoCo 中依次走过去。

第 4 步：接 ROS2 /cmd_vel
  让外部导航模块只需要发布 Twist。

第 5 步：接 SLAM / 视觉定位
  用 ORB-SLAM3 / DPVO / VGGT 提供位姿或地图。

第 6 步：接 Nav2 或自写规划器
  从目标点生成速度命令。

第 7 步：考虑 IsaacLab 重新训练更适合当前 MuJoCo 和真实 Go2 的 policy
  加 domain randomization、延迟、摩擦、质量扰动。
```

---

## 17. 一句话总结

这套交付流程的核心思想是：

```text
不要自己手写复杂步态。
用已经训练好的 Go2 低层 RL policy 负责稳定运动。
MuJoCo runner 负责把仿真状态严格转换成 policy 训练时看到的 observation。
policy 输出 action 后，再通过安全的 PD 内环驱动 MuJoCo Go2。
上层 SLAM、导航、视觉模块只需要产生 cmd_vx / cmd_vy / cmd_yaw。
```

