# Go2 RL 底层强化学习控制方案评估与执行计划

本文用于回答当前问题：

```text
能不能用 unitree_rl_gym / Isaac Gym 或 IsaacLab 训练 Go2 底层强化学习策略，
再用社区部署框架把策略迁移到 MuJoCo / 实机，实现稳定鲁棒运动？
```

结论：

```text
方案可行，但不能理解为“一键完美迁移”。

训练端可以用：
1. 宇树官方 unitree_rl_gym 中的 go2 任务，偏 Isaac Gym / legged_gym 路线。
2. TheoBounac 的 IsaacLab Go2 路线，偏 IsaacLab 路线，并提供 Go2 预训练策略和实机部署参考。

部署端可以参考：
TheoBounac/Deploy_SimToReal_RL_Go2 的低层部署脚本。

但 Isaac Gym / IsaacLab 训练出的 policy 迁移到 MuJoCo 时，需要严格对齐：
观测量顺序、动作顺序、关节顺序、默认关节角、PD 参数、控制周期、坐标系、接触模型。
```

## 1. 当前本地仓库状态

### 1.1 宇树官方 unitree_rl_gym

本地路径：

```text
/home/ros/unitree_dev/projects/open_source_unitree_rl_gym
```

远端：

```text
https://github.com/unitreerobotics/unitree_rl_gym.git
```

它的 README 中说明支持：

```text
Go2 / G1 / H1 / H1_2
```

Go2 训练配置在：

```text
/home/ros/unitree_dev/projects/open_source_unitree_rl_gym/legged_gym/envs/go2/go2_config.py
```

重要限制：

```text
当前本地这份官方 unitree_rl_gym 里有 Go2 训练配置，
但 deploy/pre_train 下没有 Go2 的预训练 motion.pt，
deploy/deploy_mujoco/configs 下也没有 Go2 的 go2.yaml。
```

所以它适合：

```text
学习 Go2 底层 RL 训练原理
自己训练 Go2 policy
理解 action -> 目标关节角 -> PD 控制 -> 物理仿真的闭环
```

但它暂时不适合直接：

```text
一键拿官方 Go2 预训练模型去 MuJoCo 跑稳定步态
```

### 1.2 TheoBounac/Deploy_SimToReal_RL_Go2

本地路径：

```text
/home/ros/unitree_dev/projects/open_source_deploy_simtoreal_rl_go2
```

远端：

```text
https://github.com/TheoBounac/Deploy_SimToReal_RL_Go2.git
```

它的定位是：

```text
在 IsaacLab 中训练 Go2 RL 策略，并通过 Unitree SDK2 Python 部署到真实 Go2。
同时使用 ROS2 和 Kalman filter 做实时状态估计。
```

它提供了：

```text
deploy_real/configs/go2.yaml
deploy_real/deploy_real_isaaclab.py
pre_train/policy_rough.pt
pre_train/policy_rough_2.pt
```

我已经做过本地接口检查：

```bash
cd /home/ros/unitree_dev
bash scripts/go2_rl_policy_check.sh
```

检查结果：

```text
policy_rough.pt    输入观测维度 (1, 52)，输出动作维度 (1, 12)
policy_rough_2.pt  输入观测维度 (1, 52)，输出动作维度 (1, 12)
```

这说明这两个 policy 至少满足：

```text
Go2 低层策略输入：52 维观测
Go2 低层策略输出：12 维动作
```

## 2. 这套方案的控制原理

低层 RL 控制不是高层 Sport API。

高层 Sport API 是：

```text
目标速度 vx / vy / yaw_rate
  -> 宇树内置控制器
  -> 稳定步态和电机控制由机器人内部完成
```

低层 RL policy 是：

```text
机器人状态观测 obs
  -> 神经网络 policy
  -> 12 维 action
  -> action_scale 缩放
  -> default_joint_angles + action * action_scale
  -> 12 个目标关节角
  -> PD 控制器
  -> 电机 LowCmd / 仿真器 ctrl
```

TheoBounac 的 Go2 配置里写了：

```yaml
num_obs: 52
num_actions: 12
action_scale: 0.25
control_dt: 0.005
leg_joint2motor_idx: [3,0,9,6,4,1,10,7,5,2,11,8]
default_angles: [-0.1, 0.8, -1.5, 0.1, 0.8, -1.5, -0.1, 1, -1.5, 0.1, 1, -1.5]
```

这几个参数非常关键：

```text
num_obs 决定 policy 输入维度
num_actions 决定 policy 输出维度
action_scale 决定 action 对关节目标角的影响大小
control_dt 决定控制周期
leg_joint2motor_idx 决定 IsaacLab 关节顺序到真实 Go2 电机顺序的映射
default_angles 决定 action=0 时的默认姿态
```

只要这些和 MuJoCo 里的模型不一致，就会出现：

```text
站起来后倒下
前进变后退
腿反向运动
身体抖动
原地抽搐
关节看起来顺序乱了
```

## 3. Isaac Gym / IsaacLab 策略能不能完美迁移到 MuJoCo

不能说完美迁移。

更准确的说法是：

```text
Isaac Gym / IsaacLab 训练出的策略，可以迁移到 MuJoCo 做 Sim2Sim 验证，
但迁移效果取决于训练和部署之间是否严格对齐。
```

### 3.1 为什么不能保证完美

Isaac Gym / IsaacLab 和 MuJoCo 的差异包括：

```text
1. 接触模型不同：脚底和地面的接触、摩擦、反弹、约束解算不完全一样。
2. 仿真步长不同：训练时的 policy dt、physics dt 和 MuJoCo 的 timestep 可能不一致。
3. 关节驱动不同：Isaac 的 PD / actuator 和 MuJoCo 的 actuator 参数不一定等价。
4. 机器人模型不同：URDF/MJCF 的惯量、质量、碰撞体、关节限位可能不同。
5. 观测量定义不同：base velocity、gravity vector、IMU、foot force 的坐标系可能不同。
6. 动作顺序不同：IsaacLab、MuJoCo、真实 Go2 的关节顺序经常不一样。
7. 归一化不同：obs scale、cmd scale、action scale、clip 范围必须一致。
```

所以 `.pt` 文件只是策略本体，不是完整控制器。

完整控制器还必须包含：

```text
观测构造方式
归一化参数
动作缩放方式
关节顺序映射
PD 参数
控制频率
安全限幅
急停逻辑
```

### 3.2 什么情况下迁移成功率高

迁移成功率高的条件：

```text
1. MuJoCo 使用的 Go2 模型和训练模型的质量、惯量、关节轴、限位接近。
2. 观测量 52 维顺序和训练时完全一致。
3. 12 维 action 到 12 个关节的映射完全一致。
4. default_angles、action_scale、kp、kd 和训练/部署配置一致。
5. 控制频率严格一致，例如 policy 20 ms 更新，底层 PD 更高频执行。
6. 训练时做过 domain randomization，包括摩擦、质量、延迟、噪声、推扰。
7. 先在零速度、低速直行、低速转向下测试，再逐步扩大速度范围。
```

### 3.3 对当前项目的判断

当前方案是可行的，但应该按这个优先级执行：

```text
第一优先级：不要直接上实机。
第二优先级：先复现 TheoBounac 预训练 policy 的接口和观测构造。
第三优先级：把 policy 接入 MuJoCo，做 Sim2Sim。
第四优先级：如果 MuJoCo 稳定，再考虑架空实机低速测试。
第五优先级：最后才是落地实机测试。
```

## 4. 推荐执行路线

### 阶段 0：先确认策略文件可用

运行：

```bash
cd /home/ros/unitree_dev
bash scripts/go2_rl_policy_check.sh
```

期望看到：

```text
输入观测维度: (1, 52)
输出动作维度: (1, 12)
```

这一步只检查 `.pt` 模型能不能加载，不连接机器人，不启动仿真。

### 阶段 1：学习官方 unitree_rl_gym 的 Go2 训练端

重点读：

```text
/home/ros/unitree_dev/projects/open_source_unitree_rl_gym/legged_gym/envs/go2/go2_config.py
/home/ros/unitree_dev/projects/open_source_unitree_rl_gym/legged_gym/envs/base/legged_robot.py
/home/ros/unitree_dev/projects/open_source_unitree_rl_gym/legged_gym/scripts/train.py
/home/ros/unitree_dev/projects/open_source_unitree_rl_gym/legged_gym/scripts/play.py
```

你要重点理解：

```text
default_joint_angles 是什么
action_scale 是什么
PD stiffness/damping 是什么
commands 是怎么采样的
observations 是怎么拼出来的
reward 是怎么鼓励稳定行走的
reset 条件是什么
```

如果 Isaac Gym 环境配置完整，训练命令是：

```bash
cd /home/ros/unitree_dev/projects/open_source_unitree_rl_gym
conda activate unitree-rl
python legged_gym/scripts/train.py --task=go2 --headless --num_envs=1024
```

播放训练结果：

```bash
python legged_gym/scripts/play.py --task=go2 --num_envs=1
```

注意：

```text
这条路线依赖 Isaac Gym，不是 IsaacLab。
如果你不想再单独维护 Isaac Gym 环境，可以优先走现有 IsaacLab 路线。
```

### 阶段 2：利用现有 IsaacLab 环境做 Go2 训练/播放

你当前已经有 IsaacLab 环境：

```text
/home/ros/isaac_go2/IsaacLab
conda 环境：env_isaaclab312
```

检查 IsaacLab：

```bash
cd /home/ros/unitree_dev
bash scripts/isaaclab_check.sh
```

小规模训练冒烟测试：

```bash
cd /home/ros/unitree_dev
NUM_ENVS=32 MAX_ITERATIONS=50 TASK=Isaac-Velocity-Flat-Unitree-Go2-v0 \
  bash scripts/isaaclab_go2_train_small.sh
```

播放时要注意：

```text
没有 checkpoint 时，zero_agent 只是零动作看场景，不代表稳定行走 policy。
要让机器人真正按策略走，需要指定 checkpoint 或使用任务内置预训练 checkpoint。
```

如果你训练出了 checkpoint，可以用：

```bash
cd /home/ros/unitree_dev
CHECKPOINT=/path/to/model.pt NUM_ENVS=4 TASK=Isaac-Velocity-Flat-Unitree-Go2-v0 \
  bash scripts/isaaclab_go2_play.sh
```

### 阶段 3：基于 TheoBounac policy 做 MuJoCo Sim2Sim

这一步是接下来真正要开发的重点。

当前第一版 runner 已创建：

```text
/home/ros/unitree_dev/projects/go2_control_demos/go2_mujoco_rl_policy_runner.py
```

它已经包含：

```text
PD_STAND_UP 缓起
STABILIZE 稳定等待
RL_CONTROL 低频 policy + 高频 PD
DAMPING_STOP 阻尼安全态
TheoBounac 52 维 obs 拼接
prev_action 历史动作
projected_gravity 机身坐标系计算
policy/qpos/actuator 显式关节顺序转换
关节映射验证模式
```

已完成的无窗口验证：

```bash
cd /home/ros/unitree_dev

env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/go2_mujoco_rl_policy_runner.py \
  --headless-test --headless-duration 30
```

结果：

```text
30 秒零速度自检通过。
状态机完整经过 PD_STAND_UP -> STABILIZE -> RL_CONTROL。
policy 接管后未触发 DAMPING_STOP。
```

小速度命令初步验证：

```bash
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/go2_mujoco_rl_policy_runner.py \
  --headless-test --headless-duration 12 --cmd-vx 0.05
```

结果：

```text
12 秒内未触发摔倒保护。
这只说明小速度命令没有立刻破坏稳定性，不代表已经完成行走验证。
```

目标：

```text
把 TheoBounac 的 52 维观测构造和 12 维 action 输出，
接到本地 MuJoCo Go2 模型里，
先实现零速度站立，再实现低速直行、转向。
```

需要做的代码工作：

```text
1. 新建一个 MuJoCo RL policy runner。
2. 加载 policy_rough.pt。
3. 从 MuJoCo data 中读取：
   - base quaternion
   - base angular velocity
   - base linear velocity
   - 12 个关节位置
   - 12 个关节速度
   - 上一帧 action
   - 速度命令 cmd
   - 足底接触/足力估计
4. 拼成和 go2.yaml 完全一致的 52 维 obs。
5. policy(obs) 得到 12 维 action。
6. target_joint = default_angles + action * action_scale。
7. 按 MuJoCo 的关节顺序写 ctrl 或 qpos 目标。
8. 用 PD 控制推进 MuJoCo。
9. 加入安全限幅、姿态检测、摔倒自动停止。
```

最关键的检查点：

```text
关节顺序必须对齐。
坐标系必须对齐。
速度方向必须对齐。
默认角必须对齐。
控制频率必须对齐。
```

#### 3.1 下一步 MuJoCo policy runner 的硬性设计约束

在真正写 runner 之前，先把约束写死。下面这些不是“优化项”，而是能不能稳定 Sim2Sim 的前提。

##### 1. 训练阶段必须重视 Domain Randomization

如果后续要自己训练 policy，并希望它从 IsaacLab 迁移到 MuJoCo、再迁移到真机，训练时必须加入足够的域随机化。

建议至少覆盖：

```text
1. 地面摩擦系数随机化
2. 机器人 base 质量随机化
3. 机器人 base 质心随机化
4. 关节初始位置扰动
5. 初始机身速度扰动
6. 观测噪声
7. 周期性推扰或外力扰动
8. 动作延迟 / 电机响应延迟
9. PD 参数扰动
10. 传感器延迟和速度估计噪声
```

当前本地 IsaacLab 的通用速度任务已经包含一部分随机化：

```text
观测噪声：base_lin_vel、base_ang_vel、projected_gravity、joint_pos、joint_vel、height_scan
base 质量随机化：大约 0.8 到 1.25 倍
base 质心随机化：x/y/z 小范围扰动
reset 初始位姿和速度随机化
interval push：每 10 到 15 秒给机器人一次水平速度扰动
```

但当前通用配置里的摩擦范围看起来是固定值：

```text
static_friction_range: (0.8, 0.8)
dynamic_friction_range: (0.6, 0.6)
```

这对真正 Sim2Real 来说还不够。后续训练更鲁棒的 Go2 policy 时，建议直接启用基础级别的强随机化，例如：

```text
static_friction_range: 0.2 到 1.2
dynamic_friction_range: 0.2 到 1.0
base mass scale: 0.8 到 1.25
base com: x/y 约 ±0.05 m，z 约 ±0.01 m
push disturbance: 每 10 到 15 秒一次中低强度水平扰动
observation noise: 按训练配置开启
action delay / motor lag: 后续加入
```

注意：

```text
成熟四足 locomotion 任务通常能承受较强的基础随机化。
如果从头训练 Go2 policy，优先建议直接打开摩擦、质量、质心、观测噪声、推扰等基础随机化。
这样能减少 policy 对单一仿真物理参数的过拟合，提高 Sim2Sim / Sim2Real 通过率。
只有在训练明显不收敛、奖励长期不增长或姿态长期失败时，再临时缩小随机化范围做排查。
```

##### 2. 必须加入 PD 缓起阶段

RL policy 不能直接从趴下、摔倒、四脚姿态混乱的状态接管。

下一步 MuJoCo runner 必须分成至少 4 个状态：

```text
INIT
  加载模型、policy、关节映射，初始化 MuJoCo。

PD_STAND_UP
  用传统 PD 控制把 12 个关节在 2 到 3 秒内平滑拉到默认站立角。

STABILIZE
  保持默认站立角 1 到 2 秒，等待机身姿态和关节速度稳定。

RL_CONTROL
  只有姿态正常、关节速度正常、没有明显摔倒时，才把控制权交给 policy。
```

PD 缓起必须使用插值：

```text
target_q(t) = start_q * (1 - alpha) + default_q * alpha
alpha 从 0 平滑增加到 1
```

不要一步把关节角跳到 default angles。这样会导致：

```text
仿真中瞬间大力矩
关节速度异常
身体弹飞
policy 接管时 obs 已经不可信
实机中可能伤电机或伤人
```

##### 3. 52 维 obs 必须按 policy 的来源逐维复刻

这里最重要的一点：

```text
不要把“通用 IsaacLab Go2 任务”的 obs 和 “TheoBounac 预训练 policy”混用。
```

当前本地 IsaacLab 自带 Go2 速度任务的 policy obs 包含：

```text
base_lin_vel
base_ang_vel
projected_gravity
velocity_commands
joint_pos
joint_vel
actions
height_scan
```

而 TheoBounac 预训练 policy 的实机部署脚本实际拼出来的是 52 维：

```text
0:4     foot_force / 100
4:7     base_lin_vel
7:10    base_ang_vel / gyroscope
10:13   projected_gravity
13:16   cmd * cmd_scale * max_cmd
16:28   q - default_joint
28:40   dq
40:52   last_action
```

所以对于 `policy_rough.pt` 和 `policy_rough_2.pt`，下一步 MuJoCo runner 必须先复刻 TheoBounac 的这套 52 维格式。

特别注意：

```text
go2.yaml 里写了 ang_vel_scale、dof_pos_scale、dof_vel_scale，
但 deploy_real_isaaclab.py 的实际 obs 拼接并没有把所有 scale 都乘进去。
```

因此，第一版 runner 的原则是：

```text
优先复刻部署脚本的实际行为，而不是按变量名猜测。
```

之后如果我们要使用自己在 IsaacLab 训练的新 policy，则必须重新导出对应训练环境的 obs 定义，再重写 obs builder。

当前 `go2_mujoco_rl_policy_runner.py` 已经按这个原则实现，并补充了 3 个关键修正：

```text
1. base_lin_vel:
   MuJoCo free joint 的 qvel[0:3] 是世界坐标系线速度，
   进入 policy 前需要旋转到机身坐标系。

2. base_ang_vel:
   MuJoCo free joint 的 qvel[3:6] 已经是机身局部角速度，
   和实机 IMU gyroscope 的语义更接近，
   所以不能再做 rotate_world_to_body。
   如果重复旋转，policy 会看到错误的身体角速度，
   常见表现是抖动、补偿方向错误、原地抽搐或摔倒。

3. foot_force:
   TheoBounac 实机部署脚本直接使用 foot_force / 100。
   MuJoCo 的 mj_contactForce 是理想刚体接触力，可能出现尖峰，
   所以当前 runner 默认仍按 /100 复刻，但额外加了 clip，
   并提供二值接触模式用于排查接触力尺度问题。
```

相关参数：

```bash
--foot-force-scale 100
--foot-force-clip 5
--foot-force-binary
--foot-contact-threshold 20
```

排查建议：

```text
如果默认模式下站立明显抖动，可以试试 --foot-force-binary。
如果二值模式更稳定，说明当前 MuJoCo 中接触力幅值和 policy 训练/实机部署分布差距较大。
如果两种模式都不稳定，应优先检查关节顺序、角速度坐标系、default_q、kp/kd。
```

##### 4. last_action 必须是上一帧 policy action

TheoBounac 的 52 维 obs 中最后 12 维是：

```text
40:52 last_action
```

这里的 `last_action` 必须是：

```text
上一帧网络 policy 推理输出的 12 维 action
```

不能用：

```text
MuJoCo 高频 PD 循环中的 ctrl
target_q
当前关节位置 q
当前关节速度 dq
```

原因：

```text
policy 训练时看到的是上一轮神经网络动作，用它表达动作历史和平滑性。
MuJoCo 底层 2 ms PD 循环会执行多次，但这些 inner-loop ctrl 不是 policy 语义下的 action。
```

当前 runner 中已经有一个成员变量：

```text
prev_action
```

循环逻辑应该是：

```text
1. 拼 obs 时，把 prev_action 放入 obs[40:52]。
2. policy(obs) 得到 action。
3. 用 action 计算 target_q。
4. 执行若干次 MuJoCo PD step。
5. 这一轮结束时 prev_action = action。
```

##### 5. projected_gravity 必须从世界重力旋转到机身坐标系

`obs[10:13]` 是 projected gravity，这是机器人能不能判断自身姿态的命门。

正确含义：

```text
把世界坐标系下的单位重力向量 [0, 0, -1]
通过 base 四元数的逆旋转
表达在机器人机身坐标系 Base Body Frame 下。
```

公式：

```text
g_body = R(q)^T * [0, 0, -1]
```

等价写法：

```text
g_body = R(q)^(-1) * [0, 0, -1]
```

注意：

```text
MuJoCo root qpos 中 free joint 的四元数通常是 [w, x, y, z]。
很多 Python 库或 ROS 消息习惯用 [x, y, z, w]。
下一步代码中必须显式写出 MuJoCo 的 [w, x, y, z] 转旋转矩阵函数，避免顺序混淆。
```

绝对不要：

```text
直接把世界坐标系下的 IMU/重力/姿态数据塞进 obs[10:13]。
```

如果 projected gravity 错了，policy 会以为身体朝向完全不同，常见结果就是：

```text
站不稳
原地抽搐
向错误方向补偿
前进变后退
很快摔倒
```

##### 6. 关节顺序必须用显式转换函数，不用魔术数字散落在代码里

关节顺序是 Sim2Sim 的重灾区。

常见顺序：

```text
IsaacLab / Legged Gym policy 顺序：
FL_hip, FL_thigh, FL_calf,
FR_hip, FR_thigh, FR_calf,
RL_hip, RL_thigh, RL_calf,
RR_hip, RR_thigh, RR_calf

宇树 Go2 LowCmd 电机顺序常见为：
FR_hip, FR_thigh, FR_calf,
FL_hip, FL_thigh, FL_calf,
RR_hip, RR_thigh, RR_calf,
RL_hip, RL_thigh, RL_calf

MuJoCo 顺序：
取决于 XML 中 joint / actuator 的定义顺序，必须读取模型确认。
```

TheoBounac 配置中的映射是：

```text
leg_joint2motor_idx: [3,0,9,6,4,1,10,7,5,2,11,8]
```

下一步代码里不要把这串数字到处写。

必须封装成类似：

```text
isaac_to_mujoco_joint_vector(...)
mujoco_to_isaac_joint_vector(...)
isaac_to_motor_joint_vector(...)
```

并且在接 RL 前先做关节映射物理验证：

```text
1. 让 policy 不接管。
2. 单独给某一个关节一个明显但安全的小目标角。
3. 在 MuJoCo 画面里确认动的是预期关节。
4. 12 个关节全部验证后，再接 policy。
```

例如：

```text
只动 FR_thigh，观察右前腿大腿是否弯曲。
只动 FL_calf，观察左前腿小腿是否弯曲。
只动 RR_hip，观察右后腿髋关节是否外展/内收。
```

这个验证必须先做，否则后面 policy 摔倒时很难判断是 policy 问题还是映射问题。

##### 7. action 到目标关节角的公式必须固定

policy 输出的 12 维 action 不能直接当关节角。

必须使用：

```text
target_q = default_q + action * action_scale
```

TheoBounac 的配置中：

```text
action_scale = 0.25
```

也就是：

```text
target_q = default_q + action * 0.25
```

并且 `target_q` 写入 MuJoCo 前必须做软限幅。

##### 8. 频率必须解耦

RL policy 推理频率和 MuJoCo 物理步长不能强行相同。

推荐结构：

```text
MuJoCo sim_dt:       0.002 s，即 500 Hz
RL policy_dt:        0.02 s 左右，即 50 Hz
PD inner loop 次数:  policy_dt / sim_dt，大约 10 次
```

也就是：

```text
每 1 次 policy 推理，得到 1 组 target_q。
随后 MuJoCo 用同一组 target_q 执行约 10 次 PD + mj_step。
下一轮再重新拼 obs，重新推理 policy。
```

TheoBounac 的 `go2.yaml` 里写了：

```text
control_dt: 0.005
```

但它的实机脚本主循环里还有 `time.sleep(0.025)`。因此下一步 MuJoCo runner 要把这些参数做成显式可调：

```text
sim_dt
policy_dt
pd_kp
pd_kd
decimation
```

第一版建议：

```text
sim_dt = 0.002
policy_dt = 0.02 或 0.025
decimation = round(policy_dt / sim_dt)
```

后续根据实际稳定性再微调。

##### 9. 必须有软限幅、摔倒检测和阻尼安全态

下一步 runner 至少要有 4 层安全保护：

```text
1. action 限幅
   例如先 clip 到 [-1, 1] 或按 policy 训练范围处理。

2. target_q 软限幅
   限制在 MuJoCo 模型关节限位内，并留出安全余量。

3. 速度命令限幅
   初始只允许 vx=0、vy=0、yaw_rate=0。
   站稳后再试 vx=0.05 到 0.1 m/s。

4. 姿态摔倒检测
   roll 或 pitch 超过 45 度，立即进入 FALL_STOP。
```

触发摔倒后：

```text
不要在位置控制模式下简单把 ctrl 置零。
不要继续发送上一帧大幅 target_q。
应该进入 DAMPING_STOP 阻尼安全态。
```

阻尼安全态不要只做纯阻尼。更稳的方式是：

```text
1. 触发 DAMPING_STOP 的瞬间记录当前 12 个关节角 damping_hold_qpos。
2. 初期使用很弱的 kp 把目标保持在 damping_hold_qpos。
3. 这个弱 kp 在 1 到 3 秒内逐渐衰减到 0。
4. 全程保留较小 kd，持续消耗关节速度。
```

这样机器人会带阻尼地释放动能，同时避免完全断电式软塌，而不是：

```text
目标角瞬间跳到 0
完全失去力矩像面条一样砸地
高频碰撞反弹
关节速度爆炸
```

在 MuJoCo 中，FALL_STOP 的第一版实现可以是：

```text
tau = hold_kp(t) * (q_hold - q) - kd_stop * dq
其中 hold_kp(t) 从一个小值逐渐衰减到 0
持续若干秒
然后停止仿真或等待用户重置
```

当前 runner 的默认值：

```text
damping_kd = 2.0
damping_hold_kp_initial = 8.0
damping_hold_decay_time = 2.0 s
```

对应命令行参数：

```bash
--damping-kd 2.0
--damping-hold-kp-initial 8.0
--damping-hold-decay-time 2.0
```

如果需要模拟真正断电，可以把：

```bash
--damping-hold-kp-initial 0
```

但这只适合仿真排查，不建议作为实机安全策略。

未来实机中也不能直接照搬“ctrl 置零”，需要走更严格的安全策略：

```text
低层阻尼模式
急停
平滑趴下
或切回宇树官方安全控制流程
```

##### 10. 下一步第一版目标只做站稳

不要第一版就追求走路。

第一版 MuJoCo policy runner 的验收标准是：

```text
1. 能加载 policy_rough.pt。
2. 能完成 PD 缓起。
3. 能拼出 52 维 obs。
4. 能执行 policy 推理。
5. 零速度命令下能站稳 30 到 60 秒。
6. 摔倒检测能正常触发。
7. 结束程序后机器人不会进入凌乱关节状态。
```

只有这个通过后，再做：

```text
低速直行
原地转向
低速绕圈
路径点跟踪
```

### 阶段 4：只在 Sim2Sim 稳定后再考虑实机

实机前必须满足：

```text
1. MuJoCo 中零速度稳定站立 60 秒。
2. MuJoCo 中低速直行 0.1 m/s 稳定。
3. MuJoCo 中低速转向 0.2 rad/s 稳定。
4. MuJoCo 中急停能立刻回到安全站立或趴下。
5. 代码里有 pitch/roll 过大自动停机。
6. 代码里有限制最大 action、最大关节角、最大速度命令。
7. 实机必须先架空测试。
```

不满足这些条件，不要落地实机。

## 5. 当前建议

从工程效率看：

```text
导航、SLAM、视觉项目主线：继续使用 Go2 高层 Sport 控制。
底层运动控制学习支线：使用 unitree_rl_gym + TheoBounac policy 学 RL 控制链路。
```

也就是：

```text
完整项目展示：高层控制更快、更安全、更稳定。
底层理解深度：RL policy 路线更有技术含量，但风险和调参成本更高。
```

接下来建议的第一件事：

```text
先写 MuJoCo policy runner，只做 policy_rough.pt 的 Sim2Sim。
第一版目标不是走路，而是：
1. 加载 policy
2. 拼 obs
3. 输出 action
4. 在 MuJoCo 中保持站立不倒
```

站稳以后，再做：

```text
低速前进
原地转向
低速绕圈
路径点跟踪
```

## 6. 参考链接

```text
宇树官方 unitree_rl_gym：
https://github.com/unitreerobotics/unitree_rl_gym

TheoBounac Go2 Sim-to-Real RL 部署项目：
https://github.com/TheoBounac/Deploy_SimToReal_RL_Go2

IsaacLab：
https://github.com/isaac-sim/IsaacLab

Unitree SDK2 Python：
https://github.com/unitreerobotics/unitree_sdk2_python
```
