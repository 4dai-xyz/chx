# Go2 MuJoCo 运动控制学习代码

这个目录现在只保留一条清晰主线：

```text
PD 站立 -> 手写步态 -> Isaac Lab policy 部署 -> 后续 ROS2/Nav2 接入
```

如果要把当前已经验证成功的 TheoBounac Go2 低层 RL policy + MuJoCo runner 流程交付给别人继续开发，先读：

```text
/home/ros/unitree_dev/docs/Go2_MuJoCo_RL底层控制交付工作流说明.md
```

这份文档说明了 `policy_rough.pt` 的来源和作用、`go2.yaml` 的配置含义、`go2_mujoco_rl_policy_runner.py` 的控制原理、运行命令、验收方法、打包文件清单和后续 SLAM/导航开发入口。

## 1. 当前文件架构

| 文件 | 当前定位 | 是否主线 |
|---|---|---|
| `go2_physics_stand.py` | 动力学 PD 站立/趴下，学习 q/kp/kd/tau 如何驱动关节 | 是 |
| `go2_walk_runner.py` | 不依赖 RL，用保守低速步态生成目标关节角，学习“速度指令 -> 步态 -> PD”的底层链路 | 是 |
| `go2_rl_runner.py` | 当前主入口，把 Isaac Lab 导出的 48 维 policy 接到 MuJoCo PD 控制器 | 是 |
| `go2_mujoco_rl_policy_runner.py` | TheoBounac Go2 52 维预训练 policy 的 MuJoCo Sim2Sim 安全 runner，包含 PD 缓起、last_action、projected gravity、关节映射、阻尼安全态 | 是 |
| `mujoco_go2_control_demos.py` | 早期运动学可视化 demo，适合理解直行/转向/路径点，不代表真实底层控制 | 辅助 |
| `export_go2_policy.py` | 从 Isaac Lab rsl_rl checkpoint 导出 TorchScript policy | 工具 |
| `tools/cmd_vel_bridge.py` | 后续 ROS2 `/cmd_vel` 桥接预留 | 工具 |
| `tools/create_dummy_policy.py` | 创建零动作 dummy policy，用于早期验证 policy 接口 | 工具 |
| `tools/test_cmd_vel_pub.py` | 后续测试 ROS2 cmd_vel 发布 | 工具 |
| `policies/go2_flat_199.pt` | 早期 checkpoint 导出的 policy，动作小，适合做稳定链路验证 | 策略 |
| `policies/go2_flat_2000.pt` | 2000 iteration checkpoint 导出的 policy，动作激进，当前 MuJoCo 中需要安全降档 | 策略 |

已经清理掉的旧入口：

```text
go2_policy_runner.py      旧 47 维 dummy policy 路线，和当前 48 维 Isaac Lab policy 不一致
go2_cmd_vel_runner.py     旧 dummy policy + cmd_vel 路线，功能已并入 go2_rl_runner.py
__pycache__/              Python 缓存文件，不属于源码
```

## 2. 运行前固定使用干净环境

你的默认 shell 里有 DPVO/conda 的 `LD_LIBRARY_PATH` 和 `PYTHONPATH`，运行 MuJoCo demo 时统一用下面这种前缀，避免环境串味：

```bash
cd /home/ros/unitree_dev

env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python <要运行的 python 文件>
```

## 3. 第一阶段：PD 站立

运行：

```bash
cd /home/ros/unitree_dev

env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/go2_physics_stand.py
```

你要看懂的核心公式：

```text
tau = kp * (q_target - q_current) + kd * (0 - dq_current)
```

这一阶段只证明一件事：

```text
MuJoCo 物理世界里的 Go2 可以被 12 个关节目标角 + PD 力矩控制器稳定站起来。
```

## 4. 第二阶段：手写步态

先做无窗口稳定性自检：

```bash
cd /home/ros/unitree_dev

env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/go2_walk_runner.py --headless-test --test-vx 0.15
```

再测试更大的低速命令：

```bash
cd /home/ros/unitree_dev

env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/go2_walk_runner.py --headless-test --test-vx 0.25
```

当前验证结果：

```text
vx=0.15: 8 秒不倒，dx=+0.103，机身高度稳定，pitch/roll 很小。
vx=0.25: 8 秒不倒，dx=+0.119，位移更明显，但仍然属于低速实验。
vx=-0.15: 8 秒不倒，dx=-0.261，但机身高度更低、pitch 更大，后退暂时少用。
yaw=0.15: 8 秒不倒，可以用于很小的转向稳定性观察。
```

运行：

```bash
cd /home/ros/unitree_dev

env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/go2_walk_runner.py
```

键盘：

```text
↑/↓ 前进后退
←/→ 左右横移
Q/E 原地转向
空格/R 急停
```

操作节奏：

```text
1. 先等机器狗完全站稳。
2. 只按一次 ↑，观察 5 秒。
3. 如果高度、pitch、roll 稳定，再按第二次 ↑。
4. 不要连续猛按；当前每次按键只增加 0.10。
5. 如果姿态明显倾斜，先按空格急停。
```

这一阶段要理解：

```text
cmd_vel -> 步态相位 -> 12 个目标关节角 -> PD 力矩 -> MuJoCo 物理运动
```

它不是强化学习，但它和 policy 部署有相同的后半段：

```text
目标关节角 -> PD -> 物理仿真
```

当前这个手写步态是“低速稳定实验台”，不是最终运动控制器。它的目标是先让你看懂底层闭环，并且让键盘输入不要把姿态立刻打崩。

## 5. 第三阶段：Isaac Lab policy 部署

先跑无窗口自检：

```bash
cd /home/ros/unitree_dev

env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/go2_rl_runner.py --headless-test
```

当前验证结果：

```text
go2_flat_2000.pt + action_scale=0.08:
  8 秒不倒，但 max|action| 很大且频繁 clip，说明策略在当前 MuJoCo XML 中偏激进。

go2_flat_199.pt + action_scale=0.12:
  8 秒稳定，动作很小，更适合做 policy 链路验证，但不代表已经学会行走。
```

打开窗口运行：

```bash
cd /home/ros/unitree_dev

env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/go2_rl_runner.py
```

如果要让 ROS2 `/cmd_vel` 通过文件桥接喂给 MuJoCo runner，开两个终端：

终端 1：

```bash
cd /home/ros/unitree_dev
source /opt/ros/humble/setup.bash
python3 projects/go2_control_demos/tools/cmd_vel_bridge.py
```

终端 2：

```bash
cd /home/ros/unitree_dev

env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/go2_rl_runner.py --cmd-source file
```

终端 3 发送一个很小的测试速度：

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

更保守的稳定链路验证：

```bash
cd /home/ros/unitree_dev

env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/go2_rl_runner.py \
  --policy projects/go2_control_demos/policies/go2_flat_199.pt \
  --action-scale 0.12
```

逐步加大 2000 策略动作幅度：

```bash
cd /home/ros/unitree_dev

env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/go2_rl_runner.py \
  --action-scale 0.12
```

如果不倒，再尝试：

```bash
--action-scale 0.15
```

不要一开始就使用训练值：

```bash
--action-scale 0.25
```

因为当前测试已经证明它很容易导致站起后倒下。

## 5.5 第三阶段补充：TheoBounac Go2 52 维 policy 的 MuJoCo Sim2Sim

新 runner：

```text
/home/ros/unitree_dev/projects/go2_control_demos/go2_mujoco_rl_policy_runner.py
```

它和旧 `go2_rl_runner.py` 的区别：

```text
go2_rl_runner.py:
  使用本地 IsaacLab flat policy，观测维度是 48。

go2_mujoco_rl_policy_runner.py:
  使用 TheoBounac/Deploy_SimToReal_RL_Go2 的 policy_rough.pt，
  观测维度是 52，包含 foot_force、base_lin_vel、base_ang_vel、
  projected_gravity、cmd、q、dq、last_action。
```

先跑无窗口零速度自检：

```bash
cd /home/ros/unitree_dev

env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/go2_mujoco_rl_policy_runner.py \
  --headless-test --headless-duration 30
```

当前验证结果：

```text
30 秒零速度自检通过。
状态机完整经过 PD_STAND_UP -> STABILIZE -> RL_CONTROL。
policy 接管后高度约 0.322m，pitch/roll 很小。
max|action| 约 1.12，没有 action clip。
已修正 base_ang_vel 坐标系：MuJoCo freejoint qvel[3:6] 已经是机身局部角速度，不再重复旋转。
```

再做很小的速度命令自检：

```bash
cd /home/ros/unitree_dev

env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/go2_mujoco_rl_policy_runner.py \
  --headless-test --headless-duration 12 --cmd-vx 0.05
```

当前验证结果：

```text
vx=0.05 时 12 秒没有触发摔倒保护。
这只说明小速度命令通道没有立刻破坏稳定性，不代表已经完成行走验证。
```

打开窗口运行：

```bash
cd /home/ros/unitree_dev

env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/go2_mujoco_rl_policy_runner.py
```

默认情况下键盘速度命令是锁住的，只验证零速度站稳。如果要手动给非常小的速度命令：

```bash
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/go2_mujoco_rl_policy_runner.py \
  --enable-keyboard-cmd
```

按键：

```text
↑/↓ 前后
←/→ 左右
Q/E 转向
空格/R 清零速度命令
```

强烈建议：

```text
一次只按一下。
先从 vx=0.05 级别观察。
不要连续猛按。
如果进入 DAMPING_STOP，说明姿态异常，先不要继续加速。
```

关节映射验证：

```bash
cd /home/ros/unitree_dev

env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/go2_mujoco_rl_policy_runner.py \
  --verify-joints --verify-joint FR_thigh_joint
```

可验证的 joint 名称：

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

这个验证非常重要：

```text
先确认单独动某个 joint 时，MuJoCo 里确实是对应关节在动。
12 个关节都确认后，再正式验证 RL 行走。
```

当前 runner 的核心安全机制：

```text
PD_STAND_UP:
  先用传统 PD 从 home 姿态平滑站到 policy 默认角。

STABILIZE:
  保持默认角等待机身稳定。

RL_CONTROL:
  每 0.02 秒调用一次 policy。
  MuJoCo 每 0.002 秒执行一次底层 PD。
  也就是每次 policy 推理后，PD 内环执行约 10 次。

DAMPING_STOP:
  如果高度过低或 roll/pitch 超过 45 度，不把 ctrl 直接置零，
  而是先锁定触发瞬间的当前关节角，使用很弱的 kp 短暂保持，
  再让 kp 在约 2 秒内衰减到 0，同时保留 -kd * dq 阻尼。
```

当前 runner 的关键可调参数：

```bash
--foot-force-scale 100
--foot-force-clip 5
--foot-force-binary
--foot-contact-threshold 20
--damping-kd 2.0
--damping-hold-kp-initial 8.0
--damping-hold-decay-time 2.0
```

足端力说明：

```text
TheoBounac 实机脚本使用 foot_force / 100。
MuJoCo 的 mj_contactForce 可能出现接触尖峰，所以这里默认 /100 后再 clip 到 0~5。
如果怀疑足端力幅值导致 policy 抖动，可以加 --foot-force-binary 改成 0/1 接触指示器。
```

## 6. 为什么 Isaac 训练数据到了 MuJoCo 会倒

原因不是单一 bug，而是 sim-to-sim 差异：

```text
Isaac Lab:
  Go2 USD 模型
  PhysX 接触求解
  DCMotor stiffness=25.0, damping=0.5
  dt=0.005, decimation=4
  默认站姿来自 Isaac asset

MuJoCo:
  unitree_mujoco Go2 XML
  MuJoCo 接触求解
  torque motor + 手写 PD
  dt=0.002
  当前更稳定的站姿来自 stand_go2.py
```

所以现在的正确目标不是“硬把 Isaac policy 在 MuJoCo 中跑得像 Isaac 一样”，而是按顺序掌握：

```text
1. MuJoCo PD 控制能站稳
2. 手写步态能解释运动控制链路
3. Isaac policy 能在 MuJoCo 中安全接管且不倒
4. 逐步调 action_scale、default_pose、kp/kd、policy checkpoint
5. 回到 Isaac Lab 验证 policy 是否真的学会跟踪速度
6. 再考虑 MuJoCo 中更精确的 sim-to-sim 对齐
```

## 7. 当前最推荐学习顺序

```text
第 1 天：
  go2_physics_stand.py
  目标：完全看懂 PD、关节顺序、qpos/actuator 映射

第 2 天：
  go2_walk_runner.py
  目标：看懂手写步态如何生成 12 个目标关节角

第 3 天：
  go2_rl_runner.py --headless-test
  目标：看懂 48 维 observation、last_action、action_scale、decimation

第 4 天：
  回 Isaac Lab play/train
  目标：确认 policy 在训练环境里是否真的能走，而不是只在 MuJoCo 里猜

第 5 天：
  把 `/cmd_vel` 接入 go2_rl_runner.py 或 ROS2 bridge
  目标：为后续 Nav2、SLAM、路径跟踪打基础
```

## 8. 实机安全边界

当前目录里的代码只用于仿真学习。

不要把 `go2_rl_runner.py`、`go2_walk_runner.py` 里的底层 PD/action 直接发送到真实 Go2。

真实 Go2 上机前必须满足：

```text
1. 先用宇树高层 SportClient 或官方安全接口。
2. 速度限幅从极小值开始。
3. 地面空旷，机器狗悬空或有保护条件。
4. 手机 App / 遥控器 / 急停可随时接管。
5. 明确区分仿真 domain 和实机网卡，避免误发 lowcmd。
```
