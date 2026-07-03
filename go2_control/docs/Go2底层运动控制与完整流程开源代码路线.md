# Go2 底层运动控制与完整流程开源代码路线

> 当前状态更新：这条“底层运动控制 / policy / LowCmd”路线先暂缓。
>
> 你现在更适合切换到“宇树官方高层控制 + ROS2 SLAM/视觉导航”路线：
>
> ```text
> /home/ros/unitree_dev/docs/Go2高层控制与SLAM视觉导航路线.md
> ```
>
> 原因：你的核心项目目标是 SLAM、视觉导航和最终上实机，而不是重新实现四足步态控制。Go2 已经内置稳定运动控制器，我们应该让上层导航只输出 `/cmd_vel`，再由宇树高层 Sport 控制器完成稳定行走。

> 新增补充：如果你想继续学习 Go2 的底层强化学习控制，不再走手写步态，而是走
> `unitree_rl_gym / IsaacLab policy / Sim2Sim / Sim2Real` 路线，请优先看：
>
> ```text
> /home/ros/unitree_dev/docs/Go2_RL底层强化学习控制方案评估与执行计划.md
> ```
>
> 这份文档已经单独评估了宇树官方 `unitree_rl_gym`、TheoBounac 的
> `Deploy_SimToReal_RL_Go2`、IsaacLab 到 MuJoCo 迁移风险，以及下一步执行顺序。

本文回答当前最关键的 3 个问题：

```text
1. 现在的 MuJoCo demo 到底算不算底层运动控制？
2. 如果想在仿真里学习运动、定位、建图、导航完整流程，应该参考哪套开源代码？
3. 能不能借鉴宇树/开源控制器，实现“输入期望状态 -> 输出电机控制指令”？
```

结论先放在前面：

```text
当前 mujoco_go2_control_demos.py 是运动学可视化教学 demo，不是底层运动控制。
真正学习 Go2 底层运动控制，建议主线切到 unitree_rl_gym。
完整 ROS2 SLAM/Nav2 应用流程，建议主线继续用 go2_ros2_sdk。
最终工程应该把二者通过 cmd_vel / odom / scan / tf 等 ROS2 标准接口连接起来。
```

## 1. 先承认当前 MuJoCo demo 的边界

当前文件：

```text
/home/ros/unitree_dev/projects/go2_control_demos/mujoco_go2_control_demos.py
```

它实现的是：

```text
高层速度指令 vx / yaw_rate
  -> 限幅和平滑
  -> 运动学更新 x / y / yaw
  -> sin/cos 生成腿部动画
  -> 直接写 MuJoCo qpos
  -> viewer 显示
```

它没有做：

```text
LowCmd
关节 PD 控制
力矩控制
足底接触
mujoco.mj_step 物理推进
策略 policy 推理
真实机器人可迁移控制
```

所以它不满足“学习底层运动控制”的最终目标。

它的价值是：

```text
1. 帮你理解 vx / yaw_rate / 路径点 / 圆弧运动 / 状态机
2. 作为后续 ROS2 cmd_vel 的上层逻辑原型
3. 避免初学阶段一上来被不稳定步态、摔倒、抽搐卡死
```

但它不能证明：

```text
Go2 真能稳定走
电机命令合理
kp/kd 合理
步态可实机迁移
```

真正底层运动控制应该是：

```text
期望速度/状态
  -> 控制器或策略 policy
  -> 12 个关节目标角 / 速度 / 力矩
  -> LowCmd 或 MuJoCo ctrl
  -> 物理引擎/真实电机
  -> LowState / IMU / 关节反馈
  -> 下一轮控制
```

## 2. 我建议选的开源代码主线

现在不要试图找“一份代码同时完美解决运动、定位、建图、导航、实机部署、目标检测”。四足机器人的完整系统通常就是多套仓库组合。

当前最适合你的组合是：

| 目标 | 推荐主线 | 本地路径 |
|---|---|---|
| 底层运动控制、策略、Sim2Sim、Sim2Real | `unitree_rl_gym` | `/home/ros/unitree_dev/projects/open_source_unitree_rl_gym` |
| ROS2 实机桥接、相机、雷达、SLAM、Nav2 | `go2_ros2_sdk` | `/home/ros/unitree_dev/projects/go2_ros2_sdk_ws` |
| 底层 DDS / LowCmd / MuJoCo 官方仿真 | `unitree_mujoco` | `/home/ros/unitree_dev/src/unitree_mujoco` |
| 高真实感场景、传感器、训练策略 | Isaac Lab | `/home/ros/isaac_go2/IsaacLab` |

对应关系：

```text
unitree_rl_gym 解决“怎么让腿稳定走”
go2_ros2_sdk 解决“怎么在 ROS2 里感知、建图、导航”
unitree_mujoco 解决“怎么接 LowCmd / LowState 做 sim-to-real 验证”
Isaac Lab 解决“怎么做大规模训练、高真实感传感器和场景”
```

## 3. 新下载的 unitree_rl_gym 是什么

我已经把宇树官方仓库克隆到：

```text
/home/ros/unitree_dev/projects/open_source_unitree_rl_gym
```

它的 README 写得很清楚，强化学习运动控制基本流程是：

```text
Train -> Play -> Sim2Sim -> Sim2Real
```

其中：

```text
Train：在 Isaac Gym 中训练策略
Play：在 Isaac Gym 中查看策略效果
Sim2Sim：把策略部署到 MuJoCo，验证策略不只适配 Isaac Gym
Sim2Real：把策略部署到真实机器人
```

它支持：

```text
Go2
G1
H1
H1_2
```

Go2 训练配置在：

```text
/home/ros/unitree_dev/projects/open_source_unitree_rl_gym/legged_gym/envs/go2/go2_config.py
```

MuJoCo 策略部署代码在：

```text
/home/ros/unitree_dev/projects/open_source_unitree_rl_gym/deploy/deploy_mujoco/deploy_mujoco.py
```

真实机器人部署代码在：

```text
/home/ros/unitree_dev/projects/open_source_unitree_rl_gym/deploy/deploy_real/deploy_real.py
```

## 4. unitree_rl_gym 里真正值得你学的底层控制链路

### 4.1 Go2 训练配置

看这个文件：

```text
legged_gym/envs/go2/go2_config.py
```

里面定义了 Go2 的默认站立角：

```python
default_joint_angles = {
    'FL_hip_joint': 0.1,
    'RL_hip_joint': 0.1,
    'FR_hip_joint': -0.1,
    'RR_hip_joint': -0.1,
    'FL_thigh_joint': 0.8,
    'RL_thigh_joint': 1.0,
    'FR_thigh_joint': 0.8,
    'RR_thigh_joint': 1.0,
    'FL_calf_joint': -1.5,
    'RL_calf_joint': -1.5,
    'FR_calf_joint': -1.5,
    'RR_calf_joint': -1.5,
}
```

还定义了控制参数：

```python
control_type = 'P'
stiffness = {'joint': 20.}
damping = {'joint': 0.5}
action_scale = 0.25
decimation = 4
```

这里已经出现了你真正要学的东西：

```text
action 不是直接力矩
action 会变成目标关节角
目标关节角 = 默认关节角 + action * action_scale
然后由 PD 控制器追踪目标角
```

### 4.2 MuJoCo 策略部署代码

看这个文件：

```text
deploy/deploy_mujoco/deploy_mujoco.py
```

核心函数：

```python
def pd_control(target_q, q, kp, target_dq, dq, kd):
    return (target_q - q) * kp + (target_dq - dq) * kd
```

这就是底层控制最核心的公式：

```text
tau = kp * (q_target - q_current)
    + kd * (dq_target - dq_current)
```

主循环里做的事情是：

```text
1. 从 MuJoCo 读取 qpos / qvel / quat / omega
2. 拼 observation
3. policy(obs) 输出 action
4. target_dof_pos = action * action_scale + default_angles
5. PD 计算 tau
6. d.ctrl[:] = tau
7. mujoco.mj_step()
```

这就是你想要的：

```text
输入期望速度/状态
  -> 策略/控制器
  -> 输出关节目标
  -> 转成电机控制
```

### 4.3 真实机器人部署代码

看这个文件：

```text
deploy/deploy_real/deploy_real.py
```

它和 MuJoCo 版很像，只是输出从 `d.ctrl` 变成了 `LowCmd`：

```text
1. 订阅 LowState
2. 读取关节角、关节速度、IMU、遥控器
3. 拼 observation
4. policy(obs) 输出 action
5. target_dof_pos = default_angles + action * action_scale
6. 填 low_cmd.motor_cmd[motor_idx].q / kp / kd / tau
7. 计算 CRC
8. 发布 LowCmd
```

对应源码中的关键逻辑：

```python
self.action = self.policy(obs_tensor).detach().numpy().squeeze()
target_dof_pos = self.config.default_angles + self.action * self.config.action_scale

self.low_cmd.motor_cmd[motor_idx].q = target_dof_pos[i]
self.low_cmd.motor_cmd[motor_idx].qd = 0
self.low_cmd.motor_cmd[motor_idx].kp = self.config.kps[i]
self.low_cmd.motor_cmd[motor_idx].kd = self.config.kds[i]
self.low_cmd.motor_cmd[motor_idx].tau = 0
```

这就是“策略控制器 -> 真实电机命令”的完整结构。

## 5. 目前 unitree_rl_gym 对 Go2 的限制

我已经检查本地仓库，当前情况是：

```text
Go2 训练配置存在：legged_gym/envs/go2/go2_config.py
Go2 URDF 存在：resources/robots/go2/urdf/go2.urdf
但 deploy_mujoco/configs 当前只自带 g1/h1/h1_2
deploy/pre_train 当前只自带 g1/h1/h1_2 的 motion.pt
deploy_real/configs 当前也只自带 g1/h1/h1_2
```

也就是说：

```text
Go2 可以作为训练任务学习。
但 Go2 的 MuJoCo 部署配置和预训练策略可能需要我们自己补。
```

这并不影响学习，因为你真正要学的控制框架都在：

```text
go2_config.py
deploy_mujoco.py
deploy_real.py
```

下一步可以做的是：

```text
参考 g1.yaml
为 Go2 写 deploy_mujoco/configs/go2.yaml
把 Go2 的 policy_path / xml_path / default_angles / kps / kds / action_scale / obs 维度补齐
```

但前提是：

```text
你需要一个 Go2 的 policy.pt
或者先训练一个 Go2 policy
```

## 6. 完整流程应该怎么组合

你想要的是：

```text
运动
定位
导航
建图
```

现实中它应该拆成两个闭环：

### 6.1 运动控制闭环

```text
cmd_vel / 期望速度
  -> policy 或 MPC/步态控制器
  -> 关节目标角 / 力矩
  -> MuJoCo / 真实电机
  -> 关节状态 + IMU
  -> 下一轮 policy
```

这一段用：

```text
unitree_rl_gym
unitree_mujoco
```

### 6.2 ROS2 导航闭环

```text
相机 / 雷达 / IMU / 里程计
  -> SLAM / 定位
  -> map / odom / tf
  -> Nav2
  -> cmd_vel
  -> 运动控制后端
```

这一段用：

```text
go2_ros2_sdk
slam_toolbox
Nav2
ORB-SLAM3 / DPVO / VGGT
```

最终你要搭的系统应该长这样：

```text
SLAM / Nav2 / 路径点控制器
  -> cmd_vel
  -> locomotion backend
       A. MuJoCo kinematic demo，用于教学
       B. unitree_rl_gym policy + MuJoCo，用于底层控制学习
       C. go2_ros2_sdk WebRTC Move，用于实机高层运动
       D. LowCmd policy deploy，用于实机底层运动
```

## 7. 建议你现在的最快主线

### 第 1 步：把当前状态分清楚

保留当前 kinematic demo：

```text
projects/go2_control_demos/mujoco_go2_control_demos.py
```

用途：

```text
学习 cmd_vel、路径点、状态机
```

不要再把它当作：

```text
底层运动控制器
```

### 第 2 步：读官方底层控制主线

按顺序读：

```text
1. unitree_rl_gym/legged_gym/envs/go2/go2_config.py
2. unitree_rl_gym/deploy/deploy_mujoco/deploy_mujoco.py
3. unitree_rl_gym/deploy/deploy_real/deploy_real.py
4. unitree_mujoco/example/python/stand_go2.py
5. unitree_mujoco/simulate_python/unitree_sdk2py_bridge.py
```

你要从这些文件里学会：

```text
default_angles 是什么
action_scale 是什么
observation 怎么拼
policy 输出 action 后怎么转成 target_dof_pos
PD 控制怎么算 tau
LowCmd 怎么填
LowState 怎么读
控制频率和 decimation 是什么
```

### 第 3 步：自己写一个“Go2 policy MuJoCo runner”

目标文件建议：

```text
projects/go2_control_demos/go2_policy_mujoco_runner.py
```

第一版功能：

```text
加载 Go2 MuJoCo 模型
读取 qpos/qvel/quat/omega
拼 observation
加载 policy.pt
输出 target_dof_pos
PD 算 tau
mujoco.mj_step()
```

这才是你要的真正底层仿真控制。

### 第 4 步：再接 ROS2

把 policy runner 改成订阅：

```text
/cmd_vel
```

让它接收：

```text
linear.x
linear.y
angular.z
```

然后把 `cmd_vel` 放进 observation 的命令部分。

这样你就能做到：

```text
路径点控制器 / Nav2
  -> /cmd_vel
  -> Go2 policy
  -> MuJoCo 电机控制
```

### 第 5 步：再接 SLAM / Nav2

这一步用：

```text
go2_ros2_sdk
ros2_orbslam3
Nav2
```

目标是：

```text
仿真/实机发布 odom / tf / scan / image
SLAM 输出 map / pose
Nav2 输出 cmd_vel
policy runner 执行 cmd_vel
```

## 8. 你第三个问题的直接回答

你问：

```text
既然 MuJoCo 没法调用宇树自己的高层控制解算，
是不是可以学习/借鉴它怎么写，模仿实现：
输入期望状态 -> 输出电机控制指令？
```

答案是：

```text
可以，而且这正是正确路线。
```

但不要试图复刻宇树闭源高层控制器本身。更现实的做法是：

```text
学习开源策略控制器的结构
用 policy 或简化 MPC/步态控制器替代闭源高层控制器
输出 LowCmd 或 MuJoCo ctrl
```

推荐从 `unitree_rl_gym` 学这个结构，因为它已经把关键环节写出来了：

```text
observation
policy
action_scale
default_angles
PD control
LowCmd
Sim2Sim
Sim2Real
```

这比手写正弦步态更接近真实工程，也更适合你后续扩展 SLAM、导航、目标检测和具身智能模块。

## 9. 本阶段验收标准

你下一阶段不应该只看“狗在屏幕上动了没有”，而应该用这些标准验收：

```text
1. 能解释 action 为什么不是电机力矩
2. 能解释 target_dof_pos = default_angles + action * action_scale
3. 能解释 PD 控制公式
4. 能解释 observation 包含哪些量
5. 能解释 cmd_vel 怎么进入 policy
6. 能解释 MuJoCo 中 d.ctrl 和真实 LowCmd 的区别
7. 能跑通一个 policy 在 MuJoCo 里的闭环控制
8. 能把路径点控制器输出的 cmd_vel 接到 policy runner
```

达到这些之后，才算真正开始进入：

```text
四足机器人底层运动控制
```

而不是只是在做可视化演示。
