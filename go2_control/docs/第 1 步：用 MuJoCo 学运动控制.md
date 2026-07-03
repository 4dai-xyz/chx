# 第 1 步：用 MuJoCo 学运动控制

本文是 `仿真运动控制与具身导航最快入门路线.md` 中“第 1 步：用 MuJoCo 学运动控制”的展开教程。

当前阶段目标不是马上做 SLAM、导航、Isaac 场景或真实 Go2 控制，而是先在 MuJoCo 中搞清楚：

```text
我发什么控制命令
仿真器怎么接收命令
关节/速度/状态怎么变化
机器狗为什么会站起、趴下、前进、转向、停止
后续怎么从运动控制扩展到路径点跟踪
```

## 1. 当前阶段总目标

你要先复现 6 个控制 demo：

```text
1. 站立/趴下
2. 直行
3. 原地转向
4. 直行一段距离后停止
5. 低速绕圈
6. 路径点跟踪：给 3 个目标点，让 Go2 按顺序走过去
```

完成这 6 个 demo 后，再完整更新本文档，把每个 demo 的：

```text
运行命令
代码文件
控制原理
关键参数
实验现象
常见问题
下一步改进
```

全部补齐。

## 1.1 现在路线重新收束

你现在已经从最早的“看见机器狗动起来”，进入到“学习底层运动控制”的阶段，所以后续不要再把所有 demo 混在一起看。

当前 `projects/go2_control_demos` 目录已经整理成 4 层：

| 层级 | 文件 | 你要学什么 |
|---|---|---|
| 运动学可视化 | `mujoco_go2_control_demos.py` | 理解直行、转向、定距停止、绕圈、路径点跟踪的几何逻辑 |
| 动力学 PD 站立 | `go2_physics_stand.py` | 理解关节角、关节速度、PD 力矩、qpos/actuator 映射 |
| 手写步态 | `go2_walk_runner.py` | 先做低速稳定实验，理解 `cmd_vel -> 步态相位 -> 目标关节角 -> PD -> MuJoCo` |
| Isaac policy 部署 | `go2_rl_runner.py` | 理解 `observation -> policy -> action -> 目标关节角 -> PD -> MuJoCo` |

也就是说：

```text
mujoco_go2_control_demos.py 只是早期可视化，不代表真实底层控制。
go2_physics_stand.py 才是底层控制第一课。
go2_walk_runner.py 是不用 RL 的可解释低速步态实验。
go2_rl_runner.py 是 Isaac Lab 训练策略部署到 MuJoCo 的主线。
```

我已经清理掉旧的重复入口：

```text
go2_policy_runner.py      旧 47 维 dummy policy 路线，和当前 Isaac 48 维策略不一致
go2_cmd_vel_runner.py     旧 dummy policy + cmd_vel 路线，功能已并入 go2_rl_runner.py
```

后续学习时，以 `README.md` 和 `go2_rl_runner.py` 为准。

## 2. 先记住当前 MuJoCo 控制链路

当前 Python 版 MuJoCo 仿真链路是：

```text
你的控制程序
  -> 发布 rt/lowcmd
  -> unitree_mujoco 的 bridge 接收 LowCmd
  -> bridge 把 q / dq / kp / kd / tau 转成 MuJoCo 电机 ctrl
  -> MuJoCo 执行 mj_step()
  -> 仿真器发布 rt/lowstate 和 rt/sportmodestate
  -> 你可以读取状态，再继续控制
```

最核心的控制公式在 bridge 里：

```text
ctrl = tau
     + kp * (q_target - q_current)
     + kd * (dq_target - dq_current)
```

也就是说，第一阶段你主要学习的是：

```text
低层关节 PD 控制
```

而不是一开始就直接用：

```text
SportClient.Move(vx, vy, vyaw)
```

原因是当前 Python MuJoCo bridge 主要订阅 `rt/lowcmd`，并没有完整模拟实机高层 sport service。

## 3. 最应该参考的代码

### 3.1 低层站立 demo，第一优先级

```text
/home/ros/unitree_dev/src/unitree_mujoco/example/python/stand_go2.py
```

这是你现在最应该先读、先跑、先改的文件。

它演示了：

```text
如何初始化 DDS
如何创建 rt/lowcmd 发布器
如何填写 LowCmd
如何设置 12 个关节的目标角度
如何设置 kp / kd
如何计算 CRC
如何持续发布控制命令
如何让 Go2 从趴下到站立，再从站立到趴下
```

里面最重要的是两个姿态数组：

```python
stand_up_joint_pos
stand_down_joint_pos
```

它们分别表示：

```text
stand_up_joint_pos    站立姿态的 12 个关节角
stand_down_joint_pos  趴下姿态的 12 个关节角
```

### 3.2 MuJoCo Python 仿真入口

```text
/home/ros/unitree_dev/src/unitree_mujoco/simulate_python/unitree_mujoco.py
```

它不是控制程序，而是仿真器入口。

它负责：

```text
加载 Go2 MJCF 场景
启动 MuJoCo viewer
初始化 DDS
创建 UnitreeSdk2Bridge
循环 mujoco.mj_step()
刷新窗口画面
```

你要理解它的作用，但第一个控制 demo 不需要改它。

### 3.3 MuJoCo Python DDS 桥接

```text
/home/ros/unitree_dev/src/unitree_mujoco/simulate_python/unitree_sdk2py_bridge.py
```

这个文件告诉你当前 Python MuJoCo 仿真支持哪些 DDS topic。

重点：

```text
订阅：rt/lowcmd
发布：rt/lowstate
发布：rt/sportmodestate
发布：rt/wirelesscontroller
```

最关键的是 `LowCmdHandler()`：

```text
它接收 rt/lowcmd
读取每个 motor_cmd 的 q / dq / kp / kd / tau
写入 mj_data.ctrl
驱动 MuJoCo 中的关节电机
```

### 3.4 MuJoCo Python 配置文件

```text
/home/ros/unitree_dev/src/unitree_mujoco/simulate_python/config.py
```

当前仿真配置：

```python
ROBOT = "go2"
DOMAIN_ID = 1
INTERFACE = "lo"
```

所以你写 MuJoCo 控制程序时，应该使用：

```python
ChannelFactoryInitialize(1, "lo")
```

含义：

```text
Domain 1 表示仿真 DDS domain
lo 表示本机回环网卡
这样不会误发到真实 Go2
```

### 3.5 高层 SportClient 示例，暂时只作为参考

```text
/home/ros/unitree_dev/src/unitree_sdk2_python/example/go2/high_level/go2_sport_client.py
/home/ros/unitree_dev/src/unitree_sdk2_python/unitree_sdk2py/go2/sport/sport_client.py
```

这两个文件适合后面学实机高层控制时参考。

例如：

```python
sport_client.StandUp()
sport_client.StandDown()
sport_client.Move(0.3, 0, 0)
sport_client.Move(0, 0, 0.5)
sport_client.StopMove()
```

但当前 Python MuJoCo 仿真第一阶段不建议直接把它当主线，因为当前 bridge 主要接的是 `rt/lowcmd`。

## 4. 第一个 demo：站立/趴下

### 4.1 启动 MuJoCo

打开第一个终端：

```bash
cd /home/ros/unitree_dev
bash scripts/run_mujoco_python.sh
```

看到 MuJoCo 窗口和 Go2 后，再开第二个终端。

### 4.2 运行站立 demo

第二个终端运行：

```bash
cd /home/ros/unitree_dev
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python src/unitree_mujoco/example/python/stand_go2.py
```

终端出现：

```text
Press enter to start
```

按回车。

### 4.3 预期现象

你应该看到：

```text
Go2 从趴下姿态逐渐站起来
保持一小段时间
然后再逐渐趴下
```

这个 demo 完成后，说明：

```text
MuJoCo 仿真正常
DDS 通信正常
rt/lowcmd 能控制仿真电机
低层关节 PD 控制链路通了
```

## 5. 当前真正要掌握的底层控制路线

### 5.1 第一步：动力学 PD 站立

运行：

```bash
cd /home/ros/unitree_dev

env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/go2_physics_stand.py
```

要看懂：

```text
1. qpos[7:] 是 12 个关节角。
2. qvel[6:] 是 12 个关节速度。
3. d.ctrl[:] 是 12 个电机力矩。
4. actuator 顺序和 qpos 顺序不一样，所以必须做映射。
5. PD 控制公式是 tau = kp*(q_target-q) + kd*(0-dq)。
```

验收标准：

```text
Go2 能从趴下/低姿态平滑站起来，并保持稳定，不抽搐、不倒地。
```

### 5.2 第二步：手写步态

现在这一步先不要追求“走得快、转得漂亮”。当前目标只有一个：

```text
键盘输入以后，姿态不要立刻崩掉。
```

先做无窗口自检：

```bash
cd /home/ros/unitree_dev

env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/go2_walk_runner.py --headless-test --test-vx 0.15
```

再测试稍大一点的低速命令：

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

键盘操作节奏：

```text
1. 等 Go2 完全站稳。
2. 只按一次 ↑，观察 5 秒。
3. 如果高度、pitch、roll 都稳定，再按第二次 ↑。
4. 不要连续猛按；当前每次按键只增加 0.10。
5. 如果姿态明显倾斜，先按空格急停。
```

要看懂：

```text
cmd_vel
  -> 步态相位 phase
  -> 每条腿的 hip/thigh/calf 目标角
  -> PD 力矩
  -> MuJoCo 接触和动力学
```

这一步的意义是：你不用神经网络，也能亲手写出一个“控制器雏形”。虽然它不一定非常稳，但它让你知道 RL policy 最后输出的东西本质上也是目标关节角。

当前 `go2_walk_runner.py` 已经加入：

```text
1. 命令限幅：vx 最大 0.25，yaw 最大 0.25。
2. 命令缓变：键盘期望命令不会瞬间打到关节。
3. 低幅步态：大腿最大摆幅默认 0.10rad。
4. 关节目标限幅：避免目标角贴近机械极限。
5. 力矩限幅：避免 PD 力矩超过 MuJoCo actuator ctrlrange。
6. 跌倒保护：机身太低或姿态角过大自动停止。
7. headless-test：不用打开窗口也能快速验证一组参数会不会倒。
8. 前进方向校准：`vx > 0` 已经校准为 Go2 头部方向，也就是世界 `+x` 方向。
```

### 5.3 第三步：Isaac Lab policy 部署到 MuJoCo

当前主入口：

```text
/home/ros/unitree_dev/projects/go2_control_demos/go2_rl_runner.py
```

先做无窗口自检：

```bash
cd /home/ros/unitree_dev

env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/go2_rl_runner.py --headless-test
```

当前测试结论：

```text
go2_flat_2000.pt + action_scale=0.08:
  可以 8 秒不倒，但 max|action| 很大且频繁 clip，说明该策略在当前 MuJoCo XML 中偏激进。

go2_flat_199.pt + action_scale=0.12:
  可以 8 秒稳定，动作小，适合作为 policy 链路验证。
```

打开窗口运行：

```bash
cd /home/ros/unitree_dev

env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/go2_rl_runner.py
```

更保守的策略链路验证：

```bash
cd /home/ros/unitree_dev

env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/go2_rl_runner.py \
  --policy projects/go2_control_demos/policies/go2_flat_199.pt \
  --action-scale 0.12
```

如果你要试更大的动作幅度，不要直接跳到训练值 `0.25`，而是：

```bash
--action-scale 0.12
--action-scale 0.15
--action-scale 0.18
```

逐步加大。只要出现机身快速变低、倾斜、倒地、腿部抽搐，就说明当前策略/模型/参数还没对齐。

### 5.4 为什么 Isaac 训练出来的策略到 MuJoCo 会倒

这不是一个单独 bug，而是典型 sim-to-sim gap：

```text
Isaac Lab:
  Go2 USD 模型
  PhysX 接触
  DCMotor stiffness=25.0, damping=0.5
  sim dt=0.005
  decimation=4
  action scale=0.25

MuJoCo:
  unitree_mujoco Go2 XML
  MuJoCo 接触
  torque motor + 手写 PD
  sim dt=0.002
  当前更稳的站姿来自 stand_go2.py
```

所以现在正确的学习目标不是“马上让 Isaac policy 在 MuJoCo 中完美行走”，而是：

```text
1. 确认 observation 维度和顺序正确。
2. 确认 action 能变成目标关节角。
3. 确认 PD 和关节映射正确。
4. 用 action_scale 降档让系统先不倒。
5. 回 Isaac Lab play 里确认 policy 本身是否真的学会走。
6. 再慢慢做 MuJoCo 和 Isaac 的模型参数对齐。
```

### 5.5 下一阶段验收标准

这一阶段你真正要掌握的是：

```text
1. 我能解释 48 维 observation 每一段是什么。
2. 我能解释 policy 输出的 12 维 action 如何变成目标关节角。
3. 我能解释为什么 action_scale 太大会倒。
4. 我能解释为什么 qpos 顺序和 actuator 顺序必须转换。
5. 我能用 --headless-test 判断当前策略是否稳定。
6. 我知道当前 policy 在 MuJoCo 中“不倒”和“会走”不是同一件事。
```

## 5. 站立 demo 的核心原理

`stand_go2.py` 做了一个简单插值：

```text
趴下关节角 -> 站立关节角
站立关节角 -> 趴下关节角
```

核心代码逻辑是：

```python
cmd.motor_cmd[i].q = phase * stand_up_joint_pos[i] + (1 - phase) * stand_down_joint_pos[i]
cmd.motor_cmd[i].kp = phase * 50.0 + (1 - phase) * 20.0
cmd.motor_cmd[i].dq = 0.0
cmd.motor_cmd[i].kd = 3.5
cmd.motor_cmd[i].tau = 0.0
```

参数含义：

| 参数 | 含义 |
|---|---|
| `q` | 目标关节角 |
| `dq` | 目标关节速度 |
| `kp` | 位置刚度，越大越用力追目标角 |
| `kd` | 速度阻尼，越大越抑制抖动 |
| `tau` | 前馈力矩 |
| `crc` | Unitree DDS 消息校验 |

第一阶段你必须理解：

```text
Go2 并不是收到一个“站起来”魔法命令。
它是收到 12 个关节目标角，然后通过 PD 控制慢慢插值到站立姿态。
```

## 6. 已实现的 2 到 6 号 demo

现在已经新增统一入口：

```text
/home/ros/unitree_dev/projects/go2_control_demos/mujoco_go2_control_demos.py
```

另外保留了一个路径点跟踪独立入口：

```text
/home/ros/unitree_dev/projects/go2_control_demos/waypoint_follower_demo.py
```

旧文件：

```text
/home/ros/unitree_dev/projects/go2_control_demos/nav_closed_loop_demo.py.py
```

现在只是兼容入口，会调用新的路径点跟踪 demo，不再使用之前不稳定的手写黑盒步态。

### 6.1 先说清楚：这版 demo 的定位

这次实现的是：

```text
MuJoCo 运动学可视化教学 demo
```

它的核心目标是先让你稳定看到：

```text
直行
原地转向
直行定距停止
低速绕圈
3 个路径点跟踪
```

它采用：

```text
高层速度指令 vx / yaw_rate
  -> 速度限幅和平滑
  -> 更新平面位姿 x / y / yaw
  -> 生成腿部步态动画
  -> 写入 MuJoCo qpos 并刷新画面
```

注意它不是：

```text
可直接上实机的低层关节控制器
```

原因是稳定四足步态需要完整的动力学控制器、接触状态估计、姿态反馈、足端轨迹和安全保护。之前你看到的踉跄、倒地、抽搐，就是手写低层正弦步态直接驱动物理模型时很容易出现的问题。为了学习路线更稳，这一步先把“运动命令和导航状态机”学明白，再进入真正的低层步态或强化学习策略。

### 6.2 运行方式

这批 demo 不需要先运行：

```bash
bash scripts/run_mujoco_python.sh
```

因为它自己会打开 MuJoCo viewer。

统一运行格式：

```bash
cd /home/ros/unitree_dev
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/mujoco_go2_control_demos.py --demo 任务名
```

可选任务名：

```text
straight   直行
turn       原地转向
distance   直行一段距离后停止
circle     低速绕圈
waypoints  3 个路径点跟踪
all        按顺序运行以上全部 demo
```

如果只想看帮助：

```bash
cd /home/ros/unitree_dev
.venv-unitree/bin/python projects/go2_control_demos/mujoco_go2_control_demos.py --help
```

### 6.3 Demo 2：直行

运行：

```bash
cd /home/ros/unitree_dev
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/mujoco_go2_control_demos.py --demo straight
```

预期现象：

```text
Go2 先从趴下姿态站起来
然后以约 0.12 m/s 低速向前走 5 秒
最后平滑停止并趴下
```

你要重点读的代码：

```python
def demo_straight(self) -> None:
    self.run_for(
        duration=5.0,
        command_fn=lambda _elapsed: (0.12, 0.0),
    )
```

这里的 `(0.12, 0.0)` 含义是：

| 参数 | 含义 |
|---|---|
| `vx = 0.12` | 机器人自身坐标系向前 0.12 m/s |
| `yaw_rate = 0.0` | 不转向 |

### 6.4 Demo 3：原地转向

运行：

```bash
cd /home/ros/unitree_dev
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/mujoco_go2_control_demos.py --demo turn
```

预期现象：

```text
Go2 先站起来
然后在原地低速左转
最后平滑停止并趴下
```

核心指令：

```python
command_fn=lambda _elapsed: (0.0, 0.35)
```

含义：

| 参数 | 含义 |
|---|---|
| `vx = 0.0` | 不前进 |
| `yaw_rate = 0.35` | 左转，单位 rad/s |

这一节你要理解：

```text
原地转向不是改变 x/y，而是改变 yaw。
```

### 6.5 Demo 4：直行一段距离后停止

运行：

```bash
cd /home/ros/unitree_dev
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/mujoco_go2_control_demos.py --demo distance
```

预期现象：

```text
Go2 先站起来
向前走约 1.0 m
接近目标距离时速度逐渐减小
到达后平滑停止并趴下
```

核心控制逻辑：

```text
起点 start_x / start_y
当前位置 pose.x / pose.y
已走距离 distance = hypot(pose.x - start_x, pose.y - start_y)
如果 distance >= 1.0 m，就停车
```

代码里对应：

```python
distance = math.hypot(self.pose.x - start_x, self.pose.y - start_y)
target_vx = clamp(0.35 * remaining, 0.04, 0.12)
```

这里已经开始出现导航里常见的思想：

```text
误差越大，速度越大
误差越小，速度越小
到阈值内，切换状态
```

### 6.6 Demo 5：低速绕圈

运行：

```bash
cd /home/ros/unitree_dev
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/mujoco_go2_control_demos.py --demo circle
```

预期现象：

```text
Go2 站起来
一边前进一边缓慢左转
整体轨迹接近圆弧/绕圈
最后平滑停止并趴下
```

核心指令：

```python
vx_cmd = 0.10
yaw_cmd = 0.25
radius = vx_cmd / yaw_cmd
```

这里的运动学关系是：

```text
圆弧半径 R = 线速度 vx / 角速度 yaw_rate
```

所以：

```text
vx 越大，圈越大
yaw_rate 越大，圈越小
```

### 6.7 Demo 6：3 个路径点跟踪

运行统一入口：

```bash
cd /home/ros/unitree_dev
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/mujoco_go2_control_demos.py --demo waypoints
```

也可以运行独立入口：

```bash
cd /home/ros/unitree_dev
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python projects/go2_control_demos/waypoint_follower_demo.py
```

默认路径点：

```python
waypoints = ((0.8, 0.0), (0.8, 0.5), (0.2, 0.5))
```

预期现象：

```text
Go2 先朝第 1 个目标点走
到达后短暂停稳
再转向并走向第 2 个目标点
再转向并走向第 3 个目标点
全部完成后平滑停止并趴下
```

核心状态机：

```text
读取当前 pose = x, y, yaw
读取当前目标点 target_x, target_y
计算 dx / dy / distance
计算 target_yaw = atan2(dy, dx)
计算 yaw_error = target_yaw - yaw
如果 distance < 0.08 m，切换下一个目标点
如果 yaw_error 较大，先原地转向
否则，一边前进一边小幅修正航向
```

代码里的关键判断：

```python
if distance < 0.08:
    waypoint_index += 1
elif abs(yaw_error) > 0.20:
    target_vx = 0.0
    target_yaw_rate = clamp(1.3 * yaw_error, -0.35, 0.35)
else:
    target_vx = clamp(0.45 * distance, 0.04, 0.12)
    target_yaw_rate = clamp(1.0 * yaw_error, -0.22, 0.22)
```

这就是最基础的“局部控制器”雏形。

## 7. 当前代码结构

现在 `projects/go2_control_demos/` 里建议重点看这几个文件：

```text
go2_control_demos/
├── mujoco_go2_control_demos.py      # 2 到 6 号 demo 的统一入口，重点阅读
```

`mujoco_go2_control_demos.py` 里的核心函数：

| 函数 | 作用 |
|---|---|
| `stand_up()` | 从趴下姿态平滑站起来 |
| `stand_down()` | 从站立姿态平滑趴下 |
| `step_velocity()` | 执行一步高层速度控制，更新 `x/y/yaw` |
| `make_walk_joint_pose()` | 根据 `vx/yaw_rate` 生成腿部步态动画 |
| `demo_straight()` | 直行 |
| `demo_turn()` | 原地转向 |
| `demo_distance_stop()` | 直行定距停止 |
| `demo_circle()` | 低速绕圈 |
| `demo_waypoints()` | 3 个路径点跟踪 |

## 8. 代码原理：先理解这 4 层

### 8.1 第 1 层：速度命令

目前每个 demo 最终都可以抽象成：

```text
vx: 机器人自身坐标系前进速度
yaw_rate: 机器人绕 z 轴旋转速度
```

这和 ROS2 里的 `/cmd_vel` 思想一致：

```text
geometry_msgs/Twist.linear.x  -> vx
geometry_msgs/Twist.angular.z -> yaw_rate
```

### 8.2 第 2 层：限幅和平滑

代码里有：

```python
class MotionLimits:
    max_vx = 0.16
    max_yaw_rate = 0.45
    max_acc_vx = 0.25
    max_acc_yaw_rate = 0.80
```

它的作用是：

```text
不要让速度突然从 0 跳到很大
不要让转向角速度突然跳变
所有 demo 都从低速开始
```

真实机器人也必须有这一层。

### 8.3 第 3 层：平面位姿更新

当前教学 demo 用运动学公式更新位姿：

```python
pose.yaw += yaw_rate * dt
pose.x += cos(yaw) * vx * dt
pose.y += sin(yaw) * vx * dt
```

含义是：

```text
机器人朝哪个方向，就沿哪个方向前进
角速度会持续改变朝向
```

这是后续里程计、导航、路径跟踪的基础。

### 8.4 第 4 层：腿部步态动画

代码里：

```python
make_walk_joint_pose(vx, yaw_rate)
```

负责根据速度命令生成腿部摆动动画。

它使用了对角腿相位：

```text
FR/RL 同相
FL/RR 同相
两组对角腿相差 pi
```

这只是教学动画，不是完整动力学步态控制器。真正能在物理仿真里抗扰稳定行走，需要下一阶段继续学习：

```text
足端轨迹
接触检测
姿态反馈
MPC / WBC
强化学习策略
```

## 9. 接下来建议你这样学

按这个顺序推进：

```text
1. 运行 --demo straight，确认能看到稳定直行
2. 打开 mujoco_go2_control_demos.py，读 demo_straight()
3. 把直行速度 0.12 改成 0.06 / 0.16，对比画面
4. 运行 --demo turn，理解 yaw_rate 如何改变朝向
5. 运行 --demo distance，理解“距离误差 -> 速度 -> 停止阈值”
6. 运行 --demo circle，理解 vx / yaw_rate / 半径的关系
7. 运行 --demo waypoints，理解路径点状态机
8. 修改 3 个路径点坐标，观察路径变化
9. 把路径点状态机改成读取外部列表，为后面 SLAM/Nav2 铺路
10. 下一阶段再写 ROS2 `/cmd_vel` 版，把键盘控制、路径点跟踪和仿真连接起来
```

## 10. 安全原则

当前所有控制 demo 先只在 MuJoCo 仿真中跑。

不要直接把低层关节控制发给真实 Go2。

原因：

```text
低层关节控制会直接影响电机目标角和力矩
参数不合适可能让实机摔倒或撞击
仿真中的安全参数不能直接照搬到实机
实机需要急停、限幅、遥控接管、安全空间
```

仿真阶段也要养成习惯：

```text
限制最大关节角
限制最大 kp/kd
限制最大速度
限制最大角速度
准备 Stop / Damp / 趴下动作
每个 demo 先低速、小幅度测试
```

## 11. 后续更新约定

现在 2 到 6 号 demo 已经有第一版稳定教学实现。后续每次改控制代码时，本文档都要继续同步更新。

更新内容包括：

```text
1. 每个 demo 的最终代码文件路径
2. 每个 demo 的运行命令
3. 每个 demo 的关键参数
4. 每个 demo 的预期现象
5. 每个 demo 的失败现象和排查方法
6. 每个 demo 的截图或录屏说明
7. 如何从 MuJoCo demo 迁移到 ROS2 / Isaac / 实机
```

以后本项目新增或修改控制代码时，默认要求：

```text
关键代码后面加详细中文注释
控制参数必须说明物理含义
DDS topic 必须说明发布/订阅方向
状态机必须说明每个状态的作用
安全限幅必须说明为什么这么设
```
