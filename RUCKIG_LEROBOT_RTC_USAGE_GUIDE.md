# Ruckig 在 LeRobot RTC 真实机器人场景中的使用指南

> 基于对 `/home/playeee/lerobot/examples/rtc/eval_with_real_robot.py` 及相关源码的分析，说明如何把 Ruckig 接入 LeRobot 的 Real-Time Chunking（RTC）链路。

---

## 1. 当前 LeRobot RTC 的动作执行链路

在 `eval_with_real_robot.py` 中，真实机器人控制采用**双线程架构**：

- **`get_actions` 线程**：不断调用策略（SmolVLA / Pi0 / Pi0.5）生成动作 chunk，通过 `ActionQueue.merge()` 写入共享队列。
- **`actor_control` 线程**：以固定频率 `cfg.fps`（默认 10 Hz）从队列取动作，经过 `robot_action_processor` 后直接调用 `robot.send_action(action)` 发送给机器人。

```text
策略模型 → ActionQueue.merge() → action_queue → actor_control.get() →
robot_action_processor → robot.send_action() → 电机
```

核心代码位置：

- [`/home/playeee/lerobot/examples/rtc/eval_with_real_robot.py`](file:///home/playeee/lerobot/examples/rtc/eval_with_real_robot.py#L700-L850)：`actor_control` 执行循环
- [`/home/playeee/lerobot/src/lerobot/policies/rtc/action_queue.py`](file:///home/playeee/lerobot/src/lerobot/policies/rtc/action_queue.py)：双队列缓冲与 RTC 合并逻辑
- [`/home/playeee/lerobot/src/lerobot/robots/so_follower/so_follower.py`](file:///home/playeee/lerobot/src/lerobot/robots/so_follower/so_follower.py#L180-L210)：`send_action` 直接把目标位置写入电机 Goal_Position

### 当前链路的问题

1. **离散目标点**：策略输出的是每步目标位置，电机靠自身 PID 去追，没有显式速度/加速度/加加速度约束。
2. **机械冲击**：如果相邻目标点差距大，电机可能产生较大速度和加速度，造成抖动或冲击。
3. **RTC 只保证"策略意图连续"**：RTC 通过前缀注意力让新 chunk 与旧 chunk 在重叠区域尽量一致，但它不保证运动学状态（速度、加速度）连续。
4. **多自由度不同步**：各关节独立执行，无法保证同时到达目标。

Ruckig 正好填补这一层：**把离散动作目标转换成满足 p-v-a-j 约束的平滑轨迹**。

---

## 2. Ruckig 能做什么

Ruckig 是一个**实时加加速度限制（Type V）轨迹生成器**，输入：

- 当前状态：`(current_position, current_velocity, current_acceleration)`
- 目标状态：`(target_position, target_velocity, target_acceleration)`
- 约束：`(max_velocity, max_acceleration, max_jerk)`

输出：每个控制周期对应的位置、速度、加速度，且保证：

- 时间最优或接近时间最优
- 速度、加速度、加加速度均在约束范围内
- 多自由度可时间同步
- 位置、速度、加速度在段间 C² 连续

Python 安装：

```bash
pip install ruckig
```

---

## 3. Ruckig 应该接在哪一层

推荐在 **`ActionQueue` 和 `robot.send_action` 之间**新增一个 `RuckigInterpolator` 模块：

```text
策略模型 → ActionQueue.merge() → action_queue →
RuckigInterpolator.plan() → 每个控制周期采样 →
robot_action_processor → robot.send_action() → 电机
```

不要改动策略模型本身，也不要改动 `ActionQueue` 的核心逻辑。Ruckig 作为**后处理平滑层**，对策略输出透明。

---

## 4. Chunk / 窗口粒度选择

RTC 场景下有三种典型用法，按平滑程度和计算开销排序：

### 方案 A：以整个 chunk 终点为目标（最简单）

- 取当前 chunk 的**最后一个动作**作为 Ruckig 目标。
- 当前机器人反馈状态作为初始状态。
- Ruckig 生成从当前到 chunk 终点的完整轨迹。
- 在 chunk 执行期间按 `cfg.fps` 采样。

**优点**：实现简单，Ruckig 调用频率低。
**缺点**：如果 chunk 很长，Ruckig 只保证到终点平滑，中间可能跟不上策略意图。

### 方案 B：按 `execution_horizon` 拆分子窗口（推荐）

- 把 chunk 按 `rtc.execution_horizon` 拆成多个子窗口。
- 每个子窗口终点作为 Ruckig 目标。
- 上一个子窗口的终点状态作为下一个子窗口的起点。

**优点**：平滑性更好，与 RTC 的 "执行视野" 概念天然对齐。
**缺点**：Ruckig 调用频率稍高。

### 方案 C：Receding Horizon（最实时）

- 每个控制周期都用**当前真实反馈**作为起点，重新规划到未来最近的目标点。
- 适用于高频伺服控制。

**优点**：最大程度消除累积误差，响应最快。
**缺点**：Ruckig 调用频率等于控制频率，需要保证每次计算在控制周期内完成。

**推荐**：在 LeRobot RTC 中先用**方案 B**，因为它和 `execution_horizon` 概念一致，且计算量可控。

---

## 5. 窗口间连续性处理

Ruckig 的输入输出天然是完整运动学状态。只要遵守两条规则，段间就是 C² 连续：

1. **上一段终点状态作为下一段起点状态**：

   ```python
   input.current_position = prev_output.new_position
   input.current_velocity = prev_output.new_velocity
   input.current_acceleration = prev_output.new_acceleration
   ```

2. **目标状态合理**：
   - 如果 VLA 只输出位置，用相邻目标点**有限差分**估计目标速度/加速度。
   - 如果目标速度/加速度与当前状态差距过大，Ruckig 会自动在约束内规划，但可能需要更长时间。

### 目标速度/加速度估计

```python
def estimate_target_state(actions, idx, fps, action_dim):
    """用相邻动作点估计目标速度和加速度。"""
    dt = 1.0 / fps
    target_pos = actions[idx]

    if idx + 1 < len(actions):
        target_vel = (actions[idx + 1] - actions[idx]) / dt
    else:
        target_vel = torch.zeros(action_dim)

    if idx + 2 < len(actions):
        target_acc = (actions[idx + 2] - 2 * actions[idx + 1] + actions[idx]) / (dt ** 2)
    else:
        target_acc = torch.zeros(action_dim)

    return target_pos, target_vel, target_acc
```

---

## 6. 具体代码集成示例

### 6.1 新增 `RuckigInterpolator` 类

```python
# lerobot/processor/ruckig_interpolator.py
import torch
import numpy as np
from ruckig import InputParameter, OutputParameter, Ruckig, Result


class RuckigInterpolator:
    def __init__(self, dofs: int, dt: float, max_vel, max_acc, max_jerk):
        """
        Args:
            dofs: 动作维度（如关节数）
            dt: 控制周期（秒），与 cfg.fps 对应，例如 10Hz -> 0.1s
            max_vel/max_acc/max_jerk: 各自由度约束，list/tuple of shape (dofs,)
        """
        self.dofs = dofs
        self.dt = dt
        self.otg = Ruckig(dofs, dt)
        self.inp = InputParameter(dofs)
        self.out = OutputParameter(dofs)

        self.inp.max_velocity = list(max_vel)
        self.inp.max_acceleration = list(max_acc)
        self.inp.max_jerk = list(max_jerk)

    def plan(
        self,
        current_pos,
        current_vel,
        current_acc,
        target_pos,
        target_vel,
        target_acc,
    ):
        """规划一段轨迹，返回是否成功以及轨迹对象。"""
        self.inp.current_position = list(current_pos)
        self.inp.current_velocity = list(current_vel)
        self.inp.current_acceleration = list(current_acc)

        self.inp.target_position = list(target_pos)
        self.inp.target_velocity = list(target_vel)
        self.inp.target_acceleration = list(target_acc)

        result = self.otg.update(self.inp, self.out)
        return result, self.out.trajectory

    def sample(self, trajectory, t: float):
        """从轨迹中采样 t 时刻的状态。"""
        state = trajectory.at_time(t)
        return (
            np.array(state.position),
            np.array(state.velocity),
            np.array(state.acceleration),
        )
```

### 6.2 修改 `actor_control` 调用 Ruckig

在 `eval_with_real_robot.py` 的 `actor_control` 中，把直接 `action_queue.get()` → `send_action` 改成：

```python
from lerobot.processor.ruckig_interpolator import RuckigInterpolator


def actor_control(robot, robot_action_processor, action_queue, shutdown_event, cfg):
    fps = cfg.fps
    dt = 1.0 / fps
    action_features = robot.action_features()
    dofs = len(action_features)

    # 根据机器人标定设置约束（示例值，需按实际机器人调整）
    max_vel = [1.0] * dofs
    max_acc = [2.0] * dofs
    max_jerk = [4.0] * dofs

    interpolator = RuckigInterpolator(dofs, dt, max_vel, max_acc, max_jerk)

    # 维护当前运动学状态
    current_pos = get_current_joint_positions(robot)
    current_vel = [0.0] * dofs
    current_acc = [0.0] * dofs

    # 当前正在执行的轨迹
    active_trajectory = None
    trajectory_start_time = 0.0
    trajectory_idx = 0

    while not shutdown_event.is_set():
        loop_start = time.perf_counter()

        # 如果当前轨迹执行完，从队列取下一个目标并重新规划
        if active_trajectory is None:
            action = action_queue.get()
            if action is None:
                # 队列为空，保持当前位置或上一次状态
                time.sleep(dt)
                continue

            target_pos = action.cpu().numpy()
            # 估计目标速度和加速度（也可用队列中后续点）
            target_vel = np.zeros(dofs)
            target_acc = np.zeros(dofs)

            result, active_trajectory = interpolator.plan(
                current_pos, current_vel, current_acc,
                target_pos, target_vel, target_acc,
            )

            if result == Result.Error:
                logger.error("Ruckig planning failed, skipping this action")
                active_trajectory = None
                continue

            trajectory_start_time = loop_start
            trajectory_idx = 0

        # 在当前轨迹上采样
        t_elapsed = time.perf_counter() - trajectory_start_time
        duration = active_trajectory.get_duration()

        if t_elapsed >= duration:
            # 到达目标，更新当前状态，准备下一段
            pos, vel, acc = interpolator.sample(active_trajectory, duration)
            current_pos, current_vel, current_acc = pos, vel, acc
            active_trajectory = None
            continue

        pos, vel, acc = interpolator.sample(active_trajectory, t_elapsed)

        # 构造动作字典并发送
        action_dict = {
            name: float(pos[i]) for i, name in enumerate(action_features)
        }
        action_processed = robot_action_processor((action_dict, None))
        robot.send_action(action_processed)

        # 可视化记录
        if cfg.display_data:
            log_executed_action(torch.from_numpy(pos), action_features, time.perf_counter())

        # 维持循环频率
        elapsed = time.perf_counter() - loop_start
        time.sleep(max(0, dt - elapsed - 0.001))
```

### 6.3 获取当前关节位置

可以从机器人观测读取：

```python
def get_current_joint_positions(robot: RobotWrapper):
    obs = robot.get_observation()
    feature_names = robot.action_features()
    return np.array([obs[name].item() for name in feature_names])
```

> 注意：`robot.get_observation()` 会访问电机总线，与 `get_actions` 线程有锁竞争。如果希望避免，可以在 `actor_control` 中缓存上一次 `send_action` 实际发出的位置作为当前位置近似。

---

## 7. 与 RTC 的配合关系

| 层级 | 负责什么 | 当前实现 | Ruckig 作用 |
|------|---------|---------|------------|
| 策略模型 | 生成动作 chunk | SmolVLA / Pi0 / Pi0.5 | 不改动 |
| RTC | chunk 间语义一致性 | `RTCProcessor.denoise_step()` | 不改动 |
| ActionQueue | 动作缓冲与合并 | `ActionQueue.merge()` | 不改动 |
| **Ruckig 插值层** | **运动学平滑** | **新增** | **核心作用** |
| 机器人底层 | 执行单步目标 | `send_action()` | 接收平滑后的目标 |

**关键理解**：

- RTC 保证"策略意图连续"（新 chunk 与前一个 chunk 的 leftover 语义一致）。
- Ruckig 保证"运动学状态连续"（位置、速度、加速度平滑过渡，满足约束）。
- 两者互补，应同时使用。

---

## 8. 多自由度同步

对于机械臂，建议开启 Ruckig 的时间同步：

```python
from ruckig import Synchronization

self.inp.synchronization = Synchronization.Time
```

这样所有关节会**同时到达目标**，避免末端走偏。如果某些维度（如 gripper）不需要同步，可对 gripper 单独用一个 Ruckig 实例。

---

## 9. 需要注意的问题

### 9.1 动作空间：关节空间 vs 末端位姿

- 如果策略输出的是**关节角度**（如 SO-100），直接对每个关节做 Ruckig。
- 如果策略输出的是**末端位姿**（6D pose），需要先做 IK 得到关节目标，再对关节做 Ruckig。

### 9.2 旋转表示

如果动作包含旋转（欧拉角、轴角、四元数）：

- **欧拉角**：注意周期性和万向锁；不建议直接做 Ruckig。
- **轴角 / 角速度**：可视为三维向量，用 Ruckig 规划。
- **四元数**：Ruckig 不直接支持，需转成角速度空间或在 SO(3) 上单独插值。

### 9.3 约束设置

`max_velocity`、`max_acceleration`、`max_jerk` 必须根据实际机器人标定：

- 可从电机 datasheet 获取最大速度和加速度。
- 加加速度限制建议从实验中调试，避免过大导致抖动，过小导致迟缓。
- SO-100 等低成本舵机通常速度和加速度较低，需保守设置。

### 9.4 实时性

- Ruckig 计算复杂度低，单次计算通常在微秒级。
- Python 绑定由于 GIL 和调用开销，可能比 C++ 慢一个数量级；如果控制频率很高（>100Hz），建议用 C++ 实现该层。
- 对于 10Hz 的 LeRobot RTC 场景，Python 版本完全足够。

### 9.5 初始状态超约束

如果当前速度/加速度超出 `max_velocity` / `max_acceleration`，Ruckig 会返回 `Result.Error`。处理方式：

- 用 Ruckig 的 **BrakeProfile** 先把状态拉回约束范围。
- 或者在初始化时把当前速度/加速度 clip 到约束内。

### 9.6 目标不可达

如果策略输出的目标位置超出关节限位，Ruckig 本身不会处理。需要在 `send_action` 前或在 `robot_action_processor` 中对位置进行 clip。

---

## 10. 推荐的最小可运行改造步骤

1. **安装依赖**：

   ```bash
   pip install ruckig
   ```

2. **新增 `lerobot/processor/ruckig_interpolator.py`**，实现 `RuckigInterpolator`。

3. **修改 `eval_with_real_robot.py`**：
   - 在 `actor_control` 中实例化 `RuckigInterpolator`。
   - 用 Ruckig 采样结果替换直接从队列取出的动作。
   - 把队列中的目标作为 Ruckig 的 `target_position`。

4. **标定约束**：根据实际机器人调整 `max_velocity`、`max_acceleration`、`max_jerk`。

5. **可视化验证**：启用 `display_data=true`，在 rerun 中对比 `action/executed` 和 `action/chunk/*/planned`，观察是否更平滑。

---

## 11. 总结

在 LeRobot RTC 中引入 Ruckig，本质上是在**策略输出**和**机器人执行**之间增加一个**运动学平滑层**：

- 策略负责"去哪里"；
- RTC 负责"下一个 chunk 怎么和当前意图接得上"；
- Ruckig 负责"怎么以有限的速度、加速度、加加速度平滑地走过去"。

这样可以在不改动策略和 RTC 的前提下，显著改善真实机器人的运动平滑性，减少机械冲击，并保证 chunk 切换时的运动学连续性。
