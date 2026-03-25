# FeedbackControl

本文档依据 `assignment_3_2026/controller.py` 的**当前完整实现**重写，覆盖控制结构、参数、接口、坐标系、日志与运行方式。

---

## 1. 代码位置与角色

- 主控制器：`assignment_3_2026/controller.py`
- 仿真入口：`assignment_3_2026/run.py`
- 底层飞控：`assignment_3_2026/src/tello_controller.py`

`controller.py` 负责高层位置与偏航控制，输出期望速度/偏航角速度；底层 `tello_controller.py` 负责更高频的速度/姿态/角速度/电机控制。

---

## 2. 控制器总体架构（controller.py）

当前实现为**两层级联 PID + 扰动观测器（DOBC）+ 偏航 PID**：

1. **外环位置 PID（Position PID）**
   - 输入：位置误差（先在世界系计算，再旋转到 yaw 对齐机体系）
   - 输出：`desired_vx, desired_vy, desired_vz`

2. **速度估计（差分 + EMA）**
   - 通过 `(pos - prev_pos) / dt` 估算世界系速度
   - 经过 `_EMA_ALPHA = 0.4` 平滑后，再旋转到 yaw 对齐机体系

3. **DOBC 扰动观测器**
   - 观测量：`d_raw = v_act - v_cmd_prev`（世界系）
   - 更新：`d_hat = alpha * d_raw + (1 - alpha) * d_hat`
   - 限幅：`[-1.0, 1.0]`
   - 实际策略：即使 `wind_enabled=True`，代码中补偿比例也为 `0.0`，因此保持“估计运行、补偿关闭”

4. **内环速度控制（前馈 + 速度 PID 修正）**
   - 结构：`cmd_v = desired_v + PID(desired_v - est_v)`
   - 当前内环 PID 增益为 0，等价于前馈主导（保留修正结构便于后续调参）

5. **偏航 PID（Yaw PID）**
   - 误差：`wrap_angle(target_yaw - yaw)`，包络到 `[-pi, pi]`
   - 输出：`cmd_yaw_rate`

6. **命令限幅**
   - `cmd_vx/cmd_vy/cmd_vz ∈ [-1.0, 1.0]`
   - `cmd_yaw_rate ∈ [-1.745, 1.745]`（rad/s）

---

## 3. 关键类与全局状态

### 3.1 DataLogger

- 类：`DataLogger`
- 默认输出文件：`flight_log.csv`
- 每个控制周期写入一行，字段为：
  - `time`
  - `pos_x,pos_y,pos_z,yaw`
  - `target_x,target_y,target_z,target_yaw`
  - `error_x,error_y,error_z,error_yaw`
  - `cmd_vx,cmd_vy,cmd_vz,cmd_yaw_rate`

### 3.2 PID

- 类：`PID(kp, ki, kd, integral_limit, output_limit)`
- 特点：
  - 积分限幅防 windup
  - 首次更新时导数项置零，避免尖峰
  - 可选输出限幅

### 3.3 DisturbanceObserver

- 类：`DisturbanceObserver(alpha=0.02)`
- 当前实例：`dobc = DisturbanceObserver(alpha=0.005)`
- 状态：`d_hat`（3 轴世界系扰动估计）

### 3.4 控制器内部记忆变量

- `_prev_pos`：上一周期位置（用于速度差分）
- `_ema_vel`：EMA 平滑后的速度估计（世界系）
- `_EMA_ALPHA = 0.4`：速度估计平滑系数
- `_prev_cmd_world`：上一周期世界系命令速度（供 DOBC 使用）

---

## 4. 当前默认参数（与代码一致）

### 外环位置 PID

- `POS_KP = 2.4`
- `POS_KI = 2.0`
- `POS_KD = 0.9`
- 每轴 PID：`integral_limit=0.3`，`output_limit=1.0`

### 内环速度 PID

- `VEL_KP = 0.0`
- `VEL_KI = 0.0`
- `VEL_KD = 0.0`
- 每轴 PID：`integral_limit=0.5`，`output_limit=0.3`

### 偏航 PID

- `YAW_KP = 2.4`
- `YAW_KI = 0.0`
- `YAW_KD = 0.1`
- PID 输出限幅：`1.745`

### DOBC

- `alpha = 0.005`
- `d_hat` 限幅：`[-1.0, 1.0]`
- 风补偿缩放：`0.0`（仅估计，不主动补偿）

---

## 5. 输入输出接口

函数签名：

```python
controller(state, target_pos, dt, wind_enabled=False)
```

输入：

- `state`: `[pos_x, pos_y, pos_z, roll, pitch, yaw]`
- `target_pos`: `(target_x, target_y, target_z, target_yaw)`
- `dt`: 控制步长（`run.py` 中位置环调用步长 `1/50 = 0.02s`）
- `wind_enabled`: 是否处于风扰场景

输出：

- `(cmd_vx, cmd_vy, cmd_vz, cmd_yaw_rate)`
  - 速度命令定义在 yaw 对齐机体系
  - 偏航角速度单位为 rad/s

---

## 6. 坐标系与旋转关系（核心）

`controller.py` 同时使用两类坐标系：

- **世界系（World）**：位置、姿态、DOBC 内部估计
- **yaw 对齐机体系（Yaw-rotated body）**：控制误差与速度命令计算

核心处理：

1. 世界系位置误差 `error_world = target - pos`
2. 用当前 `yaw` 将 `error_world` 旋转到机体系，送入位置 PID
3. 世界系速度估计同样旋转到机体系用于速度误差
4. DOBC 在世界系更新，估计结果再旋转到机体系（尽管当前补偿比例为 0）
5. 输出命令存回世界系 `_prev_cmd_world` 供下一周期 DOBC 使用

---

## 7. 单周期控制流程（按源码顺序）

1. 解析 `state/target`
2. 计算世界系位置误差
3. 旋转到 yaw 对齐机体系
4. 外环位置 PID 生成 `desired_v*`
5. 差分 + EMA 估计速度
6. DOBC 更新扰动估计（使用上周期命令）
7. 计算速度误差并执行前馈 + 内环修正
8. 计算偏航角速度命令
9. 对速度与偏航角速度限幅
10. 保存世界系命令到 `_prev_cmd_world`
11. 写入 `flight_log.csv`
12. 返回控制输出

---

## 8. 与 run.py 的配合关系

`run.py` 中：

- 物理仿真步长：`1/1000 s`
- 位置控制调用步长：`1/50 s`
- 每个位置控制周期调用一次 `controller.controller(...)`
- 控制器输出被 `check_action` 再次裁剪后送入 `tello_controller.compute_control(...)`

键盘控制：

- `k`：开关风扰
- `r`：重置并 `importlib.reload(controller)`（会重置控制器模块级状态）
- `← / →`：切换目标点
- `q`：退出

---

## 9. 运行方式

```bash
cd assignment_3_2026
python run.py
```

运行后可在该目录看到 `flight_log.csv`（由 `controller.py` 自动生成）。
