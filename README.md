# FeedbackControl

本仓库核心是 `assignment_3_2026/controller.py` 中的无人机位置控制器。  
该控制器采用**两层级联 PID + 扰动观测器（DOBC）**结构，在仿真中输出速度与偏航角速度指令。

---

## 1. 控制器文件位置

- 主控制器：`assignment_3_2026/controller.py`
- 底层飞控（已存在，不需重复实现）：`assignment_3_2026/src/tello_controller.py`
- 仿真入口：`assignment_3_2026/run.py`

---

## 2. 控制架构总览

`controller.py` 的总体结构：

1. **外环位置 PID（Position PID）**
   - 输入：位置误差（世界坐标误差旋转到机体偏航坐标系）
   - 输出：期望速度 `desired_vx, desired_vy, desired_vz`

2. **内环速度控制（Feedforward + 小增益 PID）**
   - 结构：`cmd_vel = desired_vel + PID(vel_error)`
   - 说明：内环 PID 增益在当前版本设为 0，主要由前馈速度工作，避免与 `tello_controller` 的内置速度环“打架”。

3. **扰动观测器 DOBC（Disturbance Observer）**
   - 根据 `v_act - v_cmd` 进行低通估计，得到风扰估计 `d_hat`
   - 当前代码中风补偿缩放为 `0.0`（保留估计器运行，不主动叠加补偿）

4. **偏航 PID（Yaw PID）**
   - 使用角度包络到 `[-pi, pi]` 的误差进行控制
   - 输出 `yaw_rate` 并限幅

5. **输出限幅**
   - 线速度：`[-1.0, 1.0] m/s`
   - 偏航角速度：`[-1.745, 1.745] rad/s`

---

## 3. 控制器输入输出接口

函数签名：

```python
controller(state, target_pos, dt, wind_enabled=False)
```

### 输入

- `state`：长度为 6 的状态向量  
  `[pos_x, pos_y, pos_z, roll, pitch, yaw]`（世界系姿态角）
- `target_pos`：长度为 4 的目标  
  `(target_x, target_y, target_z, target_yaw)`
- `dt`：控制步长（`run.py` 中位置控制频率为 `50 Hz`，即 `0.02 s`）
- `wind_enabled`：是否启用风扰标志

### 输出

- `(cmd_vx, cmd_vy, cmd_vz, cmd_yaw_rate)`
  - 速度命令在**偏航对齐机体系**下解释
  - 偏航角速度单位为 `rad/s`

---

## 4. 坐标系说明（非常关键）

- 状态位置、姿态以及 DOBC 估计最初在**世界坐标系**；
- 控制输出速度是在**仅随 yaw 旋转的机体系**（yaw-rotated body frame）；
- 因此代码中会把世界系误差/速度/扰动估计旋转到该机体系后再参与控制。

这也是 `controller.py` 中 `cos(yaw), sin(yaw)` 旋转计算的原因。

---

## 5. 主要参数（当前默认值）

### 外环位置 PID

- `POS_KP = 3.0`
- `POS_KI = 2.5`
- `POS_KD = 1.2`
- 单轴积分限幅：`integral_limit=0.3`
- 单轴输出限幅：`output_limit=1.0`

### 内环速度 PID（小校正环）

- `VEL_KP = 0.0`
- `VEL_KI = 0.0`
- `VEL_KD = 0.0`
- 单轴积分限幅：`0.5`
- 单轴输出限幅：`0.3`

### 偏航 PID

- `YAW_KP = 2.0`
- `YAW_KI = 0.05`
- `YAW_KD = 0.1`
- 输出限幅：`1.745 rad/s`

### DOBC

- `alpha = 0.005`（低通 EMA 平滑系数；越小越平滑、响应越慢）
- 估计量 `d_hat` 限幅：`[-1.0, 1.0]`
- 风补偿缩放：`0.0`（当前禁用主动补偿，仅保留扰动估计）

---

## 6. 控制流程（对应 controller.py 主函数）

每个控制周期主要步骤：

1. 读取位置、偏航、目标；
2. 计算世界系位置误差；
3. 将误差旋转到偏航对齐机体系；
4. 外环 PID 生成期望速度；
5. 由位置差分 + EMA（Exponential Moving Average，指数滑动平均）估计当前速度；
6. 更新 DOBC 并得到风扰估计；
7. 计算速度误差并做内环修正（前馈 + 小反馈）；
8. 计算偏航角速度；
9. 执行限幅并返回控制量；
10. 记录日志到 `flight_log.csv`。

---

## 7. 日志输出

控制器内置 `DataLogger`，默认生成：

- `flight_log.csv`

字段包括时间、状态、目标、误差、速度命令、偏航命令，便于后处理与调参。

---

## 8. 如何运行

在仓库根目录执行：

```bash
cd assignment_3_2026
python run.py
```

### 运行时快捷键（`run.py`）

- `k`：开关风扰
- `r`：重置无人机和控制器
- `← / →`：切换目标点
- `q`：退出

---

## 9. 与底层控制器的关系

`src/tello_controller.py` 已经包含：

- 速度控制环
- 姿态控制环
- 角速度控制环
- 电机混控与 RPM 约束

因此 `controller.py` 的内环采取“前馈为主、反馈为辅”的设计，避免重复实现并减少耦合冲突。

---

## 10. 调参建议

1. 先调外环（位置）：
   - `KP` 影响响应速度
   - `KD` 影响阻尼
   - `KI` 用于消除稳态误差（注意积分饱和）
2. 内环小增益从 0 开始，仅在必要时增加少量阻尼校正；
3. 风扰场景先观察 DOBC 估计稳定性，再决定是否引入主动补偿比例；
4. 每次改动后结合 `flight_log.csv` 评估过冲、收敛时间和稳态误差。
