# Loophand：触觉剪切力抓取控制

> 用指尖/指节的 GEN3 三维触觉传感器感知**切向力变化（剪切率）**和**滑移比**，通过 **PID + 导纳控制 + 欠驱动逆运动学** 实时调节 Aero Hand Open 的抓取力，目标是在物体开始滑落之前把法向力补上去。

本目录是 Loophand 项目的核心自研模块。Aero Hand Open 的硬件、固件、SDK、ROS 2 仓库位于 [`third_party/aero-hand-open`](../../third_party/aero-hand-open/)，在本项目中作为第三方依赖使用。

目录：

1. [设计目标](#1-设计目标)
2. [系统架构](#2-系统架构)
3. [算法说明](#3-算法说明)
4. [状态机](#4-状态机)
5. [目录结构](#5-目录结构)
6. [硬件与系统要求](#6-硬件与系统要求)
7. [安装](#7-安装)
8. [运行](#8-运行)
9. [GUI 与可视化](#9-gui-与可视化)
10. [参数调优](#10-参数调优)
11. [安全注意事项](#11-安全注意事项)
12. [故障排查](#12-故障排查)
13. [已知限制与后续工作](#13-已知限制与后续工作)
14. [上游与许可](#14-上游与许可)

---

## 1. 设计目标

传统的位置控制抓取只能“闭到某个角度”，无法知道物体是否正在滑落。本模块把触觉信号引入闭环：

1. **接近**：手指以固定速度闭合，直到检测到接触。
2. **接触锚定**：接触瞬间重置导纳状态，避免把接近阶段的力误差带入闭环。
3. **力闭环**：根据当前切向力估计维持抓取所需的最小法向力，并用导纳控制器把手指往“既有位移”方向微调。
4. **滑移保护**：当滑移比超过估计摩擦系数时，额外增加法向力；当法向力不足时禁止手指回退（棘轮）。
5. **可视化与调参**：3D 手模型、力箭头、力/关节曲线、独立调参示波器。

> 当前实现面向 **左手 Aero Hand Open**，使用 7 个主动关节和 9 个触觉通道。

---

## 2. 系统架构

```text
┌────────────────────────────── 应用层 ──────────────────────────────┐
│  app/main_gui.py        PyQt5 总控界面（3D、曲线、启动/抓取按钮）    │
│  viz/shear_debugger.py  独立进程调参示波器（PID / 导纳滑条）          │
│  viz/hand_viz_3d.py     URDF + STL 实时渲染、力箭头                  │
└──────────────────────────────────┬─────────────────────────────────┘
                                   │ multiprocessing.Queue (100 Hz → 30 Hz)
┌──────────────────────────────────▼─────────────────────────────────┐
│  控制层  controller/shear_grasp_controller.py                       │
│                                                                     │
│  IDLE ──START_GRASP──► PRE_SHAPE ──到位──► ACTIVE_GRASP              │
│                                                │                    │
│       ┌────────────────────────────────────────┘                    │
│       │ 每根手指独立：                                              │
│       │   未接触 → 定速闭合                                          │
│       │   已接触 → 力目标 Fn* + 切向 PID → 导纳 → 阻尼最小二乘 IK   │
│       └─────────────────────────────────────────────────────────────│
└──────────────────────────────────┬─────────────────────────────────┘
                                   │ 7 个主动关节角 / 力矩
┌──────────────────────────────────▼─────────────────────────────────┐
│  感知层  driver/tactile_driver.py                                   │
│  921600 串口 → AA56 自动回传帧 → 28 通道识别 → 力三元组 → 去皮/死区  │
└──────────────────────────────────┬─────────────────────────────────┘
                                   │ 原始关节角 / 电流 / 温度
┌──────────────────────────────────▼─────────────────────────────────┐
│  机器人抽象  aero_open_sdk（third_party/aero-hand-open/sdk）         │
└──────────────────────────────────┬─────────────────────────────────┘
                                   │ USB 串口 921600
┌──────────────────────────────────▼─────────────────────────────────┐
│  固件 / 硬件  third_party/aero-hand-open/firmware + hardware         │
└────────────────────────────────────────────────────────────────────┘
```

### 2.1 单次控制步（100 Hz）

`controller/shear_grasp_controller.py::step()` 的顺序：

1. 处理 GUI 指令队列（`START_GRASP` / `OPEN_TUNER`）。
2. 读取 7 个主动关节角，映射为 16 个 URDF 关节角。
3. 读取触觉最新数据；对每个传感器做低通、剪切率、滑移比计算。
4. 按状态机执行：
   - `IDLE`：不下发运动指令，只向 GUI 推数据。
   - `PRE_SHAPE`：拇指 CMC 外展到 `1.65 rad`。
   - `ACTIVE_GRASP`：逐指判断接触并闭环。
5. 对每根手指：
   - **未接触**：以固定步长闭合（食指/中指/无名指/小指 `0.03 rad/step`，拇指 `0.01 rad/step`）。
   - **已接触**：锚定导纳状态 → 计算目标法向力 → 切向力 PID → 导纳产生笛卡尔位移 → IK 转关节增量。
6. 关节增量限幅（`±0.06 rad/step`）后下发 `hand.set_joint_positions()`。
7. 把 `q / forces / kinematics / state` 放入 GUI 队列。

---

## 3. 算法说明

### 3.1 触觉信号

对每个通道的力向量 `f = [fx, fy, fz]`：

```text
f_lp   = α · f + (1-α) · f_lp                  # 一阶低通，α = 0.4
dF/dt  = (f_lp - f_lp_prev) / Δt              # 剪切率（再做 0.1 的低通）
Ft     = || f_lp[0:2] ||                       # 切向力
Fn     = f_lp[2]                               # 法向力
slip   = Ft / max(|Fn|, 1e-3)                  # 滑移比
```

驱动层还会做：

- **去皮（tare）**：静止采集 2 s（`variants` 中为 4 s），逐通道求均值作为零点；解析时减去偏移。
- **死区**：`|f| < 2`（原始计数）置零，抑制静态噪声。

### 3.2 目标法向力

```python
f_min_safe     = max(min_force, 0.2)                 # min_force = 8.0 N
f_physics_need = (Ft / mu_est) * safety_margin       # mu_est = 0.4, safety = 1.5
f_target       = max(f_min_safe, f_physics_need)

if slip > mu_est:
    f_target += (slip - mu_est) * 5.0                # 滑移补偿
```

含义：切向力越大，允许的最大静摩擦越小，需要的法向力越大；检测到滑移时再额外加力。

### 3.3 切向力 PID

```python
shear_error = -shear_rate[0:2]
f_tangent   = PID(shear_error)       # Kp/Ki/Kd 初始 0.1 / 0.0 / 0.0
```

切向力指令与目标法向力组合成本体系指令：

```python
f_cmd_local = [f_tangent[0], f_target, f_tangent[1]]
f_cmd_world = R · f_cmd_local
f_err       = f_cmd_world - R · f_measured
if ||f_err|| < 0.02: f_err = 0       # 死区
```

### 3.4 导纳控制

`controller/admittance.py`：

```text
a = (F_ext - F_const - D·v - K·x) / M
a = clip(a, ±max_acc)
v += a·Δt ;  v = clip(v, ±max_vel)
x += v·Δt
```

主控制器参数：

| 参数 | 值 | 说明 |
| --- | --- | --- |
| `M` | 0.11 | 虚拟质量 |
| `D` | 12.0 | 阻尼，偏大以抑制振荡 |
| `K` | 0.0 | 不使用虚拟刚度（纯柔顺） |
| `max_vel` | 0.5 | 速度限幅 |
| `max_acc` | 10.0 | 加速度限幅 |

输出 `dx = v · Δt`，再乘 `ADM_DIRECTION = -1.0` 映射到“闭合”方向。

### 3.5 欠驱动逆运动学

URDF 共 21 个关节，其中 5 个指端关节为 `fixed`，Pinocchio 模型为 **16 个自由度**；16 个可动关节中只有 7 个是主动自由度：

| 主动关节 | 说明 |
| --- | --- |
| `left_thumb_cmc_abd` | 拇指外展 |
| `left_thumb_cmc_flex` | 拇指屈曲 |
| `left_thumb_mcp` | 拇指 MCP |
| `left_index_mcp_flex` | 食指 MCP |
| `left_middle_mcp_flex` | 中指 MCP |
| `left_ring_mcp_flex` | 无名指 MCP |
| `left_pinky_mcp_flex` | 小指 MCP |

其余远端关节按 0.8 / 0.6 的比例跟随 MCP：

```text
dq_pip = 0.8 · dq_mcp
dq_dip = 0.6 · dq_mcp
```

使用 Pinocchio + 阻尼最小二乘：

```text
dq_active = Jᵀ (J Jᵀ + λ² I)⁻¹ dx      # λ = 0.05
```

### 3.6 安全与防回退

- **棘轮**：若导纳速度方向为“张开”且当前法向力 `< 8 N`，把 `dx` 整体置零，避免物体还没抓住就松手。
- **关节增量限幅**：每步 `±0.06 rad`。
- **力误差死区**：`0.02 N`。
- **接触迟滞**：主控制器当前阈值为 `Fn > 0.1 N`；实验版本 `experiments/filtered_control.py` 使用 `ON=0.1 N / OFF=0.03 N` 的迟滞。
- **Ctrl-C**：主循环退出时把 7 个执行器置 0（张开）。

> 注意：主控制器**没有**舵机温度保护。`controller/hybrid_position_torque.py` 中有 80 °C 保护逻辑。长时间堵转实验请自行增加温度监控。

---

## 4. 状态机

| 状态 | 触发 | 行为 | 退出条件 |
| --- | --- | --- | --- |
| `IDLE` | 上电 / 初始化完成 | 不发送运动指令，仅推送 GUI 数据 | 收到 `START_GRASP` |
| `PRE_SHAPE` | GUI 点击 “ACTIVATE GRASP” | 拇指 CMC 外展到 `1.65 rad`，速度 `0.03 rad/step` | 误差 `< 0.05 rad` |
| `ACTIVE_GRASP` | 预姿态到位 | 逐指接触判断 + 力闭环 | 无自动退出（Ctrl-C 退出） |

`IDLE` 状态下即使手指碰到物体也不会运动，这是为了避免 GUI 一连接就误动作。

---

## 5. 目录结构

```text
src/loophand/
├── README.md                        # 本文件
├── __init__.py
├── paths.py                         # 资产路径（URDF / meshes / generated）
├── app/
│   └── main_gui.py                  # PyQt5 总控界面（启动、抓取、曲线）
├── controller/
│   ├── pid.py                       # 二维有界 PID
│   ├── admittance.py                # 导纳控制器（限幅版）
│   ├── shear_grasp_controller.py    # ★ 当前主控制器
│   ├── simple_shear_controller.py   # 无 GUI / 无状态机的简化版
│   └── hybrid_position_torque.py    # 早期位置-力矩混合控制（无触觉）
├── driver/
│   └── tactile_driver.py            # GEN3 触觉传感器串口驱动
├── viz/
│   ├── hand_viz_3d.py               # URDF + STL 3D 渲染、力箭头
│   └── shear_debugger.py            # 独立调参示波器
├── experiments/
│   ├── README.md
│   ├── filtered_control.py          # 目标力/速度平滑 + 接触迟滞实验版
│   └── variants/
│       └── shear_grasp_controller_tare4s.py  # 主控制器快照（标定 4 s）
└── assets/
    ├── aero_hand_open_left.urdf     # 左手 URDF（16 自由度）
    ├── meshes/                      # 22 个 STL
    └── generated/                   # 运行时生成文件（已 gitignore）
```

### 5.1 重组前 → 重组后映射

| 旧路径 | 新路径 |
| --- | --- |
| `loophand/grasp_control/visual_shear_control.py` | `src/loophand/controller/shear_grasp_controller.py` |
| `loophand/grasp_control/shear_control.py` | `src/loophand/controller/simple_shear_controller.py` |
| `loophand/grasp_control/grasp_control.py` | `src/loophand/controller/hybrid_position_torque.py` |
| `loophand/grasp_control/tactile_driver.py` | `src/loophand/driver/tactile_driver.py` |
| `loophand/grasp_control/admittancecontroller.py` | `src/loophand/controller/admittance.py` |
| `loophand/grasp_control/pid.py` | `src/loophand/controller/pid.py` |
| `loophand/grasp_control/hand_viz_3d.py` | `src/loophand/viz/hand_viz_3d.py` |
| `loophand/grasp_control/shear_debugger.py` | `src/loophand/viz/shear_debugger.py` |
| `loophand/grasp_control/main_gui.py` | `src/loophand/app/main_gui.py` |
| `loophand/grasp_control/test.py` | `src/loophand/experiments/filtered_control.py` |
| `loophand/grasp_control/reverse/visual_shear_control.py` | `src/loophand/experiments/variants/shear_grasp_controller_tare4s.py` |
| `loophand/grasp_control/aero_hand_open_left.urdf` | `src/loophand/assets/aero_hand_open_left.urdf` |
| `loophand/grasp_control/urdf/*.STL` | `src/loophand/assets/meshes/*.STL` |
| `loophand/grasp_control/j.html` | `docs/demos/jacobian_demo.html` |
| `loophand/grasp_control/urdf/hand_viz.html` | `docs/demos/hand_viz.html` |
| `sdk/src/aero_open_sdk/Read_Single_Sensor_Hand.py` | `tools/read_tactile_sensor.py` |

> `reverse/` 中与主版本逐字节相同的三个文件（`main_gui.py`、`shear_control.py`、`shear_debugger.py`）已去重；唯一不同的控制器快照保留在 `experiments/variants/`。

---

## 6. 硬件与系统要求

### 6.1 硬件

- **Aero Hand Open 左手**，7 个总线舵机。
- **GEN3 触觉传感器**，本模块使用以下 9 个通道：

  | 手指 | 通道 |
| --- | --- |
  | 拇指 | `Thumb-Tip` |
  | 食指 | `Index-Tip`, `Index-Mid` |
  | 中指 | `Middle-Tip`, `Middle-Mid` |
  | 无名指 | `Ring-Tip`, `Ring-Mid` |
  | 小指 | `Pinky-Tip`, `Pinky-Mid` |

  驱动本身识别 28 个通道（含掌部 Palm-1..8），当前控制只订阅上述 9 个。

- **串口**：手和触觉传感器各一个 USB 串口，默认：
  - 手：`/dev/ttyACM1`，921600 baud
  - 触觉：`/dev/ttyACM0`，921600 baud

  GUI 中可切换。请把用户加入 `dialout` 组：

  ```bash
  sudo usermod -aG dialout "$USER"
  # 重新登录后生效
  ```

### 6.2 软件

| 依赖 | 用途 | 来源 |
| --- | --- | --- |
| Python 3.10 | 运行时 | conda / 系统 |
| `numpy` | 数值计算 | pip |
| `pyserial` | 触觉传感器串口 | pip |
| `pinocchio` | 正/逆运动学 | ROS 2 Humble 或 conda |
| `pyqtgraph`, `PyQt5` | GUI、3D、示波器 | pip |
| `trimesh`, `scipy` | STL 加载、旋转矩阵 | pip |
| `aero-open-sdk` | 手部串口协议 | `third_party/aero-hand-open/sdk`（editable） |

本机已验证环境：conda `tehandcontrol`（Python 3.10）+ `source /opt/ros/humble/setup.bash`。

---

## 7. 安装

```bash
# 1. 进入项目根目录
cd /path/to/loophand

# 2. 安装手部 SDK（editable）
pip install -e third_party/aero-hand-open/sdk

# 如果离线环境提示找不到 uv_build 构建后端：
# pip install uv_build
# pip install -e third_party/aero-hand-open/sdk --no-build-isolation

# 3. 安装 GUI / 计算依赖
pip install -r requirements/grasp.txt

# 4. Pinocchio（二选一）
source /opt/ros/humble/setup.bash
# 或使用带 pinocchio 的 conda 环境

# 5. 自检
python scripts/check_env.py
```

---

## 8. 运行

所有命令都建议在项目根目录执行；`scripts/*.sh` 会自动把 `src/` 加入 `PYTHONPATH`。

### 8.1 GUI（推荐）

```bash
./scripts/run_gui.sh
# 等价于：
# PYTHONPATH=src python -m loophand.app.main_gui
```

操作顺序：

1. 选择手部/触觉串口（默认 `/dev/ttyACM1`、`/dev/ttyACM0`）。
2. 点击 **1. START SYSTEM (Connect)**：启动后台控制进程，触觉传感器开始 2 s 标定（此时不要碰传感器）。
3. 点击 **2. ACTIVATE GRASP (Start Control)**：状态机从 `IDLE` 进入 `PRE_SHAPE`，随后自动进入 `ACTIVE_GRASP`。
4. 右侧下拉框选择要观察的传感器；曲线显示力三分量和 5 个关节角。
5. **Open Parameter Tuner**：打开独立调参窗口（滑块修改 PID/导纳参数，实时生效）。

### 8.2 无 GUI（主控制器）

```bash
PYTHONPATH=src python -m loophand.controller.shear_grasp_controller
```

会直接连接默认串口并开始运行（无 `START_GRASP` 指令，状态机初始为 `IDLE`；需要手动在代码中把状态置为 `PRE_SHAPE`，或使用 GUI）。

### 8.3 简化版（无状态机）

```bash
PYTHONPATH=src python -m loophand.controller.simple_shear_controller
```

### 8.4 早期位置-力矩混合控制

```bash
PYTHONPATH=src python -m loophand.controller.hybrid_position_torque
```

不使用触觉；通过电流阈值在位置/力矩模式间切换。力矩模式需要刷入带**有符号力矩**补丁的固件（见 [`third_party/README.md`](../../third_party/README.md)）。

### 8.5 实验版本

```bash
PYTHONPATH=src python -m loophand.experiments.filtered_control
PYTHONPATH=src python -m loophand.experiments.variants.shear_grasp_controller_tare4s
```

### 8.6 触觉传感器单独调试

```bash
./scripts/run_sensor_monitor.sh
# 或
python tools/read_tactile_sensor.py
```

交互式选择串口，打印版本号、自动回传帧、LRC 校验结果与通道数据。

---

## 9. GUI 与可视化

- **左侧 3D 视图**：URDF + STL 渲染；指尖和近节为金色高亮；力箭头起点在指腹表面，长度 `1 N ≈ 1 cm`，最大 8 cm，颜色从蓝到红表示 0–8 N。
- **右侧曲线**：
  - `Tactile Force`：当前选中传感器的 `Fx/Fy/Fz`，200 点滑动窗口。
  - `Joint Angles`：前 5 个关节角（度），200 点窗口。
- **参数调参器**（独立进程）：
  - 三幅图：`Fn/Ft`、`Shear Rate/PID`、`Slip/Adm Velocity`。
  - 可切换目标传感器；滑条修改 `Kp/Ki/Kd/M/D`。
  - 参数通过 `multiprocessing.Queue` 回传主控制器，只对当前目标传感器生效。

---

## 10. 参数调优

主控制器参数集中在 `ShearGraspController.__init__()` 的 `self.params` 和模块常量中：

| 参数 | 默认值 | 作用 | 建议 |
| --- | --- | --- | --- |
| `mu_est` | 0.4 | 估计摩擦系数 | 光滑物体调低，粗糙物体调高 |
| `safety_margin` | 1.5 | 法向力安全系数 | 易碎物体调低（1.1–1.3） |
| `min_force` | 8.0 N | 最小法向力 | 太小会滑，太大会压坏物体 |
| `contact_thresh` | 0.1 N | 接触阈值 | 噪声大时调高 |
| `limit_speed` | 0.06 rad/step | 每步关节增量上限 | 降低以更稳，升高以更快 |
| `approach_other_speed` | 0.03 rad/step | 未接触手指闭合速度 | — |
| `approach_thumb_speed` | 0.01 rad/step | 拇指闭合速度 | — |
| `thumb_abd_target` | 1.65 rad | 预姿态拇指外展角 | 依物体宽度调整 |
| `pre_shape_tol` | 0.05 rad | 预姿态到位容差 | — |
| PID `Kp/Ki/Kd` | 0.1 / 0.0 / 0.0 | 切向力 PID | 先用调参器小幅增加 |
| 导纳 `M/D/K` | 0.11 / 12.0 / 0.0 | 柔顺程度 | `D` 太小会振荡，太大响应慢 |
| 低通 `alpha` | 0.4 | 触觉平滑 | 噪声大时降低 |
| 滑移增益 | 5.0 | 滑移超阈值后的加力斜率 | 保守可调低 |

调参建议：

1. 先空载观察 `Fn` 噪声和 `slip` 基线。
2. 用刚性物体测试接触阈值与 `min_force`。
3. 调 `D` 抑制振荡，再调 PID。
4. 用易滑物体观察 `Ft` 上升时 `Ftarget` 是否及时增加。

---

## 11. 安全注意事项

1. **首次运行请让手悬空**，不要直接抓取硬物或人体。
2. 触觉标定（tare）期间**不要接触传感器**，否则零点会偏移。
3. 主控制器没有温度保护；长时间堵转时请监控舵机温度，必要时断电。
4. 出现异常时优先 `Ctrl-C`（主循环会把执行器置 0）或直接断电。
5. 修改 `mu_est`、`min_force`、`safety_margin` 后先低速验证。
6. 串口断开时后台线程可能继续运行；GUI 中请用窗口关闭事件终止子进程。

---

## 12. 故障排查

| 现象 | 可能原因 | 处理 |
| --- | --- | --- |
| `Permission denied: /dev/ttyACM0` | 用户不在 `dialout` 组 | `sudo usermod -aG dialout $USER` 后重新登录 |
| `未检测到任何传感器` | 传感器未上电 / 波特率 / 串口错 | 用 `tools/read_tactile_sensor.py` 单独验证 |
| GUI 启动即退出 | `PyQt5` / `pyqtgraph` 不匹配 | 运行 `python scripts/check_env.py` |
| 3D 视图空白 / 加载失败 | mesh 路径或 STL 缺失 | 确认 `src/loophand/assets/meshes/` 有 22 个 STL |
| `ModuleNotFoundError: pinocchio` | 未 source ROS 或 conda 环境不对 | `source /opt/ros/humble/setup.bash` |
| `ModuleNotFoundError: aero_open_sdk` | SDK 未安装 / editable 路径失效 | `pip install -e third_party/aero-hand-open/sdk` |
| 点击 ACTIVATE GRASP 后不动 | 仍处于 `IDLE` / 预姿态未完成 / 未接触 | 看终端日志与 GUI 状态；确认串口正确 |
| 力读数漂移 | 未标定 / 温漂 / 接触 | 重新 tare，静置预热 |
| 抓取振荡 | 导纳阻尼太小 / PID 太大 | 增大 `D`，降低 `Kp` |
| 物体滑落但不加力 | `mu_est` 偏大 / `safety_margin` 偏小 | 降低 `mu_est`，提高 `safety_margin` |

---

## 13. 已知限制与后续工作

- **左手专用**：URDF、关节名、`SENSOR_TO_LINK_MAP` 都是左手；右手需要复制资产与映射。
- **9 个触觉通道**：掌部 8 个通道未参与控制。
- **摩擦系数固定**：`mu_est` 是常量，未做在线估计。
- **滑移检测依赖微分**：传感器噪声会放大剪切率，需配合低通/阈值。
- **主控制器无温度保护**：建议后续加入 80 °C 停机逻辑。
- **无自动化测试**：目前靠硬件在环验证。

后续可以优先做：

1. 触觉零点的温漂补偿与自动重标定。
2. 在线估计 `mu` 与滑移预警（而不是固定阈值）。
3. 温度/电流/力矩安全层。
4. ROS 2 节点化，便于与视觉/机械臂联动。
5. 数据记录与回放（rosbag 或 HDF5）。

---

## 14. 上游与许可

- 上游 Aero Hand Open：<https://github.com/TetherIA/aero-hand-open>，Apache-2.0。
- 本模块是在上游 SDK 之上新增的自研控制代码；上游组件遵循其各自仓库的许可证。
- 本地对上游的改动与补丁见 [`third_party/README.md`](../../third_party/README.md) 和 [`third_party/patches/`](../../third_party/patches/)。
