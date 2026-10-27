# Loophand

**基于 Aero Hand Open 的触觉剪切力抓取控制研究项目。**

Loophand 以 Aero Hand Open 左手为硬件平台，用指端三维触觉传感器感知切向力变化与滑移趋势，通过 **剪切率 PID + 导纳控制 + 欠驱动逆运动学** 实时调节抓取力。项目的核心是 `src/loophand` 中的自研控制模块；Aero Hand Open 的上游代码（硬件、固件、SDK、ROS 2）以无 `.git` 的普通目录快照放在 `third_party/` 中，作为第三方依赖使用。

> 当前状态：研究原型，面向左手硬件。首次实验请空载验证，详见[安全注意事项](src/loophand/README.md#11-安全注意事项)。
>
> 变更记录：[CHANGELOG.md](CHANGELOG.md)

---

## 1. 项目解决什么问题

传统机械手抓取通常只做位置闭环：手指闭合到预设角度，无法判断物体是否正在滑落。Loophand 把触觉引入闭环：

- **感知**：每个触觉通道输出三维力 `(Fx, Fy, Fz)`，从中提取切向力 `Ft`、剪切率 `dFt/dt` 和滑移比 `Ft/|Fn|`。
- **决策**：根据摩擦锥估算维持抓取所需的最小法向力，并在检测到滑移时额外加力。
- **执行**：导纳控制器生成笛卡尔微位移，经阻尼最小二乘 IK 分配到 7 个主动关节。
- **保护**：接触前定速闭合、接触后锚定；法向力不足时禁止回退；关节增量限幅。

核心模块的算法、参数、状态机和调优方法见 **[src/loophand/README.md](src/loophand/README.md)**。

---

## 2. 系统架构

```text
┌──────────────────────────────────────────────────────────────┐
│  应用层     PyQt5 总控界面 / 3D 可视化 / 独立调参示波器        │
├──────────────────────────────────────────────────────────────┤
│  控制层     剪切力抓取控制器（状态机 + PID + 导纳 + IK）        │  ← 自研核心
├──────────────────────────────────────────────────────────────┤
│  感知层     GEN3 触觉传感器串口驱动（去皮、死区、自动回传）     │  ← 自研
├──────────────────────────────────────────────────────────────┤
│  抽象层     aero-open-sdk（关节/力矩/温度/电流接口）            │  ← 第三方
├──────────────────────────────────────────────────────────────┤
│  硬件层     Aero Hand Open 左手 + 固件 + GEN3 触觉传感器        │  ← 第三方
└──────────────────────────────────────────────────────────────┘
```

数据流：100 Hz 控制循环读取关节角与触觉力 → 状态机/力闭环 → `set_joint_positions()`；同时以 30 Hz 推送到 GUI 队列。

---

## 3. 仓库结构

```text
loophand/
├── README.md                     # 本文件：项目总览
├── pyproject.toml                # 包元数据与可选依赖
├── requirements/
│   └── grasp.txt                 # 抓取控制依赖
├── src/
│   └── loophand/                 # ★ 自研触觉剪切力抓取模块
│       ├── README.md             #   ← 完整模块文档
│       ├── app/                  #   PyQt5 总控界面
│       ├── controller/           #   主控制器、PID、导纳、早期混合控制
│       ├── driver/               #   GEN3 触觉串口驱动
│       ├── viz/                  #   3D 手模型、调参示波器
│       ├── experiments/          #   实验性变体与历史快照
│       └── assets/               #   左手 URDF + 22 个 STL
├── tools/
│   └── read_tactile_sensor.py    # 触觉传感器独立调试工具
├── docs/
│   └── demos/                    # 浏览器端演示（three.js / Jacobian）
├── scripts/
│   ├── run_gui.sh                # 启动总控界面
│   ├── run_sensor_monitor.sh     # 启动传感器监视器
│   └── check_env.py              # 运行时依赖自检
├── third_party/
│   ├── README.md                 # 上游说明、本地补丁、维护策略
│   ├── patches/
│   │   └── aero-hand-open-local.patch
│   └── aero-hand-open/           # 上游代码快照（无 .git）
├── logs/                         # 运行日志（gitignore）
└── .vscode/settings.json
```

### 模块索引

| 模块 | 说明 | 文档 |
| --- | --- | --- |
| `src/loophand` | 触觉剪切力抓取控制（项目核心） | [README](src/loophand/README.md) |
| `third_party/aero-hand-open` | 上游硬件/固件/SDK/ROS 2 | [README](third_party/README.md) |
| `tools/read_tactile_sensor.py` | 触觉协议调试 | — |
| `scripts/` | 启动与自检脚本 | — |

---

## 4. 硬件与软件要求

**硬件**

- Aero Hand Open 左手，7 个总线舵机。
- GEN3 触觉传感器（本项目使用 9 个通道：拇指 1、其余四指各 2）。
- 两个 USB 串口：手默认 `/dev/ttyACM1`，触觉默认 `/dev/ttyACM0`，均为 921600 baud。

**软件**

- Linux，Python 3.10（本机验证环境：conda `tehandcontrol`）。
- `numpy`、`pyserial`、`PyQt5`、`pyqtgraph`、`trimesh`、`scipy`。
- `pinocchio`（ROS 2 Humble 或 conda 提供）。
- `aero-open-sdk`（以 editable 方式安装 `third_party/aero-hand-open/sdk`）。

串口权限：

```bash
sudo usermod -aG dialout "$USER"
# 重新登录后生效
```

---

## 5. 快速开始

```bash
cd ~/Documents/loophand

# 1. 环境（按本机实际情况）
conda activate tehandcontrol
source /opt/ros/humble/setup.bash

# 2. 安装手部 SDK 与 GUI 依赖
pip install -e third_party/aero-hand-open/sdk
pip install -r requirements/grasp.txt

# 3. 自检
python scripts/check_env.py

# 4. 启动总控界面
./scripts/run_gui.sh
```

GUI 操作顺序：选择串口 → **START SYSTEM**（触觉开始 2 s 标定，此时不要触碰传感器）→ **ACTIVATE GRASP**（预姿态 → 接触 → 力闭环）。

### 常用命令

| 任务 | 命令 |
| --- | --- |
| 总控界面 | `./scripts/run_gui.sh` |
| 主控制器（无 GUI） | `PYTHONPATH=src python -m loophand.controller.shear_grasp_controller` |
| 简化版控制器 | `PYTHONPATH=src python -m loophand.controller.simple_shear_controller` |
| 触觉传感器调试 | `./scripts/run_sensor_monitor.sh` |
| 环境自检 | `python scripts/check_env.py` |

---

## 6. 上游仓库与本地改动

上游 [TetherIA/aero-hand-open](https://github.com/TetherIA/aero-hand-open) 固定在提交 `e2060cf`（2025-11-07），完整保留在 `third_party/aero-hand-open`。上游 `.git` 已移除，该目录作为普通代码快照随主项目管理。

本地对该仓库的改动包括：

- 固件切换为 `LEFT_HAND`，并把单体 `.ino` 拆分为 `firmware/main/firmware_v0.1.0/`；
- 固件 `CTRL_TOR` 支持有符号力矩（负力矩可反向）；
- SDK 示例改用 `/dev/ttyACM0` / `/dev/ttyACM1`。

可复现的补丁文件：[third_party/patches/aero-hand-open-local.patch](third_party/patches/aero-hand-open-local.patch)（不包含新增的固件目录与 `ros2/src/sdk/` 副本）。细节与更新策略见 [third_party/README.md](third_party/README.md)。

---

## 7. 开发约定

- **源码布局**：所有 Python 包位于 `src/`；未安装时使用 `PYTHONPATH=src` 运行。
- **资源路径**：URDF 与网格统一通过 `loophand.paths` 获取，不在业务代码里写相对路径。
- **上游不改**：对 `third_party/` 的修改应尽量少，并同步更新补丁文件；自研功能放在 `src/`。
- **第三方快照**：`third_party/aero-hand-open` 不含 `.git`，升级方式见 [third_party/README.md](third_party/README.md)。
- **日志**：运行时日志写入 `logs/`（已在 `.gitignore` 中忽略）。
- **包导入**：`loophand.*` 从 `src/` 解析；VSCode 已配置 `python.analysis.extraPaths`。

---

## 8. 安全

1. 首次运行让手悬空，不要直接抓硬物或接触人体。
2. 触觉标定期间不要触碰传感器。
3. 主控制器没有舵机温度保护；长时间堵转请监控温度或断电。
4. 异常时使用 `Ctrl-C`（主循环会把执行器置 0）或直接断电。
5. 修改 `mu_est`、`min_force`、`safety_margin` 后先低速验证。

---

## 9. 后续工作

- 触觉温漂补偿与自动重标定。
- 在线摩擦系数估计与更鲁棒的滑移预警。
- 温度/电流/力矩安全层。
- ROS 2 节点化，接入视觉与机械臂。
- 数据记录与回放。

---

## 10. 许可与致谢

- **Aero Hand Open**：软件（固件与 SDK）为 Apache-2.0；硬件设计文件与文档为 CC BY-NC-SA 4.0（仅限非商业）。商用集成已购买单元请遵循上游 [LICENSE.md](third_party/aero-hand-open/LICENSE.md)。
- **dex-retargeting、Pinocchio、PyQt5、pyqtgraph、trimesh** 等依赖遵循各自许可证。
- 本仓库自研部分目前未声明开源许可证；如需对外发布，请补充 `LICENSE`。
- 感谢 TetherIA 开源 Aero Hand Open 硬件与 SDK。
