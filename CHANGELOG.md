# Changelog

本文件记录 Loophand 的显著变更。格式参考 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/)，版本号遵循语义化版本。

> 0.1.0 为研究原型：仅支持左手 Aero Hand Open；主控制器未包含舵机温度保护；暂无自动化测试。

## [0.1.0] - 2026-10-27

首个整理后的版本：把触觉剪切力抓取控制作为项目主线，Aero Hand Open 作为无 `.git` 的第三方代码快照。

### Added

- **触觉剪切力抓取控制器** `src/loophand/controller/shear_grasp_controller.py`
  - 100 Hz 控制循环；7 个主动关节、9 个触觉通道。
  - `IDLE → PRE_SHAPE → ACTIVE_GRASP` 状态机。
  - 触觉低通、剪切率、滑移比计算。
  - 基于摩擦锥的目标法向力：`Ft / μ × safety_margin`，并设置最小法向力。
  - 切向力二维 PID 与导纳控制器。
  - Pinocchio 阻尼最小二乘 IK，16 自由度映射到 7 个执行器。
  - 接触锚定、力误差死区、防回退棘轮、关节增量限幅。
- **触觉传感器驱动** `src/loophand/driver/tactile_driver.py`
  - GEN3 串口协议、28 通道识别、`AA56` 自动回传解析、LRC 校验。
  - 去皮标定、±2 原始计数死区。
- **控制基础设施**
  - `controller/pid.py`：二维有界 PID。
  - `controller/admittance.py`：带速度/加速度限幅的导纳控制器。
  - `controller/simple_shear_controller.py`：无 GUI、无状态机的简化版。
  - `controller/hybrid_position_torque.py`：早期位置/力矩混合控制。
- **GUI 与可视化**
  - `app/main_gui.py`：PyQt5 总控界面、串口选择、启动/抓取按钮、力与关节曲线。
  - `viz/hand_viz_3d.py`：URDF + STL 实时渲染、力箭头。
  - `viz/shear_debugger.py`：独立进程调参示波器，可调 PID/导纳参数。
- **实验版本**
  - `experiments/filtered_control.py`：目标力/导纳输出平滑、接触迟滞、4 s 标定。
  - `experiments/variants/shear_grasp_controller_tare4s.py`：4 s 标定的历史快照。
- **资产与工具**
  - 左手 URDF 与 22 个 STL：`src/loophand/assets/`。
  - 触觉传感器调试工具：`tools/read_tactile_sensor.py`。
  - 启动与自检脚本：`scripts/run_gui.sh`、`run_sensor_monitor.sh`、`check_env.py`。
  - 浏览器演示：`docs/demos/jacobian_demo.html`、`hand_viz.html`。
  - `pyproject.toml`、`requirements/grasp.txt`、`.vscode/settings.json`。
- **文档**
  - 项目总览 `README.md`。
  - 自研模块完整文档 `src/loophand/README.md`。
  - 实验说明 `src/loophand/experiments/README.md`。
  - 第三方说明 `third_party/README.md`。

### Changed

- 项目从平铺目录重组为 `src/loophand` 包结构；所有本地导入改为 `loophand.*`。
- URDF、网格等资产统一通过 `loophand.paths` 定位，不再依赖当前工作目录。
- Aero Hand Open 上游代码移入 `third_party/aero-hand-open`，作为设备抽象与固件来源。
- 固件切换为 `LEFT_HAND`，并把单体 `.ino` 拆分为 `firmware/main/firmware_v0.1.0/` 工程。
- 固件 `CTRL_TOR` 支持有符号力矩，负力矩可反向输出。
- SDK 示例串口改为本机设备名 `/dev/ttyACM0` / `/dev/ttyACM1`。
- 运行日志统一写入 `logs/`（`sensor_comm.log`、`force_error.log`）。
- 3D 可视化改用系统临时 URDF，加载完成后自动删除。
- editable SDK 的 `.pth` 指向新的 `third_party/aero-hand-open/sdk/src`。

### Removed

- 移除 VR 遥操作模块 `src/loophand_teleop/`、`requirements/teleop.txt` 及 `pyproject.toml` 中的 `teleop` 可选依赖。
- 移除 Aero Hand Open 上游 `.git` 元数据（约 272 MB），保留普通代码快照。
- 移除重复的 `meshes/`、`urdf/` 资产目录。
- 移除 `reverse/` 中与主版本逐字节相同的 `main_gui.py`、`shear_control.py`、`shear_debugger.py`。
- 移除实验变体中残留的个人笔记注释。

### Fixed

- 重组后模块之间的旧式平铺导入（`from tactile_driver import ...` 等）。
- 3D 视图中 STL 网格路径解析失败的问题。
- 持久化 `temp_viz_fixed.urdf` 残留问题。
- GUI 与后台进程启动时的资源路径与 `spawn` 兼容问题。

### Notes

- 已知限制：左手专用、固定摩擦系数估计、滑移检测依赖微分、主控制器无温度保护、无自动化测试。
- 上游许可证：固件/SDK 为 Apache-2.0；硬件设计文件为 CC BY-NC-SA 4.0（仅限非商业）。
- 本地自研部分尚未声明开源许可证。

[0.1.0]: https://github.com/loopkok/Loophand/releases/tag/v0.1.0
