# 实验与历史版本

本目录保存**不参与主流程**的控制器变体，用于对比实验和回归参考。默认入口是 [`../controller/shear_grasp_controller.py`](../controller/shear_grasp_controller.py)。

---

## 1. `filtered_control.py`

在主控制器基础上增加三层平滑/迟滞，用于抑制接触瞬间和力指令的抖动：

| 改动 | 主控制器 | 实验版本 |
| --- | --- | --- |
| 触觉标定 | 2.0 s | **4.0 s** |
| 目标法向力 | 直接使用瞬时值 | 一阶低通 `alpha = 0.1` 后使用 |
| 导纳输出 `dx` | 直接使用 | 一阶低通 `alpha = 0.4` 后使用 |
| 接触判断 | `Fn > 0.1 N` | 迟滞：`ON = 0.1 N`，`OFF = 0.03 N` |
| PID 初值 | `0.1 / 0.0 / 0.0` | `8.0 / 0.5 / 0.0` |
| 导纳 | `M=0.11, D=12` | `M=0.11, D=12`（每指独立） |

运行：

```bash
PYTHONPATH=src python -m loophand.experiments.filtered_control
```

适合场景：触觉噪声较大、接触瞬间力目标跳变明显。代价是响应变慢。

---

## 2. `variants/shear_grasp_controller_tare4s.py`

主控制器在某个时间点的快照，**唯一差异**是触觉标定时间 `4.0 s`（主控制器为 `2.0 s`），并保留当时的注释风格。

运行：

```bash
PYTHONPATH=src python -m loophand.experiments.variants.shear_grasp_controller_tare4s
```

用途：复现早期 4 秒标定的实验数据。不要在这里继续开发新功能，改动应回到主控制器。

---

## 3. 维护约定

- 实验代码可以复制主控制器，但**不要**让主控制器反向依赖本目录。
- 新增变体时，在本文件补充：与主控制器的差异、运行命令、适用场景。
- 若某个变体被验证更优，应把改动合并回 `controller/shear_grasp_controller.py`，而不是直接在主流程引用实验文件。
- 本目录代码不保证与主控制器同步，接口可能随主控制器演进而失效。
