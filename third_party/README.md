# 第三方依赖：Aero Hand Open

本目录存放 [TetherIA/aero-hand-open](https://github.com/TetherIA/aero-hand-open) 的上游代码快照（提交 `e2060cf`），作为 Loophand 的硬件抽象与固件来源。上游 `.git` 已移除，整个目录按普通文件管理。自研控制代码不放在这里。

```text
third_party/
├── README.md                              # 本文件
├── patches/
│   └── aero-hand-open-local.patch         # 本地改动的可复现补丁
└── aero-hand-open/                        # 上游代码快照（无 .git）
    ├── firmware/                          # ESP32 固件
    ├── hardware/                          # CAD / STEP / STL / BOM
    ├── ros2/                              # ROS 2 包（上游）
    ├── sdk/                               # Python SDK（aero-open-sdk）
    └── LICENSE.md
```

---

## 1. 版本

| 项 | 值 |
| --- | --- |
| 上游仓库 | `https://github.com/TetherIA/aero-hand-open` |
| 固定提交 | `e2060cfbc609c3a969a627121739e3ed989d89fa` |
| 提交日期 | 2025-11-07 |
| 提交说明 | `update hardware (for latest pcb) and simplify readme (#26)` |
| 本地状态 | 上游 `e2060cf` 的普通目录快照；本地改动见下文；`.git` 已移除 |

---

## 2. 本地对上游的改动

以下改动是相对上游 `e2060cf` 的记录（在移除 `.git` 前通过 `git status` / `git diff` 整理），共涉及 11 个上游已跟踪文件和 2 个新增目录。

### 2.1 上游已跟踪文件

| 文件 | 改动 | 目的 |
| --- | --- | --- |
| `firmware/main/HandConfig.h` | `RIGHT_HAND` → `LEFT_HAND` | 适配左手硬件 |
| `firmware/main/firmware_v0.1.0.ino` | 删除（566 行） | 拆分为 `firmware_v0.1.0/` 目录工程 |
| `firmware/main/firmware_v0.1.0/firmware_v0.1.0.ino` | 新增（未跟踪） | 拆分后的主固件 |
| `firmware/main/firmware_v0.1.0/*.cpp/.h` | 新增（未跟踪） | 舵机协议、回零、配置等模块 |
| `ros2/NOTICE` | 追加一条启动命令 | 记录本机 ROS 2 运行方式 |
| `ros2/.../mediapipe_mocap.py` | `ema_alpha` 0.7 → 0.3 | 降低上游手势跟踪抖动 |
| `ros2/.../mediapipe_retargeting.py` | 拇指 4 个角度 ×1.5；打印关节角 | 补偿拇指行程 |
| `sdk/examples/get_info.py` | 串口改为 `/dev/ttyACM1` | 本机设备名 |
| `sdk/examples/joint_control.py` | 串口改为 `/dev/ttyACM0` | 本机设备名 |
| `sdk/examples/position_torque_switching.py` | 串口改为 `/dev/ttyACM0` | 本机设备名 |
| `sdk/examples/run_sequence.py` | 轨迹精简为食指动作并循环；Ctrl-C 张开 | 单指测试 |
| `sdk/examples/torque_control.py` | 力矩上限 1000 → 400；注释打印 | 安全限幅 |
| `sdk/src/aero_open_sdk/aero_hand.py` | 仅文件模式 644 → 755 | 无代码改动 |

### 2.2 新增目录（上游未跟踪）

| 目录 | 说明 |
| --- | --- |
| `firmware/main/firmware_v0.1.0/` | 拆分后的 Arduino 工程，含 `build/`（约 24 MB，已在根 `.gitignore` 中忽略） |
| `ros2/src/sdk/` | SDK 的本地副本（带 `.gitrepo`，来自 `git-subrepo`）；示例文件与顶层 `sdk/` 不完全一致，目前未被自研模块使用 |

> 建议：把 `firmware_v0.1.0/build/` 清理掉再纳入版本管理；确认 `ros2/src/sdk/` 的用途后删除或合并，避免两份 SDK 漂移。

### 2.3 固件有符号力矩补丁

`firmware/main/firmware_v0.1.0/firmware_v0.1.0.ino` 中 `CTRL_TOR (0x12)` 的处理已改为：

- 从两字节负载解析 **int16 有符号**力矩；
- 限幅到 `[-1000, 1000]`；
- 按舵机安装方向 `sgn` 映射后写入；
- 负值表示反向力矩。

上游原实现只接受无符号幅值，无法反向输出。该补丁是 `controller/hybrid_position_torque.py` 等力矩模式实验的前提；主抓取控制器使用位置模式，不依赖它。

---

## 3. 补丁文件

[`patches/aero-hand-open-local.patch`](patches/aero-hand-open-local.patch) 共 803 行，是在移除 `.git` **之前**由 `git diff` 生成的本地改动记录：

```bash
# 历史命令，当前目录已无 .git，不能直接重新执行
git -C third_party/aero-hand-open diff > third_party/patches/aero-hand-open-local.patch
```

**包含**：上表中所有上游已跟踪文件的修改与删除。

**不包含**：

- 新增的 `firmware/main/firmware_v0.1.0/` 整个目录；
- 新增的 `ros2/src/sdk/`；
- 文件模式变化之外的空 diff（`aero_hand.py` 的模式变化会包含在补丁里）。

因此该补丁只能把**上游已跟踪文件**恢复到当前状态；拆分后的固件工程需要单独备份。

如需校验或复现，先把上游 `e2060cf` 克隆到临时目录，再应用补丁：

```bash
git clone https://github.com/TetherIA/aero-hand-open /tmp/aero-hand-open
git -C /tmp/aero-hand-open checkout e2060cf
cd /tmp/aero-hand-open
git apply --check /path/to/loophand/third_party/patches/aero-hand-open-local.patch
git apply /path/to/loophand/third_party/patches/aero-hand-open-local.patch
```

补丁不会创建 `firmware/main/firmware_v0.1.0/`，该目录需从本项目当前快照中单独复制。

---

## 4. SDK 安装

Loophand 通过 editable 安装使用这里的 SDK：

```bash
pip install -e third_party/aero-hand-open/sdk
```

当前机器的 `~/.local/lib/python3.10/site-packages/aero_open_sdk.pth` 指向：

```text
/home/loopkok/Documents/loophand/third_party/aero-hand-open/sdk/src
```

如果目录再次移动，需要同步更新该 `.pth`，或重新执行上面的安装命令。SDK 的构建后端是 `uv_build`；离线环境可先 `pip install uv_build`，再使用 `--no-build-isolation`。

验证：

```bash
python -c "import aero_open_sdk; print(aero_open_sdk.__file__)"
```

---

## 5. 版本管理与更新上游

上游 `.git` 已移除，`third_party/aero-hand-open` 作为**普通目录快照**随主项目一起管理，不再使用 submodule 或 subtree。这样做的好处是：

- 项目根目录可以正常 `git init`，不会出现 embedded repository / gitlink；
- clone 主项目即可拿到全部第三方代码，不依赖额外初始化；
- 代价是失去上游提交历史，升级需要手动对比。

升级流程：

```bash
# 1. 备份当前快照
tar -C third_party -czf /tmp/aero-hand-open-backup.tar.gz aero-hand-open

# 2. 克隆目标版本到临时目录
git clone https://github.com/TetherIA/aero-hand-open /tmp/aero-hand-open-new
git -C /tmp/aero-hand-open-new checkout <目标提交>

# 3. 对比本地改动（或把本地补丁应用到新副本）
diff -ru --exclude=.git /tmp/aero-hand-open-new third_party/aero-hand-open | less
```

确认差异后，把新版本覆盖到 `third_party/aero-hand-open`，再重新施加本地改动：

1. `firmware/main/HandConfig.h` 切换为 `LEFT_HAND`；
2. 保留 `firmware/main/firmware_v0.1.0/` 与有符号力矩处理；
3. SDK 示例串口改回本机设备名；
4. 重新生成/更新补丁文件与本文档中的提交号。

升级后重点回归：

- `HandConfig.h` 的手别；
- 固件串口协议（`CTRL_TOR` 是否仍有符号处理）；
- SDK 的 `AeroHand` 接口签名；
- URDF 关节名与 `src/loophand/assets/` 是否仍一致。

> 如果以后希望恢复 Git 管理，建议在项目根目录统一 `git init`，把 `third_party/` 作为普通文件提交；不要在上游目录里重新 `git clone` 出带 `.git` 的嵌套仓库。

---

## 6. 许可证

上游采用分层许可（详见 [`aero-hand-open/LICENSE.md`](aero-hand-open/LICENSE.md)）：

| 内容 | 许可证 | 说明 |
| --- | --- | --- |
| 固件与 SDK | **Apache-2.0** | 允许商业使用，需保留版权与许可声明 |
| 硬件设计文件与文档（CAD/STEP/STL、图纸、BOM） | **CC BY-NC-SA 4.0** | 仅限非商业；衍生作品需署名并以相同许可共享 |
| 购买到的实体手 | 销售条款 + 商业集成许可 | 可集成进商业产品，但不授予用设计文件量产的权利 |

在本项目中：

- 使用 SDK / 固件时保留上游 `LICENSE`、`NOTICE`；
- 不要把 `hardware/` 中的设计文件用于商业制造；
- 自研代码的许可证由项目作者另行决定（当前根目录尚未声明）。
