# CPS5 AUV Companion

基于 CPS5ROV 平台改造的小型混合式自主／遥控水下航行器伴随计算机软件。系统以树莓派 4B 为上层计算平台，通过 MAVLink Router 与 Pixhawk/ArduSub 交互，并通过 TCP 接入 DVL 局部位置数据，用于遥测监视、设备联调和按目标深度、航向、相对距离执行分阶段航行任务。

> 当前仓库主要保存 Python 上层程序、示例配置、自动化测试以及由原项目资料整理出的硬件和部署说明。代码整理结果不等同于实艇验证；接线、解锁和下水测试前，请以实际固件、接线、坐标系和安全检查结果为准。

## 主要功能

- 通过 Pymavlink 接收飞控心跳、航向和高度相关遥测。
- 通过 TCP 接收 DVL 数据，并按 `CRLF` 边界解析 JSON 格式的局部位置报文。
- 使用 YAML 集中配置 MAVLink、DVL、控制参数和执行安全门。
- 使用 PID、航段数据模型和状态机组织深度、航向与相对距离控制流程。
- 提供只读监视入口 `cps5-monitor`，不会发送模式切换、解锁或执行器命令。
- 提供任务控制入口 `cps5-control`，默认仅进行离线 dry-run；真实执行受配置、操作员确认、交互口令和传感器时效检查共同限制。
- 在深度、航向或 DVL 数据过期，以及航向或前进超时时进入故障状态并请求中性输出。
- 使用 Pytest 覆盖配置校验、DVL 分帧、PID、状态机、执行适配器和安全门等核心逻辑。

## 硬件与软件环境

| 项目 | 配置 |
| --- | --- |
| 伴随计算机 | Raspberry Pi 4B，Ubuntu 24.04 |
| 飞控 | Pixhawk 2.4.8，部署适配 CPS5 的 ArduSub 固件 |
| 地面站 | QGroundControl |
| 定位设备 | DVL，经以太网提供 TCP/JSON 局部位置数据 |
| 飞控链路 | Pixhawk USB 虚拟串口 → MAVLink Router → UDP |
| 上层程序 | Python 3.10 及以上 |
| 主要依赖 | Pymavlink、PyYAML、OpenCV |
| 开发方式 | VS Code Remote-SSH、命令行 |

以上内容来自当前代码与项目资料。实艇固件版本、最终网络地址、深度基准和 DVL 安装轴向仍需结合实际设备复核。

## 目录结构

```text
cps5-auv-companion/
├─ config/                     # 车辆与航段示例配置
├─ deployment/                 # MAVLink Router、systemd 部署说明（待完善）
├─ docs/
│  ├─ control-safety.md        # 真实执行前的安全门与操作要求
│  └─ source-materials/        # 原项目文字资料，仅作为整理依据
├─ examples/                   # 早期通信测试程序，仅供参考
├─ hardware/
│  ├─ README.md                # 硬件资料索引
│  └─ source-materials/        # 原始接线图与器件说明
├─ src/cps5_companion/
│  ├─ app.py                   # 只读监视程序
│  ├─ cli.py                   # cps5-monitor 命令行入口
│  ├─ config.py                # YAML 加载与配置校验
│  ├─ mavlink.py               # MAVLink 只读遥测接收
│  ├─ dvl.py                   # DVL TCP 客户端与 CRLF/JSON 分帧
│  ├─ models.py                # 遥测和位置数据模型
│  ├─ actuation.py             # 显式启用后的 MAVLink 输出适配器
│  ├─ safety.py                # 真实执行安全门
│  ├─ control_app.py           # 任务控制集成
│  ├─ control_cli.py           # cps5-control 命令行入口
│  ├─ control/
│  │  ├─ pid.py                # PID 控制器
│  │  ├─ mission.py            # 航段配置与状态定义
│  │  └─ controller.py         # 无硬件依赖的航段状态机
│  └─ dvltest1.py              # 保留的历史原型，禁止作为安全入口
├─ tests/                      # 自动化测试
├─ pyproject.toml              # Python 包、依赖与命令行入口
├─ README.md
└─ README_EN.md
```

## 接口分配

以下映射来自当前示例配置和历史通信资料。

| 链路 | 端点／参数 | 当前用途 |
| --- | --- | --- |
| Pixhawk → Router | `/dev/pix:1500000` | Router 访问飞控 USB 虚拟串口 |
| Router → QGC | `192.168.137.1:14550/UDP` | 向岸基地面站转发 MAVLink |
| Router → Python | `127.0.0.1:14553/UDP` | 向树莓派本机上层程序转发 MAVLink |
| Python 监听 | `udpin:0.0.0.0:14553` | Pymavlink 接收 Router 转发的消息 |
| Python → DVL | `192.168.137.3:16171/TCP` | 接收 `position_local` JSON 报文 |

历史资料中还出现过 `192.168.137.1:14551`，但其最终用途尚未确认；当前不将它解释为 DVL 端口。资料对 `192.168.137.3` 的归属也存在阶段性冲突，部署前必须以现场网络配置为准。

控制入口当前使用以下逻辑通道映射：

| 逻辑量 | MAVLink RC 通道 | 示例中性值 | 示例范围 |
| --- | --- | --- | --- |
| 升沉 | 3 | 1500 | 1100–1900 |
| 转向 | 4 | 1500 | 1100–1900 |
| 前进 | 5 | 1500 | 1100–1900 |

这些值是发送给 ArduSub 的 RC 输入，不是某个推进器的最终 PWM。通道对应关系、正负方向和限幅必须在拆桨或断开推进器动力的条件下单独验证。

## 软件流程

### 只读监视流程

1. 加载并校验车辆配置；可选加载任务文件检查航段格式。
2. 建立 MAVLink 接收连接并等待飞控心跳。
3. 按配置连接 DVL TCP 服务。
4. 分别更新航向、高度相关遥测和 DVL 局部位置。
5. 周期输出状态；全程不发送模式切换、解锁、RC 覆盖或姿态目标命令。

### 任务控制流程

1. 默认执行离线 dry-run，只校验配置并构造状态机。
2. 真实执行时依次检查 `--execute`、三项操作员确认、四项本地配置安全许可和交互口令。
3. 连接飞控与 DVL，并等待新鲜的航向、高度和位置数据。
4. 确认飞控处于配置模式后请求解锁。
5. 按航段状态机计算控制量并周期发送。
6. 正常完成、故障或退出时，依次请求中性输出、上锁和释放 RC 覆盖通道。

航段状态机如下：

| 状态 | 行为与转移条件 |
| --- | --- |
| `ADJUST_DEPTH` | 调整深度；误差进入阈值后转入航向调整 |
| `ADJUST_HEADING` | 调整目标航向；达到阈值后记录位置原点 |
| `CAPTURE_DVL_ORIGIN` | 保存当前 DVL x 坐标，不向 DVL 发送复位命令 |
| `MOVE_FORWARD` | 依据相对 x 位移执行定距前进 |
| `COMPLETE` | 当前任务完成 |
| `FAILED` | 数据过期、控制超时或运行异常，停止任务并请求中性输出 |

该状态机只实现给定深度、航向和相对距离的分阶段航行，不包含 SLAM、自主避障、全局路径规划或完整二维轨迹跟踪。

## 安装与运行

### 安装开发环境

```bash
python -m venv .venv
```

激活虚拟环境后安装项目及开发依赖：

```bash
python -m pip install -e ".[dev]"
```

### 校验示例配置

```bash
cps5-monitor --config config/vehicle.example.yaml \
  --mission config/mission.example.yaml --check-config
```

该命令不会连接网络，也不会向飞控发送命令。

### 启动只读监视

先复制本地配置并根据实际设备修改：

```bash
cp config/vehicle.example.yaml config/vehicle.local.yaml
cps5-monitor --config config/vehicle.local.yaml
```

`vehicle.local.yaml` 已被 Git 忽略，适合保存本机 IP、端口和已完成的物理验证结果。

### 运行任务 dry-run

```bash
cps5-control --config config/vehicle.example.yaml \
  --mission config/mission.example.yaml
```

该命令只加载配置并构造任务状态机，不建立网络连接。示例配置中的执行许可全部为 `false`，误加 `--execute` 也会在连接设备前被拦截。真实硬件执行流程见 [`docs/control-safety.md`](docs/control-safety.md)。

## 首次运行

1. 拆除桨叶或断开推进器动力，只给飞控、树莓派和传感器供电。
2. 核对 Pixhawk 设备路径、Router 端点、本机 IP 和 DVL 地址，排除历史地址冲突。
3. 复制示例配置为 `vehicle.local.yaml`，不要直接修改示例文件来绕过安全门。
4. 运行配置校验，确认所有字段和航段格式正确。
5. 启动 `cps5-monitor`，分别确认心跳、航向、高度相关遥测和 DVL 数据持续更新。
6. 在已知姿态和位移条件下核对深度正负号、DVL x 轴方向及单位。
7. 运行 `cps5-control` dry-run，检查状态机和任务参数。
8. 只有完成 RC 通道、深度映射和 DVL 轴向的独立验证后，才按照安全文档进行受控台架测试。

## 通信协议

### MAVLink

| 消息／调用 | 当前用途 |
| --- | --- |
| `HEARTBEAT` | 检查飞控在线状态与模式 |
| `VFR_HUD.heading` | 更新当前航向 |
| `AHRS2.altitude` | 提供当前代码使用的高度相关量 |
| `SET_MODE` | 真实控制入口请求模式切换 |
| `COMMAND_LONG` | 承载解锁／上锁命令 |
| `RC_CHANNELS_OVERRIDE` | 发送升沉、转向和前进输入 |
| `SET_ATTITUDE_TARGET` | 输出姿态目标 |

`AHRS2.altitude` 的标准含义不是天然的水深。示例配置保留了历史原型中的负号换算，但零点、方向及固件实际输出尚未完成实艇验证。

### DVL TCP/JSON

当前解析器从 TCP 字节流中累积数据，以 `\r\n` 分隔完整报文，并保留末尾未接收完整的部分。使用的局部位置报文形式如下：

```json
{"type":"position_local","x":0.0,"y":0.0}
```

TCP 不保留应用层消息边界，因此一次 `recv()` 可能得到半条、一条或多条报文。控制器使用每个航段开始时的 x 坐标作为相对原点，不再与接收线程竞争发送和读取 DVL 复位命令。

## 配置说明

| 配置项 | 作用 |
| --- | --- |
| `mavlink.endpoint` | Python 监听的 MAVLink 连接字符串 |
| `dvl.host`、`dvl.port` | DVL TCP 服务地址 |
| `dvl.forward_sign` | DVL 前向坐标符号 |
| `depth.scale`、`depth.offset_m` | 高度相关量到深度变量的换算 |
| `control.*_channel` | RC 逻辑通道映射 |
| `control.*_threshold_*` | 深度、航向和距离到达阈值 |
| `control.*_timeout_s` | 数据时效和状态超时限制 |
| `control.depth_*`、`control.forward_*` | PID／比例控制参数及输出限幅 |
| `safety.*` | 真实执行许可；仅在完成对应物理验证后设置 |

`config/mission.example.yaml` 中的每个 `segment` 表示“目标航向 + 相对距离 + 目标深度”，不是经纬度航点。历史原型中的横滚、转弯和拍照等附加动作尚未迁移；任务包含这些动作时会拒绝启动。

## 测试

```bash
python -m pytest
python -m ruff check src/cps5_companion tests \
  --exclude src/cps5_companion/dvltest1.py \
  --exclude src/cps5_companion/ph.py \
  --exclude src/cps5_companion/pai.py \
  --exclude src/cps5_companion/ticop.py
```

截至当前整理版本，本地测试结果为 `19 passed`。这些测试验证软件逻辑，不代替 SITL、台架或实艇测试。

## 常用修改位置

| 需求 | 文件 |
| --- | --- |
| 修改网络、通道和控制参数 | `config/vehicle.local.yaml` |
| 修改航段任务 | `config/mission.example.yaml` 或本地任务文件 |
| 修改配置结构与校验规则 | `src/cps5_companion/config.py` |
| 修改 MAVLink 遥测接收 | `src/cps5_companion/mavlink.py` |
| 修改 DVL 分帧与解析 | `src/cps5_companion/dvl.py` |
| 修改航段状态和转移条件 | `src/cps5_companion/control/controller.py` |
| 修改 PID | `src/cps5_companion/control/pid.py` |
| 修改 MAVLink 输出适配 | `src/cps5_companion/actuation.py` |
| 修改执行安全门 | `src/cps5_companion/safety.py`、`src/cps5_companion/control_cli.py` |
| 修改监视或控制集成 | `src/cps5_companion/app.py`、`src/cps5_companion/control_app.py` |

## 注意事项

- **不要直接运行 `dvltest1.py`。** 该历史原型在初始化阶段包含飞控连接和解锁操作，仅保留用于追溯原始实现。
- MAVLink Router 只负责转发消息，不会仲裁 QGroundControl、手柄和 Python 控制器之间的控制权；自动任务运行时不得同时发送相互竞争的驾驶输入。
- 心跳只证明飞控链路在线，不代表深度、航向、DVL 或执行器已经可用。
- 当前控制器依赖 DVL x 轴与航行前向一致；安装角度、符号和比例错误会直接影响定距控制。
- 软件请求中性输出、上锁和释放 RC 覆盖不能替代物理急停、断电措施和现场操作员。
- `examples/` 及 `docs/source-materials/` 中的内容来自不同整理阶段，地址、参数或行为可能与当前安全入口不同。
- 当前仓库尚未完成本轮真实设备连接、SITL、台架或水池验证，不应据此宣称控制精度、实时频率或航行性能。
