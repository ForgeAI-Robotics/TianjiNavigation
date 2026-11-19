# 天机 × 灵猴 TX-S2 移动机器人底盘模组

本包对应现用的 **Jetson CUDA 点云滤除 + 灵猴 TX-S2 底盘侧导航执行**。桌面控制台用于规划预览、点云和执行监控，Agent 通过 Jetson 的 HTTP 网关调用导航。两条入口共用底盘侧的任务接口和账本。

```text
桌面控制台 ──────────────────┐
Agent → Jetson HTTP 网关 ─────┴→ 底盘导航总线 → 底盘任务代理 → 定位、Hybrid A*、跟踪和停车
                                               ↓
                                   Jetson CUDA 点云滤除服务
                                               ↓
                                     底盘实时守卫和运动准入
```

Jetson 负责点云坐标变换、URDF 自体滤除和近场滤除，CUDA 用于 OBB 距离计算。底盘侧负责定位、规划、运动安全和控制。停机与到位以底盘侧返回的原命令证据为准。

底盘为苏州灵猴机器人有限公司的 **TX-S2 移动机器人底盘模组**。仓库路径中的 `s100` 是现有部署标识。[厂商产品页](https://www.linkhou.com/proInfo/225.html) · [厂商型号说明](https://www.linkhou.com/newsInfo/44.html)

## 能力索引

查阅 [Agent 导航 Skill 索引](skills/README.md)，按导航、状态查询和取消找到现有调用入口、成功判定和无运动测试方式。控制台沿用同一套底盘导航服务。

## 目录

| 路径 | 用途 |
| --- | --- |
| `jetson/navigation_bus_refactor_v1_20261116/` | Agent HTTP 网关、公开范围和转发代码 |
| `jetson/navigation_waypoints_v6/` | 桌面控制台、规划图、任务状态和日志 |
| `jetson/navigation_jetson_v5/`、`jetson/navigation_link_v2/` | 当前控制台引用的地图读取和三维点云部件 |
| `jetson/navigation_filter_offload_v1/` | Jetson 滤除服务、CUDA 源码和当前工作进程所用的冻结源码 |
| `s100/navigation_bus_refactor_v1_20261116/` | 内部总线、导航模块、GUI 任务代理和实际候选执行目录 `gui_runtime` |
| `s100/navigation_s100_v7/` | 当前任务 API、定位、规划、跟踪和守卫；总线执行入口引用这里的共享代码 |
| `config/examples/`、`tools/` | 无凭据配置模板、CUDA 包版本清单和校验工具 |

保留原目录名是为了兼容现用路径。Jetson 目录按原结构部署到 Jetson 工作区，`s100/` 目录按原结构部署到底盘侧工作区。现用脚本仍引用这两台设备的原安装路径，改目录时需同步调整服务单元、脚本和本地配置。原厂 SDK、ROS 2 Humble/Cyclone DDS、标定与碰撞配置、现场地图及编号点位由部署环境提供。仓库只包含导航源码及必要的现用模型源码，现场地图、运行日志、令牌和账户配置应通过私有部署路径补齐。

## 部署方式

| 方式 | 模块分工 | 验证状态 |
| --- | --- | --- |
| CUDA 设备与其他设备协同 | Jetson 执行 CUDA 点云滤除并提供导航网关。底盘侧负责定位、规划、跟踪和停车核验。 | 当前部署基线 |
| 单 CUDA 设备 | 感知、规划和执行服务部署在一台 CUDA 主机，通过现有传感器和控制器接口工作。 | 待验证。沿用同一导航 API，需复验并发负载、控制周期和停车反馈延迟。 |

## 部署与启动

1. **准备现场配置。** 从 `config/examples/` 复制对应模板至组件的 `config/`。还原设备私有的 SSH 主机指纹、连接配置、令牌以及办公室地图、局部场景和点位配置。控制台还需要原 Jetson 的 `navigation_jetson_v5/config/config.json`。令牌文件权限设为 `0600`。桌面控制台的 Brain 监控使用单独的只读令牌。
2. **Jetson CUDA。** JetPack 需要提供 CUDA 工具链。用现用安装的私有 CUDA 包补齐标定、碰撞配置和本机构建件。先校验，确认无误后去掉 `--check`，再部署到设备的原滤除目录。
   ```bash
   python3 tools/restore_cuda_bundle.py --check /path/to/existing/navigation_filter_offload_v1/bundle
   cd jetson/navigation_filter_offload_v1
   nvcc -O3 -shared -Xcompiler -fPIC cuda/obb.cu -o cuda/libobb.so
   ```
   Jetson 侧使用 `config/service.json` 和用户服务 `tianji-navigation-filter-offload.service`，底盘侧用 `config/filter_compute.json` 指定 `mode=jetson` 与 `backend=cuda_obb`。两端的包版本、模型和配置摘要必须匹配。
3. **底盘侧导航总线。** 准备 `gui_runtime/deployment.json`、场景配置及其原部署链接，构建底盘侧原生组件。内部总线监听 `127.0.0.1:18769`，上游任务代理用 `127.0.0.1:18768`。迁移时先用只读模板联通状态。现场正式启用时，在底盘侧的模块化导航目录运行原 `./start_gui_services.sh`，该脚本会启用导航接单能力。
4. **Jetson 网关和控制台。** 补齐 Jetson `config/jetson_gateway.json` 后，启动对应用户服务 `tianji-navigation-brain-gateway.service`。控制台安装 PyQt5、PyOpenGL 和 Paramiko，在 Jetson 的 `navigation_waypoints_v6` 目录运行 `python3 modules/navigation_gui.py`。调试模式选 `bus`，让任务走底盘侧模块化总线。

## Agent 导航接口

**交付范围为 Agent 对接接口与需求。** Agent 应用、任务编排和模型由对接方实现。对接方通过网关调用现有导航服务，两种部署方式沿用同一导航合同。

Agent 使用 **Jetson 网关地址**和网关的 Bearer 令牌。调用前读取服务状态和场景图，核对接口版本、当前地图和支持的运动模式。现场编号目标以场景图中的 `office_waypoint_1`、`office_waypoint_2`、`office_waypoint_3` 为准。

| 接口 | 用途 |
| --- | --- |
| `GET /health`、`GET /v1/status` | 读健康状态、当前命令与准入状态 |
| `GET /total_scene_graph_latest.json` | 读场景绑定、点位和公开导航合同 |
| `POST /v1/navigation/goals` | 提交导航，使用 `fq/reception-lan/v1` 合同、地图绑定和原始命令身份 |
| `GET /v1/commands/{command_id}` | 查询原命令的执行、到位和停车证据 |
| `POST /v1/navigation/cancel` | 按原 `task_id` 和 `command_id` 取消 |

接口字段见底盘侧的 [OpenAPI 定义](s100/navigation_s100_v7/config/task_api_openapi.json)和 [Jetson 网关合同](jetson/navigation_bus_refactor_v1_20261116/API_CONTRACT.txt)。操作分类见 [Agent 导航 Skill 索引](skills/README.md)。

对接方提交前保存 `task_id` 和 `command_id`，并记录目标及地图绑定。区分命令受理、到位和停车确认。提交或取消回包丢失时，查询原命令；取消受理后继续核对停车证据。首轮验收使用只读查询和离线合同测试，确认身份、状态和取消语义。

## Git

交付包含一个本地 Git 仓库和同一提交的 ZIP。确认目标仓库后，在本地 Git 目录执行：

```bash
git remote add origin '填写你的仓库地址'
git push -u origin main
```
