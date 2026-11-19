## 大脑兼容接口1.1（2026-11-10）

本侧新增fq/reception-lan/v1坐标导航兼容入口，原编号接口保留。代码版本1.1；当前运行实例和运动开关以GET /v1/capabilities为准。兼容路由：/health、/v1/status、/v1/world、/total_scene_graph_latest.json、POST /v1/navigation/goals、GET /v1/commands/{command_id}、POST /v1/navigation/cancel。

config/brain_compat.json将外部map明确定义为办公室固定session_base_start归档的同原点同轴坐标，绑定地图SHA，绝不逐任务重置原点。坐标目标写入原任务账本及密封任务级目标文件（内部ID0），不改1/2/3号点。command_id的幂等、查询、取消与内部任务ID一起持久化；服务重启不重放动作。到位<=5cm/1度，额外要求停车后的新位姿和四轮零速反馈证明；进程退出/发零命令不作为停止证据。

目前仅支持forward_path；lateral_path_aligned明确拒绝INVALID_MOTION_MODE。现场坐标标定和新的停止证据链尚未真机验收。相机强制放行与夹爪固定开启仍明确展示，不当成全传感器实测净空。

独立合同测试实例位于同级navigation_brain_compat_dev_20261110，127.0.0.1:18765，独立任务目录与令牌。模拟回包带simulated=true、physical_execution=false，不能用于真机。正式目录不带ISOLATED_CONTRACT_TEST_ROOT标记；不要在此创建该标记或混用隔离存储。正式服务默认仍只读，不因接口升级开启运动。

大脑192.168.31.144使用自身原子代码通过外部隔离测试，报告保存在本侧navigation_brain_compat_dev_20261110/experiments/brain_compat_20261110；对方源码、文档、配置保持只读，未在其服务器创建报告。下文为旧开发记录，当前兼容约定以本节和OpenAPI为准。

S100 owns navigation computation. Deployment: /home/sunrise/tianji_table_approach_dev/navigation_s100_v7.

## Agent任务API v1（2026-11-09）

Agent只提交目标编号，S100持有任务并执行原导航数据流。GUI增加“CLI调试（现有入口）/ S100任务API（与Agent共用）”切换，默认保留CLI调试。API和GUI使用同一个`onboard_navigation.py`；没有另一套简化规划器，也不把点云送往Agent计算。

```text
Agent / GUI(API调试)
  → HTTP任务接口（或SSH隧道）
  → S100任务管理、幂等、日志、取消
  → 原定位 / Hybrid A* / 跟踪 / 实时守卫 / 速度与停车链路
  ← 任务状态、等待原因、到位误差、结果和日志引用
Jetson后台状态发布 + 独立头部只读服务 → S100实测状态接收
```

### 启动与连接

S100用户服务名为`navigation-task-api.service`，项目启动脚本为`./api.sh`。默认监听`127.0.0.1:8765`，**默认只读，实机任务返回409/MOTION_DISABLED**；规划与不运动联调可使用。仅启动服务不会启动底盘。

正式使用实机任务时，由部署人员在保留现有人工急停条件的前提下启动：

```bash
# 先停默认只读实例，避免端口冲突；下行命令启用运动任务能力，但不会自动提交任务。
systemctl --user stop navigation-task-api.service
cd /home/sunrise/tianji_table_approach_dev/navigation_s100_v7
./api.sh --enable-motion --operator-estop-ready
```

持久运动模式可使用用户服务的ExecStart覆盖：先清空原ExecStart，再设为`%h/tianji_table_approach_dev/navigation_s100_v7/api.sh --enable-motion --operator-estop-ready`，随后重载并重启该用户服务。无人登录时也要求开机可用的部署，还需为对应账号配置systemd用户linger；本次没有修改操作系统登录策略。

服务不会把人工声明写成已接通硬件急停。现有相机/ShapeMask测试强制放行状态仍明确暴露在readiness和任务telemetry中；API不能通过请求参数新增放行或绕过关节/雷达/碰撞检查。

远端Agent可使用SSH转发，无需开放S100监听地址：

```bash
ssh -N -L 8765:127.0.0.1:8765 sunrise@192.168.1.10
```

也可显式使用`--host 0.0.0.0`监听局域网。鉴权为`Authorization: Bearer <token>`，令牌保存在S100的`config/task_api_token`，权限0600，日志不记录令牌。将令牌通过受控配置交给Agent，不要放入提示词或仓库。`/healthz`仅报告进程健康；业务接口均要求令牌。

### 最小调用与状态

```http
POST /v1/navigation/tasks
Authorization: Bearer <token>
Idempotency-Key: <为同一任务持久化的唯一请求编号>
Content-Type: application/json

{"goal": 1}
```

`goal`为1/2/3，默认办公室。可选`scene`为`office`/`outside`，对应场景必须已配置该点位；可选速度上限`translation_cap_mps`范围0.005～0.10，默认0.10，不修改转速。默认任务无额外总时限，遇障碍沿用原等待与恢复策略。可选`timeout_s`指定超时并停车退出。

响应202包含`id`、目标坐标/版本/地图哈希、任务类型和状态。Agent保存任务ID，通过`GET /v1/navigation/tasks/{id}`查询，或`GET .../{id}/events?after=序号&wait=1`长轮询。事件窗口有界，`truncated=true`时重新读取任务快照；完整日志仍持久化。最多一个任务占用导航，其他请求409/NAVIGATION_BUSY。同一个幂等键、相同请求返回原任务；同键不同请求409，避免网络重试重复执行。

状态含`ACCEPTED → STARTING → PLANNING / WAITING / RUNNING → SUCCEEDED / FAILED / CANCELED / INTERRUPTED`。等待净空不是任务失败；由原导航守卫判定等待与恢复。只有导航核心报告`ARRIVED`、停车收尾结束、误差≤5cm/1°且进程正常退出，才返回`SUCCEEDED`及`result.arrived=true`。规划通过是`PLAN_READY`，静态联调结束是`TEST_COMPLETE`，两者的`arrived`都为false，不能当成机器人已到位。

HTTP连接或Agent断线不会取消已受理任务。S100服务持有本地导航心跳；服务退出会请求停车，服务崩溃导致原导航stdin/心跳失效，原停车链路保留。服务重启将未完成记录标为`INTERRUPTED`，核对PID与启动标识后停止遗留的自有导航进程，**不自动重放运动任务**。取消用`POST .../{id}/cancel`；`CANCELING`表示已请求，等待`CANCELED`才表示任务进程已结束。`POST /v1/navigation/stop`取消当前API自有任务，GUI停车仍兼顾原CLI调试入口。

### Python客户端

将`sdk/tianji_navigation.py`交给Agent工程即可，依赖Python标准库，无需ROS、SDK或GUI：

```python
from tianji_navigation import NavigationClient

nav = NavigationClient("http://127.0.0.1:8765", token)
# request_id先保存到Agent任务记忆，网络重试复用，不为同一任务重新生成。
task = nav.navigate(2, idempotency_key=request_id)
result = nav.wait(task["id"])
if result["state"] == "SUCCEEDED":
    assert result["result"]["arrived"]
```

`examples/agent_navigation.py`提供完整CLI示例。`nav.wait(..., timeout=60)`只是客户端等待超时，不会隐式取消S100任务；可继续查询或显式取消。后台服务未启用运动时可先调用`nav.plan(2)`验证接入。

### 调试、日志与回滚

| 接口 | 用途 |
|---|---|
| GET `/v1/capabilities` | 版本、只读/运动模式、导航归属 |
| GET `/v1/readiness` | 默认场景的原接收状态与测试覆盖标记；不是运动授权 |
| GET `/v1/waypoints?scene=office` | 当前场景点位与版本 |
| POST `/v1/navigation/plans` | 只读规划，可显式传GUI的预览选项 |
| POST `/v1/navigation/tests` | 5～120秒输出隔离的完整链路联调 |
| GET `/v1/navigation/tasks` | 最近任务 |
| GET `/v1/navigation/tasks/{id}/plan` | 原规划器结果文件 |
| GET `/v1/openapi.json` | 完整机器接口描述 |

任务在启动导航进程前保存到`runtime/task_api/tasks/<id>/task.json`，事件为同目录`events.jsonl`。即使实时关节预检失败、尚未产生导航结果目录，错误及stderr尾部也会保存。请求受理/拒绝记录在`runtime/task_api/requests.jsonl`；原完整导航/电机/碰撞日志继续存放`results/...`，API返回相应引用。任务日志独立后台写入、状态节流，HTTP不参与高频点云与速度计算。不运动联调在原隔离模拟链路内使用CLI同样的联锁声明，并在结果中标记`isolated_test_operator_declaration=true`；不验证硬件急停，也不启用实机任务权限。

Jetson新增用户服务`tianji-navigation-head-state.service`，仅查询机械角度，独立于GUI生命期；已有串口所有者会被尊重，不发送使能或运动命令。双臂仍来自原实时状态桥；夹爪沿用当前显式固定开启配置，不伪装成电机反馈。

修改前快照：Jetson `snapshots/before_agent_api_20261109_172027/jetson_source_maps.tar.gz`；S100 `snapshots/before_agent_api_20261109_172030/s100_navigation.tar.gz`（均相对于`tianji_table_approach_dev`）。归档旁有SHA256清单。回滚先停止并禁用这两个新增服务，再恢复对应归档的源码/配置；旧任务记录不作为自动续跑指令。下文是较早版本的开发记录，当前能力以本节、配置与实际readiness为准。

Jetson only sends goal IDs and displays the returned path, and provides measured arm/camera data through the existing state bridge. No navigation point cloud is sent to Jetson for localization or planning. ROS reads, PCL GICP, Hybrid A*, continuous localization, follower preview and live whole-body collision calculations execute on S100. Original /home/sunrise/nav2 and other stacks are unchanged.

On S100, ./run.sh --goal 1 --assume-clearance creates a frozen planning preview. ./pipeline.sh --duration 15 runs local localization + path follower + live collision integration in read-only mode. No actuator interface is imported by these commands. The clearance assumption is only for frozen planning; live admission does not treat it as clearance. Capabilities explicitly inhibit real execution until fresh head/arm/gripper feedback, current camera clearance, whole-body guard and stop-chain integration are available.

Jetson entry points: Desktop 导航配置, 导航GUI.sh and 导航.sh. They invoke the pinned SSH command/display client, not a Jetson navigation engine. The old local development engine requires the explicit --local-dev flag.

Verification: S100 generated a checked ~1.4 m path to point 1 with no lateral segment. A 15 s local pipeline recorded continuous pose fixes and geometry checks, with zero motion messages. Head feedback is currently absent and camera clearance is inhibited; the GUI reports these states and keeps real execution disabled.


User-authorized forced release (2026-10-29): config/clearance_override.json explicitly enables camera-clearance and ShapeMask release for this map/URDF hash only. Raw receiver status is read-only and remains visible as raw_clearance_clear/raw_shapemask_clear. Effective state is FORCED_CLEAR_TEST, never a sensor-confirmed CLEAR. Source model/map changes invalidate the forced state. GUI test checkbox and onboard_client.py --force-release on/off manage the S100-side configuration; every change is audited. Localization, arm feedback, real LiDAR obstacles and link watchdogs are not bypassed. Real motion transport remains disabled. 28 onboard tests and a 15-second read-only integration run passed; zero motion commands.


Manual real execution (2026-10-29 later): GUI “启动实机导航（由你点击）” explicitly launches the S100 executor. It always performs fresh global localization and planning, starts continuous local positioning, checks current posture/target/map, and checks both static-map sweeps and live LiDAR sweeps. A real control bridge is lazy-created only after three consecutive valid admissions; initial invalid guards wait with zero output, and invalidity after admission latches a stop/exit requiring manual restart. The GUI operator heartbeat has a 0.5-second receiving-side timeout; decision age is separately rechecked in the local command pump and again in the S100 control process. The stop button and GUI closure request navigator shutdown. The user declared a physical emergency stop available; no hardwired estop signal is claimed verified.

The original speed ceilings are retained; initial execution dynamically limits translation to at most 0.03 m/s and additionally limits swept-surface motion since the oldest LiDAR frame. A target change, map hash change, pose/epoch error, expired link, collision or changed arm posture inhibits motion. The camera/ShapeMask test override remains explicit and audited and does not disable LiDAR or localization guards.

Validation never used --enable-motion: 31 onboard unit tests passed, the complete --preview session re-localized/replanned and exited normally with zero real motion commands, and a dedicated --preview control-protocol test admitted synthetic nonzero suggestions without ROS publishers, then verified expired decisions return zero. Native static-map guard tests accepted the clear origin and rejected an occupied obstacle. Current live near-body LiDAR ambiguity still prevents physical motion. Real motion has not been tested by the assistant; only the user may start it through the GUI.


Speed audit: original Nav2 MPPI config (/home/sunrise/nav2/configs/navigation/s2_nav2_params.yaml) has vx_max=0.20 m/s, vx_min=0, vy_max=0.04 m/s, wz_max=0.20 rad/s. New planner permits forward 0.20, reverse 0.10, lateral 0.04, angular 0.20, but the reused bridge has x/y caps 0.12 and angular cap 0.40. The new executor has an additional initial combined-translation cap of 0.03 m/s, with a stricter swept-motion budget based on cloud age and body radius. These are software caps, not motor rated maximum speeds. GUI now displays the actual initial execution cap. No speed settings were raised during this audit.

2026-10-30: manual ground and staged cable inspection

The Jetson navigation GUI now defaults to “先微移5cm识别线缆，再继续导航”. The read-only planning button requests a candidate route, with sparse near-body cable candidates temporarily deferred from that planning layer. External surfaces retain priority. This is explicitly labelled a candidate, not an executable free-space certificate. The static experiment that previously returned NoPath now produced a 2.40m differential route; its first 6cm were also independently checked against the original, un-deferred scene.

Only the real navigation start button invokes the staged execution: candidate route → locally checked inspection prefix → measured 5.2cm translation at 0.005m/s → stopped raw-cloud observation → motion-confirmed self mask → fresh full-scene replan/continuation to the same target. The 6cm bound includes stopping reserve. The older --execute-micro remains separately bounded to 5cm and does not start the full route. Per the user's later correction, approaching a suspected wire does NOT abort this first inspection move: a frozen geometric near-body wire region (up to 20cm from measured non-chassis parts, body height 0.6–1.5m) is provisionally withheld from the probe's soft obstacle test. Individual return positions can fluctuate without becoming new wire stops. This is an explicit inspection assumption, not confirmed self recognition. Independently supported external surfaces and obstacles outside that region retain their checks. Operator/pose/model/map/epoch/freshness and emergency-stop behavior remain active. The exemption is absent from normal route execution.

Evidence uses original, unfiltered scans and competing body-fixed/world-fixed predictions. After at least 5cm of measured displacement, the complete capture is replayed to include intermittent returns first seen during the move. Publishing a mask requires at least 8 informative windows, ≥80% body match, ≤15% world match, and at least 3cm of feature-relative motion within that measured 5cm experiment; conflicting/unobserved points remain unresolved. Legacy evidence still requires its original 5cm per-feature baseline. The learned 1.5cm mask is posture-, calibration-, model-, boot- and time-bound, and external surface protection still applies. Raw scans, proof, and the immutable original map remain available. Stationary preview commands never publish a learned mask. Continuation requires measured displacement >=5cm and either confirmed body features or an empty candidate set, with no fixed-environment feature detection; otherwise the robot remains stopped. Cancellation stops the entire chain.

config/manual_floor.json is a user-painted map-bound ground overlay. It adds 2D candidate support only; it does not remove 3D obstacles, change the original map hash, or assert measured ground returns. The GUI shows it in cyan and provides bounds and an on/off setting. Generated manual_floor_map.pcd is separately labelled/exported to the cloud library. The frozen-scene check added 1,640 support cells while retaining all 714 occupied cells.

Read-only tracker and motion observer now have separate sensor-relay leases. Navigation/tracking dispatch 2Hz cloud frames while motion evidence uses independent 1Hz windows; tracker pose files publish measured odometry at 10Hz. The live collision calculation retains a bounded 60cm external context around the body. A missing head GUI is handled by a scoped query-only serial reader, with no enable/target/disable commands; its lease ends with the navigation client. Unit/static tests passed on both hosts. Physical 5cm motion and full-route continuation have not been executed by the assistant; current live admission can still pause on near-point ambiguity or stale feedback, and is not claimed validated for unattended operation.

Release latency repair, 2026-10-30 afternoon: controller 192.168.1.190 became reachable again and measured arm serials resumed. The Jetson publisher now validates a just-received arm sample using a clock sampled after the read, avoiding concurrent-feedback future-time failures. S100's 2Hz full receiver_status.json is retained for diagnostics; a separate 20Hz /dev/shm/tianji_s100_joint_feedback.json carries measured joints and original component timestamps. The navigation client uses that stream and rejects it if stale; generating a new packet never freshens old measurements. This resolves the conflict between a 0.5s status-file cadence and the unchanged 0.35s posture admission deadline.

During isolated navigation there is now one ROS sensor relay. sensor_bus.py fans its unfiltered latest packets out through atomic RAM snapshots to tracking and motion observation, adding transport delay to source age and preserving original timestamps. Standalone tracking/observation commands retain their own read-only relay. The main live filter also reuses already-computed external-surface protection; eight frozen regression scans produced identical masks with zero protected points removed. All prior collision margins, admission deadlines and velocity caps are retained.

Validation: 72 tests pass on Jetson and S100. Fast joint snapshots were fresh in 40/40 sampled reads (mean snapshot-file age 17.8ms, maximum 66.1ms). A zero-ROS navigation preview reached PREVIEW_READY after three independent good scan checks. That preview subsequently latched STOP_GUARD_LOST for CABLE_PROBE_APPROACHES_UNCONFIRMED_RETURN. Frozen attribution found rear-LiDAR returns near the left gripper fingertips at 2.57cm and 4.05cm from the model; these are not claimed proven cables and remain retained. No physical 5cm motion or full-route continuation was executed. Future terminal/GUI final statuses preserve the last blocking reason and per-check counts rather than displaying only GUARDS_NOT_READY.

Later inspection-policy correction: those near-body wire-region alerts are now recorded throughout the initial 5cm instead of aborting it. If the full candidate route is blocked (including GOAL_COLLISION), an explicitly uncertified, point-cloud-ignored intent route may supply only the inspection prefix; the retained scene still checks that local prefix with the stated wire-region exception. The intent route is never armed for full navigation. A fresh normal planner, with actual occupancy and no temporary wire-region exemption, decides the remainder only after the 5cm experiment. The final zero-ROS preview completed as PREVIEW_CABLE_PROBE_VALIDATED without an early wire stop or learned-mask publication. Physical 5cm motion remains user-started from the GUI; none was executed by the assistant. Tests cover approaching and intermittent wire returns across the full inspection distance, retained surface/out-of-region collision checks, the 6cm bound, raw replay body-vs-world discrimination, and rejection of premature continuation.
