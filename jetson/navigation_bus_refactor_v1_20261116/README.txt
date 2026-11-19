导航总线隔离改造首版 / 2026-11-16

状态与边界
本版本是模块契约、总线路由和兼容适配层的首版；没有替换现有GUI、原导航服务或硬件输出入口。
当前仅支持隔离fixture和生产只读shadow。两个模式都拒绝实机navigate；请求参数不能开启运动权限。
fixture里的PLAN_READY来自非物理执行器，不代表Hybrid A*已经计算了真实可行路径，更不代表机器人到位。
原577行执行主循环和底层算法仍在原版本中，尚未整体拆完。旧代码的必要耦合暂时封装在兼容边界内，不能声称整个旧系统已零耦合。

目录（两端相同项目名）
Jetson：/home/nvidia/tianji_table_approach_dev/navigation_bus_refactor_v1_20261116
S100：/home/sunrise/tianji_table_approach_dev/navigation_bus_refactor_v1_20261116

依赖方向
大脑/调试客户端 -> HTTP总线 -> 公共消息/端口契约 -> 业务模块 -> 注入的适配器 -> 原任务接口
只有navbus/app.py组合根认识所有模块实现。navbus/bus.py只认识Module协议及模块申明的能力。
业务模块只能导入navbus/contracts.py，不得导入彼此、I/O、SDK、ROS或环境配置。测试会检查这个边界。
配置为冻结实例，消息递归只读；注册表实例私有且冻结。新业务模块无可变业务全局变量。
任务ID、幂等和取消沿用上游任务内核，总线不再建立第二份任务账本。
停止操作不会因总线审计磁盘不可写而被阻止；其他变更必须先成功记录意图才分发。
点云大数据、滤除和速度闭环不进入这个低频任务总线，避免引入S100实时链路负担。

业务模块及API
navigation：plan、navigate、test、get、cancel、events；通过TaskPort调用任务内核。
world：waypoints；通过WorldPort读取点位，不直接导入导航模块。
readiness：get；通过ReadinessPort读取状态，不产生运动放行证书。
localization：snapshot；通过DocumentPort返回原始位姿/时间信息，保留原有RAM优先规则，不刷新观测时间。
perception：status；通过DocumentPort读取状态元数据，不在总线里转发点云或计算速度。
当前定位/感知模块是状态接口，尚未把原计算工作进程本体迁入新模块。

代码入口
navbus/contracts.py：冻结消息、协议端口、模块能力描述。
navbus/bus.py：实例注册表、统一路由、运动权限与审计。
navbus/modules/：五个独立业务模块。
navbus/adapters/legacy_http.py：原任务API的HTTP适配器；shadow拒绝所有POST。
navbus/adapters/storage.py：只读状态文件和审计存储。
navbus/http_api.py：认证HTTP入口，不包含业务模块实现。
navbus/app.py：读取配置、注入依赖、启动服务。
navbus/client.py：大脑/GUI使用的SDK。
legacy_fixture/：仅保留隔离任务代理所需的7份旧自研模块，不含原厂SDK或硬件执行器。
fixture_server.py：把非物理runner注入原TaskManager副本，验证总线确实沿用原任务状态机。

启动（S100上两个终端；服务默认只监听本机）
cd /home/sunrise/tianji_table_approach_dev/navigation_bus_refactor_v1_20261116
bash run_fixture.sh
另一个终端：bash run_bus.sh
fixture任务接口：127.0.0.1:18765；总线：127.0.0.1:18766。
只读生产观察：bash run_bus.sh config/shadow_s100.json，端口18767；上游为原8765，仅允许GET。
当前生产8765、滤除18876、现有GUI配置均未切换。
测试程序结束会停止它自己启动的服务；本次验证后没有留下新测试服务或新增自启项。

总线HTTP契约
GET /healthz：模式和能力；不返回认证信息。
GET /v1/modules：认证后查看模块及其操作、变更/运动/停止能力申明。
POST /v1/bus/call：{"module":"navigation","operation":"plan","arguments":{"goal":3,"scene":"office"}}
POST /v1/navigation/goals：{"mode":"plan","goal":3,"scene":"office"}，作为大脑的简便入口。
Authorization: Bearer <总线token>；Idempotency-Key: <调用方稳定的请求ID>。
响应含request_id、module、operation、bus_mode、result；result内id是上游原任务ID。
同一业务请求重试必须沿用同一个Idempotency-Key；目标变化不能复用该ID。
token位于隔离项目runtime/bus_token或shadow_token，不在此文档或日志中输出，不要拷贝到公开交接包。

SDK示例（在S100项目目录执行，或通过SSH转发到该loopback端口）
from pathlib import Path
from navbus.client import BusClient
client = BusClient("http://127.0.0.1:18766", Path("runtime/bus_token").read_text().strip())
job = client.plan(3, request_id="agent-plan-point3-001")
result = client.wait(job["id"])
client.call("world", "waypoints", {"scene":"office"})
client.call("readiness", "get")
client.cancel(job["id"])
client.navigate(3)接口已定义，但本轮配置会返回MOTION_DISABLED，不会启动底盘。
第三台设备的大脑文件仍按只读约定未修改；本接口不是现有坐标协议的自动替换。

验证
python3 -m unittest discover -s tests -v
python3 verify_pipeline.py
S100：python3 verify_pipeline.py --shadow
11项单元/架构测试，两端均通过。
S100 10项联通案例通过：注册表、1/2/3号位仿真任务、幂等、取消、运动拒绝、观测时间保留、生产点位只读、原任务账本不变、查询延迟。
S100只读总线查询10次，中位约6.39 ms、最大约9.63 ms；这是元数据接口总耗时，不是实时导航控制性能测试。
见s100_pipeline_verification.json、pipeline_verification.json、source_preservation.json。
133份原生产Python模块的SHA256仍与快照一致；原厂导航关键脚本前后哈希一致。

快照与清理
Jetson：/home/nvidia/tianji_table_approach_dev/refactor_snapshots/before_bus_20261116_streaming
S100：/home/sunrise/tianji_table_approach_dev/refactor_snapshots/before_bus_20261116
每处有code_config_models.tar.gz和manifest.json，快照时源文件变化数均为0。
快照含私有配置，目录0700、归档0600，不应作为公开下载文件。
已从Jetson删除20份2026年10月原始raw_bag/*.db3和2个无源自研字节码缓存，合计5,216,227,952字节（约5.22 GB）。
原始录制均已在上述Jetson快照内保留；清单见该快照目录cleanup_applied.json。
还删除了本轮中断产生的无效快照临时归档约1.85 GB；有效快照保留。
当前处理后的地图voxels、点位及活跃依赖未删除；v2/v3/v5仍存在实际导入依赖，不能按版本名直接删除。
原厂SDK和/home/sunrise/nav2及其原厂导航不在清理目标内，未修改。

后续迁移顺序
1. 在保持既有任务ID、停止语义、时间戳及几何门控不变的前提下，让GUI和大脑逐步共享总线接口。
2. 把旧执行器的会话状态、资源生命周期和恢复流程提取成实例组件；先回放验证，再做无运动实链路对比。
3. 计算模块迁移完毕再删除旧调用路径，保留可回滚入口；不能用薄适配器包装成功代替整个内核已重构。

第二阶段：执行内核实际抽离（仍为隔离副本）
工作副本：engine_clone/；原navigation_s100_v7生产目录没有改动。
第一阶段代码另存于两端snapshots/phase1_before_engine_日期目录，仍保留最初两端完整快照。

engine_clone/modules/onboard_navigation.py已实际接入以下组件，不是只新增空接口：
- execution_runtime/contracts.py：Progress、ResourceBindings、StatusIdentity、StatusInputs及结构化生命周期端口。
- execution_runtime/operator.py：OperatorChannel，拥有输入与心跳线程，显式接收lease、stop、输入流、时钟、线程工厂。
- execution_runtime/status.py：StatusPublisher，拥有历史、发布节流状态和诊断队列，注入日志/时钟/健康读取接口；发布快照复制后入队，后续修改不能改写既有记录。
- execution_runtime/resources.py：Resources，分别公开stop_control和stop_observers，保留先关闭控制、收集停止证据、再关闭感知的顺序；逐项记录错误，保护常驻定位进程不被任务退出误杀。
- execution_runtime/motion.py：冻结的MotionLimits，通过显式policy构造，每会话持有自己的限速/盲区行程参数。执行入口不再依赖navigation_control导入时的速度全局值；旧模块别名保留给兼容调用者，尚未宣称全项目全局变量已经清零。

主run从577行降为535行；跨嵌套函数的nonlocal变量绑定从7个变为0，内部函数从16个降为10。
核心碰撞/运动决策循环归一化变量访问后AST相同；没有修改源时间戳、放宽门控或改变到位条件。
新旧限速函数2400组输入逐项相同。三条无运动退出路径（无路线、规划阻断异常、一般异常）的状态记录及终态对比一致。
这些对比使用明确的替身输入/输出工厂，不连接真实传感器，也不能称为真实环境导航性能或完整场景回归。
新组件源文件哈希已加入隔离副本execution_metrics.signature，避免新组件变化缺少代码版本证据。

隔离保护：除总线拒绝实机navigate之外，执行器、ControlTransport、control_agent和control_bridge都拒绝从隔离目录启用物理运动。
保护依据组件自身文件位置及隔离标记，避免错误复用common模块的ROOT缓存后误绕过。
该副本用于源码/回放与无运动验证，不能直接当作生产启动目录。现有GUI/API和原厂SDK/导航均未切换。

第二阶段验证命令（项目目录）
python3 -m unittest discover -s tests -v
python3 -m unittest discover -s engine_clone/tests -v
当前第一组24项、原策略回归第二组23项在Jetson上通过；S100结果见s100_phase2_validation.log。
第二组覆盖障碍等待、数据过期恢复、最终到位容差、输入管道阻塞及子进程退出。
engine_phase2_verification.json记录抽离指标、源哈希、AST/限速差分结果。

以上为第二阶段当时的范围；第三阶段已进一步抽离下列决策流程。旧算法/模型适配层仍保留必要依赖，见下面的完成范围。


第三阶段与本次整理的完成范围（2026-11-16）
主执行器已实际调用三个独立阶段，共抽离244行原决策源码：
execution_stages/route.py：定位锚点确认、停车重规划、路线复用、模型/地图/雷达身份核验。
execution_stages/collision.py：新关节/新底盘位姿复检、运动证书、场景更新、整机扫掠和地图复检。
execution_stages/admission.py：放行、连续清空确认、障碍/数据等待、跟踪输出及到位复核。
每阶段只通过State和冻结Ports收发数据，不导入其他阶段实现；Flow明确返回NEXT/CONTINUE/BREAK。
State只允许声明的字段；未定义输入仍报错，不用缺失数据伪造净空。
主执行器在finally中同步已写状态，使异常前的有效写入与原流程保持一致。
共享contracts仅定义边界，不保存全局会话。各阶段保留原来的私有算法适配调用，必要依赖不靠复制算法消除。
各阶段哈希已纳入execution_metrics代码签名。

这不是仅把函数改名：五条控制循环回放实际经过新的阶段调用，并与抽离前的循环比较：
正常预览放行、到位复核、定位锚点确认、关节过期、障碍等待。
另外验证了重规划失败、雷达身份变化、地图碰撞、状态隔离、只读输入、0.005m复检余量和观测时间不改写。
展开阶段后的完整碰撞/运动主循环AST与原生产决策相同；限速2400组对比仍一致。
这些测试注入传感器/时钟/控制替身；update仅记录到替身，没有ROS/CAN/底盘输出，不能称为实机到位验证。

完整检查：./verify_all.sh
S100附加生产只读链路检查：./verify_all.sh --shadow
36项总线/内核/阶段测试 + 23项旧策略回归；测试报告见refactor_completion.json和两端日志。
verify_architecture.py只读核验业务层边界、组件状态、源码语法、生产Python源码及S100原厂关键脚本哈希。
stage_extraction.py与refactor_engine.py是迁移复现工具，不在控制循环中执行；运行前必须保留隔离标记。
生成代码可以正常维护；重新从旧源生成会覆盖隔离组件，应先保存快照并执行整套差分测试。

当前交付状态
隔离模块版在Jetson和S100各保留一套，任务总线、SDK、兼容适配、内核组件及测试均已整理。
大脑/GUI的接入方式只依赖navbus.client.BusClient，示例及限制见API_CONTRACT.txt。
生产GUI、生产任务API、底盘执行器没有切换；本次不启动底盘运动，不新增自启动服务。
fixture为协议仿真，shadow为生产只读；二者均拒绝真实navigate，不能把PLAN_READY仿真当作实际几何规划成功。
旧的定位/点云/规划/模型工具作为导航模块内部的私有适配层保留，未把每个数学函数都改写为总线服务。
高频点云和速度控制不经过任务HTTP总线，避免引入额外传输和序列化负担。
原厂SDK、原厂导航、原有地图/点位、已知仍有依赖的旧版本目录均保留；只删除已验证且有快照的旧录制与孤立缓存。
恢复时使用snapshots中的版本归档或最初的完整refactor_snapshots；本次没有覆盖生产文件，所以生产无需恢复操作。

最终两端验证结果：Jetson 59项、S100 59项通过；S100另有10项总线/只读链路检查通过。
生产源码核验：Jetson本地127份、S100的103份模块与初始快照相同；S100原厂导航关键脚本哈希相同。
两端隔离执行入口--help导入成功；S100测试端口18765/18766/18767均已关闭。
主run语法树语句数由820降到500；主循环语句数459降到226。物理行数受格式展开影响，不作为精简效果指标。


GUI实测接入（2026-11-16，后续检查点）
桌面导航GUI新增“模块化总线（重构版测试）”，当前已选中。旧CLI/API选项保留。
GUI -> SSH隧道 -> 127.0.0.1:18769任务总线 -> 18768专用任务代理 -> gui_runtime/modules/onboard_navigation.py。
生产8765任务API、S100生产模块及原厂SDK/导航保持不变。本次按用户指示修改Jetson前端4个现有模块并新增适配器；哈希单列于gui_frontend_changes.json。
新的gui_runtime是显式部署候选，engine_clone的隔离标记和运动拒绝保护保留。
候选通过引导解析使用生产传感器/模型/定位/锁目录，执行阶段从自己的execution_runtime/execution_stages加载。
任务账本在gui_runtime/runtime/task_api下，不复制到总线；规划/执行产物继续由生产results提供，GUI沿用原来的下载与渲染。
新/旧执行入口共用production navigation_execution.lock和navigation.lock，专用任务代理也检查这两个锁。
新代理和总线仅loopback监听，专用token不出现在文档/日志中；GUI用既有SSH连接访问。
start_gui_services.sh幂等启动本次手动服务，不新增开机自启动；GUI选中bus后，首次操作自动检查并启动这两个服务，但不会自动发起导航任务。
停止：先对归属专用代理的候选PID发送SIGINT后备，再经总线取消任务。后备核验session、任务归属、准确命令路径与进程启动ticks，拒绝其他进程及PID复用。
实机启动仍需用户点击GUI“启动实机导航”；本轮未自动提交实机任务。
两端新增接口/部署/停止门控后，41项主测试+23项旧策略回归通过；停止测试使用注入kill替身，未给真实进程发送测试信号。
待机占用见gui_idle_resources.json：新S100总线约22MiB RSS，专用代理约20MiB RSS；这是待机统计，不是行驶性能。
30分钟只读资源采样在两端runtime/resources_*_gui_test_v2.jsonl，每秒一条，整机CPU按全核百分比，进程CPU按单核百分比。
本次没有实机前后速度/耗时对比，因此不能据此宣称导航更快或CPU更省。

2026-11-16 GUI首次实测修复：任务83fa97d11d8548ba966fc8556cfad815规划成功，但因重构漏掉micro=None初始化，在执行准备阶段STOP_ERROR；运动指令0。补回初始化，新增普通模式成功规划后完整准备路径对比与可选状态初始化核验，43+23项无运动回归通过。已更新S100候选入口；无需重启GUI，新任务读取修复后的文件。详情见micro_fix_deployment.json。


2026-11-16 长距离侧移规划失败修复（此方案已撤回，见下一节）
删点后冻结任务goal_1_preview_office_20261116_143914_663133确认使用新地图dccbc33a...，两轮Hybrid A*仍分别超时12秒。
失败来自原短距候选1.5m范围与常规搜索侧移预算；新帧目标体坐标[0.460897,-2.274348,0.025705]。
重构候选增加planner_overrides/policy.py和checked_translation_fallback.py；原生产policy.py、原厂导航/SDK及 native 二进制没有改动。
只在常规搜索失败且不是START_COLLISION/GOAL_COLLISION时尝试，最大直线距离3m、直角总长3.5m、朝向变化30度。
使用原native引擎重新生成候选，场景占用/碰撞体/安全余量不变，再检查路径所有中间采样的已知地面格和原world_guard完整扫掠。
保留原profile用于跟踪器兼容，selected.candidate_family=checked_extended_translation_fallback、independently_rechecked=true用于识别。
现有合格路线不受替换；这不是对1号位写死路线，也没有忽略残留/连续环境障碍物。
候选preview_plan和planning_worker入口都采用引导加载，初始规划与停车重规划均加载同一候选policy。
GUI重构模式的preview_plan.sh指向候选preview_plan.py；旧CLI/旧API仍保留原规划器。
完整固定帧回放：PATH_PREVIEW_READY，路线2.735245m，侧移2.274348m；独立world_guard clear=true，总耗时25.92秒，其中回退约1.13秒。
此回放只读取冻结地图和关节快照，没有实时采集、定位、控制进程或任务提交。实机仍由用户启动。
当前保留两轮常规搜索优先，所以失败后回退仍先等待约24秒；不应将它宣称为初始化提速。
另修复Jetson规划终态日志保存：plan_goals在返回/抛出失败前写last_api_task.json，避免最近任务仍显示旧导航成功。
验证报告：planner_fallback_verification.json；部署前备份位于S100 snapshots/before_planner_fallback_20261116_144929。


2026-11-16 纠正：保留障碍，通过可选中间节点绕行
用户指出应沿桌边经过2号位，而不是用长平移回退及删除地图点放行。此前对局部稀疏低点的“残留”判断证据不足。
已恢复全部4个体素：两端voxels.npz均恢复原始SHA256 4393e979...，点数187018；点位版本单调增加，坐标保持原值。
长平移扩展回退已从候选policy移除，代码及测试移入snapshots/retired_extended_translation_*，不得再部署启用。
原问题：单段Hybrid A*的累计/连续侧移预算限制，加上没有中间节点连接层，使3附近到1的约2.3m侧向任务陷入受限搜索。
新候选使用waypoint_corridor_graph.py：当长距离直达受阻时，把同场景同地图的已保存点位作为可选图节点，使用Dijkstra选择路线。
节点ID没有写死；每条边保留原短距1.5m和直角总长2m限制，原冻结地图与原已知地面格完全保留。
边由原生world_guard检查，整条拼接路线再次检查；不向虚拟中间点补地面，不跳过末端转向碰撞检查。
中间节点为途经点，不要求停下并对齐它的朝向；最终目标仍满足原位置/角度容差。
普通直接场景保留原规划偏好；图不连通/点位变化/复检失败时回到原搜索，不强制经过2号位，不删点或忽略障碍物。
候选初始规划、GUI预览和停车重规划均使用同一候选policy；原厂SDK/导航和生产103个Python模块未修改。
原图固定帧完整回放：自动选择via_waypoints=[2]，路线2.691163m，耗时0.750s，所有恢复点保留，整机与已知区域检查通过。
这仅验证该历史冻结帧，不代表当前实际场景放行；没有实机任务提交或运动指令。
报告waypoint_graph_verification.json；示意runtime/route_via_2_original_map.png；候选源码planner_overrides/{policy,waypoint_corridor_graph,frozen_world_guard}.py。

2026-11-16 拐点接近控制修复（GUI候选版）
实际任务dd088589a3614edc81f21dc620556547由用户停止；末段约45秒速度极小。记录回放发现拐点误差约20.05mm，旧位置死区20mm，而原过点条件为15mm，导致修正过小。
新增私有planner_overrides/follower.py，仅在原55mm拐点接近窗口、平移段切换处使用5mm位置死区和不超过0.015m/s的接近速度。15mm过点条件、角度死区、原下游碰撞及状态有效性检查保持不变。
候选部署记录candidate_follower_sha256；被动导入已确认GUI下一任务加载候选follower。生产103个模块、原厂脚本及完整恢复地图哈希验证未改变。
S100原主测试56项通过（1项Jetson专用跳过）、策略23项通过；补充后拐点专项5项在两端通过。历史位姿纯运动学积分约0.35秒达到切换条件，仅为模拟，不是实机性能。
本次没有提交导航任务、没有发送运动指令。由用户从GUI重新测试实际跟踪效果。报告corner_approach_verification.json。

2026-11-16 近桌前后半速（用户要求）
GUI候选版在原限速后、碰撞检查前将近场前后vx乘0.5；横移vy和旋转w保留原值，用户GUI速度上限继续有效。
利用现有自体滤除后点云：z=0.35~1.8m外部回波距实时机器人XY包络不超过0.40m触发。此为保守近场触发，桌边/椅子均可能触发，不宣称语义识别桌面。
缺失、空帧、非有限或点云年龄超过0.75秒时也降速；离开后保持2秒再恢复，避免反复切换。原过期停机/碰撞拦截照常执行。
诊断记录execution_profile.near_surface_speed；不新增拟合或点云传输。原厂SDK与103个生产模块不变。
本地主测试61项通过；S100主测试61项（1跳过）、策略23项通过，未发送运动指令。报告near_surface_speed_verification.json。下一次用户GUI启动导航生效。

2026-11-16 静默排查停顿及安装座线缆平面保护修复
任务579899462875472fa84beba0ae059d4e（goal_3_20261116_160916_125886）由用户停止。碰撞拦截约18.68秒，稳定净空确认约29.79秒，跟随约14.74秒；没有LIDAR_STALE等待。
历史最后帧两个命中点在安装座附近，均经缓存平面保护保留；已确认线缆区原逻辑跳过plane_keep检查，加上重复密集点可满足曲面支撑计数，导致不稳定地保留自体附近点。
仅在原confirmed_mount_cables区域内对缓存平面点补充独立外部支撑检查；支撑按15mm体素计数。薄桌边例外保留，并要求缓存平面有独立局部平面见证；不扩大区域、不改碰撞余量、不删地图。
修复在Jetson navigation_filter_offload_v1/bundle/modules/continuous_surface_filter.py，manifest与两端filter_compute package_id同步为8dbdaaa4666be16ceae5a3943862f6c6b6f7797a99148184606f455362e33d3a。S100生产103个源码与原厂代码未改。
重启的只有滤除服务，没有打开GUI、提交导航或发送运动指令。安装时持有两把原导航锁。近桌前后半速保持启用。
33项面/边缘/曲面等测试通过；原始冻结帧回放从2命中变为0命中，只额外去掉原线缆区内5点，区外分类不变；S100→Jetson CUDA链路两路输出与候选CPU完全一致（容差1e-10）。仅为历史回放，不代表当前环境实时净空或实机行驶验证。
报告runtime/latest_pause_fix_20261116.json；隔离测试navigation_filter_offload_v1/experiments/mount_plane_witness_20261116。
原实验verify_remote.py若使用S100未修改的旧CPU实现作基线，将有预期的分类差异；新版本验证须使用上述候选CPU实现，不能宣称与旧算法等价。

2026-11-16 大脑接口对齐模块化候选导航
当前生效：大脑兼容接口与模块化导航共用执行链（2026-11-16）

接入地址为S100上的http://127.0.0.1:18769，需通过受控SSH转发访问；本次没有在大脑服务器保存常驻转发、令牌或业务配置。
令牌使用S100本项目runtime/gui_bus_token，不能使用旧8765令牌。大脑客户端支持TIANJI_NAV_TOKEN或TIANJI_NAV_TOKEN_FILE（后者优先、要求私有普通文件）。凭据内容不写入文档。
大脑源码无需修改；原TianjiNavigationSkill保持原fq/reception-lan/v1合同。对方业务流程仍未注册该原子技能，不把接口验证称为业务接管完成。

真实数据流：
大脑TianjiNavigationSkill（本次对方文件只读）
 -> 18769 HTTP兼容入口
 -> Bus -> brain_navigation模块 -> LegacyGateway的受限兼容端口
 -> 18768候选TaskManager（与GUI编号任务共用唯一任务账本）
 -> gui_runtime/execute.sh -> gui_runtime/modules/onboard_navigation.py
 -> 当前定位/规划/拐点控制/近桌前后半速/实时守卫/控制及停车证据。
Jetson点云滤除服务仍沿用当前版本；S100原生产8765和原厂SDK/导航不改、不重启。

兼容HTTP入口（除原healthz外均需Bearer鉴权）：
GET /health
GET /v1/status
GET /v1/world
GET /total_scene_graph_latest.json
POST /v1/navigation/goals      大脑完整坐标合同，保留task_id、command_id、goal_xyt等字段
GET /v1/commands/{command_id}
POST /v1/navigation/cancel     按原task_id+command_id取消
原GUI编号调用与POST /v1/bus/call仍保留。/v1/navigation/goals检测contract_version、command_id或goal_xyt时走大脑合同；不会把任意坐标替换成最近编号点。
总线模块brain_navigation暴露health/status/world/graph/submit/get/cancel，直接bus/call返回result.http_status和result.body；上述兼容HTTP直接返回原合同正文及原HTTP状态码。

所有命令幂等、冲突、任务存储、事件、取消和成功/停车证据均由同一个上游TaskManager负责。
同command_id同请求仅执行一次；同command_id不同请求返回409；旧命令取消不会停掉新命令。
提交受理不等于到位，取消受理不等于物理停车。实际成功仍要求原到位和停车证据，不降低原5cm/1度验收。
若上游POST回包丢失，或受理后总线结果审计失败，兼容入口保留连接中断的不确定性，让现有大脑客户端只查询原command_id；不会伪造明确HTTP失败诱导新发任务。
取消意图/结果审计故障不能阻止原取消请求下发。

配置：
候选config/brain_compat.json链接到S100生产config/brain_compat.json；修正原部署误写brain_navigation_compat.json。
代理默认--scene office，并在导入旧读取器前设置该进程TIANJI_NAV_SCENE；候选readiness按manager.registry的同一场景读取地图。
办公室外部map绑定固定session_base_start，地图SHA256为4393e9799b895884d6a078396cf9f04f3ae3d4249047a0443587b097b40af384；点位1/2/3来自同一办公室配置。
其他场景的外部坐标调用仍需匹配明确的地图绑定，不自动混用办公室和外侧地图。
forward_path受支持；lateral_path_aligned继续明确拒绝，未承诺严格锁定朝向的横移合同。
相机/ShapeMask已有测试放行配置保持原值，并如实标注FORCED_CLEAR_TEST，不伪称实测净空。
空闲时LIVE_ADMISSION_NOT_CONFIRMED表示当前没有活动任务的实时路径准入证明；motion_enabled表示允许受理能力，不代表当前运动安全已通过。

客户端接入步骤（由对方维护者配置，未改对方文件）：
1. 将可达SSH转发目标设置为S100的127.0.0.1:18769；S100仅回环监听，跨网段可通过Jetson跳转。
2. 将该转发URL作为TianjiNavigationSkill(base_url)或scripts/tianji_navigation.py --base-url参数；配置新的gui_bus_token凭据。
3. 先调用check()核对地图/合同；真实派发需要调用方显式allow_motion=True，并保存原任务/命令身份。
4. 业务Planner/Runtime注册与实际行驶/停车验收另行完成。不要把本次模拟PASS当成真机到位。
本侧服务使用原start_gui_services.sh按需启动，不依赖GUI常驻，不新增开机自动任务。

验证证据：
本地主测试72项通过；S100主测试72项通过（1项Jetson专用跳过）。
原样大脑源码通过隔离模拟：成功、停车未知拒绝、提交回包丢失后只查询原命令、取消身份；每条导航命令执行次数为1。
实际部署后，大脑原客户端对18769和18768均通过GET元数据检查；生产8765继续只读。
部署时两把原导航锁保护，确认无活动候选任务；任务账本前后哈希一致。只重载两个既有候选服务。
第一次重载使用systemctl start遇到--collect临时单元已卸载，已回滚并用原start_gui_services.sh重建；后续部署按此方式完成，无任务自动提交。
对方关键源码前后SHA一致；原生产103个S100模块和原厂导航脚本保持原哈希。本次未打开GUI、未提交实机任务、未发送运动命令。
报告：runtime/brain_alignment_20261116/{live_brain_metadata_check.json,offline_actual_brain_bus.json,deployment.json,compatibility_release.json}。


Jetson统一对外网关（2026-11-16，当前入口）

大脑只访问：http://192.168.31.150:18769
完整链路：大脑 -> Jetson HTTP网关 -> SSH -> S100 127.0.0.1:18769总线 -> 18768同一候选任务代理 -> 原导航执行/反馈。
GUI调试支路继续使用现有S100总线适配器；两者共用S100任务账本，Jetson网关不缓存任务终态、不另存任务账本、不控制电机或自行重试动作。
请求与状态正文、command_id/task_id、幂等键、成功/停车证据由上游透传；受理后链路不确定时保留连接中断，由大脑只查询原命令。
网关关闭或网络断开不等于S100任务已取消/底盘已停。取消必须调用原身份的取消接口，之后查询原停止证据。

网关源码：navbus/jetson_gateway.py
Jetson配置：config/jetson_gateway.json
Jetson私有凭据：config/jetson_gateway_token（0600，仅路径公开，不复制到交接正文）
S100后端凭据由网关使用既有已校验主机密钥的SSH连接读取，不发送给大脑、不写入网关日志。
绑定Jetson当前Wi-Fi地址192.168.31.150，仅允许已配置大脑192.168.31.144和本机自检来源；没有绑定0.0.0.0、修改防火墙或开放S100端口。
Jetson地址若变化，需明确更新网关bind及大脑URL；不会自动切到全网卡。

服务：tianji-navigation-brain-gateway.service（Jetson用户服务，已运行，未启用开机自启）
启动：systemctl --user start tianji-navigation-brain-gateway.service
状态：systemctl --user status tianji-navigation-brain-gateway.service
日志：runtime/jetson_gateway.log（仅请求方法、路由摘要、状态、耗时，不含令牌和请求正文）
无需GUI常驻。已有S100候选服务仍使用其原启动器维护，网关不会擅自启用运动或自动提交目标。

验证：82项本地测试通过；大脑原源码经隔离网关的模拟成功/停车未知/丢回包/取消测试通过，明确physical_execution=false。
大脑192.168.31.144以原TianjiNavigationSkill直接访问Jetson局域网URL，GET健康/地图/状态通过，没有使用临时HTTP转发到S100。
5次状态查询端到端耗时约45~64ms，仅小样本，不代表所有负载下的性能保证。
对方关键源码与S100任务账本前后哈希不变。未修改对方文件、未打开GUI、未提交真实导航任务或发送运动指令。
大脑常驻业务URL/凭据配置和技能注册仍由其维护者处理；本次遵守对方文件只读约束。真实行驶与停止验收尚未执行。
报告runtime/brain_alignment_20261116/{direct_jetson_gateway_readonly.json,offline_actual_brain_gateway.json,jetson_gateway_release.json}。


2026-11-16 公开网关仅保留办公室范围；其他场景在公开请求、任务历史及事件读取前检查来源场景。85项回归通过。内部GUI和原地图文件不改。已只读核对清理桌面联调文档，现有3点尚未定义为3个清理工位，桶前/等待位置未映射；建议隔离大脑目录当前不存在，文档原件未修改。
