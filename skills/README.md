# 天机 Agent 导航 Skill 索引

以下名称用于归类 Agent 能力。联调入口使用现有 Jetson HTTP 网关，灵猴 TX-S2 底盘侧负责记录任务状态。桌面控制台共享这条执行链，可用于规划预览和查看反馈。

| 能力 | 输入和现有入口 | 成功依据 |
| --- | --- | --- |
| `navigate_to` | 从场景图读取公开点位和地图绑定，调用 `POST /v1/navigation/goals`。提交时带原 `task_id`、`command_id` 和导航合同字段 | 原命令回报到位，并带有效停车证据 |
| `preview_navigation` | 使用桌面控制台的规划预览入口，查看目标和规划结果 | 返回可展示的规划结果，预览始终使用规划模式 |
| `get_task_status` | 用 `GET /v1/commands/{command_id}` 查询原命令，服务状态用 `GET /v1/status` | 原命令身份匹配，读取执行阶段、终态和已有证据 |
| `cancel_task` | 用 `POST /v1/navigation/cancel` 提交原 `task_id`、`command_id`，随后查询同一命令 | 以停车证据确认停止，等待回包期间保留原命令身份 |

公开目标为 `office_waypoint_1`、`office_waypoint_2` 和 `office_waypoint_3`。提交或取消回包丢失时，先查原 `command_id`，避免重发动作。传感器状态、CUDA 自体滤除、Hybrid A*、轨迹跟踪和实时守卫属于导航实现，统一通过任务接口读取结果。

抓取、投放、清桌面编排及机械臂归位属于导航分支之外的天机执行系统，当前索引只登记本分支的导航入口。

## 实现入口和验证

- [项目启动与部署](../README.md)
- [Jetson 网关合同](../jetson/navigation_bus_refactor_v1_20261116/API_CONTRACT.txt)
- [底盘导航 OpenAPI 定义](../s100/navigation_s100_v7/config/task_api_openapi.json)
- [桌面控制台程序](../jetson/navigation_waypoints_v6/modules/navigation_gui.py)
- [Jetson CUDA 滤除说明](../jetson/navigation_filter_offload_v1/README.md)

无运动的网关合同测试从仓库根目录运行：

```bash
cd jetson/navigation_bus_refactor_v1_20261116
python3 -m unittest discover -s tests -p 'test_jetson_gateway.py'
```
