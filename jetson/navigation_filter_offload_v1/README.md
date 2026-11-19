# Jetson 导航点云滤除分摊 v1

2026-11-11 已接入现有 S100 导航的 PreparedFrames 后台计算入口。GUI 的下一次导航任务自动使用本配置；本次部署没有启动导航或底盘运动。

## 分工

- Jetson：原始雷达点云坐标变换、URDF 自体滤除、线缆／零散回波滤除、连续表面保护；OBB 距离查询使用 FP64 CUDA。
- S100：采集与配帧、定位、Hybrid A*、轨迹跟踪、实时整机扫掠和最终运动准入、电机输出。
- 分摊服务没有 ROS 发布器、运动接口或业务任务账本。一个传感器连接使用一个进程和独立平面缓存。

## 位置及启动

Jetson 项目：`/home/nvidia/tianji_table_approach_dev/navigation_filter_offload_v1`。

Jetson 用户服务：`tianji-navigation-filter-offload.service`，已设置用户登录环境自动启动；监听有线地址 `192.168.1.100:18876`，仅接受 S100 `192.168.1.10`。不依赖 GUI 渲染。

```bash
systemctl --user status tianji-navigation-filter-offload.service
```

S100 原项目仍为 `/home/sunrise/tianji_table_approach_dev/navigation_s100_v7`。模式在 `config/filter_compute.json`，当前 `mode=jetson`、`backend=cuda_obb`。凭据在单独的 0600 文件中，不写入日志。

实际调用：`PreparedFrames -> prepared_worker.py -> FilterClient -> Jetson filter service -> 原 prepare / SurfaceCache + CUDA OBB -> S100 prepared_*.npz -> 原碰撞守卫`。

## 时间及故障边界

原扫描时间、关节时间、扫描位姿均随帧保留。S100 用自己的单调时钟保留收帧时刻并计入网络／计算延迟，不使用 Jetson 的单调时钟更新新鲜度。模型／配置摘要、请求摘要、传感器和请求 ID 一并核对。

先完成模型和 CUDA 预热，再从 latest-only 总线取最新帧。网络失联时不发布新结果，不隐式换算法；原有 S100 守卫继续按原始数据年龄停止准入。重连后从最新帧恢复。

部署包 `bundle/` 是实际 S100 源码、滤除配置和 URDF 的冻结副本；版本记录在 `bundle_manifest.json`。修改 S100 滤除配置后应重新同步和验证包，不能只修改一端。原地图、目标、碰撞体积和安全阈值均未改动。

## 本次验证

- 当前双雷达各 12 帧，共 24 帧：与 S100 原始滤除的保留点坐标和滤除／保护数量一致。
- 正式 PreparedFrames 加独立被动总线：18 秒采集 128 个已处理帧，两传感器均报告 Jetson / cuda_obb；时间戳及源关节证据未改写。
- 6 项传输身份／超时／报文边界测试通过。
- 9 项现有总线／PreparedFrames 单测在隔离的 measured 合成夹爪配置下通过。直接使用现场 fixed_open 配置运行旧合成模型测试会发生模型 SHA 不匹配；未修改现场策略来通过测试。
- 所有本次验证均无运动输出、无 ROS 发布器。没有据此宣称实车行进中的停顿已解决。

证据：`results/passive_verification.json`、`results/prepared_pipeline_verification.json`、`results/pipeline_summary.json`、`results/prepared_unit_tests.log` 和 `results/deployment.json`。

前／后雷达后台完整往返中位数约 127 / 87 毫秒，S100 收发 CPU 时间中位数约 1.78 / 1.45 毫秒。批量 24 帧对比中 S100 该环节 CPU 时间由 2.06 秒降至 0.07 秒；这不是整机 CPU 降低 97% 的结论。Jetson 双计算进程运行时服务内存实测约 182 MiB，未测试 VLA 并发负载。

## 回退

在导航没有活动任务时，将 S100 `config/filter_compute.json` 的 `mode` 改为 `local`，新任务将使用原 S100 CPU 滤除。不要在运动任务中改配置；工作进程会拒绝中途配置变化。

原始文件快照路径见 `results/deployment.json`；Jetson 镜像和客户端快照见 `results/jetson_snapshot.json`。停止分摊服务与底盘急停是不同操作。
