"""Display configured placement separately from fresh execution evidence."""
def status_text(readiness,execution,now):
    cfg=readiness.get('point_filter_compute') or {}
    if not cfg:return '点云滤除：等待S100确认配置'
    host='Jetson' if cfg.get('mode')=='jetson' else 'S100'
    version=str(cfg.get('package_id') or '未提供')[:12]
    configured=f"点云滤除：已配置 {host} / {cfg.get('backend','CPU')} · 版本 {version}"
    age=now-float(execution.get('updated_wall_time',0))
    if not execution.get('active') or not 0<=age<=2:
        return configured+'\n当前无新鲜执行回执；从GUI启动后核对实际处理端。'
    timing=execution.get('perception_timing') or {}
    rows=[timing.get(sensor) or {} for sensor in ('front','back')]
    if not all(r.get('compute_host') for r in rows):return configured+'\n等待双雷达实际处理回执。'
    hosts='/'.join(sorted({r['compute_host'] for r in rows}))
    latency=' / '.join(f"{float(r.get('filter_compute_s',0))*1000:.0f}" for r in rows)
    return configured+f'\n本轮实际处理：{hosts} · 前/后雷达 {latency} ms'
