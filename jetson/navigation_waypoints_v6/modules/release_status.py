"""Human-readable navigation interlock diagnostics; never controls hardware."""
REASONS={
 'LOCALIZATION_ANCHOR_CONFIRMING':'定位出现小幅修正，停车等待新雷达证据确认',
 'LOCALIZATION_ANCHOR_CHANGED':'定位修正持续存在，停车复检路线',
 'UNCHANGED_MAP_ROUTE_REVALIDATED':'剩余地图路线复检通过，等待实时数据重新放行',
 'LOCALIZATION_RECHECK_REQUIRED':'雷达定位正在复核，保持停车',
 'GOAL_MOVED_DURING_FINAL_CHECK':'最新定位超出完成精度，停车重新确认',
 'FINE_COLLECTING_NEW_POSES':'正在收集新的位姿读数，输出为零',
 'FINE_PULSE_OBSERVING':'小段修正结束，停车观察新位姿后再决定方向',
 'FINE_WAITING_POSE_STABILITY':'误差接近精度范围，停车等待定位稳定',
 'FINE_NO_PROGRESS_HOLD':'连续修正没有测到足够运动，已停止尝试；请检查低速响应及控制拦截',
 'FINE_PULSE_BUDGET_HOLD':'已达到分段修正次数限制，仍未稳定收敛；保持停车，请检查定位误差',
 'GOAL_CANDIDATE_PENDING_STOP':'控制误差已进入范围，正在驻车并复核最终位姿；尚未完成',
 'STOP_FINAL_POSE_OUTSIDE_TOLERANCE':'已停车，但驻车后最终位姿超出到位阈值',
 'STOP_FINAL_POSE_UNCONFIRMED':'已停车，但缺少有效的最终定位证据',
 'STOP_FINAL_STOP_UNCONFIRMED':'最终停止证据未确认，不能宣告到达',
 'GOAL_TOLERANCE_REACHED':'目标位置和朝向均满足到达条件',
 'CONTROL_SHUTDOWN_FAILED':'停车控制进程收尾异常，请检查日志',
 'EXECUTION_PROCESS_EXITED_WITHOUT_FINAL_STATUS':'执行进程退出，未收到最终状态；不能标记任务成功',
 'VERIFYING_STABLE_CLEARANCE':'前后雷达新帧连续确认净空，至少0.8秒、3组后恢复',
 'CLEARANCE_OR_SHAPEMASK_NOT_CLEAR':'机械臂净空或ShapeMask未放行，停车等待',
 'ADMISSION_DEADLINE_EXPIRED':'计算结束时定位、关节或点云数据已超时',
 'JOINT_FEEDBACK_STALE':'关节角度过期',
 'POSE_MOVED_DURING_COLLISION_CHECK':'当前位姿尚未获得运动包络证明，停车重新复核',
 'SCAN_JOINT_EVIDENCE_REQUIRED':'等待雷达帧对应的完整实测姿态',
 'FRESH_S100_RECEIVED_JOINT_FEEDBACK_REQUIRED':'S100尚未收到有效的新关节角度',
 'LOCALIZATION_INITIALIZING':'正在建立首次雷达定位，输出为零',
 'LOCALIZATION_STALE':'定位数据过期',
 'LOCALIZATION_UNAVAILABLE':'定位尚未就绪',
 'BOTH_LIDARS_REQUIRED':'等待前后雷达后台处理结果',
 'FILTER_CONFIGURATION_CHANGED_REPLAN_REQUIRED':'滤除设置已变化，需要重新规划',
 'FRESH_DATA_RECOVERY_TIMEOUT':'等待新数据超时，已保持停车并退出',
 'WAIT_RAW_OBSERVER':'等待前后雷达原始采样就绪',
 'CABLE_PROBE_APPROACHES_UNCONFIRMED_RETURN':'起步方向会靠近尚未确认的近处点',
 'LIVE_WHOLE_BODY_COLLISION_OR_SELF_AMBIGUITY':'实时点云落入碰撞/自体待确认区域',
 'LIDAR_STALE':'雷达点云过期',
 'FULL_HEAD_ARM_GRIPPER_FEEDBACK_REQUIRED':'头部、双臂或夹爪反馈不完整',
 'OK':'检查通过'}
REASONS['PROBE_INITIAL_WIRES_DEFERRED_UNTIL_5CM']='初始线材候选仅记录，微移满5cm后判定'
REASONS['PROBE_DEFERRAL_OUTSIDE_BOUNDED_TEST']='微移已超出允许范围，停车'
REASONS['CABLE_PRECHECK_FOUND_STATIC_OBSTACLE']='已完成微移；检测到固定环境点，保持停车'

def arrival_text(status):
 if status.get('phase')!='ARRIVED' or not status.get('finished') or status.get('cleanup_pending'):return None
 if status.get('mode')=='REAL_MANUAL_START':return '导航任务完成：已到达目标，停车流程已结束。'
 if status.get('mode')=='PREVIEW_NO_ROS':return '零运动复核完成：定位结果满足目标到达条件。'
 return None

def explanation(status):
 reason=status.get('reason',status.get('error',''));text=REASONS.get(reason,reason)
 collision=status.get('collision_summary') or {};examples=collision.get('examples') or []
 error=status.get('goal_error')
 if error:
  text+=f"\n终点误差：位置{error['distance_m']*100:.2f}cm，朝向{error['heading_error_deg']:.2f}°。"
  if 'position_tolerance_m' in error:text+=f" 目标≤{error['position_tolerance_m']*100:g}cm / ≤{error['heading_tolerance_deg']:g}°。"
 if status.get('tracking_mode')=='goal_settling':text+=f"\n停车稳定确认：{status.get('settled_s',0):.1f}秒，{status.get('settle_samples',0)}组新定位。"
 note=status.get('control_note')
 if note:text+='\n'+REASONS.get(note,note)
 if status.get('cleanup_pending'):text+='\n正在结束停车和传感器采集流程。'
 if examples and reason=='LIVE_WHOLE_BODY_COLLISION_OR_SELF_AMBIGUITY':
  p=examples[0];height=p['point_body_m'][2]*100
  text+=f"\n拦截{collision.get('hit_count',len(examples))}点；首点高{height:.1f}cm，距{p['nearest_link']} {p['gap_cm']:.2f}cm。"
  if not p.get('inside_cleanup_height'):text+='该点超出近体滤除高度范围。'
  elif not p.get('inside_cleanup_band'):text+='该点超出近体滤除距离范围。'
 expired=status.get('expired_deadlines',[])
 if expired:
  labels={'localization_age_s':'定位','pose_age_s':'关节','cloud_age_s':'雷达','guard_age_s':'碰撞检查','clearance_age_s':'净空','operator_age_s':'操作连接'}
  text+='（'+ '、'.join(labels.get(k,k) for k in expired)+'）'
 waiting=status.get('obstacle_wait',{})
 if waiting.get('active'):
  text+=f"\n已停车等待{waiting.get('wait_s',0):.1f}秒；净空确认{waiting.get('clear_s',0):.1f}/{waiting.get('required_clear_s',.8):.1f}秒，新帧{waiting.get('clear_pairs',0)}/{waiting.get('required_pairs',3)}组。"
 recovery=status.get('route_recovery',{});metrics=recovery.get('anchor_metrics') or {};confirm=recovery.get('anchor_confirmation')
 if confirm and status.get('reason')=='LOCALIZATION_ANCHOR_CONFIRMING':
  text+=f"\n当前位置修正：{metrics.get('body_correction_m',0)*100:.2f}cm / {metrics.get('heading_correction_deg',0):.2f}°；独立新雷达证据{confirm.get('samples',0)}/3组。"
 counts=status.get('blocking_checks',{})
 if counts:
  details='；'.join(REASONS.get(k,k)+' ×'+str(v) for k,v in sorted(counts.items(),key=lambda x:-x[1])[:3])
  text+='\n本轮主要拦截：'+details
 return text
