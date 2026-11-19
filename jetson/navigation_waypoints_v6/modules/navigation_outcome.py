"""Human display of immutable downstream arrival evidence, no state override."""
import math

def explain(status):
 failures=status.get('failure_history')or[]
 failure=status.get('failure_detail')or(failures[-1]if failures else{})
 downstream=failure.get('downstream')or{};result=downstream.get('result')or{}
 original=(downstream.get('error')or{}).get('details',{}).get('original_result')or{}
 phase=original.get('navigator_phase')
 if phase!='STOP_FINAL_POSE_OUTSIDE_TOLERANCE'and original.get('code')!='STOP_FINAL_POSE_OUTSIDE_TOLERANCE':return ''
 error=result.get('goal_error')or original.get('goal_error')or{}
 distance=error.get('distance_m');heading=error.get('heading_error_deg')
 if any(type(x)not in(int,float)or not math.isfinite(x)or x<0 for x in (distance,heading)):return ''
 stop='已停车'if result.get('navigation_stopped')is True else'停止尚未确认'
 return f'{stop}；停车后位置误差{distance*100:.2f}cm（要求≤5cm），朝向{heading:.2f}°（要求≤1°）。未通过最终到位验收。'
