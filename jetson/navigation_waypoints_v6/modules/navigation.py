"""CLI for passive localization, editable goals and Hybrid A* navigation preview."""
import argparse,json,time,math,sys
from pathlib import Path
from paths import ROOT
from common import atomic_json,exclusive
import targets
from pillar_cables import settings as cable_settings,set_enabled as set_cable_ignore
from global_localization import localize
from prepare import prepare
from policy import plan

def show_targets():
 for row in targets.load()['targets'].values():
  x,y,a=row['pose'];print(f'{row["id"]}：{row["name"]}  X={x:.3f} Y={y:.3f}m 朝向={math.degrees(a):.2f}°  版本{row["revision"]}')

def run(goal_id,assume=True,localization_folder=None):
 goal=targets.get(goal_id);out=ROOT/'results'/('goal_'+str(goal_id)+'_'+time.strftime('%Y%m%d_%H%M%S')+'_'+str(time.time_ns())[-6:]);out.mkdir(parents=True)
 atomic_json(ROOT/'runtime/plan_state.json',dict(state='LOCALIZING',target=goal,execution_permission=False))
 if localization_folder is None:
  print('只读采集双雷达，全图搜索当前位置…',flush=True);loc=localize(Path(goal['map_folder']),out/'localization')
 else:
  # Explicit replay only. Never relabel the frozen capture as a current pose.
  import shutil
  source=Path(localization_folder);shutil.copytree(source,out/'localization',symlinks=True);loc=json.loads((out/'localization/localization.json').read_text());loc['offline_replay']=True
  acquisition_file=out/'localization/acquisition.json';acquisition=json.loads(acquisition_file.read_text())
  for row in acquisition['rows']:
   copied=out/'localization'/Path(row['file']).name
   if copied.exists():row.update(original_file=row['file'],file=str(copied))
  atomic_json(acquisition_file,acquisition)
 print('地图位置：'+str(loc['pose'])+'；开始Hybrid A*整机扫掠检查。',flush=True)
 recorded,world=prepare(loc,goal,out/'scene',assume_clearance=assume)
 atomic_json(ROOT/'runtime/plan_state.json',dict(state='PLANNING',target=goal,execution_permission=False))
 result=plan(recorded,world,out/'scene/frozen_input.json',out/'planning',goal)
 atomic_json(ROOT/'runtime/latest_plan.json',dict(folder=str(out),result=str(out/'planning/plan.json')))
 atomic_json(ROOT/'runtime/plan_state.json',dict(state=result['state'],folder=str(out),target=goal,execution_permission=False))
 try:
  from render_plan import render
  render(out)
 except Exception as exc:print('路线图输出失败：'+str(exc),flush=True)
 print(result['state']+'：'+str(out),flush=True)
 if result['selected']:
  route=result['selected'];print(f"模式 {route['profile']}；路径 {route['length_m']:.2f}m；横移累计 {route['lateral_distance_m']:.2f}m；末端位置误差 {route['goal_error_m']:.3f}m",flush=True)
 elif result.get('position_reached'):print('位置已在目标容差内，无需移动；近场几何冲突仍保持运动禁止。',flush=True)
 else:print('没有找到合规路线；未以手画路线替代。',flush=True)
 return result

def main():
 from self_envelope import settings as self_settings,set_enabled as set_self_enabled
 parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--goal',type=int,choices=[1,2,3]);parser.add_argument('--assume-clearance',action='store_true');parser.add_argument('--replay',type=Path);parser.add_argument('--snapshot-test',action='store_true',help='只读采集新环境，测试冻结局部快照，不使用旧目标');args=parser.parse_args()
 if args.snapshot_test and (args.goal or args.replay):parser.error('--snapshot-test 不能与 --goal/--replay 同用')
 lock=exclusive(ROOT/'runtime/navigation.lock');targets.initialize()
 if args.snapshot_test:
  from snapshot_planner_test import run as run_snapshot_test
  run_snapshot_test();return
 if args.goal:
  if not args.assume_clearance:raise ValueError('需要显式--assume-clearance；本阶段只做净空假设预览')
  run(args.goal,True,args.replay);return
 print('Hybrid A* 导航开发 · 只读预览 · 双臂净空按用户假设满足 · 底盘输出未连接。')
 print('长距离优先差速式前进/倒车/转弯；仅短距离直接平移或复杂区域局部横移。')
 print('扩大自我滤除包络：'+('开启' if self_settings().get('enabled') else '关闭')+'；碰撞体保持原值。')
 print('临时立柱线材忽略：'+('开启' if cable_settings().get('enabled') else '关闭')+'（仅导航预览的当前雷达）')
 print('命令：targets | set 1/2 X Y 朝向度 | plan 1/2 | cables on/off/status | self on/off/status | localize | status | quit')
 show_targets()
 while True:
  try:parts=input('导航预览> ').split()
  except (EOFError,KeyboardInterrupt):break
  if not parts:continue
  try:
   if parts[0] in ['q','quit','exit']:break
   if parts[0]=='targets':show_targets()
   elif parts[0]=='set' and len(parts)==5:
    row=targets.update(int(parts[1]),*map(float,parts[2:]));print('目标已更新，旧路线失效：',row['pose'])
   elif parts[0]=='plan' and len(parts)==2:run(int(parts[1]))
   elif parts[0]=='self' and len(parts)==2:
    if parts[1] in ['on','off']:set_self_enabled(parts[1]=='on')
    elif parts[1]!='status':raise ValueError('self on/off/status')
    print(json.dumps(self_settings(),ensure_ascii=False,indent=2))
   elif parts[0]=='cables' and len(parts)==2:
    if parts[1] in ['on','off']:set_cable_ignore(parts[1]=='on')
    elif parts[1]!='status':raise ValueError('cables on/off/status')
    print(json.dumps(cable_settings(),ensure_ascii=False,indent=2))
   elif parts[0]=='localize':
    out=ROOT/'results'/('localize_'+time.strftime('%Y%m%d_%H%M%S'));loc=localize(Path(targets.get(1)['map_folder']),out/'localization');print(loc['pose'])
   elif parts[0]=='status':
    show_targets();print((ROOT/'runtime/plan_state.json').read_text() if (ROOT/'runtime/plan_state.json').exists() else 'IDLE；未连接执行链')
   else:print('用法：targets / set 1或2 X Y 朝向度 / plan 1或2 / localize / status / quit')
  except Exception as exc:
   atomic_json(ROOT/'runtime/plan_state.json',dict(state='REJECTED',reason=str(exc),execution_permission=False));print('未放行：'+str(exc),flush=True)

if __name__=='__main__':main()
