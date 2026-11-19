"""Join occupied geometry, measured posture, table semantics and camera evidence."""
import math
import hashlib
import json
import numpy as np
from link import JOINTS, POSE_TOLERANCE, fresh, number, vector
from sweep import check_sweep


def candidate_digest(candidate):
    return hashlib.sha256(json.dumps(candidate,sort_keys=True,allow_nan=False).encode()).hexdigest()


def table_relationship(shapes, scene):
    if not scene.get('valid'):
        return dict(state='NO_SUPPORTED_TABLE', table_observed=False)
    a,b,c = scene['plane']; k,d = scene['edge']; low,high = scene['y_range']
    above,below,crossing = set(),set(),set()
    overlap = set()
    for shape in shapes:
        corners = np.asarray(shape['world_corners'])
        gap = corners[:,2]-a*corners[:,0]-b*corners[:,1]-c
        group = above if gap.min()>.05 else below if gap.max()<-.05 else crossing
        group.add(shape['link'])
        if (corners[:,1].max()>=low and corners[:,1].min()<=high
                and (corners[:,0]-k*corners[:,1]).max()>=d
                and corners[:,0].min()<=scene.get('observed_x_range',[d,3.])[1]):
            overlap.add(shape['link'])
    # Classify chassis overlap and arm height separately; a column remains a
    # potential blocker even when the low chassis has entered under a tabletop.
    under = 'YD_link' in below and 'YD_link' in overlap
    return dict(state='CHASSIS_UNDER_OBSERVED_TABLE' if under else 'TABLE_NEARBY',
                table_observed=True, table_height_at_origin_m=c,
                chassis_under_table=under, above_plane_links=sorted(above),
                below_plane_links=sorted(below), crossing_height_links=sorted(crossing),
                tabletop_xy_overlap_links=sorted(overlap),
                underside_thickness_known=False,
                scope='plane/edge relationship, not a semantic free-space certificate')


def assess_candidate(model, link, environment, scene_constraint, odom, candidate, now, mono, entrance_guard=None):
    result = dict(schema_version=1, evaluated_at=now, state='UNKNOWN', motion_inhibit=True,
                  candidate_id=None, reasons=[], advisory_only=True,
                  command_path_connected=False, hardware_stop_connected=False,
                  general_navigation_permission=False)
    reasons = result['reasons']
    try:
        if not isinstance(candidate,dict) or candidate.get('schema_version')!=1:
            raise ValueError('CANDIDATE_SCHEMA_INVALID')
        result['candidate_id'] = candidate.get('candidate_id')
        if not isinstance(result['candidate_id'],str) or not 1<=len(result['candidate_id'])<=128:
            raise ValueError('CANDIDATE_ID_REQUIRED')
        if not fresh(now,candidate.get('created_at'),.3):
            raise ValueError('CANDIDATE_EXPIRED_OR_FUTURE')
        result['candidate_digest']=candidate_digest(candidate)
        result['candidate_created_at']=candidate['created_at']
        delta = candidate.get('delta_base')
        if (not vector(delta,3) or not number(candidate.get('duration_s'))
                or not .05<=candidate['duration_s']<=2.):
            raise ValueError('CANDIDATE_DELTA_OR_DURATION_INVALID')
        if candidate.get('model_sha256') != model.sha256:
            raise ValueError('CANDIDATE_MODEL_MISMATCH')
        measured = link.model_joint_state(now,mono)
        positions = {n:v['position'] for n,v in measured.items()}
        if set(positions)!=set(JOINTS):
            reasons.append('FULL_MEASURED_POSE_UNAVAILABLE')
        entrance_shapes=None
        if candidate.get('mode')=='fixed_posture_entrance':
            if entrance_guard is None:raise ValueError('ENTRANCE_GUARD_NOT_CONFIGURED')
            entrance,entrance_shapes=entrance_guard.check(candidate,positions)
            result['entrance_posture']=entrance
            if entrance_shapes is None:raise ValueError(entrance['reason'])
        expected = candidate.get('joint_positions_rad')
        if (not isinstance(expected,dict) or set(expected)!=set(JOINTS) or any(
                not number(expected[n]) or n not in positions
                or abs(expected[n]-positions[n])>POSE_TOLERANCE for n in JOINTS)):
            reasons.append('CANDIDATE_POSE_MISMATCH')
        if environment is None or not fresh(now,environment.stamp,.65):
            raise ValueError('ENVIRONMENT_UNAVAILABLE_OR_EXPIRED')
        original_pose = environment.summary.get('joint_positions_rad',{})
        if set(original_pose)!=set(JOINTS) or any(n not in positions or
                abs(original_pose[n]-positions[n])>POSE_TOLERANCE for n in JOINTS):
            reasons.append('POSE_CHANGED_SINCE_ENVIRONMENT_OBSERVATION')
        if not isinstance(odom,dict) or not fresh(now,odom.get('stamp'),.2) or not vector(odom.get('xy_yaw'),3):
            raise ValueError('ODOMETRY_UNAVAILABLE_OR_EXPIRED')
        base,reference = odom['xy_yaw'],environment.base_xy_yaw
        c,s = math.cos(reference[2]),math.sin(reference[2])
        x,y = base[0]-reference[0],base[1]-reference[1]
        yaw = math.atan2(math.sin(base[2]-reference[2]),math.cos(base[2]-reference[2]))
        start = [c*x+s*y,-s*x+c*y,yaw]
        cy,sy = math.cos(yaw),math.sin(yaw)
        end = [start[0]+cy*delta[0]-sy*delta[1],start[1]+sy*delta[0]+cy*delta[1],yaw+delta[2]]
        shapes = entrance_shapes if entrance_shapes is not None else model.shapes(positions)
        sweep = check_sweep(shapes,environment,end,now,start_delta=start)
        transformed_shapes=[]
        c0,s0=math.cos(start[2]),math.sin(start[2])
        r0=np.array([[c0,-s0,0.],[s0,c0,0.],[0.,0.,1.]])
        for shape in shapes:
            shape=dict(shape)
            shape['world_corners']=(np.asarray(shape['world_corners'])@r0.T+[start[0],start[1],0.]).tolist()
            transformed_shapes.append(shape)
        result.update(sweep=sweep, observed_geometry_state=sweep['state'],
                      table_relationship=table_relationship(transformed_shapes,environment.summary.get('table',{})))
        if sweep['collision']:
            result.update(state='COLLISION_RISK', motion_inhibit=True)
            reasons.append('OBSERVED_OBSTACLE_INTERSECTS_INFLATED_SWEPT_BODY')
            return result
        if sweep['state']=='UNKNOWN':
            reasons.extend(sweep['reasons'])
        if sweep.get('current_self_or_contact_ambiguous'):
            reasons.append('SELF_RETURNS_OR_CURRENT_CONTACT_UNRESOLVED')
        # Side/reverse/rotation collision detection works, but the existing
        # camera forward-plane contract does not certify these directions.
        if delta[0]<0 or abs(delta[1])>1e-6 or abs(delta[2])>1e-6:
            reasons.append('CAMERA_COVERAGE_DOES_NOT_CERTIFY_THIS_DIRECTION')
        forward = link.evaluate(now,mono,scene_constraint,odom)
        result['forward_constraint'] = forward
        if forward['state']=='BLOCKED' or forward['stop_request']:
            result['state']='COLLISION_RISK'
            reasons.append('JETSON_CAMERA_OR_TABLE_REPORTS_BLOCKED')
            reasons.extend(forward['reasons'])
            return result
        if forward['forward_inhibit']:
            reasons.extend(forward['reasons'] or ['FORWARD_CONSTRAINT_INHIBIT'])
        if delta[0]>forward['forward_remaining_m']+1e-9:
            reasons.append('CANDIDATE_EXCEEDS_FORWARD_DISTANCE_BOUND')
        # No ray/free-space evidence is fabricated from absent point returns.
        reasons.append('UNOBSERVED_VOLUME_NOT_CERTIFIED')
        result['state'] = 'UNKNOWN' if reasons else 'OBSERVED_GEOMETRY_CLEAR'
        result['reasons'] = list(dict.fromkeys(reasons))
        return result
    except (ValueError,TypeError,KeyError,AttributeError,IndexError,OverflowError) as error:
        reasons.append(str(error))
        return result
