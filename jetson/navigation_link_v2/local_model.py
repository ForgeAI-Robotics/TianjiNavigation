"""S100-owned URDF and collision geometry; only joint values arrive from Jetson."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from link import JOINTS, POSE_TOLERANCE, number, vector, fresh
from collision_geometry import volumes
from table_approach import table_geometry


def default_urdf_path():
    home = Path.home()
    candidates = [home/'tianji_state_bridge/description_install/share/tianji_robot_description/urdf/robot.urdf',
                  home/'tianji_robot_state_bridge/description_install/share/tianji_robot_description/urdf/robot.urdf']
    return next((p for p in candidates if p.exists()), candidates[0])


class LocalRobotModel:
    def __init__(self, urdf, manifest, collision_geometry):
        self.urdf = Path(urdf).resolve(strict=True)
        self.manifest = json.loads(Path(manifest).read_text())
        self.geometry = json.loads(Path(collision_geometry).read_text())
        self.sha256 = hashlib.sha256(self.urdf.read_bytes()).hexdigest()
        if (self.sha256 != self.manifest['portable_urdf_sha256']
                or self.geometry.get('urdf_sha256') != self.sha256):
            raise ValueError('Local URDF/manifest/collision geometry mismatch')
        names = {j.attrib['name'] for j in ET.parse(self.urdf).getroot().findall('joint')}
        if not set(JOINTS).issubset(names):
            raise ValueError('Local model lacks required arm/head/gripper joints')

    def shapes(self, joint_positions):
        if not isinstance(joint_positions, dict) or any(
                not number(v) for v in joint_positions.values()):
            raise ValueError('Invalid measured joint positions')
        # The existing geometry implementation bounds all rotations when a
        # joint is absent. It never substitutes a zero angle or open gripper.
        return volumes(self.geometry, joint_positions)

    def table_constraint(self, joint_positions, table_scene, observation_time,
                         generated_at, base_xy_yaw):
        if not number(observation_time) or not number(generated_at) or not vector(base_xy_yaw, 3):
            raise ValueError('Explicit observation timestamp/base pose required')
        result = table_geometry(self.shapes(joint_positions), table_scene, margin=.05)
        complete = all(name in joint_positions for name in JOINTS)
        result.update(whole_model_sha256=self.sha256,
                      joint_positions_rad={n: joint_positions[n] for n in JOINTS if n in joint_positions},
                      observation_time=observation_time, generated_at=generated_at,
                      base_xy_yaw=list(base_xy_yaw),
                      blocking_link=(result.get('blocking') or {}).get('link'),
                      local_urdf_verified=True, pose_complete=complete)
        if not complete:
            result.update(valid=False, reason='FULL_POSE_REQUIRED_FOR_FORWARD_ESTIMATE')
        return result


class TableProjector:
    """Project an observed tabletop against S100's own model and Jetson pose.

    Successful observations are bound once. New packets must not move the
    reference base pose or silently regenerate an old scene at a new arm pose.
    """
    def __init__(self, model):
        self.model = model
        self.observation = None
        self.cached = None

    def observe(self, observation):
        # A rejected new observation invalidates the previous constraint.
        self.cached = None
        self.observation = None
        if not isinstance(observation, dict):
            raise ValueError('TABLE_OBSERVATION_OBJECT_REQUIRED')
        if observation.get('frame_id') != 'tianji/chassis_base':
            raise ValueError('TABLE_OBSERVATION_FRAME_MISMATCH')
        scene = observation.get('scene')
        if (not isinstance(scene, dict) or scene.get('valid') is not True
                or not vector(scene.get('plane'), 3) or not vector(scene.get('edge'), 2)
                or not vector(scene.get('y_range'), 2)
                or scene['y_range'][0] >= scene['y_range'][1]
                or not number(observation.get('observation_time'))
                or not vector(observation.get('base_xy_yaw'), 3)):
            raise ValueError('TABLE_OBSERVATION_GEOMETRY_INVALID')
        import copy
        self.observation = copy.deepcopy(observation)

    def constraint(self, packet, now):
        o = self.observation
        if o is None:
            return dict(valid=False, reason='NO_LOCAL_TABLE_OBSERVATION')
        if not fresh(now, o['observation_time'], .65):
            return dict(valid=False, reason='LOCAL_TABLE_OBSERVATION_EXPIRED')
        if self.cached is not None:
            return self.cached
        try:
            if packet['whole_model_sha256'] != self.model.sha256:
                raise ValueError('LOCAL_MODEL_AND_JETSON_VERSION_DIFFER')
            positions = {}
            for name in JOINTS:
                sample = packet['pose'][name]
                keys = ('position', 'minimum', 'maximum', 'covered_from', 'stamp')
                if (not all(number(sample.get(k)) for k in keys)
                        or not sample['covered_from'] <= o['observation_time'] <= sample['stamp']
                        or not fresh(now, sample['stamp'], .25)
                        or not sample['minimum'] <= sample['position'] <= sample['maximum']
                        or sample['maximum']-sample['minimum'] > POSE_TOLERANCE):
                    raise ValueError('LOCAL_TABLE_POSE_HISTORY_MISSING_OR_MOVED:' + name)
                positions[name] = sample['position']
            self.cached = self.model.table_constraint(positions, o['scene'],
                                o['observation_time'], now, o['base_xy_yaw'])
            return self.cached
        except (KeyError, TypeError, AttributeError, ValueError) as error:
            return dict(valid=False, reason=str(error))
