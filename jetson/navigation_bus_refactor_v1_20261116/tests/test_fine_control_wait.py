"""The late-arrival controller can gather poses while motion stays inhibited."""
import json
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'planner_overrides')]
from endpoint_trim import EndpointTrim
from follower import PreviewFollower


class FineControlWaitTests(unittest.TestCase):
    def setUp(self):
        self.config = json.loads((ROOT / 'engine_clone/config/motion_policy.json').read_text())
        self.config['fine_goal'].update(enabled=True, strategy='pulse_observe')
        self.plan = dict(
            target=dict(pose=[1, 0, 0], revision=1),
            start_map_pose=[0, 0, 0],
            selected=dict(path=[[0, 0, 0], [1, 0, 0]], geometry_recheck_passed=True),
        )

    def test_first_recovered_cycle_can_propose_from_recent_pose_samples(self):
        follower = PreviewFollower(self.plan, self.config)
        for stamp in (0., .16, .32):
            follower.step([.94, 0, 0], stamp, stamp, 1, True, True, control_time=stamp)
            follower.pause('CONTROL_NOT_READY')
        self.assertIsNone(follower.trim.action)
        result = follower.step([.94, 0, 0], .48, .48, 1, True, True, control_time=.48)
        self.assertEqual(result['mode'], 'fine_translation')
        self.assertIsNone(follower.trim.action['start'])
        self.assertFalse(result['execution_permission'])

    def test_gap_or_guard_loss_discards_observations(self):
        for reason, recovery in [('CONTROL_NOT_READY', 1.0), ('GUARD_LOST', .32)]:
            follower = PreviewFollower(self.plan, self.config)
            for stamp in (0., .16):
                follower.step([.94, 0, 0], stamp, stamp, 1, True, True, control_time=stamp)
            follower.pause(reason)
            result = follower.step([.94, 0, 0], recovery, recovery, 1, True, True, control_time=recovery)
            self.assertEqual(result['control_note'], 'FINE_COLLECTING_NEW_POSES')
            self.assertEqual(result['twist_preview'], [0., 0., 0.])

    def test_started_pulse_loses_admission_and_needs_fresh_poses(self):
        trim = EndpointTrim([1, 0, 0], self.config['fine_goal'])
        for stamp in (0., .16, .32):
            trim.step([.94, 0, 0], [0, 0, 0], stamp, stamp)
        trim.admitted([0, 0, 0], .32)
        trim.pause(keep_observations=True)
        result = trim.step([.94, 0, 0], [0, 0, 0], .48, .48)
        self.assertEqual(result['twist_preview'], [0., 0., 0.])
        self.assertEqual(trim.pulses, 0)

    def test_feedback_collected_during_wait_does_not_count_as_settled(self):
        trim = EndpointTrim([1, 0, 0], self.config['fine_goal'])
        for i in range(11):
            stamp = i * .15
            result = trim.step([.97, 0, 0], [0, 0, 0], stamp, stamp)
            self.assertNotEqual(result['state'], 'ARRIVED')
            trim.pause(keep_observations=True)
        result = trim.step([.97, 0, 0], [0, 0, 0], 1.65, 1.65)
        self.assertEqual(result['state'], 'FOLLOW_PREVIEW')
        self.assertEqual(result['settle_samples'], 1)
