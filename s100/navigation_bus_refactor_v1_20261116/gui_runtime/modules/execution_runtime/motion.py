"""Per-session immutable motion policy; no import-time files or globals."""
from dataclasses import dataclass
import math
from typing import Mapping


@dataclass(frozen=True)
class MotionLimits:
    translation_cap_mps: float
    unobserved_travel_budget_m: float
    lateral_cap_mps: float
    rotating_translation_cap_mps: float
    rotating_lateral_cap_mps: float
    rotating_travel_budget_m: float

    @classmethod
    def from_policy(cls, policy: Mapping):
        value = policy.get('navigation_test_limits', {})
        linear = float(value.get('translation_cap_mps', .03))
        lateral = float(value.get('lateral_cap_mps', .04))
        budget = float(value.get('unobserved_travel_budget_m', .04))
        result = cls(linear, budget, lateral,
                     float(value.get('rotating_translation_cap_mps', linear)),
                     float(value.get('rotating_lateral_cap_mps', lateral)),
                     float(value.get('rotating_unobserved_travel_budget_m', budget)))
        if not (.01 <= linear <= .10 and .02 <= budget <= .145 and .01 <= lateral <= .10):
            raise ValueError('INVALID_NAVIGATION_SPEED_LIMITS')
        if not (.01 <= result.rotating_translation_cap_mps <= linear and
                .01 <= result.rotating_lateral_cap_mps <= lateral and
                .02 <= result.rotating_travel_budget_m <= budget):
            raise ValueError('INVALID_ROTATING_SPEED_LIMITS')
        return result

    def limit(self, twist, cloud_age, radius):
        x, y, w = map(float, twist)
        if not all(math.isfinite(v) for v in (x, y, w, cloud_age, radius)) or cloud_age < 0 or radius < 0:
            raise ValueError('INVALID_TWIST_OR_OBSERVATION')
        rotating = w != 0.
        linear_cap = self.rotating_translation_cap_mps if rotating else self.translation_cap_mps
        lateral_cap = self.rotating_lateral_cap_mps if rotating else self.lateral_cap_mps
        travel_budget = self.rotating_travel_budget_m if rotating else self.unobserved_travel_budget_m
        scale = 1.
        for value, cap in ((x, .12 if x >= 0 else .1), (y, lateral_cap), (w, .2)):
            if abs(value) > cap:
                scale = min(scale, cap/abs(value))
        length = math.hypot(x, y)
        if length > linear_cap:
            scale = min(scale, linear_cap/length)
        x *= scale
        y *= scale
        w *= scale
        travel = (math.hypot(x, y)+radius*abs(w))*(max(1.2, cloud_age)+.25)
        if travel > travel_budget:
            scale = travel_budget/travel
            x *= scale
            y *= scale
            w *= scale
        return [x, y, w]


class NearSurfaceSpeed:
    """Conservative speed reduction only; never grants collision permission.

    Uses already self-filtered scan-body returns. Elevated external returns
    within 40cm of the measured robot envelope include table edges and chairs;
    this intentionally does not claim semantic table recognition.
    """
    def __init__(self):
        self.hold_until = 0.
        self.report = {}

    def apply(self, twist, prepared, shapes, cloud_age, now):
        import numpy as np
        unknown = not math.isfinite(cloud_age) or cloud_age > .75 or cloud_age < 0
        near = False
        if not unknown:
            try:
                if set(prepared) != {'front', 'back'} or not shapes:
                    raise ValueError('missing geometry')
                corners = np.concatenate([s['world_corners'] for s in shapes])
                if corners.ndim != 2 or corners.shape[1] != 3 or not np.isfinite(corners).all():
                    raise ValueError('invalid geometry')
                lo, hi = corners[:, :2].min(0), corners[:, :2].max(0)
                for item in prepared.values():
                    points = np.asarray(item['points_body'], dtype=float)
                    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
                        raise ValueError('invalid scan')
                    if not len(points):
                        raise ValueError('empty scan')
                    points = points[(points[:, 2] >= .35) & (points[:, 2] <= 1.8)]
                    gap = np.maximum(np.maximum(lo-points[:, :2], points[:, :2]-hi), 0.)
                    near |= bool(np.any(np.linalg.norm(gap, axis=1) <= .40))
            except (KeyError, TypeError, ValueError):
                unknown = True
        if near or unknown:
            self.hold_until = now + 2.
        active = near or unknown or now < self.hold_until
        result = list(map(float, twist))
        if active:
            result[0] *= .5
        self.report = dict(active=active, forward_reverse_scale=.5 if active else 1.,
                           trigger_distance_m=.40, elevated_return_near=near,
                           distance_unknown=unknown, release_hold_s=2.)
        return result
