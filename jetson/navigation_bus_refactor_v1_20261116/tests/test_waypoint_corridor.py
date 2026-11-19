"""Waypoint choices are optional graph nodes; obstacles and floor stay intact."""
import copy
import math
import unittest
from planner_overrides.waypoint_corridor_graph import propose


class CorridorGraphTests(unittest.TestCase):
    def packet(self):return dict(goal=[.39,-2.3,0.],world={'occupied_points':[[.2,-.5,.1]]},grid=dict(origin=[-2.,-4.],resolution=.05,width=120,height=120,data=[0]*14400),motion_policy=dict(short_translation_goal_distance_m=1.5,short_rectilinear_max_length_m=2.,short_rectilinear_enabled=True,lateral_distance_cost_multiplier=3.,lateral_activation_cost_m=.3,fine_goal={'heading_tolerance_deg':1.}))
    def obstacle(self,path):return not any(.08<p[0]<.30 and -.9<p[1]<.1 for p in path)
    def nodes(self,identifier=2):return [dict(id=identifier,point=[.36,-1.],revision=10)]
    def test_goes_via_optional_node_when_direct_route_is_blocked(self):
        packet=self.packet();nodes=self.nodes();before=copy.deepcopy((packet,nodes));selected,report=propose(packet,nodes,self.obstacle)
        self.assertEqual(selected['via_waypoints'],[2]);self.assertTrue(selected['independently_rechecked'])
        self.assertTrue(self.obstacle(selected['path']));self.assertEqual((packet,nodes),before)
        self.assertLess(min(math.hypot(p[0]-.36,p[1]+1.)for p in selected['path']),1e-8)
        self.assertEqual(selected['maximum_local_edge_m'],1.5)
    def test_does_not_hardcode_waypoint_two(self):
        selected,_=propose(self.packet(),self.nodes(7),self.obstacle);self.assertEqual(selected['via_waypoints'],[7])
    def test_real_obstacle_at_intermediate_waypoint_is_retained(self):
        def blocked(path):return self.obstacle(path)and not any(math.hypot(p[0]-.36,p[1]+1.)<.10 for p in path)
        selected,_=propose(self.packet(),self.nodes(),blocked);self.assertIsNone(selected)
    def test_unknown_ground_cannot_be_filled_in_at_virtual_waypoint(self):
        packet=self.packet();x=math.floor((.36+2.)/.05);y=math.floor((-1.+4.)/.05);packet['grid']['data'][y*120+x]=-1
        selected,_=propose(packet,self.nodes(),self.obstacle);self.assertIsNone(selected)
    def test_clear_long_direct_scene_retains_standard_planner_preference(self):
        selected,report=propose(self.packet(),self.nodes(),lambda path:True)
        self.assertIsNone(selected);self.assertEqual(report['status'],'DIRECT_CORRIDOR_CLEAR_USE_STANDARD_PLANNER')
    def test_no_intermediate_cannot_invent_a_long_slide(self):
        selected,_=propose(self.packet(),[],self.obstacle);self.assertIsNone(selected)
    def test_final_rotation_is_guarded(self):
        packet=self.packet();packet['goal'][2]=.1
        def guard(path):return self.obstacle(path)and all(abs(p[2])<.05 for p in path)
        selected,_=propose(packet,self.nodes(),guard);self.assertIsNone(selected)
    def test_each_edge_preserves_local_distance_limit(self):
        selected,_=propose(self.packet(),[dict(id=2,point=[.36,-1.7],revision=1)],self.obstacle)
        self.assertIsNone(selected)


if __name__=='__main__':unittest.main()
