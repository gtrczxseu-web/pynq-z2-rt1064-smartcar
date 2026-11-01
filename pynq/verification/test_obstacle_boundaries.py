"""Regression cases for changing step/epoch and competing traffic signals."""
import sys,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];PACKAGE=ROOT/'board_upload' if (ROOT/'board_upload').is_dir() else ROOT
sys.path.insert(0,str(PACKAGE));sys.path.insert(0,str(Path(__file__).resolve().parent))
import test_visual_obstacle as fixtures


class BoundaryTests(unittest.TestCase):
    def setUp(self):self.f=fixtures.ControlTests();self.f.setUp()
    def colors(self,*labels):
        f=self.f;f.clock.now+=.15;f.fresh()
        if f.m.obstacle_state=='CLEAR' and f.clock()-f.m.obstacle_stamp>=.2:
            f.m.receive_obstacle(f.clock(),dict(state='CLEAR'),dict(pose_epoch=1,source_clock_epoch=0),123,f.index)
        f.m.receive_vision(f.clock(),[dict(label=label) for label in labels],session=123);f.m.step()
    def test_blocked_count_does_not_cross_step_boundary(self):
        f=self.f;f.m.recipe['steps'].append(dict(f.m.recipe['steps'][0]))
        f.obstacle('BLOCKED');f.fresh(index=1);f.obstacle('BLOCKED')
        self.assertFalse(f.m.obstacle_hold)
        f.obstacle('BLOCKED');self.assertTrue(f.m.obstacle_hold)
    def test_clear_count_does_not_cross_pose_epoch_boundary(self):
        f=self.f;f.held();f.clear();self.assertTrue(f.m.snapshot()['obstacle_clear'])
        f.clock.now+=.2;f.fresh();f.m.latest['pose_epoch']=2
        f.m.receive_obstacle(f.clock(),dict(state='CLEAR'),dict(pose_epoch=2,source_clock_epoch=0),123,0);f.m.step()
        self.assertFalse(f.m.snapshot()['obstacle_clear']);self.assertEqual(f.m.obstacle_clear_count,1)
    def test_clear_confirmation_from_another_step_cannot_release(self):
        f=self.f;f.m.recipe['steps'].append(dict(f.m.recipe['steps'][0]));f.held();f.clear()
        f.fresh(index=1);self.assertFalse(f.m.toggle());self.assertTrue(f.m.obstacle_hold)
    def test_green_beside_red_is_confirmed_after_red_disappears(self):
        f=self.f;f.m.recipe['mode']='continuous';f.held();f.clear()
        self.colors('RED','GREEN');self.colors('RED','GREEN')
        self.assertEqual(f.m.traffic_reason,'RED');self.assertTrue(f.m.obstacle_hold)
        self.colors('GREEN');self.colors('GREEN')
        self.assertIsNone(f.m.traffic_reason);self.assertFalse(f.m.obstacle_hold)
    def test_green_seen_during_yellow_wait_can_confirm_after_five_seconds(self):
        f=self.f;f.m.recipe['mode']='continuous';f.held();f.clear()
        f.m.traffic_reason='YELLOW';f.m.yellow_at=f.clock()
        self.colors('GREEN');self.colors('GREEN');self.assertTrue(f.m.obstacle_hold)
        for unused in range(25):f.obstacle('CLEAR')
        self.assertIsNone(f.m.traffic_reason)
        self.colors('GREEN');self.colors('GREEN');self.assertFalse(f.m.obstacle_hold)


if __name__=='__main__':unittest.main()
