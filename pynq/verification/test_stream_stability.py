import sys,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
PACKAGE=ROOT/'board_upload' if (ROOT/'board_upload').is_dir() else ROOT
sys.path.insert(0,str(PACKAGE))
from light_vision import VisionOverlay

class StableTextTests(unittest.TestCase):
    def overlay(self):
        clock=[.8];v=VisionOverlay(PACKAGE,lambda:clock[0]);v.thread=object();v.digit_thread=object();v.cv=cv2
        context=(12,1,1);v.confirmed=(0,'6',context)
        v.result=(0,[dict(label='DIGIT 6 86%',rect=(100,70,20,30),color=(0,255,0))],context)
        return v,clock,dict(pose_epoch=1,source_clock_epoch=1)
    def test_confirmed_text_survives_when_old_location_box_has_expired(self):
        v,clock,sample=self.overlay();frame=np.zeros((240,320,3),np.uint8)
        with patch.object(cv2,'putText',wraps=cv2.putText) as put:
            v.overlay(frame,sample,12)
        self.assertIn('LAST 6',[c.args[1] for c in put.call_args_list])
        self.assertEqual(frame[70:100,100:120].sum(),0)
        self.assertEqual(v.snapshot()['display_digit'],'6')
        clock[0]=2.1;self.assertIsNone(v.snapshot()['display_digit'])
    def test_new_run_does_not_show_old_text_or_box(self):
        v,clock,sample=self.overlay();clock[0]=.1;frame=np.zeros((240,320,3),np.uint8)
        v.overlay(frame,sample,13)
        self.assertIsNone(v.snapshot()['display_digit']);self.assertEqual(frame[70:100,100:120].sum(),0)
    def test_new_pose_origin_does_not_show_old_confirmation(self):
        v,clock,sample=self.overlay();sample['pose_epoch']=2
        v.overlay(np.zeros((240,320,3),np.uint8),sample,12)
        self.assertIsNone(v.snapshot()['display_digit'])

class ColdStartTests(unittest.TestCase):
    def test_missing_resident_metadata_is_handled_as_no_known_dma(self):
        from live_board import require_idle_video_dma
        mmio=unittest.mock.Mock()
        with patch.dict(sys.modules,{'pynq':SimpleNamespace(PL=SimpleNamespace(ip_dict=None),MMIO=mmio)}):
            require_idle_video_dma()
        mmio.assert_not_called()
    def test_known_running_dma_is_still_rejected(self):
        from live_board import require_idle_video_dma
        mmio=unittest.mock.Mock();mmio.return_value.read.return_value=1
        pl=SimpleNamespace(ip_dict={'video':dict(type='xilinx.com:ip:axi_vdma:6.3',phys_addr=0x43000000)})
        with patch.dict(sys.modules,{'pynq':SimpleNamespace(PL=pl,MMIO=mmio)}):
            with self.assertRaises(RuntimeError):require_idle_video_dma()

if __name__=='__main__':unittest.main()
