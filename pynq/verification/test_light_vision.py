import hashlib,sys,time,unittest,tempfile
from pathlib import Path
from unittest.mock import patch
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
PACKAGE=ROOT/'board_upload' if (ROOT/'board_upload').is_dir() else ROOT
sys.path.insert(0,str(PACKAGE))
from light_vision import Recognizer,VisionOverlay,color_masks,digit_input


class VisionTests(unittest.TestCase):
    def test_model_identity(self):
        b=(PACKAGE/'models/mnist-8.onnx').read_bytes()
        self.assertEqual(len(b),26454)
        self.assertEqual(hashlib.sha256(b).hexdigest(),'2f06e72de813a8635c9bc0397ac447a601bdbfa7df4bebc278723b958831c9bf')

    def test_color_threshold_boundaries_do_not_overflow_uint8(self):
        rgb=np.array([[[100,60,60],[99,0,0],[100,61,0],[255,0,0],[0,255,0],[0,0,255],[255,255,255]]],dtype=np.uint8)
        masks=color_masks(rgb[:,:,::-1])
        self.assertEqual([m.tolist() for m in masks],[[[True,False,False,True,False,False,False]],[[False,False,False,False,True,False,False]],[[False,False,False,False,False,True,False]]])

    def test_uniform_frames_have_no_digit(self):
        for value in (0,127,255):
            self.assertIsNone(digit_input(np.full((240,320,3),value,np.uint8),cv2))

    def test_upstream_numerical_fixture_matches_logits(self):
        with tempfile.TemporaryDirectory() as tmp:
            for name in ('input_0.pb','output_0.pb'):
                (Path(tmp)/name).write_bytes((PACKAGE/'verification/data'/name).read_bytes())
            x=cv2.dnn.readTensorFromONNX(str(Path(tmp)/'input_0.pb'))
            expected=cv2.dnn.readTensorFromONNX(str(Path(tmp)/'output_0.pb'))
        net=Recognizer(PACKAGE).net;net.setInput(x)
        np.testing.assert_allclose(net.forward(),expected,rtol=1e-5,atol=1e-4)

    def scored_digit(self,probability):
        frame=np.full((240,320,3),255,np.uint8)
        cv2.putText(frame,'6',(125,165),cv2.FONT_HERSHEY_SIMPLEX,3,(0,0,0),6,cv2.LINE_AA)
        recognizer=Recognizer(PACKAGE)
        class Scores:
            def setInput(self,tensor):pass
            def forward(self):
                scores=np.full(10,1e-9,np.float32);scores[6]=probability;scores[0]=1-probability
                return np.log(scores).reshape(1,10)
        recognizer.net=Scores()
        return recognizer,frame

    def test_seventy_five_percent_digit_requires_two_matching_frames(self):
        r,frame=self.scored_digit(.75)
        self.assertFalse(any(d['label'].startswith('DIGIT') for d in r.detect(frame)))
        self.assertTrue(any(d['label'].startswith('DIGIT 6 ') for d in r.detect(frame)))

    def test_digit_below_seventy_percent_remains_unconfirmed(self):
        r,frame=self.scored_digit(.65)
        for unused in range(3):self.assertFalse(any(d['label'].startswith('DIGIT') for d in r.detect(frame)))

    def test_ambiguous_thin_six_is_not_forced_to_zero(self):
        frame=cv2.imdecode(np.frombuffer((PACKAGE/'verification/data/digit_thin_6_20261027.png').read_bytes(),np.uint8),cv2.IMREAD_COLOR)
        r=Recognizer(PACKAGE)
        for unused in range(3):self.assertFalse(any(d['label'].startswith('DIGIT') for d in r.detect(frame)))

    def test_synthetic_card_is_classified_after_two_frames(self):
        for digit in (1,2):
            r=Recognizer(PACKAGE);frame=np.full((240,320,3),255,np.uint8)
            cv2.putText(frame,str(digit),(130,160),cv2.FONT_HERSHEY_SIMPLEX,3,(0,0,0),6,cv2.LINE_AA)
            self.assertFalse(any(d['label'].startswith('DIGIT') for d in r.detect(frame)))
            self.assertTrue(any(d['label'].startswith('DIGIT %d '%digit) for d in r.detect(frame)))
            r.detect(np.zeros_like(frame))
            self.assertFalse(any(d['label'].startswith('DIGIT') for d in r.detect(frame)))

    def test_color_box_coordinates(self):
        frame=np.full((240,320,3),180,np.uint8);frame[30:60,10:35]=(0,0,255)
        found=Recognizer(PACKAGE).detect(frame)
        self.assertEqual([(d['label'],d['rect']) for d in found],[('RED',(10,30,25,30))])

    def test_colored_blocks_are_not_classified_as_black_digits(self):
        for label,color in [('RED',(0,0,255)),('GREEN',(0,180,0)),('BLUE',(255,0,0))]:
            frame=np.full((240,320,3),255,np.uint8)
            frame[90:151,130:161]=color
            recognizer=Recognizer(PACKAGE);recognizer.detect(frame)
            found=recognizer.detect(frame)
            self.assertEqual([d['label'] for d in found],[label])

    def test_small_white_card_is_found_inside_a_dark_scene(self):
        for digit in (1,2,3):
            frame=np.full((240,320,3),65,np.uint8)
            frame[55:196,105:216]=220
            cv2.putText(frame,str(digit),(130,160),cv2.FONT_HERSHEY_SIMPLEX,2,(25,25,25),4,cv2.LINE_AA)
            recognizer=Recognizer(PACKAGE);recognizer.detect(frame)
            found=recognizer.detect(frame)
            self.assertTrue(any(d['label'].startswith('DIGIT %d '%digit) for d in found),digit)

    def test_background_blue_is_rejected_but_an_interior_card_remains(self):
        frame=np.full((240,320,3),180,np.uint8)
        frame[0:100]=(255,0,0)  # Large wall touching the frame.
        frame[120:155,130:165]=(255,0,0)
        found=Recognizer(PACKAGE).detect(frame)
        self.assertEqual([(d['label'],d['rect']) for d in found],[('BLUE',(130,120,35,35))])
        frame[:]=180;frame[20:220,60:260]=(255,0,0)
        self.assertEqual(Recognizer(PACKAGE).detect(frame),[])

    def test_low_saturation_blue_cast_is_not_a_color_card(self):
        frame=np.full((240,320,3),180,np.uint8);frame[60:100,120:165]=(200,155,145)
        self.assertEqual(Recognizer(PACKAGE).detect(frame),[])

    def test_captured_dark_blue_book_with_printed_cover_is_detected(self):
        path=PACKAGE/'verification/data/blue_book_20261027.png'
        frame=cv2.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv2.IMREAD_COLOR)
        colors=[d for d in Recognizer(PACKAGE).detect(frame) if not d['label'].startswith('DIGIT')]
        self.assertEqual([d['label'] for d in colors],['BLUE'])
        x,y,w,h=colors[0]['rect']
        self.assertLessEqual(abs(x-73),3);self.assertLessEqual(abs(y-67),3)
        self.assertLessEqual(abs(w-102),3);self.assertLessEqual(abs(h-130),3)

    def test_small_print_gaps_do_not_fragment_a_color_card(self):
        frame=np.full((240,320,3),180,np.uint8);frame[60:110,100:150]=(255,0,0)
        frame[60:110:2,100:150:2]=180
        colors=[d for d in Recognizer(PACKAGE).detect(frame) if d['label']=='BLUE']
        self.assertEqual([d['rect'] for d in colors],[(100,60,50,50)])

    def test_nearly_black_saturated_noise_is_not_a_color_card(self):
        frame=np.full((240,320,3),180,np.uint8);frame[60:110,100:150]=(45,0,0)
        self.assertEqual(Recognizer(PACKAGE).detect(frame),[])

    def test_captured_warm_red_book_is_detected(self):
        frame=cv2.imdecode(np.frombuffer((PACKAGE/'verification/data/red_book_20261027.png').read_bytes(),np.uint8),cv2.IMREAD_COLOR)
        colors=[d for d in Recognizer(PACKAGE).detect(frame) if d['label'] in ('RED','GREEN','BLUE')]
        self.assertEqual([d['label'] for d in colors],['RED'])
        x,y,w,h=colors[0]['rect']
        self.assertLessEqual(abs(x-97),4);self.assertLessEqual(abs(y-56),4)
        self.assertLessEqual(abs(w-99),4);self.assertLessEqual(abs(h-152),4)

    def test_orange_and_yellow_regions_are_not_labeled_red(self):
        for hue in (24,30):
            frame=np.full((240,320,3),180,np.uint8)
            patch=np.full((50,50,3),(hue,220,180),np.uint8)
            frame[70:120,100:150]=cv2.cvtColor(patch,cv2.COLOR_HSV2BGR)
            self.assertEqual([d['label'] for d in Recognizer(PACKAGE).detect(frame)],['YELLOW'])

    def test_captured_stationary_background_has_no_target(self):
        path=PACKAGE/'verification/data/stationary_background_20261027.png'
        frame=cv2.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv2.IMREAD_COLOR)
        recognizer=Recognizer(PACKAGE)
        for unused in range(3):self.assertEqual(recognizer.detect(frame),[])

    def test_captured_shaded_paper_digit_two_is_recognized(self):
        path=PACKAGE/'verification/data/digit_card_2_20261027.png'
        frame=cv2.imdecode(np.frombuffer(path.read_bytes(),np.uint8),cv2.IMREAD_COLOR)
        recognizer=Recognizer(PACKAGE)
        self.assertFalse(any(d['label'].startswith('DIGIT') for d in recognizer.detect(frame)))
        for unused in range(3):
            found=recognizer.detect(frame)
            self.assertEqual([d['label'].split()[1] for d in found if d['label'].startswith('DIGIT')],['2'])

    def test_digit_ink_is_centered_by_mass(self):
        frame=np.full((240,320,3),255,np.uint8)
        cv2.putText(frame,'2',(130,160),cv2.FONT_HERSHEY_SIMPLEX,3,(0,0,0),6,cv2.LINE_AA)
        tensor,rect=digit_input(frame,cv2);m=cv2.moments(tensor.reshape(28,28))
        self.assertAlmostEqual(m['m10']/m['m00'],13.5,delta=.15)
        self.assertAlmostEqual(m['m01']/m['m00'],13.5,delta=.15)

    def test_slow_inference_does_not_block_camera_and_stale_boxes_expire(self):
        def slow_colors(frame,cv):
            time.sleep(.15)
            return [dict(label='TEST',rect=(10,20,30,40),color=(0,255,0))]
        clock=[0.0]
        with patch('light_vision.detect_colors',slow_colors):
            v=VisionOverlay(PACKAGE,lambda:clock[0]);frame=np.zeros((240,320,3),np.uint8)
            try:
                began=time.monotonic();v.overlay(frame)
                self.assertLess(time.monotonic()-began,.10)
                deadline=time.monotonic()+2
                while v.color_result is None and time.monotonic()<deadline:time.sleep(.01)
                self.assertIsNotNone(v.color_result);v.overlay(frame);self.assertGreater(frame[20:60,10:40].sum(),0)
                # Compare with a guide-only image: the enlarged search guide now
                # also crosses this region of the screen.
                clock[0]=2;frame[:]=0;v.overlay(frame)
                guide=VisionOverlay(PACKAGE,lambda:2);guide.thread=object();guide.digit_thread=object();guide.cv=cv2
                expected=np.zeros_like(frame);guide.overlay(expected)
                np.testing.assert_array_equal(frame,expected)
                self.assertGreater(frame.sum(),0)  # The static card guide remains visible.
                self.assertLessEqual(v.work.qsize(),1)
            finally:v.close()


if __name__=='__main__':unittest.main()
