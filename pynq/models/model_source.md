# Pretrained handwritten digit model

- Upstream: https://github.com/onnx/models/tree/main/validated/vision/classification/mnist
- Download: https://media.githubusercontent.com/media/onnx/models/main/validated/vision/classification/mnist/model/mnist-8.onnx
- SHA256: `2f06e72de813a8635c9bc0397ac447a601bdbfa7df4bebc278723b958831c9bf`
- Size: 26454 bytes. No local training or weight modification.
- Model-specific license: MIT, as stated in the upstream MNIST README:
  https://raw.githubusercontent.com/onnx/models/main/validated/vision/classification/mnist/README.md
  The source README carries `SPDX-License-Identifier: MIT`; license text is included.
- Input: float32 NCHW 1x1x28x28, white digit on black, range 0..1.
- Output: ten logits; apply softmax.

This is a single handwritten-digit classifier, not a scene detector or general OCR.
Present one large black handwritten digit on a white card, centered in the image.
Candidate extraction and HSV color-card bounding boxes run on ARM, not FPGA. Confidence is
not a validated real-world probability. No camera accuracy or ARM speed is claimed
before board testing. The display worker retains at most one pending frame, submits
at most five times per second, and expires old boxes. It has no motor/UART access.

The central corner marks show the digit region. `WAIT CARD` means no suitable
black digit on a brighter card was found; `HOLD STILL` means a candidate is being
checked. A softmax score of at least 0.70 and a top-two score gap of at least
0.25 qualify a prediction; two consecutive qualifying predictions of the same
digit show `DIGIT n`. The card does not
need to fill most of the region. Local contrast preserves shaded grey strokes.
Keep the whole digit inside the corner marks.
Confirmed digit text may remain for up to two seconds as `LAST n` when its
position box expires after 0.5 seconds. This is a recent result, not a current
target position. Confirmations do not carry across route sessions or pose origins.
Color targets must be compact, saturated cards inside the frame; large areas and
objects cut off by an image edge are ignored. The separate, unintegrated PL RGB
mask still uses its original integer thresholds; the running camera uses HSV.

Existing OpenART competition code references /sd/digit_vision.tflite. That file
was not found locally; it can be inspected/reused after retrieving it from the SD
card. Do not replace the current camera preview owner with the old UART main loop.
