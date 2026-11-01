# Runs temporarily in OpenART Plus RAM through the OpenMV USB debug protocol.
# This file is not saved to the module's SD card by the supplied notebook.
import sensor

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
# Verified on this user's OpenART 4.5.2; its brightness scale differs from the
# standard OpenMV -3..3 API. Do not copy this value to other camera firmware.
if sensor.set_brightness(255) is False:
    raise RuntimeError('Native camera brightness setting was rejected')
sensor.skip_frames(time=2500)
print('CAMERA_NATIVE_BRIGHTNESS=255')

while True:
    sensor.snapshot()
