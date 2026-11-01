# Recorded board validation

These files are preserved observations from 2026-11-01, not a fresh vehicle run performed during GitHub publication. JSON/log bytes are retained; historical local paths and status flags are not rewritten as success.

- `validation/bypass_intent_20261101/application_tests.log`: 292 application tests on the board. `offline_checks.json` records separate offline application, actual-C ideal-model and legacy-wire checks. `before_fix_tests.log` retains the failing reproduction before the fix.
- `health_final.json`: latest saved static READY / MCU V8 STOPPED check, with 269 camera frames in 60.093 s (about 4.476 FPS) and 601 telemetry frames. No motor START requested by that check.
- `validation/obstacle_near_50cm_20261101/calibration.json`: user measured cup base centre 0.5 m from camera ground projection; eight base-contact rows were 179 in 320×240 images. This is one fixed ground-plane boundary, not calibrated general metric depth.
- Actual regression images are under `pynq/verification/data/`. Near50 cup samples: 8/8 BLOCKED; far 1 m: 8/8 CLEAR; empty floor: 5/5 CLEAR. The previews show the boundary used.
- `validation/near50_display_final_20261101/`: near live reads were 19 BLOCKED / 1 UNKNOWN across only 5 distinct snapshots; far live reads 15 CLEAR / 5 UNKNOWN across 6 snapshots. Twenty state reads are not twenty independent images.
- `validation/obstacle_bypass_20261101/keil_rebuild.log`: original V8 firmware build, 0 errors / 0 warnings. `firmware.json` identifies its AXF. This is an MCU build report, not a Vivado utilization/timing report.

After the correction the user reported “验证无误”. This qualitative acceptance does not provide trial count, obstacle coverage, distance accuracy, braking distance or long-run success statistics. Original pre-acceptance `real_vehicle_retest_pending` is retained to preserve chronology.

Camera height/pitch, lighting, floor texture/reflections, object size and motion blur affect detection. Forward fixed bypass does not check either side. Reflection suppression passed these local comparisons; universal reflection rejection and all-object recognition were not established.
