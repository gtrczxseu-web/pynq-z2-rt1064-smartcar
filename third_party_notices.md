# Component provenance and notices

- `firmware/libraries/`: SEEKFREE/逐飞 library with original GPLv3 notices (`license`, `doc/gpl3_permission_statement.txt`). NXP SDK, CMSIS and FatFs retain file-level and component notices; do not remove those headers. Publication normalizes paths/includes to lowercase and underscores, without intentionally changing their algorithms.
- `pynq/models/mnist-8.onnx`: pretrained MNIST model; source and MIT notice are in `pynq/models/model_source.md` and `license_mit.txt`. No new model training was performed in this project. Character-card results are not general scene OCR claims.
- `pynq/hud_live.bit` and `hud_live.hwh`: existing working Overlay artifacts received with the project. Full editable source, upstream provenance and exact build tool metadata remain to be recovered. These are binary runtime artifacts, not evidence of a newly authored/rebuilt hardware design. The project source license does not purport to relicense vendor IP or missing upstream hardware sources.
- PYNQ, NumPy, OpenCV, Linux, GCC, Arm Compiler, Keil and Vivado are separate dependencies. Install them from their original distributions under their own terms.

The root license covers the project's own distributed code. Component-specific notices remain applicable. No proprietary tool installers, credentials, private backups or unrelated workspaces are included.
