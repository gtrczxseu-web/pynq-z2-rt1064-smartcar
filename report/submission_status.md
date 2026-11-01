# Submission status and remaining materials

Review date: 2026-11-01. Intended guide section: autonomous-topic track 3.3; team education/year group still follows registration facts. This repository is not a claim of eligibility for the Ryzen AI PC embodied-intelligence track.

| Requirement | Current status |
| --- | --- |
| PYNQ PS source, RT1064 firmware source/project/dependencies | Included |
| Working Overlay and native runtime files | Included as existing runtime artifacts |
| Exact Overlay Vivado/RTL/HLS/BD sources and XDC | Not located; must recover original project |
| Bitstream reconstruction script | Not available; do not fabricate an equivalent project |
| FPGA utilization, timing, clocks, implementation reports | Not available; cannot replace with MCU compilation or observed video FPS |
| Offline verification, selected board measurements and fixed inputs | Included; see board/readme.md and sim/results/ |
| Application/native/MCU build and check entrypoints | Included; exact export-check results recorded separately |
| Skill and collaboration records | Included as curated material with explicit scope |
| Technical design report | Included when the final checked local report is copied; its pending items remain pending |
| Independent team reproduction from clean machine | Not performed; required tool/image versions and missing PL source limit reproduction |
| Final English poster | Not included; needed if advancing to finals |

Recommended Vivado/Vitis versions in the guide are 2026.1 or 2025.2; another version requires explanation and reproducible scripts. The existing bitstream is not silently attributed to either version. MCU uses Keil Arm Compiler 6.19, which is a different toolchain.

Repository paths use lowercase English, digits, underscores/hyphens and normal file extensions. Some existing report text and raw JSON/log values still contain historic local paths and filenames; those are provenance content, not repository paths.

Open-source hosting and useful running code do not by themselves complete the FPGA submission. Recover the exact hardware project and actual implementation reports before claiming a fully rebuildable final competition package.
