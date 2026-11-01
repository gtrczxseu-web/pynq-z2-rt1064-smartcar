# Submission contents

Package date: 2026-11-01. Guide organization: autonomous-topic track 3.3. Team education/year group follows registration facts.

| Material | Repository location |
| --- | --- |
| PYNQ PS source, configuration and startup | `pynq/` |
| RT1064 source, Keil project and dependencies | `firmware/` |
| Overlay bitstream and hardware handoff | `pynq/hud_live.bit`, `pynq/hud_live.hwh` |
| Native runtime libraries and their C source | `pynq/` |
| Offline verification and fixed regression inputs | `pynq/verification/`, `sim/` |
| Selected board measurements and original results | `board/validation/` |
| Application/native/MCU build and check entrypoints | `build/` |
| Reusable workflows and manifest checker | `skill/` |
| Technical design report, 16 pages | `report/technical_report.pdf` |
| Selected prompts, responses and correction record | `report/collaboration.md` |
| Component provenance and licenses | `third_party_notices.md`, component license files |

The README provides the software check, MCU build and PYNQ runtime deployment steps. MCU compilation uses Keil Arm Compiler 6.19. Overlay files are distributed as runtime artifacts; they are distinct from MCU build outputs.

Repository paths use lowercase English, digits, underscores/hyphens and normal file extensions. Some existing report text and raw JSON/log values still contain historic local paths and filenames; those are provenance content, not repository paths.

The editable Word report remains in the local delivery folder. The repository contains its checked PDF export. Original measured outputs retain their historical timestamps, paths and state flags. No additional board run was performed for this publication.
