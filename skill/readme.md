# Reusable validation and control-debugging workflows

These workflows were distilled from this project's recorded failures and checks. They are not a claim that a different board, camera or robot has been tested. No Skill in this directory starts motors, flashes firmware or accesses a device automatically.

## Manifest validation

Scope: a local release with JSON `upload_files` mapping relative paths to SHA256 values. Applicable to other PYNQ Overlays or embedded deployment folders, independent of this vehicle task.

Usage: `python skill/check_manifest.py --manifest pynq/manifest.json --root pynq`. A successful check proves the listed bytes exist and match; it does not prove a correct board model, camera wiring, executable architecture, successful installation or motor behaviour.

Failure/observation: the complete development manifest referenced test assets removed from a source-only export. Uploading that manifest unchanged would fail the installer's file verification. Generate a distinct export manifest and preserve the original release hash as provenance. This export checks the actual exported runtime list; test/report contents are outside its runtime list. Current export validation is recorded in `sim/results/export_checks.log`.

Failure conditions: an untrusted manifest is not a signature; mutable user settings need separate handling; a matching hash does not authorize installation. Preserve installed settings and rollback material before deliberate deployment.

## Asynchronous event-to-action handoff

Scope: a controller confirms an input, requests a stop, waits for a hardware acknowledgement, then performs an action. Useful beyond cars for an actuator interlock or staged machine sequence.

Failure: a short-lived vision BLOCKED event could disappear while the MCU was still braking. Requiring the same fresh image again after HELD discarded an already confirmed detour, leaving the system waiting. `board/validation/bypass_intent_20261101/before_fix_tests.log` retains its regression failure.

Method: retain the confirmed intent with session, step and pose epoch; require a fresh acknowledgement from that context; invalidate on STOP, completion, new session/step/reference. Signal priority still applies. Never turn unconfirmed/expired UNKNOWN into a new movement request.

Verified effect: six added cases cover CLEAR/UNKNOWN during braking, reference/step isolation, explicit STOP and red/green arbitration; existing bypass regressions and actual-C ideal-model wire cases passed. See `pynq/verification/test_bypass.py`, `sim/native/test_wire_bypass.py` and saved logs. This does not establish real braking distance or obstacle coverage.

## Board update evidence boundary

Scope: PYNQ Linux services with an explicit installer and preserved package files.

Order: inspect inactive vehicle state → preserve application/config/service → hash manifest → deliberately install → inspect process/service, camera and telemetry → save receipt. Keep offline tests, board software tests and actual motor trials in separate result fields. A connection timeout is not deployment success. Rollback of a unit file alone is not restoration of source, settings and firmware.

Observed limitation: repeated state reads can return the same image. Use unique camera timestamps/frame identifiers when counting samples. Near/far live records here include fewer distinct images than reads; they must not become a twenty-frame accuracy claim.
