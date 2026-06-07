# Hardware-in-Loop Run Log

Operator-recorded log of every HIL milestone test. Each entry uses the format below
exactly so a different operator can grep / sort it later.

Format:

```
## YYYY-MM-DD M<n> — <operator name> — PASS/FAIL — <notes>
```

Notes should call out anything tuned, any PARAMS values changed, and the next person's
to-do if the test failed.

---

<!-- entries below -->

## 2026-05-19 Phase 0 smoke — JC — PASS
- Mode probe: `[('UNKNOWN', 0), ('IDLE', 1), ('NRT_JOINT_IMPEDANCE', 4), ('NRT_JOINT_POSITION', 6), ('NRT_PLAN_EXECUTION', 7), ('NRT_PRIMITIVE_EXECUTION', 8), ('NRT_CARTESIAN_MOTION_FORCE', 10), ('NRT_SUPER_PRIMITIVE', 11)]` — differs from the inlab prompt's expected v1.9 listing (no `RT_*` modes; new `UNKNOWN`, `NRT_SUPER_PRIMITIVE`; renumbered). The two modes the pipeline actually uses are stable: `NRT_PRIMITIVE_EXECUTION=8`, `NRT_JOINT_IMPEDANCE=4`. No code change needed.
- Arduino status: after HOME_ALL — `x_mm=0.0 z_mm=0.0 rot_deg=0.0 homed=true busy=false faulted=false vise_state=OPEN blade_on=false force_kg≈0`.
- Robot operational: yes. `flexiv::rdk::Robot v1.9` connected to `Rizon4-062930`, license `RDK-Professional`, 7-DOF joint state streaming at 1 Hz.
- Notes:
  - Pinned `flexivrdk==1.9.0` in lab `.venv/`. The 1.9.1 client is incompatible with this robot's firmware ("Version of this client is incompatible with robot"). `requirements.txt` is still unpinned; bring this back to dev so it can be tightened.
  - Patched `arduino_client.py` to optionally wait for the firmware's `READY` banner after serial-open (DTR auto-reset was eating the first command); enabled in the smoke CLI. Test suite: 110 passing (was 107). Bring the patch back to dev.
  - Pickup geometry change: tray now horizontal, injectable lying flat. No spec/code change is blocking before M3 HIL — `specifications.md` §1.4 Phase 1 text edit ("sits upright in a passive retainer") deferred to M5 prep.
