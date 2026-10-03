# Follow Vehicle Speed Limits

The 2022 F-150 reports its camera-recognized speed limit through
`carStateSP.speedLimit`. This feature uses that signal directly, without map
data. It updates the comma's saved cruise target while cruise is engaged or
paused, so a recognized 70-to-40 mph road transition can resume at 40 mph.

## Settings

- Settings → Developer → sunnypilot Longitudinal Control (Alpha): ON.
- Settings → BluePilot → Longitudinal Tuning → Follow Vehicle Speed Limits: ON.
- Settings → Cruise → Speed Limit → Speed Limit: Info. Assist is a separate
  speed ceiling and can still limit a manual speed adjustment.
- In that menu, Customize Source → Speed Limit Source: Car Only, so the
  displayed limit uses the same vehicle data as this feature.
- Restart after changing Follow Vehicle Speed Limits. It is off by default,
  editable only while offroad, and read once at vehicle initialization.

Experimental Mode and Concurrent Acceleration Prototype remain independent.
Longitudinal/Lateral Maneuver Mode and Joystick Debug Mode must be off for
normal driving; those modes run test commands rather than the ordinary planner.

## Behavior

Set cruise to a camera-recognized limit to start following it. First Resume may
also select a saved speed that already matches the limit. A valid limit must
remain stable for one second before acceptance. Matching permits 1 km/h of
rounding difference. Passive matching alone does not arm following.

Cancel and braking preserve the saved target and following state. While paused,
valid changes to the recognized limit update the target without engaging cruise.
Resume preserves this target even if Ford still reports the older saved speed.

Manual +/- pauses following. Adjustments apply to the comma's target: if comma
shows 40 and Ford stores 70, a physical + that changes Ford to 71 selects 41 on
comma. Resume then retains that manual target. Selecting the recognized limit
again re-arms following. SET while paused selects a new speed, with delayed PCM
feedback handled separately from Resume. Turning cruise main off clears the
session. Missing/invalid signs preserve the last selected target; invalid CAN
cannot accept a new limit or speed adjustment. Out-of-range speeds are rejected.
If several adjustments arrive together before Ford confirms a new SET speed,
ambiguous feedback is ignored. Let the new SET speed settle before holding +/-.

Only F-150 MK14 with Ford-managed cruise engagement and openpilot longitudinal
control is affected. Other vehicles and the default-off setting use the existing
cruise helper. The fork does not write Ford's saved dashboard speed or synthesize
cruise button presses. Ford's display may differ from the comma target.

## Implementation and validation

`VCruiseHelperBP` extends the existing cruise helper. `card` supplies current
vehicle sign data and publishes the resulting `vCruise`/`vCruiseCluster` to the
ordinary planner. No safety configuration, engagement event, pedal input,
acceleration/braking limit, or firmware code changes are made by this feature.
The planner can still drive below the set speed for its normal reasons.

Run `python bluepilot/tools/test_concurrent_accel_bp.py --offline-services`.
The focused checks include saved 70 → cancel → recognized 40 → Resume,
delayed SET/Resume feedback, stale Ford setpoints, manual overrides and long
presses, invalid speed/CAN data, main-off, default-off and unsupported vehicles.
The tests use actual cereal messages and the real cruise helper with an isolated
in-memory Params fixture; no vehicle or device services are contacted.
Validation: 39 speed-limit cases, 9 concurrent acceleration cases and 9 compiled
firmware tests pass. The new module, tests and offline runner pass Ruff.

These software checks do not establish on-device installation, Ford dashboard
behavior, camera recognition accuracy, or vehicle performance. Existing firmware
suite limitations for the base BluePilot release are recorded in
`BP_CONCURRENT_ACCEL_PROTOTYPE.md`.
