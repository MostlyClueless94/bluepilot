# F-150 concurrent acceleration prototype

Status: research draft, disabled by default, not validated for vehicle use.
Base: BluePilot `bp-7.0`, commit `e1d051d7ba270261b4455068bd68f1a58db15a4a`.
Intended vehicle: the user's 2022 F-150 on comma 3X.

## Intended behavior and remaining uncertainty

The driver wants stock-like arbitration: the stronger of the driver's physical
pedal request and cruise acceleration should win, without adding the requests.
Cruise should remain available during a gas nudge and continue smoothly when
the pedal is released, including when RESUME is pressed with the pedal down.

This prototype keeps a bounded cruise propulsion request available during
pedal input. It does not intercept the pedal, infer engine torque from pedal
percentage, add requests, or calculate a maximum between incompatible units.
Whether the Ford PCM arbitrates these injected ACCDATA requests exactly like
the factory cruise system is **unverified**. The PCM might suppress the cruise
request until pedal release or apply different arbitration. Passing these
tests cannot establish stock-like behavior or measured acceleration smoothness.

## Implementation

- `BPExperimentalConcurrentAccel` defaults to false. The C3X BluePilot settings
  entry is editable offroad and clearly labeled as an unverified prototype.
  Configuration is captured at car initialization and requires restart.
- Only `FORD_F_150_MK14` with CAN-FD, Ford safety mode, supported openpilot
  longitudinal control, and no dashcam-only restriction can receive the new
  safety capability. Disengage on Accelerator must be off; its behavior is
  retained when the user enables it.
- Runtime controls ignore only `gasPressedOverride` when the capability is
  configured and Experimental Mode is selected. Other longitudinal overrides
  and disengagement events are retained. The event remains logged, so the
  selfdrive state/UI can still indicate overriding during pedal overlap.
- The longitudinal PID stays active, with its integral frozen during pedal
  input to avoid wind-down from acceleration supplied by the driver. A frozen
  positive correction cannot accelerate against a nonpositive planner target.
- After existing Ford longitudinal tuning, overlap frames have neutral
  `AccBrkTot_A_Rq`, inactive predicted propulsion, and no braking, precharge,
  or stop request. Positive propulsion is limited to the existing 2.0 m/s²
  maximum. Positive request increases ramp at 1.0 m/s³ across pedal release;
  decreases and cancellation remain immediate. This is a command ramp, not
  a guarantee of actual vehicle jerk.
- The Ford firmware exception requires controls allowed, gas pressed, and
  explicit CAN-FD + longitudinal + overlap flags. It accepts only inactive or
  nonnegative bounded propulsion, neutral brake-channel acceleration, inactive
  predicted gas, and no brake/precharge/stop/parking-brake/driver-override/
  torque-minimum request. AEB-deny remains prohibited. Brake RX and cruise-off
  RX revoke controls normally. Global `get_longitudinal_allowed()` is unchanged.
- The firmware capability is static for the session; firmware has no
  Experimental Mode field. Software must enforce the runtime mode requirement.
  Firmware compiled without the existing `ALLOW_DEBUG` longitudinal support
  cannot enable the new capability. Matching application and firmware builds
  would be required for any future hardware validation.

Planner/model acceleration decisions, speed limits and lead processing remain
in place. This does not force the model to reach the set speed. Brake control
returns to normal after pedal release. Factory-ACC mode receives no new
capability. The existing BP highway follow-control thresholds are unchanged.

## Offline validation

The focused checks use actual cereal schemas, PID logic, DBC packing, and
compiled C firmware. Nine software tests and nine firmware tests pass, including
all 1,024 raw propulsion values, all 8,192 raw brake-channel acceleration
values, prediction values, partial flags/reinitialization, brake/cruise-off,
relay malfunction, AEB-deny, request ramping, and generated overlap/release
frames passed through the firmware TX hook.

The complete preserved safety suite was compared with the untouched base in
the same macOS/Python 3.12 environment. The baseline reports 8,706 tests with
947 skipped tests and 22,556 failure/subtest entries, all in existing Ford tests.
The prototype reports 8,715 tests with the same failure entries and skipped
tests, and no new errors or regressions.
These are **not passing full-suite results**. The full-suite requirement in
`docs/SAFETY.md` remains unmet, including on the base branch.

New Python logic passes Ruff; the modified C safety code compiles with
`-Wall -Wextra -Werror`, both with and without `ALLOW_DEBUG`. Existing unrelated
lint issues remain in the original Ford controller and settings UI. A complete
comma device application/firmware build, UI execution, recorded-route replay,
hardware-in-the-loop testing and vehicle validation have not been performed.

For an isolated desktop test environment, install numpy, pycapnp, cffi, pytest,
hypothesis==6.47.0, tree-sitter, tree-sitter-c, tqdm, pycryptodome, setproctitle,
zstandard and ruff. Run from the checkout:

```sh
python bluepilot/tools/test_concurrent_accel_bp.py --offline-services
python bluepilot/tools/test_concurrent_accel_bp.py --offline-services --all-safety --report /tmp/bp-safety.json
```

`--offline-services` substitutes only unused device-service imports, with Params
construction raising an error if attempted. It does not mock firmware, packing,
schemas or longitudinal control. This harness tests unit boundaries, not live
process messaging or the full application.

## Before vehicle use

The draft needs a passing complete safety suite and a matching device build.
Bench testing or a factory-ACC recording must establish how this truck's PCM
handles concurrent requests, especially light pedal + stronger cruise,
stronger pedal + weaker cruise, RESUME with pedal held, and pedal release.
Measured acceleration/jerk, braking, cancel, stopped traffic and model/lead
deceleration require validation. Until then, leave the feature disabled and
do not treat this branch as a road-ready release.
