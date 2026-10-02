"""Default-off Ford acceleration overlap prototype; ECU arbitration is unverified.

Keep the physical pedal input untouched and send a bounded cruise request in
parallel. This does not compute max(pedal %, acceleration) or add the inputs.
"""

import math

from opendbc.car import DT_CTRL, structs
from opendbc.car.ford.values import CAR, CarControllerParams, FordFlags, FordSafetyFlags

PARAM_BP = "BPExperimentalConcurrentAccel"
MAX_POSITIVE_JERK_BP = 1.0  # m/s^3; request ramp, not measured vehicle jerk


def configure_concurrent_accel_bp(CP, requested: bool, disengage_on_gas: bool) -> None:
  """Configure once at car initialization, before CarParams reaches pandad."""
  if CP.brand != "ford" or not len(CP.safetyConfigs):
    return
  config = CP.safetyConfigs[-1]
  config.safetyParam &= ~FordSafetyFlags.CONCURRENT_ACCEL_BP.value
  if (requested and not disengage_on_gas and CP.carFingerprint == CAR.FORD_F_150_MK14 and
      CP.openpilotLongitudinalControl and not CP.dashcamOnly and CP.flags & FordFlags.CANFD and
      config.safetyModel == structs.CarParams.SafetyModel.ford and
      config.safetyParam & FordSafetyFlags.LONG_CONTROL and config.safetyParam & FordSafetyFlags.CANFD):
    config.safetyParam |= FordSafetyFlags.CONCURRENT_ACCEL_BP.value


def concurrent_accel_configured_bp(CP) -> bool:
  """Use the same immutable configuration on the controls and CAN sides."""
  required = FordSafetyFlags.LONG_CONTROL | FordSafetyFlags.CANFD | FordSafetyFlags.CONCURRENT_ACCEL_BP
  return bool(CP.brand == "ford" and CP.carFingerprint == CAR.FORD_F_150_MK14 and
              CP.openpilotLongitudinalControl and not CP.dashcamOnly and CP.flags & FordFlags.CANFD and
              len(CP.safetyConfigs) and CP.safetyConfigs[-1].safetyModel == structs.CarParams.SafetyModel.ford and
              (CP.safetyConfigs[-1].safetyParam & required) == required)


def concurrent_accel_enabled_bp(CP, experimental_mode: bool) -> bool:
  """Experimental Mode is a runtime requirement in addition to the saved opt-in."""
  return concurrent_accel_configured_bp(CP) and experimental_mode


def longitudinal_override_bp(events, concurrent: bool) -> bool:
  """Ignore only the gas override when explicitly configured for overlap."""
  return any(e.overrideLongitudinal and not (concurrent and e.name == "gasPressedOverride") for e in events)


def constrain_concurrent_accel_bp(result, gas_pressed: bool, active: bool, previous_gas: float):
  """Constrain final Ford output after pitch, lead and brake hysteresis tuning.

  While the pedal is pressed, the brake channel is neutral, predicted gas is
  inactive and no braking/precharge/stop request is sent. Reductions in the
  propulsion request remain immediate; increases ramp across pedal release.
  """
  if not active:
    return result._replace(accel=0.0, gas=CarControllerParams.INACTIVE_GAS,
                           accel_pred_send=CarControllerParams.INACTIVE_GAS,
                           brake_actuate=False, precharge_actuate=False, stopping=False)

  gas = result.gas
  if (not math.isfinite(gas) or (gas_pressed and gas < 0) or
      (not gas_pressed and (result.brake_actuate or result.precharge_actuate))):
    gas = CarControllerParams.INACTIVE_GAS
  elif gas >= 0:
    last = max(previous_gas, 0.0) if math.isfinite(previous_gas) else 0.0
    step = MAX_POSITIVE_JERK_BP * CarControllerParams.ACC_CONTROL_STEP * DT_CTRL
    gas = min(gas, last + step, CarControllerParams.ACCEL_MAX)

  if gas_pressed:
    return result._replace(accel=0.0, gas=gas, accel_pred_send=CarControllerParams.INACTIVE_GAS,
                           brake_actuate=False, precharge_actuate=False, stopping=False)
  return result._replace(gas=gas)
