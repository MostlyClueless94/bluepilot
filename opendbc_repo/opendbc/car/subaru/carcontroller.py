import numpy as np
from openpilot.common.params import Params
from opendbc.can import CANPacker
from opendbc.car import Bus, make_tester_present_msg, structs
from opendbc.car.carlog import carlog
from opendbc.car.lateral import AngleSteeringLimits, apply_driver_steer_torque_limits, apply_std_steer_angle_limits, common_fault_avoidance
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.subaru import subarucan
from opendbc.car.subaru.carstate import (
  MANUAL_YIELD_TORQUE_THRESHOLD_DEFAULT,
  MANUAL_YIELD_TORQUE_THRESHOLD_MIN,
  MANUAL_YIELD_TORQUE_THRESHOLD_VALUES,
)
from opendbc.car.subaru.values import DBC, GLOBAL_ES_ADDR, CanBus, CarControllerParams, SubaruFlags

from opendbc.sunnypilot.car.subaru.stop_and_go import SnGCarController

# FIXME: These limits aren't exact. The real limit is more than likely over a larger time period and
# involves the total steering angle change rather than rate, but these limits work well for now
MAX_STEER_RATE = 25  # deg/s
MAX_STEER_RATE_FRAMES = 7  # tx control frames needed before torque can be cut
MADS_ONLY_MIN_SPEED = 0.44704  # m/s (1 mph)
MADS_ONLY_MAX_STEER_ANGLE = 180.0  # deg
MADS_ONLY_MAX_STEER_ANGLE_MAX = 545.0  # deg, matches Subaru angle-LKAS safety max
MADS_ONLY_FAULT_GUARD_LOW_SPEED = 8.0 * 0.44704  # m/s
MADS_ONLY_FAULT_GUARD_HIGH_SPEED = 15.0 * 0.44704  # m/s
MADS_ONLY_FAULT_GUARD_LOW_SPEED_CAP = 180.0  # deg
MADS_ONLY_FAULT_GUARD_RATE_THRESHOLD = 120.0  # deg/s
MADS_ONLY_FAULT_GUARD_QUIET_RATE = 60.0  # deg/s
MADS_ONLY_FAULT_GUARD_QUIET_FRAMES = 10  # steering command frames (~200 ms with STEER_STEP=2)
MADS_ONLY_FAULT_GUARD_ANGLE_MARGIN = 20.0  # deg below active cap before high-rate guard arms
MADS_ONLY_FAULT_GUARD_REENABLE_MARGIN = 5.0  # deg below active cap before quiet countdown can run
MADS_ONLY_FAULT_GUARD_REENABLE_UNWIND_FRAMES = 5  # steering command frames after quiet guard clears
MADS_ONLY_FAULT_GUARD_REENABLE_UNWIND_STEP = 3.0  # deg/frame centerward clamp after quiet guard clears
MAX_STEERING_LOW_SPEED_GUARD_ENTER_SPEED = 11.0 * 0.44704  # m/s
MAX_STEERING_LOW_SPEED_GUARD_EXIT_SPEED = 12.5 * 0.44704  # m/s
MAX_STEERING_LOW_SPEED_GUARD_TARGET_ANGLE = 15.0  # deg
MAX_STEERING_LOW_SPEED_GUARD_MEASURED_ANGLE = 30.0  # deg
MAX_STEERING_LOW_SPEED_GUARD_RATE = 60.0  # deg/s
MAX_STEERING_LOW_SPEED_GUARD_QUIET_TARGET_ANGLE = 10.0  # deg
MAX_STEERING_LOW_SPEED_GUARD_QUIET_MEASURED_ANGLE = 10.0  # deg
MAX_STEERING_LOW_SPEED_GUARD_QUIET_RATE = 60.0  # deg/s
MAX_STEERING_LOW_SPEED_GUARD_QUIET_FRAMES = 10  # steering command frames (~200 ms with STEER_STEP=2)
ANGLE_DRIVER_OVERRIDE_HOLD_FRAMES = 10  # steering command frames (~200 ms with STEER_STEP=2)
ANGLE_DRIVER_OVERRIDE_RAMP_FRAMES = 36  # validated default reclaim ramp (steering command frames, ~720 ms with STEER_STEP=2)
ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_MIN = 0
ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_MAX = 6
ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_DEFAULT = 4
ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_EXPONENTS = [1.0, 1.25, 1.5, 2.0, 2.5, 3.0, 3.5]
ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_LEVEL_MIN = 1
ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_LEVEL_MAX = 3
ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_LEVEL_DEFAULT = 2
ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_CONFIRM_FRAME_OPTIONS = [4, 8, 12]  # additional steering command frames (~80/160/240 ms)
ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_RATE_THRESHOLDS = [3.0, 2.0, 1.0]  # deg/s
ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_ANGLE_DELTA = 1.0  # deg
ANGLE_DRIVER_OVERRIDE_SOFT_HOLD_LEVEL_MIN = 0
ANGLE_DRIVER_OVERRIDE_SOFT_HOLD_LEVEL_MAX = 3
ANGLE_DRIVER_OVERRIDE_SOFT_HOLD_LEVEL_DEFAULT = 0
ANGLE_DRIVER_OVERRIDE_SOFT_HOLD_FACTORS = [0.0, 0.75, 0.60, 0.50]
ANGLE_DRIVER_OVERRIDE_SOFT_HOLD_FRAMES = [0, 25, 50, 75]
# Soft-capture engage blending (MostlyClueless experiment — not for stable branches)
# Maps UI level 0 (off) / 1–5 to (ramp_frames, alpha_start) pairs.
# Level 5 is the most damped (longest ramp, gentlest start).
SOFT_CAPTURE_LEVEL_PARAMS = [
  # (ramp_frames, alpha_start)
  (0, 1.0),    # 0 — disabled (instant snap, stock behavior)
  (15, 0.25),  # 1 — light
  (22, 0.15),  # 2 — mild
  (30, 0.08),  # 3 — medium
  (40, 0.05),  # 4 — strong
  (50, 0.02),  # 5 — max
]
SUBARU_UNWIND_RATE_LEVEL_VALUES = (
  0.8, 1.0, 1.2, 1.5, 1.8, 2.1, 2.4, 2.8, 3.2, 3.6, 4.0,
  4.5, 5.0, 5.5, 6.0, 6.5, 7.0, 7.5, 8.0, 9.0, 10.0,
)
SUBARU_UNWIND_RATE_LEVEL_MIN = 0
SUBARU_UNWIND_RATE_LEVEL_MAX = len(SUBARU_UNWIND_RATE_LEVEL_VALUES) - 1
SUBARU_TURN_IN_RATE_LEVEL_VALUES = SUBARU_UNWIND_RATE_LEVEL_VALUES
SUBARU_TURN_IN_RATE_LEVEL_MIN = 0
SUBARU_TURN_IN_RATE_LEVEL_MAX = len(SUBARU_TURN_IN_RATE_LEVEL_VALUES) - 1


class CarController(CarControllerBase, SnGCarController):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    SnGCarController.__init__(self, CP, CP_SP)
    self.apply_torque_last = 0
    self.apply_angle_last = 0

    self.cruise_button_prev = 0
    self.steer_rate_counter = 0
    self._debug_state = {}

    self.p = CarControllerParams(CP)
    self.packer = CANPacker(DBC[CP.carFingerprint][Bus.pt])
    self.params = Params()
    self.mc_subaru_manual_yield_resume_softness_enabled = False
    self.mc_subaru_manual_yield_resume_softness = ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_DEFAULT
    self.mc_subaru_manual_yield_release_guard_enabled = False
    self.mc_subaru_manual_yield_release_guard_level = ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_LEVEL_DEFAULT
    self.mc_subaru_manual_steering_soft_hold_level = ANGLE_DRIVER_OVERRIDE_SOFT_HOLD_LEVEL_DEFAULT
    self.mc_subaru_manual_yield_torque_threshold_enabled = False
    self.mc_subaru_manual_yield_torque_threshold = MANUAL_YIELD_TORQUE_THRESHOLD_DEFAULT
    self.mc_subaru_soft_capture_enabled = False
    self.mc_subaru_soft_capture_level = 3
    self.mc_subaru_mads_tighter_turns_enabled = False
    self.mc_subaru_mads_max_steering_angle = MADS_ONLY_MAX_STEER_ANGLE
    self.mc_subaru_unwind_rate_level = SUBARU_UNWIND_RATE_LEVEL_MIN
    self.mc_subaru_turn_in_rate_level = SUBARU_TURN_IN_RATE_LEVEL_MIN
    self.mc_subaru_max_steering_experiment = False
    self.subaru_manual_yield_full_release_active = False
    self.subaru_effective_lkas_active = False
    self.mads_lkas_fault_guard_quiet_frames = 0
    self.mads_lkas_fault_guard_source = "none"
    self.mads_lkas_reenable_unwind_clamp_frames = 0
    self.max_steering_low_speed_guard_active = False
    self.max_steering_low_speed_guard_quiet_frames = 0
    self.angle_driver_override_hold_frames = 0
    self.angle_driver_override_ramp_frames = 0
    self.angle_driver_override_ramp_total_frames = ANGLE_DRIVER_OVERRIDE_RAMP_FRAMES
    self.angle_driver_override_ramp_start_angle = 0.0
    self.angle_driver_override_ramp_softness_exponent = ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_EXPONENTS[ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_DEFAULT]
    self.angle_driver_override_release_guard_pending = False
    self.angle_driver_override_release_guard_confirm_frames = 0
    self.angle_driver_override_release_guard_required_frames = 0
    self.angle_driver_override_release_guard_reference_angle = 0.0
    self.angle_driver_override_release_guard_rate_threshold = 0.0
    self.angle_driver_override_soft_hold_frames = 0
    self.angle_driver_override_soft_hold_threshold = 0
    self.lat_active_prev = False
    self.soft_capture_frame = -(SOFT_CAPTURE_LEVEL_PARAMS[-1][0] + 1)
    self._update_params()

  def _log_transition(self, key, value, message):
    if self._debug_state.get(key) != value:
      carlog.info(f"subaru[{self.CP.carFingerprint}] {message}")
      self._debug_state[key] = value

  def _get_int_param(self, key: str, default: int = 0) -> int:
    value = self.params.get(key, return_default=True)
    try:
      return int(value)
    except (TypeError, ValueError):
      return default

  def _get_bool_param(self, key: str, default: bool = False) -> bool:
    value = self.params.get(key, return_default=True)
    if value is None:
      return default
    if isinstance(value, bool):
      return value
    if isinstance(value, bytes):
      return value not in (b"", b"0")
    if isinstance(value, str):
      return value not in ("", "0", "false", "False")
    return bool(value)

  @staticmethod
  def _get_resume_softness_exponent(softness_setting: int) -> float:
    return ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_EXPONENTS[int(np.clip(
      softness_setting,
      ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_MIN,
      ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_MAX,
    ))]

  @staticmethod
  def _get_release_guard_confirm_frames(level: int) -> int:
    idx = int(np.clip(level, ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_LEVEL_MIN, ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_LEVEL_MAX)) - 1
    return ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_CONFIRM_FRAME_OPTIONS[idx]

  @staticmethod
  def _get_release_guard_rate_threshold(level: int) -> float:
    idx = int(np.clip(level, ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_LEVEL_MIN, ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_LEVEL_MAX)) - 1
    return ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_RATE_THRESHOLDS[idx]

  @staticmethod
  def _clamp_manual_yield_torque_threshold(threshold: int) -> int:
    return min(
      MANUAL_YIELD_TORQUE_THRESHOLD_VALUES,
      key=lambda value: (abs(value - threshold), value),
    )

  @staticmethod
  def _get_soft_hold_threshold(active_threshold: int, level: int) -> int:
    if level <= ANGLE_DRIVER_OVERRIDE_SOFT_HOLD_LEVEL_MIN:
      return active_threshold

    clamped_level = int(np.clip(
      level,
      ANGLE_DRIVER_OVERRIDE_SOFT_HOLD_LEVEL_MIN,
      ANGLE_DRIVER_OVERRIDE_SOFT_HOLD_LEVEL_MAX,
    ))
    scaled_threshold = active_threshold * ANGLE_DRIVER_OVERRIDE_SOFT_HOLD_FACTORS[clamped_level]
    rounded_threshold = int(np.floor(scaled_threshold / 5.0 + 0.5) * 5)
    return int(np.clip(rounded_threshold, MANUAL_YIELD_TORQUE_THRESHOLD_MIN, active_threshold))

  def _get_stock_manual_yield_torque_threshold(self) -> int:
    return 75 if self.CP.flags & SubaruFlags.PREGLOBAL else MANUAL_YIELD_TORQUE_THRESHOLD_DEFAULT

  def _get_active_manual_yield_torque_threshold(self) -> int:
    if not self.mc_subaru_manual_yield_torque_threshold_enabled:
      return self._get_stock_manual_yield_torque_threshold()

    return self.mc_subaru_manual_yield_torque_threshold

  def _manual_yield_handoff_enabled(self) -> bool:
    return self.mc_subaru_manual_yield_resume_softness_enabled or self.mc_subaru_manual_yield_release_guard_enabled

  def _get_mads_only_max_steer_angle(self) -> float:
    if not self.mc_subaru_mads_tighter_turns_enabled:
      return MADS_ONLY_MAX_STEER_ANGLE
    return float(np.clip(
      self.mc_subaru_mads_max_steering_angle,
      MADS_ONLY_MAX_STEER_ANGLE,
      MADS_ONLY_MAX_STEER_ANGLE_MAX,
    ))

  def _get_active_angle_limits(self, mads_only: bool = False) -> AngleSteeringLimits:
    base = self.p.ANGLE_LIMITS
    unwind_level = int(np.clip(self.mc_subaru_unwind_rate_level, SUBARU_UNWIND_RATE_LEVEL_MIN, SUBARU_UNWIND_RATE_LEVEL_MAX))
    unwind_mid = SUBARU_UNWIND_RATE_LEVEL_VALUES[unwind_level]
    turn_in_level = int(np.clip(self.mc_subaru_turn_in_rate_level, SUBARU_TURN_IN_RATE_LEVEL_MIN, SUBARU_TURN_IN_RATE_LEVEL_MAX))
    turn_in_mid = SUBARU_TURN_IN_RATE_LEVEL_VALUES[turn_in_level] if mads_only else base.ANGLE_RATE_LIMIT_UP[1][1]
    return AngleSteeringLimits(
      base.STEER_ANGLE_MAX,
      (list(base.ANGLE_RATE_LIMIT_UP[0]), [5.0, turn_in_mid, 0.15]),
      (list(base.ANGLE_RATE_LIMIT_DOWN[0]), [5.0, unwind_mid, 0.15]),
      base.MAX_LATERAL_ACCEL,
      base.MAX_LATERAL_JERK,
      base.MAX_ANGLE_RATE,
    )

  @staticmethod
  def _get_mads_only_active_steer_angle_cap(selected_cap: float, speed: float) -> float:
    low_speed_cap = min(selected_cap, MADS_ONLY_FAULT_GUARD_LOW_SPEED_CAP)
    if speed <= MADS_ONLY_FAULT_GUARD_LOW_SPEED:
      return low_speed_cap
    if speed >= MADS_ONLY_FAULT_GUARD_HIGH_SPEED:
      return selected_cap

    return float(np.interp(
      speed,
      [MADS_ONLY_FAULT_GUARD_LOW_SPEED, MADS_ONLY_FAULT_GUARD_HIGH_SPEED],
      [low_speed_cap, selected_cap],
    ))

  def _reset_mads_lkas_fault_guard(self) -> None:
    self.mads_lkas_fault_guard_quiet_frames = 0
    self.mads_lkas_fault_guard_source = "none"

  def _start_mads_lkas_fault_guard(self, source: str) -> None:
    if self.mads_lkas_fault_guard_quiet_frames <= 0 or self.mads_lkas_fault_guard_source == "none":
      self.mads_lkas_fault_guard_source = source
    self.mads_lkas_fault_guard_quiet_frames = MADS_ONLY_FAULT_GUARD_QUIET_FRAMES

  def _update_mads_lkas_fault_guard(self, mads_only: bool, speed: float, measured_angle: float,
                                    steering_rate: float, active_cap: float, angle_limited: bool) -> tuple[bool, str]:
    if not mads_only:
      self._reset_mads_lkas_fault_guard()
      self.mads_lkas_reenable_unwind_clamp_frames = 0
      return False, "none"

    if speed > MADS_ONLY_FAULT_GUARD_HIGH_SPEED:
      self._reset_mads_lkas_fault_guard()
      self.mads_lkas_reenable_unwind_clamp_frames = 0
      return False, "none"

    angle_threshold = max(
      MADS_ONLY_MAX_STEER_ANGLE,
      min(MADS_ONLY_FAULT_GUARD_LOW_SPEED_CAP, active_cap - MADS_ONLY_FAULT_GUARD_ANGLE_MARGIN),
    )
    high_rate_risk = (
      abs(measured_angle) >= angle_threshold
      and abs(steering_rate) >= MADS_ONLY_FAULT_GUARD_RATE_THRESHOLD
    )

    if angle_limited:
      self._start_mads_lkas_fault_guard("mads_angle_limit")
      return True, "mads_angle_limit"

    if high_rate_risk:
      self._start_mads_lkas_fault_guard("mads_rate_guard")
      return True, "mads_rate_guard"

    if self.mads_lkas_fault_guard_quiet_frames <= 0:
      self.mads_lkas_fault_guard_source = "none"
      return False, "none"

    quiet_enough = (
      speed <= MADS_ONLY_FAULT_GUARD_HIGH_SPEED
      and abs(steering_rate) <= MADS_ONLY_FAULT_GUARD_QUIET_RATE
      and abs(measured_angle) < active_cap - MADS_ONLY_FAULT_GUARD_REENABLE_MARGIN
    )
    if quiet_enough:
      self.mads_lkas_fault_guard_quiet_frames -= 1
      if self.mads_lkas_fault_guard_quiet_frames <= 0:
        self.mads_lkas_reenable_unwind_clamp_frames = MADS_ONLY_FAULT_GUARD_REENABLE_UNWIND_FRAMES
    else:
      self.mads_lkas_fault_guard_quiet_frames = MADS_ONLY_FAULT_GUARD_QUIET_FRAMES

    return True, f"{self.mads_lkas_fault_guard_source}_quiet"

  def _apply_mads_lkas_reenable_unwind_clamp(self, apply_steer: float, lkas_request: bool) -> tuple[float, bool, bool]:
    clamp_active = lkas_request and self.mads_lkas_reenable_unwind_clamp_frames > 0
    if not clamp_active:
      return apply_steer, False, False

    clamped = False
    if abs(apply_steer) < abs(self.apply_angle_last):
      clamped_steer = float(np.clip(
        apply_steer,
        self.apply_angle_last - MADS_ONLY_FAULT_GUARD_REENABLE_UNWIND_STEP,
        self.apply_angle_last + MADS_ONLY_FAULT_GUARD_REENABLE_UNWIND_STEP,
      ))
      clamped = not np.isclose(clamped_steer, apply_steer)
      apply_steer = clamped_steer

    self.mads_lkas_reenable_unwind_clamp_frames -= 1
    return apply_steer, True, clamped

  def _reset_max_steering_low_speed_guard(self) -> None:
    self.max_steering_low_speed_guard_active = False
    self.max_steering_low_speed_guard_quiet_frames = 0

  def _update_max_steering_low_speed_guard(self, mads_only: bool, speed: float, target_angle: float,
                                           measured_angle: float, steering_rate: float) -> tuple[bool, str]:
    if not (self.mc_subaru_max_steering_experiment and mads_only):
      self._reset_max_steering_low_speed_guard()
      return False, "none"

    if speed > MAX_STEERING_LOW_SPEED_GUARD_EXIT_SPEED:
      self._reset_max_steering_low_speed_guard()
      return False, "speed_exit"

    turn_risk = (
      abs(target_angle) >= MAX_STEERING_LOW_SPEED_GUARD_TARGET_ANGLE
      or abs(measured_angle) >= MAX_STEERING_LOW_SPEED_GUARD_MEASURED_ANGLE
      or abs(steering_rate) >= MAX_STEERING_LOW_SPEED_GUARD_RATE
    )
    if speed <= MAX_STEERING_LOW_SPEED_GUARD_ENTER_SPEED and turn_risk:
      self.max_steering_low_speed_guard_active = True
      self.max_steering_low_speed_guard_quiet_frames = MAX_STEERING_LOW_SPEED_GUARD_QUIET_FRAMES
      return True, "max_low_speed_turn"

    if not self.max_steering_low_speed_guard_active:
      return False, "none"

    quiet_enough = (
      abs(target_angle) < MAX_STEERING_LOW_SPEED_GUARD_QUIET_TARGET_ANGLE
      and abs(measured_angle) < MAX_STEERING_LOW_SPEED_GUARD_QUIET_MEASURED_ANGLE
      and abs(steering_rate) <= MAX_STEERING_LOW_SPEED_GUARD_QUIET_RATE
    )
    if quiet_enough:
      self.max_steering_low_speed_guard_quiet_frames -= 1
      if self.max_steering_low_speed_guard_quiet_frames <= 0:
        self._reset_max_steering_low_speed_guard()
    else:
      self.max_steering_low_speed_guard_quiet_frames = MAX_STEERING_LOW_SPEED_GUARD_QUIET_FRAMES

    return True, "max_low_speed_quiet"

  @staticmethod
  def _apply_mads_only_steer_target_cap(steer_target: float, mads_only: bool, lkas_request: bool,
                                        max_steer_angle: float) -> tuple[float, bool]:
    if not (mads_only and lkas_request):
      return steer_target, False

    capped_target = float(np.clip(steer_target, -max_steer_angle, max_steer_angle))
    return capped_target, not np.isclose(capped_target, steer_target)

  def _get_soft_capture_level(self) -> int:
    if not self.mc_subaru_soft_capture_enabled:
      return 0

    return int(np.clip(
      self.mc_subaru_soft_capture_level,
      1,
      len(SOFT_CAPTURE_LEVEL_PARAMS) - 1,
    ))

  def _get_soft_capture_angle(self, model_target: float, wheel_angle: float) -> float:
    level = self._get_soft_capture_level()
    if level == 0:
      return model_target

    ramp_frames, alpha_start = SOFT_CAPTURE_LEVEL_PARAMS[level]
    frames_since_engage = max(0, self.frame - self.soft_capture_frame)

    if frames_since_engage >= ramp_frames:
      return model_target

    t = frames_since_engage / ramp_frames
    alpha = float(np.interp(t, [0.0, 1.0], [alpha_start, 1.0]))
    return wheel_angle + alpha * (model_target - wheel_angle)

  def _update_params(self):
    self.mc_subaru_manual_yield_resume_softness_enabled = self._get_bool_param("MCSubaruManualYieldResumeSoftnessEnabled")
    manual_yield_resume_softness = int(np.clip(
      self._get_int_param("MCSubaruManualYieldResumeSoftness", ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_DEFAULT),
      ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_MIN,
      ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_MAX,
    ))
    self.mc_subaru_manual_yield_resume_softness = manual_yield_resume_softness if self.mc_subaru_manual_yield_resume_softness_enabled \
      else ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_DEFAULT
    self.mc_subaru_manual_yield_release_guard_enabled = self._get_bool_param("MCSubaruManualYieldReleaseGuardEnabled")
    self.mc_subaru_manual_yield_release_guard_level = int(np.clip(
      self._get_int_param("MCSubaruManualYieldReleaseGuardLevel", ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_LEVEL_DEFAULT),
      ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_LEVEL_MIN,
      ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_LEVEL_MAX,
    ))
    self.mc_subaru_manual_steering_soft_hold_level = int(np.clip(
      self._get_int_param("MCSubaruManualSteeringSoftHoldLevel", ANGLE_DRIVER_OVERRIDE_SOFT_HOLD_LEVEL_DEFAULT),
      ANGLE_DRIVER_OVERRIDE_SOFT_HOLD_LEVEL_MIN,
      ANGLE_DRIVER_OVERRIDE_SOFT_HOLD_LEVEL_MAX,
    ))
    self.mc_subaru_manual_yield_torque_threshold_enabled = self._get_bool_param("MCSubaruManualYieldTorqueThresholdEnabled")
    self.mc_subaru_manual_yield_torque_threshold = self._clamp_manual_yield_torque_threshold(
      self._get_int_param("MCSubaruManualYieldTorqueThreshold", MANUAL_YIELD_TORQUE_THRESHOLD_DEFAULT)
    )
    self.mc_subaru_soft_capture_enabled = self._get_bool_param("MCSubaruSoftCaptureEnabled")
    self.mc_subaru_soft_capture_level = int(np.clip(
      self._get_int_param("MCSubaruSoftCaptureLevel", 3),
      1,
      len(SOFT_CAPTURE_LEVEL_PARAMS) - 1,
    ))
    self.mc_subaru_mads_tighter_turns_enabled = self._get_bool_param("MCSubaruMadsTighterTurnsEnabled")
    self.mc_subaru_mads_max_steering_angle = float(np.clip(
      self._get_int_param("MCSubaruMadsMaxSteeringAngle", int(MADS_ONLY_MAX_STEER_ANGLE)),
      MADS_ONLY_MAX_STEER_ANGLE,
      MADS_ONLY_MAX_STEER_ANGLE_MAX,
    ))
    self.mc_subaru_unwind_rate_level = int(np.clip(
      self._get_int_param("MCSubaruUnwindRateLevel", SUBARU_UNWIND_RATE_LEVEL_MIN),
      SUBARU_UNWIND_RATE_LEVEL_MIN,
      SUBARU_UNWIND_RATE_LEVEL_MAX,
    ))
    self.mc_subaru_turn_in_rate_level = int(np.clip(
      self._get_int_param("MCSubaruTurnInRateLevel", SUBARU_TURN_IN_RATE_LEVEL_MIN),
      SUBARU_TURN_IN_RATE_LEVEL_MIN,
      SUBARU_TURN_IN_RATE_LEVEL_MAX,
    ))
    self.mc_subaru_max_steering_experiment = self._get_bool_param("MCSubaruMaxSteeringExperiment")

  def _reset_angle_driver_override_ramp(self):
    self.angle_driver_override_ramp_frames = 0
    self.angle_driver_override_ramp_total_frames = ANGLE_DRIVER_OVERRIDE_RAMP_FRAMES
    self.angle_driver_override_ramp_start_angle = 0.0
    self.angle_driver_override_ramp_softness_exponent = ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_EXPONENTS[ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_DEFAULT]

  def _reset_angle_driver_override_release_guard(self):
    self.angle_driver_override_release_guard_pending = False
    self.angle_driver_override_release_guard_confirm_frames = 0
    self.angle_driver_override_release_guard_required_frames = 0
    self.angle_driver_override_release_guard_reference_angle = 0.0
    self.angle_driver_override_release_guard_rate_threshold = 0.0

  def _reset_angle_driver_override_soft_hold(self):
    self.angle_driver_override_soft_hold_frames = 0
    self.angle_driver_override_soft_hold_threshold = 0

  def _reset_angle_driver_override_state(self):
    self.angle_driver_override_hold_frames = 0
    self._reset_angle_driver_override_release_guard()
    self._reset_angle_driver_override_ramp()

  def _start_angle_driver_override_ramp(self, measured_angle: float):
    self.angle_driver_override_ramp_frames = ANGLE_DRIVER_OVERRIDE_RAMP_FRAMES
    self.angle_driver_override_ramp_total_frames = ANGLE_DRIVER_OVERRIDE_RAMP_FRAMES
    self.angle_driver_override_ramp_start_angle = measured_angle
    self.angle_driver_override_ramp_softness_exponent = self._get_resume_softness_exponent(
      self.mc_subaru_manual_yield_resume_softness
    )

  def _start_angle_driver_override_release_guard(self, measured_angle: float):
    self.angle_driver_override_release_guard_pending = True
    self.angle_driver_override_release_guard_confirm_frames = 0
    self.angle_driver_override_release_guard_required_frames = self._get_release_guard_confirm_frames(
      self.mc_subaru_manual_yield_release_guard_level
    )
    self.angle_driver_override_release_guard_reference_angle = measured_angle
    self.angle_driver_override_release_guard_rate_threshold = self._get_release_guard_rate_threshold(
      self.mc_subaru_manual_yield_release_guard_level
    )

  def _update_angle_driver_override_release_guard(self, measured_angle: float, steering_rate: float) -> bool:
    if not self.angle_driver_override_release_guard_pending:
      return False

    within_angle_window = abs(measured_angle - self.angle_driver_override_release_guard_reference_angle) <= \
      ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_ANGLE_DELTA
    within_rate_window = abs(steering_rate) <= self.angle_driver_override_release_guard_rate_threshold

    if within_angle_window and within_rate_window:
      self.angle_driver_override_release_guard_confirm_frames += 1
    else:
      self.angle_driver_override_release_guard_confirm_frames = 0
      self.angle_driver_override_release_guard_reference_angle = measured_angle

    if self.angle_driver_override_release_guard_confirm_frames >= self.angle_driver_override_release_guard_required_frames:
      self._reset_angle_driver_override_release_guard()
      return True

    return False

  def _update_angle_driver_override_soft_hold(self, steering_pressed: bool, lkas_allowed: bool,
                                             steering_torque: float) -> tuple[bool, int, int]:
    active_threshold = self._get_active_manual_yield_torque_threshold()
    level = self.mc_subaru_manual_steering_soft_hold_level
    soft_threshold = self._get_soft_hold_threshold(active_threshold, level)

    if not lkas_allowed or level <= ANGLE_DRIVER_OVERRIDE_SOFT_HOLD_LEVEL_MIN:
      self._reset_angle_driver_override_soft_hold()
      return steering_pressed, active_threshold, soft_threshold

    if steering_pressed:
      self.angle_driver_override_soft_hold_frames = ANGLE_DRIVER_OVERRIDE_SOFT_HOLD_FRAMES[level]
      self.angle_driver_override_soft_hold_threshold = soft_threshold
      return True, active_threshold, soft_threshold

    soft_pressure_active = (
      self.angle_driver_override_soft_hold_frames > 0
      and abs(steering_torque) >= soft_threshold
    )
    if self.angle_driver_override_soft_hold_frames > 0:
      self.angle_driver_override_soft_hold_frames -= 1

    if self.angle_driver_override_soft_hold_frames <= 0:
      self.angle_driver_override_soft_hold_threshold = 0
    else:
      self.angle_driver_override_soft_hold_threshold = soft_threshold

    return soft_pressure_active, active_threshold, soft_threshold

  def _update_angle_driver_override_state(self, steering_pressed: bool, lkas_allowed: bool,
                                          measured_angle: float, steering_rate: float) -> tuple[bool, bool]:
    if not lkas_allowed or not self._manual_yield_handoff_enabled():
      self._reset_angle_driver_override_state()
      return False, False

    if steering_pressed:
      self.angle_driver_override_hold_frames = ANGLE_DRIVER_OVERRIDE_HOLD_FRAMES
      self._reset_angle_driver_override_release_guard()
      self._reset_angle_driver_override_ramp()
      return True, False

    if self.angle_driver_override_hold_frames > 0:
      self.angle_driver_override_hold_frames -= 1
      if self.angle_driver_override_hold_frames == 0:
        if self.mc_subaru_manual_yield_release_guard_enabled:
          self._start_angle_driver_override_release_guard(measured_angle)
          return True, False
        if self.mc_subaru_manual_yield_resume_softness_enabled:
          return True, True
        return False, False
      return True, False

    if self.angle_driver_override_release_guard_pending:
      if self._update_angle_driver_override_release_guard(measured_angle, steering_rate):
        if self.mc_subaru_manual_yield_resume_softness_enabled:
          return True, True
        return False, False
      return True, False

    return False, False

  def _apply_angle_driver_override_ramp(self, live_steer_target: float) -> tuple[float, bool]:
    if self.angle_driver_override_ramp_frames <= 0:
      return live_steer_target, False

    progress = (self.angle_driver_override_ramp_total_frames - self.angle_driver_override_ramp_frames + 1) / \
      self.angle_driver_override_ramp_total_frames
    eased_progress = progress ** self.angle_driver_override_ramp_softness_exponent
    ramped_target = self.angle_driver_override_ramp_start_angle + eased_progress * (
      live_steer_target - self.angle_driver_override_ramp_start_angle
    )

    self.angle_driver_override_ramp_frames -= 1
    if self.angle_driver_override_ramp_frames <= 0:
      self._reset_angle_driver_override_ramp()

    return ramped_target, True

  def _get_angle_lkas_target(self, raw_target: float) -> float:
    # MostlyClueless: retired low-speed angle tuning; keep the angle target stock/raw.
    return raw_target

  def handle_angle_lateral(self, CC, CS):
    # Angle-LKAS can hard fault during very low-speed MADS lateral-only maneuvers.
    # Keep MADS behavior above 1 mph, but cap both measured angle and requested target in lateral-only mode.
    mads_only = CC.latActive and not CC.enabled
    drive_allowed = CS.out.gearShifter == structs.CarState.GearShifter.drive and not CS.out.standstill
    raw_steer_target = self._get_angle_lkas_target(CC.actuators.steeringAngleDeg)
    mads_only_selected_steer_angle = self._get_mads_only_max_steer_angle()
    mads_only_active_steer_angle = self._get_mads_only_active_steer_angle_cap(
      mads_only_selected_steer_angle,
      CS.out.vEgoRaw,
    )
    mads_angle_limited = mads_only and abs(CS.out.steeringAngleDeg) >= mads_only_active_steer_angle
    mads_fault_guard_active, mads_fault_guard_reason = self._update_mads_lkas_fault_guard(
      mads_only,
      CS.out.vEgoRaw,
      CS.out.steeringAngleDeg,
      CS.out.steeringRateDeg,
      mads_only_active_steer_angle,
      mads_angle_limited,
    )
    max_steering_low_speed_guard_active, max_steering_low_speed_guard_reason = self._update_max_steering_low_speed_guard(
      mads_only and drive_allowed,
      CS.out.vEgoRaw,
      raw_steer_target,
      CS.out.steeringAngleDeg,
      CS.out.steeringRateDeg,
    )
    mads_only_ok = (
      CS.out.vEgoRaw > MADS_ONLY_MIN_SPEED
      and not mads_angle_limited
      and not mads_fault_guard_active
      and not max_steering_low_speed_guard_active
    )
    lkas_allowed = CC.latActive and (CC.enabled or not mads_only or mads_only_ok) and drive_allowed
    effective_steering_pressed, manual_yield_threshold, soft_hold_threshold = self._update_angle_driver_override_soft_hold(
      CS.out.steeringPressed,
      lkas_allowed,
      CS.out.steeringTorque,
    )
    angle_driver_override, ramp_will_start = self._update_angle_driver_override_state(
      effective_steering_pressed,
      lkas_allowed,
      CS.out.steeringAngleDeg,
      CS.out.steeringRateDeg,
    )
    mads_only_inhibited = mads_only and not mads_only_ok
    self.subaru_manual_yield_full_release_active = lkas_allowed and (effective_steering_pressed or angle_driver_override)
    manual_yield_active = angle_driver_override or self.subaru_manual_yield_full_release_active
    lkas_request = lkas_allowed and not manual_yield_active
    self.subaru_effective_lkas_active = CC.latActive and not self.subaru_manual_yield_full_release_active and not mads_only_inhibited

    inhibit_reason = "none"
    if not CC.latActive:
      inhibit_reason = "lat_inactive"
    elif self.subaru_manual_yield_full_release_active:
      inhibit_reason = "manual_yield_full_release"
    elif angle_driver_override:
      inhibit_reason = "manual_override"
    elif CS.out.gearShifter != structs.CarState.GearShifter.drive:
      inhibit_reason = "gear_not_drive"
    elif CS.out.standstill:
      inhibit_reason = "standstill"
    elif mads_only and CS.out.vEgoRaw <= MADS_ONLY_MIN_SPEED:
      inhibit_reason = "mads_below_min_speed"
    elif mads_angle_limited:
      inhibit_reason = "mads_angle_limit"
    elif mads_only and max_steering_low_speed_guard_active:
      inhibit_reason = max_steering_low_speed_guard_reason
    elif mads_only and mads_fault_guard_active:
      inhibit_reason = mads_fault_guard_reason
    elif mads_only and not mads_only_ok:
      inhibit_reason = "mads_inhibited"

    self._log_transition("angle_lkas_inhibit", inhibit_reason, f"angle LKAS inhibit={inhibit_reason}")
    self._log_transition(
      "angle_driver_override_hold",
      self.angle_driver_override_hold_frames > 0,
      (
        f"angle driver override hold active={self.angle_driver_override_hold_frames > 0} "
        + f"frames={self.angle_driver_override_hold_frames} steeringPressed={CS.out.steeringPressed} "
        + f"effectiveSteeringPressed={effective_steering_pressed}"
      ),
    )
    self._log_transition(
      "angle_driver_override_soft_hold",
      (
        self.mc_subaru_manual_steering_soft_hold_level,
        self.angle_driver_override_soft_hold_frames,
        effective_steering_pressed,
        manual_yield_threshold,
        soft_hold_threshold,
      ),
      (
        f"angle driver override soft hold level={self.mc_subaru_manual_steering_soft_hold_level} "
        + f"frames={self.angle_driver_override_soft_hold_frames} steeringPressed={CS.out.steeringPressed} "
        + f"effectiveSteeringPressed={effective_steering_pressed} steeringTorque={CS.out.steeringTorque:.2f} "
        + f"activeThreshold={manual_yield_threshold} softThreshold={soft_hold_threshold}"
      ),
    )
    self._log_transition(
      "angle_driver_override_release_guard",
      self.angle_driver_override_release_guard_pending,
      (
        f"angle driver override release guard active={self.angle_driver_override_release_guard_pending} "
        + f"frames={self.angle_driver_override_release_guard_confirm_frames}/"
        + f"{self.angle_driver_override_release_guard_required_frames} "
        + f"referenceAngle={self.angle_driver_override_release_guard_reference_angle:.2f} "
        + f"rateThreshold={self.angle_driver_override_release_guard_rate_threshold:.2f}"
      ),
    )

    steer_target = raw_steer_target

    if ramp_will_start:
      self._start_angle_driver_override_ramp(CS.out.steeringAngleDeg)

    if lkas_request:
      steer_target, manual_override_ramp_active = self._apply_angle_driver_override_ramp(steer_target)
    else:
      manual_override_ramp_active = False

    handoff_active = angle_driver_override or ramp_will_start or manual_override_ramp_active
    effective_lkas_recapture_active = self.subaru_effective_lkas_active

    self._log_transition(
      "angle_driver_override_ramp",
      manual_override_ramp_active,
      (
        f"angle driver override ramp active={manual_override_ramp_active} "
        + f"framesRemaining={self.angle_driver_override_ramp_frames} totalFrames={self.angle_driver_override_ramp_total_frames} "
        + f"softnessExponent={self.angle_driver_override_ramp_softness_exponent:.2f} "
        + f"start={self.angle_driver_override_ramp_start_angle:.2f} "
        + f"steerTarget={steer_target:.2f}"
      ),
    )

    self._log_transition(
      "angle_manual_yield_full_release",
      self.subaru_manual_yield_full_release_active,
      (
        f"angle manual yield full release active={self.subaru_manual_yield_full_release_active} "
        + f"steeringPressed={CS.out.steeringPressed} "
        + f"effectiveSteeringPressed={effective_steering_pressed} "
        + f"manualYieldActive={manual_yield_active} effectiveLkasState={self.subaru_effective_lkas_active}"
      ),
    )
    self._log_transition(
      "mads_lkas_fault_guard",
      (mads_fault_guard_active, mads_fault_guard_reason, self.mads_lkas_fault_guard_quiet_frames,
       self.mads_lkas_fault_guard_source),
      (
        f"mads LKAS fault guard active={mads_fault_guard_active} reason={mads_fault_guard_reason} "
        + f"source={self.mads_lkas_fault_guard_source} quietFrames={self.mads_lkas_fault_guard_quiet_frames} "
        + f"selectedCap={mads_only_selected_steer_angle:.2f} activeCap={mads_only_active_steer_angle:.2f} "
        + f"measuredAngle={CS.out.steeringAngleDeg:.2f} measuredRate={CS.out.steeringRateDeg:.2f} "
        + f"speed={CS.out.vEgoRaw:.2f} madsOnly={mads_only}"
      ),
    )
    self._log_transition(
      "max_steering_low_speed_guard",
      (
        self.mc_subaru_max_steering_experiment,
        max_steering_low_speed_guard_active,
        max_steering_low_speed_guard_reason,
        self.max_steering_low_speed_guard_quiet_frames,
        self.max_steering_low_speed_guard_active,
        lkas_request,
      ),
      (
        f"max steering low-speed guard max={self.mc_subaru_max_steering_experiment} "
        + f"active={max_steering_low_speed_guard_active} reason={max_steering_low_speed_guard_reason} "
        + f"quietFrames={self.max_steering_low_speed_guard_quiet_frames} "
        + f"latched={self.max_steering_low_speed_guard_active} "
        + f"target={raw_steer_target:.2f} measuredAngle={CS.out.steeringAngleDeg:.2f} "
        + f"measuredRate={CS.out.steeringRateDeg:.2f} speed={CS.out.vEgoRaw:.2f} "
        + f"madsOnly={mads_only} lkasRequest={lkas_request}"
      ),
    )

    if effective_lkas_recapture_active and not self.lat_active_prev and lkas_request:
      self.soft_capture_frame = self.frame
    self.lat_active_prev = effective_lkas_recapture_active

    if lkas_request:
      steer_target = self._get_soft_capture_angle(steer_target, CS.out.steeringAngleDeg)

    mads_target_before_cap = steer_target
    steer_target, mads_target_clamped = self._apply_mads_only_steer_target_cap(
      steer_target,
      mads_only,
      lkas_request,
      mads_only_active_steer_angle,
    )

    apply_steer = apply_std_steer_angle_limits(
      steer_target,
      self.apply_angle_last,
      CS.out.vEgoRaw,
      CS.out.steeringAngleDeg,
      lkas_request,
      self._get_active_angle_limits(mads_only),
    )

    apply_steer_before_mads_cap = apply_steer
    apply_steer, mads_apply_clamped = self._apply_mads_only_steer_target_cap(
      apply_steer,
      mads_only,
      lkas_request,
      mads_only_active_steer_angle,
    )

    apply_steer_before_reenable_clamp = apply_steer
    apply_steer, reenable_unwind_clamp_active, reenable_unwind_clamped = self._apply_mads_lkas_reenable_unwind_clamp(
      apply_steer,
      lkas_request,
    )

    if not lkas_request:
      apply_steer = CS.out.steeringAngleDeg

    mads_cap_clamped = mads_target_clamped or mads_apply_clamped
    self._log_transition(
      "mads_only_target_cap",
      mads_cap_clamped,
      (
        f"mads target cap clamped={mads_cap_clamped} rawTarget={raw_steer_target:.2f} "
        + f"targetBeforeCap={mads_target_before_cap:.2f} cappedTarget={steer_target:.2f} "
        + f"applyBeforeCap={apply_steer_before_mads_cap:.2f} apply={apply_steer:.2f} "
        + f"selectedCap={mads_only_selected_steer_angle:.2f} activeCap={mads_only_active_steer_angle:.2f} "
        + f"measuredAngle={CS.out.steeringAngleDeg:.2f} speed={CS.out.vEgoRaw:.2f} madsOnly={mads_only}"
      ),
    )
    self._log_transition(
      "mads_lkas_reenable_unwind_clamp",
      (reenable_unwind_clamp_active, reenable_unwind_clamped, self.mads_lkas_reenable_unwind_clamp_frames),
      (
        f"mads LKAS reenable unwind clamp active={reenable_unwind_clamp_active} clamped={reenable_unwind_clamped} "
        + f"framesRemaining={self.mads_lkas_reenable_unwind_clamp_frames} "
        + f"before={apply_steer_before_reenable_clamp:.2f} apply={apply_steer:.2f} "
        + f"lastApplied={self.apply_angle_last:.2f} step={MADS_ONLY_FAULT_GUARD_REENABLE_UNWIND_STEP:.2f}"
      ),
    )
    self._log_transition(
      "angle_lkas_request",
      lkas_request,
      (
        f"angle LKAS request={lkas_request} inhibit={inhibit_reason} rawTarget={raw_steer_target:.2f} "
        + f"target={steer_target:.2f} apply={apply_steer:.2f} lastApplied={self.apply_angle_last:.2f} "
        + f"measuredAngle={CS.out.steeringAngleDeg:.2f} speed={CS.out.vEgoRaw:.2f} "
        + f"madsOnly={mads_only} madsSelectedCap={mads_only_selected_steer_angle:.2f} "
        + f"madsActiveCap={mads_only_active_steer_angle:.2f} madsCapClamped={mads_cap_clamped} "
        + f"madsFaultGuardActive={mads_fault_guard_active} madsFaultGuardReason={mads_fault_guard_reason} "
        + f"maxGuardActive={max_steering_low_speed_guard_active} maxGuardReason={max_steering_low_speed_guard_reason} "
        + f"measuredRate={CS.out.steeringRateDeg:.2f} "
        + f"handoffActive={handoff_active} rampActive={manual_override_ramp_active} "
        + f"manualYieldActive={manual_yield_active} steeringTorque={CS.out.steeringTorque:.2f} "
        + f"effectiveSteeringPressed={effective_steering_pressed} "
        + f"fullReleaseActive={self.subaru_manual_yield_full_release_active} effectiveLkasState={self.subaru_effective_lkas_active} "
        + f"latActive={CC.latActive} enabled={CC.enabled}"
      ),
    )

    self.apply_angle_last = apply_steer
    return subarucan.create_steering_control_angle(self.packer, apply_steer, lkas_request)

  def handle_torque_lateral(self, CC, CS):
    apply_torque = int(round(CC.actuators.torque * self.p.STEER_MAX))

    new_torque = int(round(apply_torque))
    apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.p)

    if not CC.latActive:
      apply_torque = 0

    if self.CP.flags & SubaruFlags.PREGLOBAL:
      msg = subarucan.create_preglobal_steering_control(self.packer, self.frame // self.p.STEER_STEP, apply_torque, CC.latActive)
    else:
      apply_steer_req = CC.latActive

      if self.CP.flags & SubaruFlags.STEER_RATE_LIMITED:
        # Steering rate fault prevention
        self.steer_rate_counter, apply_steer_req = common_fault_avoidance(
          abs(CS.out.steeringRateDeg) > MAX_STEER_RATE,
          apply_steer_req,
          self.steer_rate_counter,
          MAX_STEER_RATE_FRAMES,
        )

      msg = subarucan.create_steering_control(self.packer, apply_torque, apply_steer_req)

    self.apply_torque_last = apply_torque
    return msg

  def update(self, CC, CC_SP, CS, now_nanos):
    if self.frame % 100 == 0:
      self._update_params()

    actuators = CC.actuators
    hud_control = CC.hudControl
    pcm_cancel_cmd = CC.cruiseControl.cancel

    can_sends = []

    # *** steering ***
    if (self.frame % self.p.STEER_STEP) == 0:
      if self.CP.flags & SubaruFlags.LKAS_ANGLE:
        can_sends.append(self.handle_angle_lateral(CC, CS))
      else:
        can_sends.append(self.handle_torque_lateral(CC, CS))

    # *** longitudinal ***

    if CC.longActive:
      apply_throttle = int(round(np.interp(actuators.accel, CarControllerParams.THROTTLE_LOOKUP_BP, CarControllerParams.THROTTLE_LOOKUP_V)))
      apply_rpm = int(round(np.interp(actuators.accel, CarControllerParams.RPM_LOOKUP_BP, CarControllerParams.RPM_LOOKUP_V)))
      apply_brake = int(round(np.interp(actuators.accel, CarControllerParams.BRAKE_LOOKUP_BP, CarControllerParams.BRAKE_LOOKUP_V)))

      # limit min and max values
      cruise_throttle = np.clip(apply_throttle, CarControllerParams.THROTTLE_MIN, CarControllerParams.THROTTLE_MAX)
      cruise_rpm = np.clip(apply_rpm, CarControllerParams.RPM_MIN, CarControllerParams.RPM_MAX)
      cruise_brake = np.clip(apply_brake, CarControllerParams.BRAKE_MIN, CarControllerParams.BRAKE_MAX)
    else:
      cruise_throttle = CarControllerParams.THROTTLE_INACTIVE
      cruise_rpm = CarControllerParams.RPM_MIN
      cruise_brake = CarControllerParams.BRAKE_MIN

    # *** alerts and pcm cancel ***
    if self.CP.flags & SubaruFlags.PREGLOBAL:
      if self.frame % 5 == 0:
        # 1 = main, 2 = set shallow, 3 = set deep, 4 = resume shallow, 5 = resume deep
        # disengage ACC when OP is disengaged
        if pcm_cancel_cmd:
          cruise_button = 1
        # turn main on if off and past start-up state
        elif not CS.out.cruiseState.available and CS.ready:
          cruise_button = 1
        else:
          cruise_button = CS.cruise_button

        # unstick previous mocked button press
        if cruise_button == 1 and self.cruise_button_prev == 1:
          cruise_button = 0
        self.cruise_button_prev = cruise_button

        can_sends.append(subarucan.create_preglobal_es_distance(self.packer, cruise_button, CS.es_distance_msg))

    else:
      if self.frame % 10 == 0:
        subaru_lkas_state_active = CC.latActive
        if self.CP.flags & SubaruFlags.LKAS_ANGLE:
          subaru_lkas_state_active = self.subaru_effective_lkas_active

        can_sends.append(subarucan.create_es_dashstatus(self.packer, self.frame // 10, CS.es_dashstatus_msg, CC.enabled,
                                                        self.CP.openpilotLongitudinalControl, CC.longActive, hud_control.leadVisible))

        can_sends.append(subarucan.create_es_lkas_state(self.packer, self.frame // 10, CS.es_lkas_state_msg, subaru_lkas_state_active, hud_control.visualAlert,
                                                        hud_control.leftLaneVisible, hud_control.rightLaneVisible,
                                                        hud_control.leftLaneDepart, hud_control.rightLaneDepart))

        if self.CP.flags & SubaruFlags.SEND_INFOTAINMENT:
          can_sends.append(subarucan.create_es_infotainment(self.packer, self.frame // 10, CS.es_infotainment_msg, hud_control.visualAlert))

      if self.CP.openpilotLongitudinalControl:
        if self.frame % 5 == 0:
          can_sends.append(subarucan.create_es_status(self.packer, self.frame // 5, CS.es_status_msg,
                                                      self.CP.openpilotLongitudinalControl, CC.longActive, cruise_rpm))

          can_sends.append(subarucan.create_es_brake(self.packer, self.frame // 5, CS.es_brake_msg,
                                                     self.CP.openpilotLongitudinalControl, CC.longActive, cruise_brake))

          can_sends.append(subarucan.create_es_distance(self.packer, self.frame // 5, CS.es_distance_msg, 0, pcm_cancel_cmd,
                                                        self.CP.openpilotLongitudinalControl, cruise_brake > 0, cruise_throttle))
      else:
        if pcm_cancel_cmd:
          if not (self.CP.flags & SubaruFlags.HYBRID):
            bus = CanBus.alt if self.CP.flags & SubaruFlags.GLOBAL_GEN2 else CanBus.main
            can_sends.append(subarucan.create_es_distance(self.packer, CS.es_distance_msg["COUNTER"] + 1, CS.es_distance_msg, bus, pcm_cancel_cmd))

      if self.CP.flags & SubaruFlags.DISABLE_EYESIGHT:
        # Tester present (keeps eyesight disabled)
        if self.frame % 100 == 0:
          can_sends.append(make_tester_present_msg(GLOBAL_ES_ADDR, CanBus.camera, suppress_response=True))

        # Create all of the other eyesight messages to keep the rest of the car happy when eyesight is disabled
        if self.frame % 5 == 0:
          can_sends.append(subarucan.create_es_highbeamassist(self.packer))

        if self.frame % 10 == 0:
          can_sends.append(subarucan.create_es_static_1(self.packer))

        if self.frame % 2 == 0:
          can_sends.append(subarucan.create_es_static_2(self.packer))

    can_sends.extend(SnGCarController.create_stop_and_go(self, self.packer, CC, CS, self.frame))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last
    new_actuators.torque = self.apply_torque_last / self.p.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last

    self.frame += 1
    return new_actuators, can_sends
