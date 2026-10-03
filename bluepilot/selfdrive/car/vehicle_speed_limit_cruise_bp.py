"""Track a driver-selected cruise target using Ford's recognized speed limits.

This changes only the software speed ceiling. It does not engage cruise, send
button presses, change the Ford cluster, or replace the acceleration planner.
"""
import math

from openpilot.common.constants import CV
from openpilot.selfdrive.car.cruise import VCruiseHelper, V_CRUISE_MIN, V_CRUISE_MAX
from opendbc.car.ford.values import CAR

PARAM_BP = "BPFollowVehicleSpeedLimits"
MATCH_TOLERANCE_KPH_BP = 1.0
LIMIT_SETTLE_FRAMES_BP = 100  # card runs at 100 Hz; require a stable sign for one second
BUTTON_FEEDBACK_FRAMES_BP = 100
SET_ADJUST_FEEDBACK_WINDOW_KPH_BP = 2.5  # SET rounding plus a single short +/- selection


def valid_speed_bp(value: float) -> bool:
  return math.isfinite(value) and V_CRUISE_MIN <= value <= V_CRUISE_MAX


class VehicleSpeedLimitCruiseBP:
  """Maintain the saved target through cancel/brake and delayed PCM feedback."""

  def __init__(self):
    self.frame = 0
    self.limit_candidate = None
    self.limit_frames = 0
    self.limit_kph = None
    self.reset_session()

  def reset_session(self):
    """Forget saved software speed and intent when cruise main is turned off."""
    self.target_kph = None
    self.following = False
    self.initialized = False
    self.arm_requested = False
    self.stock_last_kph = None
    self.set_pending = False
    self.set_deadline = 0
    self.set_target_hint = None
    self.adjust_until = 0
    self.adjust_feedback_pending = False
    self.adjust_direction = 0
    self.adjust_buttons = set()

  def update(self, *, available: bool, enabled: bool, can_valid: bool,
             stock_kph: float, ego_kph: float, limit_kph: float,
             buttons: list[tuple[str, bool]]) -> float | None:
    """Return a saved target, or None to retain the ordinary cruise helper."""
    self.frame += 1
    if not can_valid:
      return self.target_kph

    limit_valid = valid_speed_bp(limit_kph)
    if limit_valid:
      if self.limit_candidate != limit_kph:
        self.limit_candidate, self.limit_frames = limit_kph, 1
      else:
        self.limit_frames += 1
      if self.limit_frames >= LIMIT_SETTLE_FRAMES_BP:
        self.limit_kph = limit_kph
    else:
      self.limit_candidate, self.limit_frames = None, 0
    current_limit_ready = limit_valid and self.limit_kph == limit_kph

    if not available:
      self.reset_session()
      return None

    stock_valid = valid_speed_bp(stock_kph)
    stock_changed = stock_valid and self.stock_last_kph is not None and stock_kph != self.stock_last_kph
    manual_adjustment = False
    for name, pressed in buttons:
      if name in ("accelCruise", "decelCruise"):
        if pressed:
          self.adjust_buttons.add(name)
          if self.initialized:
            self.following = False
            self.arm_requested = False
            self.adjust_feedback_pending = True
            self.adjust_direction = 1 if name == "accelCruise" else -1
        else:
          self.adjust_buttons.discard(name)
        self.adjust_until = self.frame + BUTTON_FEEDBACK_FRAMES_BP
      elif name == "setCruise" and pressed:
        self.following = False
        self.arm_requested = True
        self.set_pending = True
        self.set_deadline = self.frame + BUTTON_FEEDBACK_FRAMES_BP
        self.set_target_hint = ego_kph if valid_speed_bp(ego_kph) else None
        self.adjust_until = 0
        self.adjust_feedback_pending = False
        self.adjust_direction = 0
        self.adjust_buttons.clear()
        if self.initialized and valid_speed_bp(ego_kph):
          self.target_kph = ego_kph
      elif name == "resumeCruise" and pressed and not self.initialized:
        # The first Resume selects an existing saved speed. Later Resumes must
        # retain our current target, including a manual override.
        self.arm_requested = True
      elif name in ("cancel", "mainCruise") and pressed:
        self.set_pending = False
        self.arm_requested = False
        self.adjust_buttons.clear()
        self.adjust_until = 0
        self.adjust_feedback_pending = False

    if self.adjust_buttons:
      self.adjust_until = self.frame + BUTTON_FEEDBACK_FRAMES_BP

    if enabled and stock_valid:
      set_feedback_matches = (self.set_target_hint is not None and
                              abs(stock_kph - self.set_target_hint) <= MATCH_TOLERANCE_KPH_BP)
      set_feedback_changed = stock_changed
      if self.adjust_feedback_pending and self.set_target_hint is not None and self.stock_last_kph is not None:
        # SET and '+' can be pressed before the PCM echoes either action. An
        # echo near the new SET target belongs to SET; a delta near the old
        # divergent PCM target does not confirm that SET succeeded.
        old_distance = abs(self.stock_last_kph - self.set_target_hint)
        set_feedback_changed = stock_changed and (old_distance <= MATCH_TOLERANCE_KPH_BP or
                                                   abs(stock_kph - self.set_target_hint) <= SET_ADJUST_FEEDBACK_WINDOW_KPH_BP)
      if self.set_pending and (set_feedback_changed or (self.frame >= self.set_deadline and set_feedback_matches)):
        # An unchanged old PCM speed is never confirmation of a new SET. Keep
        # the provisional target if feedback takes longer than the guard time,
        # and still accept the matching/new PCM value when it finally arrives.
        self.target_kph = stock_kph
        self.initialized = True
        self.set_pending = False
        manual_adjustment = True
        if (self.adjust_feedback_pending and self.set_target_hint is not None and
            self.adjust_direction * (stock_kph - self.set_target_hint) > 0.25):
          # A combined SET/+ echo already contains the manual selection.
          self.adjust_feedback_pending = False
          self.arm_requested = True
      elif not self.initialized:
        self.target_kph = ego_kph if self.set_pending and valid_speed_bp(ego_kph) else stock_kph
        self.initialized = True

    if (self.initialized and not self.set_pending and not manual_adjustment and stock_changed and
        (self.frame <= self.adjust_until or self.adjust_feedback_pending) and self.stock_last_kph is not None):
      # The PCM may still store 70 while our target is 40. A physical '+' that
      # changes the PCM to 71 means 41 here, not a jump back to 71.
      delta = stock_kph - self.stock_last_kph
      if abs(delta) <= 20.0:
        self.target_kph = min(max(self.target_kph + delta, V_CRUISE_MIN), V_CRUISE_MAX)
        manual_adjustment = True
        self.arm_requested = True
      self.adjust_feedback_pending = False

    if stock_valid:
      self.stock_last_kph = stock_kph

    if (self.initialized and self.arm_requested and not self.set_pending and
        not self.adjust_buttons and not self.adjust_feedback_pending and
        self.frame > self.adjust_until and current_limit_ready and
        abs(self.target_kph - self.limit_kph) <= MATCH_TOLERANCE_KPH_BP):
      self.following = True
      self.arm_requested = False

    if self.following and current_limit_ready and not manual_adjustment:
      self.target_kph = self.limit_kph
    return self.target_kph


class VCruiseHelperBP(VCruiseHelper):
  """Default-off Ford extension; the ordinary helper owns all other setups."""

  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.follow_vehicle_limits_bp = bool(
      self.params.get_bool(PARAM_BP) and CP.brand == "ford" and
      CP.carFingerprint == str(CAR.FORD_F_150_MK14) and CP.openpilotLongitudinalControl and
      CP.pcmCruise and CP_SP.pcmCruiseSpeed and not CP.dashcamOnly and not CP.passive
    )
    self.vehicle_limit_kph_bp = 0.0
    self.vehicle_cruise_bp = VehicleSpeedLimitCruiseBP()

  def update_vehicle_speed_limit_bp(self, speed_limit_ms: float):
    """Accept the current vehicle camera limit in its schema-defined SI units."""
    self.vehicle_limit_kph_bp = speed_limit_ms * CV.MS_TO_KPH

  def update_v_cruise(self, CS, enabled, is_metric):
    super().update_v_cruise(CS, enabled, is_metric)
    if self.follow_vehicle_limits_bp:
      target = self.vehicle_cruise_bp.update(
        available=CS.cruiseState.available, enabled=CS.cruiseState.enabled, can_valid=CS.canValid,
        stock_kph=CS.cruiseState.speed * CV.MS_TO_KPH,
        ego_kph=(CS.vEgoCluster if CS.vEgoCluster > 0.0 else CS.vEgo) * CV.MS_TO_KPH,
        limit_kph=self.vehicle_limit_kph_bp, buttons=[(str(b.type), b.pressed) for b in CS.buttonEvents],
      )
      if target is not None:
        self.v_cruise_kph = self.v_cruise_cluster_kph = target

  def initialize_v_cruise(self, CS, experimental_mode: bool, dynamic_experimental_control: bool):
    # card initializes from the previous CS after updating from current CS.
    # Never replace a freshly followed limit with that older PCM set speed.
    if self.follow_vehicle_limits_bp and self.vehicle_cruise_bp.target_kph is not None:
      self.v_cruise_kph = self.v_cruise_cluster_kph = self.vehicle_cruise_bp.target_kph
    else:
      super().initialize_v_cruise(CS, experimental_mode, dynamic_experimental_control)
