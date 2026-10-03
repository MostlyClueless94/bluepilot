"""Saved cruise speed follows vehicle signs without changing engagement."""
import math

import pytest

from cereal import car, custom
from opendbc.car.ford.values import CAR
from openpilot.common.constants import CV
from openpilot.bluepilot.selfdrive.car.vehicle_speed_limit_cruise_bp import (
  BUTTON_FEEDBACK_FRAMES_BP, LIMIT_SETTLE_FRAMES_BP, PARAM_BP,
  VCruiseHelperBP, VehicleSpeedLimitCruiseBP,
)


def mph(value):
  return value * CV.MPH_TO_KPH


def drive(state, frames=1, **changes):
  values = dict(available=True, enabled=True, can_valid=True, stock_kph=mph(70),
                ego_kph=mph(70), limit_kph=mph(70), buttons=[])
  values.update(changes)
  for _ in range(frames):
    result = state.update(**values)
  return result


def arm_at_70():
  state = VehicleSpeedLimitCruiseBP()
  drive(state, enabled=False)
  drive(state, enabled=False, buttons=[("setCruise", True)])
  drive(state, buttons=[("setCruise", False)])
  drive(state, max(LIMIT_SETTLE_FRAMES_BP, BUTTON_FEEDBACK_FRAMES_BP) + 1)
  assert state.following
  assert state.target_kph == pytest.approx(mph(70))
  return state


def test_matching_passive_stock_speed_does_not_arm():
  state = VehicleSpeedLimitCruiseBP()
  drive(state, LIMIT_SETTLE_FRAMES_BP + BUTTON_FEEDBACK_FRAMES_BP + 2)
  assert not state.following
  drive(state, LIMIT_SETTLE_FRAMES_BP, limit_kph=mph(40))
  assert state.target_kph == pytest.approx(mph(70))


@pytest.mark.parametrize("pause_button", [[], [("cancel", True)]])
def test_saved_speed_follows_new_sign_while_paused_and_resume_keeps_it(pause_button):
  state = arm_at_70()
  drive(state, enabled=False, buttons=pause_button)
  drive(state, LIMIT_SETTLE_FRAMES_BP, enabled=False, ego_kph=mph(40), limit_kph=mph(40))
  assert state.target_kph == pytest.approx(mph(40))
  assert state.following

  drive(state, enabled=False, limit_kph=mph(40), buttons=[("resumeCruise", True)])
  drive(state, enabled=True, limit_kph=mph(40), buttons=[("resumeCruise", False)])
  drive(state, 10, enabled=True, limit_kph=mph(40))
  assert state.target_kph == pytest.approx(mph(40))


def test_rejected_short_sign_change_preserves_saved_target():
  state = arm_at_70()
  drive(state, LIMIT_SETTLE_FRAMES_BP - 1, enabled=False, limit_kph=mph(40))
  assert state.target_kph == pytest.approx(mph(70))
  drive(state, limit_kph=mph(70))
  assert state.target_kph == pytest.approx(mph(70))


def test_manual_plus_translates_delayed_pcm_delta_and_pauses_following():
  state = arm_at_70()
  drive(state, LIMIT_SETTLE_FRAMES_BP, limit_kph=mph(40))
  drive(state, limit_kph=mph(40), buttons=[("accelCruise", True)])
  drive(state, limit_kph=mph(40), buttons=[("accelCruise", False)])
  drive(state, 5, limit_kph=mph(40))
  assert not state.following
  assert drive(state, stock_kph=mph(71), limit_kph=mph(40)) == pytest.approx(mph(41))
  drive(state, BUTTON_FEEDBACK_FRAMES_BP + 1, stock_kph=mph(71), limit_kph=mph(40))
  drive(state, LIMIT_SETTLE_FRAMES_BP, stock_kph=mph(71), limit_kph=mph(50))
  assert state.target_kph == pytest.approx(mph(41))
  assert not state.following
  drive(state, enabled=False, stock_kph=mph(71), limit_kph=mph(50))
  drive(state, stock_kph=mph(71), limit_kph=mph(50), buttons=[("resumeCruise", True)])
  assert state.target_kph == pytest.approx(mph(41))


def test_manual_minus_takes_priority_on_sign_settling_frame():
  state = arm_at_70()
  drive(state, LIMIT_SETTLE_FRAMES_BP - 1, limit_kph=mph(40))
  result = drive(state, stock_kph=mph(69), limit_kph=mph(40), buttons=[("decelCruise", True)])
  assert result == pytest.approx(mph(69))
  assert not state.following


def test_held_button_accumulates_pcm_deltas_once_each():
  state = arm_at_70()
  drive(state, LIMIT_SETTLE_FRAMES_BP, limit_kph=mph(40))
  drive(state, limit_kph=mph(40), buttons=[("accelCruise", True)])
  for stock in (71, 72, 73):
    drive(state, 10, stock_kph=mph(stock), limit_kph=mph(40))
  assert state.target_kph == pytest.approx(mph(43))
  drive(state, stock_kph=mph(73), limit_kph=mph(40), buttons=[("accelCruise", False)])
  assert not state.following


@pytest.mark.parametrize("invalid", [0, -1, math.nan, math.inf, -math.inf, 300])
def test_invalid_sign_does_not_replace_saved_target(invalid):
  state = arm_at_70()
  drive(state, LIMIT_SETTLE_FRAMES_BP + 2, enabled=False, limit_kph=invalid)
  assert state.target_kph == pytest.approx(mph(70))


@pytest.mark.parametrize("invalid", [0, -1, math.nan, math.inf, mph(254), mph(255)])
def test_invalid_pcm_speed_does_not_corrupt_manual_target(invalid):
  state = arm_at_70()
  drive(state, LIMIT_SETTLE_FRAMES_BP, limit_kph=mph(40))
  drive(state, limit_kph=mph(40), buttons=[("accelCruise", True)])
  drive(state, stock_kph=invalid, limit_kph=mph(40))
  assert state.target_kph == pytest.approx(mph(40))
  drive(state, stock_kph=mph(71), limit_kph=mph(40))
  assert state.target_kph == pytest.approx(mph(41))


def test_invalid_can_cannot_accept_sign_or_button_or_pcm_change():
  state = arm_at_70()
  drive(state, LIMIT_SETTLE_FRAMES_BP + 2, can_valid=False, stock_kph=mph(71),
        limit_kph=mph(40), buttons=[("accelCruise", True)])
  assert state.target_kph == pytest.approx(mph(70))
  assert state.following
  assert not state.adjust_buttons


def test_set_while_paused_adopts_new_pcm_value_after_delayed_feedback():
  state = arm_at_70()
  drive(state, LIMIT_SETTLE_FRAMES_BP, enabled=False, limit_kph=mph(40))
  drive(state, enabled=False, ego_kph=mph(45), limit_kph=mph(40), buttons=[("setCruise", True)])
  assert state.target_kph == pytest.approx(mph(45))
  assert not state.following
  drive(state, enabled=True, ego_kph=mph(45), limit_kph=mph(40), buttons=[("setCruise", False)])
  drive(state, 5, enabled=True, ego_kph=mph(45), limit_kph=mph(40))
  assert state.target_kph == pytest.approx(mph(45))
  drive(state, enabled=True, stock_kph=mph(45), ego_kph=mph(45), limit_kph=mph(40))
  assert state.target_kph == pytest.approx(mph(45))
  assert not state.set_pending


def test_first_set_uses_current_speed_until_pcm_feedback_arrives():
  state = VehicleSpeedLimitCruiseBP()
  drive(state, enabled=False, stock_kph=mph(70), ego_kph=mph(40), limit_kph=mph(40))
  drive(state, enabled=False, stock_kph=mph(70), ego_kph=mph(40), limit_kph=mph(40),
        buttons=[("setCruise", True)])
  assert drive(state, enabled=True, stock_kph=mph(70), ego_kph=mph(40), limit_kph=mph(40)) == pytest.approx(mph(40))
  drive(state, stock_kph=mph(40), ego_kph=mph(40), limit_kph=mph(40))
  drive(state, LIMIT_SETTLE_FRAMES_BP + 1, stock_kph=mph(40), ego_kph=mph(40), limit_kph=mph(40))
  assert state.following
  assert state.target_kph == pytest.approx(mph(40))


def test_first_set_does_not_revert_to_old_pcm_value_after_feedback_guard():
  state = VehicleSpeedLimitCruiseBP()
  drive(state, enabled=False, ego_kph=mph(40), limit_kph=mph(40))
  drive(state, enabled=False, ego_kph=mph(40), limit_kph=mph(40), buttons=[("setCruise", True)])
  drive(state, ego_kph=mph(40), limit_kph=mph(40), buttons=[("setCruise", False)])
  drive(state, LIMIT_SETTLE_FRAMES_BP + BUTTON_FEEDBACK_FRAMES_BP + 20,
        ego_kph=mph(40), limit_kph=mph(40))
  assert state.target_kph == pytest.approx(mph(40))
  assert state.set_pending
  assert not state.following
  drive(state, stock_kph=mph(40), ego_kph=mph(40), limit_kph=mph(40))
  assert not state.set_pending
  assert state.following
  assert state.target_kph == pytest.approx(mph(40))


def test_manual_plus_waits_for_late_feedback_without_rearming_old_target():
  state = arm_at_70()
  drive(state, LIMIT_SETTLE_FRAMES_BP, limit_kph=mph(40))
  drive(state, limit_kph=mph(40), buttons=[("accelCruise", True)])
  drive(state, limit_kph=mph(40), buttons=[("accelCruise", False)])
  drive(state, BUTTON_FEEDBACK_FRAMES_BP + 20, limit_kph=mph(40))
  assert state.target_kph == pytest.approx(mph(40))
  assert not state.following
  drive(state, stock_kph=mph(71), limit_kph=mph(40))
  assert state.target_kph == pytest.approx(mph(41))
  assert not state.following
  drive(state, LIMIT_SETTLE_FRAMES_BP, stock_kph=mph(71), limit_kph=mph(50))
  assert state.target_kph == pytest.approx(mph(41))
  assert not state.following


def test_manual_plus_during_pending_first_set_accepts_combined_pcm_feedback():
  state = VehicleSpeedLimitCruiseBP()
  drive(state, enabled=False, ego_kph=mph(40), limit_kph=mph(40))
  drive(state, enabled=False, ego_kph=mph(40), limit_kph=mph(40), buttons=[("setCruise", True)])
  drive(state, ego_kph=mph(40), limit_kph=mph(40), buttons=[("setCruise", False)])
  drive(state, ego_kph=mph(40), limit_kph=mph(40), buttons=[("accelCruise", True)])
  drive(state, ego_kph=mph(40), limit_kph=mph(40), buttons=[("accelCruise", False)])
  drive(state, BUTTON_FEEDBACK_FRAMES_BP + 20, ego_kph=mph(40), limit_kph=mph(40))
  assert not state.following
  drive(state, stock_kph=mph(41), ego_kph=mph(40), limit_kph=mph(40))
  assert state.target_kph == pytest.approx(mph(41))
  assert not state.following


def test_new_set_and_plus_adopt_separate_pcm_echoes_after_internal_divergence():
  state = arm_at_70()
  drive(state, LIMIT_SETTLE_FRAMES_BP, limit_kph=mph(50))
  assert state.target_kph == pytest.approx(mph(50))
  drive(state, enabled=False, ego_kph=mph(60), limit_kph=mph(50), buttons=[("setCruise", True)])
  drive(state, ego_kph=mph(60), limit_kph=mph(50), buttons=[("setCruise", False)])
  drive(state, ego_kph=mph(60), limit_kph=mph(50), buttons=[("accelCruise", True)])
  drive(state, ego_kph=mph(60), limit_kph=mph(50), buttons=[("accelCruise", False)])
  assert drive(state, stock_kph=mph(60), ego_kph=mph(60), limit_kph=mph(50)) == pytest.approx(mph(60))
  assert drive(state, stock_kph=mph(61), ego_kph=mph(60), limit_kph=mph(50)) == pytest.approx(mph(61))
  drive(state, BUTTON_FEEDBACK_FRAMES_BP + 1, stock_kph=mph(61), ego_kph=mph(60), limit_kph=mph(50))
  assert state.target_kph == pytest.approx(mph(61))
  assert not state.set_pending
  assert not state.following


def test_pending_set_ignores_minus_echo_from_old_pcm_speed():
  state = arm_at_70()
  drive(state, enabled=False, ego_kph=mph(60), buttons=[("setCruise", True)])
  drive(state, ego_kph=mph(60), buttons=[("setCruise", False)])
  drive(state, ego_kph=mph(60), buttons=[("decelCruise", True)])
  drive(state, ego_kph=mph(60), buttons=[("decelCruise", False)])
  assert drive(state, stock_kph=mph(69), ego_kph=mph(60)) == pytest.approx(mph(60))
  assert state.set_pending
  assert not state.following
  assert drive(state, stock_kph=mph(60), ego_kph=mph(60)) == pytest.approx(mph(60))
  assert not state.set_pending
  assert drive(state, stock_kph=mph(59), ego_kph=mph(60)) == pytest.approx(mph(59))
  assert not state.following


def test_set_to_unchanged_pcm_speed_then_plus_does_not_leave_set_pending():
  state = arm_at_70()
  drive(state, enabled=False, buttons=[("setCruise", True)])
  drive(state, buttons=[("setCruise", False)])
  drive(state, buttons=[("accelCruise", True)])
  drive(state, buttons=[("accelCruise", False)])
  assert drive(state, stock_kph=mph(71)) == pytest.approx(mph(71))
  assert not state.set_pending
  drive(state, BUTTON_FEEDBACK_FRAMES_BP + 1, stock_kph=mph(71))
  assert state.target_kph == pytest.approx(mph(71))
  assert not state.following


def test_metric_manual_increment_uses_pcm_delta_after_limit_following():
  state = VehicleSpeedLimitCruiseBP()
  values = dict(stock_kph=100.0, ego_kph=100.0, limit_kph=100.0)
  drive(state, enabled=False, **values)
  drive(state, enabled=False, buttons=[("setCruise", True)], **values)
  drive(state, buttons=[("setCruise", False)], **values)
  drive(state, LIMIT_SETTLE_FRAMES_BP + BUTTON_FEEDBACK_FRAMES_BP + 1, **values)
  assert state.following
  drive(state, LIMIT_SETTLE_FRAMES_BP, stock_kph=100.0, ego_kph=60.0, limit_kph=60.0)
  assert state.target_kph == pytest.approx(60.0)
  drive(state, stock_kph=100.0, ego_kph=60.0, limit_kph=60.0, buttons=[("accelCruise", True)])
  drive(state, stock_kph=100.0, ego_kph=60.0, limit_kph=60.0, buttons=[("accelCruise", False)])
  assert drive(state, stock_kph=101.0, ego_kph=60.0, limit_kph=60.0) == pytest.approx(61.0)
  assert not state.following
  assert drive(state, BUTTON_FEEDBACK_FRAMES_BP + 1, stock_kph=101.0,
               ego_kph=60.0, limit_kph=60.0) == pytest.approx(61.0)
  assert not state.following


def test_main_off_clears_saved_session():
  state = arm_at_70()
  assert drive(state, available=False, enabled=False) is None
  assert not state.initialized
  assert not state.following
  assert not state.arm_requested
  drive(state, LIMIT_SETTLE_FRAMES_BP + 1, enabled=False, limit_kph=mph(40))
  assert state.target_kph is None


class MemoryParamsBP:
  def __init__(self, values):
    self.values = values

  def get_bool(self, key):
    return bool(self.values.get(key, False))

  def get(self, key, return_default=False):
    return self.values.get(key, 1 if return_default else None)


@pytest.fixture
def make_helper(monkeypatch):
  from openpilot.sunnypilot.selfdrive.car import cruise_ext

  def create(opt_in=True, **changes):
    values = {PARAM_BP: opt_in}
    monkeypatch.setattr(cruise_ext, "Params", lambda: MemoryParamsBP(values))
    cp = car.CarParams.new_message(brand="ford", carFingerprint=str(CAR.FORD_F_150_MK14),
                                  openpilotLongitudinalControl=True, pcmCruise=True, dashcamOnly=False)
    cp_sp = custom.CarParamsSP.new_message(pcmCruiseSpeed=True)
    for name, value in changes.items():
      if name == "pcmCruiseSpeed":
        cp_sp.pcmCruiseSpeed = value
      else:
        setattr(cp, name, value)
    return VCruiseHelperBP(cp, cp_sp)

  return create


def car_state(speed_mph=70, enabled=True, buttons=None):
  return car.CarState.new_message(
    canValid=True, vEgoCluster=speed_mph * CV.MPH_TO_MS,
    cruiseState={"available": True, "enabled": enabled, "speed": speed_mph * CV.MPH_TO_MS,
                 "speedCluster": speed_mph * CV.MPH_TO_MS},
    buttonEvents=buttons or [],
  )


@pytest.mark.parametrize("changes", [
  {"brand": "toyota"}, {"carFingerprint": "FORD MAVERICK 1ST GEN"},
  {"openpilotLongitudinalControl": False}, {"pcmCruise": False},
  {"pcmCruiseSpeed": False}, {"dashcamOnly": True}, {"passive": True},
])
def test_opt_in_is_scoped_to_supported_f150(make_helper, changes):
  assert not make_helper(**changes).follow_vehicle_limits_bp


def test_default_off_uses_unchanged_pcm_set_speed(make_helper):
  helper = make_helper(opt_in=False)
  helper.update_vehicle_speed_limit_bp(40 * CV.MPH_TO_MS)
  cs = car_state()
  helper.update_v_cruise(cs, enabled=True, is_metric=False)
  assert not helper.follow_vehicle_limits_bp
  assert helper.v_cruise_kph == pytest.approx(mph(70))
  assert helper.v_cruise_cluster_kph == pytest.approx(mph(70))


def test_subclass_preserves_followed_target_during_previous_state_initialize(make_helper):
  helper = make_helper()
  helper.update_vehicle_speed_limit_bp(70 * CV.MPH_TO_MS)
  helper.update_v_cruise(car_state(enabled=False), enabled=False, is_metric=False)
  helper.update_v_cruise(car_state(buttons=[{"type": "setCruise", "pressed": True}]),
                         enabled=True, is_metric=False)
  for _ in range(LIMIT_SETTLE_FRAMES_BP + BUTTON_FEEDBACK_FRAMES_BP + 1):
    helper.update_v_cruise(car_state(), enabled=True, is_metric=False)
  helper.update_vehicle_speed_limit_bp(40 * CV.MPH_TO_MS)
  cs = car_state(enabled=False)
  for _ in range(LIMIT_SETTLE_FRAMES_BP):
    helper.update_v_cruise(cs, enabled=False, is_metric=False)
  before = cs.to_dict()
  helper.initialize_v_cruise(cs, experimental_mode=True, dynamic_experimental_control=False)
  assert helper.v_cruise_kph == pytest.approx(mph(40))
  assert helper.v_cruise_cluster_kph == pytest.approx(mph(40))
  assert cs.to_dict() == before
  assert helper.CP.pcmCruise
  assert helper.CP_SP.pcmCruiseSpeed
