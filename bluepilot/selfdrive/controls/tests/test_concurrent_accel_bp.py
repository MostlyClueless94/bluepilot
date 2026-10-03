import math
import pytest

from cereal import car, log
from opendbc.car import get_safety_config
from opendbc.car.ford.values import CAR, FordFlags
from opendbc.car.structs import CarParamsSP
from opendbc.sunnypilot.car.ford.concurrent_accel_bp import (
  configure_concurrent_accel_bp, concurrent_accel_configured_bp, concurrent_accel_enabled_bp,
  constrain_concurrent_accel_bp, longitudinal_override_bp,
)
from opendbc.sunnypilot.car.ford.longitudinal_ext import LongitudinalResult
from openpilot.selfdrive.controls.lib.longcontrol import LongControl


class TestConcurrentAccelBP:
  def setup_method(self):
    self.CP = car.CarParams.new_message(brand="ford", carFingerprint=str(CAR.FORD_F_150_MK14),
                                      openpilotLongitudinalControl=True, flags=int(FordFlags.CANFD))
    self.CP.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.ford, 3)]
    self.CP.longitudinalTuning.kpBP = [0.]
    self.CP.longitudinalTuning.kpV = [0.]
    self.CP.longitudinalTuning.kiBP = [0.]
    self.CP.longitudinalTuning.kiV = [.5]
    self.CP.vEgoStarting = .5
    self.CP.stopAccel = -.5
    self.CP.stoppingDecelRate = .8
    self.CS = car.CarState.new_message(vEgo=20., aEgo=2.0, gasPressed=True)

  def _result(self, gas=1.0, accel=1.0, brake=False, precharge=False, stopping=False):
    return LongitudinalResult(accel, gas, brake, precharge, -5.0, stopping, 100.0, False)

  def test_configuration_defaults_and_scope(self):
    for requested, disengage, expected in ((False, False, False), (True, True, False), (True, False, True)):
      configure_concurrent_accel_bp(self.CP, requested, disengage)
      assert concurrent_accel_configured_bp(self.CP) == expected
    for field, value in (("brand", "toyota"), ("carFingerprint", str(CAR.FORD_MUSTANG_MACH_E_MK1)),
                         ("openpilotLongitudinalControl", False), ("dashcamOnly", True), ("flags", 0)):
      CP = self.CP.as_reader().as_builder()
      setattr(CP, field, value)
      configure_concurrent_accel_bp(CP, True, False)
      assert not concurrent_accel_configured_bp(CP)
    for flags in (0, 1, 2, 4, 5, 6):
      self.CP.safetyConfigs[0].safetyParam = flags
      assert not concurrent_accel_configured_bp(self.CP)
    self.CP.safetyConfigs = []
    configure_concurrent_accel_bp(self.CP, True, False)
    assert not concurrent_accel_configured_bp(self.CP)
    self.CP.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.noOutput, 7)]
    configure_concurrent_accel_bp(self.CP, True, False)
    assert not concurrent_accel_configured_bp(self.CP)

  def test_only_gas_override_is_ignored(self):
    gas = log.OnroadEvent.new_message(name="gasPressedOverride", overrideLongitudinal=True)
    other = log.OnroadEvent.new_message(name="steerOverride", overrideLongitudinal=True)
    assert longitudinal_override_bp([gas], False)
    assert not longitudinal_override_bp([gas], True)
    assert longitudinal_override_bp([gas, other], True)

  def test_experimental_mode_is_required_at_runtime(self):
    configure_concurrent_accel_bp(self.CP, True, False)
    assert concurrent_accel_enabled_bp(self.CP, True)
    assert not concurrent_accel_enabled_bp(self.CP, False)
    configure_concurrent_accel_bp(self.CP, False, False)
    assert not concurrent_accel_enabled_bp(self.CP, True)

  def test_resume_while_pressed_and_first_release_update(self):
    controller = LongControl(self.CP, CarParamsSP())
    # Driver acceleration greater than planned acceleration for five seconds.
    for _ in range(500):
      output = controller.update(True, self.CS, .8, False, (-3.5, 2.), concurrent_accel=True)
      assert output == pytest.approx(.8)
      assert controller.pid.i == pytest.approx(0.)
      assert controller.long_control_state == car.CarControl.Actuators.LongControlState.pid
    self.CS.gasPressed = False
    self.CS.aEgo = .8
    assert controller.update(True, self.CS, .8, False, (-3.5, 2.), concurrent_accel=True) == pytest.approx(.8)

  def test_default_pid_and_disengagement_still_apply(self):
    controller = LongControl(self.CP, CarParamsSP())
    controller.update(True, self.CS, .8, False, (-3.5, 2.))
    assert controller.pid.i < 0.
    assert controller.update(False, self.CS, .8, False, (-3.5, 2.), concurrent_accel=True) == 0.
    assert controller.pid.i == 0.

  def test_frozen_integrator_cannot_accelerate_against_deceleration_plan(self):
    controller = LongControl(self.CP, CarParamsSP())
    controller.pid.i = 1.
    output = controller.update(True, self.CS, -.1, False, (-3.5, 2.), concurrent_accel=True)
    assert output <= 0.

  def test_overlap_neutralizes_brake_and_invalid_propulsion(self):
    for gas in (-5., -.1, math.nan, math.inf, -math.inf):
      result = constrain_concurrent_accel_bp(self._result(gas, -2., True, True, True), True, True, 1.)
      assert result.gas == -5.
      assert result.accel == 0.
      assert not result.brake_actuate or result.precharge_actuate or result.stopping
      assert result.accel_pred_send == -5.

  def test_positive_request_ramps_across_release_and_reductions_are_immediate(self):
    gas = -5.
    for pressed in [True] * 10 + [False] * 10:
      result = constrain_concurrent_accel_bp(self._result(2.), pressed, True, gas)
      assert result.gas - max(gas, 0.) == pytest.approx(.02)
      gas = result.gas
    reduced = constrain_concurrent_accel_bp(self._result(.1), False, True, gas)
    assert reduced.gas == .1
    for pressed in (False, True):
      inactive = constrain_concurrent_accel_bp(self._result(2.), pressed, False, gas)
      assert inactive.gas == -5.
      assert inactive.accel == 0.

  def test_braking_after_release_suppresses_propulsion(self):
    for brake, precharge in ((True, False), (False, True)):
      result = constrain_concurrent_accel_bp(self._result(1., -1., brake, precharge), False, True, 1.)
      assert result.gas == -5.
      assert result.accel == -1.
    result = constrain_concurrent_accel_bp(self._result(-.3, -.3), False, True, 1.)
    assert result.gas == -.3
