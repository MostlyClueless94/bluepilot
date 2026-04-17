import inspect
import unittest
from types import SimpleNamespace

from openpilot.common.params import Params
from opendbc.car.lateral import apply_std_steer_angle_limits
from opendbc.car import structs
from opendbc.car.subaru.fingerprints import FW_VERSIONS
from opendbc.car.subaru import subarucan
from opendbc.car.subaru.carcontroller import (
  ANGLE_DRIVER_OVERRIDE_HOLD_FRAMES,
  ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_DEFAULT,
  ANGLE_DRIVER_OVERRIDE_RAMP_FRAMES,
  ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_EXPONENTS,
  ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_CONFIRM_FRAME_OPTIONS,
  ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_RATE_THRESHOLDS,
  ANGLE_DRIVER_OVERRIDE_SOFT_HOLD_FRAMES,
  ANGLE_DRIVER_OVERRIDE_SOFT_HOLD_LEVEL_MAX,
  ANGLE_DRIVER_OVERRIDE_SOFT_HOLD_LEVEL_MIN,
  CarController,
  MAX_STEERING_LOW_SPEED_GUARD_ENTER_SPEED,
  MAX_STEERING_LOW_SPEED_GUARD_EXIT_SPEED,
  MAX_STEERING_LOW_SPEED_GUARD_QUIET_FRAMES,
  MADS_ONLY_FAULT_GUARD_HIGH_SPEED,
  MADS_ONLY_FAULT_GUARD_LOW_SPEED,
  MADS_ONLY_FAULT_GUARD_LOW_SPEED_CAP,
  MADS_ONLY_FAULT_GUARD_QUIET_FRAMES,
  MADS_ONLY_FAULT_GUARD_QUIET_RATE,
  MADS_ONLY_FAULT_GUARD_REENABLE_MARGIN,
  MADS_ONLY_FAULT_GUARD_REENABLE_UNWIND_FRAMES,
  MADS_ONLY_FAULT_GUARD_REENABLE_UNWIND_STEP,
  MADS_ONLY_FAULT_GUARD_RATE_THRESHOLD,
  MADS_ONLY_MAX_STEER_ANGLE,
  MADS_ONLY_MAX_STEER_ANGLE_MAX,
  MADS_ONLY_MIN_SPEED,
  SOFT_CAPTURE_LEVEL_PARAMS,
  SUBARU_TURN_IN_RATE_LEVEL_MAX,
  SUBARU_TURN_IN_RATE_LEVEL_MIN,
  SUBARU_TURN_IN_RATE_LEVEL_VALUES,
  SUBARU_UNWIND_RATE_LEVEL_MAX,
  SUBARU_UNWIND_RATE_LEVEL_MIN,
  SUBARU_UNWIND_RATE_LEVEL_VALUES,
)
from opendbc.car.subaru.carstate import (
  CarState,
  MANUAL_YIELD_TORQUE_THRESHOLD_DEFAULT,
  MANUAL_YIELD_TORQUE_THRESHOLD_MAX,
  MANUAL_YIELD_TORQUE_THRESHOLD_MIN,
  MANUAL_YIELD_TORQUE_THRESHOLD_VALUES,
)
from opendbc.car.subaru.interface import CarInterface
from opendbc.car.subaru.values import CAR
from opendbc.car.tests.routes import routes


class TestSubaruCarController(unittest.TestCase):
  PARAM_KEYS = (
    "MCSubaruManualYieldResumeSoftnessEnabled",
    "MCSubaruManualYieldResumeSoftness",
    "MCSubaruManualYieldReleaseGuardEnabled",
    "MCSubaruManualYieldReleaseGuardLevel",
    "MCSubaruManualYieldTorqueThresholdEnabled",
    "MCSubaruManualYieldTorqueThreshold",
    "MCSubaruManualYieldFilteredDetectionEnabled",
    "MCSubaruManualSteeringSoftHoldLevel",
    "MCSubaruShowAdvancedDevControls",
    "MCSubaruMaxSteeringExperiment",
    "MCSubaruSoftCaptureEnabled",
    "MCSubaruSoftCaptureLevel",
    "MCSubaruMadsTighterTurnsEnabled",
    "MCSubaruMadsMaxSteeringAngle",
    "MCSubaruUnwindRateLevel",
    "MCSubaruTurnInRateLevel",
  )

  def setUp(self):
    self.params = Params()
    for key in self.PARAM_KEYS:
      self.params.remove(key)

  def tearDown(self):
    for key in self.PARAM_KEYS:
      self.params.remove(key)

  @staticmethod
  def _build_cs(v_ego_raw, steering_angle_deg, steering_pressed=False, standstill=False,
                steering_rate_deg=0.0, steering_torque=0.0):
    return SimpleNamespace(out=SimpleNamespace(
      vEgoRaw=v_ego_raw,
      steeringAngleDeg=steering_angle_deg,
      steeringRateDeg=steering_rate_deg,
      steeringTorque=steering_torque,
      gearShifter=structs.CarState.GearShifter.drive,
      standstill=standstill,
      steeringPressed=steering_pressed,
    ))

  @staticmethod
  def _build_cc(lat_active, enabled, steering_angle_deg):
    return SimpleNamespace(
      latActive=lat_active,
      enabled=enabled,
      actuators=SimpleNamespace(steeringAngleDeg=steering_angle_deg),
    )

  def _build_controller(self, *, soft_capture_enabled=False, soft_capture_level=3,
                        resume_softness_enabled=False, resume_softness_setting=None,
                        release_guard_enabled=False, release_guard_level=2,
                        mads_tighter_turns_enabled=False, mads_max_steering_angle=180,
                        unwind_rate_level=0, turn_in_rate_level=0,
                        soft_hold_level=0, torque_threshold_enabled=False, torque_threshold=80,
                        max_steering_experiment=False):
    self.params.put_bool("MCSubaruSoftCaptureEnabled", soft_capture_enabled)
    self.params.put("MCSubaruSoftCaptureLevel", str(soft_capture_level))
    self.params.put_bool("MCSubaruManualYieldResumeSoftnessEnabled", resume_softness_enabled)
    self.params.put_bool("MCSubaruManualYieldReleaseGuardEnabled", release_guard_enabled)
    self.params.put("MCSubaruManualYieldReleaseGuardLevel", str(release_guard_level))
    self.params.put("MCSubaruManualSteeringSoftHoldLevel", str(soft_hold_level))
    self.params.put_bool("MCSubaruManualYieldTorqueThresholdEnabled", torque_threshold_enabled)
    self.params.put("MCSubaruManualYieldTorqueThreshold", str(torque_threshold))
    self.params.put_bool("MCSubaruMadsTighterTurnsEnabled", mads_tighter_turns_enabled)
    self.params.put("MCSubaruMadsMaxSteeringAngle", str(mads_max_steering_angle))
    self.params.put("MCSubaruUnwindRateLevel", str(unwind_rate_level))
    self.params.put("MCSubaruTurnInRateLevel", str(turn_in_rate_level))
    self.params.put_bool("MCSubaruMaxSteeringExperiment", max_steering_experiment)
    if resume_softness_setting is not None:
      self.params.put("MCSubaruManualYieldResumeSoftness", str(resume_softness_setting))
    CP = CarInterface.get_non_essential_params(CAR.SUBARU_OUTBACK_2023)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.SUBARU_OUTBACK_2023)
    return CarController({}, CP, CP_SP)

  def _build_carstate(self, *, torque_threshold_enabled=False, torque_threshold=80):
    self.params.put_bool("MCSubaruManualYieldTorqueThresholdEnabled", torque_threshold_enabled)
    self.params.put("MCSubaruManualYieldTorqueThreshold", str(torque_threshold))
    CP = CarInterface.get_non_essential_params(CAR.SUBARU_OUTBACK_2023)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.SUBARU_OUTBACK_2023)
    return CarState(CP, CP_SP)

  @staticmethod
  def _set_resume_profile(controller, softness_setting=4):
    controller.mc_subaru_manual_yield_resume_softness = softness_setting

  @staticmethod
  def _build_release_guard_cs(v_ego_raw, steering_angle_deg=10.0, steering_rate_deg=0.0, steering_pressed=False):
    return TestSubaruCarController._build_cs(
      v_ego_raw,
      steering_angle_deg,
      steering_pressed=steering_pressed,
      steering_rate_deg=steering_rate_deg,
    )

  def _prime_angle_driver_override_ramp(self, controller, cc, v_ego_raw=8.0, measured_angle=10.0,
                                        softness_setting=4, use_current_profile=False):
    if not use_current_profile:
      controller.mc_subaru_manual_yield_resume_softness_enabled = True
      self._set_resume_profile(controller, softness_setting)
    controller.apply_angle_last = measured_angle

    controller.handle_angle_lateral(cc, self._build_cs(v_ego_raw, measured_angle, steering_pressed=True))
    released_cs = self._build_cs(v_ego_raw, measured_angle, steering_pressed=False)
    for _ in range(ANGLE_DRIVER_OVERRIDE_HOLD_FRAMES):
      controller.handle_angle_lateral(cc, released_cs)

    expected_softness_setting = controller.mc_subaru_manual_yield_resume_softness
    self.assertEqual(controller.angle_driver_override_hold_frames, 0)
    self.assertEqual(controller.angle_driver_override_ramp_frames, ANGLE_DRIVER_OVERRIDE_RAMP_FRAMES)
    self.assertEqual(controller.angle_driver_override_ramp_total_frames, ANGLE_DRIVER_OVERRIDE_RAMP_FRAMES)
    self.assertAlmostEqual(controller.angle_driver_override_ramp_start_angle, measured_angle)
    self.assertAlmostEqual(controller.angle_driver_override_ramp_softness_exponent, ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_EXPONENTS[expected_softness_setting])
    return released_cs

  def _prime_angle_driver_override_release_guard(self, controller, cc, *, v_ego_raw=8.0, measured_angle=10.0,
                                                 steering_rate_deg=0.0, softness_setting=4,
                                                 use_current_profile=False):
    if not use_current_profile:
      controller.mc_subaru_manual_yield_resume_softness_enabled = True
      self._set_resume_profile(controller, softness_setting)
    controller.apply_angle_last = measured_angle

    controller.handle_angle_lateral(cc, self._build_cs(v_ego_raw, measured_angle, steering_pressed=True))
    released_cs = self._build_release_guard_cs(v_ego_raw, measured_angle, steering_rate_deg=steering_rate_deg)
    for _ in range(ANGLE_DRIVER_OVERRIDE_HOLD_FRAMES):
      controller.handle_angle_lateral(cc, released_cs)

    self.assertEqual(controller.angle_driver_override_hold_frames, 0)
    self.assertTrue(controller.angle_driver_override_release_guard_pending)
    self.assertEqual(controller.angle_driver_override_ramp_frames, 0)
    return released_cs

  def test_manual_yield_full_release_in_mads_only_when_handoff_off(self):
    controller = self._build_controller()
    expected_controller = self._build_controller()
    cs = self._build_cs(9.5, 20.56, steering_pressed=True)
    cc = self._build_cc(True, False, 19.86)

    controller.apply_angle_last = cs.out.steeringAngleDeg

    msg = controller.handle_angle_lateral(cc, cs)
    inhibited = subarucan.create_steering_control_angle(expected_controller.packer, cs.out.steeringAngleDeg, False)

    self.assertEqual(msg, inhibited)
    self.assertTrue(controller.subaru_manual_yield_full_release_active)
    self.assertFalse(controller.subaru_effective_lkas_active)
    self.assertAlmostEqual(controller.apply_angle_last, cs.out.steeringAngleDeg)
    self.assertEqual(controller.angle_driver_override_hold_frames, 0)
    self.assertEqual(controller.angle_driver_override_ramp_frames, 0)

  def test_manual_yield_full_release_in_full_engaged_when_handoff_off(self):
    controller = self._build_controller()
    expected_controller = self._build_controller()
    cs = self._build_cs(9.5, 20.56, steering_pressed=True)
    cc = self._build_cc(True, True, 19.86)

    controller.apply_angle_last = cs.out.steeringAngleDeg

    msg = controller.handle_angle_lateral(cc, cs)
    inhibited = subarucan.create_steering_control_angle(expected_controller.packer, cs.out.steeringAngleDeg, False)

    self.assertEqual(msg, inhibited)
    self.assertTrue(controller.subaru_manual_yield_full_release_active)
    self.assertFalse(controller.subaru_effective_lkas_active)
    self.assertAlmostEqual(controller.apply_angle_last, cs.out.steeringAngleDeg)
    self.assertEqual(controller.angle_driver_override_hold_frames, 0)
    self.assertEqual(controller.angle_driver_override_ramp_frames, 0)

  def test_angle_driver_override_hold_does_not_persist_in_mads_only_when_handoff_off(self):
    controller = self._build_controller()
    cs_pressed = self._build_cs(8.0, 10.0, steering_pressed=True)
    cc = self._build_cc(True, False, 14.0)

    controller.apply_angle_last = cs_pressed.out.steeringAngleDeg
    controller.handle_angle_lateral(cc, cs_pressed)

    cs_released = self._build_cs(8.0, 10.0, steering_pressed=False)
    msg = controller.handle_angle_lateral(cc, cs_released)
    inhibited = subarucan.create_steering_control_angle(controller.packer, cs_released.out.steeringAngleDeg, False)

    self.assertNotEqual(msg, inhibited)
    self.assertEqual(controller.angle_driver_override_hold_frames, 0)
    self.assertEqual(controller.angle_driver_override_ramp_frames, 0)
    self.assertGreater(controller.apply_angle_last, cs_released.out.steeringAngleDeg)

  def test_angle_driver_override_hold_does_not_persist_in_full_engaged_when_handoff_off(self):
    controller = self._build_controller()
    cs_pressed = self._build_cs(8.0, 10.0, steering_pressed=True)
    cc = self._build_cc(True, True, 14.0)

    controller.apply_angle_last = cs_pressed.out.steeringAngleDeg
    controller.handle_angle_lateral(cc, cs_pressed)

    cs_released = self._build_cs(8.0, 10.0, steering_pressed=False)
    msg = controller.handle_angle_lateral(cc, cs_released)
    inhibited = subarucan.create_steering_control_angle(controller.packer, cs_released.out.steeringAngleDeg, False)

    self.assertNotEqual(msg, inhibited)
    self.assertEqual(controller.angle_driver_override_hold_frames, 0)
    self.assertEqual(controller.angle_driver_override_ramp_frames, 0)
    self.assertGreater(controller.apply_angle_last, cs_released.out.steeringAngleDeg)

  def test_angle_driver_override_default_profile_leaves_custom_handoff_off(self):
    controller = self._build_controller()
    cc = self._build_cc(True, True, 14.0)
    cs = self._build_cs(8.0, 10.0, steering_pressed=True)

    controller.apply_angle_last = cs.out.steeringAngleDeg
    controller.handle_angle_lateral(cc, cs)

    self.assertFalse(controller._manual_yield_handoff_enabled())
    self.assertTrue(controller.subaru_manual_yield_full_release_active)
    self.assertFalse(controller.subaru_effective_lkas_active)
    self.assertEqual(controller.mc_subaru_manual_yield_resume_softness, ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_DEFAULT)
    self.assertEqual(controller.angle_driver_override_hold_frames, 0)
    self.assertEqual(controller.angle_driver_override_ramp_frames, 0)

  def test_angle_driver_override_resume_softness_profiles_map_to_expected_exponents(self):
    expected_exponents = {
      0: 1.0,
      1: 1.25,
      2: 1.5,
      3: 2.0,
      4: 2.5,
      5: 3.0,
      6: 3.5,
    }

    for softness_setting, expected_exponent in expected_exponents.items():
      controller = self._build_controller()
      cc = self._build_cc(True, True, 14.0)

      self._prime_angle_driver_override_ramp(controller, cc, softness_setting=softness_setting)

      self.assertAlmostEqual(controller.angle_driver_override_ramp_softness_exponent, expected_exponent)

  def test_angle_driver_override_resume_softness_toggle_off_disables_reclaim_ramp(self):
    controller = self._build_controller(
      resume_softness_enabled=False,
      resume_softness_setting=6,
    )
    cc = self._build_cc(True, True, 14.0)

    self.assertEqual(controller.mc_subaru_manual_yield_resume_softness, 4)

    cs_pressed = self._build_cs(8.0, 10.0, steering_pressed=True)
    controller.apply_angle_last = cs_pressed.out.steeringAngleDeg
    controller.handle_angle_lateral(cc, cs_pressed)

    cs_released = self._build_cs(8.0, 10.0, steering_pressed=False)
    controller.handle_angle_lateral(cc, cs_released)

    self.assertEqual(controller.angle_driver_override_hold_frames, 0)
    self.assertEqual(controller.angle_driver_override_ramp_frames, 0)
    self.assertGreater(controller.apply_angle_last, cs_released.out.steeringAngleDeg)
    self.assertLessEqual(controller.apply_angle_last, cc.actuators.steeringAngleDeg)

  def test_angle_driver_override_resume_softness_reenable_restores_saved_custom_exponent(self):
    disabled = self._build_controller(
      resume_softness_enabled=False,
      resume_softness_setting=6,
    )
    reenabled = self._build_controller(
      resume_softness_enabled=True,
      resume_softness_setting=6,
    )

    self.assertEqual(disabled.mc_subaru_manual_yield_resume_softness, ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_DEFAULT)
    self.assertEqual(reenabled.mc_subaru_manual_yield_resume_softness, 6)

  def test_angle_driver_override_release_guard_off_preserves_current_reclaim_timing(self):
    controller = self._build_controller(release_guard_enabled=False, release_guard_level=3)
    cc = self._build_cc(True, True, 14.0)

    self._prime_angle_driver_override_ramp(controller, cc)

    self.assertFalse(controller.angle_driver_override_release_guard_pending)
    self.assertEqual(controller.angle_driver_override_ramp_frames, ANGLE_DRIVER_OVERRIDE_RAMP_FRAMES)

  def test_angle_driver_override_release_guard_blocks_immediate_reclaim_after_hold_expiry(self):
    controller = self._build_controller(release_guard_enabled=True, release_guard_level=2)
    cc = self._build_cc(True, True, 14.0)
    released_cs = self._prime_angle_driver_override_release_guard(controller, cc)

    required_frames = ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_CONFIRM_FRAME_OPTIONS[1]
    self.assertEqual(controller.angle_driver_override_release_guard_required_frames, required_frames)
    self.assertEqual(controller.angle_driver_override_release_guard_rate_threshold, ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_RATE_THRESHOLDS[1])

    for expected_frames in range(1, required_frames):
      controller.handle_angle_lateral(cc, released_cs)
      self.assertTrue(controller.angle_driver_override_release_guard_pending)
      self.assertEqual(controller.angle_driver_override_release_guard_confirm_frames, expected_frames)
      self.assertEqual(controller.angle_driver_override_ramp_frames, 0)

  def test_angle_driver_override_release_guard_starts_ramp_after_quiet_confirmation(self):
    controller = self._build_controller(release_guard_enabled=True, release_guard_level=2)
    cc = self._build_cc(True, True, 14.0)
    released_cs = self._prime_angle_driver_override_release_guard(controller, cc)

    for _ in range(ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_CONFIRM_FRAME_OPTIONS[1]):
      controller.handle_angle_lateral(cc, released_cs)

    self.assertFalse(controller.angle_driver_override_release_guard_pending)
    self.assertEqual(controller.angle_driver_override_ramp_frames, ANGLE_DRIVER_OVERRIDE_RAMP_FRAMES)
    self.assertEqual(controller.angle_driver_override_ramp_total_frames, ANGLE_DRIVER_OVERRIDE_RAMP_FRAMES)
    self.assertAlmostEqual(controller.angle_driver_override_ramp_start_angle, released_cs.out.steeringAngleDeg)

  def test_angle_driver_override_release_guard_without_resume_softness_does_not_start_ramp(self):
    controller = self._build_controller(
      release_guard_enabled=True,
      release_guard_level=2,
      resume_softness_enabled=False,
      resume_softness_setting=6,
    )
    cc = self._build_cc(True, True, 14.0)
    released_cs = self._prime_angle_driver_override_release_guard(controller, cc, use_current_profile=True)

    self.assertEqual(controller.mc_subaru_manual_yield_resume_softness, ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_DEFAULT)

    for _ in range(ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_CONFIRM_FRAME_OPTIONS[1]):
      controller.handle_angle_lateral(cc, released_cs)

    self.assertFalse(controller.angle_driver_override_release_guard_pending)
    self.assertEqual(controller.angle_driver_override_ramp_frames, 0)
    self.assertEqual(controller.angle_driver_override_ramp_total_frames, ANGLE_DRIVER_OVERRIDE_RAMP_FRAMES)

  def test_angle_driver_override_full_release_reports_lkas_inactive_with_release_guard_enabled(self):
    controller = self._build_controller(release_guard_enabled=True)
    cc = self._build_cc(True, False, 14.0)
    cs = self._build_cs(8.0, 10.0, steering_pressed=True)
    controller.apply_angle_last = cs.out.steeringAngleDeg

    msg = controller.handle_angle_lateral(cc, cs)
    expected = subarucan.create_steering_control_angle(controller.packer, cs.out.steeringAngleDeg, False)

    self.assertEqual(msg, expected)
    self.assertTrue(controller.subaru_manual_yield_full_release_active)
    self.assertFalse(controller.subaru_effective_lkas_active)
    self.assertTrue(cc.latActive)

  def test_legacy_full_release_param_off_is_ignored(self):
    self.params.put_bool("MCSubaruManualYieldFullReleaseEnabled", False)
    controller = self._build_controller(release_guard_enabled=True)
    cc = self._build_cc(True, False, 14.0)
    cs = self._build_cs(8.0, 10.0, steering_pressed=True)
    controller.apply_angle_last = cs.out.steeringAngleDeg

    msg = controller.handle_angle_lateral(cc, cs)
    expected = subarucan.create_steering_control_angle(controller.packer, cs.out.steeringAngleDeg, False)

    self.assertEqual(msg, expected)
    self.assertTrue(controller.subaru_manual_yield_full_release_active)
    self.assertFalse(controller.subaru_effective_lkas_active)

  def test_angle_driver_override_full_release_exit_can_start_soft_capture_recapture(self):
    controller = self._build_controller(
      release_guard_enabled=True,
      resume_softness_enabled=False,
      soft_capture_enabled=True,
      soft_capture_level=2,
    )
    cc = self._build_cc(True, False, 14.0)
    released_cs = self._prime_angle_driver_override_release_guard(controller, cc, use_current_profile=True)

    self.assertTrue(controller.subaru_manual_yield_full_release_active)
    self.assertFalse(controller.subaru_effective_lkas_active)

    for _ in range(ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_CONFIRM_FRAME_OPTIONS[1]):
      controller.handle_angle_lateral(cc, released_cs)

    self.assertFalse(controller.subaru_manual_yield_full_release_active)
    self.assertTrue(controller.subaru_effective_lkas_active)
    self.assertEqual(controller.soft_capture_frame, controller.frame)

  def test_angle_driver_override_release_guard_cancels_when_driver_input_returns(self):
    controller = self._build_controller(release_guard_enabled=True, release_guard_level=2)
    cc = self._build_cc(True, True, 14.0)
    released_cs = self._prime_angle_driver_override_release_guard(controller, cc)

    controller.handle_angle_lateral(cc, released_cs)
    self.assertTrue(controller.angle_driver_override_release_guard_pending)

    pressed_cs = self._build_release_guard_cs(8.0, steering_angle_deg=10.0, steering_rate_deg=2.5, steering_pressed=True)
    msg = controller.handle_angle_lateral(cc, pressed_cs)
    expected = subarucan.create_steering_control_angle(controller.packer, pressed_cs.out.steeringAngleDeg, False)

    self.assertEqual(msg, expected)
    self.assertFalse(controller.angle_driver_override_release_guard_pending)
    self.assertEqual(controller.angle_driver_override_hold_frames, ANGLE_DRIVER_OVERRIDE_HOLD_FRAMES)
    self.assertEqual(controller.angle_driver_override_ramp_frames, 0)

  def test_angle_driver_override_release_guard_levels_change_confirmation_strictness(self):
    light = self._build_controller(release_guard_enabled=True, release_guard_level=1)
    strong = self._build_controller(release_guard_enabled=True, release_guard_level=3)
    cc = self._build_cc(True, True, 14.0)
    light_released_cs = self._prime_angle_driver_override_release_guard(light, cc, steering_rate_deg=2.5)
    strong_released_cs = self._prime_angle_driver_override_release_guard(strong, cc, steering_rate_deg=2.5)

    for _ in range(ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_CONFIRM_FRAME_OPTIONS[0]):
      light.handle_angle_lateral(cc, light_released_cs)
      strong.handle_angle_lateral(cc, strong_released_cs)

    self.assertFalse(light.angle_driver_override_release_guard_pending)
    self.assertEqual(light.angle_driver_override_ramp_frames, ANGLE_DRIVER_OVERRIDE_RAMP_FRAMES)
    self.assertTrue(strong.angle_driver_override_release_guard_pending)
    self.assertEqual(strong.angle_driver_override_release_guard_confirm_frames, 0)
    self.assertEqual(strong.angle_driver_override_ramp_frames, 0)

  def test_angle_driver_override_release_guard_reenable_restores_saved_strength(self):
    disabled = self._build_controller(release_guard_enabled=False, release_guard_level=3)
    enabled = self._build_controller(release_guard_enabled=True, release_guard_level=3)
    cc = self._build_cc(True, True, 14.0)

    self.assertFalse(disabled.mc_subaru_manual_yield_release_guard_enabled)
    self.assertEqual(disabled.mc_subaru_manual_yield_release_guard_level, 3)

    released_cs = self._prime_angle_driver_override_release_guard(enabled, cc)

    self.assertEqual(
      enabled.angle_driver_override_release_guard_required_frames,
      ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_CONFIRM_FRAME_OPTIONS[2],
    )
    self.assertEqual(
      enabled.angle_driver_override_release_guard_rate_threshold,
      ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_RATE_THRESHOLDS[2],
    )
    self.assertTrue(enabled.angle_driver_override_release_guard_pending)
    self.assertAlmostEqual(released_cs.out.steeringAngleDeg, 10.0)

  def test_angle_driver_override_release_guard_preserves_fixed_resume_timing_and_custom_softness_after_confirmation(self):
    controller = self._build_controller(
      release_guard_enabled=True,
      release_guard_level=2,
      resume_softness_enabled=True,
      resume_softness_setting=6,
    )
    cc = self._build_cc(True, True, 14.0)
    released_cs = self._prime_angle_driver_override_release_guard(controller, cc, use_current_profile=True)

    for _ in range(ANGLE_DRIVER_OVERRIDE_RELEASE_GUARD_CONFIRM_FRAME_OPTIONS[1]):
      controller.handle_angle_lateral(cc, released_cs)

    self.assertEqual(controller.angle_driver_override_ramp_frames, ANGLE_DRIVER_OVERRIDE_RAMP_FRAMES)
    self.assertEqual(controller.angle_driver_override_ramp_total_frames, ANGLE_DRIVER_OVERRIDE_RAMP_FRAMES)
    self.assertAlmostEqual(controller.angle_driver_override_ramp_softness_exponent, ANGLE_DRIVER_OVERRIDE_RAMP_SOFTNESS_EXPONENTS[6])

  def test_manual_steering_soft_hold_level_zero_preserves_current_behavior(self):
    controller = self._build_controller(soft_hold_level=0)
    cc = self._build_cc(True, True, 14.0)
    cs_pressed = self._build_cs(8.0, 10.0, steering_pressed=True, steering_torque=100.0)
    controller.apply_angle_last = cs_pressed.out.steeringAngleDeg

    controller.handle_angle_lateral(cc, cs_pressed)

    self.assertEqual(controller.angle_driver_override_soft_hold_frames, 0)
    self.assertEqual(controller.angle_driver_override_soft_hold_threshold, 0)

    cs_released = self._build_cs(8.0, 10.0, steering_pressed=False, steering_torque=50.0)
    msg = controller.handle_angle_lateral(cc, cs_released)
    inhibited = subarucan.create_steering_control_angle(controller.packer, cs_released.out.steeringAngleDeg, False)

    self.assertNotEqual(msg, inhibited)
    self.assertFalse(controller.subaru_manual_yield_full_release_active)
    self.assertGreater(controller.apply_angle_last, cs_released.out.steeringAngleDeg)

  def test_manual_steering_soft_hold_levels_map_to_scaled_thresholds(self):
    expected = {
      1: (60, 25),
      2: (50, 50),
      3: (40, 75),
    }

    for level, (expected_threshold, expected_frames) in expected.items():
      with self.subTest(level=level):
        controller = self._build_controller(soft_hold_level=level)

        self.assertEqual(controller.mc_subaru_manual_steering_soft_hold_level, level)
        self.assertEqual(controller._get_soft_hold_threshold(80, level), expected_threshold)
        self.assertEqual(ANGLE_DRIVER_OVERRIDE_SOFT_HOLD_FRAMES[level], expected_frames)

  def test_manual_steering_soft_hold_clamps_saved_level_and_threshold_floor(self):
    below = self._build_controller(soft_hold_level=-3)
    above = self._build_controller(soft_hold_level=99)

    self.assertEqual(below.mc_subaru_manual_steering_soft_hold_level, ANGLE_DRIVER_OVERRIDE_SOFT_HOLD_LEVEL_MIN)
    self.assertEqual(above.mc_subaru_manual_steering_soft_hold_level, ANGLE_DRIVER_OVERRIDE_SOFT_HOLD_LEVEL_MAX)
    self.assertEqual(CarController._get_soft_hold_threshold(45, 3), MANUAL_YIELD_TORQUE_THRESHOLD_MIN)

  def test_manual_steering_soft_hold_scales_from_custom_manual_yield_threshold(self):
    controller = self._build_controller(
      soft_hold_level=2,
      torque_threshold_enabled=True,
      torque_threshold=120,
    )

    self.assertEqual(controller._get_active_manual_yield_torque_threshold(), 120)
    self.assertEqual(controller._get_soft_hold_threshold(120, 1), 90)
    self.assertEqual(controller._get_soft_hold_threshold(120, 2), 70)
    self.assertEqual(controller._get_soft_hold_threshold(120, 3), 60)

  def test_manual_steering_soft_hold_does_not_make_initial_yield_easier(self):
    controller = self._build_controller(soft_hold_level=2)
    cc = self._build_cc(True, True, 14.0)
    cs = self._build_cs(8.0, 10.0, steering_pressed=False, steering_torque=50.0)
    controller.apply_angle_last = cs.out.steeringAngleDeg

    msg = controller.handle_angle_lateral(cc, cs)
    inhibited = subarucan.create_steering_control_angle(controller.packer, cs.out.steeringAngleDeg, False)

    self.assertNotEqual(msg, inhibited)
    self.assertFalse(controller.subaru_manual_yield_full_release_active)
    self.assertEqual(controller.angle_driver_override_soft_hold_frames, 0)
    self.assertGreater(controller.apply_angle_last, cs.out.steeringAngleDeg)

  def test_manual_steering_soft_hold_accepts_light_pressure_after_real_press(self):
    controller = self._build_controller(soft_hold_level=2)
    cc = self._build_cc(True, True, 14.0)
    cs_pressed = self._build_cs(8.0, 10.0, steering_pressed=True, steering_torque=100.0)
    controller.apply_angle_last = cs_pressed.out.steeringAngleDeg

    controller.handle_angle_lateral(cc, cs_pressed)
    self.assertEqual(controller.angle_driver_override_soft_hold_frames, ANGLE_DRIVER_OVERRIDE_SOFT_HOLD_FRAMES[2])
    self.assertEqual(controller.angle_driver_override_soft_hold_threshold, 50)

    cs_soft_pressure = self._build_cs(8.0, 10.0, steering_pressed=False, steering_torque=50.0)
    msg = controller.handle_angle_lateral(cc, cs_soft_pressure)
    expected = subarucan.create_steering_control_angle(controller.packer, cs_soft_pressure.out.steeringAngleDeg, False)

    self.assertEqual(msg, expected)
    self.assertTrue(controller.subaru_manual_yield_full_release_active)
    self.assertEqual(controller.angle_driver_override_soft_hold_frames, ANGLE_DRIVER_OVERRIDE_SOFT_HOLD_FRAMES[2] - 1)
    self.assertAlmostEqual(controller.apply_angle_last, cs_soft_pressure.out.steeringAngleDeg)

  def test_manual_steering_soft_hold_low_pressure_does_not_refresh_window(self):
    controller = self._build_controller(soft_hold_level=1)
    cc = self._build_cc(True, True, 14.0)
    cs_pressed = self._build_cs(8.0, 10.0, steering_pressed=True, steering_torque=100.0)
    cs_soft_pressure = self._build_cs(8.0, 10.0, steering_pressed=False, steering_torque=60.0)
    controller.apply_angle_last = cs_pressed.out.steeringAngleDeg

    controller.handle_angle_lateral(cc, cs_pressed)

    for expected_frames in range(ANGLE_DRIVER_OVERRIDE_SOFT_HOLD_FRAMES[1] - 1, -1, -1):
      msg = controller.handle_angle_lateral(cc, cs_soft_pressure)
      expected = subarucan.create_steering_control_angle(controller.packer, cs_soft_pressure.out.steeringAngleDeg, False)
      self.assertEqual(msg, expected)
      self.assertEqual(controller.angle_driver_override_soft_hold_frames, expected_frames)

    msg = controller.handle_angle_lateral(cc, cs_soft_pressure)
    inhibited = subarucan.create_steering_control_angle(controller.packer, cs_soft_pressure.out.steeringAngleDeg, False)

    self.assertNotEqual(msg, inhibited)
    self.assertFalse(controller.subaru_manual_yield_full_release_active)

  def test_manual_steering_soft_hold_real_press_refreshes_window(self):
    controller = self._build_controller(soft_hold_level=1)
    cc = self._build_cc(True, True, 14.0)
    cs_pressed = self._build_cs(8.0, 10.0, steering_pressed=True, steering_torque=100.0)
    cs_soft_pressure = self._build_cs(8.0, 10.0, steering_pressed=False, steering_torque=60.0)
    controller.apply_angle_last = cs_pressed.out.steeringAngleDeg

    controller.handle_angle_lateral(cc, cs_pressed)
    for _ in range(10):
      controller.handle_angle_lateral(cc, cs_soft_pressure)

    self.assertLess(controller.angle_driver_override_soft_hold_frames, ANGLE_DRIVER_OVERRIDE_SOFT_HOLD_FRAMES[1])

    controller.handle_angle_lateral(cc, cs_pressed)

    self.assertEqual(controller.angle_driver_override_soft_hold_frames, ANGLE_DRIVER_OVERRIDE_SOFT_HOLD_FRAMES[1])

  def test_manual_steering_soft_hold_flows_into_existing_release_guard_after_window_expires(self):
    controller = self._build_controller(
      soft_hold_level=1,
      release_guard_enabled=True,
      release_guard_level=2,
      resume_softness_enabled=True,
    )
    cc = self._build_cc(True, True, 14.0)
    cs_pressed = self._build_cs(8.0, 10.0, steering_pressed=True, steering_torque=100.0)
    cs_soft_pressure = self._build_cs(8.0, 10.0, steering_pressed=False, steering_torque=60.0)
    cs_released = self._build_cs(8.0, 10.0, steering_pressed=False, steering_torque=0.0)
    controller.apply_angle_last = cs_pressed.out.steeringAngleDeg

    controller.handle_angle_lateral(cc, cs_pressed)
    for _ in range(ANGLE_DRIVER_OVERRIDE_SOFT_HOLD_FRAMES[1]):
      controller.handle_angle_lateral(cc, cs_soft_pressure)

    self.assertEqual(controller.angle_driver_override_soft_hold_frames, 0)
    self.assertEqual(controller.angle_driver_override_hold_frames, ANGLE_DRIVER_OVERRIDE_HOLD_FRAMES)

    for _ in range(ANGLE_DRIVER_OVERRIDE_HOLD_FRAMES):
      controller.handle_angle_lateral(cc, cs_released)

    self.assertTrue(controller.angle_driver_override_release_guard_pending)
    self.assertEqual(controller.angle_driver_override_ramp_frames, 0)

  def test_angle_driver_override_ramp_progresses_monotonically_toward_live_target_in_mads_only(self):
    controller = self._build_controller()
    cc = self._build_cc(True, False, 14.0)
    cs_released = self._prime_angle_driver_override_ramp(controller, cc)

    ramped_angles = []
    for _ in range(6):
      controller.handle_angle_lateral(cc, cs_released)
      ramped_angles.append(controller.apply_angle_last)

    self.assertTrue(all(left <= right for left, right in zip(ramped_angles, ramped_angles[1:], strict=True)))
    self.assertGreater(ramped_angles[-1], cs_released.out.steeringAngleDeg)
    self.assertLessEqual(ramped_angles[-1], cc.actuators.steeringAngleDeg)

  def test_angle_driver_override_ramp_uses_live_target_in_full_engaged(self):
    controller = self._build_controller()
    cc_release = self._build_cc(True, True, 14.0)
    cs_released = self._prime_angle_driver_override_ramp(controller, cc_release)
    cc_changed = self._build_cc(True, True, 18.0)

    ramped_angles = []
    for _ in range(10):
      controller.handle_angle_lateral(cc_changed, cs_released)
      ramped_angles.append(controller.apply_angle_last)

    self.assertTrue(all(left <= right for left, right in zip(ramped_angles, ramped_angles[1:], strict=True)))
    self.assertGreater(ramped_angles[-1], 14.0)
    self.assertLessEqual(ramped_angles[-1], cc_changed.actuators.steeringAngleDeg)

  def test_angle_driver_override_softer_profiles_reduce_the_initial_reclaim_delta(self):
    cc = self._build_cc(True, True, 14.0)

    standard_controller = self._build_controller()
    extra_soft_controller = self._build_controller()
    max_soft_controller = self._build_controller()

    standard_released_cs = self._prime_angle_driver_override_ramp(standard_controller, cc, softness_setting=0)
    extra_soft_released_cs = self._prime_angle_driver_override_ramp(extra_soft_controller, cc, softness_setting=4)
    max_soft_released_cs = self._prime_angle_driver_override_ramp(max_soft_controller, cc, softness_setting=6)

    standard_controller.handle_angle_lateral(cc, standard_released_cs)
    extra_soft_controller.handle_angle_lateral(cc, extra_soft_released_cs)
    max_soft_controller.handle_angle_lateral(cc, max_soft_released_cs)

    standard_delta = standard_controller.apply_angle_last - standard_released_cs.out.steeringAngleDeg
    extra_soft_delta = extra_soft_controller.apply_angle_last - extra_soft_released_cs.out.steeringAngleDeg
    max_soft_delta = max_soft_controller.apply_angle_last - max_soft_released_cs.out.steeringAngleDeg

    self.assertGreater(standard_delta, 0.0)
    self.assertGreater(extra_soft_delta, 0.0)
    self.assertGreater(max_soft_delta, 0.0)
    self.assertLess(extra_soft_delta, standard_delta)
    self.assertLess(max_soft_delta, extra_soft_delta)

  def test_angle_driver_override_ramp_cancels_when_driver_input_returns(self):
    controller = self._build_controller()
    cc = self._build_cc(True, True, 14.0)
    cs_released = self._prime_angle_driver_override_ramp(controller, cc)

    controller.handle_angle_lateral(cc, cs_released)
    self.assertLess(controller.angle_driver_override_ramp_frames, ANGLE_DRIVER_OVERRIDE_RAMP_FRAMES)

    cs_pressed = self._build_cs(8.0, 10.0, steering_pressed=True, steering_rate_deg=2.0)
    msg = controller.handle_angle_lateral(cc, cs_pressed)
    expected = subarucan.create_steering_control_angle(controller.packer, cs_pressed.out.steeringAngleDeg, False)

    self.assertEqual(msg, expected)
    self.assertEqual(controller.angle_driver_override_hold_frames, ANGLE_DRIVER_OVERRIDE_HOLD_FRAMES)
    self.assertEqual(controller.angle_driver_override_ramp_frames, 0)
    self.assertAlmostEqual(controller.apply_angle_last, cs_pressed.out.steeringAngleDeg)

  def test_soft_capture_disabled_is_a_no_op(self):
    controller = self._build_controller(soft_capture_enabled=False, soft_capture_level=5)
    controller.soft_capture_frame = controller.frame

    self.assertEqual(controller._get_soft_capture_level(), 0)
    self.assertAlmostEqual(controller._get_soft_capture_angle(18.0, 10.0), 18.0)

  def test_soft_capture_reenable_restores_saved_level(self):
    disabled = self._build_controller(soft_capture_enabled=False, soft_capture_level=5)
    reenabled = self._build_controller(soft_capture_enabled=True, soft_capture_level=5)

    self.assertEqual(disabled._get_soft_capture_level(), 0)
    self.assertEqual(reenabled._get_soft_capture_level(), 5)

  def test_soft_capture_engage_edge_starts_ramp_and_reduces_first_reclaim_step(self):
    baseline = self._build_controller(soft_capture_enabled=False)
    softened = self._build_controller(soft_capture_enabled=True, soft_capture_level=3)
    cc = self._build_cc(True, True, 14.0)
    cs = self._build_cs(8.0, 10.0)

    baseline.apply_angle_last = cs.out.steeringAngleDeg
    softened.apply_angle_last = cs.out.steeringAngleDeg

    baseline.handle_angle_lateral(cc, cs)
    softened.handle_angle_lateral(cc, cs)

    self.assertEqual(softened.soft_capture_frame, 0)
    self.assertTrue(softened.lat_active_prev)
    self.assertLess(softened.apply_angle_last, baseline.apply_angle_last)

  def test_soft_capture_higher_levels_reduce_the_initial_blend_delta(self):
    light = self._build_controller(soft_capture_enabled=True, soft_capture_level=1)
    medium = self._build_controller(soft_capture_enabled=True, soft_capture_level=3)
    maximum = self._build_controller(soft_capture_enabled=True, soft_capture_level=5)

    for controller in (light, medium, maximum):
      controller.soft_capture_frame = 0
      controller.frame = 0

    model_target = 20.0
    wheel_angle = 10.0
    light_delta = light._get_soft_capture_angle(model_target, wheel_angle) - wheel_angle
    medium_delta = medium._get_soft_capture_angle(model_target, wheel_angle) - wheel_angle
    max_delta = maximum._get_soft_capture_angle(model_target, wheel_angle) - wheel_angle

    self.assertGreater(light_delta, medium_delta)
    self.assertGreater(medium_delta, max_delta)

  def test_soft_capture_ramp_completes_and_returns_full_model_control(self):
    controller = self._build_controller(soft_capture_enabled=True, soft_capture_level=3)
    ramp_frames, _ = SOFT_CAPTURE_LEVEL_PARAMS[3]
    controller.soft_capture_frame = 0
    controller.frame = ramp_frames

    self.assertAlmostEqual(controller._get_soft_capture_angle(18.0, 10.0), 18.0)

  def test_soft_capture_does_not_stack_on_manual_override_reclaim(self):
    baseline = self._build_controller(soft_capture_enabled=False)
    softened = self._build_controller(soft_capture_enabled=True, soft_capture_level=5)
    cc = self._build_cc(True, True, 14.0)

    baseline_released_cs = self._prime_angle_driver_override_ramp(baseline, cc)
    softened_released_cs = self._prime_angle_driver_override_ramp(softened, cc)

    self.assertEqual(softened.soft_capture_frame, -(SOFT_CAPTURE_LEVEL_PARAMS[-1][0] + 1))

    baseline.handle_angle_lateral(cc, baseline_released_cs)
    softened.handle_angle_lateral(cc, softened_released_cs)

    self.assertAlmostEqual(softened.apply_angle_last, baseline.apply_angle_last)
    self.assertEqual(softened.soft_capture_frame, -(SOFT_CAPTURE_LEVEL_PARAMS[-1][0] + 1))

  def test_mads_only_below_one_mph_still_inhibits_angle_lkas(self):
    controller = self._build_controller()
    cs = self._build_cs(0.22352, 10.0)
    cc = self._build_cc(True, False, 14.0)
    controller.apply_angle_last = cs.out.steeringAngleDeg

    msg = controller.handle_angle_lateral(cc, cs)
    expected = subarucan.create_steering_control_angle(controller.packer, cs.out.steeringAngleDeg, False)

    self.assertEqual(msg, expected)
    self.assertAlmostEqual(controller.apply_angle_last, cs.out.steeringAngleDeg)

  def test_mads_only_just_above_one_mph_allows_angle_lkas(self):
    controller = self._build_controller()
    cs = self._build_cs(MADS_ONLY_MIN_SPEED + 0.01, 10.0)
    cc = self._build_cc(True, False, 14.0)
    controller.apply_angle_last = cs.out.steeringAngleDeg

    msg = controller.handle_angle_lateral(cc, cs)
    inhibited = subarucan.create_steering_control_angle(controller.packer, cs.out.steeringAngleDeg, False)

    self.assertNotEqual(msg, inhibited)
    self.assertGreater(controller.apply_angle_last, cs.out.steeringAngleDeg)

  def test_mads_only_standstill_still_inhibits_above_one_mph(self):
    controller = self._build_controller()
    cs = self._build_cs(MADS_ONLY_MIN_SPEED + 0.5, 10.0, standstill=True)
    cc = self._build_cc(True, False, 14.0)
    controller.apply_angle_last = cs.out.steeringAngleDeg

    msg = controller.handle_angle_lateral(cc, cs)
    expected = subarucan.create_steering_control_angle(controller.packer, cs.out.steeringAngleDeg, False)

    self.assertEqual(msg, expected)
    self.assertAlmostEqual(controller.apply_angle_last, cs.out.steeringAngleDeg)

  def test_mads_only_angle_limit_still_inhibits_above_one_mph(self):
    controller = self._build_controller()
    cs = self._build_cs(MADS_ONLY_MIN_SPEED + 0.5, MADS_ONLY_MAX_STEER_ANGLE)
    cc = self._build_cc(True, False, MADS_ONLY_MAX_STEER_ANGLE + 4.0)
    controller.apply_angle_last = cs.out.steeringAngleDeg

    msg = controller.handle_angle_lateral(cc, cs)
    expected = subarucan.create_steering_control_angle(controller.packer, cs.out.steeringAngleDeg, False)

    self.assertEqual(msg, expected)
    self.assertAlmostEqual(controller.apply_angle_last, cs.out.steeringAngleDeg)
    self.assertFalse(controller.subaru_effective_lkas_active)

  def test_mads_only_tighter_turns_toggle_off_preserves_stock_angle_gate(self):
    controller = self._build_controller(
      mads_tighter_turns_enabled=False,
      mads_max_steering_angle=int(MADS_ONLY_MAX_STEER_ANGLE_MAX),
    )

    self.assertEqual(controller._get_mads_only_max_steer_angle(), MADS_ONLY_MAX_STEER_ANGLE)

    cs = self._build_cs(MADS_ONLY_MIN_SPEED + 0.5, MADS_ONLY_MAX_STEER_ANGLE)
    cc = self._build_cc(True, False, MADS_ONLY_MAX_STEER_ANGLE + 4.0)
    controller.apply_angle_last = cs.out.steeringAngleDeg

    msg = controller.handle_angle_lateral(cc, cs)
    expected = subarucan.create_steering_control_angle(controller.packer, cs.out.steeringAngleDeg, False)

    self.assertEqual(msg, expected)
    self.assertAlmostEqual(controller.apply_angle_last, cs.out.steeringAngleDeg)

  def test_mads_only_tighter_turns_levels_increase_angle_cap(self):
    for angle_cap in (180, 190, 199, 200, 240, 360, 545):
      controller = self._build_controller(
        mads_tighter_turns_enabled=True,
        mads_max_steering_angle=angle_cap,
      )

      self.assertEqual(controller._get_mads_only_max_steer_angle(), float(angle_cap))

  def test_mads_only_active_cap_is_speed_shaped(self):
    selected_cap = 240.0

    self.assertEqual(
      CarController._get_mads_only_active_steer_angle_cap(selected_cap, MADS_ONLY_FAULT_GUARD_LOW_SPEED - 0.01),
      MADS_ONLY_FAULT_GUARD_LOW_SPEED_CAP,
    )
    self.assertEqual(
      CarController._get_mads_only_active_steer_angle_cap(selected_cap, MADS_ONLY_FAULT_GUARD_HIGH_SPEED + 0.01),
      selected_cap,
    )

    mid_cap = CarController._get_mads_only_active_steer_angle_cap(
      selected_cap,
      (MADS_ONLY_FAULT_GUARD_LOW_SPEED + MADS_ONLY_FAULT_GUARD_HIGH_SPEED) / 2,
    )
    self.assertGreater(mid_cap, MADS_ONLY_FAULT_GUARD_LOW_SPEED_CAP)
    self.assertLess(mid_cap, selected_cap)

  def test_mads_only_tighter_turns_clamps_invalid_saved_angle_caps(self):
    below_floor = self._build_controller(
      mads_tighter_turns_enabled=True,
      mads_max_steering_angle=60,
    )
    above_ceiling = self._build_controller(
      mads_tighter_turns_enabled=True,
      mads_max_steering_angle=900,
    )

    self.assertEqual(below_floor._get_mads_only_max_steer_angle(), MADS_ONLY_MAX_STEER_ANGLE)
    self.assertEqual(above_ceiling._get_mads_only_max_steer_angle(), MADS_ONLY_MAX_STEER_ANGLE_MAX)

  def test_mads_only_tighter_turns_allows_selected_higher_angle_cap(self):
    controller = self._build_controller(
      mads_tighter_turns_enabled=True,
      mads_max_steering_angle=180,
    )
    cs = self._build_cs(MADS_ONLY_MIN_SPEED + 0.5, 150.0)
    cc = self._build_cc(True, False, 154.0)
    controller.apply_angle_last = cs.out.steeringAngleDeg

    msg = controller.handle_angle_lateral(cc, cs)
    inhibited = subarucan.create_steering_control_angle(controller.packer, cs.out.steeringAngleDeg, False)

    self.assertNotEqual(msg, inhibited)
    self.assertGreater(controller.apply_angle_last, cs.out.steeringAngleDeg)

  def test_mads_only_tighter_turns_clamps_requested_target_to_selected_cap(self):
    for angle_cap, requested_angle in ((190, 286.0), (199, 226.0)):
      controller = self._build_controller(
        mads_tighter_turns_enabled=True,
        mads_max_steering_angle=angle_cap,
      )
      cs = self._build_cs(MADS_ONLY_FAULT_GUARD_HIGH_SPEED + 0.5, angle_cap - 10.0)
      cc = self._build_cc(True, False, requested_angle)
      controller.apply_angle_last = float(angle_cap)

      controller.handle_angle_lateral(cc, cs)

      self.assertLessEqual(abs(controller.apply_angle_last), float(angle_cap))
      self.assertAlmostEqual(controller.apply_angle_last, float(angle_cap))

  def test_mads_only_tighter_turns_clamps_observed_low_speed_fault_shape(self):
    controller = self._build_controller(
      mads_tighter_turns_enabled=True,
      mads_max_steering_angle=199,
    )
    cs = self._build_cs(MADS_ONLY_FAULT_GUARD_LOW_SPEED - 0.1, 176.2)
    cc = self._build_cc(True, False, 226.2)
    controller.apply_angle_last = MADS_ONLY_FAULT_GUARD_LOW_SPEED_CAP

    controller.handle_angle_lateral(cc, cs)

    self.assertLessEqual(abs(controller.apply_angle_last), MADS_ONLY_FAULT_GUARD_LOW_SPEED_CAP)
    self.assertAlmostEqual(controller.apply_angle_last, MADS_ONLY_FAULT_GUARD_LOW_SPEED_CAP)

  def test_mads_only_tighter_turns_toggle_off_clamps_requested_target_to_stock_cap(self):
    controller = self._build_controller(
      mads_tighter_turns_enabled=False,
      mads_max_steering_angle=int(MADS_ONLY_MAX_STEER_ANGLE_MAX),
    )
    cs = self._build_cs(MADS_ONLY_MIN_SPEED + 0.5, MADS_ONLY_MAX_STEER_ANGLE - 10.0)
    cc = self._build_cc(True, False, 286.0)
    controller.apply_angle_last = MADS_ONLY_MAX_STEER_ANGLE

    controller.handle_angle_lateral(cc, cs)

    self.assertLessEqual(abs(controller.apply_angle_last), MADS_ONLY_MAX_STEER_ANGLE)
    self.assertAlmostEqual(controller.apply_angle_last, MADS_ONLY_MAX_STEER_ANGLE)

  def test_mads_only_tighter_turns_clamps_manual_yield_ramp_output_to_cap(self):
    controller = self._build_controller(
      mads_tighter_turns_enabled=True,
      mads_max_steering_angle=190,
      resume_softness_enabled=True,
      resume_softness_setting=0,
    )
    cs = self._build_cs(MADS_ONLY_FAULT_GUARD_HIGH_SPEED + 0.5, 180.0)
    cc = self._build_cc(True, False, 300.0)
    controller.apply_angle_last = 190.0
    controller.angle_driver_override_ramp_frames = 1
    controller.angle_driver_override_ramp_total_frames = 1
    controller.angle_driver_override_ramp_start_angle = 180.0
    controller.angle_driver_override_ramp_softness_exponent = 1.0

    controller.handle_angle_lateral(cc, cs)

    self.assertLessEqual(abs(controller.apply_angle_last), 190.0)
    self.assertAlmostEqual(controller.apply_angle_last, 190.0)

  def test_mads_only_tighter_turns_clamps_soft_capture_output_to_cap(self):
    controller = self._build_controller(
      mads_tighter_turns_enabled=True,
      mads_max_steering_angle=190,
      soft_capture_enabled=True,
      soft_capture_level=1,
    )
    cs = self._build_cs(MADS_ONLY_FAULT_GUARD_HIGH_SPEED + 0.5, 180.0)
    cc = self._build_cc(True, False, 300.0)
    controller.apply_angle_last = 190.0

    controller.handle_angle_lateral(cc, cs)

    self.assertEqual(controller.soft_capture_frame, 0)
    self.assertLessEqual(abs(controller.apply_angle_last), 190.0)
    self.assertAlmostEqual(controller.apply_angle_last, 190.0)

  def test_mads_only_fault_guard_caps_low_speed_request_to_180(self):
    controller = self._build_controller(
      mads_tighter_turns_enabled=True,
      mads_max_steering_angle=199,
    )
    cs = self._build_cs(MADS_ONLY_FAULT_GUARD_LOW_SPEED - 0.1, 170.0)
    cc = self._build_cc(True, False, 260.0)
    controller.apply_angle_last = MADS_ONLY_FAULT_GUARD_LOW_SPEED_CAP

    controller.handle_angle_lateral(cc, cs)

    self.assertLessEqual(abs(controller.apply_angle_last), MADS_ONLY_FAULT_GUARD_LOW_SPEED_CAP)
    self.assertAlmostEqual(controller.apply_angle_last, MADS_ONLY_FAULT_GUARD_LOW_SPEED_CAP)

  def test_mads_only_fault_guard_inhibits_high_rate_low_speed_high_angle(self):
    controller = self._build_controller(
      mads_tighter_turns_enabled=True,
      mads_max_steering_angle=240,
    )
    cs = self._build_cs(
      MADS_ONLY_FAULT_GUARD_LOW_SPEED - 0.1,
      165.0,
      steering_rate_deg=MADS_ONLY_FAULT_GUARD_RATE_THRESHOLD + 1.0,
    )
    cc = self._build_cc(True, False, 170.0)
    controller.apply_angle_last = cs.out.steeringAngleDeg

    msg = controller.handle_angle_lateral(cc, cs)
    expected = subarucan.create_steering_control_angle(controller.packer, cs.out.steeringAngleDeg, False)

    self.assertEqual(msg, expected)
    self.assertEqual(controller.mads_lkas_fault_guard_quiet_frames, MADS_ONLY_FAULT_GUARD_QUIET_FRAMES)
    self.assertFalse(controller.subaru_effective_lkas_active)

  def test_mads_only_fault_guard_waits_for_quiet_window_before_reenable(self):
    controller = self._build_controller(
      mads_tighter_turns_enabled=True,
      mads_max_steering_angle=240,
    )
    cc = self._build_cc(True, False, 170.0)
    risky_cs = self._build_cs(
      MADS_ONLY_FAULT_GUARD_LOW_SPEED - 0.1,
      165.0,
      steering_rate_deg=MADS_ONLY_FAULT_GUARD_RATE_THRESHOLD + 1.0,
    )
    quiet_cs = self._build_cs(MADS_ONLY_FAULT_GUARD_LOW_SPEED - 0.1, 150.0, steering_rate_deg=0.0)
    controller.apply_angle_last = risky_cs.out.steeringAngleDeg

    controller.handle_angle_lateral(cc, risky_cs)
    self.assertEqual(controller.mads_lkas_fault_guard_quiet_frames, MADS_ONLY_FAULT_GUARD_QUIET_FRAMES)

    for expected_frames in range(MADS_ONLY_FAULT_GUARD_QUIET_FRAMES - 1, -1, -1):
      msg = controller.handle_angle_lateral(cc, quiet_cs)
      expected = subarucan.create_steering_control_angle(controller.packer, quiet_cs.out.steeringAngleDeg, False)
      self.assertEqual(msg, expected)
      self.assertEqual(controller.mads_lkas_fault_guard_quiet_frames, expected_frames)

    msg = controller.handle_angle_lateral(cc, quiet_cs)
    inhibited = subarucan.create_steering_control_angle(controller.packer, quiet_cs.out.steeringAngleDeg, False)
    self.assertNotEqual(msg, inhibited)

  def test_max_steering_low_speed_guard_inhibits_turn_entry_without_manual_yield(self):
    controller = self._build_controller(
      mads_tighter_turns_enabled=True,
      mads_max_steering_angle=199,
      turn_in_rate_level=20,
      unwind_rate_level=20,
      torque_threshold_enabled=True,
      torque_threshold=500,
      max_steering_experiment=True,
    )
    cs = self._build_cs(
      8.5 * 0.44704,
      -1.1,
      steering_pressed=False,
      steering_rate_deg=-6.5,
      steering_torque=7.0,
    )
    cc = self._build_cc(True, False, -19.1)
    controller.apply_angle_last = cs.out.steeringAngleDeg

    msg = controller.handle_angle_lateral(cc, cs)
    expected = subarucan.create_steering_control_angle(controller.packer, cs.out.steeringAngleDeg, False)

    self.assertEqual(msg, expected)
    self.assertTrue(controller.max_steering_low_speed_guard_active)
    self.assertEqual(controller.max_steering_low_speed_guard_quiet_frames, MAX_STEERING_LOW_SPEED_GUARD_QUIET_FRAMES)
    self.assertEqual(controller.mc_subaru_manual_yield_torque_threshold, 500)
    self.assertFalse(controller.subaru_manual_yield_full_release_active)

  def test_max_steering_low_speed_guard_latches_high_angle_and_rate_shapes(self):
    controller = self._build_controller(
      mads_tighter_turns_enabled=True,
      mads_max_steering_angle=199,
      max_steering_experiment=True,
    )
    cc = self._build_cc(True, False, 6.0)
    high_angle_cs = self._build_cs(9.0 * 0.44704, 57.4, steering_rate_deg=12.0)
    high_rate_cs = self._build_cs(9.5 * 0.44704, 6.0, steering_rate_deg=75.0)
    controller.apply_angle_last = high_angle_cs.out.steeringAngleDeg

    high_angle_msg = controller.handle_angle_lateral(cc, high_angle_cs)
    high_angle_expected = subarucan.create_steering_control_angle(controller.packer, high_angle_cs.out.steeringAngleDeg, False)
    self.assertEqual(high_angle_msg, high_angle_expected)
    self.assertTrue(controller.max_steering_low_speed_guard_active)

    high_rate_msg = controller.handle_angle_lateral(cc, high_rate_cs)
    high_rate_expected = subarucan.create_steering_control_angle(controller.packer, high_rate_cs.out.steeringAngleDeg, False)
    self.assertEqual(high_rate_msg, high_rate_expected)
    self.assertTrue(controller.max_steering_low_speed_guard_active)

  def test_max_steering_low_speed_guard_keeps_max_available_above_exit_speed(self):
    controller = self._build_controller(
      mads_tighter_turns_enabled=True,
      mads_max_steering_angle=199,
      turn_in_rate_level=20,
      unwind_rate_level=20,
      max_steering_experiment=True,
    )
    cs = self._build_cs(MAX_STEERING_LOW_SPEED_GUARD_EXIT_SPEED + 0.2, 40.0, steering_rate_deg=80.0)
    cc = self._build_cc(True, False, 160.0)
    controller.apply_angle_last = cs.out.steeringAngleDeg

    msg = controller.handle_angle_lateral(cc, cs)
    inhibited = subarucan.create_steering_control_angle(controller.packer, cs.out.steeringAngleDeg, False)

    self.assertNotEqual(msg, inhibited)
    self.assertFalse(controller.max_steering_low_speed_guard_active)
    self.assertGreater(controller.apply_angle_last, cs.out.steeringAngleDeg)

  def test_max_steering_low_speed_guard_ignores_straight_low_speed_crawl(self):
    controller = self._build_controller(
      mads_tighter_turns_enabled=True,
      mads_max_steering_angle=199,
      max_steering_experiment=True,
    )
    cs = self._build_cs(MAX_STEERING_LOW_SPEED_GUARD_ENTER_SPEED - 0.2, 5.0, steering_rate_deg=0.0)
    cc = self._build_cc(True, False, 5.0)
    controller.apply_angle_last = cs.out.steeringAngleDeg

    msg = controller.handle_angle_lateral(cc, cs)
    inhibited = subarucan.create_steering_control_angle(controller.packer, cs.out.steeringAngleDeg, False)

    self.assertNotEqual(msg, inhibited)
    self.assertFalse(controller.max_steering_low_speed_guard_active)

  def test_max_steering_low_speed_guard_reenables_after_quiet_window_or_speed_exit(self):
    controller = self._build_controller(
      mads_tighter_turns_enabled=True,
      mads_max_steering_angle=199,
      max_steering_experiment=True,
    )
    risky_cs = self._build_cs(MAX_STEERING_LOW_SPEED_GUARD_ENTER_SPEED - 0.2, 32.0, steering_rate_deg=10.0)
    quiet_cs = self._build_cs(MAX_STEERING_LOW_SPEED_GUARD_ENTER_SPEED - 0.2, 0.0, steering_rate_deg=0.0)
    speed_exit_cs = self._build_cs(MAX_STEERING_LOW_SPEED_GUARD_EXIT_SPEED + 0.2, 40.0, steering_rate_deg=80.0)
    cc = self._build_cc(True, False, 0.0)
    controller.apply_angle_last = risky_cs.out.steeringAngleDeg

    controller.handle_angle_lateral(cc, risky_cs)
    self.assertTrue(controller.max_steering_low_speed_guard_active)

    for expected_frames in range(MAX_STEERING_LOW_SPEED_GUARD_QUIET_FRAMES - 1, -1, -1):
      msg = controller.handle_angle_lateral(cc, quiet_cs)
      expected = subarucan.create_steering_control_angle(controller.packer, quiet_cs.out.steeringAngleDeg, False)
      self.assertEqual(msg, expected)
      self.assertEqual(controller.max_steering_low_speed_guard_quiet_frames, expected_frames)

    msg = controller.handle_angle_lateral(cc, quiet_cs)
    inhibited = subarucan.create_steering_control_angle(controller.packer, quiet_cs.out.steeringAngleDeg, False)
    self.assertNotEqual(msg, inhibited)

    controller.handle_angle_lateral(cc, risky_cs)
    self.assertTrue(controller.max_steering_low_speed_guard_active)
    msg = controller.handle_angle_lateral(cc, speed_exit_cs)
    inhibited = subarucan.create_steering_control_angle(controller.packer, speed_exit_cs.out.steeringAngleDeg, False)
    self.assertNotEqual(msg, inhibited)
    self.assertFalse(controller.max_steering_low_speed_guard_active)

  def test_max_steering_low_speed_guard_off_preserves_non_max_behavior(self):
    controller = self._build_controller(
      mads_tighter_turns_enabled=True,
      mads_max_steering_angle=199,
      turn_in_rate_level=20,
      unwind_rate_level=20,
      torque_threshold_enabled=True,
      torque_threshold=500,
      max_steering_experiment=False,
    )
    cs = self._build_cs(8.5 * 0.44704, -1.1, steering_pressed=False, steering_rate_deg=-6.5)
    cc = self._build_cc(True, False, -19.1)
    controller.apply_angle_last = cs.out.steeringAngleDeg

    msg = controller.handle_angle_lateral(cc, cs)
    inhibited = subarucan.create_steering_control_angle(controller.packer, cs.out.steeringAngleDeg, False)

    self.assertNotEqual(msg, inhibited)
    self.assertFalse(controller.max_steering_low_speed_guard_active)

  def test_max_steering_low_speed_guard_ignored_during_full_engaged_lateral(self):
    controller = self._build_controller(
      mads_tighter_turns_enabled=True,
      mads_max_steering_angle=199,
      turn_in_rate_level=20,
      unwind_rate_level=20,
      max_steering_experiment=True,
    )
    cs = self._build_cs(MAX_STEERING_LOW_SPEED_GUARD_ENTER_SPEED - 0.2, 57.4, steering_rate_deg=75.0)
    cc = self._build_cc(True, True, 120.0)
    controller.apply_angle_last = cs.out.steeringAngleDeg

    msg = controller.handle_angle_lateral(cc, cs)
    inhibited = subarucan.create_steering_control_angle(controller.packer, cs.out.steeringAngleDeg, False)

    self.assertNotEqual(msg, inhibited)
    self.assertFalse(controller.max_steering_low_speed_guard_active)

  def test_mads_only_angle_limit_latches_fault_guard_before_reenable(self):
    controller = self._build_controller(
      mads_tighter_turns_enabled=True,
      mads_max_steering_angle=199,
      unwind_rate_level=20,
    )
    cc = self._build_cc(True, False, -184.92)
    angle_limit_cs = self._build_cs(4.79, -187.97, steering_rate_deg=-38.5)
    controller.apply_angle_last = -187.41

    msg = controller.handle_angle_lateral(cc, angle_limit_cs)
    expected = subarucan.create_steering_control_angle(controller.packer, angle_limit_cs.out.steeringAngleDeg, False)

    self.assertEqual(msg, expected)
    self.assertEqual(controller.mads_lkas_fault_guard_source, "mads_angle_limit")
    self.assertEqual(controller.mads_lkas_fault_guard_quiet_frames, MADS_ONLY_FAULT_GUARD_QUIET_FRAMES)
    self.assertFalse(controller.subaru_effective_lkas_active)

    fault_shape_cs = self._build_cs(3.50, -179.59, steering_rate_deg=MADS_ONLY_FAULT_GUARD_QUIET_RATE + 15.0)
    fault_shape_cc = self._build_cc(True, False, -39.17)

    msg = controller.handle_angle_lateral(fault_shape_cc, fault_shape_cs)
    expected = subarucan.create_steering_control_angle(controller.packer, fault_shape_cs.out.steeringAngleDeg, False)

    self.assertEqual(msg, expected)
    self.assertEqual(controller.mads_lkas_fault_guard_source, "mads_angle_limit")
    self.assertEqual(controller.mads_lkas_fault_guard_quiet_frames, MADS_ONLY_FAULT_GUARD_QUIET_FRAMES)
    self.assertFalse(controller.subaru_effective_lkas_active)

  def test_mads_only_angle_limit_guard_requires_angle_margin_before_reenable(self):
    controller = self._build_controller(
      mads_tighter_turns_enabled=True,
      mads_max_steering_angle=199,
    )
    cc = self._build_cc(True, False, -39.17)
    angle_limit_cs = self._build_cs(4.79, -187.97, steering_rate_deg=-38.5)
    near_cap_cs = self._build_cs(3.50, -(MADS_ONLY_FAULT_GUARD_LOW_SPEED_CAP - 1.0), steering_rate_deg=0.0)
    quiet_cs = self._build_cs(
      3.50,
      -(MADS_ONLY_FAULT_GUARD_LOW_SPEED_CAP - MADS_ONLY_FAULT_GUARD_REENABLE_MARGIN - 1.0),
      steering_rate_deg=MADS_ONLY_FAULT_GUARD_QUIET_RATE,
    )
    controller.apply_angle_last = angle_limit_cs.out.steeringAngleDeg

    controller.handle_angle_lateral(cc, angle_limit_cs)
    msg = controller.handle_angle_lateral(cc, near_cap_cs)
    expected = subarucan.create_steering_control_angle(controller.packer, near_cap_cs.out.steeringAngleDeg, False)

    self.assertEqual(msg, expected)
    self.assertEqual(controller.mads_lkas_fault_guard_quiet_frames, MADS_ONLY_FAULT_GUARD_QUIET_FRAMES)

    for expected_frames in range(MADS_ONLY_FAULT_GUARD_QUIET_FRAMES - 1, -1, -1):
      msg = controller.handle_angle_lateral(cc, quiet_cs)
      expected = subarucan.create_steering_control_angle(controller.packer, quiet_cs.out.steeringAngleDeg, False)
      self.assertEqual(msg, expected)
      self.assertEqual(controller.mads_lkas_fault_guard_quiet_frames, expected_frames)

    msg = controller.handle_angle_lateral(cc, quiet_cs)
    inhibited = subarucan.create_steering_control_angle(controller.packer, quiet_cs.out.steeringAngleDeg, False)
    self.assertNotEqual(msg, inhibited)

  def test_mads_only_reenable_unwind_clamp_limits_first_centerward_steps(self):
    controller = self._build_controller(
      mads_tighter_turns_enabled=True,
      mads_max_steering_angle=199,
      unwind_rate_level=20,
    )
    cc = self._build_cc(True, False, -39.17)
    angle_limit_cs = self._build_cs(4.79, -187.97, steering_rate_deg=-38.5)
    quiet_cs = self._build_cs(
      3.50,
      -(MADS_ONLY_FAULT_GUARD_LOW_SPEED_CAP - MADS_ONLY_FAULT_GUARD_REENABLE_MARGIN - 1.0),
      steering_rate_deg=0.0,
    )
    controller.apply_angle_last = angle_limit_cs.out.steeringAngleDeg

    controller.handle_angle_lateral(cc, angle_limit_cs)
    for _ in range(MADS_ONLY_FAULT_GUARD_QUIET_FRAMES):
      controller.handle_angle_lateral(cc, quiet_cs)

    self.assertEqual(controller.mads_lkas_reenable_unwind_clamp_frames, MADS_ONLY_FAULT_GUARD_REENABLE_UNWIND_FRAMES)

    for expected_frames in range(MADS_ONLY_FAULT_GUARD_REENABLE_UNWIND_FRAMES - 1, -1, -1):
      last_angle = controller.apply_angle_last
      controller.handle_angle_lateral(cc, quiet_cs)
      self.assertAlmostEqual(abs(controller.apply_angle_last - last_angle), MADS_ONLY_FAULT_GUARD_REENABLE_UNWIND_STEP)
      self.assertEqual(controller.mads_lkas_reenable_unwind_clamp_frames, expected_frames)

    last_angle = controller.apply_angle_last
    controller.handle_angle_lateral(cc, quiet_cs)
    self.assertGreater(abs(controller.apply_angle_last - last_angle), MADS_ONLY_FAULT_GUARD_REENABLE_UNWIND_STEP)

  def test_mads_only_angle_limit_comparator_delays_calm_high_angle_reenable_until_margin(self):
    controller = self._build_controller(
      mads_tighter_turns_enabled=True,
      mads_max_steering_angle=199,
    )
    cc = self._build_cc(True, False, -201.17)
    angle_limit_cs = self._build_cs(4.00, -182.71, steering_rate_deg=-79.5)
    calm_near_cap_cs = self._build_cs(4.29, -184.29, steering_rate_deg=12.5)
    quiet_cs = self._build_cs(4.29, -178.0, steering_rate_deg=12.5)
    controller.apply_angle_last = -182.54

    controller.handle_angle_lateral(cc, angle_limit_cs)
    msg = controller.handle_angle_lateral(cc, calm_near_cap_cs)
    expected = subarucan.create_steering_control_angle(controller.packer, calm_near_cap_cs.out.steeringAngleDeg, False)

    self.assertEqual(msg, expected)
    self.assertEqual(controller.mads_lkas_fault_guard_source, "mads_angle_limit")
    self.assertEqual(controller.mads_lkas_fault_guard_quiet_frames, MADS_ONLY_FAULT_GUARD_QUIET_FRAMES)

    for _ in range(MADS_ONLY_FAULT_GUARD_QUIET_FRAMES):
      controller.handle_angle_lateral(cc, quiet_cs)

    msg = controller.handle_angle_lateral(cc, quiet_cs)
    inhibited = subarucan.create_steering_control_angle(controller.packer, quiet_cs.out.steeringAngleDeg, False)
    self.assertNotEqual(msg, inhibited)

  def test_mads_only_tighter_turns_does_not_override_standstill_or_gear_gates(self):
    controller = self._build_controller(
      mads_tighter_turns_enabled=True,
      mads_max_steering_angle=545,
    )
    cc = self._build_cc(True, False, 154.0)

    standstill_cs = self._build_cs(MADS_ONLY_MIN_SPEED + 0.5, 150.0, standstill=True)
    controller.apply_angle_last = standstill_cs.out.steeringAngleDeg
    standstill_msg = controller.handle_angle_lateral(cc, standstill_cs)
    standstill_expected = subarucan.create_steering_control_angle(controller.packer, standstill_cs.out.steeringAngleDeg, False)
    self.assertEqual(standstill_msg, standstill_expected)

    reverse_cs = self._build_cs(MADS_ONLY_MIN_SPEED + 0.5, 150.0)
    reverse_cs.out.gearShifter = structs.CarState.GearShifter.reverse
    controller.apply_angle_last = reverse_cs.out.steeringAngleDeg
    reverse_msg = controller.handle_angle_lateral(cc, reverse_cs)
    reverse_expected = subarucan.create_steering_control_angle(controller.packer, reverse_cs.out.steeringAngleDeg, False)
    self.assertEqual(reverse_msg, reverse_expected)

  def test_mads_only_tighter_turns_does_not_affect_full_engaged_lateral(self):
    controller = self._build_controller(
      mads_tighter_turns_enabled=True,
      mads_max_steering_angle=190,
    )
    cs = self._build_cs(MADS_ONLY_MIN_SPEED + 0.5, 176.0)
    cc = self._build_cc(True, True, 286.0)
    controller.apply_angle_last = 240.0

    msg = controller.handle_angle_lateral(cc, cs)
    inhibited = subarucan.create_steering_control_angle(controller.packer, cs.out.steeringAngleDeg, False)

    self.assertNotEqual(msg, inhibited)
    self.assertGreater(controller.apply_angle_last, 190.0)

  def test_mads_only_tighter_turns_does_not_touch_torque_lkas_path(self):
    source = inspect.getsource(CarController.handle_torque_lateral)

    self.assertNotIn("MCSubaruMadsTighterTurnsEnabled", source)
    self.assertNotIn("MCSubaruMadsMaxSteeringAngle", source)
    self.assertNotIn("MCSubaruManualSteeringSoftHoldLevel", source)
    self.assertNotIn("angle_driver_override_soft_hold", source)

  def test_full_engaged_lateral_ignores_mads_only_low_speed_floor(self):
    controller = self._build_controller()
    cs = self._build_cs(0.22352, 10.0)
    cc = self._build_cc(True, True, 14.0)
    controller.apply_angle_last = cs.out.steeringAngleDeg

    msg = controller.handle_angle_lateral(cc, cs)
    inhibited = subarucan.create_steering_control_angle(controller.packer, cs.out.steeringAngleDeg, False)

    self.assertNotEqual(msg, inhibited)
    self.assertGreater(controller.apply_angle_last, cs.out.steeringAngleDeg)

  def test_retired_low_speed_tuning_stack_keeps_raw_angle_target(self):
    controller = self._build_controller()

    target = controller._get_angle_lkas_target(1.2)

    self.assertAlmostEqual(target, 1.2)

  def test_subaru_unwind_rate_level_zero_matches_stock_angle_limits(self):
    controller = self._build_controller(unwind_rate_level=0)

    limits = controller._get_active_angle_limits()

    self.assertIsNot(limits, controller.p.ANGLE_LIMITS)
    self.assertEqual(limits.STEER_ANGLE_MAX, controller.p.ANGLE_LIMITS.STEER_ANGLE_MAX)
    self.assertEqual(limits.ANGLE_RATE_LIMIT_UP, controller.p.ANGLE_LIMITS.ANGLE_RATE_LIMIT_UP)
    self.assertEqual(limits.ANGLE_RATE_LIMIT_DOWN, controller.p.ANGLE_LIMITS.ANGLE_RATE_LIMIT_DOWN)

  def test_subaru_unwind_rate_level_updates_only_down_limit(self):
    for level, expected_mid in enumerate(SUBARU_UNWIND_RATE_LEVEL_VALUES):
      with self.subTest(level=level):
        controller = self._build_controller(unwind_rate_level=level)

        limits = controller._get_active_angle_limits()

        self.assertEqual(limits.STEER_ANGLE_MAX, controller.p.ANGLE_LIMITS.STEER_ANGLE_MAX)
        self.assertEqual(limits.ANGLE_RATE_LIMIT_UP, controller.p.ANGLE_LIMITS.ANGLE_RATE_LIMIT_UP)
        self.assertEqual(limits.ANGLE_RATE_LIMIT_DOWN[0], controller.p.ANGLE_LIMITS.ANGLE_RATE_LIMIT_DOWN[0])
        self.assertEqual(limits.ANGLE_RATE_LIMIT_DOWN[1], [5.0, expected_mid, 0.15])

  def test_subaru_unwind_rate_level_clamps_saved_values(self):
    below = self._build_controller(unwind_rate_level=-5)
    above = self._build_controller(unwind_rate_level=99)

    self.assertEqual(below.mc_subaru_unwind_rate_level, SUBARU_UNWIND_RATE_LEVEL_MIN)
    self.assertEqual(below._get_active_angle_limits().ANGLE_RATE_LIMIT_DOWN[1], [5.0, SUBARU_UNWIND_RATE_LEVEL_VALUES[0], 0.15])
    self.assertEqual(above.mc_subaru_unwind_rate_level, SUBARU_UNWIND_RATE_LEVEL_MAX)
    self.assertEqual(above._get_active_angle_limits().ANGLE_RATE_LIMIT_DOWN[1], [5.0, SUBARU_UNWIND_RATE_LEVEL_VALUES[-1], 0.15])

  def test_subaru_unwind_rate_level_increases_only_unwind_delta(self):
    stock = self._build_controller(unwind_rate_level=0)
    level_10 = self._build_controller(unwind_rate_level=10)
    high = self._build_controller(unwind_rate_level=20)
    prev_angle = 100.0
    speed = 5.0
    measured_angle = 100.0

    stock_unwind = apply_std_steer_angle_limits(0.0, prev_angle, speed, measured_angle, True, stock._get_active_angle_limits())
    level_10_unwind = apply_std_steer_angle_limits(0.0, prev_angle, speed, measured_angle, True, level_10._get_active_angle_limits())
    high_unwind = apply_std_steer_angle_limits(0.0, prev_angle, speed, measured_angle, True, high._get_active_angle_limits())
    stock_windup = apply_std_steer_angle_limits(200.0, prev_angle, speed, measured_angle, True, stock._get_active_angle_limits())
    level_10_windup = apply_std_steer_angle_limits(200.0, prev_angle, speed, measured_angle, True, level_10._get_active_angle_limits())
    high_windup = apply_std_steer_angle_limits(200.0, prev_angle, speed, measured_angle, True, high._get_active_angle_limits())

    self.assertAlmostEqual(stock_unwind, 99.2)
    self.assertAlmostEqual(level_10_unwind, 96.0)
    self.assertAlmostEqual(high_unwind, 90.0)
    self.assertAlmostEqual(stock_windup, 100.8)
    self.assertAlmostEqual(level_10_windup, stock_windup)
    self.assertAlmostEqual(high_windup, stock_windup)

  def test_subaru_unwind_rate_level_keeps_windup_stock_at_every_level(self):
    base = self._build_controller(unwind_rate_level=0)._get_active_angle_limits()
    prev_angle = 100.0
    speed = 5.0
    measured_angle = 100.0
    stock_windup = apply_std_steer_angle_limits(200.0, prev_angle, speed, measured_angle, True, base)

    for level in range(SUBARU_UNWIND_RATE_LEVEL_MIN, SUBARU_UNWIND_RATE_LEVEL_MAX + 1):
      with self.subTest(level=level):
        controller = self._build_controller(unwind_rate_level=level)
        windup = apply_std_steer_angle_limits(200.0, prev_angle, speed, measured_angle, True, controller._get_active_angle_limits())
        self.assertAlmostEqual(windup, stock_windup)

  def test_subaru_turn_in_rate_level_zero_matches_stock_angle_limits(self):
    controller = self._build_controller(turn_in_rate_level=0)

    full_limits = controller._get_active_angle_limits(mads_only=False)
    mads_limits = controller._get_active_angle_limits(mads_only=True)

    self.assertEqual(full_limits.ANGLE_RATE_LIMIT_UP, controller.p.ANGLE_LIMITS.ANGLE_RATE_LIMIT_UP)
    self.assertEqual(full_limits.ANGLE_RATE_LIMIT_DOWN, controller.p.ANGLE_LIMITS.ANGLE_RATE_LIMIT_DOWN)
    self.assertEqual(mads_limits.ANGLE_RATE_LIMIT_UP, controller.p.ANGLE_LIMITS.ANGLE_RATE_LIMIT_UP)
    self.assertEqual(mads_limits.ANGLE_RATE_LIMIT_DOWN, controller.p.ANGLE_LIMITS.ANGLE_RATE_LIMIT_DOWN)

  def test_subaru_turn_in_rate_level_updates_only_mads_windup_limit(self):
    for level, expected_mid in enumerate(SUBARU_TURN_IN_RATE_LEVEL_VALUES):
      with self.subTest(level=level):
        controller = self._build_controller(turn_in_rate_level=level)

        full_limits = controller._get_active_angle_limits(mads_only=False)
        mads_limits = controller._get_active_angle_limits(mads_only=True)

        self.assertEqual(full_limits.ANGLE_RATE_LIMIT_UP, controller.p.ANGLE_LIMITS.ANGLE_RATE_LIMIT_UP)
        self.assertEqual(full_limits.ANGLE_RATE_LIMIT_DOWN, controller.p.ANGLE_LIMITS.ANGLE_RATE_LIMIT_DOWN)
        self.assertEqual(mads_limits.ANGLE_RATE_LIMIT_UP[0], controller.p.ANGLE_LIMITS.ANGLE_RATE_LIMIT_UP[0])
        self.assertEqual(mads_limits.ANGLE_RATE_LIMIT_UP[1], [5.0, expected_mid, 0.15])
        self.assertEqual(mads_limits.ANGLE_RATE_LIMIT_DOWN, controller.p.ANGLE_LIMITS.ANGLE_RATE_LIMIT_DOWN)

  def test_subaru_turn_in_rate_level_clamps_saved_values(self):
    below = self._build_controller(turn_in_rate_level=-5)
    above = self._build_controller(turn_in_rate_level=99)

    self.assertEqual(below.mc_subaru_turn_in_rate_level, SUBARU_TURN_IN_RATE_LEVEL_MIN)
    self.assertEqual(below._get_active_angle_limits(mads_only=True).ANGLE_RATE_LIMIT_UP[1], [5.0, SUBARU_TURN_IN_RATE_LEVEL_VALUES[0], 0.15])
    self.assertEqual(above.mc_subaru_turn_in_rate_level, SUBARU_TURN_IN_RATE_LEVEL_MAX)
    self.assertEqual(above._get_active_angle_limits(mads_only=True).ANGLE_RATE_LIMIT_UP[1], [5.0, SUBARU_TURN_IN_RATE_LEVEL_VALUES[-1], 0.15])

  def test_subaru_turn_in_and_unwind_rate_levels_are_independent(self):
    turn_in_only = self._build_controller(turn_in_rate_level=20, unwind_rate_level=0)
    unwind_only = self._build_controller(turn_in_rate_level=0, unwind_rate_level=20)
    both = self._build_controller(turn_in_rate_level=20, unwind_rate_level=20)

    self.assertEqual(turn_in_only._get_active_angle_limits(mads_only=True).ANGLE_RATE_LIMIT_UP[1], [5.0, 10.0, 0.15])
    self.assertEqual(turn_in_only._get_active_angle_limits(mads_only=True).ANGLE_RATE_LIMIT_DOWN, turn_in_only.p.ANGLE_LIMITS.ANGLE_RATE_LIMIT_DOWN)
    self.assertEqual(unwind_only._get_active_angle_limits(mads_only=True).ANGLE_RATE_LIMIT_UP, unwind_only.p.ANGLE_LIMITS.ANGLE_RATE_LIMIT_UP)
    self.assertEqual(unwind_only._get_active_angle_limits(mads_only=True).ANGLE_RATE_LIMIT_DOWN[1], [5.0, 10.0, 0.15])
    self.assertEqual(both._get_active_angle_limits(mads_only=True).ANGLE_RATE_LIMIT_UP[1], [5.0, 10.0, 0.15])
    self.assertEqual(both._get_active_angle_limits(mads_only=True).ANGLE_RATE_LIMIT_DOWN[1], [5.0, 10.0, 0.15])

  def test_subaru_turn_in_rate_level_changes_only_mads_turn_in_delta(self):
    stock = self._build_controller(turn_in_rate_level=0)
    high = self._build_controller(turn_in_rate_level=20)
    prev_angle = 100.0
    speed = 5.0
    measured_angle = 100.0

    stock_mads_windup = apply_std_steer_angle_limits(200.0, prev_angle, speed, measured_angle, True, stock._get_active_angle_limits(mads_only=True))
    high_mads_windup = apply_std_steer_angle_limits(200.0, prev_angle, speed, measured_angle, True, high._get_active_angle_limits(mads_only=True))
    high_full_windup = apply_std_steer_angle_limits(200.0, prev_angle, speed, measured_angle, True, high._get_active_angle_limits(mads_only=False))
    high_mads_unwind = apply_std_steer_angle_limits(0.0, prev_angle, speed, measured_angle, True, high._get_active_angle_limits(mads_only=True))

    self.assertAlmostEqual(stock_mads_windup, 100.8)
    self.assertAlmostEqual(high_mads_windup, 110.0)
    self.assertAlmostEqual(high_full_windup, stock_mads_windup)
    self.assertAlmostEqual(high_mads_unwind, 99.2)

  def test_manual_yield_torque_threshold_only_changes_when_enabled(self):
    disabled = self._build_carstate(torque_threshold_enabled=False, torque_threshold=MANUAL_YIELD_TORQUE_THRESHOLD_MAX)
    below_floor = self._build_carstate(torque_threshold_enabled=True, torque_threshold=MANUAL_YIELD_TORQUE_THRESHOLD_MIN - 25)
    at_floor = self._build_carstate(torque_threshold_enabled=True, torque_threshold=MANUAL_YIELD_TORQUE_THRESHOLD_MIN)
    above_stock = self._build_carstate(torque_threshold_enabled=True, torque_threshold=120)
    invalid_gap = self._build_carstate(torque_threshold_enabled=True, torque_threshold=175)
    above_ceiling = self._build_carstate(torque_threshold_enabled=True, torque_threshold=MANUAL_YIELD_TORQUE_THRESHOLD_MAX + 1)

    self.assertEqual(disabled._get_active_manual_yield_torque_threshold(), MANUAL_YIELD_TORQUE_THRESHOLD_DEFAULT)
    self.assertEqual(below_floor._get_active_manual_yield_torque_threshold(), MANUAL_YIELD_TORQUE_THRESHOLD_MIN)
    self.assertEqual(at_floor._get_active_manual_yield_torque_threshold(), MANUAL_YIELD_TORQUE_THRESHOLD_MIN)
    self.assertEqual(above_stock._get_active_manual_yield_torque_threshold(), 120)
    self.assertEqual(invalid_gap._get_active_manual_yield_torque_threshold(), 150)
    self.assertEqual(above_ceiling._get_active_manual_yield_torque_threshold(), MANUAL_YIELD_TORQUE_THRESHOLD_MAX)

  def test_manual_yield_torque_threshold_accepts_discrete_high_experimental_values(self):
    for threshold in (150, 200, 250, 300, 350, 400, 450, 500):
      with self.subTest(threshold=threshold):
        cs = self._build_carstate(torque_threshold_enabled=True, torque_threshold=threshold)
        self.assertEqual(cs._get_active_manual_yield_torque_threshold(), threshold)

    self.assertIn(500, MANUAL_YIELD_TORQUE_THRESHOLD_VALUES)
    self.assertNotIn(155, MANUAL_YIELD_TORQUE_THRESHOLD_VALUES)
    self.assertNotIn(175, MANUAL_YIELD_TORQUE_THRESHOLD_VALUES)

  def test_manual_yield_torque_threshold_always_uses_filtered_detection(self):
    self.params.put_bool("MCSubaruManualYieldFilteredDetectionEnabled", False)
    cs = self._build_carstate(torque_threshold_enabled=True, torque_threshold=MANUAL_YIELD_TORQUE_THRESHOLD_MIN)
    source = inspect.getsource(CarState._get_manual_yield_steering_pressed)

    self.assertTrue(cs.mc_subaru_manual_yield_filtered_detection_enabled)
    self.assertNotIn("return threshold_exceeded", source)
    self.assertIn("return self.update_steering_pressed(threshold_exceeded, MANUAL_YIELD_FILTERED_DETECTION_FRAMES)", source)
    self.assertFalse(cs._get_manual_yield_steering_pressed(MANUAL_YIELD_TORQUE_THRESHOLD_MAX))
    for _ in range(5):
      pressed = cs._get_manual_yield_steering_pressed(MANUAL_YIELD_TORQUE_THRESHOLD_MAX)
    self.assertTrue(pressed)

  def test_outback_2023_angle_steering_route_still_present(self):
    route = next(route for route in routes if route.platform == CAR.SUBARU_OUTBACK_2023)
    self.assertEqual(route.platform, CAR.SUBARU_OUTBACK_2023)

  def test_crosstrek_2025_fw_versions_still_present(self):
    self.assertIn(CAR.SUBARU_CROSSTREK_2025, FW_VERSIONS)


if __name__ == "__main__":
  unittest.main()
