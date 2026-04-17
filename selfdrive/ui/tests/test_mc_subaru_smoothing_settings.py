from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
MC_CUSTOM = REPO_ROOT / "selfdrive/ui/sunnypilot/layouts/settings/mc_custom.py"
PARAMS_KEYS = REPO_ROOT / "common/params_keys.h"
PARAMS_METADATA = REPO_ROOT / "sunnypilot/sunnylink/params_metadata.json"

REMOVED_SUBARU_TUNING_PARAMS = (
  "MCSubaruSmoothingTune",
  "MCSubaruSmoothingStrength",
  "MCSubaruCenterDampingStrength",
  "MCSubaruManualYieldResumeSpeedEnabled",
  "MCSubaruManualYieldResumeSpeed",
)


def _read(path: Path) -> str:
  return path.read_text(encoding="utf-8")


def test_mc_custom_hosts_exact_subaru_lateral_tuning_controls():
  source = _read(MC_CUSTOM)
  assert 'param="DynamicPathColor"' in source
  assert 'param="CustomModelPathColor"' in source
  assert 'param="MCShowVehicleBrakeStatus"' in source
  assert 'SectionHeader(tr("Subaru"))' in source
  assert 'param="MCSubaruMatchVehicleSpeedometer"' in source

  assert 'param="MCSubaruAdvancedTuning"' in source
  assert '"Show Lateral Tuning Settings"' in source
  assert 'param="MCSubaruShowAdvancedDevControls"' in source
  assert '"Show Advanced Dev Controls"' in source
  assert 'param="MCSubaruManualYieldTorqueThresholdEnabled"' in source
  assert 'param="MCSubaruManualYieldTorqueThreshold"' in source
  assert 'param="MCSubaruManualYieldResumeSoftnessEnabled"' in source
  assert 'param="MCSubaruManualYieldResumeSoftness"' in source
  assert 'param="MCSubaruManualYieldReleaseGuardEnabled"' in source
  assert 'param="MCSubaruManualYieldReleaseGuardLevel"' in source
  assert 'param="MCSubaruManualSteeringSoftHoldLevel"' in source
  assert '"Manual Steering Hold"' in source
  assert "MANUAL_STEERING_SOFT_HOLD_LABELS = [\"Off\", \"L1 Light\", \"L2 Medium\", \"L3 Strong\"]" in source
  assert 'param="MCSubaruSoftCaptureEnabled"' in source
  assert 'param="MCSubaruSoftCaptureLevel"' in source
  assert '"MAX Steering Experiment"' in source
  assert '"MCSubaruMaxSteeringExperiment"' in source
  assert 'param="MCSubaruMaxSteeringExperiment"' in source
  assert (
    "self._subaru_soft_capture,\n"
    "      self._subaru_soft_capture_strength,\n"
    "      self._manual_steering_soft_hold,\n"
    "      self._manual_yield_release_guard_enabled,\n"
    "      self._manual_yield_release_guard_level,\n"
    "      self._subaru_advanced_dev_controls,"
  ) in source

  assert 'param="MCSubaruManualYieldFilteredDetectionEnabled"' not in source
  assert '"Filtered Yield Detection"' not in source
  assert 'param="MCSubaruMadsTighterTurnsEnabled"' not in source
  assert 'param="MCSubaruMadsMaxSteeringAngle"' not in source
  assert 'param="MCSubaruTurnInRateLevel"' not in source
  assert 'param="MCSubaruUnwindRateLevel"' not in source
  assert '"Tighter MADS Turns"' not in source
  assert '"MADS Steering Angle Cap"' not in source
  assert '"Turn-In Rate"' not in source
  assert '"Unwind Rate"' not in source
  for param in REMOVED_SUBARU_TUNING_PARAMS:
    assert param not in source


def test_mc_custom_preserves_lateral_menu_hierarchy_and_simple_max_toggle_logic():
  source = _read(MC_CUSTOM)
  assert 'def _set_subaru_section_visibility(self, advanced_tuning_enabled: bool, advanced_dev_controls_enabled: bool) -> None:' in source
  assert 'self._manual_yield_torque_threshold_enabled.set_visible(advanced_tuning_enabled)' in source
  assert 'self._manual_yield_torque_threshold.set_visible(advanced_tuning_enabled)' in source
  assert 'self._manual_yield_resume_softness_enabled.set_visible(advanced_tuning_enabled)' in source
  assert 'self._manual_yield_resume_softness.set_visible(advanced_tuning_enabled)' in source
  assert 'self._manual_yield_release_guard_enabled.set_visible(advanced_tuning_enabled)' in source
  assert 'self._manual_yield_release_guard_level.set_visible(advanced_tuning_enabled)' in source
  assert 'self._manual_steering_soft_hold.set_visible(advanced_tuning_enabled)' in source
  assert 'self._subaru_soft_capture.set_visible(advanced_tuning_enabled)' in source
  assert 'self._subaru_soft_capture_strength.set_visible(advanced_tuning_enabled)' in source
  assert 'self._subaru_advanced_dev_controls.set_visible(advanced_tuning_enabled)' in source
  assert 'self._subaru_max_steering_experiment.set_visible(advanced_tuning_enabled and advanced_dev_controls_enabled)' in source
  assert 'self._manual_yield_filtered_detection.set_visible(advanced_tuning_enabled)' not in source
  assert 'self._subaru_mads_tighter_turns.set_visible' not in source
  assert 'self._subaru_mads_steering_angle_cap.set_visible' not in source
  assert 'self._subaru_turn_in_rate_level.set_visible' not in source
  assert 'self._subaru_unwind_rate_level.set_visible' not in source

  assert 'def _enable_max_steering_experiment(self) -> None:' in source
  assert 'def _disable_max_steering_experiment(self) -> None:' in source
  assert 'MAX_STEERING_EXPERIMENT_PARAM_TYPES' in source
  assert 'MAX_STEERING_EXPERIMENT_DEFAULTS' in source
  assert 'MAX_STEERING_EXPERIMENT_SNAPSHOT' not in source
  assert 'ConfirmDialog(' not in source
  assert '"MCSubaruManualYieldTorqueThreshold": "int"' in source
  assert '"MCSubaruMadsTighterTurnsEnabled": True' in source
  assert '"MCSubaruMadsMaxSteeringAngle": 199' in source
  assert '"MCSubaruTurnInRateLevel": 20' in source
  assert '"MCSubaruUnwindRateLevel": 20' in source
  assert '"MCSubaruManualYieldTorqueThreshold": 500' in source
  assert '"MCSubaruManualYieldTorqueThreshold": ("int", 80)' not in source
  assert 'self._params.put(key, value)' in source
  assert 'self._params.put(key, str(value))' not in source


def test_params_keys_register_subaru_lateral_menu_rework_defaults():
  source = _read(PARAMS_KEYS)
  assert '{"MCSubaruAdvancedTuning", {PERSISTENT | BACKUP, BOOL, "0"}}' in source
  assert '{"MCSubaruShowAdvancedDevControls", {PERSISTENT | BACKUP, BOOL, "0"}}' in source
  assert '{"MCSubaruMaxSteeringExperiment", {PERSISTENT | BACKUP, BOOL, "0"}}' in source
  assert "MCSubaruMaxSteeringExperimentSnapshot" not in source
  assert '{"MCSubaruManualYieldFilteredDetectionEnabled", {PERSISTENT | BACKUP, BOOL, "1"}}' in source
  assert '{"MCSubaruManualYieldTorqueThreshold", {PERSISTENT | BACKUP, INT, "80"}}' in source
  assert '{"MCSubaruManualYieldResumeSoftness", {PERSISTENT | BACKUP, INT, "4"}}' in source
  assert '{"MCSubaruManualYieldReleaseGuardLevel", {PERSISTENT | BACKUP, INT, "2"}}' in source
  assert '{"MCSubaruManualSteeringSoftHoldLevel", {PERSISTENT | BACKUP, INT, "0"}}' in source
  assert '{"MCSubaruSoftCaptureLevel", {PERSISTENT | BACKUP, INT, "3"}}' in source
  assert '{"MCSubaruMadsMaxSteeringAngle", {PERSISTENT | BACKUP, INT, "180"}}' in source
  assert '{"SubaruStopAndGo", {PERSISTENT | BACKUP, BOOL, "0"}}' in source
  assert '{"SubaruStopAndGoManualParkingBrake", {PERSISTENT | BACKUP, BOOL, "0"}}' in source
  for param in REMOVED_SUBARU_TUNING_PARAMS:
    assert param not in source


def test_params_metadata_describes_subaru_lateral_menu_rework():
  source = _read(PARAMS_METADATA)
  assert '"MCSubaruAdvancedTuning"' in source
  assert '"title": "Show Lateral Tuning Settings"' in source
  assert '"MCSubaruShowAdvancedDevControls"' in source
  assert '"title": "Show Advanced Dev Controls"' in source
  assert '"MCSubaruMaxSteeringExperiment"' in source
  assert '"title": "MAX Steering Experiment"' in source
  assert 'MADS steering angle cap to 199' in source
  assert 'Turning it off directly restores the default steering test values' in source
  assert 'keeping your saved torque threshold value' in source
  assert '180 deg is the default turning request limit' in source
  assert 'turn-in rate to L20 500 deg/s' in source
  assert 'unwind rate to L20 500 deg/s' in source
  assert 'manual yield torque threshold to 500' in source
  assert 'may produce LKAS faults' in source
  assert '"title": "Filtered Yield Detection"' not in source
  assert 'Ford-style filtering for Subaru manual-yield detection' not in source
  assert '"MCSubaruManualYieldTorqueThresholdEnabled"' in source
  assert '"title": "Custom Yield Torque"' in source
  assert '"MCSubaruManualYieldResumeSoftnessEnabled"' in source
  assert '"title": "Custom Resume Softness"' in source
  assert '"MCSubaruManualYieldReleaseGuardEnabled"' in source
  assert '"title": "Manual Yield Release Guard"' in source
  assert '"MCSubaruManualSteeringSoftHoldLevel"' in source
  assert '"title": "Manual Steering Hold"' in source
  assert '"label": "L3 Strong"' in source
  assert 'L1 is the lightest adjustment and still needs the most hand pressure' in source
  assert 'L3 is the strongest hold and accepts the lightest hand pressure for longer' in source
  assert 'does not make initial manual-yield easier' in source
  assert '"MCSubaruSoftCaptureEnabled"' in source
  assert '"title": "Soft-Capture Engage Blend"' in source
  assert '"MCSubaruSoftCaptureLevel"' in source
  assert '"title": "Soft-Capture Strength"' in source
  for param in REMOVED_SUBARU_TUNING_PARAMS:
    assert param not in source
