from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
TICI_SUBARU = REPO_ROOT / "selfdrive/ui/sunnypilot/layouts/settings/vehicle/brands/subaru.py"
TICI_MC_CUSTOM = REPO_ROOT / "selfdrive/ui/sunnypilot/layouts/settings/mc_custom.py"
TICI_FORD = REPO_ROOT / "selfdrive/ui/sunnypilot/layouts/settings/vehicle/brands/ford.py"
MICI_VEHICLE = REPO_ROOT / "selfdrive/ui/bp/mici/layouts/settings/vehicle_mici.py"
MICI_SUBARU = REPO_ROOT / "selfdrive/ui/sunnypilot/mici/layouts/subaru.py"
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


def test_tici_subaru_brand_page_only_hosts_stop_and_go_controls():
  source = _read(TICI_SUBARU)
  assert "class SubaruSettings(BrandSettings):" in source
  assert 'param="SubaruStopAndGo"' in source
  assert 'param="SubaruStopAndGoManualParkingBrake"' in source
  assert "MCSubaruMatchVehicleSpeedometer" not in source
  assert 'param="MCSubaruAdvancedTuning"' not in source
  assert "Manual Yield Resume" not in source


def test_tici_subaru_brand_page_restores_stop_and_go_platform_logic():
  source = _read(TICI_SUBARU)
  assert "from opendbc.car.subaru.values import CAR, SubaruFlags" in source
  assert "platform = bundle.get(\"platform\")" in source
  assert "config = CAR[platform].config" in source
  assert "self.has_stop_and_go = not (config.flags & (SubaruFlags.GLOBAL_GEN2 | SubaruFlags.HYBRID))" in source
  assert "toggle.action_item.set_enabled(self.has_stop_and_go and ui_state.is_offroad())" in source
  assert 'Enable "Always Offroad" in Device panel, or turn vehicle off to toggle.' in source
  assert "strict=True" in source


def test_tici_mc_custom_subaru_lateral_menu_matches_requested_controls():
  source = _read(TICI_MC_CUSTOM)
  assert 'param="MCSubaruMatchVehicleSpeedometer"' in source
  assert 'param="MCSubaruAdvancedTuning"' in source
  assert '"Show Lateral Tuning Settings"' in source
  assert 'param="MCSubaruShowAdvancedDevControls"' in source
  assert '"Show Advanced Dev Controls"' in source

  assert 'param="MCSubaruManualYieldTorqueThresholdEnabled"' in source
  assert 'param="MCSubaruManualYieldTorqueThreshold"' in source
  assert '"Custom Yield Torque"' in source
  assert '"Manual Yield Torque Threshold"' in source
  assert 'param="MCSubaruManualYieldResumeSoftnessEnabled"' in source
  assert 'param="MCSubaruManualYieldResumeSoftness"' in source
  assert '"Custom Resume Softness"' in source
  assert 'param="MCSubaruManualYieldReleaseGuardEnabled"' in source
  assert 'param="MCSubaruManualYieldReleaseGuardLevel"' in source
  assert '"Manual Yield Release Guard"' in source
  assert 'param="MCSubaruManualSteeringSoftHoldLevel"' in source
  assert '"Manual Steering Hold"' in source
  assert "MANUAL_STEERING_SOFT_HOLD_LABELS = [\"Off\", \"L1 Light\", \"L2 Medium\", \"L3 Strong\"]" in source
  assert 'param="MCSubaruSoftCaptureEnabled"' in source
  assert 'param="MCSubaruSoftCaptureLevel"' in source
  assert '"Soft-Capture Engage Blend"' in source
  assert '"Soft-Capture Strength"' in source
  assert (
    "self._subaru_soft_capture,\n"
    "      self._subaru_soft_capture_strength,\n"
    "      self._manual_steering_soft_hold,\n"
    "      self._manual_yield_release_guard_enabled,\n"
    "      self._manual_yield_release_guard_level,\n"
    "      self._subaru_advanced_dev_controls,"
  ) in source

  assert '"MAX Steering Experiment"' in source
  assert '"MCSubaruMaxSteeringExperiment"' in source
  assert 'param="MCSubaruMaxSteeringExperiment"' in source
  assert "ConfirmDialog(" not in source
  assert "MAX_STEERING_EXPERIMENT_PARAM_TYPES" in source
  assert "MAX_STEERING_EXPERIMENT_DEFAULTS" in source
  assert "MAX_STEERING_EXPERIMENT_VALUES" in source
  assert '"MCSubaruManualYieldTorqueThreshold": "int"' in source
  assert '"MCSubaruMadsTighterTurnsEnabled": True' in source
  assert '"MCSubaruMadsMaxSteeringAngle": 199' in source
  assert '"MCSubaruTurnInRateLevel": 20' in source
  assert '"MCSubaruUnwindRateLevel": 20' in source
  assert '"MCSubaruManualYieldTorqueThresholdEnabled": True' in source
  assert '"MCSubaruManualYieldTorqueThreshold": 500' in source
  assert '"MCSubaruManualYieldTorqueThreshold": ("int", 80)' not in source
  assert "self._params.put(key, value)" in source
  assert "self._params.put(key, str(value))" not in source
  assert 'Turning this off directly restores the default steering' in source
  assert 'keeping your saved torque threshold value' in source
  assert 'Controlled testing only' in source
  assert 'greatly reduces normal manual-yield behavior' in source
  assert 'intentionally drop LKAS request' in source

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
  assert '"Advanced Tuning"' not in source


def test_ford_brand_page_does_not_gain_subaru_controls():
  source = _read(TICI_FORD)
  assert "MCSubaru" not in source
  assert "SubaruStopAndGo" not in source
  assert "Manual Yield Resume" not in source
  assert "Advanced Tuning" not in source
  assert "Show Lateral Tuning Settings" not in source


def test_mici_vehicle_menu_adds_subaru_entry_only_for_subaru_brand():
  source = _read(MICI_VEHICLE)
  assert "from openpilot.selfdrive.ui.sunnypilot.mici.layouts.subaru import SubaruLayoutMici" in source
  assert 'self._btn_subaru = BigButtonBP(tr("subaru settings")' in source
  assert 'self._btn_subaru.set_click_callback(self._on_subaru_settings)' in source
  assert 'is_subaru = get_vehicle_brand() == "subaru"' in source
  assert "self._btn_subaru.set_visible(is_subaru)" in source
  assert "gui_app.push_widget(SubaruLayoutMici(back_callback=gui_app.pop_widget))" in source


def test_mici_subaru_layout_exact_lateral_menu_and_max_experiment():
  source = _read(MICI_SUBARU)
  assert 'GreyBigButton("stop and\\ngo")' in source
  assert 'GreyBigButton("lateral\\ntuning")' in source
  assert 'BigParamControl("stop and go\\n(beta)", "SubaruStopAndGo")' in source
  assert '"MCSubaruMatchVehicleSpeedometer"' in source
  assert 'BigParamControl("show lateral\\ntuning settings", "MCSubaruAdvancedTuning")' in source
  assert 'BigParamControl("custom yield\\ntorque", "MCSubaruManualYieldTorqueThresholdEnabled")' in source
  assert 'BigButton("manual yield\\ntorque")' in source
  assert 'BigParamControl("custom resume\\nsoftness", "MCSubaruManualYieldResumeSoftnessEnabled")' in source
  assert 'BigButton("manual yield\\nresume softness")' in source
  assert 'BigParamControl("manual yield\\nrelease guard", "MCSubaruManualYieldReleaseGuardEnabled")' in source
  assert 'BigButton("release guard\\nstrength")' in source
  assert 'BigButton("manual steering\\nhold")' in source
  assert '"MCSubaruManualSteeringSoftHoldLevel"' in source
  assert 'MANUAL_STEERING_SOFT_HOLD_LABELS = ["Off", "L1 Light", "L2 Medium", "L3 Strong"]' in source
  assert 'BigParamControl("soft-capture\\nengage blend", "MCSubaruSoftCaptureEnabled")' in source
  assert 'BigButton("soft-capture\\nstrength")' in source
  assert (
    "self._subaru_soft_capture_toggle,\n"
    "      self._subaru_soft_capture_strength_btn,\n"
    "      self._manual_steering_soft_hold_btn,\n"
    "      self._manual_yield_release_guard_toggle,\n"
    "      self._manual_yield_release_guard_btn,\n"
    "      self._subaru_advanced_dev_controls_toggle,"
  ) in source
  assert 'BigParamControl("show advanced\\ndev controls", "MCSubaruShowAdvancedDevControls")' in source
  assert 'BigToggle(' in source
  assert '"MAX steering\\nexperiment"' in source
  assert 'BigConfirmationDialogV2(' not in source
  assert 'slide to enable' not in source
  assert 'MAX_STEERING_EXPERIMENT_PARAM_TYPES' in source
  assert 'MAX_STEERING_EXPERIMENT_DEFAULTS' in source
  assert '"MCSubaruManualYieldTorqueThreshold": "int"' in source
  assert '"MCSubaruManualYieldTorqueThreshold": ("int", 80)' not in source
  assert '"MCSubaruMadsMaxSteeringAngle": 199' in source
  assert '"MCSubaruTurnInRateLevel": 20' in source
  assert '"MCSubaruUnwindRateLevel": 20' in source
  assert '"MCSubaruManualYieldTorqueThreshold": 500' in source
  assert 'ui_state.params.put(key, value)' in source
  assert 'ui_state.params.put(key, str(value))' not in source

  assert 'BigParamControl("filtered yield\\ndetection", "MCSubaruManualYieldFilteredDetectionEnabled")' not in source
  assert 'BigParamControl("advanced\\ntuning", "MCSubaruAdvancedTuning")' not in source
  assert 'BigParamControl("tighter MADS\\nturns", "MCSubaruMadsTighterTurnsEnabled")' not in source
  assert 'BigButton("MADS steering\\nangle cap")' not in source
  assert 'BigButton("turn-in rate\\nlevel")' not in source
  assert 'BigButton("unwind rate\\nlevel")' not in source
  for param in REMOVED_SUBARU_TUNING_PARAMS:
    assert param not in source


def test_mici_subaru_layout_visibility_matches_requested_menu_hierarchy():
  source = _read(MICI_SUBARU)
  assert "def _set_advanced_tuning_visibility(self, enabled: bool, advanced_dev_enabled: bool) -> None:" in source
  assert 'self._manual_yield_torque_threshold_toggle.set_visible(enabled)' in source
  assert 'self._manual_yield_torque_threshold_btn.set_visible(enabled)' in source
  assert 'self._manual_yield_resume_softness_toggle.set_visible(enabled)' in source
  assert 'self._manual_yield_resume_softness_btn.set_visible(enabled)' in source
  assert 'self._manual_yield_release_guard_toggle.set_visible(enabled)' in source
  assert 'self._manual_yield_release_guard_btn.set_visible(enabled)' in source
  assert 'self._manual_steering_soft_hold_btn.set_visible(enabled)' in source
  assert 'self._subaru_soft_capture_toggle.set_visible(enabled)' in source
  assert 'self._subaru_soft_capture_strength_btn.set_visible(enabled)' in source
  assert 'self._subaru_advanced_dev_controls_toggle.set_visible(enabled)' in source
  assert 'self._subaru_max_steering_experiment_toggle.set_visible(enabled and advanced_dev_enabled)' in source
  assert 'self._manual_yield_filtered_detection_toggle.set_visible(enabled)' not in source
  assert 'self._subaru_mads_tighter_turns_toggle.set_visible' not in source
  assert 'self._subaru_mads_steering_angle_cap_btn.set_visible' not in source
  assert 'self._subaru_turn_in_rate_level_btn.set_visible' not in source
  assert 'self._subaru_unwind_rate_level_btn.set_visible' not in source


def test_subaru_params_and_metadata_match_lateral_menu_rework():
  params_source = _read(PARAMS_KEYS)
  metadata_source = _read(PARAMS_METADATA)
  assert '{"MCSubaruAdvancedTuning", {PERSISTENT | BACKUP, BOOL, "0"}}' in params_source
  assert '{"MCSubaruShowAdvancedDevControls", {PERSISTENT | BACKUP, BOOL, "0"}}' in params_source
  assert '{"MCSubaruMaxSteeringExperiment", {PERSISTENT | BACKUP, BOOL, "0"}}' in params_source
  assert "MCSubaruMaxSteeringExperimentSnapshot" not in params_source
  assert '{"MCSubaruManualYieldFilteredDetectionEnabled", {PERSISTENT | BACKUP, BOOL, "1"}}' in params_source
  assert '{"MCSubaruManualYieldTorqueThreshold", {PERSISTENT | BACKUP, INT, "80"}}' in params_source
  assert '{"MCSubaruManualYieldResumeSoftness", {PERSISTENT | BACKUP, INT, "4"}}' in params_source
  assert '{"MCSubaruManualYieldReleaseGuardLevel", {PERSISTENT | BACKUP, INT, "2"}}' in params_source
  assert '{"MCSubaruManualSteeringSoftHoldLevel", {PERSISTENT | BACKUP, INT, "0"}}' in params_source
  assert '{"MCSubaruSoftCaptureLevel", {PERSISTENT | BACKUP, INT, "3"}}' in params_source
  assert '{"MCSubaruMadsMaxSteeringAngle", {PERSISTENT | BACKUP, INT, "180"}}' in params_source
  assert '{"MCSubaruUnwindRateLevel", {PERSISTENT | BACKUP, INT, "0"}}' in params_source
  assert '{"MCSubaruTurnInRateLevel", {PERSISTENT | BACKUP, INT, "0"}}' in params_source

  assert '"MCSubaruAdvancedTuning"' in metadata_source
  assert '"title": "Show Lateral Tuning Settings"' in metadata_source
  assert '"MCSubaruShowAdvancedDevControls"' in metadata_source
  assert '"title": "Show Advanced Dev Controls"' in metadata_source
  assert '"MCSubaruMaxSteeringExperiment"' in metadata_source
  assert '"title": "MAX Steering Experiment"' in metadata_source
  assert 'Turning it off directly restores the default steering test values' in metadata_source
  assert 'keeping your saved torque threshold value' in metadata_source
  assert 'greatly reduces normal manual-yield behavior' in metadata_source
  assert 'intentionally drop LKAS request' in metadata_source
  assert '"MCSubaruManualSteeringSoftHoldLevel"' in metadata_source
  assert '"title": "Manual Steering Hold"' in metadata_source
  assert '"label": "L1 Light"' in metadata_source
  assert 'L1 is the lightest adjustment and still needs the most hand pressure' in metadata_source
  assert 'L3 is the strongest hold and accepts the lightest hand pressure for longer' in metadata_source
  assert 'does not make initial manual-yield easier' in metadata_source
  assert 'MADS steering angle cap to 199' in metadata_source
  assert '180 deg is the default turning request limit' in metadata_source
  assert 'turn-in rate to L20 500 deg/s' in metadata_source
  assert 'manual yield torque threshold to 500' in metadata_source
  assert '"title": "Filtered Yield Detection"' not in metadata_source
  assert 'Ford-style filtering for Subaru manual-yield detection' not in metadata_source
  for param in REMOVED_SUBARU_TUNING_PARAMS:
    assert param not in params_source
    assert param not in metadata_source
