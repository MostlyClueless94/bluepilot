from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
TICI_SETTINGS = REPO_ROOT / "selfdrive/ui/sunnypilot/layouts/settings/settings.py"
TICI_BLUEPILOT = REPO_ROOT / "selfdrive/ui/bp/layouts/settings/bluepilot.py"
TICI_VEHICLE_BP = REPO_ROOT / "selfdrive/ui/bp/layouts/settings/vehicle_bp.py"
MICI_BLUEPILOT = REPO_ROOT / "selfdrive/ui/bp/mici/layouts/settings/bluepilot.py"
MICI_VEHICLE = REPO_ROOT / "selfdrive/ui/bp/mici/layouts/settings/vehicle_mici.py"


def _read(path: Path) -> str:
  return path.read_text(encoding="utf-8")


def test_tici_bluepilot_menu_no_longer_owns_vehicle_profiles():
  source = _read(TICI_BLUEPILOT)

  assert "Vehicle Profiles" not in source
  assert "Vehicle Profile" not in source
  assert "Relearn Current Profile" not in source
  assert "Reset Current Profile" not in source
  assert "snapshot_active_profile" not in source
  assert "reset_active_profile" not in source


def test_tici_vehicle_tab_uses_bluepilot_profile_extension():
  settings_source = _read(TICI_SETTINGS)
  vehicle_source = _read(TICI_VEHICLE_BP)

  assert "from openpilot.selfdrive.ui.bp.layouts.settings.vehicle_bp import VehicleLayoutBP as VehicleLayout" in settings_source
  assert "class VehicleLayoutBP(VehicleLayout):" in vehicle_source
  assert "snapshot_active_profile(self._params)" in vehicle_source
  assert "reset_active_profile(self._params)" in vehicle_source
  assert "self._vehicle_profile_status" in vehicle_source
  assert "self._vehicle_profile_relearn_btn" in vehicle_source
  assert "self._vehicle_profile_reset_btn" in vehicle_source
  assert vehicle_source.index("self._vehicle_profile_status") < vehicle_source.index("] + self._brand_items")


def test_mici_bluepilot_menu_no_longer_owns_vehicle_profiles():
  source = _read(MICI_BLUEPILOT)

  assert "vehicle_profile_status" not in source
  assert "vehicle_profile_relearn" not in source
  assert "vehicle_profile_reset" not in source
  assert "snapshot_active_profile" not in source
  assert "reset_active_profile" not in source


def test_mici_vehicle_menu_owns_vehicle_profiles():
  source = _read(MICI_VEHICLE)

  assert "from bluepilot.params.vehicle_profiles import reset_active_profile, snapshot_active_profile" in source
  assert 'self._vehicle_profile_status = BigButtonBP(' in source
  assert 'tr("vehicle profile")' in source
  assert 'tr("relearn current profile")' in source
  assert 'tr("reset current profile")' in source
  assert "snapshot_active_profile(self._params)" in source
  assert "reset_active_profile(self._params)" in source
  assert (
    "self._btn_current,\n"
    "      self._vehicle_profile_status,\n"
    "      self._vehicle_profile_relearn,\n"
    "      self._vehicle_profile_reset,\n"
    "      self._btn_clear,"
  ) in source
