"""
BluePilot: MICI vehicle menu — current vehicle, clear manual fingerprint, select vehicle.

Uses horizontal NavScroller (same pattern as WiFi / preferred network).
"""

from __future__ import annotations

import os
from collections.abc import Callable

from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.ui.bp.mici.widgets.button_bp import BigButtonBP
from openpilot.selfdrive.ui.bp.mici.widgets.vehicle_select_mici import (
  VehicleMakeSelectMici,
  load_car_platforms,
)
from openpilot.selfdrive.ui.sunnypilot.mici.layouts.subaru import SubaruLayoutMici
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets import DialogResult
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog
from openpilot.system.ui.widgets.scroller import NavScroller
from bluepilot.params.vehicle_profiles import reset_active_profile, snapshot_active_profile

CAR_LIST_JSON = os.path.join(BASEDIR, "sunnypilot", "selfdrive", "car", "car_list.json")


def _truncate(s: str, max_len: int = 36) -> str:
  s = s.strip()
  if len(s) <= max_len:
    return s
  return s[: max_len - 3] + "..."


def get_vehicle_status_text() -> tuple[str, str]:
  """
  Returns (title_line, value_line) for the current vehicle display.
  Mirrors TICI PlatformSelector.refresh() semantics.
  """
  if bundle := ui_state.params.get("CarPlatformBundle"):
    name = bundle.get("name", "")
    if isinstance(name, bytes):
      name = name.decode("utf-8", errors="replace")
    return (tr("manual selection"), _truncate(str(name)))
  if ui_state.CP is not None and ui_state.CP.carFingerprint != "MOCK":
    fp = str(ui_state.CP.carFingerprint)
    return (tr("auto fingerprint"), _truncate(fp))
  return (tr("vehicle"), tr("unrecognized"))


def get_vehicle_brand() -> str:
  if bundle := ui_state.params.get("CarPlatformBundle"):
    brand = bundle.get("brand", "")
    if isinstance(brand, bytes):
      brand = brand.decode("utf-8", errors="replace")
    return str(brand)
  if ui_state.CP is not None and ui_state.CP.carFingerprint != "MOCK":
    return str(ui_state.CP.brand)
  return ""


class VehicleLayoutMici(NavScroller):
  """Three-button horizontal strip: current vehicle | clear | select."""

  def __init__(self, back_callback: Callable[[], None]):
    super().__init__()
    self.set_back_callback(back_callback)
    self._params = Params()
    self._platforms: dict = {}
    try:
      self._platforms = load_car_platforms()
    except OSError as e:
      from openpilot.common.swaglog import cloudlog

      cloudlog.error(f"MICI vehicle: could not load {CAR_LIST_JSON}: {e}")

    self._btn_current = BigButtonBP(tr("current vehicle"), "", "../../sunnypilot/selfdrive/assets/offroad/icon_vehicle.png")
    self._vehicle_profile_status = BigButtonBP(
      tr("vehicle profile"),
      "",
      "icons_mici/settings/device/info.png",
      value_size=24
    )
    self._vehicle_profile_relearn = BigButtonBP(
      tr("relearn current profile"),
      "",
      "icons_mici/settings/device/info.png"
    )
    self._vehicle_profile_reset = BigButtonBP(
      tr("reset current profile"),
      "",
      "icons_mici/settings/device/reboot.png"
    )
    self._btn_clear = BigButtonBP(tr("clear vehicle"), "", "../../sunnypilot/selfdrive/assets/offroad/icon_vehicle.png")
    self._btn_select = BigButtonBP(tr("select vehicle"), "", "../../sunnypilot/selfdrive/assets/offroad/icon_vehicle.png")
    self._btn_subaru = BigButtonBP(tr("subaru settings"), "", "../../sunnypilot/selfdrive/assets/offroad/icon_vehicle.png")

    self._btn_current.set_enabled(False)
    self._vehicle_profile_relearn.set_click_callback(self._relearn_vehicle_profile)
    self._vehicle_profile_reset.set_click_callback(self._reset_vehicle_profile)
    self._btn_clear.set_click_callback(self._on_clear)
    self._btn_select.set_click_callback(self._on_select)
    self._btn_subaru.set_click_callback(self._on_subaru_settings)

    self._scroller.add_widgets([
      self._btn_current,
      self._vehicle_profile_status,
      self._vehicle_profile_relearn,
      self._vehicle_profile_reset,
      self._btn_clear,
      self._btn_select,
      self._btn_subaru,
    ])

  def show_event(self):
    super().show_event()
    ui_state.update_params()
    self._refresh_display()

  def _update_state(self):
    super()._update_state()
    ui_state.update_params()
    self._refresh_display()

  def _refresh_display(self):
    t, v = get_vehicle_status_text()
    self._btn_current.set_text(t)
    self._btn_current.set_value(v)
    has_manual = bool(ui_state.params.get("CarPlatformBundle"))
    is_subaru = get_vehicle_brand() == "subaru"
    self._btn_clear.set_enabled(has_manual)
    self._btn_select.set_enabled(len(self._platforms) > 0)
    self._btn_subaru.set_visible(is_subaru)
    self._btn_subaru.set_enabled(is_subaru)
    self._update_vehicle_profile_controls()

  def _get_vehicle_profile_value(self) -> str:
    try:
      active = self._params.get("BPVehicleProfileActive") or ""
    except Exception:
      active = ""
    if isinstance(active, bytes):
      active = active.decode("utf-8", errors="replace")
    active = str(active)
    return active if active else tr("none")

  def _get_vehicle_profile_description(self) -> str:
    try:
      status = self._params.get("BPVehicleProfileStatus") or {}
    except Exception:
      status = {}
    if not isinstance(status, dict):
      return tr("waiting")
    if status.get("restartRequired"):
      return tr("restart required")
    if status.get("modelFallback"):
      return tr("default model used")
    action = str(status.get("action") or "").lower()
    changed = status.get("changed")
    if changed:
      return f"{action}: {changed} changed"
    return action or tr("ready")

  def _update_vehicle_profile_controls(self):
    try:
      active = bool(self._params.get("BPVehicleProfileActive"))
    except Exception:
      active = False
    self._vehicle_profile_status.set_value(f"{self._get_vehicle_profile_value()} / {self._get_vehicle_profile_description()}")
    self._vehicle_profile_relearn.set_enabled(active)
    self._vehicle_profile_reset.set_enabled(active)

  def _relearn_vehicle_profile(self):
    def handle_confirm(result: DialogResult):
      if result == DialogResult.CONFIRM:
        snapshot_active_profile(self._params)
        self._update_vehicle_profile_controls()
        cloudlog.info("BluePilot MICI: relearned active vehicle profile")

    gui_app.push_widget(ConfirmDialog(
      tr("Save current model and settings into this vehicle profile?"),
      tr("Relearn Profile"),
      callback=handle_confirm
    ))

  def _reset_vehicle_profile(self):
    def handle_confirm(result: DialogResult):
      if result == DialogResult.CONFIRM:
        reset_active_profile(self._params)
        self._update_vehicle_profile_controls()
        cloudlog.info("BluePilot MICI: reset active vehicle profile")

    gui_app.push_widget(ConfirmDialog(
      tr("Forget this vehicle profile so it can be learned again?"),
      tr("Reset Profile"),
      callback=handle_confirm
    ))

  def _on_clear(self):
    if ui_state.params.get("CarPlatformBundle"):
      ui_state.params.remove("CarPlatformBundle")
    self._refresh_display()

  def _on_select(self):
    if not self._platforms:
      return

    def on_complete():
      self._refresh_display()

    gui_app.push_widget(VehicleMakeSelectMici(self._platforms, on_stack_done=on_complete))

  def _on_subaru_settings(self):
    if get_vehicle_brand() != "subaru":
      return
    gui_app.push_widget(SubaruLayoutMici(back_callback=gui_app.pop_widget))
