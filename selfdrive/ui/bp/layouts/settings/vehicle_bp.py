from openpilot.common.params import Params
from openpilot.common.params_pyx import UnknownKeyName
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle import VehicleLayout
from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.factory import BrandSettingsFactory
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.sunnypilot.widgets.list_view import ButtonActionSP, ListItemSP, button_item_sp
from openpilot.system.ui.widgets import DialogResult
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog
from openpilot.system.ui.widgets.scroller_tici import Scroller
from bluepilot.params.vehicle_profiles import reset_active_profile, snapshot_active_profile


class VehicleLayoutBP(VehicleLayout):
  """BluePilot Vehicle settings extension with vehicle profile controls."""

  def __init__(self):
    super().__init__()
    self._params = Params()

    self._vehicle_profile_action = ButtonActionSP(lambda: tr("ACTIVE"))
    self._vehicle_profile_action.set_value(lambda: self._get_vehicle_profile_value())
    self._vehicle_profile_status = ListItemSP(
      lambda: tr("Vehicle Profile"),
      description=lambda: self._get_vehicle_profile_description(),
      action_item=self._vehicle_profile_action,
    )
    self._vehicle_profile_relearn_btn = button_item_sp(
      lambda: tr("Relearn Current Profile"),
      lambda: tr("RELEARN"),
      lambda: tr("Save the current model and settings into this vehicle's profile."),
      callback=self._relearn_vehicle_profile,
    )
    self._vehicle_profile_reset_btn = button_item_sp(
      lambda: tr("Reset Current Profile"),
      lambda: tr("RESET"),
      lambda: tr("Forget this vehicle's saved profile so it can be learned again."),
      callback=self._reset_vehicle_profile,
    )
    self._rebuild_items()
    self._update_vehicle_profile_controls()

  @staticmethod
  def _safe_get(params: Params, key: str, default=None):
    try:
      val = params.get(key, return_default=True)
      return val if val not in (None, b"", "") else default
    except UnknownKeyName:
      return default

  def _rebuild_items(self):
    self.items = [
      self._vehicle_item,
      self._legend_widget,
      self._vehicle_profile_status,
      self._vehicle_profile_relearn_btn,
      self._vehicle_profile_reset_btn,
    ] + self._brand_items
    self._scroller = Scroller(self.items, line_separator=True, spacing=0)

  def _update_brand_settings(self):
    self._vehicle_item._title = self._platform_selector.text
    self._vehicle_item.title_color = self._platform_selector.color
    vehicle_text = tr("REMOVE") if ui_state.params.get("CarPlatformBundle") else tr("SELECT")
    self._vehicle_item.action_item.set_text(vehicle_text)

    brand = self.get_brand()
    if brand != self._current_brand:
      self._current_brand = brand
      self._brand_settings = BrandSettingsFactory.create_brand_settings(brand)
      self._brand_items = self._brand_settings.items if self._brand_settings else []
      self._rebuild_items()

    self._update_vehicle_profile_controls()

  def _get_vehicle_profile_value(self) -> str:
    active = self._safe_get(self._params, "BPVehicleProfileActive") or ""
    if isinstance(active, bytes):
      active = active.decode("utf-8", errors="replace")
    active = str(active)
    if not active:
      return tr("None")
    return active if len(active) <= 18 else active[:15] + "..."

  def _get_vehicle_profile_description(self) -> str:
    status = self._safe_get(self._params, "BPVehicleProfileStatus") or {}
    if not isinstance(status, dict):
      return tr("Learns and restores model/settings for each fingerprinted vehicle.")

    action = str(status.get("action", "")).capitalize()
    message = status.get("message") or tr("Learns and restores model/settings for each fingerprinted vehicle.")
    details = []
    if status.get("restartRequired"):
      details.append(tr("restart required"))
    if status.get("modelFallback"):
      details.append(tr("default model used"))
    if status.get("changed"):
      details.append(f"{status.get('changed')} {tr('settings changed')}")
    return f"{message} ({', '.join(details)})" if details else f"{action}: {message}" if action else str(message)

  def _update_vehicle_profile_controls(self):
    active = bool(self._safe_get(self._params, "BPVehicleProfileActive"))
    self._vehicle_profile_relearn_btn.action_item.set_enabled(active)
    self._vehicle_profile_reset_btn.action_item.set_enabled(active)

  def _relearn_vehicle_profile(self):
    def handle_confirm(result: DialogResult):
      if result == DialogResult.CONFIRM:
        snapshot_active_profile(self._params)
        self._update_vehicle_profile_controls()
        cloudlog.info("BluePilot: relearned active vehicle profile")

    gui_app.push_widget(ConfirmDialog(
      tr("Save the current model and settings into this vehicle's profile?"),
      tr("Relearn Profile"),
      callback=handle_confirm
    ))

  def _reset_vehicle_profile(self):
    def handle_confirm(result: DialogResult):
      if result == DialogResult.CONFIRM:
        reset_active_profile(self._params)
        self._update_vehicle_profile_controls()
        cloudlog.info("BluePilot: reset active vehicle profile")

    gui_app.push_widget(ConfirmDialog(
      tr("Forget this vehicle's saved profile? It will be learned again on the next drive."),
      tr("Reset Profile"),
      callback=handle_confirm
    ))

  def show_event(self):
    super().show_event()
    self._update_vehicle_profile_controls()
