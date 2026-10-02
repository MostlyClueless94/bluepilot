"""Test the opt-in Ford overlap capability using compiled firmware and real DBCs."""
import unittest
from types import SimpleNamespace

from opendbc.can import CANPacker
from opendbc.car.ford.values import FordSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.sunnypilot.car.ford.concurrent_accel_bp import constrain_concurrent_accel_bp
from opendbc.sunnypilot.car.ford.fordcan_ext import create_acc_msg
from opendbc.sunnypilot.car.ford.longitudinal_ext import LongitudinalResult


class TestFordConcurrentAccelBP(unittest.TestCase):
  FLAGS = FordSafetyFlags.LONG_CONTROL | FordSafetyFlags.CANFD | FordSafetyFlags.CONCURRENT_ACCEL_BP
  # Shared wrong-mode tests enumerate the TX allowlist of every test class.
  TX_MSGS = [[0x083, 0], [0x083, 2], [0x186, 0], [0x18A, 0], [0x3CA, 0], [0x3D6, 0], [0x3D8, 0]]

  def setUp(self):
    self.packer = CANPackerSafety("ford_lincoln_base_pt")
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(0)
    self._init(self.FLAGS)

  def _init(self, flags):
    self.assertEqual(self.safety.set_safety_hooks(CarParams.SafetyModel.ford, flags), 0)
    self.safety.init_tests()
    self.safety.set_controls_allowed(True)
    self._gas_pedal(True)

  def _gas_pedal(self, pressed):
    msg = self.packer.make_can_msg_safety("EngVehicleSpThrottle", 0, {"ApedPos_Pc_ActlArb": 10 if pressed else 0})
    self.assertTrue(self.safety.safety_rx_hook(msg))
    self.assertEqual(self.safety.get_gas_pressed_prev(), pressed)

  def _command(self, gas=1.0, **overrides):
    values = {"AccPrpl_A_Rq": gas, "AccPrpl_A_Pred": -5.0, "AccBrkTot_A_Rq": 0.0,
              "AccBrkPrchg_B_Rq": 0, "AccBrkDecel_B_Rq": 0, "AccStopStat_B_Rq": 0}
    values.update(overrides)
    return self.packer.make_can_msg_safety("ACCDATA", 0, values)

  def _tx(self, gas=1.0, **overrides):
    return self.safety.safety_tx_hook(self._command(gas, **overrides))

  def test_gas_range_while_pedal_pressed(self):
    for allowed in (False, True):
      self.safety.set_controls_allowed(allowed)
      for raw_gas in range(1024):
        gas = round(raw_gas * .01 - 5.0, 2)
        with self.subTest(allowed=allowed, raw_gas=raw_gas):
          self.assertEqual(self._tx(gas), raw_gas == 0 or (allowed and 500 <= raw_gas <= 700))
    # The exception is local to this Ford TX path, not a global override.
    self.assertFalse(self.safety.get_longitudinal_allowed())

  def test_pedal_overlap_requires_neutral_brake_channel(self):
    for raw_accel in range(8192):
      accel = raw_accel * .0039 - 20.0
      with self.subTest(raw_accel=raw_accel):
        self.assertEqual(self._tx(AccBrkTot_A_Rq=accel), raw_accel == 5128)

  def test_pedal_overlap_requires_inactive_prediction(self):
    for raw_gas in range(1024):
      with self.subTest(raw_gas=raw_gas):
        self.assertEqual(self._tx(AccPrpl_A_Pred=raw_gas * .01 - 5.0), raw_gas == 0)

  def test_no_brakes_or_driver_override_or_aeb_disable(self):
    for signal in ("AccBrkPrchg_B_Rq", "AccBrkDecel_B_Rq", "AccStopStat_B_Rq", "AccBrkPrkEl_B_Rq",
                   "CmbbOvrrd_B_RqDrv", "CmbbEngTqMn_B_Rq", "CmbbDeny_B_Actl"):
      for gas in (-5.0, 0.0, 1.0, 2.0):
        with self.subTest(signal=signal, gas=gas):
          self.assertFalse(self._tx(gas, **{signal: 1}))

  def test_capability_requires_all_flags_and_resets(self):
    # Reinitialization must clear the capability; partial flags cannot enable it.
    for flags in (3, 1, 0, 2, 4, 5, 6):
      self._init(self.FLAGS)
      self.assertTrue(self._tx())
      self._init(flags)
      with self.subTest(flags=flags):
        self.assertFalse(self._tx())

  def test_pedal_release_restores_normal_longitudinal_limits(self):
    self.assertTrue(self._tx())
    self._gas_pedal(False)
    self.assertTrue(self._tx(-.5, AccBrkTot_A_Rq=-3.5, AccBrkDecel_B_Rq=1))
    self.assertFalse(self._tx(2.01))
    self.assertFalse(self._tx(AccBrkTot_A_Rq=-3.51))

  def test_driver_brake_and_cruise_off_cancel_positive_request(self):
    for brake, cruise in ((2, 5), (1, 0)):
      self._init(self.FLAGS)
      self.assertTrue(self._tx())
      msg = self.packer.make_can_msg_safety("EngBrakeData", 0, {"BpedDrvAppl_D_Actl": brake, "CcStat_D_Actl": cruise})
      self.assertTrue(self.safety.safety_rx_hook(msg))
      self.assertFalse(self.safety.get_controls_allowed())
      self.assertFalse(self._tx())
      self.assertTrue(self._tx(-5.0))

  def test_relay_malfunction_blocks_overlap(self):
    self.safety.set_relay_malfunction(True)
    self.assertFalse(self._tx())

  def test_generated_overlap_and_release_frames_pass_firmware(self):
    packer = CANPacker("ford_lincoln_base_pt")
    previous = -5.
    for pressed in [True] * 30 + [False] * 30:
      self._gas_pedal(pressed)
      requested = LongitudinalResult(1., 1., False, False, -5., False, 100., False)
      result = constrain_concurrent_accel_bp(requested, pressed, True, previous)
      addr, data, bus = create_acc_msg(packer, SimpleNamespace(main=0), True, result.gas, result.accel,
                                       result.accel_pred_send, result.stopping, result.brake_actuate,
                                       result.precharge_actuate, result.target_speed)
      self.assertTrue(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, data)))
      self.assertGreater(result.gas, 0.)
      self.assertLessEqual(result.gas - max(previous, 0.), .020001)
      previous = result.gas
