import unittest

# BluePilot: target-platform and angle-controller regression dependencies.
from types import SimpleNamespace

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.subaru.carcontroller import CarController
from opendbc.car.subaru.interface import CarInterface
from opendbc.car.subaru.values import CAR, DBC, SubaruFlags, SubaruSafetyFlags
from opendbc.car.structs import CarParams
# End BluePilot

from opendbc.car.subaru.fingerprints import FW_VERSIONS


class TestSubaruFingerprint(unittest.TestCase):
  def test_fw_version_format(self):
    for platform, fws_per_ecu in FW_VERSIONS.items():
      for (ecu, _, _), fws in fws_per_ecu.items():
        fw_size = len(fws[0])
        for fw in fws:
          assert len(fw) == fw_size, f"{platform} {ecu}: {len(fw)} {fw_size}"

  # BluePilot: lock the target vehicle's distinct recognition and control gate.
  def test_outback_2025_firmware_is_unique(self):
    target_fw = FW_VERSIONS[CAR.SUBARU_OUTBACK_2025]
    for target_ecu, target_versions in target_fw.items():
      for candidate, candidate_fw in FW_VERSIONS.items():
        if candidate != CAR.SUBARU_OUTBACK_2025 and target_ecu in candidate_fw:
          assert set(target_versions).isdisjoint(candidate_fw[target_ecu]), (target_ecu, candidate)

  def test_only_outback_2025_angle_candidate_is_enabled(self):
    target_cp = CarInterface.get_non_essential_params(CAR.SUBARU_OUTBACK_2025)
    CarInterface.get_non_essential_params_sp(target_cp, CAR.SUBARU_OUTBACK_2025)

    assert not target_cp.dashcamOnly
    assert target_cp.steerControlType == CarParams.SteerControlType.angle
    assert target_cp.flags & SubaruFlags.GLOBAL_GEN2
    assert target_cp.flags & SubaruFlags.LKAS_ANGLE
    assert target_cp.safetyConfigs[0].safetyParam & SubaruSafetyFlags.GEN2
    assert target_cp.safetyConfigs[0].safetyParam & SubaruSafetyFlags.LKAS_ANGLE
    assert not target_cp.alphaLongitudinalAvailable
    assert not target_cp.openpilotLongitudinalControl

    dormant_cp = CarInterface.get_non_essential_params(CAR.SUBARU_OUTBACK_2023)
    CarInterface.get_non_essential_params_sp(dormant_cp, CAR.SUBARU_OUTBACK_2023)
    assert dormant_cp.dashcamOnly
  # End BluePilot


# BluePilot: focused regression coverage for the new angle-command controller path.
class TestSubaruAngleController(unittest.TestCase):
  def setUp(self):
    self.CP = CarInterface.get_non_essential_params(CAR.SUBARU_OUTBACK_2025)
    self.CP_SP = CarInterface.get_non_essential_params_sp(self.CP, CAR.SUBARU_OUTBACK_2025)
    self.dbc_name = DBC[self.CP.carFingerprint][Bus.pt]

  def run_controller(self, lat_active, desired_angle, measured_angle, v_ego=15.0):
    controller = CarController({Bus.pt: self.dbc_name}, self.CP, self.CP_SP)
    # Trigger the 50 Hz steering path without frame-zero HUD message copies.
    controller.frame = controller.p.STEER_STEP

    cc = structs.CarControl()
    cc.enabled = lat_active
    cc.latActive = lat_active
    cc.actuators.steeringAngleDeg = desired_angle

    cs = SimpleNamespace(out=structs.CarState(vEgoRaw=v_ego, steeringAngleDeg=measured_angle))
    new_actuators, can_sends = controller.update(cc.as_reader(), structs.CarControlSP(), cs, 0)

    angle_msgs = [msg for msg in can_sends if msg[0] == 0x124]
    self.assertEqual(len(angle_msgs), 1)
    self.assertEqual(angle_msgs[0][2], 0)

    parser = CANParser(self.dbc_name, [("ES_LKAS_ANGLE", 50)], 0)
    self.assertEqual(parser.update([0, [angle_msgs[0]]]), {0x124})
    return new_actuators, parser.vl["ES_LKAS_ANGLE"]

  def test_active_sends_angle_command(self):
    new_actuators, values = self.run_controller(lat_active=True, desired_angle=20.0, measured_angle=0.0)

    self.assertEqual(values["LKAS_Request"], 1)
    self.assertEqual(values["SET_3"], 3)
    self.assertGreater(abs(values["LKAS_Output"]), 0.0)
    self.assertAlmostEqual(values["LKAS_Output"], new_actuators.steeringAngleDeg, places=2)

  def test_inactive_tracks_measured_angle(self):
    measured_angle = 37.25
    new_actuators, values = self.run_controller(lat_active=False, desired_angle=-100.0, measured_angle=measured_angle)

    self.assertEqual(values["LKAS_Request"], 0)
    self.assertEqual(values["SET_3"], 3)
    self.assertAlmostEqual(values["LKAS_Output"], measured_angle, places=2)
    self.assertAlmostEqual(new_actuators.steeringAngleDeg, measured_angle, places=2)
# End BluePilot
