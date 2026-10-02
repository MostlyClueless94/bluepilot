"""Offline prototype checks; never connects to a vehicle or writes device Params."""
import argparse
import json
import sys
import types
import unittest  # noqa: TID251 -- existing firmware suite uses unittest
from pathlib import Path

ROOT_BP = Path(__file__).resolve().parents[2]


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--all-safety", action="store_true", help="Run the complete preserved firmware safety suite")
  parser.add_argument("--offline-services", action="store_true", help="Replace unused imports of unbuilt device services")
  parser.add_argument("--report", type=Path, help="Save firmware test IDs for comparison with an untouched baseline")
  args = parser.parse_args()
  sys.path.insert(0, str(ROOT_BP))

  if args.offline_services:
    # Ford's constant imports pull in these native services. Firmware tests do
    # not call them. Raise if that changes; never substitute control algorithms,
    # DBC packing, cereal schemas, or the compiled safety hooks under test.
    params = types.ModuleType("openpilot.common.params")

    class UnavailableParamsBP:
      def __init__(self, *args, **kwargs):
        raise RuntimeError("Device Params unavailable in offline firmware unit tests")

    params.Params = UnavailableParamsBP
    sys.modules[params.__name__] = params
    messaging = types.ModuleType("cereal.messaging")
    sys.modules[messaging.__name__] = messaging
    hardware = types.ModuleType("openpilot.system.hardware")
    hardware.PC = True
    sys.modules[hardware.__name__] = hardware

  loader = unittest.TestLoader()
  if args.all_safety:
    suite = loader.discover(str(ROOT_BP / "opendbc/safety/tests"), top_level_dir=str(ROOT_BP))
  else:
    suite = loader.loadTestsFromName("opendbc.safety.tests.test_ford_concurrent_accel_bp")
  result = unittest.TextTestRunner(verbosity=1).run(suite)
  if args.report:
    report = {"run": result.testsRun, "failures": [str(test) for test, _ in result.failures],
              "errors": [(str(test), error) for test, error in result.errors],
              "skipped": [(str(test), reason) for test, reason in result.skipped]}
    args.report.write_text(json.dumps(report, indent=2) + "\n")

  import pytest
  # One-shot process: explicit test path, no application conftest or cache writes.
  software_status = pytest.main(["-c", "/dev/null", "--rootdir", str(ROOT_BP), "--confcutdir", str(ROOT_BP / "bluepilot"),  # noqa: TID251
                                 "-p", "no:cacheprovider", "-q",
                                 str(ROOT_BP / "bluepilot/selfdrive/controls/tests/test_concurrent_accel_bp.py")])
  return int(not result.wasSuccessful() or software_status != 0)


if __name__ == "__main__":
  sys.exit(main())
