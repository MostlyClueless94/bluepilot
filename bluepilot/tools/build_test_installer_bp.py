"""Configure comma's AGNOS installer for this BluePilot test repository.

Only the reserved repository URL field is changed. The installer implementation
and branch field remain intact; the output is for comma 3X, not desktop execution.
"""
import argparse
import hashlib
import json
import re
import struct
import urllib.request
from pathlib import Path

OWNER_BP = "MostlyClueless94"
BRANCH_BP = "codex/experimental-resume-study"
REPOSITORY_BP = f"https://github.com/{OWNER_BP}/bluepilot.git"
ORIGINAL_REPOSITORY_BP = f"https://github.com/{OWNER_BP}/openpilot.git"
SOURCE_BP = f"https://installer.comma.ai/{OWNER_BP}/{BRANCH_BP}"


def configure_installer_bp(data: bytes) -> tuple[bytes, dict]:
  """Require an ARM64 ELF and replace one reserved URL slot without resizing."""
  if (data[:6] != b"\x7fELF\x02\x01" or len(data) < 64 or struct.unpack_from("<H", data, 18)[0] != 183):
    raise ValueError("Expected a little-endian ARM64 ELF installer for comma 3X")
  source = ORIGINAL_REPOSITORY_BP.encode()
  slots = list(re.finditer(re.escape(source) + rb"\? +(?=\x00)", data))
  branch_slots = list(re.finditer(re.escape(BRANCH_BP.encode()) + rb"\? +(?=\x00)", data))
  if len(slots) != 1 or len(branch_slots) != 1:
    raise ValueError("Installer's reserved repository or branch field is not unique")
  slot = slots[0]
  replacement = REPOSITORY_BP.encode() + b"?"
  if len(replacement) > slot.end() - slot.start():
    raise ValueError("Repository URL does not fit the reserved installer slot")
  replacement = replacement.ljust(slot.end() - slot.start(), b" ")
  output = data[:slot.start()] + replacement + data[slot.end():]
  if len(output) != len(data) or output[:slot.start()] != data[:slot.start()] or output[slot.end():] != data[slot.end():]:
    raise ValueError("Unexpected change outside the reserved repository slot")
  manifest = {
    "device": "comma 3X (tici)", "source_url": SOURCE_BP, "repository_url": REPOSITORY_BP,
    "branch": BRANCH_BP, "source_sha256": hashlib.sha256(data).hexdigest(),
    "installer_sha256": hashlib.sha256(output).hexdigest(), "bytes": len(output),
    "modified_slot": {"offset": slot.start(), "length": len(replacement)},
  }
  return output, manifest


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--agnos-version", default="18.5", help="Installed AGNOS version used to select comma's installer")
  parser.add_argument("--input", type=Path, help="Use a previously downloaded official installer")
  parser.add_argument("--output", type=Path, default=Path("i"))
  args = parser.parse_args()
  if args.input:
    data = args.input.read_bytes()
  else:
    request = urllib.request.Request(SOURCE_BP, headers={"User-Agent": f"AGNOSSetup-{args.agnos_version}",
                                                       "X-openpilot-device-type": "tici"})
    with urllib.request.urlopen(request, timeout=30) as response:
      data = response.read()
  output, manifest = configure_installer_bp(data)
  manifest["agnos_version"] = args.agnos_version
  args.output.write_bytes(output)
  args.output.chmod(0o755)
  args.output.with_suffix(".json").write_text(json.dumps(manifest, indent=2) + "\n")
  print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
  main()
