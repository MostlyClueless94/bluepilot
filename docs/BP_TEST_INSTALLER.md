# comma 3X prototype installer

Enter this full URL in the comma's Custom Software installer:

```text
https://raw.githubusercontent.com/MostlyClueless94/bluepilot/codex/install/i
```

The URL is case-sensitive. The installer clones
`https://github.com/MostlyClueless94/bluepilot.git`, branch
`codex/experimental-resume-study`. The application commit at publication is
`021405516531eb53e9ab2a2828355965cfa74c2c` (the installer follows the branch).
The separate `codex/install` branch hosts the installer and its provenance.

For an existing BluePilot installation, use Settings → Software → Uninstall,
then choose Custom Software during setup and enter the URL above. Connect to
Wi-Fi and keep the device powered while it downloads, builds and restarts.
This is a source branch, so the comma will compile application and panda
firmware on-device. Device build/installation success has not been verified
from this desktop. Save any local recordings/settings you want to preserve
before uninstalling.

After installation, the toggle is at:

**Settings → BluePilot → Longitudinal Tuning → Concurrent Acceleration Prototype**

It starts off by default, is adjustable offroad and requires a restart after
changing it. Openpilot longitudinal control and Experimental Mode must be
enabled, and Disengage on Accelerator must be off. The car configuration must
identify the CAN-FD F-150 MK14, with Ford safety mode and no dashcam restriction.

This installer makes the research branch available for testing; it does not
establish native PCM arbitration, braking behavior or actual acceleration
smoothness. The complete safety suite still has the existing baseline Ford
failures described in `BP_CONCURRENT_ACCEL_PROTOTYPE.md`. Initial validation
should be controlled testing, with brake and cancel checks before acceleration
overlap. Report the branch/version, any build errors, and the recording of the
test so the controller and firmware behavior can be checked.

## Installer provenance and checks

`i` is comma's ARM64 AGNOS installer obtained with AGNOS version 18.5 and
device type `tici`. Only its reserved repository URL string was changed from
`MostlyClueless94/openpilot.git` to `MostlyClueless94/bluepilot.git`.
The branch string and executable code are unchanged, as is the file size.
The marker/padding scheme is defined in
`selfdrive/ui/installer/installer.cc` (`get_str` and `GIT_URL`).

`i.json` records the source URL, source/output SHA-256, byte length and modified
slot. `bluepilot/tools/build_test_installer_bp.py` reproduces the configuration
and refuses unexpected ELF architecture, duplicate/missing slots, or URL
overflow. The published download is checked against the local artifact hash.
The installer binary itself has not been executed on a comma from this task.

To return to official BluePilot 7.0, its installer URL is:

```text
https://installer.comma.ai/BluePilotDev/bp-7.0
```
