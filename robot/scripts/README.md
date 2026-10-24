# robot/scripts

Operator-facing helper scripts for the inchworm print-crawl robot.

## attach_usb.py — WSL USB attach helper

Attaches all required inchworm USB devices to WSL via usbipd-win.  BusIds are
resolved fresh from `usbipd state` JSON each run (they are not hardcoded),
using VID:PID + USB serial number to identify each device.

### Requirements

- Windows host with [usbipd-win](https://github.com/dorssel/usbipd-win) installed.
- `powershell.exe` accessible from WSL (standard on WSL 2).
- Devices plugged into Windows USB ports.

### Usage

```bash
# From the robot/ directory:
cd robot

# Attach all devices (normal use):
uv run python scripts/attach_usb.py

# Preview what would be attached without executing (safe):
uv run python scripts/attach_usb.py --dry-run

# Use a specific hardware config:
uv run python scripts/attach_usb.py --config /path/to/hardware.yaml
```

### Output

Per-device lines are printed for all 7 roles:

```
[OK]      arm_left: already attached (BusId 10-4)
[OK]      arm_right: attached BusId 9-3
[OK]      dexarm: attached BusId 11-3
[OK]      relay: attached BusId 9-2
[MISSING] cam_left: not found in usbipd device list (VID:PID 32e6:9251, serial='5423534724'). ...
[OK]      cam_right: attached BusId 10-2
[OK]      cam_scanner: attached BusId 11-4

6/7 devices OK
```

Exit code is 0 if all devices attached successfully, 1 if any are missing or
failed to attach.

### Troubleshooting

**Device shows `[MISSING]`:**
- Verify the device is physically plugged into a Windows USB port.
- Run `usbipd list` in a Windows PowerShell to confirm the device appears.
- If it appears but usbipd has not bound it, run (as Windows administrator):
  `usbipd bind --busid <busid>`

**`usbipd attach` fails with access denied:**
- `usbipd attach --wsl` does not require administrator rights in recent
  usbipd-win versions, but the device must be bound first (see above).
