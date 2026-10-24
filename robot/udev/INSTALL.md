# Installing udev rules for the inchworm robot

This directory contains `99-inchworm.rules`, a udev rules file that creates
stable `/dev/inchworm_*` symlinks for inchworm USB devices. Without these
rules, Linux assigns device nodes (`/dev/ttyACM0`, `/dev/video0`, etc.) in
USB enumeration order, which changes across reboots and re-attachments.

## What the rules provide

| Symlink | Device | Identity |
|---|---|---|
| `/dev/inchworm_arm_left` | Left SO-101 arm (CH343) | serial `5B91045044` |
| `/dev/inchworm_arm_right` | Right SO-101 arm (CH343) | serial `5B91030298` |
| `/dev/inchworm_dexarm` | Rotrics DexArm (STM32 CDC) | serial `306133643438` |
| `/dev/inchworm_relay` | USB relay / laser switch (CH340) | VID:PID `1a86:7523` |
| `/dev/inchworm_cam_scanner` | Arducam OV9281 scanner camera | serial `UC762` |

**cam_left and cam_right do not have udev rules** — see below.

## Camera role resolution: cam_left and cam_right

Both Waveshare OV9281 cameras (`32e6:9251`) report the **same USB serial
`5423534724`** to Linux. udev cannot distinguish them: a rule using
`ATTRS{serial}=="5423534724"` matches both cameras simultaneously.

**Mechanism (equivalent to AC#3):** `inchworm.discovery` resolves `cam_left`
and `cam_right` at runtime using `identify_by: usbip_busid`. The
`windows_busid` field in `hardware.yaml` encodes the physical dock port:

- `cam_left` → BusId `9-4` (dock port must not change)
- `cam_right` → BusId `10-2` (dock port must not change)

Discovery reads `/sys/devices/platform/vhci_hcd.0/status` (world-readable) to
map each BusId to its local Linux USB device, then reads the sysfs
`video4linux/` children to find the `/dev/videoN` capture node (index 0).

This produces a stable role→node mapping regardless of enumeration order:

| Role | BusId | sysfs device | /dev node |
|---|---|---|---|
| `cam_left` | `9-4` | `1-5` | `/dev/video0` |
| `cam_right` | `10-2` | `1-6` | `/dev/video2` |

If the dock port changes, update `windows_busid` in
`robot/config/hardware.yaml` under `cameras.left_arm.identity` or
`cameras.right_arm.identity`. Do **not** add `KERNELS==` (port-path) pins to
this rules file: WSL usbip assigns ports in attach order, so port-path values
are not stable identities.

## Identity strategy — no port-path pins

All rules key on **VID:PID** and (where available) **USB serial number**.
Under WSL usbip, every device sits on the virtual host controller and port
numbers are assigned in **attach order** — they are **not stable identities**.
Port-path (`KERNELS==`) constraints are therefore intentionally absent.

One device has no USB serial and is matched by VID:PID alone:

- **relay** (`1a86:7523`): matched solely by VID:PID. The rule **relies on at
  most one CH340 1a86:7523 being attached** at a time; `inchworm.discovery`
  raises `RelayAmbiguousError` at runtime if more than one is found.

The standard `/dev/serial/by-id/` symlinks created by `60-serial.rules` remain
in place and are used by `inchworm.discovery`. These rules add shorter aliases.

## Installation

**One-time, on each Linux machine that runs the robot:**

```bash
sudo cp robot/udev/99-inchworm.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Then re-attach devices (or replug them) and verify:

```bash
ls -l /dev/inchworm_*
```

Expected output (all devices present and attached):

```
lrwxrwxrwx ... /dev/inchworm_arm_left    -> ttyACM1
lrwxrwxrwx ... /dev/inchworm_arm_right   -> ttyACM0
lrwxrwxrwx ... /dev/inchworm_cam_scanner -> video4
lrwxrwxrwx ... /dev/inchworm_dexarm      -> ttyACM2
lrwxrwxrwx ... /dev/inchworm_relay       -> ttyUSB0
```

Exact node suffixes (`ACM0`, `video4`, …) will differ; only the symlinks are
stable. Note that `cam_left` and `cam_right` do **not** appear here — use
`inchworm-discover` or `python -m inchworm.discover` to see their resolved
`/dev/videoN` nodes.

## Troubleshooting

**Symlink absent after install:**

```bash
udevadm info -a /dev/ttyACMx | grep -E 'serial|idVendor|idProduct'
```

Compare the printed serial against the value in `hardware.yaml`. For relay
(no USB serial), check that no second CH340 1a86:7523 device is attached.

**cam_left or cam_right not resolving:**

Run `inchworm-discover` (or `python -m inchworm.discover`) and check the
`by-id` column shows the Windows BusId (e.g. `9-4`). If status is
`not_attached`, the device was not forwarded to WSL:

```bash
# On Windows (PowerShell, administrator):
usbipd bind --busid 9-4
usbipd attach --wsl --busid 9-4
```

If the dock port changed, update `windows_busid` in `hardware.yaml`.

**Permission denied on device node:**

The rules set `MODE="0666"`. If you still get permission errors, add your user
to the `dialout` (serial) and `video` groups:

```bash
sudo usermod -aG dialout,video "$USER"
```

Then log out and back in.

## WSL / usbipd note

On WSL 2 (Windows host), udev runs inside WSL but devices must first be
forwarded from Windows with usbipd. Install the rules inside WSL, then use
`robot/scripts/attach_usb.py` to attach all devices before running udev
rules. See `robot/scripts/README.md` for attach-helper usage.
