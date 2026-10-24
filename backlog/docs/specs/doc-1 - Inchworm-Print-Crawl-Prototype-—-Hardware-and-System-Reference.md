---
id: doc-1
title: Inchworm Print-Crawl Prototype — Hardware and System Reference
type: specification
created_date: '2026-10-20 21:06'
updated_date: '2026-10-24 16:07'
---
Single source of truth for hardware, geometry and system constraints of the inchworm print-crawl robot. Tasks reference this doc instead of restating hardware facts. Update this doc (via `backlog doc update doc-1`) whenever a measured value is corrected; do not let per-task copies drift.

## 1. System concept

A fixed chassis on the ground carries three robot arms. Two SO-ARM101 (SO-101) arms grip attachment points integrated into a printed tube. A third arm, a Rotrics DexArm, carries a 3D print head and a laser-profilometer camera assembly.

The chassis does **not** move. **The tube moves.** The two SO-101 arms alternately grip the tube's attachment points and translate the tube along its axis, one attachment pitch per cycle ("inchworm" motion, from the tube's point of view). After each advance the arms return to a folded transit pose that brings the printed part as close as possible to the DexArm print head and maximizes pose repeatability. Then the laser leveling step re-measures the part surface, and the DexArm prints the next segment, extending the tube.

## 2. Mechanical layout and geometry

- Attachment-point pitch: **183.676 mm**, center to center, along the tube axis.
- The DexArm is mounted **90 degrees** from the two SO-101 arms; all three are in line on the chassis.
- One cycle advances the tube by exactly one pitch (183.676 mm) unless a task explicitly states otherwise.
- **Both SO-101 arms hold the tube at the start of every cycle and at all times while printing** (owner, 2026-10-24). Only during a regrip does one arm release while the other holds.
- Arms have a restricted reachable envelope: the picture at `pictures/robot.png` shows the real built configuration and the folded pose. Arms cannot move anywhere in their nominal workspace; joint limits and self/chassis/tube collision envelopes must be derived and enforced in software.
- Reference imagery and CAD: `pictures/robot.png`, `CAD/Assembly 1.step`, `assets/robot-assembly.glb`.

## 3. Arms

### SO-ARM101 (x2) — crawl arms
- 5 DoF + gripper, Feetech STS/SCS-class serial bus servos, daisy-chained on one serial bus per arm.
- Each arm reaches the host as a CH343 USB-serial adapter. On Windows these enumerate as `USB-Enhanced-SERIAL CH343` (COM3 / COM5 observed); under WSL they appear as `/dev/ttyACM*` or `/dev/ttyUSB*` after `usbipd attach`.
- Servo IDs have already been assigned by the owner. **No purpose-built leader arms exist.** Per **decision-1** (2026-10-24), demonstrations use **cross-arm teleoperation**: one follower, torque off, acts as the leader for the other, while the tube is held by a **fixture**. A torque-off arm can never hold the tube.
- Gearing: both followers use STS3215 at **1/345 on all joints**. A purpose-built SO-101 leader uses 1/191 (pan), 1/345 (shoulder lift), 1/191 (elbow) and 1/147 (wrist flex, wrist roll, gripper), so a follower used as a leader is stiffer to backdrive (source: LeRobot `docs/source/so101.mdx`).
- **Custom gripper**: the owner replaced the stock gripper with a custom design in which the **lock/latch mechanism and the jaw share the same servo ID**. There is no separate lock servo. Lock vs unlock is therefore a *position band* of the single gripper servo, not a separate actuator. Locking must be commanded by code (never left to a learned policy) and must hold position until the next regrip.

### Rotrics DexArm — print/scan arm
- Delta-style desktop arm, controlled over USB serial with a **Marlin-style G-code dialect** (Rotrics firmware). Has a module interface for the 3D print head.
- Carries both the print head and the laser-line scanner assembly.
- Slicing for print segments is done by the owner in **Cura**; a correct DexArm machine/printer profile must be installed in Cura first. The agent may request a sliced G-code file as a human task but must not assume a profile already exists.

## 4. Cameras

- SO-101 arms (x2): Waveshare **OV9281** 1 MP mono USB camera (A), global shutter, up to 120 FPS, UVC.
- DexArm scanner: Arducam **OV9281** mono global-shutter USB camera, 720p, 100 FPS, UVC, low-distortion M12 lens, no microphone.
- All are mono (Y8/greyscale) global-shutter UVC devices. Global shutter matters: laser-line and insertion imagery must not be rolling-shutter skewed during motion.
- **Measured 2026-10-24 (`lsusb -v` in WSL), correcting the line above:** none of the three cameras advertises a Y8/GREY format. The two arm cameras enumerate as `32e6:9251` "openaicam" and offer **MJPEG only** (640x480, 800x600, 1280x720). The Arducam `0c45:6366` offers MJPEG and uncompressed YUY2 (up to 1280x800). All three are **USB 2.0 High-Speed devices** (`bcdUSB 2.00`, link at 480M). Mono output must therefore come from decoding and converting to greyscale in software. The sensors may still be mono, but the USB format is not.
- **Measured 2026-10-24, camera identity:** both arm cameras report the **same USB serial `5423534724`** (sysfs `serial` and udev `ID_SERIAL` are identical). Linux serial, by-id and udev rules therefore **cannot tell them apart**. Windows assigns the second camera a port-derived instance ID (`7&69F6256&0&2`) because the serials collide. The only stable distinguisher is the physical dock port, visible as the usbipd BusId: owner mapping cam_left = 9-4, cam_right = 10-2. **Do not move the arm-camera cables between dock ports.** Discovery maps BusId to the WSL node through `/sys/devices/platform/vhci_hcd.0/status` (AA-5).

## 5. Laser line scanner

- Laser: red line laser module, adjustable focus, ~39 in line length, 12 mm barrel, USB powered.
- Power switching: **USB serial relay module** (overcurrent-protected intelligent control switch) — the laser is switched on/off by issuing serial commands to this relay, so laser state is software-controlled and must be explicitly turned off on abort/shutdown.
- Triangulation geometry as designed by the owner:
  - Laser-to-camera angle: **40 degrees**
  - Camera/laser baseline offset: **67 mm**
  - Design standoff from the scanned surface: **80 mm**
  - Resulting sensitivity: approximately **0.1 mm of height change per pixel**
- These are design values, not calibrated values. Calibration must measure the actual mapping and report residuals against the 0.1 mm/px expectation.

## 6. USB topology constraint (critical)

Every arm and camera connects through **one centralized USB hub/dock**. Owner update 2026-10-24: the dock was replaced with a **USB 3.2** dock (observed hub chips: VIA `2109:0822/2822` and Realtek `0bda:0409/5409` SuperSpeed/High-Speed pairs). The owner reports the two arm-camera cables are rated 480 Mbps (USB 2.0).

Because every camera is a USB 2.0 High-Speed device, camera traffic travels on the dock's **USB 2.0 (480 Mbps) side**, not its SuperSpeed side. A USB 3.2 dock therefore does **not** by itself add camera bandwidth, and USB 2.0 cables cost nothing. Whether two cameras can stream at once is an **unmeasured** question. Until a measured test and an explicit owner decision say otherwise, the invariant below stands unchanged.

Total bandwidth is shared, so:

- **Only one camera may stream at a time.** This is a hard system invariant, not a preference.
- Software must enforce exclusive camera access with a single arbiter/lock: acquire -> open device -> stream -> close device -> release. A camera that is not in use must be closed, not merely paused.
- Serial control traffic for the arms is low-bandwidth and may be concurrent with one camera stream, but bulk camera streams must never overlap.

## 7. Host environment

- Control host: **WSL2** (Linux 6.18 microsoft-standard-WSL2, Ubuntu-class userland), Python 3.10, `uv` available. Note: LeRobot 0.6.2 (main as of 2026-10-23) requires Python >= 3.12, so its environment needs a uv-managed 3.12 interpreter (see AA-22).
- USB devices reach WSL via **usbipd-win**: `usbipd bind` (one-time, requires Windows administrator) then `usbipd attach --wsl`. Binding is a human/admin action; attaching per boot can be scripted.
- Observed at planning time (2026-10-20, verified by running `usbipd state` and an attach attempt):
  - Both SO-101 CH343 adapters are **persisted but have no BusId**, i.e. previously bound and *not currently physically connected*: `USB\\VID_1A86&PID_55D3\\5B91030298` (COM3) and `USB\\VID_1A86&PID_55D3\\5B91045044` (COM5). These serial numbers are the stable identities to key the arm role map on.
  - No DexArm, no OV9281 cameras and no USB relay appeared in `usbipd state` at all.
  - WSL had no `/dev/ttyUSB*`, `/dev/ttyACM*` or `/dev/video*` nodes.
  - `usbipd attach` on an unbound device fails with `Device is not shared; run 'usbipd bind --busid <id>' as administrator first`, so each device needs a one-time admin bind.
  - usbipd-win version present: 5.3.0.
- GPU: NVIDIA RTX 3050 Laptop, **4 GB VRAM**. System RAM 7 GB, 8 CPU cores. Policy training must fit this budget (small image resolution, small batch, gradient accumulation, mono single-camera input).

## 8. Software conventions

- All robot code lives under `robot/` as a `uv`-managed Python package, mirroring the existing `sim/` package layout (`pyproject.toml`, `src/`, `tests/`, `uv run pytest`).
- Hardware access is abstracted behind drivers with a simulation/mock backend so logic can be tested without attached hardware. Any task that cannot run against hardware must still ship tests against mocks.
- Units: millimeters and degrees at the interface layer; document any internal use of radians or meters.
- Every motion-capable entry point must support a dry-run mode that prints the planned trajectory without commanding servos.

## 9. Autonomy contract: code -> policy -> code

The system must run unattended. Each regrip follows a fixed handoff:

1. **Code** moves the arm on a planned trajectory to a coarse pre-insertion pose near the target attachment point.
2. **Code** hands control to the learned (LeRobot) insertion policy, which performs only the final insertion.
3. A **supervisor** owned by code bounds the policy: step/time limit, workspace bounds, success detection, and abort/retry. The policy never controls the lock and never runs unbounded.
4. On detected seating, **code** takes back control and drives the shared gripper servo into the locked position band, verifying it holds.
5. **Code** continues the cycle (fold, level, print).

No step in this chain may require operator intervention.

## 10. Known-unknowns to resolve during execution

- Exact servo ID map per arm and per joint, plus the measured locked/unlocked position bands of the custom gripper servo.
- True joint limits and collision envelope in the as-built configuration.
- Chassis-to-tube and camera-to-arm transforms (hand-eye calibration).
- Actual laser triangulation calibration vs the 40 deg / 67 mm / 80 mm design values.
- Whether seating can be detected from servo current/position alone or needs vision.
