# inchworm-robot

Control software for the inchworm print-crawl robot: two SO-101 arms and a
Rotrics DexArm on a fixed chassis, moving a printed tube. Hardware facts live in
backlog doc-1; this package must not restate them in code.

## Quick start

```bash
cd robot
uv run pytest            # all tests run against the mock backend, no hardware
```

## Layout

| Path | What it holds |
|---|---|
| `config/hardware.yaml` | Declarative hardware description: device identities, geometry, laser design values, USB constraint. Data, not code. Calibration tasks write results back here. |
| `src/inchworm/config.py` | Typed, versioned loader (`load_config`). Rejects unknown fields, missing fields and wrong types with a `ConfigError` naming the field path. |
| `src/inchworm/constants.py` | The system constants (attachment pitch, laser design values, single-camera limit), read once from the config. Import them from here; never restate the literals. |
| `src/inchworm/backends.py` | Driver backend abstraction. `create_backend(config)` returns a `MockBackend` or `HardwareBackend`. |
| `src/inchworm/units.py` | Unit type aliases (`Millimeters`, `Degrees`, `MillimetersPerPixel`). |

## Units convention

**Millimeters and degrees at every interface boundary**: function arguments,
return values, config fields, logged values and anything crossing a module or
driver boundary.

Enforced two ways:

1. **Naming.** Every dimensional config field ends in its unit: `_mm`, `_deg`,
   `_mm_per_px`. `tests/test_config.py::test_dimensional_fields_carry_unit_suffix`
   fails if a float field in the schema lacks one.
2. **Type annotations.** Use `inchworm.units.Millimeters` / `Degrees` /
   `MillimetersPerPixel` on interface signatures and constants.

A module that needs meters or radians internally (for example a kinematics
solver) converts at its own boundary and documents the internal unit where it
is used. Radians or meters must never leak out of a public function.

## Backend selection

The mock backend serves in-memory serial and camera transports, so arm, DexArm,
relay and camera logic runs with zero hardware attached.

- Config: `backend: mock` or `backend: hardware` in `config/hardware.yaml`.
- Override: `INCHWORM_BACKEND=mock|hardware` takes precedence over the config.
- Config file override: `INCHWORM_CONFIG=/path/to/hardware.yaml`.

`HardwareBackend` currently refuses to open any device. Real transports arrive
with device discovery (AA-5) and the per-device drivers (AA-8, AA-12, AA-14,
AA-29). Camera transports must go through the AA-6 single-camera arbiter.

## Device identity

Devices are keyed on `vendor_id:product_id` plus `usb_serial` wherever the
device reports one. The relay (CH340) and the right-arm camera report no USB
serial, so they are marked `identify_by: port_path` and must be resolved by
where they are plugged in. `windows_busid_observed` is informational only:
usbipd BusIds change across reconnects.
