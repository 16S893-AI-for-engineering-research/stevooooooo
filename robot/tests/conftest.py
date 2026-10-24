"""Suite-wide hardware isolation (orchestrator 2026-10-24).

A test once reached the real left-arm port /dev/ttyACM0 because its mock
patched the wrong function. Only the port's exclusive lock stopped it.
Tests must NEVER be able to touch hardware, whatever a single test patches,
so this autouse fixture replaces the real serial and V4L2 open paths with
functions that fail the test immediately.
"""
import pytest


class HardwareAccessInTest(AssertionError):
    pass


@pytest.fixture(autouse=True)
def _forbid_real_hardware(monkeypatch):
    def _no_serial(*args, **kwargs):
        raise HardwareAccessInTest(
            f"test tried to open a REAL serial port: args={args} kwargs={kwargs}"
        )

    def _no_camera(*args, **kwargs):
        raise HardwareAccessInTest(f"test tried to open a REAL camera: {args}")

    try:
        import serial  # pyserial
        monkeypatch.setattr(serial, "Serial", _no_serial)
    except ImportError:
        pass
    try:
        import cv2
        monkeypatch.setattr(cv2, "VideoCapture", _no_camera)
    except ImportError:
        pass
    yield
