"""Auto-trapper: coordinate three Thorlabs Kinesis stages and a Thorlabs TSI camera.

- X motor (vertical):   serial 27260320
- Y motor (propagation): serial 27260324
- Z motor (horizontal):  serial 27260050

Flow per attempt:
1) Move to a fresh lateral/vertical (X,Z) position at least MIN_LATERAL_SEP_MM away from last.
2) Sweep the reservoir along the beam (Y) over SWEEP_MM.
3) Capture frames and check for a trapped particle (bright spot) on the camera.

Requirements:
- pylablib (for Thorlabs Kinesis stages)
- thorlabs_tsi_sdk (pip) and the vendor DLL (64-bit) available at DLL_PATH or via THORLABS_TSI_DLL.
- OpenCV (opencv-python-headless is fine if no GUI needed).

Run with your venv Python:
    .venv/Scripts/python.exe auto_trapper.py
"""

import os
import sys
import time
import itertools
from pathlib import Path
from typing import Iterable, Iterator, Optional, Tuple

import cv2
import numpy as np
from cffi import FFI
import pylablib

# Compatibility shim for older pylablib (<=1.4.4):
# some pylablib releases use a different name for the FTDI/FT232 backend
# and code (or third-party libs) may expect `FT232DeviceBackend`. If the
# attribute is missing, try to alias it to whatever backend is available.
try:
    import pylablib.core.devio.comm_backend as _comm_backend
except Exception:
    _comm_backend = None

if _comm_backend is not None and not hasattr(_comm_backend, "FT232DeviceBackend"):
    for _alt in ("FT232HDeviceBackend", "FT2XXDeviceBackend", "FTDIBackend", "FTDIDeviceBackend", "SerialDeviceBackend"):
        if hasattr(_comm_backend, _alt):
            setattr(_comm_backend, "FT232DeviceBackend", getattr(_comm_backend, _alt))
            break

from pylablib.devices import Thorlabs
from thorlabs_tsi_sdk.tl_camera import TLCameraSDK

# -----------------------
# User-tunable parameters
# -----------------------
DLL_PATH = Path(
    r"C:/Program Files/Thorlabs/Scientific Imaging/Scientific Camera Support/Scientific Camera Interfaces/SDK/Native Toolkit/dlls/Native_64_lib/thorlabs_tsi_camera_sdk.dll"
)
# If you have tsi_sdk.dll instead, set THORLABS_TSI_DLL env var to its full path.

EXPOSURE_US = 5000
IMAGE_TIMEOUT_MS = 500  # per frame wait
FRAMES_TO_CHECK = 10     # frames per attempt
MIN_LATERAL_SEP_MM = 0.5
SWEEP_MM = 10.0          # reservoir sweep distance along propagation (Y)
SWEEP_CENTER_MM = 0.0    # Y center position; sweep goes +/- SWEEP_MM/2
MOVE_WAIT = 0.1
MOTOR_SCALE = 34304  # steps/mm for Z825B; adjust for your stage model

SERIAL_X = "27260320"  # vertical
SERIAL_Y = "27260324"  # propagation
SERIAL_Z = "27260050"  # horizontal

# Grid of lateral positions to iterate through (X,Z in mm)
X_POSITIONS = [0.0, 0.6, -0.6, 1.2, -1.2]
Z_POSITIONS = [0.0, 0.6, -0.6, 1.2, -1.2]

# Detection thresholds
MIN_BRIGHTNESS = 25      # on 0-255 normalized frame
MIN_REGION_AREA = 5      # pixels


# -----------------------
# Helper functions
# -----------------------

def ensure_dll() -> None:
    """Add the camera SDK DLL directory to the process search path."""
    dll_override = os.environ.get("THORLABS_TSI_DLL")
    target = Path(dll_override) if dll_override else DLL_PATH
    dll_dir = target.parent
    os.environ["PATH"] = str(dll_dir) + os.pathsep + os.environ.get("PATH", "")
    if hasattr(os, "add_dll_directory"):
        os.add_dll_directory(str(dll_dir))


def connect_motors() -> Tuple[Thorlabs.KinesisMotor, Thorlabs.KinesisMotor, Thorlabs.KinesisMotor]:
    """Open motors by serial. More robust to different pylablib return types and
    provides diagnostic output and substring matching fallbacks.
    """
    raw = Thorlabs.list_kinesis_devices()

    # Normalize into a dict mapping string-serial -> info
    devices: dict = {}
    try:
        if isinstance(raw, dict):
            for k, v in raw.items():
                devices[str(k)] = v
        elif isinstance(raw, list):
            for item in raw:
                if isinstance(item, tuple) and len(item) >= 1:
                    sn = item[0]
                    info = item[1] if len(item) > 1 else None
                    devices[str(sn)] = info
                else:
                    devices[str(item)] = None
        else:
            # fallback: try to iterate
            for k, v in dict(raw).items():
                devices[str(k)] = v
    except Exception:
        # best-effort fallback
        try:
            for item in raw:
                devices[str(item)] = None
        except Exception:
            pass

    print(f"[DEBUG] Detected Kinesis devices: {list(devices.keys())}")

    def _find_serial(preferred: str) -> Optional[str]:
        # Exact match
        if preferred in devices:
            return preferred
        # Substring match in serial strings
        for sn in devices:
            if preferred and preferred in sn:
                return sn
        # Try looking in info dicts for serial-like fields
        for sn, info in devices.items():
            try:
                if info is None:
                    continue
                s = str(info)
                if preferred and preferred in s:
                    return sn
            except Exception:
                continue
        return None

    sx = _find_serial(SERIAL_X)
    sy = _find_serial(SERIAL_Y)
    sz = _find_serial(SERIAL_Z)

    missing = [name for name, found in (("X", sx), ("Y", sy), ("Z", sz)) if found is None]
    if missing:
        raise RuntimeError(
            f"Missing motors: {missing}. Found devices: {list(devices.keys())}.\n"
            "If the serials in this script are incorrect, set SERIAL_X/SERIAL_Y/SERIAL_Z to one of the listed ids "
            "or export environment variables to override."
        )

    def open_motor(sn: str) -> Thorlabs.KinesisMotor:
        m = Thorlabs.KinesisMotor(sn, scale=MOTOR_SCALE)
        m.open()
        return m

    mx = open_motor(sx)
    my = open_motor(sy)
    mz = open_motor(sz)
    return mx, my, mz


def home_motor(motor: Thorlabs.KinesisMotor, name: str) -> None:
    print(f"[INFO] Homing {name}...")
    motor.home()
    while motor.is_moving():
        time.sleep(MOVE_WAIT)
    time.sleep(0.05)
    print(f"[INFO] {name} homed to {motor.get_position():.3f} mm")


def move_abs(motor: Thorlabs.KinesisMotor, name: str, target: float) -> None:
    """Absolute move with simple poll."""
    print(f"[MOVE] {name} -> {target:.3f} mm")
    motor.move_to(target)
    while motor.is_moving():
        time.sleep(MOVE_WAIT)
    time.sleep(0.05)
    print(f"[MOVE] {name} at {motor.get_position():.3f} mm")


def move_rel(motor: Thorlabs.KinesisMotor, name: str, delta: float) -> None:
    print(f"[MOVE] {name} by {delta:.3f} mm")
    motor.move_by(delta)
    while motor.is_moving():
        time.sleep(MOVE_WAIT)
    time.sleep(0.05)
    print(f"[MOVE] {name} at {motor.get_position():.3f} mm")


def lateral_positions(min_sep: float) -> Iterator[Tuple[float, float]]:
    """Yield (x,z) pairs cycling through grid, enforcing min separation from last."""
    grid = list(itertools.product(X_POSITIONS, Z_POSITIONS))
    last: Optional[Tuple[float, float]] = None
    while True:
        for pos in grid:
            if last is None:
                yield pos
                last = pos
                continue
            dx = pos[0] - last[0]
            dz = pos[1] - last[1]
            if abs(dx) >= min_sep or abs(dz) >= min_sep:
                yield pos
                last = pos


def detect_trap(frame: np.ndarray) -> Tuple[bool, float, Tuple[int, int]]:
    """Very simple bright-spot detector on a single frame."""
    img8 = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(img8)
    if max_val < MIN_BRIGHTNESS:
        return False, float(max_val), (int(max_loc[0]), int(max_loc[1]))
    _, mask = cv2.threshold(img8, max_val - 5, 255, cv2.THRESH_BINARY)
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    for i in range(1, num_labels):
        area = stats[i, cv2.CC_STAT_AREA]
        if area >= MIN_REGION_AREA:
            return True, float(max_val), (int(max_loc[0]), int(max_loc[1]))
    return False, float(max_val), (int(max_loc[0]), int(max_loc[1]))


# -----------------------
# Main routine
# -----------------------

def main() -> None:
    ensure_dll()

    # Connect hardware
    mx = my = mz = None
    sdk = None
    cam = None
    try:
        mx, my, mz = connect_motors()
        home_motor(mx, "X (vertical)")
        home_motor(my, "Y (propagation)")
        home_motor(mz, "Z (horizontal)")

        sdk = TLCameraSDK()
        available = sdk.discover_available_cameras()
        if not available:
            raise RuntimeError("No TSI cameras found")
        cam = sdk.open_camera(available[0])
        cam.exposure_time_us = EXPOSURE_US
        cam.frames_per_trigger_zero_for_unlimited = 0
        cam.image_poll_timeout_ms = IMAGE_TIMEOUT_MS
        cam.arm(4)
        cam.issue_software_trigger()

        print("[INFO] Starting trap attempts. Ctrl+C to stop.")
        pos_iter = lateral_positions(MIN_LATERAL_SEP_MM)
        attempt = 0
        trapped = False
        while not trapped:
            attempt += 1
            x_target, z_target = next(pos_iter)
            print(f"\n[INFO] Attempt {attempt}: moving to X={x_target:.3f} mm, Z={z_target:.3f} mm")
            move_abs(mx, "X", x_target)
            move_abs(mz, "Z", z_target)

            # Sweep along Y
            start_y = SWEEP_CENTER_MM - SWEEP_MM / 2
            end_y = SWEEP_CENTER_MM + SWEEP_MM / 2
            move_abs(my, "Y", start_y)
            move_abs(my, "Y", end_y)
            move_abs(my, "Y", SWEEP_CENTER_MM)

            # Check frames for trap
            cam.issue_software_trigger()
            trapped = False
            best_max = 0.0
            for i in range(FRAMES_TO_CHECK):
                frame = cam.get_pending_frame_or_null()
                if frame is None:
                    continue
                ok, max_val, max_loc = detect_trap(frame.image_buffer)
                best_max = max(best_max, max_val)
                if ok:
                    print(f"[TRAP] Bright spot detected at {max_loc} (max={max_val:.1f}) on attempt {attempt}")
                    trapped = True
                    break
            if not trapped:
                print(f"[INFO] No trap detected (peak={best_max:.1f}). Trying next position.")

    except KeyboardInterrupt:
        print("[INFO] Stopped by user.")
    except Exception as exc:
        print(f"[ERROR] {exc}")
    finally:
        if cam:
            try:
                cam.disarm()
            except Exception:
                pass
            cam.dispose()
        if sdk:
            sdk.dispose()
        for m, name in ((mx, "X"), (my, "Y"), (mz, "Z")):
            if m and m.is_opened():
                try:
                    m.close()
                    print(f"[INFO] Closed {name} motor")
                except Exception:
                    pass


if __name__ == "__main__":
    main()
