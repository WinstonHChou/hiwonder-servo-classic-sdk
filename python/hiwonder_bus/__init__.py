# SPDX-License-Identifier: MIT
#
# hiwonder_bus
#
# The generalized Python API of the HiwonderBus SDK: `SerialBus` + `Servo`,
# degree-based like alexxcode/LX16A-bus, plus vectorized bus operations that
# release the GIL and coalesce multi-servo commands into one write(2).
#
# For the drop-in ethanlipson/PyLX-16A spelling, see the `pylx16a` package.
#
"""Hiwonder/LewanSoul classic serial bus servos (0x55 0x55 protocol).

Quickstart
----------
::

    from hiwonder_bus import SerialBus, Servo

    bus = SerialBus("/dev/ttyUSB0")             # 115200 8N1, low-latency
    servo = Servo(bus, 1)
    servo.initialize()                          # pull the hardware registers
    servo.move(90.0, 500)                       # 90 deg over 500 ms

    ids = bus.scan_ids()                        # who is on the bus?
    angles = bus.read_angles(ids)               # one sweep, NaN where it failed

Synchronized moves
------------------
::

    from hiwonder_bus import SerialBus, Servo, move_sync

    bus = SerialBus("/dev/ttyUSB0")
    targets = [Servo(bus, i) for i in range(1, 11)]
    move_sync(bus, targets, [90.0] * 10, time_ms=800)   # one write(2)

Real-time loops (a dead servo must not stop the run)
----------------------------------------------------
::

    from hiwonder_bus import ErrorMode

    bus.error_mode = ErrorMode.NON_THROWING
    while True:
        angles = bus.read_angles(ids)   # NaN where a servo dropped out
        ...
"""

from __future__ import annotations

from typing import Optional

__version__ = "0.1.0"

try:  # pragma: no cover - exercised implicitly by every install
    from ._hiwonder_bus import (
        Bus,
        BusConfig,
        BusStats,
        Command,
        ErrorCode,
        ErrorMode,
        LX16A,
        LX16ABus,
        LedTriggers,
        SerialBus,
        Servo,
        ServoConfig,
        Telemetry,
        angle_to_ticks,
        build_packet,
        checksum,
        make_simulated_bus,
        move_sync,
        parse_response,
        read_angles,
        read_telemetry,
        set_torque,
        simulate_disconnect,
        sleep_ms,
        ticks_to_angle,
        version,
    )

    _EXTENSION_OK = True
    _IMPORT_ERROR: BaseException = ImportError("<not raised>")
except ImportError as exc:  # pragma: no cover
    _EXTENSION_OK = False
    _IMPORT_ERROR = exc

from .exceptions import (
    ServoArgumentError,
    ServoChecksumError,
    ServoError,
    ServoFirmwareError,
    ServoIOError,
    ServoLogicalError,
    ServoTimeoutError,
)

if not _EXTENSION_OK:  # pragma: no cover
    raise ImportError(
        "hiwonder_bus's native extension `_hiwonder_bus` could not be imported.\n"
        "Build it in place with `pip install -e .` (needs a C++17 compiler, "
        f"CMake >=3.16 and pybind11). Underlying error: {_IMPORT_ERROR!r}"
    ) from _IMPORT_ERROR

__all__ = [
    "Bus",
    "BusConfig",
    "BusStats",
    "Command",
    "ErrorCode",
    "ErrorMode",
    "LX16A",
    "LX16ABus",
    "LedTriggers",
    "SerialBus",
    "Servo",
    "ServoConfig",
    "Telemetry",
    "angle_to_ticks",
    "build_packet",
    "checksum",
    "connect",
    "make_simulated_bus",
    "move_sync",
    "parse_response",
    "read_angles",
    "read_telemetry",
    "set_torque",
    "simulate_disconnect",
    "sleep_ms",
    "ticks_to_angle",
    "version",
    # Exceptions (PyLX-16A compatible)
    "ServoError",
    "ServoTimeoutError",
    "ServoChecksumError",
    "ServoArgumentError",
    "ServoLogicalError",
    "ServoIOError",
    "ServoFirmwareError",
]


def connect(
    port: str,
    *,
    baudrate: int = 115200,
    read_timeout_ms: int = 20,
    write_timeout_ms: int = 100,
    low_latency: bool = True,
    error_mode: Optional[ErrorMode] = None,
) -> SerialBus:
    """Open ``port`` and return a ready-to-use :class:`SerialBus`.

    Parameters
    ----------
    port:
        Character device, e.g. ``/dev/ttyUSB0``, ``/dev/ttyACM0`` or
        ``/dev/ttyTHS1``.
    baudrate:
        Line speed; the classic bus is 115200.
    read_timeout_ms:
        ``poll(2)`` budget for one servo reply.
    write_timeout_ms:
        ``poll(2)`` budget for draining our own transmit bytes.
    low_latency:
        Ask the kernel for ``ASYNC_LOW_LATENCY`` so USB-TTL bridges do not
        impose their 16 ms receive timer.
    error_mode:
        ``ErrorMode.STRICT`` (default, PyLX-16A behaviour) or
        ``ErrorMode.NON_THROWING`` for real-time loops.

    Raises
    ------
    ServoIOError
        When the device cannot be opened in strict mode.
    """
    bus = SerialBus(
        device="",
        baudrate=baudrate,
        read_timeout_ms=read_timeout_ms,
        write_timeout_ms=write_timeout_ms,
        low_latency=low_latency,
        error_mode=ErrorMode.STRICT if error_mode is None else error_mode,
    )
    if not bus.open(port):  # pragma: no cover - requires hardware
        raise ServoIOError(f"could not open {port}: {bus.last_error()}", None)
    return bus


def __getattr__(name: str):
    """Last-resort attribute lookup. Kept for forward compatibility.

    `LX16A` is already bound above as the *native* alias of `Servo` (matching
    `HiwonderBus::LX16A == HiwonderBus::Servo`). The drop-in class with the
    static-initializer surface lives in the `pylx16a` package -- import that one
    when you want PyLX-16A semantics.
    """
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
